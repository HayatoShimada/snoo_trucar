"""止まって撮ったコマが足りない station は送らずに撮り直す・4xx は送り直さない（snoo_trucar#32）"""

import json
from types import SimpleNamespace

import pytest
import requests

from snoo_trucar.config import CarConfig, Config, ControlConfig
from snoo_trucar.control_proc import CMD_N, PLAN_N
from snoo_trucar.controller import PLAN_DONE, PLAN_RESHOOT, STOP_REASONS, Controller, scan_schedule
from snoo_trucar.estimator import TargetEstimator
from snoo_trucar.shm import SeqBuffer
from snoo_trucar.station import MAX_RESHOOTS, StationClient

S = 1_000_000_000
DT = 0.01


def _run(c, est, t, seconds):
    out = []
    for _ in range(int(round(seconds / DT))):
        t[0] += int(DT * S)
        out.append(c.step(est, t[0], DT).scan_index)
    return out


def test_controller_reshoots_the_same_station():
    """PLAN_RESHOOT を受けたら、同じ station の番号・同じ向きの番号で撮り直し、また返事を待つ"""
    c = Controller(ControlConfig(mode="explore"), CarConfig())
    est = TargetEstimator(53.5, 41.4, 33.0)
    t = [S]
    n = len(c.scan_poses)
    first = _run(c, est, t, scan_schedule(0.0, 0.0, c.scan_poses)[-1] + 0.05)
    assert c.station_wait == 0 and c._ex_phase == "plan"
    c.last_step = (1.5, STOP_REASONS.index("clear"))  # 直前の 1 歩（prev_step の元）は残す
    c.set_plan((0, PLAN_RESHOOT) + (0.0,) * (PLAN_N - 2))
    again = _run(c, est, t, 0.05)
    assert c._ex_phase == "scan" and c.station_wait == -1 and c._ex_stop == 0
    again += _run(c, est, t, scan_schedule(0.0, 0.0, c.scan_poses)[-1] + 0.3)
    assert {i for i in again if i >= 0} == {i for i in first if i >= 0} == set(range(n))
    assert c.station_wait == 0 and c._ex_phase == "plan"
    assert c.last_step == (1.5, STOP_REASONS.index("clear"))
    # 次の station は 1（撮り直しで番号は進まない）
    c.set_plan((0, PLAN_DONE) + (0.0,) * (PLAN_N - 2))
    _run(c, est, t, DT)
    assert c.mode == "idle"


class _Server:
    """requests.post の代わり。status を返し、送られた meta を覚える"""

    def __init__(self, status=200):
        self.status, self.sent = status, []

    def __call__(self, url, data, files, timeout):
        self.sent.append(json.loads(data["meta"]))
        body = {"action": "move", "speed": 12, "segments": [{"steer_deg": 0, "distance_cm": 20}]}
        if self.status != 200:
            body = {"detail": "止まって撮ったコマ（moving でないもの）が 1 つも無い"}
        return SimpleNamespace(status_code=self.status, text=json.dumps(body), json=lambda: body)


@pytest.fixture
def client(tmp_path, monkeypatch):
    srv = _Server()
    monkeypatch.setattr(requests, "post", srv)
    d = tmp_path / "explore_a"
    d.mkdir()
    rec = SimpleNamespace(dir=d, entries=[], missed=[3], last_offer_ns=0, write_errors=0)
    buf = SeqBuffer(PLAN_N)
    v = [0.0] * CMD_N
    v[23] = -1  # 走っていない
    runner = SimpleNamespace(cfg=Config(), recorder=rec, battery_v=7.9, last_us=None, last_cmd=v)
    c = StationClient(runner, buf)
    yield c, srv, rec, buf, v
    buf.close()


def _still(rec, k, n):
    for i in range(n):
        name = f"{k}_{i:02d}.jpg"
        (rec.dir / name).write_bytes(b"jpg")
        rec.entries.append({"file": name, "station": k, "moving": False,
                            "pan_deg": 0, "tilt_deg": 0})  # fmt: skip


def test_zero_frames_are_reshot_instead_of_sent(client):
    c, srv, rec, buf, v = client
    v[18] = 0
    c._once()
    assert srv.sent == []  # 送らない
    assert tuple(buf.read()[1][:2]) == (0, PLAN_RESHOOT)
    assert "0/10 枚" in c.last and "撮る前に次の向きへ替わった 1 向き" in c.last
    c._once()
    assert srv.sent == [] and c._reshoots[0] == 1  # 撮り直し始めるまで、もう一度は頼まない
    # 制御プロセスが撮り直し（待つのをやめ）、撮り終えてまた 0 を待つ
    v[18] = -1
    c._once()
    _still(rec, 0, 10)
    v[18] = 0
    c._once()
    assert len(srv.sent) == 1 and len(srv.sent[0]["frames"]) == 10
    assert buf.read()[1][1] == 1  # PLAN_MOVE


def test_zero_frames_give_up_after_the_reshoot_limit(client):
    """撮り直しても 0 枚のままなら、送らずに探索をやめる（止まって待ち続けない）"""
    c, srv, rec, buf, v = client
    for _ in range(MAX_RESHOOTS):
        v[18] = 0
        c._once()
        assert buf.read()[1][1] == PLAN_RESHOOT
        v[18] = -1
        c._once()
    v[18] = 0
    c._once()
    assert srv.sent == [] and tuple(buf.read()[1][:2]) == (0, PLAN_DONE)
    assert "探索をやめる" in c.last


def test_few_frames_are_sent_after_the_reshoot_limit(client):
    c, srv, rec, buf, v = client
    _still(rec, 0, 2)
    for _ in range(MAX_RESHOOTS):
        v[18] = 0
        c._once()
        v[18] = -1
        c._once()
    v[18] = 0
    c._once()
    assert len(srv.sent) == 1 and len(srv.sent[0]["frames"]) == 2


def test_rejected_station_is_not_resent_but_reshot(client, monkeypatch):
    """4xx（中身がおかしい）は送り直さない。撮り直して、上限を過ぎたら探索をやめる"""
    c, srv, rec, buf, v = client
    srv.status = 400
    _still(rec, 0, 10)
    v[18] = 0
    c._once()
    assert len(srv.sent) == 1 and buf.read()[1][1] == PLAN_RESHOOT
    for _ in range(5):  # 3 秒おきの送り直しはしない
        monkeypatch.setattr("snoo_trucar.station.time.monotonic", lambda: 1e9)
        c._once()
    assert len(srv.sent) == 1
    for _ in range(MAX_RESHOOTS):
        v[18] = -1
        c._once()
        v[18] = 0
        c._once()
    assert len(srv.sent) == 1 + MAX_RESHOOTS and buf.read()[1][1] == PLAN_DONE


def test_unreachable_server_is_still_retried(client, monkeypatch):
    """届かない（5xx・つながらない）は今まで通り RETRY_S おきに送り直す"""
    c, srv, rec, buf, v = client
    srv.status = 503
    _still(rec, 0, 10)
    v[18] = 0
    c._once()
    assert len(srv.sent) == 1 and buf.read()[0] == 0  # 何も書かない
    monkeypatch.setattr("snoo_trucar.station.time.monotonic", lambda: 1e12)
    c._once()
    assert len(srv.sent) == 2


def test_map_recorder_notes_poses_it_could_not_save(tmp_path, monkeypatch):
    """向きが替わる前に撮れなかった向き・書けなかった画像を、撮れなかった訳として覚える"""
    from snoo_trucar.controller import MODES
    from snoo_trucar.mapping import MapRecorder

    rec = MapRecorder(tmp_path)
    clock = [0.0]
    monkeypatch.setattr("snoo_trucar.mapping.time.monotonic", lambda: clock[0])
    cmd = [0.0] * CMD_N
    cmd[14] = MODES.index("explore")
    cmd[13] = 0
    rec.offer(5, b"j", cmd, None)
    cmd[13] = 1  # 落ち着く前に次の向きへ
    rec.offer(6, b"j", cmd, None)
    assert rec.missed == [0] and rec.last_offer_ns == 6
    clock[0] = 2.0

    def broken(self, data):
        raise OSError("書けない")

    monkeypatch.setattr("pathlib.Path.write_bytes", broken)
    rec.offer(7, b"j", cmd, None)
    assert rec.write_errors == 1 and rec.entries == [] and 1 not in rec._saved
