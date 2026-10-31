"""起動し直しても explore を同じセッションで続ける（snoo_trucar#30）"""

import json
from types import SimpleNamespace

from snoo_trucar.config import CarConfig, Config, ControlConfig
from snoo_trucar.control_proc import CMD_N, MODE_N, PLAN_N
from snoo_trucar.controller import DRIVE_INDEX, MODES, Controller, scan_schedule
from snoo_trucar.estimator import TargetEstimator
from snoo_trucar.resume import MAX_AGE_S, ExploreState, phase_of
from snoo_trucar.shm import SeqBuffer

S = 1_000_000_000
DT = 0.01


def test_write_and_load(tmp_path):
    p = tmp_path / "state" / "explore.json"
    st = ExploreState(p)
    st.write("explore_20261031_100000", 4, "scan", now=1000.0)
    assert not (p.parent / "explore.json.tmp").exists()  # 書いてから入れ替える
    d = json.loads(p.read_text())
    assert d == {"mode": "explore", "session_id": "explore_20261031_100000", "next_station": 4,
                 "started": 1000.0, "updated": 1000.0, "phase": "scan"}  # fmt: skip
    st.write("explore_20261031_100000", 5, "step", now=1100.0)
    st2 = ExploreState(p)
    assert st2.load(now=1100.0 + MAX_AGE_S - 1)["next_station"] == 5
    assert st2.started == 1000.0  # 始めた時刻は引き継ぐ


def test_old_or_broken_state_does_not_resume(tmp_path):
    p = tmp_path / "explore.json"
    st = ExploreState(p)
    st.write("explore_a", 2, "scan", now=1000.0)
    assert st.load(now=1000.0 + MAX_AGE_S + 1) is None  # 10 分より古い
    p.write_text("{")
    assert st.load(now=1000.0) is None
    bad = {"mode": "explore", "session_id": "../x", "next_station": 1, "updated": 1000.0}
    p.write_text(json.dumps(bad))
    assert st.load(now=1000.0) is None
    assert ExploreState(None).load() is None
    assert ExploreState(tmp_path / "none.json").load() is None


def _runner(tmp_path, resume=False):
    from snoo_trucar.runner import Runner

    cfg = Config()
    cfg.explore_state = str(tmp_path / "explore.json")
    r = Runner(cfg, resume=resume)
    r.mode_buf = SeqBuffer(MODE_N)
    return r


def test_long_press_to_idle_deletes_the_state(tmp_path):
    r = _runner(tmp_path)
    try:
        r.explore_state.write("explore_a", 3, "scan")
        r.set_mode("explore")  # explore に入るだけでは消さない
        assert (tmp_path / "explore.json").exists()
        r.set_mode("idle")  # USR の長押し（buttons.event_to_mode("long")）
        assert not (tmp_path / "explore.json").exists()
        assert tuple(r.mode_buf.read()[1]) == (MODES.index("idle"), -1.0, 0.0)
    finally:
        r.mode_buf.close()


def test_runner_resumes_only_a_fresh_state(tmp_path):
    ExploreState(tmp_path / "explore.json").write("explore_a", 7, "step")
    r = _runner(tmp_path, resume=True)
    r.mode_buf.close()
    assert r.resume["session_id"] == "explore_a" and r.resume["next_station"] == 7
    assert r.cfg.control.mode == "explore"
    # --mode を決めて起動したら戻らない
    r2 = _runner(tmp_path, resume=False)
    r2.mode_buf.close()
    assert r2.resume is None and r2.cfg.control.mode == "idle"
    # explore_max_s を過ぎていたら戻らない
    ExploreState(tmp_path / "explore.json").write("explore_a", 7, "step")
    st = json.loads((tmp_path / "explore.json").read_text())
    st["started"] -= 10_000
    (tmp_path / "explore.json").write_text(json.dumps(st))
    r3 = _runner(tmp_path, resume=True)
    r3.mode_buf.close()
    assert r3.resume is None and not (tmp_path / "explore.json").exists()


def test_controller_resumes_by_shooting_first():
    """戻ったら、まずその場で撮り、続きの station 番号で返事を待つ。走らない。prev_step は無し"""
    c = Controller(ControlConfig(mode="explore"), CarConfig())
    est = TargetEstimator(53.5, 41.4, 33.0)
    c.resume_explore(7, elapsed_s=120.0)
    assert c._ex_phase == "scan" and c._ex_stop == 7 and c.step_station == -1
    assert c._ex_total == 120.0
    t, speeds, idx = S, [], set()
    for _ in range(int((scan_schedule(0.0, 0.0, c.scan_poses)[-1] + 1.0) / DT)):
        t += int(DT * S)
        x = c.step(est, t, DT)
        speeds.append(x.speed)
        idx.add(x.scan_index)
    n = len(c.scan_poses)
    assert set(range(7 * n, 8 * n)) <= idx and max(speeds) == 0.0
    assert c.station_wait == 7 and c._ex_phase == "plan"


def test_station_client_writes_the_state_and_deletes_it_when_explore_ends(tmp_path):
    from snoo_trucar.station import StationClient

    d = tmp_path / "explore_a"
    d.mkdir()
    rec = SimpleNamespace(dir=d, entries=[])
    st = ExploreState(tmp_path / "explore.json")
    v = [0.0] * CMD_N
    v[14], v[18], v[21], v[23] = MODES.index("explore"), -1, 3, -1
    buf = SeqBuffer(PLAN_N)
    runner = SimpleNamespace(cfg=Config(), recorder=rec, battery_v=7.9, last_us=None, last_cmd=v)
    c = StationClient(runner, buf, st)
    try:
        v[13] = 30  # 撮っている
        c._note_state(v, rec)
        got = json.loads(st.path.read_text())
        assert (got["session_id"], got["next_station"], got["phase"]) == ("explore_a", 3, "scan")
        v[13], v[18], v[21] = -1, 3, 4  # 撮り終えて 3 の返事を待つ（3 はまだ終わっていない）
        c._note_state(v, rec)
        assert json.loads(st.path.read_text())["next_station"] == 3
        v[13], v[18] = DRIVE_INDEX, -1  # 返事が来て走っている
        c._note_state(v, rec)
        got = json.loads(st.path.read_text())
        assert (got["next_station"], got["phase"]) == (4, "step")
        v[14] = MODES.index("idle")  # 調べ終えた・時間切れ
        c._note_state(v, rec)
        assert not st.path.exists()
    finally:
        buf.close()


def test_phase_of():
    v = [0.0] * CMD_N
    v[13], v[18] = 3, -1
    assert phase_of(v) == "scan"
    v[18] = 2
    assert phase_of(v) == "plan"
    v[13], v[18] = DRIVE_INDEX, -1
    assert phase_of(v) == "step"


def test_map_recorder_continues_the_same_session(tmp_path, monkeypatch):
    """同じ置き場に足し、前の分を scan.json に残す。走っている間の画像の名前を上書きしない。
    同じ向きを撮り直したら、同じ名前を 2 つ送らない"""
    from snoo_trucar.mapping import MapRecorder

    clock = [0.0]
    monkeypatch.setattr("snoo_trucar.mapping.time.monotonic", lambda: clock[0])
    cmd = [0.0] * CMD_N
    cmd[14] = MODES.index("explore")
    rec = MapRecorder(tmp_path)
    cmd[13], cmd[21] = DRIVE_INDEX, 1
    rec.offer(1, b"d", cmd, None)
    clock[0] += 1.0
    cmd[13] = 10  # station 1 の 1 つ目の向き
    rec.offer(2, b"s", cmd, None)
    clock[0] += 1.0
    rec.offer(3, b"s", cmd, None)
    name = rec.dir.name
    assert [e["file"][:6] for e in rec.entries] == ["d00001", "10_pan"]
    rec.finish()  # 起動し直す（SIGTERM で finish まで回る）

    rec2 = MapRecorder(tmp_path)
    rec2.resume(name)
    cmd[13] = 10  # 戻って station 1 を撮り直す
    rec2.offer(4, b"s2", cmd, None)
    clock[0] += 1.0
    rec2.offer(5, b"s2", cmd, None)
    cmd[13], cmd[21] = DRIVE_INDEX, 2
    clock[0] += 1.0
    rec2.offer(6, b"d", cmd, None)
    assert rec2.dir.name == name
    files = [e["file"] for e in rec2.entries]
    assert len(files) == len(set(files)) == 3 and files[-1].startswith("d00002")
    assert [e["t_capture_ns"] for e in rec2.entries if e["file"].startswith("10_")] == [5]
    rec2.finish()
    scan = json.loads((tmp_path / name / "scan.json").read_text())
    assert len(scan["frames"]) == 3


def test_station_after_resume_sends_the_same_session_and_no_prev_step(tmp_path, monkeypatch):
    import requests

    from snoo_trucar.station import StationClient

    sent = []

    def post(url, data, files, timeout):
        sent.append(json.loads(data["meta"]))
        body = {"action": "move", "speed": 12, "segments": [{"steer_deg": 0, "distance_cm": 20}]}
        return SimpleNamespace(status_code=200, text="", json=lambda: body)

    monkeypatch.setattr(requests, "post", post)
    d = tmp_path / "explore_a"
    d.mkdir()
    entries = []
    for i in range(10):
        (d / f"{70 + i}.jpg").write_bytes(b"j")
        entries.append({"file": f"{70 + i}.jpg", "station": 7, "moving": False, "pan_deg": 0,
                        "tilt_deg": 0})  # fmt: skip
    rec = SimpleNamespace(dir=d, entries=entries)
    v = [0.0] * CMD_N
    v[14], v[18], v[21], v[23] = MODES.index("explore"), 7, 8, -1  # 戻って 7 を撮り終えた
    buf = SeqBuffer(PLAN_N)
    runner = SimpleNamespace(cfg=Config(), recorder=rec, battery_v=7.9, last_us=None, last_cmd=v)
    c = StationClient(runner, buf, ExploreState(tmp_path / "explore.json"))
    try:
        c._once()
        assert sent[0]["session_id"] == "explore_a" and sent[0]["station_id"] == 7
        assert sent[0]["prev_step"] is None
    finally:
        buf.close()
