"""#21 のレビューの残り（snoo_trucar#35）：短い 1 歩の詰まり・clear.jsonl・history"""

import json
from types import SimpleNamespace

import numpy as np

from snoo_trucar.config import Config
from snoo_trucar.control_proc import C_STEP_REASON, C_STEP_STATION, CMD_N, PLAN_N
from snoo_trucar.controller import STOP_REASONS
from snoo_trucar.motion import MotionWatch
from snoo_trucar.shm import SeqBuffer


def _drive(m, t, seconds, frame, reverse=False):
    """5 Hz で、走らせている間の画像を渡す。frame(t) が画像"""
    end = t + seconds
    while t < end:
        m.feed(t, frame(t), True, reverse=reverse)
        t += 0.2
    return t


def _still(m, t, seconds, img):
    end = t + seconds
    while t < end:
        m.feed(t, img, False)
        t += 0.2
    return t


def test_short_forward_step_without_image_motion_is_noted():
    rng = np.random.default_rng(0)
    base = rng.normal(0, 20, (60, 80)).astype(np.float32)

    def noisy(_t):
        return base + rng.normal(0, 2, base.shape).astype(np.float32)

    def scene(t):  # 走ると画像が横に流れる
        return np.roll(base, int(t * 30), axis=1)

    m = MotionWatch()
    t = _still(m, 0.0, 2.0, base)
    t = _drive(m, t, 1.0, noisy)  # 20 cm（約 1 s）走ったが、画像が動かない
    t = _still(m, t, 0.4, base)
    assert m.stuck is False  # 制御プロセスの詰まりの検知は間に合わない
    (run,) = m.take_runs()
    assert run["moved"] is False and not run["reverse"]
    t = _drive(m, t, 1.0, scene)
    t = _still(m, t, 0.4, base)
    (run,) = m.take_runs()
    assert run["moved"] is True
    _drive(m, t, 0.4, noisy)  # 短すぎて分からない
    m.feed(t + 0.6, base, False)
    assert m.take_runs()[0]["moved"] is None


def test_station_marks_a_short_still_step_as_stuck(monkeypatch, tmp_path):
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
    runs = [[{"seconds": 1.0, "reverse": False, "diff": 3.0, "noise": 6.0, "moved": False}]]
    motion = SimpleNamespace(take_runs=lambda: runs.pop() if runs else [])
    rec = SimpleNamespace(dir=d, entries=[])
    v = [0.0] * CMD_N
    v[C_STEP_STATION], v[C_STEP_REASON] = 0, STOP_REASONS.index("done")
    buf = SeqBuffer(PLAN_N)
    runner = SimpleNamespace(cfg=Config(), recorder=rec, battery_v=7.9, last_us=None, last_cmd=v,
                             motion=motion)  # fmt: skip
    c = StationClient(runner, buf)
    c._session = "explore_a"
    c._moves[0] = [(0.0, 20.0, False, None)]
    try:
        c._post(1, v, rec)
        prev = sent[-1]["prev_step"]
        assert prev["stop_reason"] == "stuck" and prev["image_motion"][0]["moved"] is False
        c._post(1, v, rec)  # 送り直しでも同じ
        assert sent[-1]["prev_step"]["stop_reason"] == "stuck"
    finally:
        buf.close()


def test_clear_jsonl_stays_open_and_is_flushed_every_second(tmp_path, monkeypatch):
    from snoo_trucar.mapping import MapRecorder

    clock = [100.0]
    monkeypatch.setattr("snoo_trucar.mapping.time.monotonic", lambda: clock[0])
    opened = []
    real_open = open

    def counting_open(path, *a, **kw):
        opened.append(str(path))
        return real_open(path, *a, **kw)

    monkeypatch.setattr("builtins.open", counting_open)
    rec = MapRecorder(tmp_path)
    rec.dir = tmp_path / "explore_a"
    rec.dir.mkdir()
    f = rec.dir / "clear.jsonl"
    for i in range(60):  # 1 秒に 60 回
        rec.note_clear(i, 50.0, 0.9, None, None)
        clock[0] += 1 / 60
    assert sum(p.endswith("clear.jsonl") for p in opened) == 1  # 1 回だけ開く
    assert len(f.read_text().splitlines()) >= 1  # 1 秒ごとに flush した
    rec.finish()
    assert len(f.read_text().splitlines()) == 60 and rec._clear_f is None


def test_history_is_kept_only_in_the_simulation():
    from snoo_trucar.runner import Runner

    r = Runner(Config(), quiet=True)
    r._record(0.0, [0.0] * CMD_N)
    assert r.history == []
