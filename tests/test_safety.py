"""止まるべき時に止まるか（snoo_trucar#21 のレビュー）"""

from snoo_trucar.config import CarConfig, ControlConfig
from snoo_trucar.controller import PLAN_MOVE, STOP_REASONS, Controller, scan_schedule
from snoo_trucar.estimator import TargetEstimator

S = 1_000_000_000
DT = 0.01


def _explore():
    ctl = ControlConfig(mode="explore")
    return Controller(ctl, CarConfig()), ctl, TargetEstimator(53.5, 41.4, 33.0)


class Clock:
    def __init__(self, c, est):
        self.c, self.est, self.t = c, est, S

    def run(self, seconds, us=200.0):
        x = None
        for _ in range(int(round(seconds / DT))):
            self.t += int(DT * S)
            x = self.c.step(self.est, self.t, DT, us)
        return x


def test_set_mode_forgets_the_previous_explore():
    """explore をやめて始め直したら、前の 1 歩・待っている station・撮る予定を引き継がない"""
    c, ctl, est = _explore()
    k = Clock(c, est)
    scan_s = scan_schedule(30.0, 0.0)[-1]
    k.run(scan_s + 0.05)
    assert c.station_wait == 0
    c.set_plan((0, PLAN_MOVE, 12.0, 1, 0.0, 40.0, 0, 0, 0, 0))
    k.run(0.3)
    assert c._ex_phase == "step"
    c.set_mode("idle")
    k.run(0.05)
    c.set_mode("explore")
    assert c.plan is None and c.station_wait == -1 and c._ex_phase == "scan"
    assert c._ex_sched_stop == -1 and c._ex_moved == 0.0 and c._low_left == 0.0 and c.low_passed == 0
    # 新しい explore の station 0 を撮り終えたら、返事を待つ（前の返事では走らない）
    x = k.run(scan_s + 0.05)
    assert c.station_wait == 0 and c._ex_phase == "plan" and x.speed == 0
    assert k.run(1.0).speed == 0
    # 前の explore の遅れた返事（待っていないときに来た）は使わない
    c2, _, _ = _explore()
    c2.set_plan((0, PLAN_MOVE, 12.0, 1, 0.0, 40.0, 0, 0, 0, 0))
    assert c2.plan is None



def test_station_drops_the_answer_when_explore_changed(tmp_path):
    """返事を待つ間に explore が始め直されたら、前の explore の分の plan は書かない"""
    from types import SimpleNamespace

    from snoo_trucar.config import Config
    from snoo_trucar.control_proc import PLAN_N
    from snoo_trucar.shm import SeqBuffer
    from snoo_trucar.station import StationClient

    cfg = Config()
    cfg.server.map_url = "http://127.0.0.1:1/nothing"
    d = tmp_path / "explore_b"
    d.mkdir()
    rec = SimpleNamespace(dir=d, entries=[])
    plan = SeqBuffer(PLAN_N)
    runner = SimpleNamespace(cfg=cfg, recorder=rec, battery_v=7.9, last_us=120.0, last_cmd=[0.0] * 23)
    c = StationClient(runner, plan)
    try:
        c._session = "explore_a"  # 送ったときの explore
        c._post(0, runner.last_cmd, rec)
        assert plan.read()[0] == 0  # 書いていない
        c._session = "explore_b"
        c._post(0, runner.last_cmd, rec)
        assert plan.read()[0] == 1
    finally:
        plan.close()


def _server_step(c, k, cm=80.0, speed=12.0):
    """サーバーの 1 歩（まっすぐ cm）を走り始める"""
    c._ex_phase, c.station_wait = "plan", 0
    c.set_plan((0, PLAN_MOVE, speed, 1, 0.0, cm, 0, 0, 0, 0))
    return k.run(DT)


def test_server_step_stops_when_clear_goes_stale():
    """返事が途切れて clear_cm が古くなったら、null と同じく 0.3 s で止まる（欄が無いのと同じにしない）"""
    from snoo_trucar.controller import CLEAR_STALE, CLEAR_VALUE

    c, ctl, est = _explore()
    k = Clock(c, est)
    assert _server_step(c, k).speed > 0
    c.set_clear(CLEAR_VALUE, 100.0, k.t, k.t)
    assert k.run(0.6).speed > 0
    c.age_clear(k.t, int(0.5 * S))  # 最後の返事（撮った時刻）から 0.6 s
    assert c.clear_state == CLEAR_STALE
    assert k.run(0.2).speed > 0
    x = k.run(0.15)
    assert x.speed == 0 and STOP_REASONS[c.last_step[1]] == "clear_unknown"
    # 返事がまだ 1 度も来ていない（制御プロセスの初め）も古いのと同じ。走り出してから 0.3 s で止まる
    c2, _, est2 = _explore()
    k2 = Clock(c2, est2)
    c2.set_clear(CLEAR_STALE, -1.0, k2.t)
    k2.run(1.0)
    assert _server_step(c2, k2).speed > 0
    assert k2.run(0.25).speed > 0
    assert k2.run(0.1).speed == 0 and STOP_REASONS[c2.last_step[1]] == "clear_unknown"


def test_pi_only_drive_ignores_stale_clear():
    """Pi だけの探索（サーバーに届かない）では、古い clear_cm は今までどおり超音波だけで走る"""
    from snoo_trucar.controller import CLEAR_STALE

    c, ctl, est = _explore()
    c._ex_us_at = 0.0
    c._ex_phase = "drive"
    c.set_clear(CLEAR_STALE, -1.0, S)
    assert c.step(est, S + S, DT, 200.0).speed > 0


def test_server_step_limits_a_long_clear_skip():
    """clear_skip（出せない）が続く間は止めないが、速さ 10・20 cm まで。超音波が 60 cm より先なら上限なし"""
    from snoo_trucar.controller import CLEAR_SKIP, CLEAR_SKIP_MAX_CM, CLEAR_SKIP_SPEED

    c, ctl, est = _explore()
    k = Clock(c, est)
    _server_step(c, k, cm=80.0, speed=15.0)
    c.set_clear(CLEAR_SKIP, -1.0, k.t, k.t)
    v = CLEAR_SKIP_SPEED * c.car.cm_s_per_speed
    x = k.run(CLEAR_SKIP_MAX_CM / v - 0.1, us=50.0)
    assert x.speed == CLEAR_SKIP_SPEED
    x = k.run(0.2, us=50.0)
    assert x.speed == 0 and STOP_REASONS[c.last_step[1]] == "clear_skip"
    # 超音波が新しく 60 cm より先 → 上限なし
    c2, _, est2 = _explore()
    k2 = Clock(c2, est2)
    _server_step(c2, k2, cm=80.0, speed=15.0)
    c2.set_clear(CLEAR_SKIP, -1.0, k2.t, k2.t)
    x = k2.run(2.0, us=150.0)
    assert x.speed == 15.0


def test_stale_ultrasonic_never_counts_as_far():
    """超音波が古い（スレッドが止まった・測れていない）ときは、-1 と違って前に進まない"""
    c, ctl, est = _explore()
    k = Clock(c, est)
    c._ex_phase = "drive"
    assert k.run(0.1, us=200.0).speed > 0
    c.us_stale = True  # 制御プロセスが、300 ms より新しい超音波が無いと判断した
    x = k.run(0.05, us=None)
    assert x.speed <= 0
    # サーバーの 1 歩でも止まる
    c2, _, est2 = _explore()
    k2 = Clock(c2, est2)
    c2.set_clear(0, -1.0, k2.t)
    k2.run(0.1, us=200.0)
    assert _server_step(c2, k2).speed > 0
    c2.us_stale = True
    assert k2.run(0.05, us=None).speed == 0 and STOP_REASONS[c2.last_step[1]] == "ultrasonic"


def test_no_echo_is_far_only_just_after_a_real_reading():
    """-1（反響なし）を遠いとみなすのは、1 s 以内に測れていたときだけ"""
    from snoo_trucar.controller import EXPLORE_US_TRUST_S

    assert EXPLORE_US_TRUST_S <= 1.0
    c, ctl, est = _explore()
    k = Clock(c, est)
    c.set_clear(0, -1.0, k.t)
    k.run(0.1, us=200.0)
    assert _server_step(c, k).speed > 0
    assert k.run(0.5, us=None).speed > 0  # 測れたばかり → 遠くまで空いているとみなす
    x = k.run(0.6, us=None)
    assert x.speed == 0 and STOP_REASONS[c.last_step[1]] == "ultrasonic"


def test_control_proc_marks_old_ultrasonic_as_stale():
    from snoo_trucar.control_proc import US_MAX_AGE_NS, us_is_stale

    assert us_is_stale(S, 0)  # まだ 1 度も来ていない
    assert not us_is_stale(S + US_MAX_AGE_NS, S)
    assert us_is_stale(S + US_MAX_AGE_NS + 1, S)


SIGTERM_SCRIPT = r"""
import sys
sys.path.insert(0, sys.argv[1])
log = open(sys.argv[2], "a")
from snoo_trucar import hal

class Rec(hal.FakeCar):
    def stop(self):
        super().stop()
        log.write("stop\n")
        log.flush()

hal.FakeCar = Rec
from snoo_trucar.config import Config
from snoo_trucar.runner import Runner

cfg = Config()
cfg.control.cpu = -1
cfg.control.mode = "keep"
print("ready", flush=True)
Runner(cfg, sim=True, quiet=True, buttons=False).run()
log.write("returned\n")
"""


def test_sigterm_stops_the_motors(tmp_path):
    """systemctl stop（SIGTERM）でも finally まで回り、モーターを止める"""
    import signal
    import subprocess
    import sys
    import time
    from pathlib import Path

    repo = str(Path(__file__).resolve().parents[1])
    log = tmp_path / "log"
    p = subprocess.Popen([sys.executable, "-c", SIGTERM_SCRIPT, repo, str(log)],
                         stdout=subprocess.PIPE, text=True)  # fmt: skip
    try:
        assert p.stdout.readline().strip() == "ready"
        time.sleep(1.5)  # 制御プロセスとスレッドが回り始める
        p.send_signal(signal.SIGTERM)
        code = p.wait(timeout=10)
    finally:
        p.kill()
        p.stdout.close()
    lines = log.read_text().split() if log.exists() else []
    assert code == 0 and "stop" in lines and lines[-1] == "returned"


def test_stop_command_stops_the_motors_without_the_main_process(tmp_path, monkeypatch):
    """ExecStopPost の snoo_trucar stop は、picarx だけで止める。picarx が無くても落ちずに失敗を返す"""
    import sys
    import types
    from pathlib import Path

    from snoo_trucar import cli
    from snoo_trucar import config as config_mod

    cfg = config_mod.Config()
    cfg.car.picarx_config = str(tmp_path / "px" / "picar-x.conf")
    monkeypatch.setattr(config_mod, "load", lambda path=None: cfg)
    calls = []

    class Picarx:
        def __init__(self, config):
            calls.append("init")

        def stop(self):
            calls.append("stop")

    monkeypatch.setitem(sys.modules, "picarx", types.SimpleNamespace(Picarx=Picarx))
    assert cli.main(["stop"]) == 0 and calls == ["init", "stop"]
    monkeypatch.setitem(sys.modules, "picarx", None)  # import できない
    assert cli.main(["stop"]) == 1
    unit = (Path(__file__).resolve().parents[1] / "deploy" / "snoo-trucar.service").read_text()
    assert "ExecStopPost=" in unit and "snoo_trucar stop" in unit


class RecCar:
    def __init__(self):
        self.log = []

    def apply(self, steer, speed, pan, tilt):
        self.log.append(("apply", speed))

    def stop(self):
        self.log.append(("stop",))


class TornCmd:
    """制御プロセスが書いている途中で死んだ（seqlock が奇数のまま）: 初めの n 回だけ読める"""

    def __init__(self, n, speed=20.0):
        self.n, self.speed = n, speed

    def read(self):
        import time

        if self.n <= 0:
            return None
        self.n -= 1
        from snoo_trucar.control_proc import CMD_N

        v = [0.0] * CMD_N
        v[0], v[2], v[14] = time.monotonic_ns(), self.speed, 5
        return 1, v


def test_actuator_stops_when_the_command_cannot_be_read():
    """cmd が読めないまま（書いている途中で止まった）でも、古くなったら止める"""
    from snoo_trucar.config import Config
    from snoo_trucar.runner import Runner

    cfg = Config()
    r = Runner(cfg, seconds=0.4, quiet=True)
    car = RecCar()
    r._actuate_loop(car, TornCmd(3))
    assert ("apply", 20.0) in car.log
    last_apply = max(i for i, e in enumerate(car.log) if e[0] == "apply")
    assert ("stop",) in car.log[last_apply:]


def test_actuator_stops_and_fails_when_the_control_process_died():
    import pytest

    from snoo_trucar.config import Config
    from snoo_trucar.runner import ControlDied, Runner

    class Dead:
        exitcode = -9

        def is_alive(self):
            return False

    r = Runner(Config(), seconds=1.0, quiet=True)
    car = RecCar()
    with pytest.raises(ControlDied):
        r._actuate_loop(car, TornCmd(100), Dead())
    assert car.log[-1] == ("stop",) and not any(e[0] == "apply" for e in car.log)


def test_receive_thread_survives_bad_replies():
    """壊れた JSON・on_clear の失敗で、受信のスレッドが死なない"""
    import json
    import threading

    from snoo_trucar import control_proc
    from snoo_trucar.config import Config
    from snoo_trucar.link import Link
    from snoo_trucar.protocol import make_reply
    from snoo_trucar.shm import SeqBuffer

    cfg = Config()
    cfg.server.host = cfg.server.fallback_host = "127.0.0.1"
    meas, clear = SeqBuffer(control_proc.MEAS_N), SeqBuffer(control_proc.CLEAR_N)
    link = Link(cfg, meas, clear=clear)
    stop = threading.Event()
    import time

    now = time.monotonic_ns()
    good = json.loads(make_reply(1, now, 3.0, []))
    good["clear_cm"] = 80.0
    packets = [b"[1, 2]", b'{"seq": "x"}', json.dumps(good).encode(), json.dumps(good).encode()]

    class Sock:
        def recvfrom(self, n):
            if not packets:
                stop.set()
                raise TimeoutError
            return packets.pop(0), ("127.0.0.1", 1)

        def close(self):
            pass

    real = link.sock
    link.sock = Sock()
    calls = []

    def on_clear(reply):
        calls.append(1)
        raise OSError("ディスクがいっぱい")

    link.on_clear = on_clear
    try:
        link.receive_forever(stop)
        assert link.stats.replies == 2 and len(calls) == 2
        assert clear.read()[1][2] == 1  # 値がある
    finally:
        real.close()
        meas.close()
        clear.close()


def test_camera_thread_survives_send_errors():
    import threading
    from types import SimpleNamespace

    from snoo_trucar.link import camera_loop

    stop = threading.Event()
    sent, n = [], [0]

    class Cam:
        def grab(self):
            n[0] += 1
            if n[0] > 5:
                stop.set()
            if n[0] == 2:
                raise RuntimeError("カメラが一瞬止まった")
            return n[0], b"j"

    def send(t, j):
        if t == 1:
            raise OSError(90, "Message too long")
        sent.append(t)

    link = SimpleNamespace(trace=None, can_send=lambda now: True, send=send, min_interval_ns=lambda: 0)
    camera_loop(Cam(), link, stop)
    assert sent == [3, 4, 5, 6]


def test_ultrasonic_thread_survives_read_errors():
    import threading

    from snoo_trucar import control_proc
    from snoo_trucar.config import Config
    from snoo_trucar.runner import Runner
    from snoo_trucar.shm import SeqBuffer

    r = Runner(Config(), quiet=True)
    us = SeqBuffer(control_proc.US_N)
    n = [0]

    class Car:
        def read_ultrasonic(self):
            n[0] += 1
            if n[0] == 1:
                raise OSError("I2C")
            if n[0] >= 3:
                r.stop.set()
            return 50.0

    try:
        t = threading.Thread(target=r._ultrasonic_loop, args=(Car(), us))
        t.start()
        t.join(5)
        assert n[0] >= 3 and us.read()[1][1] == 50.0
    finally:
        us.close()


def test_runner_stops_the_car_when_a_thread_dies():
    import threading

    import pytest

    from snoo_trucar.config import Config
    from snoo_trucar.runner import Runner, ThreadDied

    r = Runner(Config(), seconds=2.0, quiet=True)

    def broken(stop):
        raise ValueError("想定していない失敗")

    t = threading.Thread(target=r._watched, args=("壊れたスレッド", broken, r.stop), daemon=True)
    t.start()
    t.join(1)
    car = RecCar()
    with pytest.raises(ThreadDied):
        r._actuate_loop(car, TornCmd(1000))
    assert car.log[-1] == ("stop",)


def test_pi_only_reverse_is_at_most_30cm():
    """Pi だけの切り返しで下がるのも、サーバーの 1 歩と同じく 30 cm まで（後ろは見えない）"""
    c, ctl, est = _explore()
    ctl.explore_server = False
    k = Clock(c, est)
    c._ex_phase, c._ex_t = "back", 0.0
    back = 0
    while True:  # 初めの 1 回の下がり
        k.t += int(DT * S)
        if c.step(est, k.t, DT, 40.0).speed >= 0:
            break
        back += 1
    v = ctl.explore_speed * c.car.cm_s_per_speed
    assert back > 0 and back * DT * v <= ctl.explore_reverse_max_cm + 0.5


def _frames():
    import numpy as np

    rng = np.random.default_rng(0)
    scene = rng.uniform(0, 255, (60, 200)).astype(np.float32)

    def frame(x):
        a = scene[:, x : x + 80] + rng.normal(0, 3, (60, 80))
        return a - a.mean()

    return frame


def test_motion_watch_notices_a_stuck_reverse_quickly():
    """下がる（30 cm で 1.2 s ほど）間にも詰まりに気づく。前に進むときは今までどおり"""
    from snoo_trucar.motion import MotionWatch

    frame = _frames()
    w = MotionWatch()
    t = 0.0
    for _ in range(10):
        w.feed(t, frame(0), False)
        t += 0.2
    t0, hit = t, None
    while t - t0 < 2.0:
        if w.feed(t, frame(0), True, reverse=True) and hit is None:
            hit = t - t0
        t += 0.2
    assert hit is not None and hit <= 1.0 + 1e-6
    # 前に進むときは、動き出しを長めに見る（今までどおり 1 s 以内では詰まりとしない）
    w2 = MotionWatch()
    t = 0.0
    for _ in range(10):
        w2.feed(t, frame(0), False)
        t += 0.2
    t0 = t
    while t - t0 < 1.1:
        assert not w2.feed(t, frame(0), True)
        t += 0.2


def test_motion_watch_restarts_when_the_direction_changes():
    """進む → すぐ下がるで元の場所に戻ると、1 秒前の画像と似ている。向きが変わったら測り直す"""
    from snoo_trucar.motion import MotionWatch

    frame = _frames()
    w = MotionWatch()
    t = 0.0
    for _ in range(10):
        w.feed(t, frame(0), False)
        t += 0.2
    for k in range(15):
        w.feed(t, frame(k * 4), True)
        t += 0.2
    assert abs(w._moving_since - 10 * 0.2) < 1e-6
    w.feed(t, frame(56), True, reverse=True)
    assert w._moving_since == t  # 動き出しから数え直す


def _low_pass(c, k):
    """サーバーの 1 歩を走り始め、画像 20 cm・超音波 70 cm（低い物）で越え始める"""
    from snoo_trucar.controller import CLEAR_VALUE, LOW_OBSTACLE_SPEED

    _server_step(c, k, cm=80.0, speed=12.0)
    c.set_clear(CLEAR_VALUE, 20.0, k.t, k.t)
    x = k.run(DT, us=70.0)
    assert x.speed == LOW_OBSTACLE_SPEED and c.low_passed == 1


def test_low_obstacle_pass_only_once_per_step():
    from snoo_trucar.controller import CLEAR_VALUE, LOW_OBSTACLE_CM, LOW_OBSTACLE_SPEED

    c, ctl, est = _explore()
    k = Clock(c, est)
    _low_pass(c, k)
    c.set_clear(CLEAR_VALUE, 100.0, k.t, k.t)  # 越えた
    k.run(LOW_OBSTACLE_CM / (LOW_OBSTACLE_SPEED * c.car.cm_s_per_speed) + 0.1, us=70.0)
    assert c._low_left <= 0 and c._ex_phase == "step"
    c.set_clear(CLEAR_VALUE, 20.0, k.t, k.t)  # また近い。超音波は先 → でも 2 回目は越えない
    x = k.run(DT, us=70.0)
    assert x.speed == 0 and STOP_REASONS[c.last_step[1]] == "clear" and c.low_passed == 1


def test_low_obstacle_pass_stops_when_clear_shrinks():
    """越えている間に、進んだ分より画像の空きが縮んだら（前に別の物が出てきた）止まる"""
    from snoo_trucar.controller import CLEAR_VALUE, LOW_OBSTACLE_SPEED

    c, ctl, est = _explore()
    k = Clock(c, est)
    _low_pass(c, k)
    v = LOW_OBSTACLE_SPEED * c.car.cm_s_per_speed
    # 進んだ分だけ近づく（同じ低い物）→ 越え続ける
    for _ in range(20):
        moved = (k.t - S) / S * v
        c.set_clear(CLEAR_VALUE, max(0.0, 20.0 - moved), k.t, k.t)
        assert k.run(DT, us=70.0).speed == LOW_OBSTACLE_SPEED
    # 急に 8 cm 先 → 止まる
    c.set_clear(CLEAR_VALUE, 8.0, k.t, k.t)
    x = k.run(DT, us=70.0)
    assert x.speed == 0 and STOP_REASONS[c.last_step[1]] == "clear"


def test_server_step_counts_only_the_distance_actually_driven():
    """電池が少ない・guard が待たせている間は進んだことにしない（偽の done・偽の prev_step を送らない）"""
    from snoo_trucar.guard import HOLD

    c, ctl, est = _explore()
    k = Clock(c, est)
    c.set_clear(0, -1.0, k.t)
    v = 12.0 * c.car.cm_s_per_speed
    _server_step(c, k, cm=20.0, speed=12.0)
    c.drive_ok = False  # メインプロセスが、電池が少ないので速さ 0 で書いている
    k.run(20.0 / v + 0.5)
    assert c._ex_phase == "step" and c.step_driven[0] < 0.5
    c.drive_ok = True
    k.run(20.0 / v + 0.1)
    assert STOP_REASONS[c.last_step[1]] == "done" and abs(c.step_driven[0] - 20.0) < 0.5
    # guard が待たせている（HOLD）間も進まない
    c2, _, est2 = _explore()
    k2 = Clock(c2, est2)
    c2.set_clear(0, -1.0, k2.t)
    _server_step(c2, k2, cm=20.0, speed=12.0)
    c2.guard.state = HOLD
    c2.guard._calm_since = None
    c2.guard._stress = 0.9  # 落ち着かない
    k2.run(20.0 / v + 0.5)
    assert c2._ex_phase == "step" and sum(c2.step_driven) < 0.5


def test_server_step_reports_the_segments_actually_driven():
    """途中で止まったら、走った区切りまで（最後は止まった所まで）を返す"""
    c, ctl, est = _explore()
    k = Clock(c, est)
    c.set_clear(0, -1.0, k.t)
    v = 15.0 * c.car.cm_s_per_speed
    c._ex_phase, c.station_wait = "plan", 3
    c.set_plan((3, PLAN_MOVE, 40.0, 2, 20.0, 15.0, 0.0, 40.0, 0, 0))  # 速さ 40 は 15 に抑える
    k.run(DT)
    assert c.step_station == 3 and c._ex_step[0] == ctl.explore_speed
    k.run((15.0 + 10.0) / v)
    k.run(DT, us=ctl.explore_step_stop_cm - 5)  # 2 つ目の区切りの途中で止まる
    assert STOP_REASONS[c.last_step[1]] == "ultrasonic"
    assert abs(c.step_driven[0] - 15.0) < 0.5 and abs(c.step_driven[1] - 10.0) < 1.0 and c.step_driven[2] == 0


def test_station_prev_step_from_what_was_driven(monkeypatch, tmp_path):
    """prev_step は制御プロセスが数えた距離・使った速さ・走った区切り。25 s 待って Pi だけで決めた後に
    遅れて来た返事は、次の prev_step に使わない"""
    import json
    from types import SimpleNamespace

    import requests

    from snoo_trucar.config import Config
    from snoo_trucar.control_proc import CMD_N, PLAN_N
    from snoo_trucar.shm import SeqBuffer
    from snoo_trucar.station import StationClient

    sent = []

    def post(url, data, files, timeout):
        sent.append(json.loads(data["meta"]))
        body = {"action": "move", "speed": 40, "segments": [{"steer_deg": 20, "distance_cm": 15},
                                                             {"steer_deg": 0, "distance_cm": 40}]}  # fmt: skip
        return SimpleNamespace(status_code=200, text="", json=lambda: body)

    monkeypatch.setattr(requests, "post", post)
    cfg = Config()
    d = tmp_path / "explore_a"
    d.mkdir()
    rec = SimpleNamespace(dir=d, entries=[])
    plan = SeqBuffer(PLAN_N)
    v = [0.0] * CMD_N
    runner = SimpleNamespace(cfg=cfg, recorder=rec, battery_v=7.9, last_us=120.0, last_cmd=v)
    c = StationClient(runner, plan)
    c._session = "explore_a"
    try:
        c._post(0, v, rec)
        # station 0 の 1 歩: 速さ 15 で、1 つ目 15 cm、2 つ目 10 cm で止まった
        v[19], v[20], v[22] = 1.2, STOP_REASONS.index("ultrasonic"), 0
        v[23], v[24], v[25], v[26], v[27], v[28] = 0, 15.0, 25.0, 15.0, 10.0, 0.0
        c._post(1, v, rec)
        prev = sent[-1]["prev_step"]
        assert prev["est_distance_cm"] == 25.0 and prev["speed"] == 15.0 and not prev["reverse"]
        assert prev["segments"] == [{"steer_deg": 20.0, "distance_cm": 15.0},
                                    {"steer_deg": 0.0, "distance_cm": 10.0}]  # fmt: skip
        assert prev["steer_deg"] == 0.0 and prev["stop_reason"] == "ultrasonic"
        # station 1 の返事は遅れ、制御プロセスは Pi だけで決めた（step_station = -1）
        v[23] = -1
        c._post(2, v, rec)
        assert sent[-1]["prev_step"] == {"stop_reason": "local"}
    finally:
        plan.close()


def _explore_with_snoo(session_s=None):
    from snoo_trucar.guard import GuardConfig

    ctl = ControlConfig(mode="explore")
    if session_s is not None:
        ctl.guard = GuardConfig(session_s=session_s)
    c = Controller(ctl, CarConfig())
    c._ex_us_at = 0.0
    c._ex_phase = "drive"
    c.set_clear(0, -1.0, S)
    return c, ctl, TargetEstimator(53.5, 41.4, 33.0)


def _see(est, t, dist, rate=0.0):
    est.valid = True
    est.last_seen_ns = t
    est.bearing.x[:] = (0.0, 0.0)
    est.dist.x[:] = (dist, rate)


def test_explore_applies_guard_flee_rule_when_snoo_is_visible():
    """explore で遠く（150 cm より先）に snoo が見えている間も、近づいた直後に速く離れたら下がる"""
    from snoo_trucar.guard import BACKOFF, OK

    c, ctl, est = _explore_with_snoo()
    t = S
    for _ in range(20):
        t += int(DT * S)
        _see(est, t, 300.0)
        x = c.step(est, t, DT, 200.0)
    assert x.speed > 0 and c.guard.state == OK
    for _ in range(int(ctl.guard.flee_hold_s / DT) + 2):
        t += int(DT * S)
        _see(est, t, 300.0, rate=80.0)  # snoo が速く離れる
        x = c.step(est, t, DT, 200.0)
    assert c.guard.state == BACKOFF and x.speed < 0


def test_explore_play_time_limit_counts_only_while_snoo_is_around():
    """遊びの時間（session_s）は snoo が見えている間だけ数える。いないなら探索を続ける"""
    from snoo_trucar.guard import REST

    c, ctl, est = _explore_with_snoo(session_s=2.0)
    t = S
    for _ in range(int(3.0 / DT)):  # snoo がいない
        t += int(DT * S)
        c.step(est, t, DT, 200.0)
    assert c.guard.state != REST
    c._ex_phase, c._ex_moved = "drive", 0.0
    for _ in range(int(2.5 / DT)):
        t += int(DT * S)
        _see(est, t, 300.0)
        x = c.step(est, t, DT, 200.0)
    assert c.guard.state == REST and x.speed == 0


def test_explore_still_stops_and_watches_a_near_snoo():
    from snoo_trucar.controller import TRACK

    c, ctl, est = _explore_with_snoo()
    _see(est, S, 100.0)
    x = c.step(est, S, DT, 200.0)
    assert x.state == TRACK and x.speed == 0 and c.mode == "explore"
