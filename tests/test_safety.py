"""止まるべき時に止まるか（snoo_trucar#21 のレビュー）"""

from snoo_trucar.config import CarConfig, ControlConfig
from snoo_trucar.controller import STOP_REASONS, Controller, scan_schedule
from snoo_trucar.estimator import TargetEstimator
from snoo_trucar.station import plan_row

S = 1_000_000_000
DT = 0.01


def _explore():
    ctl = ControlConfig(mode="explore")
    return Controller(ctl, CarConfig()), ctl, TargetEstimator(53.5, 41.4, 33.0)


def plan(station, segs, speed=12.0, back_speed=8.0):
    """plan の共有メモリの 1 行。segs は [(ハンドル, cm, 後ろへか[, clear_cm の下限])]"""
    segs = [(st, cm, b, (x[0] if x else None)) for st, cm, b, *x in segs]
    return plan_row(station, segs, speed, back_speed)


class Clock:
    def __init__(self, c, est):
        self.c, self.est, self.t = c, est, S

    def run(self, seconds, us=None):
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
    c.set_plan(plan(0, [(0.0, 40.0, False)]))
    k.run(0.3)
    assert c._ex_phase == "step"
    c.set_mode("idle")
    k.run(0.05)
    c.set_mode("explore")
    assert c.plan is None and c.station_wait == -1 and c._ex_phase == "scan"
    assert c._ex_sched_stop == -1 and c._ex_moved == 0.0 and c.low_passed == 0
    # 新しい explore の station 0 を撮り終えたら、返事を待つ（前の返事では走らない）
    x = k.run(scan_s + 0.05)
    assert c.station_wait == 0 and c._ex_phase == "plan" and x.speed == 0
    assert k.run(1.0).speed == 0
    # 前の explore の遅れた返事（待っていないときに来た）は使わない
    c2, _, _ = _explore()
    c2.set_plan(plan(0, [(0.0, 40.0, False)]))
    assert c2.plan is None



def test_station_does_not_plan_on_the_pi_when_the_server_is_unreachable(tmp_path):
    """地図のサーバーに届かなければ、plan を書かずに送り直す（Pi だけで道を決めない）"""
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
    buf = SeqBuffer(PLAN_N)
    runner = SimpleNamespace(cfg=cfg, recorder=rec, battery_v=7.9, last_us=120.0, last_cmd=[0.0] * 31)
    c = StationClient(runner, buf)
    try:
        c._session = "explore_a"  # 送ったときの explore
        assert c._post(0, runner.last_cmd, rec) is False  # 届かない → 送り直す
        assert buf.read()[0] == 0  # 書いていない（Pi で道を決めない）
    finally:
        buf.close()


def _server_step(c, k, cm=80.0, speed=12.0):
    """サーバーの 1 歩（まっすぐ cm）を走り始める"""
    c._ex_phase, c.station_wait = "plan", 0
    c.set_plan(plan(0, [(0.0, cm, False)], speed))
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


def test_server_step_limits_a_long_clear_skip():
    """clear_skip（出せない）が続く間は止めないが、速さ 10・20 cm まで（超音波では延ばさない）"""
    from snoo_trucar.controller import CLEAR_SKIP, CLEAR_SKIP_MAX_CM, CLEAR_SKIP_SPEED

    c, ctl, est = _explore()
    k = Clock(c, est)
    _server_step(c, k, cm=80.0, speed=15.0)
    c.set_clear(CLEAR_SKIP, -1.0, k.t, k.t)
    v = CLEAR_SKIP_SPEED * c.car.cm_s_per_speed
    x = k.run(CLEAR_SKIP_MAX_CM / v - 0.1, us=150.0)
    assert x.speed == CLEAR_SKIP_SPEED
    x = k.run(0.2, us=150.0)
    assert x.speed == 0 and STOP_REASONS[c.last_step[1]] == "clear_skip"


def test_ultrasonic_never_stops_the_car():
    """超音波は当てにならないので、近い値が続いても・測れなくても止まらない（clear_cm と guard で見る）"""
    from snoo_trucar.controller import CLEAR_VALUE

    c, ctl, est = _explore()
    k = Clock(c, est)
    c.set_clear(CLEAR_VALUE, 100.0, k.t, k.t)
    assert _server_step(c, k).speed > 0
    for us in (5.0, 5.0, None, 3.0):
        c.set_clear(CLEAR_VALUE, 100.0, k.t, k.t)
        assert k.run(0.1, us=us).speed > 0
    assert c._ex_phase == "step"


def test_server_step_stops_when_the_camera_cannot_see():
    """前を見ているカメラ（clear_cm）が途切れたら、0.3 s で止まる"""
    from snoo_trucar.controller import CLEAR_STALE, CLEAR_VALUE

    c, ctl, est = _explore()
    k = Clock(c, est)
    c.set_clear(CLEAR_VALUE, 100.0, k.t, k.t)
    assert _server_step(c, k).speed > 0
    c.set_clear(CLEAR_STALE, -1.0, k.t)
    x = k.run(0.4)
    assert x.speed == 0 and STOP_REASONS[c.last_step[1]] == "clear_unknown"


def test_server_step_uses_the_larger_of_the_own_and_the_server_clear_limit():
    """区切りの min_clear_cm（サーバーの見込み）が explore_clear_stop_cm より大きければそこで止まる。
    小さければ Pi の explore_clear_stop_cm（サーバーは止まる近さを縮められない）"""
    from snoo_trucar.controller import CLEAR_VALUE

    c, ctl, est = _explore()
    k = Clock(c, est)
    c._ex_phase, c.station_wait = "plan", 0
    c.set_plan(plan(0, [(0.0, 80.0, False, 40.0)]))
    c.set_clear(CLEAR_VALUE, 45.0, k.t, k.t)
    assert k.run(DT).speed > 0
    c.set_clear(CLEAR_VALUE, 35.0, k.t, k.t)
    x = k.run(DT)
    assert x.speed == 0 and STOP_REASONS[c.last_step[1]] == "clear"
    c2, _, est2 = _explore()
    k2 = Clock(c2, est2)
    c2._ex_phase, c2.station_wait = "plan", 0
    c2.set_plan(plan(0, [(0.0, 80.0, False, 5.0)]))
    c2.set_clear(CLEAR_VALUE, ctl.explore_clear_stop_cm - 1, k2.t, k2.t)
    assert k2.run(DT).speed == 0 and STOP_REASONS[c2.last_step[1]] == "clear"


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
    """読めなくても落ちず、stuck・drive_ok は渡す（超音波では止まらない）"""
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
        assert n[0] >= 3 and us.read()[1][1] == 50.0 and r.last_us_t_ns > 0
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


def test_reverse_segments_are_at_most_30cm_in_total():
    """後ろへの区切りは、合わせて explore_reverse_max_cm まで（後ろは見えない）"""
    c, ctl, est = _explore()
    k = Clock(c, est)
    c._ex_phase, c.station_wait = "plan", 0
    c.set_plan(plan(0, [(28.0, 25.0, True), (-28.0, 10.0, False), (28.0, 25.0, True)]))
    k.run(DT)
    segs = c._ex_step[2]
    assert sum(cm for _, cm, b, _ in segs if b) <= ctl.explore_reverse_max_cm + 1e-9
    assert [b for *_, b, _ in segs] == [True, False, True]


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


def test_low_obstacles_are_not_driven_over_any_more():
    """前の版は、超音波が先を見ていれば画像の近い物を「低い物」として越えた。今は clear_cm で止まる"""
    from snoo_trucar.controller import CLEAR_VALUE

    c, ctl, est = _explore()
    k = Clock(c, est)
    _server_step(c, k)
    c.set_clear(CLEAR_VALUE, 20.0, k.t, k.t)
    x = k.run(DT, us=200.0)
    assert x.speed == 0 and STOP_REASONS[c.last_step[1]] == "clear" and c.low_passed == 0


def test_server_step_counts_only_the_distance_actually_driven():
    """電池が少ない・guard が待たせている間は進んだことにしない（偽の done・偽の prev_step を送らない）"""
    from snoo_trucar.guard import HOLD

    c, ctl, est = _explore()
    k = Clock(c, est)
    c.set_clear(1, 100.0, k.t, k.t)  # 前は空いている（CLEAR_VALUE）
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
    c2.set_clear(1, 100.0, k2.t, k2.t)
    _server_step(c2, k2, cm=20.0, speed=12.0)
    c2.guard.state = HOLD
    c2.guard._calm_since = None
    c2.guard._stress = 0.9  # 落ち着かない
    k2.run(20.0 / v + 0.5)
    assert c2._ex_phase == "step" and sum(c2.step_driven) < 0.5


def test_server_step_reports_the_segments_actually_driven():
    """途中で止まったら、走った区切りまで（最後は止まった所まで）を返す"""
    from snoo_trucar.controller import CLEAR_VALUE

    c, ctl, est = _explore()
    k = Clock(c, est)
    c.set_clear(1, 100.0, k.t, k.t)  # 前は空いている（CLEAR_VALUE）
    v = 15.0 * c.car.cm_s_per_speed
    c._ex_phase, c.station_wait = "plan", 3
    c.set_plan(plan(3, [(20.0, 15.0, False), (0.0, 40.0, False)], speed=40.0))  # 速さ 40 は 15 に
    k.run(DT)
    assert c.step_station == 3 and c._ex_step[0] == ctl.explore_speed
    k.run((15.0 + 10.0) / v)
    c.set_clear(CLEAR_VALUE, 10.0, k.t, k.t)  # 2 つ目の区切りの途中で止まる
    k.run(DT)
    assert STOP_REASONS[c.last_step[1]] == "clear"
    assert abs(c.step_driven[0] - 15.0) < 0.5 and abs(c.step_driven[1] - 10.0) < 1.0
    assert c.step_driven[2] == 0


def test_mixed_gear_step_runs_each_segment_in_its_own_direction():
    """前 10 cm → 後ろ 10 cm → 前 10 cm。向きを替えるときは止まって待ち、区切りごとの距離を数える。
    下がる区切りでは clear_cm を見ない（下がるときは前の画像は当てにならない）"""
    from snoo_trucar.controller import CLEAR_VALUE, GEAR_PAUSE_S

    c, ctl, est = _explore()
    k = Clock(c, est)
    c.set_clear(CLEAR_VALUE, 100.0, k.t, k.t)
    c._ex_phase, c.station_wait = "plan", 0
    c.set_plan(plan(0, [(28.0, 10.0, False), (-28.0, 10.0, True), (28.0, 10.0, False)],
                    speed=12.0, back_speed=8.0))  # fmt: skip
    speeds = []
    for _ in range(int(6.0 / DT)):
        if c._ex_step[2] and c._ex_seg == 2:
            c.set_clear(CLEAR_VALUE, 100.0, k.t, k.t)
        if c._ex_seg == 1:
            c.set_clear(CLEAR_VALUE, 5.0, k.t, k.t)  # 下がる間は前が近く見えても止まらない
        x = k.run(DT)
        speeds.append(x.speed)
        if c._ex_phase != "step":
            break
    assert STOP_REASONS[c.last_step[1]] == "done"
    assert 12.0 in speeds and -8.0 in speeds
    assert all(abs(d - 10.0) < 0.6 for d in c.step_driven[:3])
    # 前から後ろへ替わる所で、GEAR_PAUSE_S の間は 0
    i = speeds.index(-8.0)
    assert all(v == 0 for v in speeds[i - int(GEAR_PAUSE_S / DT) + 1 : i])


def test_reverse_segment_stops_when_stuck():
    c, ctl, est = _explore()
    k = Clock(c, est)
    c._ex_phase, c.station_wait = "plan", 0
    c.set_plan(plan(0, [(0.0, 20.0, True)]))
    assert k.run(DT).speed < 0
    c.stuck = True
    x = k.run(DT)
    assert x.speed == 0 and STOP_REASONS[c.last_step[1]] == "stuck"


def test_station_prev_step_from_what_was_driven(monkeypatch, tmp_path):
    """prev_step は制御プロセスが数えた距離・使った速さ・走った区切り（区切りごとの前・後ろ）"""
    import json
    from types import SimpleNamespace

    import requests

    from snoo_trucar.config import Config
    from snoo_trucar.control_proc import CMD_N, PLAN_N
    from snoo_trucar.shm import SeqBuffer
    from snoo_trucar.station import SUPPORTS, StationClient

    sent = []

    def post(url, data, files, timeout):
        sent.append(json.loads(data["meta"]))
        body = {"action": "move", "speed": 40, "reverse_speed": 8,
                "segments": [{"steer_deg": 20, "distance_cm": 15, "reverse": True},
                             {"steer_deg": 0, "distance_cm": 40, "reverse": False}]}  # fmt: skip
        return SimpleNamespace(status_code=200, text="", json=lambda: body)

    monkeypatch.setattr(requests, "post", post)
    cfg = Config()
    d = tmp_path / "explore_a"
    d.mkdir()
    rec = SimpleNamespace(dir=d, entries=[])
    buf = SeqBuffer(PLAN_N)
    v = [0.0] * CMD_N
    runner = SimpleNamespace(cfg=cfg, recorder=rec, battery_v=7.9, last_us=120.0, last_us_t_ns=5,
                             last_cmd=v)  # fmt: skip
    c = StationClient(runner, buf)
    c._session = "explore_a"
    try:
        assert c._post(0, v, rec)
        assert "gear_segments" in sent[-1]["supports"] and sent[-1]["supports"] == SUPPORTS
        assert sent[-1]["ultrasonic"] == {"cm": 120.0, "t_ns": 5}
        row = buf.read()[1]
        assert row[3] == 2 and tuple(map(float, row[5:8])) == (20.0, -15.0, -1.0) and row[4] == 8.0
        # station 0 の 1 歩：後ろへ 15 cm、前へ 10 cm で止まった
        v[19], v[20], v[22] = 1.2, STOP_REASONS.index("clear"), 0
        v[23], v[24], v[25], v[26], v[27], v[28] = 0, 15.0, 25.0, 15.0, 10.0, 0.0
        c._post(1, v, rec)
        prev = sent[-1]["prev_step"]
        assert prev["est_distance_cm"] == 25.0 and prev["speed"] == 15.0 and not prev["reverse"]
        assert prev["segments"] == [{"steer_deg": 20.0, "distance_cm": 15.0, "reverse": True},
                                    {"steer_deg": 0.0, "distance_cm": 10.0, "reverse": False}]  # fmt: skip
        assert prev["steer_deg"] == 0.0 and prev["stop_reason"] == "clear"
        # 走っていない（step_station = -1）なら prev_step は無し
        v[23] = -1
        c._post(2, v, rec)
        assert sent[-1]["prev_step"] is None
    finally:
        buf.close()


def _explore_with_snoo(session_s=None):
    from snoo_trucar.guard import GuardConfig

    ctl = ControlConfig(mode="explore")
    if session_s is not None:
        ctl.guard = GuardConfig(session_s=session_s)
    c = Controller(ctl, CarConfig())
    c._ex_phase, c.station_wait = "plan", 0
    c.set_plan(plan(0, [(0.0, 400.0, False)]))
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
    c._ex_phase, c.station_wait = "plan", 1
    c.set_plan(plan(1, [(0.0, 400.0, False)]))
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


def test_server_step_starts_while_waiting_for_the_first_clear():
    """1 歩を始めた瞬間は clear の欄がまだ無い（止まっていたフレームの返事）。すぐには止めず、
    clear_skip（走り出した直後）が来ればそのまま進む。0.5 s 待っても欄が無ければ（clear_cm を出さない
    サーバー）、見えていないので clear_skip と同じく遅く・20 cm まで"""
    from snoo_trucar.controller import CLEAR_ABSENT, CLEAR_SKIP, CLEAR_SKIP_MAX_CM, CLEAR_SKIP_SPEED

    c, ctl, est = _explore()
    k = Clock(c, est)
    c.set_clear(CLEAR_ABSENT, -1.0, k.t)
    c._ex_phase, c.station_wait = "plan", 0
    c.set_plan(plan(0, [(0.0, 80.0, False)]))
    assert k.run(DT).speed > 0
    assert k.run(0.1).speed > 0  # 欄が無いまま 0.1 s
    c.set_clear(CLEAR_SKIP, -1.0, k.t)  # 走り出したフレームの返事（settling）
    assert k.run(0.2).speed > 0
    c2, _, est2 = _explore()
    k2 = Clock(c2, est2)
    c2.set_clear(CLEAR_ABSENT, -1.0, k2.t)
    _server_step(c2, k2)
    assert k2.run(0.6).speed == CLEAR_SKIP_SPEED
    k2.run(CLEAR_SKIP_MAX_CM / (CLEAR_SKIP_SPEED * c2.car.cm_s_per_speed) + 0.1)
    assert STOP_REASONS[c2.last_step[1]] == "clear_skip"


