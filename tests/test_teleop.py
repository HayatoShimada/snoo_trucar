"""teleop モード：deadman・いつも効く安全・モードの出入り（snoo_trucar#27）"""

from types import SimpleNamespace

from snoo_trucar.config import CarConfig, ControlConfig
from snoo_trucar.control_proc import C_CLEAR_CM, C_TELEOP_LIMIT, CMD_N, TELEOP_N
from snoo_trucar.controller import (
    CLEAR_NULL,
    CLEAR_VALUE,
    MODES,
    TELEOP,
    Controller,
    limit_names,
)
from snoo_trucar.estimator import Detection, TargetEstimator

S = 1_000_000_000
DT = 0.01


class Car:
    def __init__(self):
        self.c = Controller(ControlConfig(mode="idle", max_speed=30.0), CarConfig())
        self.est = TargetEstimator(53.5, 41.4, 33.0)
        self.t = S
        self.c.step(self.est, self.t, DT)
        self.c.set_mode("teleop")
        self.c.set_clear(CLEAR_VALUE, 200.0, self.t, self.t)

    def send(self, steer=0.0, speed=0.0, pan=0.0, tilt=0.0, ok=1.0, seq=1):
        self.c.set_teleop((self.t, seq, steer, speed, pan, tilt, ok))

    def run(self, seconds, us=None, **cmd):
        """seconds の間 DT ごとに回す。cmd を渡せば、毎回その指令を受けたことにする"""
        x = None
        for _ in range(int(round(seconds / DT))):
            self.t += int(DT * S)
            if cmd:
                self.send(**cmd)
            x = self.c.step(self.est, self.t, DT, us)
        return x

    def limits(self):
        return limit_names(self.c.teleop_limit)


def test_drives_the_command_within_the_limits():
    car = Car()
    x = car.run(0.5, steer=40.0, speed=20.0, pan=10.0, tilt=-30.0)
    assert x.state == TELEOP and x.speed == 15.0 and x.steer == 28.0
    assert abs(x.pan - 10.0) < 1.0 and abs(x.tilt + 20.0) < 1.0  # チルトは -20 まで
    assert car.limits() == []
    assert car.run(0.1, speed=-20.0).speed == -10.0


def test_deadman_stops_after_0_3_s_and_keeps_the_head():
    car = Car()
    car.run(0.5, steer=10.0, speed=12.0, pan=20.0)
    pan = car.c.cmd.pan
    assert car.run(0.25).speed == 12.0  # まだ 0.3 s たっていない
    x = car.run(0.1)
    assert x.speed == 0.0 and x.steer == 10.0 and abs(x.pan - pan) < 2.0
    assert car.limits() == ["poll"]


def test_server_stale_or_stop_does_not_drive():
    car = Car()
    x = car.run(0.2, speed=12.0, ok=0.0)
    assert x.speed == 0.0 and car.limits() == ["deadman"]
    x = car.run(0.2, speed=12.0, ok=-1.0)
    assert x.speed == 0.0 and car.limits() == ["stopped"]


def test_never_received_and_old_commands_do_not_drive():
    car = Car()
    assert car.run(0.2).speed == 0.0 and car.limits() == ["deadman"]
    # teleop に入る前に受けた指令は使わない
    car.c.set_mode("idle")
    car.run(0.05)
    car.send(speed=12.0)
    car.c.set_mode("teleop")
    assert car.run(0.05).speed == 0.0


def test_clear_cm_stops_forward_but_not_reverse():
    car = Car()
    car.c.set_clear(CLEAR_VALUE, 15.0, car.t, car.t)
    x = car.run(0.1, speed=12.0)
    assert x.speed == 0.0 and car.limits() == ["clear"]
    assert car.run(0.1, speed=-8.0).speed == -8.0
    car.c.set_clear(CLEAR_NULL, -1.0, car.t, car.t)  # 分からない：前へは 10 まで
    assert car.run(0.1, speed=15.0).speed == 10.0 and car.limits() == ["clear"]
    assert car.run(0.1, speed=8.0).speed == 8.0 and car.limits() == []


def test_ultrasonic_never_stops_teleop():
    car = Car()
    for us in (None, -1.0, 5.0, 15.0, 15.0, 15.0):
        assert car.run(0.1, us=us, speed=12.0).speed == 12.0


def test_guard_stops_short_of_a_near_snoo():
    car = Car()
    car.c.set_clear(CLEAR_VALUE, 200.0, car.t, car.t)
    for _ in range(10):  # snoo が 35 cm に見えている
        car.t += int(0.03 * S)
        car.est.measure(Detection(car.t, 0.5, 0.5, 0.3, 33.0 / 35.0, 0.9), car.t, None)
        x = car.run(0.03, speed=15.0)
    assert x.speed == 0.0 and "guard" in car.limits()


def test_low_battery_stops_but_the_head_moves():
    car = Car()
    car.c.drive_ok = False
    x = car.run(0.5, speed=12.0, pan=25.0)
    assert x.speed == 0.0 and x.pan > 20.0 and car.limits() == ["battery"]


def test_other_modes_report_mode():
    car = Car()
    car.c.set_mode("watch")
    car.run(0.05)
    assert car.limits() == ["mode"]


# --- 問い合わせ ---


class Http:
    def __init__(self):
        self.answer = {"seq": 0, "command": None, "age_s": None, "stale": True, "stop": True,
                       "request_id": 0, "request_mode": None}  # fmt: skip
        self.queries = []
        self.down = False

    def get(self, url, params=None, timeout=None):
        if self.down:
            raise ConnectionError("届かない")
        assert url.endswith("/v1/snoo/teleop/latest") and timeout <= 0.25
        self.queries.append(params)
        ans = dict(self.answer)
        return SimpleNamespace(status_code=200, json=lambda: ans)


class Buf:
    def __init__(self):
        self.rows = []

    def write(self, row):
        assert len(row) == TELEOP_N
        self.rows.append(tuple(row))


def _client():
    from snoo_trucar.teleop import TeleopClient

    v = [0.0] * CMD_N
    v[14] = MODES.index("teleop")
    modes = []

    def set_mode(m):
        modes.append(m)
        v[14] = MODES.index(m)

    runner = SimpleNamespace(last_cmd=v, battery=("", True), battery_v=7.8, last_us=None,
                             current_mode=lambda: MODES[int(v[14])], set_mode=set_mode)  # fmt: skip
    http, buf = Http(), Buf()
    return TeleopClient(runner, buf, "http://srv:8720", session=http), http, buf, v, modes


def test_client_writes_commands_and_reports_what_was_written():
    t, http, buf, v, modes = _client()
    http.answer.update(seq=5, age_s=0.05, stale=False, stop=False,
                       command={"steer_deg": 11.2, "speed": 15.0, "pan_deg": 0.0, "tilt_deg": -4.0})
    t.once()
    assert buf.rows[-1][1:] == (5, 11.2, 15.0, 0.0, -4.0, 1.0) and t.after == 5
    v[1], v[2], v[3], v[4] = 11.2, 10.0, 0.0, -4.0
    v[C_TELEOP_LIMIT], v[C_CLEAR_CM] = 1 << 6, 42.0  # clear で落とした
    t.once()
    q = http.queries[-1]
    assert q["after"] == 5 and q["speed"] == 10.0 and q["limit"] == "clear"
    assert q["clear_cm"] == 42.0
    assert q["supports"] == "teleop,ui_mode"
    http.answer.update(stop=True, reason="button")
    t.once()
    assert buf.rows[-1][-1] == -1.0
    http.answer.update(stale=True, age_s=0.5)
    t.once()
    assert buf.rows[-1][-1] == 0.0
    n = len(buf.rows)
    http.down = True
    try:
        t.once()
    except ConnectionError:
        pass
    assert len(buf.rows) == n  # 書かない（制御プロセスが 0.3 s で止まる）


def test_client_changes_mode_only_when_request_id_grows():
    t, http, buf, v, modes = _client()
    v[14] = MODES.index("idle")
    http.answer.update(request_id=3, request_mode="teleop")
    t.once()
    assert modes == []  # 起動する前の頼みでは変えない
    http.answer.update(request_id=4, request_mode="teleop")
    t.once()
    assert modes == ["teleop"]
    v[14] = MODES.index("idle")  # USR の長押し
    t.once()
    t.once()
    assert modes == ["teleop"]  # 同じ頼みで teleop に戻らない
    http.answer.update(request_id=5, request_mode="explore")  # teleop・idle のほかは受けない
    t.once()
    assert modes == ["teleop"]
