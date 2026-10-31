"""agent モード：エージェントの指令を問い合わせて実行する（snoo_trucar#28）"""

import math
from types import SimpleNamespace

from snoo_trucar.config import CarConfig, ControlConfig
from snoo_trucar.control_proc import AGENT_N, C_AGENT_ID, CMD_N, PLAN_N
from snoo_trucar.controller import (
    A_LOOK,
    A_MOVE,
    A_SCAN,
    A_STOP,
    AGENT_REASONS,
    AS_CANCELLED,
    AS_DONE,
    AS_FAILED,
    AS_RUNNING,
    CLEAR_VALUE,
    MODES,
    PLAN_MOVE,
    Controller,
    scan_schedule,
)
from snoo_trucar.estimator import Detection, TargetEstimator
from snoo_trucar.guard import HOLD

S = 1_000_000_000
DT = 0.01
NAN = math.nan


class Car:
    """制御プロセスの代わり（Controller を DT ごとに回す）"""

    def __init__(self, mode="agent"):
        self.c = Controller(ControlConfig(mode=mode), CarConfig())
        self.est = TargetEstimator(53.5, 41.4, 33.0)
        self.t = S
        self.hb = S
        self.c.set_clear(CLEAR_VALUE, 200.0, self.t, self.t)

    def send(self, cid, kind, a=NAN, b=NAN, c=NAN, d=NAN):
        self.c.set_agent((self.hb, cid, kind, a, b, c, d))

    def run(self, seconds, beat=True):
        out = []
        for _ in range(int(round(seconds / DT))):
            self.t += int(DT * S)
            if beat:
                self.hb = self.t
                self.c.set_agent((self.hb, self.c._agent_seen, 0, NAN, NAN, NAN, NAN))
                self.c.set_clear(CLEAR_VALUE, 200.0, self.t, self.t)
            x = self.c.step(self.est, self.t, DT)
            out.append((x.steer, x.speed, x.pan, x.tilt))
        return out

    def result(self):
        c = self.c
        return c.agent_id, c.agent_status, AGENT_REASONS[c.agent_reason]


def test_move_is_limited_and_reports_the_distance():
    car = Car()
    car.send(1, A_MOVE, 50.0, 80.0, 40.0, 0.0)  # 頼んだのは上限より大きい
    out = car.run(3.0)
    assert max(s for _, s, _, _ in out) == 10.0  # 速さ 10 まで
    assert max(st for st, _, _, _ in out) == 30.0  # ハンドル ±30° まで
    assert car.result() == (1, AS_DONE, "done")
    assert 29.0 <= car.c.agent_moved <= 31.0  # 30 cm まで
    assert out[-1][1] == 0.0


def test_reverse_move():
    car = Car()
    car.send(1, A_MOVE, 0.0, 10.0, 8.0, 1.0)
    out = car.run(1.5)
    assert min(s for _, s, _, _ in out) == -8.0 and car.result() == (1, AS_DONE, "done")


def test_heartbeat_lost_stops_the_move():
    car = Car()
    car.send(1, A_MOVE, 0.0, 30.0, 5.0, 0.0)
    assert car.run(0.5)[-1][1] > 0
    out = car.run(3.2, beat=False)  # 問い合わせが通らない
    assert out[-1][1] == 0.0 and car.result() == (1, AS_FAILED, "heartbeat")
    stopped_at = next(i for i, x in enumerate(out) if x[1] == 0.0)
    assert 2.95 <= stopped_at * DT <= 3.05  # 最後に通ってから 3 s


def test_stop_cancels_the_running_move():
    car = Car()
    car.send(1, A_MOVE, 0.0, 30.0, 5.0, 0.0)
    car.run(0.3)
    car.send(2, A_STOP)
    assert car.c.agent_status == AS_DONE and car.c.agent_id == 2
    assert car.run(0.1)[-1][1] == 0.0


def test_move_stops_for_guard_battery_snoo_and_clear():
    car = Car()
    car.c.guard.state = HOLD
    car.send(1, A_MOVE, 0.0, 30.0, 5.0, 0.0)
    assert car.run(0.05)[-1][1] == 0.0 and car.result()[1:] == (AS_FAILED, "guard")
    car.c.guard.state = 0
    car.c.drive_ok = False
    car.send(2, A_MOVE, 0.0, 30.0, 5.0, 0.0)
    car.run(0.05)
    assert car.result()[1:] == (AS_FAILED, "battery")
    car.c.drive_ok = True
    car.send(3, A_MOVE, 0.0, 30.0, 5.0, 0.0)
    # snoo が 100 cm に見えた（150 cm より近い）
    car.est.measure(Detection(car.t, 0.5, 0.5, 0.2, 33.0 / 100.0, 0.9), car.t, None)
    car.run(0.05)
    assert car.result()[1:] == (AS_FAILED, "snoo_near")
    car2 = Car()
    car2.send(1, A_MOVE, 0.0, 30.0, 5.0, 0.0)
    car2.run(0.2)
    car2.c.set_clear(CLEAR_VALUE, 10.0, car2.t, car2.t)
    car2.t += int(DT * S)
    car2.c.step(car2.est, car2.t, DT)
    assert car2.result()[1:] == (AS_FAILED, "clear")


def test_look_and_scan_do_not_drive():
    car = Car()
    car.send(1, A_LOOK, 50.0, NAN)  # パンは ±35° に収める。チルトは今のまま
    out = car.run(0.6)
    assert car.result() == (1, AS_DONE, "done")
    assert abs(out[-1][2] - 35.0) < 5.0 and max(abs(s) for _, s, _, _ in out) == 0.0
    car.send(2, A_SCAN)
    out = car.run(scan_schedule(35.0, 0.0, car.c.scan_poses)[-1] + 0.1)
    assert car.c.station_wait == 0 and car.c.agent_status == AS_RUNNING
    car.c.set_plan((0, PLAN_MOVE, 12.0, 1, 12.0, 0.0, 40.0, -1.0) + (0.0,) * (PLAN_N - 8))
    out += car.run(1.0)
    assert car.result() == (2, AS_DONE, "done")
    assert max(abs(s) for _, s, _, _ in out) == 0.0  # 返ってきた 1 歩は走らない
    assert abs(out[-1][2] - 35.0) < 5.0  # look の向きに戻る


def test_commands_outside_agent_mode_never_run_later():
    car = Car(mode="idle")
    car.send(1, A_MOVE, 0.0, 30.0, 5.0, 0.0)
    car.run(0.1)
    car.c.set_mode("agent")
    out = car.run(1.0)  # 同じ指令が問い合わせのたびに書かれても、動かさない
    assert max(s for _, s, _, _ in out) == 0.0 and car.c.agent_id == 0


def test_leaving_agent_mode_cancels():
    car = Car()
    car.send(1, A_MOVE, 0.0, 30.0, 5.0, 0.0)
    car.run(0.3)
    car.c.set_mode("idle")  # USR の長押し
    assert car.result() == (1, AS_CANCELLED, "mode")
    assert car.run(0.1)[-1][1] == 0.0


# --- メインプロセスの問い合わせ ---


class Http:
    """requests.Session の代わり。commands/next に並べた指令を順に返し、結果を覚える"""

    def __init__(self):
        self.queue, self.results, self.queries = [], [], []
        self.now, self.down = 1000.0, False

    def get(self, url, params=None, timeout=None):
        if self.down:
            raise ConnectionError("届かない")
        self.queries.append(params)
        c = next((x for x in self.queue if x["id"] > params["after"]), None)
        body = {"now": self.now, "heartbeat_s": 3.0, "command": c}
        return SimpleNamespace(status_code=200, json=lambda: body)

    def post(self, url, json=None, timeout=None):
        if self.down:
            raise ConnectionError("届かない")
        cid = int(url.rsplit("/", 2)[-2])
        self.results.append((cid, json["status"], json["reason"], json["detail"]))
        return SimpleNamespace(status_code=200)

    def statuses(self, cid):
        return [(s, r) for i, s, r, _ in self.results if i == cid]


class Buf:
    def __init__(self):
        self.rows = []

    def write(self, row):
        assert len(row) == AGENT_N
        self.rows.append(list(row))


def _client(mode="agent"):
    from snoo_trucar.agent import AgentClient

    v = [0.0] * CMD_N
    v[14] = MODES.index(mode)
    runner = SimpleNamespace(last_cmd=v, battery=("", True), battery_v=7.8, last_us=None,
                             current_mode=lambda: MODES[int(v[14])])  # fmt: skip
    http, buf = Http(), Buf()
    return AgentClient(runner, buf, "http://srv:8720", session=http), http, buf, v


def _cmd(cid, kind, **kw):
    return {"id": cid, "t": 999.0, "expires_at": 1015.0, "kind": kind, **kw}


def test_client_refuses_outside_agent_mode_and_bad_commands():
    a, http, buf, v = _client(mode="explore")
    http.queue.append(_cmd(1, "move", steer_deg=0, distance_cm=20, speed=8))
    a.once()
    assert http.statuses(1) == [("accepted", None), ("refused", "not_agent_mode")]
    assert http.queries[-1]["mode"] == "explore" and "move" in http.queries[-1]["supports"]
    v[14] = MODES.index("agent")
    http.queue += [_cmd(2, "move", distance_cm=-5, speed=8), _cmd(3, "dance")]
    a.once()
    a.once()
    assert http.statuses(2)[-1] == ("refused", "bad_params")
    assert http.statuses(3)[-1] == ("refused", "bad_params")
    a.runner.battery = ("", False)
    http.queue.append(_cmd(4, "move", steer_deg=0, distance_cm=20, speed=8))
    a.once()
    assert http.statuses(4)[-1] == ("refused", "battery_low")
    a.runner.battery = ("", True)
    v[17] = HOLD
    http.queue.append(_cmd(5, "move", steer_deg=0, distance_cm=20, speed=8))
    a.once()
    assert http.statuses(5)[-1] == ("refused", "guard")
    http.queue.append(_cmd(6, "look", pan_deg=10, expires_at=999.0))  # 期限切れ
    a.once()
    assert http.statuses(6)[-1] == ("expired", None)
    assert all(int(r[1]) == 0 for r in buf.rows)  # どれも制御プロセスに渡していない


def test_client_runs_a_move_and_reports_the_result():
    a, http, buf, v = _client()
    http.queue.append(_cmd(1, "move", steer_deg=-10, distance_cm=20, speed=8))
    a.once()
    assert buf.rows[-1][1:7] == [1.0, A_MOVE, -10.0, 20.0, 8.0, 0.0]
    http.queue.append(_cmd(2, "look", pan_deg=10))
    v[C_AGENT_ID:C_AGENT_ID + 5] = [1, AS_RUNNING, 0, 0.0, 0.0]
    a.once()
    assert http.statuses(2)[-1] == ("refused", "busy")  # 1 つずつ
    assert http.queries[-1]["running"] == 1
    v[C_AGENT_ID:C_AGENT_ID + 5] = [1, AS_DONE, AGENT_REASONS.index("done"), 19.6, 2.1]
    a.once()
    st = http.statuses(1)
    assert st[:3] == [("accepted", None), ("running", None), ("done", None)]
    detail = [d for i, s, r, d in http.results if i == 1 and s == "done"][0]
    assert detail["moved_cm"] == 19.6 and detail["stop_reason"] == "done" and detail["speed"] == 8
    assert a.running is None and len(a.moves) == 1


def test_client_heartbeat_and_unsent_results():
    """問い合わせが通るたびに時刻を書く（通らなければ書かない → 制御プロセスが 3 s で止まる）。
    書けなかった結果は、通ったときに書き直す"""
    a, http, buf, v = _client()
    a.once()
    n = len(buf.rows)
    http.down = True
    try:
        a.once()
    except ConnectionError:
        pass
    assert len(buf.rows) == n
    http.queue.append(_cmd(1, "move", steer_deg=0, distance_cm=20, speed=8))
    http.down = False
    a.once()
    v[C_AGENT_ID:C_AGENT_ID + 5] = [1, AS_FAILED, AGENT_REASONS.index("heartbeat"), 4.0, 3.0]
    http.down = True
    a._watch()  # 書けない
    assert a.unsent
    http.down = False
    a.once()
    assert http.statuses(1)[-1] == ("failed", "heartbeat") and not a.unsent


def test_client_stop_and_button_cancel():
    a, http, buf, v = _client()
    http.queue.append(_cmd(1, "move", steer_deg=0, distance_cm=20, speed=8))
    a.once()
    http.queue.append(_cmd(2, "stop"))
    a.once()
    assert buf.rows[-1][1:3] == [2.0, A_STOP]
    assert http.statuses(1)[-1] == ("cancelled", "stop") and http.statuses(2)[-1] == ("done", None)
    http.queue.append(_cmd(3, "move", steer_deg=0, distance_cm=20, speed=8))
    a.once()
    a.on_button()  # USR の長押し
    assert http.statuses(3)[-1] == ("cancelled", "button")
    # 長押しより前に並んだ指令は、agent に戻っても実行しない
    http.queue.append(_cmd(4, "look", pan_deg=5, t=999.5))
    a.once()
    assert http.statuses(4)[-1] == ("cancelled", "button")


def test_prev_step_for_agent_scans():
    a, http, buf, v = _client()
    assert a.prev_step()["est_distance_cm"] == 0.0  # 動いていない
    a.moves = [{"steer_deg": 0, "moved_cm": 20.0, "reverse": False, "seconds": 2.0, "speed": 8,
                "stop_reason": "done"},
               {"steer_deg": -10, "moved_cm": 12.5, "reverse": False, "seconds": 1.5, "speed": 8,
                "stop_reason": "clear"}]  # fmt: skip
    p = a.prev_step()
    assert [s["distance_cm"] for s in p["segments"]] == [20.0, 12.5]
    assert p["est_distance_cm"] == 32.5 and p["stop_reason"] == "clear" and not p["reverse"]
    a.moves.append({"steer_deg": 0, "moved_cm": 5.0, "reverse": True, "seconds": 1.0, "speed": 8,
                    "stop_reason": "done"})  # fmt: skip
    assert a.prev_step() == {"stop_reason": "agent"}  # 向きが混じった
    a.prev_step_sent()
    v[2] = -12.0  # 指令の無い間に guard が下がった
    a._watch()
    assert a.prev_step() == {"stop_reason": "agent"}
