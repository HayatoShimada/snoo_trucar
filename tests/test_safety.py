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
