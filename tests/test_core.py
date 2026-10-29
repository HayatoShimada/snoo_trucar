import dataclasses
import math

import pytest

from snoo_trucar.buttons import ClickCounter, event_to_mode
from snoo_trucar.config import CarConfig, ControlConfig, Config
from snoo_trucar.controller import MODES, IDLE, MAP, MAP_HOLD_S, MAP_PANS, MAP_TILTS, SEARCH, TRACK, Controller
from snoo_trucar.estimator import Detection, TargetEstimator
from snoo_trucar.mapping import MapRecorder
from snoo_trucar.protocol import make_reply, pack_frame, parse_reply, pick_snoo, unpack_frame
from snoo_trucar.shm import SeqBuffer

S = 1_000_000_000


def test_protocol_roundtrip():
    data = pack_frame(7, 123456789, 640, 480, b"\xff\xd8jpeg")
    assert unpack_frame(data) == (7, 123456789, 640, 480, b"\xff\xd8jpeg")
    reply = parse_reply(
        make_reply(7, 123456789, 9.5, [{"box": [0.2, 0.4, 0.6, 0.8], "cat": 0.9, "snoo": None}])
    )
    assert reply.seq == 7 and reply.t_capture_ns == 123456789
    b = reply.boxes[0]
    assert (b.cx, b.cy, b.w, b.h) == pytest.approx((0.4, 0.6, 0.4, 0.4))
    assert b.snoo is None


def test_pick_snoo():
    reply = parse_reply(make_reply(1, 0, 0, [
        {"box": [0, 0, 0.1, 0.1], "cat": 0.9, "snoo": 0.2},  # 別の猫
        {"box": [0.5, 0.5, 0.7, 0.7], "cat": 0.8, "snoo": 0.9},
        {"box": [0.1, 0.1, 0.2, 0.2], "cat": 0.1, "snoo": 0.99},  # 猫ではなさそう
    ]))  # fmt: skip
    box, _ = pick_snoo(reply.boxes, 0.35, 0.6, True)
    assert box.cx == pytest.approx(0.6)
    # snoo を判定しないサーバー
    reply = parse_reply(make_reply(1, 0, 0, [{"box": [0, 0, 0.2, 0.2], "cat": 0.7, "snoo": None}]))
    assert pick_snoo(reply.boxes, 0.35, 0.6, True)[0] is None
    assert pick_snoo(reply.boxes, 0.35, 0.6, False)[0] is not None
    # 暗い画像で出る、画面全体の箱
    reply = parse_reply(make_reply(1, 0, 0, [{"box": [0, 0, 1, 1], "cat": 0.5, "snoo": None}]))
    assert pick_snoo(reply.boxes, 0.35, 0.6, False)[0] is None


def test_seqbuffer():
    a = SeqBuffer(3)
    b = SeqBuffer(3, a.name)
    assert b.read() == (0, pytest.approx([0, 0, 0]))
    a.write((1.0, 2.0, 3.0))
    n, v = b.read()
    assert n == 1 and list(v) == [1.0, 2.0, 3.0]
    b.close()
    a.close()


def test_estimator_bearing_and_latency():
    est = TargetEstimator(53.5, 41.4, 33.0)
    est.record_pose(0, 20.0, 0.0)  # カメラは右 20°
    # 画面の中央に、高さ 0.33（≒ 100 cm）
    est.measure(Detection(0, 0.5, 0.5, 0.2, 0.33, 0.9), int(0.05 * S))
    assert est.bearing.x[0] == pytest.approx(20.0, abs=0.5)
    assert est.dist.x[0] == pytest.approx(100.0, rel=0.05)
    # 右に 30°/s で動く snoo を 30 fps・60 ms 遅れで見ると、推定は今の位置に追いつく
    fx = 0.5 / math.tan(math.radians(53.5 / 2))
    est.record_pose(0, 0.0, 0.0)
    for k in range(1, 60):
        t = k / 30
        true_then = 20.0 + 30.0 * (t - 0.06)
        cx = 0.5 + fx * math.tan(math.radians(true_then - 20.0))
        est._pose.clear()
        est.record_pose(0, 20.0, 0.0)
        est.measure(Detection(int((t - 0.06) * S), cx, 0.5, 0.2, 0.33, 0.9), int(t * S))
    assert est.bearing.x[1] == pytest.approx(30.0, rel=0.2)
    assert est.bearing.x[0] == pytest.approx(20.0 + 30.0 * (59 / 30), abs=2.0)


def _controller(mode):
    ctl = ControlConfig(mode=mode)
    return Controller(ctl, CarConfig()), ctl


def _seen(est, now_ns, bearing, dist):
    est.valid = True
    est.last_seen_ns = now_ns
    est.bearing.x[:] = (bearing, 0.0)
    est.dist.x[:] = (dist, 0.0)


def test_controller_modes():
    est = TargetEstimator(53.5, 41.4, 33.0)
    c, ctl = _controller("idle")
    assert c.step(est, S, 1e-3, None).state == IDLE

    c, ctl = _controller("watch")
    cmd = c.step(est, S, 1e-3, None)
    assert cmd.state == SEARCH and cmd.speed == 0
    _seen(est, S, 10.0, 200.0)
    cmd = c.step(est, S, 1e-3, None)
    assert cmd.state == TRACK and cmd.speed == 0 and cmd.pan > 0  # watch では走らない

    c, ctl = _controller("keep")
    cmd = c.step(est, S, 1e-3, None)
    assert cmd.speed > 0 and cmd.steer > 0  # 遠くて右 → 右に切って進む
    _seen(est, S, 10.0, 30.0)
    cmd = c.step(est, S, 1e-3, None)
    assert cmd.speed < 0 and cmd.steer < 0  # 近すぎる → 逆に切って下がる
    _seen(est, S, 0.0, 300.0)
    cmd = c.step(est, S, 1e-3, ctl.stop_distance_cm - 1)
    assert cmd.speed <= 0  # 正面に障害物

    c, ctl = _controller("follow")
    _seen(est, S, 0.0, 30.0)
    assert c.step(est, S, 1e-3, None).speed == 0  # follow は下がらない

    # 見失ったら探す
    c, ctl = _controller("keep")
    _seen(est, S, 0.0, 200.0)
    assert c.step(est, S + int((ctl.lost_s + 0.1) * S), 1e-3, None).state == SEARCH


def test_controller_map_scan_then_idle():
    est = TargetEstimator(53.5, 41.4, 33.0)
    c, _ = _controller("map")
    seen = set()
    dt = 0.01
    for k in range(int(len(MAP_PANS) * len(MAP_TILTS) * MAP_HOLD_S / dt) + 10):
        cmd = c.step(est, S + int(k * dt * S), dt, None)
        if cmd.state == MAP:
            seen.add(cmd.scan_index)
            assert cmd.speed == 0
    assert seen == set(range(len(MAP_PANS) * len(MAP_TILTS)))
    assert c.mode == "idle" and cmd.state == IDLE


def test_click_counter():
    def run(presses):
        c, events, t = ClickCounter(), [], 0.0
        for down, up in presses:
            while t < down:
                events.append(c.feed(False, t))
                t += 0.01
            while t < up:
                events.append(c.feed(True, t))
                t += 0.01
        for _ in range(100):
            events.append(c.feed(False, t))
            t += 0.01
        return [e for e in events if e]

    assert run([(0.1, 0.2)]) == ["click:1"]
    assert run([(0.1, 0.2), (0.4, 0.5)]) == ["click:2"]
    assert run([(0.1, 0.2), (0.4, 0.5), (0.7, 0.8)]) == ["click:3"]
    assert run([(0.1, 2.0)]) == ["long"]
    assert [event_to_mode(e) for e in ("click:1", "click:2", "click:3", "long", "click:5")] == [
        "watch", "keep", "map", "idle", None,
    ]  # fmt: skip


def test_map_recorder(tmp_path, monkeypatch):
    rec = MapRecorder(tmp_path)
    clock = [0.0]
    monkeypatch.setattr("snoo_trucar.mapping.time.monotonic", lambda: clock[0])
    cmd = [0.0] * 15
    cmd[14] = MODES.index("map")
    for index in (0, 1):
        cmd[13], cmd[3] = index, -35.0 + 10 * index
        for _ in range(16):  # 1.6 s（MAP_HOLD_S）
            rec.offer(1, b"jpeg", cmd, 50.0)
            clock[0] += 0.1
    cmd[13], cmd[14] = -1, MODES.index("idle")
    rec.offer(1, b"jpeg", cmd, None)
    (d,) = tmp_path.iterdir()
    import json

    scan = json.loads((d / "scan.json").read_text())
    assert [f["pan_deg"] for f in scan["frames"]] == [-35.0, -25.0]
    assert all((d / f["file"]).exists() for f in scan["frames"])


def test_sim_finds_snoo():
    """シミュレーションで、見えない位置にいる snoo を見つけて、keep で距離を保つ"""
    from snoo_trucar.runner import Runner

    cfg = Config()
    cfg.control.mode = "keep"
    cfg.control.cpu = -1
    r = Runner(cfg, sim=True, seconds=20, quiet=True, buttons=False)
    r.run()
    h = r.history
    first_track = next(t for t, state, *_ in h if state == TRACK)
    assert first_track < 8.0
    track = [x for x in h if x[1] == TRACK and x[0] > first_track + 1]
    assert len(track) > 0.5 * len([x for x in h if x[0] > first_track + 1])
    errs = [abs(b_est - b_true) for _, _, b_est, _, b_true, *_ in track]
    assert sorted(errs)[len(errs) // 2] < 3.0  # 方位の推定の誤差（中央値）
    dists = [x[5] for x in track]  # 真の距離
    assert min(dists) > 15.0  # ぶつかっていない


def test_confirm_needs_repeated_hits():
    from snoo_trucar.link import Confirm

    c = Confirm(3, 0.5, 0.7)
    ms = 1_000_000
    assert not c.hit(0)  # 1 回だけの誤検出は通さない
    assert not c.hit(1000 * ms)  # 間があいた
    assert not c.hit(1033 * ms)
    assert c.hit(1066 * ms)  # 0.5 秒以内に 3 回
    assert c.hit(1500 * ms)  # 追っている間は 1 回でも通す
    assert not c.hit(3000 * ms)  # 見失ったあとは、また 3 回要る


def test_low_light_boost_raises_gain_only_when_ae_is_saturated():
    from snoo_trucar.camera import LowLightBoost

    boost = LowLightBoost(frame_us=16666, max_gain=16.0, target=60.0, every=1)
    assert boost.update(10000, 4.0, 30.0) is None  # AE にまだ余裕がある
    assert boost.update(16500, 8.0, 70.0) is None  # 上限だが十分明るい
    ctl = boost.update(16500, 8.0, 30.0)  # 上限で暗い → 8 × 60/30 = 16
    assert ctl == {"AnalogueGainMode": 1, "AnalogueGain": 16.0}
    assert boost.update(16500, 16.0, 58.0) is None  # 目標に近ければ変えない
    assert boost.update(16500, 16.0, 200.0) == {"AnalogueGainMode": 0}  # 明るくなったら AE に戻す
    assert boost.gain is None


def test_capture_time_is_the_middle_of_the_centre_row():
    from snoo_trucar.camera import capture_time_ns

    # 読み出しの始まり 1 s、露光 16 ms、読み出し 15 ms → 1 s − 8 ms + 7.5 ms
    assert capture_time_ns(1_000_000_000, 16000, 15.0) == 999_500_000


def test_controller_leads_the_target_by_lead_s():
    est = TargetEstimator(53.5, 41.4, 33.0)
    controller, ctl = _controller("watch")
    _seen(est, S, 10.0, 100.0)
    est.bearing.x[1] = 100.0  # 右に 100°/s
    goal = 10.0 + 100.0 * ctl.lead_s
    for _ in range(2000):  # カメラが目標に落ち着くまで回す
        cmd = controller.step(est, S, 0.001, None)
    assert cmd.pan == pytest.approx(goal, abs=0.1)


def test_trace_sections_from_capture_to_applied(tmp_path):
    import json

    from snoo_trucar.trace import Trace

    ms = 1_000_000
    trace = Trace(tmp_path / "t.jsonl")
    t = 10 * S
    for point, at in (("grab", 19), ("encoded", 24), ("sent", 25), ("reply", 42), ("meas", 43),
                      ("used", 44), ("applied", 60)):  # fmt: skip
        trace.mark(t, point, t + at * ms)
    trace.mark(t, "reply", t + 99 * ms)  # 閉じたフレームには足さない
    line = trace.summary()
    assert "カメラ 19.0/19.0/19.0" in line
    assert "往復 17.0/17.0/17.0" in line
    assert "撮影→書き込み 60.0/60.0/60.0" in line
    frame = json.loads((tmp_path / "t.jsonl").read_text())
    assert frame["applied"] - frame["capture"] == 60 * ms
    trace.close()


def test_trace_closes_frames_that_never_reach_the_motors():
    from snoo_trucar.trace import MAX_OPEN, Trace

    trace = Trace()
    for k in range(MAX_OPEN + 5):  # snoo が見えていないフレームは返事まで
        trace.mark(k * S, "grab", k * S + 19_000_000)
        trace.mark(k * S, "reply", k * S + 40_000_000)
    line = trace.summary()
    assert "撮影→返事 40.0/40.0/40.0" in line and "撮影→書き込み" not in line


def test_servo_step_shift_and_crossing():
    import numpy as np

    from snoo_trucar.servo_step import crossing, shift_px

    rng = np.random.default_rng(0)
    a = rng.normal(size=(120, 160)).astype(np.float32)
    b = np.roll(a, 7, axis=1)  # 右に 7 画素
    assert shift_px(a, b) == pytest.approx(7.0, abs=0.3)
    assert crossing([0.0, 10.0, 20.0], [0.0, 0.2, 1.0], 0.5) == pytest.approx(13.75)


def test_route_prefers_lan_and_falls_back_to_tailscale():
    from snoo_trucar.link import Route

    names = {"lan": "192.168.0.248", "ts": "100.108.168.101"}

    def resolve(host):
        if host not in names:
            raise OSError(host)
        return names[host]

    r = Route(["lan", "ts"], 8710, switch_s=1.0, retry_s=30.0, resolve=resolve)
    assert r.addr(0) == ("192.168.0.248", 8710)
    r.on_reply(int(0.5 * S))
    assert r.addr(int(1.4 * S)) == ("192.168.0.248", 8710)  # 返事から 1 s 以内
    assert r.addr(int(1.6 * S)) == ("100.108.168.101", 8710)  # 返事が途切れたら tailscale へ
    r.on_reply(int(19.5 * S))
    assert r.addr(int(20 * S)) is not None and r.host == "ts"  # 返事が来ている間は替えない
    r.on_reply(int(31.5 * S))
    assert r.addr(int(32 * S)) == ("192.168.0.248", 8710)  # 30 s たったら LAN を試し直す
    del names["lan"]  # 家の外：LAN の名前が引けなければすぐ次へ
    r2 = Route(["lan", "ts"], 8710, 1.0, 30.0, resolve=resolve)
    assert r2.addr(0) == ("100.108.168.101", 8710)


def test_estimator_does_not_mistake_turning_for_snoo_moving():
    """車体が右へ 60°/s で回っても、止まっている snoo の速度は 0 のまま（snoo_trucar#17）"""
    est = TargetEstimator(53.5, 41.4, 33.0)
    fx = 0.5 / math.tan(math.radians(53.5 / 2))
    rate, lag = 60.0, 0.04
    for k in range(0, 1001):  # 1 ms ごとに 1 秒
        now = k * S // 1000
        est.set_ego(rate)
        est.predict_to(now)
        est.record_pose(now, 0.0, 0.0)
        if k % 17 == 0 and k / 1000 >= lag:
            t = k / 1000 - lag
            true_then = 10.0 - rate * t  # 撮った時の方位（右へ回るので左へずれる）
            cx = 0.5 + fx * math.tan(math.radians(true_then))
            est.measure(Detection(int(t * S), cx, 0.5, 0.2, 0.33, 0.9), now)
    assert est.bearing.x[0] == pytest.approx(10.0 - rate * 1.0, abs=1.0)
    assert est.bearing.x[1] == pytest.approx(0.0, abs=5.0)


def test_controller_turns_the_body_even_inside_the_distance_band():
    """距離が目標の内でも、首の角度が大きければゆっくり動いて車体の向きを変える（snoo_trucar#18）"""
    from snoo_trucar.controller import ego_yaw_rate

    controller, ctl = _controller("keep")
    steer, speed = controller._approach(25.0, ctl.keep_distance_cm, None)
    assert speed == ctl.creep_speed and steer > 0
    # 右へ回る向きで、速さによらず帯域は heading_hz（角速度 = 2π f × 方位）
    rate = ego_yaw_rate(steer, speed, controller.car)
    want = math.degrees(2 * math.pi * ctl.heading_hz * math.radians(25.0))
    assert rate == pytest.approx(want, rel=0.05)
    # 正面なら止まったまま
    assert controller._approach(2.0, ctl.keep_distance_cm, None) == (0.0, 0.0)


def test_bench_keeps_track_of_a_circling_snoo():
    from snoo_trucar.bench import run

    r = run("turn", seed=0).summary()
    assert r["見失った回数"] == 0
    assert r["向きのずれ p95"] < 8.0
    assert r["いちばん近づいた距離"] > 25.0


def test_battery_state():
    from snoo_trucar.runner import battery_state

    assert battery_state(7.8, 6.6) == ("電池=7.8V", True)
    assert battery_state(6.2, 6.6)[1] is False  # 少ない
    label, ok = battery_state(0.0, 6.6)
    assert not ok and "入っていない" in label
    assert battery_state(None, 6.6)[1] is True  # 測れないときは止めない


def test_guard_stress_backs_off_then_waits_for_calm():
    from snoo_trucar.guard import BACKOFF, HOLD, OK, REST, Guard, GuardConfig

    g = Guard(GuardConfig(), 1.0)
    dt = 0.01
    assert g.apply(True, dt, 5.0, 20.0, 100.0, 0.0) == (5.0, 20.0)
    for s in (0.6, 0.6):
        g.observe_stress(s)
    assert g.state == OK  # 0.5 以上でも 2 回では引かない
    g.observe_stress(0.6)
    assert g.state == BACKOFF
    moved = 0.0
    while g.state == BACKOFF:
        steer, speed = g.apply(True, dt, 5.0, 20.0, 100.0, 0.0)
        assert speed < 0 and steer == 0
        moved -= speed * dt
    assert g.state == HOLD and 20 <= moved <= 30
    g.observe_stress(0.1)
    for _ in range(int(4.9 / dt)):
        assert g.apply(True, dt, 0.0, 20.0, 100.0, 0.0)[1] == 0
    g.observe_stress(0.3)  # まだ落ち着いていない → 待ち直す
    for _ in range(int(3 / dt)):
        g.apply(True, dt, 0.0, 20.0, 100.0, 0.0)
    assert g.state == HOLD
    g.observe_stress(0.1)
    for _ in range(int(5.1 / dt)):
        g.apply(True, dt, 0.0, 20.0, 100.0, 0.0)
    assert g.state == OK
    # 1 回の遊びで 3 回引いたら休む
    g.observe_stress(0.9)
    g.state = OK
    g.observe_stress(0.9)
    assert g.state == REST and g.apply(True, dt, 5.0, 20.0, 100.0, 0.0) == (0.0, 0.0)


def test_guard_rules_without_play_state():
    from snoo_trucar.guard import BACKOFF, REST, Guard, GuardConfig

    c = GuardConfig()
    g = Guard(c, 1.0)
    dt = 0.01
    # 1. 自分が動いて 30 cm より近づかない。下がるのはよい
    assert g.apply(True, dt, 0.0, 20.0, 35.0, 0.0)[1] == 0
    assert g.apply(True, dt, 0.0, -20.0, 35.0, 0.0)[1] == -20.0
    # 2. 近づいた直後に 50 cm/s で離れたらストレス
    g.apply(True, dt, 0.0, 20.0, 100.0, 0.0)
    for _ in range(int(c.flee_hold_s / dt) + 1):
        g.apply(True, dt, 0.0, 0.0, 100.0, 60.0)
    assert g.state == BACKOFF and g.triggers == 1
    # 近づいていなければ、離れていっても引かない
    g = Guard(c, 1.0)
    for _ in range(int(2 / dt)):
        g.apply(True, dt, 0.0, 0.0, 100.0, 60.0)
    assert g.triggers == 0
    # 3. 3 分で休む。休む間は走らない
    for _ in range(int(c.session_s / 0.1) + 1):
        g.apply(True, 0.1, 0.0, 10.0, 200.0, 0.0)
    assert g.state == REST
    assert g.apply(True, 0.1, 0.0, 10.0, 200.0, 0.0) == (0.0, 0.0)


def test_play_stress_in_reply():
    from snoo_trucar.protocol import make_reply

    det = {"box": [0.1, 0.1, 0.3, 0.4], "cat": 0.9, "snoo": 0.9,
           "play": {"state": "stalk", "p": {"stress": 0.7}}}
    assert parse_reply(make_reply(1, 2, 3.0, [det])).boxes[0].stress == 0.7
    det["play"] = None
    assert parse_reply(make_reply(1, 2, 3.0, [det])).boxes[0].stress is None


def test_explore_scans_drives_and_turns():
    from snoo_trucar.controller import DRIVE_INDEX, EXPLORE, EXPLORE_PANS, EXPLORE_TILTS

    est = TargetEstimator(53.5, 41.4, 33.0)
    c, ctl = _controller("explore")
    ctl.explore_server = False  # Pi だけで決める探索
    dt = 0.01
    t = [0.0]

    def run(seconds, us):
        out = []
        for _ in range(int(seconds / dt)):
            t[0] += dt
            out.append(dataclasses.replace(c.step(est, S + int(t[0] * S), dt, us)))  # step は同じものを返す
        return out

    from snoo_trucar.controller import scan_schedule

    n_poses = len(EXPLORE_PANS) * len(EXPLORE_TILTS)
    scan_s = scan_schedule(0.0, 0.0)[-1]
    assert scan_s < n_poses * 0.65  # 1 向き 0.6 s ほど
    cmds = run(scan_s - 0.05, 200.0)
    assert all(x.state == EXPLORE and x.speed == 0 for x in cmds)  # 止まって撮る
    assert {x.scan_index for x in cmds} == set(range(n_poses))
    cmds = run(0.5, 200.0)
    assert cmds[-1].speed > 0 and cmds[-1].scan_index == DRIVE_INDEX  # 前が空いていれば進む
    # 前が塞がった → 切り返しで回る（ハンドルを切って下がる → 逆に切って進む）
    x = run(0.1, 40.0)[-1]
    assert x.speed < 0 and abs(x.steer) == c.car.steer_max
    back_steer = x.steer
    x = run(ctl.explore_turn_s, 60.0)[-1]  # 進む間は 50 cm 以上空いていれば進む
    assert x.speed > 0 and x.steer == -back_steer
    # 前が空いたら、撮り直さずにそのまま進む
    cmds = run(ctl.explore_turn_s + 0.5, 200.0)
    assert cmds[-1].speed > 0 and cmds[-1].steer == 0
    cmds = run(ctl.explore_step_cm / (ctl.explore_speed * c.car.cm_s_per_speed) + 0.1, 200.0)
    assert cmds[-1].speed == 0 and cmds[-1].scan_index == n_poses  # 次の場所で撮り始める
    # -1（反響なし）は、少し前まで測れていたなら遠くまで空いているとみなして進む
    c2, _ = _controller("explore")
    c2._ex_us_at = 0.0
    c2._ex_phase, c2._ex_t = "check", 1.0
    assert c2.step(est, S, dt, None).speed == 0 and c2._ex_phase == "drive"
    # 超音波がずっと測れなければ、前にも後ろにも動かない
    c3, _ = _controller("explore")
    for k in range(int(3 * scan_s / dt)):
        assert c3.step(est, S + int(k * dt * S), dt, None).speed == 0
    # snoo が遠くにいても探索は続ける。近くに見えたら止まってカメラで追うだけ
    _seen(est, S + int(t[0] * S), 10.0, 300.0)
    assert c.step(est, S + int(t[0] * S), dt, 200.0).state == EXPLORE
    _seen(est, S + int(t[0] * S), 10.0, 100.0)
    x = c.step(est, S + int(t[0] * S), dt, 200.0)
    assert x.state == TRACK and x.speed == 0 and c.mode == "explore"


def test_map_recorder_explore_records_while_driving(tmp_path, monkeypatch):
    from snoo_trucar.controller import DRIVE_INDEX

    rec = MapRecorder(tmp_path)
    clock = [0.0]
    monkeypatch.setattr("snoo_trucar.mapping.time.monotonic", lambda: clock[0])
    cmd = [0.0] * 15
    cmd[14], cmd[13], cmd[2] = MODES.index("explore"), DRIVE_INDEX, 15.0
    for _ in range(20):  # 1 s 走る → 5 枚
        rec.offer(1, b"jpeg", cmd, 120.0)
        clock[0] += 0.05
    cmd[13] = -1  # snoo を見ている間は撮らないが、終えない
    rec.offer(1, b"jpeg", cmd, 120.0)
    assert rec.dir is not None
    cmd[14] = MODES.index("idle")
    rec.offer(1, b"jpeg", cmd, None)
    (d,) = tmp_path.iterdir()
    import json

    scan = json.loads((d / "scan.json").read_text())
    assert d.name.startswith("explore_") and scan["mode"] == "explore"
    assert len(scan["frames"]) == 5 and all(f["moving"] and f["speed"] == 15.0 for f in scan["frames"])


def test_motion_watch_notices_when_the_image_stops_moving():
    import numpy as np

    from snoo_trucar.motion import MotionWatch

    rng = np.random.default_rng(0)
    scene = rng.uniform(0, 255, (60, 200)).astype(np.float32)
    w = MotionWatch()
    t = 0.0

    def frame(x):
        a = scene[:, x : x + 80] + rng.normal(0, 3, (60, 80))
        return a - a.mean()

    for _ in range(10):  # 止まっている → ざらつきを測る
        w.feed(t, frame(0), False)
        t += 0.2
    for k in range(15):  # 走って画像が流れる → 詰まっていない
        assert not w.feed(t, frame(k * 4), True)
        t += 0.2
    stuck = [w.feed(t + k * 0.2, frame(60), True) for k in range(15)]  # 走らせているのに動かない
    assert not stuck[0] and stuck[-1]
    assert not w.feed(t + 4.0, frame(60), False)  # 止めたら解く


def test_explore_backs_off_when_stuck():
    est = TargetEstimator(53.5, 41.4, 33.0)
    c, ctl = _controller("explore")
    c._ex_us_at = 0.0
    c._ex_phase, c._ex_t = "drive", 0.0
    assert c.step(est, S, 0.01, 200.0).speed > 0
    c.stuck = True  # 進めていない
    x = c.step(est, S, 0.01, 200.0)
    assert x.speed < 0 and c._ex_phase == "back"


def test_sno2_header_and_clear_cm():
    import json

    data = pack_frame(7, 123, 640, 480, b"jpg", (10.0, -15.0, 5.0, -2.0))
    assert data[:4] == b"SNO2"
    assert unpack_frame(data) == (7, 123, 640, 480, b"jpg")
    r = parse_reply(make_reply(1, 2, 3.0, []))
    assert not r.clear_present  # 欄が無い → 止めない
    msg = json.loads(make_reply(1, 2, 3.0, []))
    msg["clear_cm"] = None
    r = parse_reply(json.dumps(msg).encode())
    assert r.clear_present and r.clear_cm is None
    msg["clear_cm"], msg["clear_conf"] = 42.0, 0.8
    r = parse_reply(json.dumps(msg).encode())
    assert r.clear_cm == 42.0 and r.clear_conf == 0.8


def test_explore_uses_image_clearance():
    est = TargetEstimator(53.5, 41.4, 33.0)
    c, ctl = _controller("explore")
    c._ex_us_at = 0.0
    c._ex_phase = "drive"
    c.set_clear(0, -1.0, S)  # 欄が無い → 超音波だけで進む
    assert c.step(est, S, 0.01, 200.0).speed > 0
    c.set_clear(1, ctl.explore_clear_stop_cm + 50, S)
    assert c.step(est, S, 0.01, 200.0).speed > 0
    c.set_clear(1, ctl.explore_clear_stop_cm - 5, S)  # 画像で前が近い → 向きを変える
    assert c.step(est, S, 0.01, 200.0).speed < 0 and c._ex_phase == "back"
    c._ex_phase = "drive"
    c.set_clear(2, -1.0, S)  # null が 0.3 s 続いたら止まって撮り直す
    assert c.step(est, S + int(0.1 * S), 0.01, 200.0).speed > 0
    assert c.step(est, S + int(0.35 * S), 0.01, 200.0).speed == 0 and c._ex_phase == "scan"


def test_explore_asks_the_server_for_the_next_step():
    from snoo_trucar.controller import EXPLORE_PANS, EXPLORE_TILTS, PLAN_DONE, PLAN_FALLBACK, PLAN_MOVE, STOP_REASONS

    est = TargetEstimator(53.5, 41.4, 33.0)
    c, ctl = _controller("explore")
    dt = 0.01
    t = [0.0]

    def run(seconds, us=200.0):
        x = None
        for _ in range(int(seconds / dt)):
            t[0] += dt
            x = c.step(est, S + int(t[0] * S), dt, us)
        return x

    from snoo_trucar.controller import scan_schedule

    scan_s = scan_schedule(30.0, 0.0)[-1]  # 首が 30° 振れていても足りるように（先頭の待ちが長い分）
    x = run(scan_s + 0.05)
    assert c.station_wait == 0 and x.speed == 0  # 撮り終えて、サーバーの返事を待つ
    assert run(5.0).speed == 0
    c.plan = (0, PLAN_MOVE, 12.0, 1, 10.0, 20.0, 0, 0, 0, 0)  # 右に 10° で 20 cm
    x = run(0.1)
    assert x.steer == 10.0 and x.speed == 12.0 and c.station_wait == -1
    run(20.0 / (12.0 * c.car.cm_s_per_speed))
    assert c._ex_phase == "scan" and STOP_REASONS[c.last_step[1]] == "done"
    assert abs(c.last_step[0] - 20.0 / (12.0 * c.car.cm_s_per_speed)) < 0.1
    # 次の station。超音波で前が近いと途中で止めて、理由を残す
    run(scan_s + 0.05)
    assert c.station_wait == 1
    c.plan = (1, PLAN_MOVE, 12.0, 1, 0.0, 40.0, 0, 0, 0, 0)
    run(0.5)
    run(0.05, us=ctl.explore_step_stop_cm - 5)
    assert STOP_REASONS[c.last_step[1]] == "ultrasonic"
    # サーバーに届かない → Pi だけで決める。終わり → 待機
    run(scan_s + 0.05)
    c.plan = (2, PLAN_FALLBACK) + (0.0,) * 8
    assert run(0.1).speed > 0 and c._ex_phase == "drive"
    c2, _ = _controller("explore")
    c2.station_wait, c2._ex_phase, c2.plan = 0, "plan", (0, PLAN_DONE) + (0.0,) * 8
    c2.step(est, S, dt, 200.0)
    assert c2.mode == "idle"


def test_station_client_posts_and_writes_the_plan(tmp_path):
    import json
    import threading
    from http.server import BaseHTTPRequestHandler, HTTPServer
    from types import SimpleNamespace

    from snoo_trucar.control_proc import PLAN_N
    from snoo_trucar.controller import PLAN_FALLBACK, PLAN_MOVE
    from snoo_trucar.station import StationClient

    got = {}

    class H(BaseHTTPRequestHandler):
        def do_POST(self):
            body = self.rfile.read(int(self.headers["Content-Length"]))
            got["body"] = body
            out = json.dumps({"action": "move", "steer_deg": -8, "speed": 12, "distance_cm": 30,
                              "pose": {"x": 0, "y": 0, "yaw_deg": 0}}).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(out)))
            self.end_headers()
            self.wfile.write(out)

        def log_message(self, *a):
            pass

    srv = HTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    cfg = Config()
    cfg.server.map_url = f"http://127.0.0.1:{srv.server_port}/v1/snoo/map/station"
    d = tmp_path / "explore_x"
    d.mkdir()
    (d / "00_a.jpg").write_bytes(b"jpg0")
    (d / "d00001_b.jpg").write_bytes(b"jpg1")
    rec = SimpleNamespace(dir=d, entries=[{"file": "00_a.jpg", "station": 0},
                                          {"file": "d00001_b.jpg", "station": 1}])
    plan = SeqBuffer(PLAN_N)
    runner = SimpleNamespace(cfg=cfg, recorder=rec, battery_v=7.9, last_us=120.0, last_cmd=[0.0] * 22)
    c = StationClient(runner, plan)
    try:
        c._session = "explore_x"
        c._post(0, runner.last_cmd, rec)
        assert b'"station_id": 0' in got["body"] and b"jpg0" in got["body"] and b"jpg1" not in got["body"]
        assert tuple(plan.read()[1]) == (0, PLAN_MOVE, 12, 1, -8, 30, 0, 0, 0, 0)
        cfg.server.map_url = "http://127.0.0.1:1/nothing"  # 届かない → Pi だけで決める
        c._post(1, runner.last_cmd, rec)
        assert plan.read()[1][:2].tolist() == [1, PLAN_FALLBACK]
    finally:
        srv.shutdown()
        plan.close()


def test_explore_server_segments_and_reverse():
    from snoo_trucar.controller import PLAN_MOVE, STOP_REASONS

    est = TargetEstimator(53.5, 41.4, 33.0)
    c, ctl = _controller("explore")
    dt = 0.01
    v = 12.0 * c.car.cm_s_per_speed

    def go(plan, seconds, us=200.0):
        c._ex_phase, c.station_wait, c.plan, c._ex_t = "plan", 0, plan, 0.0
        out = []
        for k in range(int(seconds / dt)):
            x = c.step(est, S + int(k * dt * S), dt, us)
            out.append((x.steer, x.speed, x.pan))
        return out

    # 曲がってからまっすぐ: 右 25° で 15 cm → まっすぐ 25 cm。曲がる間は首も右へ
    out = go((0, PLAN_MOVE, 12.0, 2, 25.0, 15.0, 0.0, 25.0, 0, 0), 40 / v + 0.3)
    turn = [o for o in out if o[1] > 0 and o[0] == 25.0]
    straight = [o for o in out if o[1] > 0 and o[0] == 0.0]
    assert abs(len(turn) * dt - 15 / v) < 0.05 and abs(len(straight) * dt - 25 / v) < 0.05
    assert turn[-1][2] > 25.0  # 首は曲がる方へ（ハンドルの 1.5 倍、30° まで）
    assert STOP_REASONS[c.last_step[1]] == "done"
    # 下がる: 前が近くても止めない（前の超音波は見ない）。距離は 30 cm まで
    out = go((0, PLAN_MOVE, -12.0, 1, 20.0, 80.0, 0, 0, 0, 0), 80 / v, us=10.0)
    back = [o for o in out if o[1] < 0]
    assert all(o[0] == 20.0 for o in back)
    assert abs(len(back) * dt - ctl.explore_reverse_max_cm / v) < 0.05


def test_explore_passes_low_obstacles():
    from snoo_trucar.controller import LOW_OBSTACLE_SPEED, PLAN_MOVE, STOP_REASONS

    est = TargetEstimator(53.5, 41.4, 33.0)
    c, ctl = _controller("explore")
    dt = 0.01
    c._ex_phase, c.station_wait, c.plan = "plan", 0, (0, PLAN_MOVE, 12.0, 1, 0.0, 40.0, 0, 0, 0, 0)
    c.step(est, S, dt, 200.0)
    # 画像では 20 cm 先に何かある。超音波は 70 cm → 低い物。遅くして越える
    c.set_clear(1, 20.0, S)
    x = c.step(est, S, dt, 70.0)
    assert x.speed == LOW_OBSTACLE_SPEED and c.low_passed == 1
    # 超音波も近い → 止まる
    c2, _ = _controller("explore")
    c2._ex_phase, c2.station_wait, c2.plan = "plan", 0, (0, PLAN_MOVE, 12.0, 1, 0.0, 40.0, 0, 0, 0, 0)
    c2.step(est, S, dt, 200.0)
    c2.set_clear(1, 20.0, S)
    assert c2.step(est, S, dt, 40.0).speed == 0 and STOP_REASONS[c2.last_step[1]] == "clear"


def test_link_sends_fewer_frames_while_idle():
    import threading
    from types import SimpleNamespace

    from snoo_trucar.link import camera_loop

    cfg = Config()
    sent, mode = [], ["idle"]
    stop = threading.Event()
    t = [0]

    class Cam:
        def grab(self):
            t[0] += 16_666_667
            if t[0] > 4_000_000_000:
                stop.set()
            return t[0], b"j"

    link = SimpleNamespace(trace=None, can_send=lambda now: True, send=lambda tn, j: sent.append(tn),
                           min_interval_ns=lambda: int(1e9 / cfg.server.idle_fps) if mode[0] == "idle" else 0)
    import snoo_trucar.link as L

    real = L.time.monotonic_ns
    L.time.monotonic_ns = lambda: t[0]
    try:
        camera_loop(Cam(), link, stop)
        idle_n = len(sent)
        sent.clear(); t[0] = 0; stop.clear(); mode[0] = "watch"
        camera_loop(Cam(), link, stop)
    finally:
        L.time.monotonic_ns = real
    assert 18 <= idle_n <= 22 and len(sent) >= 230  # 4 s: 待機は 5 fps、行動中は 60 fps


def test_real_link_idle_interval():
    from snoo_trucar import control_proc
    from snoo_trucar.link import Link

    cfg = Config()
    cfg.server.host = cfg.server.fallback_host = "127.0.0.1"
    meas = SeqBuffer(control_proc.MEAS_N)
    link = Link(cfg, meas)
    try:
        link.mode_fn = lambda: "idle"
        assert link.min_interval_ns() == int(1e9 / cfg.server.idle_fps)
        link.mode_fn = lambda: "explore"
        assert link.min_interval_ns() == 0
        assert link._frames == {}  # 初期化が最後まで済んでいる
    finally:
        link.close()
        meas.close()


def test_link_remembers_cat_boxes_by_capture_time():
    from snoo_trucar import control_proc
    from snoo_trucar.link import RECENT_DETS_N, Link

    cfg = Config()
    cfg.server.host = cfg.server.fallback_host = "127.0.0.1"
    meas = SeqBuffer(control_proc.MEAS_N)
    link = Link(cfg, meas)
    try:
        det = {"box": [0.1, 0.2, 0.3, 0.6], "cat": 0.9, "snoo": 0.9}
        link._on_packet(make_reply(1, 111, 5.0, [det]))
        (d,) = link.recent_dets[111]
        assert d["box"] == [0.1, 0.2, 0.3, 0.6] and d["cat"] == 0.9
        for t in range(RECENT_DETS_N + 5):
            link._on_packet(make_reply(2, 1000 + t, 5.0, []))
        assert 111 not in link.recent_dets and len(link.recent_dets) == RECENT_DETS_N
    finally:
        link.close()
        meas.close()


def test_low_battery_shuts_down_after_hold(monkeypatch):
    from types import SimpleNamespace

    from snoo_trucar import runner as R

    calls = []
    monkeypatch.setattr(R.subprocess, "run", lambda args, **k: calls.append(args) or SimpleNamespace(returncode=0, stderr=""))
    monkeypatch.setattr(R.os, "sync", lambda: None)
    r = R.Runner(Config(), sim=True, quiet=True)
    car = SimpleNamespace(stop=lambda: calls.append("stop"))
    hold = r.cfg.car.shutdown_hold_s
    assert not r._low_battery_shutdown(0.0, 0.0, car)  # 入っていない（Pi は別の電源）
    assert not r._low_battery_shutdown(7.5, 1.0, car)
    assert not r._low_battery_shutdown(6.2, 2.0, car)
    assert not r._low_battery_shutdown(6.5, 3.0, car)  # 戻った → 数え直し
    assert not r._low_battery_shutdown(6.2, 4.0, car)
    assert not r._low_battery_shutdown(6.2, 4.0 + hold - 0.1, car)
    assert r._low_battery_shutdown(6.2, 4.0 + hold, car)
    assert calls == ["stop", ["sudo", "-n", "/usr/sbin/shutdown", "-h", "now"]] and r.stop.is_set()
