import math

import pytest

from snoo_trucar.buttons import ClickCounter, event_to_mode
from snoo_trucar.config import CarConfig, ControlConfig, Config
from snoo_trucar.controller import IDLE, MAP, MAP_HOLD_S, MAP_PANS, MAP_TILTS, SEARCH, TRACK, Controller
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
    for index in (0, 1):
        cmd[13], cmd[3] = index, -35.0 + 10 * index
        for _ in range(10):
            rec.offer(1, b"jpeg", cmd, 50.0)
            clock[0] += 0.1
    cmd[13] = -1
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
