"""制御ループ（既定 1 kHz、config の rate_hz）。別プロセスで、CPU を 1 つに固定して回す（GIL の影響を受けないように）。

入力（共有メモリ）: 検出（meas）、超音波（us）、行動（mode）
出力（共有メモリ）: コマンドと推定値（cmd）

ループの中は推定（predict / update）と制御則だけ。I2C への書き込みはメインプロセスが 50 Hz で行う。
周期を守るため、残りが長ければ sleep し、短ければ回って待つ。
"""

from __future__ import annotations

import os
import time

from .config import Config
from .controller import CLEAR_STALE, MAX_SEGMENTS, MODES, PLAN_SEG_N, Controller, ego_yaw_rate
from .estimator import Detection, TargetEstimator
from .shm import SeqBuffer

# meas: t_capture_ns, cx, cy, w, h, score, stress（判定なしは -1）
MEAS_N = 7
# us: t_ns, cm（測れないとき -1。snoo の距離の推定にだけ使う）, stuck（走らせているのに画像が動かない。
#     motion.py）, drive_ok（電池が足りていて、メインプロセスが速さを書いている。進んだ距離を数えるため）
US_N = 4
# mode: controller.MODES の番号, 続きから始める station（-1 は始めから）,
#       前のプロセスで探った秒（resume.py）
MODE_N = 3
# cmd: t_ns, steer, speed, pan, tilt, state, valid, bearing, dist, since_seen,
#      loop_hz, jitter_p99_us, overruns, scan_index, mode,
#      meas_t_capture（最後に使った検出の撮った時刻）, meas_used_ns（それで推定を更新した時刻）,
#      guard（guard.OK / BACKOFF / HOLD / REST）,
#      station_wait（サーバーの返事を待っている station、-1 で待っていない）, last_step_s, last_step_reason,
#      ex_station（explore で今の station。走っている間は次の station）, low_passed（直前の 1 歩で低い物を越えた回数）,
#      step_station（直前の 1 歩はどの station の返事か。-1 は走っていない）, step_speed（前へ使った速さ）,
#      step_driven_cm（実際に書いた速さで数えた距離）, 区切りごとの距離 × controller.MAX_SEGMENTS
CMD_N = 26 + MAX_SEGMENTS
# clear: t_capture_ns, cm, state（controller.CLEAR_*。0: 返事に欄が無い / 1: 値がある / 2: null /
#        3: clear_skip。protocol.Reply）
CLEAR_N = 3
# plan: station, action（controller.PLAN_*）, 前の速さ, 区切りの数, 後ろの速さ,
#       (steer_deg, 符号付きの distance_cm（負は後ろへ）, min_clear_cm（無ければ -1）) × controller.MAX_SEGMENTS
PLAN_N = 5 + PLAN_SEG_N * MAX_SEGMENTS
CLEAR_MAX_AGE_NS = 500_000_000  # これより古い clear_cm は使わない（分からない = controller.CLEAR_STALE）
US_MAX_AGE_NS = 300_000_000  # これより古い超音波は使わない（snoo の距離の推定に）
SPIN_NS = 150_000  # これより残りが短ければ sleep せずに回って待つ


def us_is_stale(now_ns: int, us_t_ns: int) -> bool:
    """超音波の最後の 1 回（-1 も含む）が来ていない・古い"""
    return us_t_ns == 0 or now_ns - us_t_ns > US_MAX_AGE_NS


def run(cfg: Config, meas_name: str, us_name: str, mode_name: str, cmd_name: str,
        stop_event, clear_name: str | None = None, plan_name: str | None = None) -> None:  # fmt: skip
    ctl = cfg.control
    if ctl.cpu >= 0:
        try:
            os.sched_setaffinity(0, {ctl.cpu})
        except OSError:
            pass
    meas, us, out = SeqBuffer(MEAS_N, meas_name), SeqBuffer(US_N, us_name), SeqBuffer(CMD_N, cmd_name)
    mode_buf = SeqBuffer(MODE_N, mode_name)
    clear_buf = SeqBuffer(CLEAR_N, clear_name) if clear_name else None
    last_clear = 0
    plan_buf = SeqBuffer(PLAN_N, plan_name) if plan_name else None
    last_plan = 0
    est = TargetEstimator(cfg.camera.hfov_deg, cfg.camera.vfov_deg, ctl.distance_k)
    controller = Controller(ctl, cfg.car)
    if clear_buf is not None:
        # 返事がまだ来ていないのは、途切れているのと同じ（サーバーの 1 歩では 0.3 s で止まる）
        controller.set_clear(CLEAR_STALE, -1.0, time.monotonic_ns())
    period = int(1e9 / ctl.rate_hz)
    last_meas = last_us = last_mode = 0
    us_cm: float | None = None
    us_t = 0
    used_capture = used_ns = 0
    out_vals = [0.0] * CMD_N
    # 統計（1 秒ごと）
    n = overruns = 0
    lates: list[int] = []
    loop_hz = p99 = 0.0
    stat_t = time.monotonic_ns()

    now = time.monotonic_ns()
    deadline = now + period
    prev = now
    try:
        while not stop_event.is_set():
            now = time.monotonic_ns()
            dt = (now - prev) * 1e-9
            prev = now

            r = us.read()
            if r is not None and r[0] != last_us:
                last_us = r[0]
                us_t, cm = int(r[1][0]), r[1][1]
                us_cm = cm if cm > 0 else None
                controller.stuck = bool(r[1][2])
                controller.drive_ok = bool(r[1][3])
            if us_cm is not None and us_is_stale(now, us_t):
                us_cm = None

            if clear_buf is not None:
                r = clear_buf.read()
                if r is not None and r[0] != last_clear:
                    last_clear = r[0]
                    controller.set_clear(int(r[1][2]), r[1][1], now, int(r[1][0]))
                controller.age_clear(now, CLEAR_MAX_AGE_NS)

            if plan_buf is not None:
                r = plan_buf.read()
                if r is not None and r[0] != last_plan:
                    last_plan = r[0]
                    controller.set_plan(r[1])

            r = mode_buf.read()
            if r is not None and r[0] != last_mode:
                last_mode = r[0]
                controller.set_mode(MODES[int(r[1][0])])
                if MODES[int(r[1][0])] == "explore" and r[1][1] >= 0:
                    controller.resume_explore(int(r[1][1]), float(r[1][2]))

            est.predict_to(now)
            r = meas.read()
            if r is not None and r[0] != last_meas:
                last_meas = r[0]
                v = r[1]
                est.measure(Detection(int(v[0]), v[1], v[2], v[3], v[4], v[5]), now, us_cm)
                used_capture, used_ns = int(v[0]), time.monotonic_ns()
                controller.observe_stress(None if v[6] < 0 else v[6])

            cmd = controller.step(est, now, dt, us_cm)
            if ctl.ego_ff:
                est.set_ego(ego_yaw_rate(cmd.steer, cmd.speed, cfg.car))
            est.record_pose(now, cmd.pan, cmd.tilt)

            since = est.since_seen_s(now)
            out_vals[:] = (
                now, cmd.steer, cmd.speed, cmd.pan, cmd.tilt, cmd.state, float(est.valid),
                est.bearing.x[0], est.dist.x[0], min(since, 1e6), loop_hz, p99, overruns,
                cmd.scan_index, MODES.index(controller.mode), used_capture, used_ns, cmd.guard,
                controller.station_wait, *controller.last_step, controller._ex_stop, controller.low_passed,
                controller.step_station, controller._ex_step[0], sum(controller.step_driven),
                *controller.step_driven,
            )  # fmt: skip
            out.write(out_vals)

            n += 1
            if now - stat_t >= 1_000_000_000:
                loop_hz = n * 1e9 / (now - stat_t)
                lates.sort()
                p99 = lates[int(len(lates) * 0.99)] / 1e3 if lates else 0.0
                n, lates, stat_t = 0, [], now

            # 次の周期まで待つ
            remain = deadline - time.monotonic_ns()
            if remain > SPIN_NS:
                time.sleep((remain - SPIN_NS) * 1e-9)
            while time.monotonic_ns() < deadline:
                pass
            late = time.monotonic_ns() - deadline
            lates.append(late)
            deadline += period
            if late > period:
                overruns += 1
                deadline = time.monotonic_ns() + period  # 遅れを取り戻そうとして連続で回らない
    finally:
        meas.close()
        us.close()
        mode_buf.close()
        if clear_buf is not None:
            clear_buf.close()
        if plan_buf is not None:
            plan_buf.close()
        out.close()
