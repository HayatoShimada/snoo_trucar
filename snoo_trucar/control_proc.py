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
from .controller import MODES, Controller, ego_yaw_rate
from .estimator import Detection, TargetEstimator
from .shm import SeqBuffer

# meas: t_capture_ns, cx, cy, w, h, score, stress（判定なしは -1）
MEAS_N = 7
# us: t_ns, cm（測れないとき -1）, stuck（走らせているのに画像が動かない。motion.py）
US_N = 3
# mode: controller.MODES の番号
MODE_N = 1
# cmd: t_ns, steer, speed, pan, tilt, state, valid, bearing, dist, since_seen,
#      loop_hz, jitter_p99_us, overruns, scan_index, mode,
#      meas_t_capture（最後に使った検出の撮った時刻）, meas_used_ns（それで推定を更新した時刻）,
#      guard（guard.OK / BACKOFF / HOLD / REST）
CMD_N = 18
US_MAX_AGE_NS = 300_000_000
SPIN_NS = 150_000  # これより残りが短ければ sleep せずに回って待つ


def run(cfg: Config, meas_name: str, us_name: str, mode_name: str, cmd_name: str,
        stop_event) -> None:  # fmt: skip
    ctl = cfg.control
    if ctl.cpu >= 0:
        try:
            os.sched_setaffinity(0, {ctl.cpu})
        except OSError:
            pass
    meas, us, out = SeqBuffer(MEAS_N, meas_name), SeqBuffer(US_N, us_name), SeqBuffer(CMD_N, cmd_name)
    mode_buf = SeqBuffer(MODE_N, mode_name)
    est = TargetEstimator(cfg.camera.hfov_deg, cfg.camera.vfov_deg, ctl.distance_k)
    controller = Controller(ctl, cfg.car)
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
            if us_cm is not None and now - us_t > US_MAX_AGE_NS:
                us_cm = None

            r = mode_buf.read()
            if r is not None and r[0] != last_mode:
                last_mode = r[0]
                controller.set_mode(MODES[int(r[1][0])])

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
        out.close()
