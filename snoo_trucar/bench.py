"""制御のベンチ。推定・制御則と、ハードの遅れ（書き込み・サーボ・検出の遅れと揺れ・量子化）を
1 ms 刻みで実時間を待たずに回し、制御の良し悪しを数字で出す（snoo_trucar#20）。

  python -m snoo_trucar.bench [--scenario dash|turn|all] [--seeds 3]

sim.py（実時間で制御プロセスを動かす結合の試験）と違い、時刻を自分で進めるので、
同じ種なら毎回同じ結果になり、数十秒の筋書きが 1〜2 秒で終わる。

ハードのモデル（Plant）のうち、サーボの値は #2 で測るまでの仮の値。
検出の遅れは 2026-10-29 に trace で測った分布（撮影 → 返事 p50 39〜40 ms、p95 約 44 ms）に合わせた。
"""

from __future__ import annotations

import argparse
import math
import random
import statistics
from collections import deque
from dataclasses import dataclass, field

from .config import Config
from .controller import SEARCH, TRACK, Controller, ego_yaw_rate
from .estimator import Detection, TargetEstimator

MS = 1_000_000  # ns
SPEED_CM_S = 1.0  # picarx の速さ 1 あたり（cm/s。sim.py と同じ仮の値。#8 で測る）
WHEELBASE_CM = 14.0  # 前後の車軸の間（PiCar-X の URDF、cau-se/ARCHES-PiCar-X）


@dataclass
class Plant:
    """ハードの遅れのモデル。"""

    write_hz: float = 200.0  # 書き込みのループ（config の actuator_hz）
    pwm_hz: float = 50.0  # サーボが新しい角度を受け取る周期
    # サーボ（仮の値。#2 で測る）: むだ時間・一次遅れの時定数・最大速度・不感帯
    servo_dead_s: float = 0.010
    servo_tau_s: float = 0.030
    servo_rate_dps: float = 400.0
    servo_deadband_deg: float = 0.5
    # モーター（仮の値）: 速さの一次遅れ
    motor_tau_s: float = 0.15
    # 撮影 → 返事の遅れ（s）。正規分布（中央 0.0395・標準偏差 0.0025）、まれに大きく遅れる・届かない
    latency_mean_s: float = 0.0395
    latency_sd_s: float = 0.0025
    latency_spike_p: float = 0.01
    latency_spike_s: float = 0.04
    drop_p: float = 0.005
    # 検出の箱の中心のぶれ（画面比）と、箱の高さのぶれ（比。猫の姿勢で変わるので大きめ）
    box_noise: float = 0.005
    height_noise: float = 0.08


class Servo:
    """書いた角度を PWM の周期で受け取り、むだ時間の後、速さの上限つきの一次遅れで動く。"""

    def __init__(self, plant: Plant, angle: float = 0.0):
        self.p = plant
        self.angle = angle
        self.latched = angle
        self.queue: deque[tuple[float, float]] = deque()  # (効き始める時刻, 角度)
        self.goal = angle

    def latch(self, t: float, written: float) -> None:
        """PWM の周期の頭で、最後に書かれた角度を受け取る。"""
        if written != self.latched:
            self.latched = written
            self.queue.append((t + self.p.servo_dead_s, written))

    def step(self, t: float, dt: float) -> None:
        while self.queue and self.queue[0][0] <= t:
            self.goal = self.queue.popleft()[1]
        err = self.goal - self.angle
        if abs(err) < self.p.servo_deadband_deg:
            return
        v = err / self.p.servo_tau_s
        v = max(-self.p.servo_rate_dps, min(self.p.servo_rate_dps, v))
        self.angle += v * dt


@dataclass
class Result:
    name: str
    point_err: list[float] = field(default_factory=list)  # 画面の中央と snoo の角度のずれ（度）
    lost: int = 0  # 追っていて SEARCH に戻った回数
    visible: int = 0
    frames: int = 0
    min_dist: float = math.inf

    def summary(self) -> dict[str, float]:
        e = sorted(self.point_err) or [math.nan]
        rms = math.sqrt(sum(x * x for x in e) / len(e))
        return {
            "向きのずれ RMS": rms,
            "向きのずれ p95": e[min(len(e) - 1, int(len(e) * 0.95))],
            "見えていた割合": self.visible / max(1, self.frames),
            "見失った回数": self.lost,
            "いちばん近づいた距離": self.min_dist,
        }


def cat_dash(t: float) -> tuple[float, float]:
    """車の 120 cm 前に座っていて、3 s から右へ 80 cm/s で 1 s 走り、6 s から左へ戻る。"""
    y = 0.0
    if 3.0 <= t < 4.0:
        y = -80.0 * (t - 3.0)
    elif 4.0 <= t < 6.0:
        y = -80.0
    elif 6.0 <= t < 7.0:
        y = -80.0 + 80.0 * (t - 6.0)
    return 120.0, y


def cat_circle(t: float) -> tuple[float, float]:
    """車の 150 cm 前を中心に、半径 80 cm の円を 30 cm/s で歩く。"""
    w = 30.0 / 80.0
    return 150.0 + 80.0 * math.cos(w * t), 80.0 * math.sin(w * t)


SCENARIOS = {
    # 名前: (猫の動き, 行動, 秒)
    "dash": (cat_dash, "watch", 9.0),
    "turn": (cat_circle, "keep", 40.0),
}


def run(
    scenario: str, seed: int = 0, cfg: Config | None = None, plant: Plant | None = None
) -> Result:
    cfg = cfg or Config()
    plant = plant or Plant()
    cat_at, mode, seconds = SCENARIOS[scenario]
    cfg.control.mode = mode
    rng = random.Random(seed)
    cam = cfg.camera
    fx = 0.5 / math.tan(math.radians(cam.hfov_deg) / 2)
    est = TargetEstimator(cam.hfov_deg, cam.vfov_deg, cfg.control.distance_k)
    controller = Controller(cfg.control, cfg.car)
    pan, steer = Servo(plant), Servo(plant)
    speed = 0.0
    x = y = yaw = 0.0  # 車（yaw は左回りが +）
    written = [0.0, 0.0, 0.0]  # 書いた ハンドル・速さ・パン（hal.Car.apply と同じ量子化）
    inbox: list[tuple[int, Detection]] = []
    res = Result(scenario)
    dt = 0.001
    frame_dt = 1.0 / cam.fps
    write_dt, pwm_dt = 1.0 / plant.write_hz, 1.0 / plant.pwm_hz
    next_frame = next_write = next_pwm = 0.0
    tracking = False
    for k in range(int(seconds / dt)):
        t = k * dt
        now = int(t * 1e9)
        # 車と猫を動かす
        v = speed * SPEED_CM_S
        yaw -= v / WHEELBASE_CM * math.tan(math.radians(steer.angle)) * dt  # 右に切ると右へ
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        cx_, cy_ = cat_at(t)
        dx, dy = cx_ - x, cy_ - y
        dist = math.hypot(dx, dy)
        res.min_dist = min(res.min_dist, dist)
        bearing = -((math.degrees(math.atan2(dy, dx) - yaw) + 180) % 360 - 180)  # 右が +
        # カメラ（サーボの実際の向きで撮る）
        if t >= next_frame:
            next_frame += frame_dt
            rel = bearing - pan.angle
            res.frames += 1
            if abs(rel) < cam.hfov_deg / 2 - 2 and dist > 15:
                res.visible += 1
                if rng.random() >= plant.drop_p:
                    lat = max(0.03, rng.gauss(plant.latency_mean_s, plant.latency_sd_s))
                    if rng.random() < plant.latency_spike_p:
                        lat += plant.latency_spike_s
                    u = 0.5 + fx * math.tan(math.radians(rel)) + rng.gauss(0, plant.box_noise)
                    h = cfg.control.distance_k / dist
                    h *= max(0.3, rng.gauss(1.0, plant.height_noise))
                    inbox.append((now + int(lat * 1e9), Detection(now, u, 0.5, h * 0.8, h, 0.9)))
        # 制御ループ（1 kHz）
        est.predict_to(now)
        for item in [d for d in inbox if d[0] <= now]:
            inbox.remove(item)
            est.measure(item[1], now)
        cmd = controller.step(est, now, dt, None)
        if cfg.control.ego_ff:  # control_proc と同じ
            est.set_ego(ego_yaw_rate(cmd.steer, cmd.speed, cfg.car))
        est.record_pose(now, cmd.pan, cmd.tilt)
        # 書き込み（量子化は hal.Car.apply と同じ）と PWM
        if t >= next_write:
            next_write += write_dt
            written[0], written[1] = round(cmd.steer), round(cmd.speed)
            if abs(written[2] - cmd.pan) >= 0.5:
                written[2] = cmd.pan
        if t >= next_pwm:
            next_pwm += pwm_dt
            steer.latch(t, written[0])
            pan.latch(t, written[2])
        pan.step(t, dt)
        steer.step(t, dt)
        speed += (written[1] - speed) * min(1.0, dt / plant.motor_tau_s)
        # 記録
        if cmd.state == TRACK:
            tracking = True
            res.point_err.append(abs(bearing - pan.angle))
        elif cmd.state == SEARCH and tracking:
            tracking = False
            res.lost += 1
    return res


def compare(seeds: int = 3, scenarios=None, cfg_fn=None, plant: Plant | None = None) -> dict:
    """筋書きごとに種を変えて回し、指標の平均を返す。cfg_fn(cfg) で設定を変えられる。"""
    out = {}
    for name in scenarios or SCENARIOS:
        rows = []
        for s in range(seeds):
            cfg = Config()
            if cfg_fn is not None:
                cfg_fn(cfg)
            rows.append(run(name, s, cfg, plant).summary())
        out[name] = {k: statistics.fmean(r[k] for r in rows) for k in rows[0]}
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scenario", default="all", choices=["all", *SCENARIOS])
    parser.add_argument("--seeds", type=int, default=3)
    args = parser.parse_args()
    names = None if args.scenario == "all" else [args.scenario]
    for name, m in compare(args.seeds, names).items():
        print(name + ": " + "  ".join(f"{k} {v:.2f}" for k, v in m.items()))


if __name__ == "__main__":
    main()
