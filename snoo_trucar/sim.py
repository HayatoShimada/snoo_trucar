"""シミュレーション。カメラとサーバーの代わりに、部屋の中を歩く snoo を見たことにして検出を書く。

車体は FakeCar のコマンドで動かす（自転車モデル）。制御ループ・推定・行動をハードなしで試すためのもの。
"""

from __future__ import annotations

import math
import random
import threading
import time
from collections import deque

from .config import Config
from .hal import FakeCar
from .shm import SeqBuffer

SPEED_CM_S = 1.0  # picarx の速さ 1 あたり（cm/s）。速さ 20 で 20 cm/s くらい
WHEELBASE_CM = 9.5


class SimWorld:
    def __init__(self, cfg: Config, car: FakeCar, meas: SeqBuffer, seed: int = 0,
                 fps: float | None = None, latency_s: float = 0.06, cat_speed_cm_s: float = 15.0):  # fmt: skip
        self.cfg = cfg
        self.car = car
        self.meas = meas
        self.rng = random.Random(seed)
        self.fps = fps or cfg.camera.fps
        self.latency_s = latency_s
        self.cat_speed = cat_speed_cm_s
        # 車は原点で +x を向く。snoo は左後ろのほうにいる（最初は見えない）
        self.x = self.y = self.yaw = 0.0
        self.cx, self.cy = 120.0, 150.0
        self.cat_heading = 0.0
        self.t = 0.0
        self.visible_frames = 0
        self._queue: deque[tuple[float, tuple]] = deque()

    def truth(self) -> tuple[float, float]:
        """車から見た snoo の方位（度、右が +）と距離（cm）"""
        dx, dy = self.cx - self.x, self.cy - self.y
        ang = math.degrees(math.atan2(dy, dx)) - math.degrees(self.yaw)
        bearing = -((ang + 180) % 360 - 180)  # 左が +y なので、右を + にする
        return bearing, math.hypot(dx, dy)

    def step(self, dt: float, capture: bool = True) -> None:
        car, cam = self.car, self.cfg.camera
        v = car.speed * SPEED_CM_S
        self.yaw -= v / WHEELBASE_CM * math.tan(math.radians(car.steer)) * dt  # 右に切ると右に曲がる
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        # snoo はゆっくり向きを変えながら歩く。壁（±300 cm）で向きを変える
        self.cat_heading += self.rng.gauss(0, 1.0) * dt
        self.cx += self.cat_speed * math.cos(self.cat_heading) * dt
        self.cy += self.cat_speed * math.sin(self.cat_heading) * dt
        if abs(self.cx) > 300 or abs(self.cy) > 300:
            self.cat_heading += math.pi
            self.cx = max(-300, min(300, self.cx))
            self.cy = max(-300, min(300, self.cy))
        self.t += dt

        bearing, dist = self.truth()
        rel = bearing - car.pan
        car.ultrasonic_cm = dist if abs(bearing) < 15 and dist < 300 else None
        h = self.cfg.control.distance_k / max(dist, 1.0)
        fx = 0.5 / math.tan(math.radians(cam.hfov_deg) / 2)
        if capture and abs(rel) < cam.hfov_deg / 2 - 2 and dist > 15 and h < 1.0:
            u = 0.5 + fx * math.tan(math.radians(rel)) + self.rng.gauss(0, 0.005)
            fy = 0.5 / math.tan(math.radians(cam.vfov_deg) / 2)
            elev = -math.degrees(math.atan(5.0 / dist))  # snoo の中心はカメラより 5 cm 低い
            vv = 0.5 + fy * math.tan(math.radians(car.tilt - elev))
            box = (u, vv, h * 0.8, h * (1 + self.rng.gauss(0, 0.03)), 0.9, -1.0)
            self.visible_frames += 1
            self._queue.append((self.t + self.latency_s, (time.monotonic_ns(), *box)))

    def deliver(self) -> None:
        while self._queue and self._queue[0][0] <= self.t:
            self.meas.write(self._queue.popleft()[1])

    def run(self, stop: threading.Event) -> None:
        frame = 1.0 / self.fps
        sub = 5  # 1 フレームの間に車と猫を 5 回動かす
        nxt = time.monotonic()
        while not stop.is_set():
            for i in range(sub):
                self.step(frame / sub, capture=(i == 0))
                self.deliver()
            nxt += frame
            time.sleep(max(0.0, nxt - time.monotonic()))
