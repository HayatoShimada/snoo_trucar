"""snoo の位置の推定。車体から見た方位（度）と距離（cm）を、等速モデルのカルマンフィルタで追う。

画像の検出は 20〜30 Hz で、撮ってから届くまで数十 ms 遅れる。制御ループ（5 kHz）では predict だけを回して
間を埋め、検出が届いたら撮った時刻まで戻した状態で update し、今の時刻まで進め直す。
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np


@dataclass
class Detection:
    t_ns: int  # 撮った時刻（time.monotonic_ns）
    cx: float  # 箱の中心（画面比 0..1）
    cy: float
    w: float
    h: float
    score: float


class Axis:
    """1 次元の等速モデル [位置, 速度]"""

    def __init__(self, q: float, r: float, p0: float):
        self.x = np.zeros(2)
        self.P = np.diag([p0, p0])
        self.q = q  # 加速度の分散
        self.r = r  # 観測の分散

    def reset(self, z: float, p0: float) -> None:
        self.x[:] = (z, 0.0)
        self.P = np.diag([self.r, p0])

    def predict(self, dt: float) -> None:
        if dt <= 0:
            return
        x, P = self.x, self.P
        x[0] += x[1] * dt
        dt2 = dt * dt
        p00 = P[0, 0] + dt * (P[0, 1] + P[1, 0]) + dt2 * P[1, 1]
        p01 = P[0, 1] + dt * P[1, 1]
        P[0, 0] = p00 + self.q * dt2 * dt2 / 4
        P[0, 1] = P[1, 0] = p01 + self.q * dt2 * dt / 2
        P[1, 1] += self.q * dt2

    def update(self, z: float, r: float | None = None) -> None:
        r = self.r if r is None else r
        P = self.P
        s = P[0, 0] + r
        k0, k1 = P[0, 0] / s, P[1, 0] / s
        y = z - self.x[0]
        self.x[0] += k0 * y
        self.x[1] += k1 * y
        p00, p01, p11 = P[0, 0], P[0, 1], P[1, 1]
        P[0, 0] = (1 - k0) * p00
        P[0, 1] = P[1, 0] = (1 - k0) * p01
        P[1, 1] = p11 - k1 * p01


class TargetEstimator:
    def __init__(self, hfov_deg: float, vfov_deg: float, distance_k: float):
        self.fx = 0.5 / math.tan(math.radians(hfov_deg) / 2)  # 画面比あたりの焦点距離
        self.fy = 0.5 / math.tan(math.radians(vfov_deg) / 2)
        self.distance_k = distance_k
        # 方位は 1 秒に 90 度くらいまで振れる。距離は 1 秒に 1 m くらい
        self.bearing = Axis(q=200.0**2, r=2.0**2, p0=60.0**2)
        self.elev = Axis(q=100.0**2, r=2.0**2, p0=30.0**2)
        self.dist = Axis(q=100.0**2, r=15.0**2, p0=100.0**2)
        self.t_ns: int | None = None
        self.last_seen_ns: int | None = None
        self.valid = False
        # 撮った時刻のカメラの向きを引くための履歴（t_ns, pan, tilt）
        self._pose: deque[tuple[int, float, float]] = deque(maxlen=4096)

    def record_pose(self, t_ns: int, pan: float, tilt: float) -> None:
        if not self._pose or self._pose[-1][1:] != (pan, tilt) or t_ns - self._pose[-1][0] > 5e6:
            self._pose.append((t_ns, pan, tilt))

    def pose_at(self, t_ns: int) -> tuple[float, float]:
        pan, tilt = (self._pose[0][1], self._pose[0][2]) if self._pose else (0.0, 0.0)
        for t, p, ti in reversed(self._pose):
            if t <= t_ns:
                return p, ti
        return pan, tilt

    def predict_to(self, t_ns: int) -> None:
        if self.t_ns is None:
            self.t_ns = t_ns
            return
        dt = (t_ns - self.t_ns) * 1e-9
        if dt > 0:
            for a in (self.bearing, self.elev, self.dist):
                a.predict(dt)
            self.t_ns = t_ns

    def measure(self, det: Detection, now_ns: int, ultrasonic_cm: float | None = None) -> None:
        pan, tilt = self.pose_at(det.t_ns)
        bearing = pan + math.degrees(math.atan((det.cx - 0.5) / self.fx))
        elev = tilt - math.degrees(math.atan((det.cy - 0.5) / self.fy))
        dist = self.distance_k / max(det.h, 0.02)
        dist_r = (0.25 * dist) ** 2
        if not self.valid:
            self.bearing.reset(bearing, 30.0**2)
            self.elev.reset(elev, 30.0**2)
            self.dist.reset(dist, 50.0**2)
            self.dist.P[0, 0] = dist_r
            self.t_ns = det.t_ns
            self.valid = True
        else:
            # 撮った時刻まで戻す代わりに、遅れの分だけ観測を先に進めてから update する
            lag = max(0.0, (now_ns - det.t_ns) * 1e-9)
            self.predict_to(now_ns)
            self.bearing.update(bearing + self.bearing.x[1] * lag)
            self.elev.update(elev + self.elev.x[1] * lag)
            self.dist.update(dist + self.dist.x[1] * lag, dist_r)
        # 超音波は正面の物までの距離。snoo がほぼ正面にいて、画像の距離と大きく違わないときだけ使う
        if ultrasonic_cm is not None and abs(self.bearing.x[0]) < 10.0:
            if 0.6 * self.dist.x[0] < ultrasonic_cm < 1.5 * self.dist.x[0]:
                self.dist.update(ultrasonic_cm, 5.0**2)
        self.predict_to(now_ns)
        self.last_seen_ns = det.t_ns

    def forget(self) -> None:
        self.valid = False
        self.last_seen_ns = None

    def since_seen_s(self, now_ns: int) -> float:
        return math.inf if self.last_seen_ns is None else (now_ns - self.last_seen_ns) * 1e-9
