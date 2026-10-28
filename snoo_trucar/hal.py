"""PiCar-X のハードウェア（robot_hat / picarx）。無いマシンや --sim では FakeCar を使う。

I2C の書き込みは 1 回 0.45 ms ほどかかるので、値が変わったときだけ書く。
"""

from __future__ import annotations

import time

from .config import CarConfig


class Car:
    def __init__(self, cfg: CarConfig):
        import os
        import sys
        import types

        # robot_hat は import 時に pyaudio（音）を読む。使わないので無ければ空のものを置く
        if "pyaudio" not in sys.modules:
            try:
                import pyaudio  # noqa: F401
            except ImportError:
                sys.modules["pyaudio"] = types.SimpleNamespace(paInt16=8)
        from picarx import Picarx

        os.makedirs(os.path.dirname(cfg.picarx_config), exist_ok=True)
        self.px = Picarx(config=cfg.picarx_config)
        self._last = [None, None, None, None]
        self.ultrasonic_ok = True

    def apply(self, steer: float, speed: float, pan: float, tilt: float) -> None:
        px, last = self.px, self._last
        steer, speed = round(steer), round(speed)
        if last[0] != steer:
            px.set_dir_servo_angle(steer)
            last[0] = steer
            last[1] = None  # forward() はハンドルの角度で左右の速さを変えるので、速さも書き直す
        if last[1] != speed:
            if speed > 0:
                px.forward(speed)
            elif speed < 0:
                px.backward(-speed)
            else:
                px.stop()
            last[1] = speed
        if last[2] is None or abs(last[2] - pan) >= 0.5:
            px.set_cam_pan_angle(pan)
            last[2] = pan
        if last[3] is None or abs(last[3] - tilt) >= 0.5:
            px.set_cam_tilt_angle(tilt)
            last[3] = tilt

    def read_ultrasonic(self) -> float | None:
        """cm。測れなければ None（1 回で最大 20 ms ほど待つ）"""
        cm = self.px.ultrasonic._read()
        return cm if cm is not None and cm > 0 else None

    def stop(self) -> None:
        self.px.stop()
        self._last[1] = 0

    def close(self) -> None:
        self.px.stop()
        self.px.set_dir_servo_angle(0)
        self.px.set_cam_pan_angle(0)
        self.px.set_cam_tilt_angle(0)
        time.sleep(0.05)


class FakeCar:
    """コマンドを覚えるだけ。シミュレーション（sim.py）が車体の動きに使う"""

    def __init__(self, cfg: CarConfig | None = None):
        self.steer = self.speed = self.pan = self.tilt = 0.0
        self.writes = 0
        self.ultrasonic_cm: float | None = None
        self.ultrasonic_ok = True

    def apply(self, steer, speed, pan, tilt) -> None:
        self.steer, self.speed, self.pan, self.tilt = steer, speed, pan, tilt
        self.writes += 1

    def read_ultrasonic(self):
        time.sleep(0.02)
        return self.ultrasonic_cm

    def stop(self) -> None:
        self.speed = 0.0

    def close(self) -> None:
        self.stop()
