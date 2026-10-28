"""カメラ（Picamera2）。JPEG にして返す。"""

from __future__ import annotations

import time

from .config import CameraConfig


class Camera:
    def __init__(self, cfg: CameraConfig):
        import simplejpeg
        from libcamera import Transform
        from picamera2 import Picamera2

        self._encode = simplejpeg.encode_jpeg
        self.cfg = cfg
        self.cam = Picamera2()
        frame_us = int(1e6 / cfg.fps)
        conf = self.cam.create_video_configuration(
            main={"size": (cfg.width, cfg.height), "format": "RGB888"},
            transform=Transform(hflip=cfg.rotate180, vflip=cfg.rotate180),
            controls={"FrameDurationLimits": (frame_us, frame_us)},
            buffer_count=4,
        )
        self.cam.configure(conf)
        self.cam.start()

    def grab(self) -> tuple[int, bytes]:
        """(撮った時刻 ns（CLOCK_MONOTONIC）, JPEG)"""
        req = self.cam.capture_request()
        try:
            arr = req.make_array("main")
            # SensorTimestamp は露光の始まり（CLOCK_MONOTONIC の ns）。無ければ今
            t_ns = req.get_metadata().get("SensorTimestamp") or time.monotonic_ns()
        finally:
            req.release()
        # Picamera2 の RGB888 は B, G, R の順に並ぶ
        jpeg = self._encode(arr, quality=self.cfg.jpeg_quality, colorspace="BGR")
        return int(t_ns), jpeg

    def close(self) -> None:
        self.cam.stop()
        self.cam.close()


class FileCamera:
    """カメラの代わりに画像ファイルを fps で流す（カメラの無いときに、サーバーとハードを通しで試す）"""

    def __init__(self, cfg: CameraConfig, path: str):
        from pathlib import Path

        self.jpeg = Path(path).read_bytes()
        self.period = 1.0 / cfg.fps
        self._next = time.monotonic()

    def grab(self) -> tuple[int, bytes]:
        self._next += self.period
        time.sleep(max(0.0, self._next - time.monotonic()))
        return time.monotonic_ns(), self.jpeg

    def close(self) -> None:
        pass
