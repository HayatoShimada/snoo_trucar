"""カメラ（Picamera2）。JPEG にして返す。

60 fps では 1 枚の露光が 16.6 ms までになり、暗い部屋では 30 fps の半分の明るさになる。
libcamera の自動露出（AE）はゲインを 8 までしか上げないので、露光もゲインも上限に張り付いて
まだ暗いときだけ、ゲインを手で 16 まで上げる（LowLightBoost）。fps は落とさない。
"""

from __future__ import annotations

import time

from .config import CameraConfig


class LowLightBoost:
    """AE が露光・ゲインとも上限なのに暗いとき、アナログゲインを max_gain まで上げる。

    明るさ（画素の平均）が target に近づくようにゲインを決め、AE の上限（8）より下がれば AE に戻す。
    制御は数フレーム遅れて効くので、every フレームに 1 回だけ変える。
    """

    AE_GAIN_MAX = 8.0

    def __init__(self, frame_us: int, max_gain: float, target: float, every: int = 8):
        self.frame_us = frame_us
        self.max_gain = max_gain
        self.target = target
        self.every = every
        self.gain: float | None = None  # 手で決めているゲイン（None なら AE に任せている）
        self._n = 0

    def update(self, exposure_us: float, gain: float, mean: float) -> dict | None:
        """今のフレームの露光・ゲイン・明るさから、カメラに送る制御（変えないときは None）を返す。"""
        self._n += 1
        if self._n % self.every:
            return None
        if self.gain is None:
            saturated = exposure_us >= 0.9 * self.frame_us and gain >= self.AE_GAIN_MAX - 0.1
            if not saturated or mean >= self.target or self.max_gain <= self.AE_GAIN_MAX:
                return None
            want = self.AE_GAIN_MAX * self.target / max(mean, 1.0)
        else:
            want = gain * self.target / max(mean, 1.0)
            if want < self.AE_GAIN_MAX:
                self.gain = None
                return {"AnalogueGainMode": 0}  # 明るくなったので AE に戻す
        want = min(self.max_gain, want)
        if self.gain is not None and abs(want - self.gain) < 0.5:
            return None
        self.gain = want
        return {"AnalogueGainMode": 1, "AnalogueGain": want}


def capture_time_ns(sensor_ts_ns: int, exposure_us: float, readout_ms: float) -> int:
    """画面の中央の行を露光した時間の真ん中（CLOCK_MONOTONIC の ns）。

    Pi の SensorTimestamp は 1 行目の読み出しの始まり（＝1 行目の露光の終わり）。
    30 fps（露光 33 ms）で受け取りまでが 22 ms だったので、露光の始まりではない。
    中央の行は露光の半分だけ前、読み出しの半分だけ後になる。
    """
    return int(sensor_ts_ns - exposure_us * 500 + readout_ms * 5e5)


class Camera:
    def __init__(self, cfg: CameraConfig):
        import simplejpeg
        from libcamera import Transform
        from picamera2 import Picamera2

        # ISP から YUV420 で受け取り、色の変換をせずに JPEG にする（RGB から 5.5 ms → 2.5 ms、
        # 大きさも 3 分の 2）。JPEG はもともと YUV 4:2:0 なので画質は変わらない
        self._encode = simplejpeg.encode_jpeg_yuv_planes
        self.cfg = cfg
        self.cam = Picamera2()
        frame_us = int(1e6 / cfg.fps)
        conf = self.cam.create_video_configuration(
            main={"size": (cfg.width, cfg.height), "format": "YUV420"},
            transform=Transform(hflip=cfg.rotate180, vflip=cfg.rotate180),
            controls={"FrameDurationLimits": (frame_us, frame_us)},
            buffer_count=4,
        )
        self.cam.configure(conf)
        self.cam.start()
        self.boost = LowLightBoost(frame_us, cfg.max_gain, cfg.brightness_target)
        self.exposure_us = 0.0
        self.gain = 1.0
        self.last_times = (0, 0)  # 直前のフレームの (受け取った時刻, JPEG にし終えた時刻)
        self.last_capture_ns = 0  # 直前のフレームの撮った時刻と明るさ（Y）。servo_step が使う
        self.last_y = None

    def grab(self) -> tuple[int, bytes]:
        """(撮った時刻 ns（CLOCK_MONOTONIC。中央の行の露光の真ん中）, JPEG)"""
        req = self.cam.capture_request()
        try:
            arr = req.make_array("main")
            md = req.get_metadata()
        finally:
            req.release()
        t_grab = time.monotonic_ns()
        ts = md.get("SensorTimestamp")
        self.exposure_us = md.get("ExposureTime", 0)
        self.gain = md.get("AnalogueGain", 1.0)
        t_ns = (
            capture_time_ns(ts, self.exposure_us, self.cfg.readout_ms) if ts else time.monotonic_ns()
        )
        # YUV420 は Y（h 行）の下に U・V（それぞれ h/4 行、幅 w の行に w/2 ずつ 2 行分）が続く
        h, w = self.cfg.height, self.cfg.width
        y = arr[:h]
        u = arr[h : h + h // 4].reshape(h // 2, w // 2)
        v = arr[h + h // 4 : h + h // 2].reshape(h // 2, w // 2)
        ctl = self.boost.update(self.exposure_us, self.gain, float(y[::16, ::16].mean()))
        if ctl is not None:
            self.cam.set_controls(ctl)
        jpeg = self._encode(y, u, v, quality=self.cfg.jpeg_quality)
        self.last_capture_ns, self.last_y = int(t_ns), y
        self.last_times = (t_grab, time.monotonic_ns())
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
