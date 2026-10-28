"""カメラの画像を検出サーバーに送り、返事を共有メモリ（meas）に書く。"""

from __future__ import annotations

import socket
import threading
import time
from dataclasses import dataclass, field

from .config import Config
from .protocol import pack_frame, parse_reply, pick_snoo
from .shm import SeqBuffer


@dataclass
class LinkStats:
    sent: int = 0
    replies: int = 0
    dropped: int = 0  # 遅すぎた返事
    snoo_hits: int = 0
    last_rtt_ms: float = 0.0
    last_infer_ms: float = 0.0
    last_reply_ns: int = 0
    jpeg_bytes: int = 0
    _in_flight: dict[int, int] = field(default_factory=dict)


class Confirm:
    """続けて見えたときだけ通す。追っている間（最後に通してから coast_s 以内）は 1 回でも通す"""

    def __init__(self, hits: int, window_s: float, coast_s: float):
        self.hits = hits
        self.window_ns = int(window_s * 1e9)
        self.coast_ns = int(coast_s * 1e9)
        self._recent: list[int] = []
        self._last_pass: int | None = None

    def hit(self, t_ns: int) -> bool:
        if self._last_pass is not None and t_ns - self._last_pass <= self.coast_ns:
            self._last_pass = t_ns
            return True
        self._recent = [t for t in self._recent if t_ns - t <= self.window_ns] + [t_ns]
        if len(self._recent) >= self.hits:
            self._recent = []
            self._last_pass = t_ns
            return True
        return False


class Link:
    def __init__(self, cfg: Config, meas: SeqBuffer):
        self.cfg = cfg
        self.meas = meas
        self.addr = (socket.gethostbyname(cfg.server.host), cfg.server.port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
        self.sock.settimeout(0.2)
        self.stats = LinkStats()
        self._lock = threading.Lock()
        self._seq = 0
        self.confirm = Confirm(cfg.control.confirm_hits, cfg.control.confirm_window_s,
                               cfg.control.coast_s)  # fmt: skip

    def can_send(self, now_ns: int) -> bool:
        timeout_ns = int(self.cfg.server.reply_timeout_s * 1e9)
        with self._lock:
            flight = self.stats._in_flight
            for seq in [s for s, t in flight.items() if now_ns - t > timeout_ns]:
                del flight[seq]
            return len(flight) < self.cfg.server.max_in_flight

    def send(self, t_capture_ns: int, jpeg: bytes) -> None:
        cam = self.cfg.camera
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        data = pack_frame(self._seq, t_capture_ns, cam.width, cam.height, jpeg)
        with self._lock:
            self.stats._in_flight[self._seq] = time.monotonic_ns()
            self.stats.sent += 1
            self.stats.jpeg_bytes = len(jpeg)
        self.sock.sendto(data, self.addr)

    def receive_forever(self, stop: threading.Event) -> None:
        ctl = self.cfg.control
        timeout_ns = int(self.cfg.server.reply_timeout_s * 1e9)
        while not stop.is_set():
            try:
                data, _ = self.sock.recvfrom(65535)
            except TimeoutError:
                continue
            except OSError:
                if stop.is_set():
                    return
                raise
            now = time.monotonic_ns()
            try:
                reply = parse_reply(data)
            except (ValueError, KeyError):
                continue
            with self._lock:
                st = self.stats
                sent_ns = st._in_flight.pop(reply.seq, None)
                # サーバーは溜まった古いフレームを返事なしで捨てるので、それより前の分は諦める
                for seq in [s for s in st._in_flight if s < reply.seq]:
                    del st._in_flight[seq]
                st.replies += 1
                st.last_reply_ns = now
                st.last_infer_ms = reply.infer_ms
                if sent_ns is not None:
                    st.last_rtt_ms = (now - sent_ns) / 1e6
                if now - reply.t_capture_ns > timeout_ns:
                    st.dropped += 1
                    continue
            box, score = pick_snoo(
                reply.boxes, ctl.cat_threshold, ctl.snoo_threshold, ctl.require_snoo,
                ctl.max_box_area,
            )
            if box is not None:
                self.stats.snoo_hits += 1
                if self.confirm.hit(reply.t_capture_ns):
                    self.meas.write((reply.t_capture_ns, box.cx, box.cy, box.w, box.h, score))

    def close(self) -> None:
        self.sock.close()


def camera_loop(camera, link: Link, stop: threading.Event, on_frame=None) -> None:
    while not stop.is_set():
        t_ns, jpeg = camera.grab()
        if on_frame is not None:
            on_frame(t_ns, jpeg)
        if link.can_send(time.monotonic_ns()):
            link.send(t_ns, jpeg)
