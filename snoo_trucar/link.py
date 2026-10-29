"""カメラの画像を検出サーバーに送り、返事を共有メモリ（meas）に書く。"""

from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

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
        self.started = False  # 直前の hit で追い始めた（見失ってから続けて見えた）

    def hit(self, t_ns: int) -> bool:
        self.started = False
        if self._last_pass is not None and t_ns - self._last_pass <= self.coast_ns:
            self._last_pass = t_ns
            return True
        self._recent = [t for t in self._recent if t_ns - t <= self.window_ns] + [t_ns]
        if len(self._recent) >= self.hits:
            self._recent = []
            self._last_pass = t_ns
            self.started = True
            return True
        return False


class Route:
    """送り先を選ぶ。hosts の先頭（家の LAN）に送り、返事が来なければ次（tailscale）に替える。

    先頭以外に替えた後は、retry_s 秒ごとに先頭を試し直す（家の外では、そのたびに switch_s 秒ほど
    返事が途切れる）。名前は替えるたびに引き直す（DHCP で IP が変わっても追える）。
    """

    def __init__(self, hosts: list[str], port: int, switch_s: float, retry_s: float,
                 resolve=socket.gethostbyname):  # fmt: skip
        self.hosts = [h for h in hosts if h]
        self.port = port
        self.switch_ns = int(switch_s * 1e9)
        self.retry_ns = int(retry_s * 1e9)
        self.resolve = resolve
        self.index = 0
        self.since_ns: int | None = None  # 今の送り先に替えた時刻
        self.last_reply_ns = 0
        self._addr: tuple[str, int] | None = None

    @property
    def host(self) -> str:
        return self.hosts[self.index]

    def _use(self, index: int, now_ns: int) -> None:
        self.index, self.since_ns, self._addr = index, now_ns, None

    def addr(self, now_ns: int) -> tuple[str, int] | None:
        """今送る先（引けなければ次の名前を試す。どれも引けなければ None）。"""
        if self.since_ns is None:
            self.since_ns = now_ns
        heard = max(self.since_ns, self.last_reply_ns)
        if now_ns - heard > self.switch_ns:
            self._use((self.index + 1) % len(self.hosts), now_ns)
        elif self.index and now_ns - self.since_ns > self.retry_ns:
            self._use(0, now_ns)
        for _ in self.hosts:
            if self._addr is None:
                try:
                    self._addr = (self.resolve(self.host), self.port)
                except OSError:
                    self._use((self.index + 1) % len(self.hosts), now_ns)
                    continue
            return self._addr
        return None

    def on_reply(self, now_ns: int) -> None:
        self.last_reply_ns = now_ns


class Link:
    SEEN_GAP_NS = 10_000_000_000  # 見つけたときの写真は 10 秒に 1 枚まで

    def __init__(self, cfg: Config, meas: SeqBuffer, seen_dir: Path | None = None, trace=None,
                 clear: SeqBuffer | None = None):  # fmt: skip
        self.cfg = cfg
        self.meas = meas
        self.trace = trace  # trace.Trace（無ければ測らない）
        srv = cfg.server
        self.route = Route([srv.host, srv.fallback_host], srv.port, srv.switch_after_s,
                           srv.retry_host_s)  # fmt: skip
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
        self.sock.settimeout(0.2)
        self.stats = LinkStats()
        self._lock = threading.Lock()
        self._seq = 0
        self.confirm = Confirm(cfg.control.confirm_hits, cfg.control.confirm_window_s,
                               cfg.control.coast_s)  # fmt: skip
        # 追い始めたときの画像と箱を残す（snoo を本当に見ていたかを後から人が確かめるため）
        self.seen_dir = seen_dir
        # 通り道の空き（clear_cm）を制御プロセスへ。control_proc.CLEAR_N
        self.clear = clear
        # 撮った時刻に書いていた指令 (steer_deg, speed, pan_deg, tilt_deg) を返す関数（SNO2 に載せる）
        self.pose_fn = None
        # 返事に clear_cm の欄か clear_skip があったら呼ぶ（reply）。explore の記録に残して、超音波と比べるため
        self.on_clear = None
        self._frames: dict[int, bytes] = {}
        self._last_seen_save = 0

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
        pose = self.pose_fn() if self.cfg.server.header_v2 and self.pose_fn is not None else None
        data = pack_frame(self._seq, t_capture_ns, cam.width, cam.height, jpeg, pose)
        with self._lock:
            self.stats._in_flight[self._seq] = time.monotonic_ns()
            if self.seen_dir is not None:
                self._frames[self._seq] = jpeg
                for seq in [q for q in self._frames if q not in self.stats._in_flight]:
                    del self._frames[seq]
            self.stats.sent += 1
            self.stats.jpeg_bytes = len(jpeg)
        addr = self.route.addr(time.monotonic_ns())
        if addr is None:
            return  # どの名前も引けない（ネットワークが無い）
        self.sock.sendto(data, addr)
        if self.trace is not None:
            self.trace.mark(t_capture_ns, "sent")

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
            self.route.on_reply(now)
            if self.trace is not None:
                self.trace.mark(reply.t_capture_ns, "reply", now)
            with self._lock:
                st = self.stats
                sent_ns = st._in_flight.pop(reply.seq, None)
                jpeg = self._frames.pop(reply.seq, None)
                # サーバーは溜まった古いフレームを返事なしで捨てるので、それより前の分は諦める
                for seq in [s for s in st._in_flight if s < reply.seq]:
                    del st._in_flight[seq]
                st.replies += 1
                st.last_reply_ns = now
                st.last_infer_ms = reply.infer_ms
                if sent_ns is not None:
                    st.last_rtt_ms = (now - sent_ns) / 1e6
                    if self.trace is not None:
                        # 往復のうちサーバーの外（通信・tailscale・受信スレッドが起きるまで）
                        self.trace.loop("往復のうちサーバー", reply.infer_ms)
                        self.trace.loop("往復のうち外", st.last_rtt_ms - reply.infer_ms)
                if now - reply.t_capture_ns > timeout_ns:
                    st.dropped += 1
                    continue
            if self.clear is not None:
                # 0: 欄が無い（超音波だけで走る）/ 1: 値がある / 2: null（間に合わなかった）
                state = 0 if not reply.clear_present else 1 if reply.clear_cm is not None else 2
                cm = reply.clear_cm if reply.clear_cm is not None else -1.0
                self.clear.write((reply.t_capture_ns, cm, state))
            if self.on_clear is not None and (reply.clear_present or reply.clear_skip):
                self.on_clear(reply)
            box, score = pick_snoo(
                reply.boxes, ctl.cat_threshold, ctl.snoo_threshold, ctl.require_snoo,
                ctl.max_box_area,
            )
            if box is not None:
                self.stats.snoo_hits += 1
                if self.confirm.hit(reply.t_capture_ns):
                    stress = -1.0 if box.stress is None else box.stress
                    self.meas.write((reply.t_capture_ns, box.cx, box.cy, box.w, box.h, score, stress))
                    if self.trace is not None:
                        self.trace.mark(reply.t_capture_ns, "meas")
                    if self.confirm.started and jpeg is not None:
                        self._save_seen(now, jpeg, reply, box)

    def _save_seen(self, now_ns: int, jpeg: bytes, reply, box) -> None:
        if now_ns - self._last_seen_save < self.SEEN_GAP_NS:
            return
        self._last_seen_save = now_ns
        self.seen_dir.mkdir(parents=True, exist_ok=True)
        stem = self.seen_dir / time.strftime("seen_%Y%m%d_%H%M%S")
        stem.with_suffix(".jpg").write_bytes(jpeg)
        info = {"t_capture_ns": reply.t_capture_ns, "picked": asdict(box),
                "boxes": [asdict(b) for b in reply.boxes]}  # fmt: skip
        stem.with_suffix(".json").write_text(json.dumps(info, indent=1))

    def close(self) -> None:
        self.sock.close()


def camera_loop(camera, link: Link, stop: threading.Event, on_frame=None) -> None:
    while not stop.is_set():
        t_ns, jpeg = camera.grab()
        if link.trace is not None:
            t_grab, t_encoded = getattr(camera, "last_times", (0, 0))
            link.trace.mark(t_ns, "grab", t_grab or None)
            if t_encoded:
                link.trace.mark(t_ns, "encoded", t_encoded)
        if on_frame is not None:
            on_frame(t_ns, jpeg)
        if link.can_send(time.monotonic_ns()):
            link.send(t_ns, jpeg)
