"""1 フレームが撮られてから、その結果でサーボ・モーターに書くまでの時刻を集め、区間ごとに出す。

時刻はすべて Pi の CLOCK_MONOTONIC（ns）。フレームは撮った時刻（t_capture_ns）で見分ける。

- capture: 露光の真ん中（camera.capture_time_ns）
- grab: libcamera から受け取った
- encoded: JPEG にし終えた
- sent: サーバーに送った
- reply: 返事を受け取った
- meas: 共有メモリ（meas）に書いた（snoo を選べたフレームだけ）
- used: 制御プロセスがその検出で推定を更新した
- applied: その推定から出したコマンドを I2C に書いた（書き込みループの 0 次ホールドの分も入る）

返事までは全部のフレームで測れる。meas から先は snoo（require_snoo=false なら猫）が見えているときだけ。
"""

from __future__ import annotations

import json
import threading
import time
from collections import OrderedDict, defaultdict
from pathlib import Path

POINTS = ("capture", "grab", "encoded", "sent", "reply", "meas", "used", "applied")
# 区間（名前, 始め, 終わり）。最後の 2 つは全体
SECTIONS = (
    ("カメラ", "capture", "grab"),
    ("JPEG", "grab", "encoded"),
    ("送るまで", "encoded", "sent"),
    ("往復", "sent", "reply"),
    ("選ぶ", "reply", "meas"),
    ("制御が読む", "meas", "used"),
    ("書くまで", "used", "applied"),
    ("撮影→返事", "capture", "reply"),
    ("撮影→書き込み", "capture", "applied"),
)
MAX_OPEN = 64  # 書き込みまで届かないフレーム（snoo が見えていない等）は、これを超えたら古い順に閉じる


def percentiles(values: list[float]) -> tuple[float, float, float]:
    v = sorted(values)
    return v[len(v) // 2], v[min(len(v) - 1, int(len(v) * 0.95))], v[-1]


class Trace:
    """フレームごとの時刻を集める。カメラ・受信・書き込みのスレッドから呼ぶのでロックする。"""

    def __init__(self, path: Path | None = None):
        self._lock = threading.Lock()
        self._open: OrderedDict[int, dict[str, int]] = OrderedDict()
        self._sections: dict[str, list[float]] = defaultdict(list)
        self._loops: dict[str, list[float]] = defaultdict(list)
        self._file = None
        if path is not None:
            path.parent.mkdir(parents=True, exist_ok=True)
            self._file = open(path, "a", buffering=1)

    def mark(self, t_capture_ns: int, point: str, t_ns: int | None = None) -> None:
        t_ns = time.monotonic_ns() if t_ns is None else t_ns
        with self._lock:
            frame = self._open.get(t_capture_ns)
            if frame is None:
                if point not in ("capture", "grab"):
                    return  # もう閉じたフレーム
                frame = self._open[t_capture_ns] = {"capture": t_capture_ns}
                while len(self._open) > MAX_OPEN:
                    self._close(*self._open.popitem(last=False))
            frame.setdefault(point, t_ns)
            if point == "applied":
                self._close(t_capture_ns, self._open.pop(t_capture_ns))

    def loop(self, name: str, ms: float) -> None:
        """ループ 1 回の時間や周期の揺れ（ms）を記録する。"""
        with self._lock:
            self._loops[name].append(ms)

    def _close(self, _key: int, frame: dict[str, int]) -> None:
        for name, a, b in SECTIONS:
            if a in frame and b in frame:
                self._sections[name].append((frame[b] - frame[a]) / 1e6)
        if self._file is not None:
            self._file.write(json.dumps(frame) + "\n")

    def summary(self) -> str:
        """「区間 p50/p95/最大 ms」を並べた 1 行。集めた値は消す（閉じていないフレームは残す）。"""
        with self._lock:
            parts = [
                f"{name} {'/'.join(f'{x:.1f}' for x in percentiles(self._sections[name]))}"
                for name, _, _ in SECTIONS
                if self._sections.get(name)
            ]
            parts += [
                f"{name} {'/'.join(f'{x:.1f}' for x in percentiles(v))}"
                for name, v in self._loops.items()
                if v
            ]
            self._sections.clear()
            self._loops.clear()
        return "区間 p50/p95/最大 ms: " + ("  ".join(parts) if parts else "（まだ無い）")

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
