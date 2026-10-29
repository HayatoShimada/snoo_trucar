"""map のときの撮影。カメラが向きごとに落ち着いたら 1 枚ずつ保存し、向きと超音波を scan.json に書く。

地図（パノラマ・奥行き・間取り）を組み立てるのは home-linux 側。ここは素材を集めるだけ。
"""

from __future__ import annotations

import json
import time
from pathlib import Path

SETTLE_S = 1.2  # 向きを変えてから撮るまで待つ（サーボが止まり、ぶれが収まるまで）


class MapRecorder:
    def __init__(self, root: Path):
        self.root = root
        self.dir: Path | None = None
        self.entries: list[dict] = []
        self._index = -1
        self._since = 0.0
        self._saved: set[int] = set()

    def offer(self, t_ns: int, jpeg: bytes, cmd, ultrasonic_cm: float | None) -> None:
        """カメラの 1 フレームごとに呼ぶ。cmd は制御プロセスの出力（control_proc.CMD_N）"""
        index = int(cmd[13]) if cmd is not None else -1
        if index < 0:
            self.finish()
            return
        if self.dir is None:
            self.dir = self.root / time.strftime("map_%Y%m%d_%H%M%S")
            self.dir.mkdir(parents=True, exist_ok=True)
            self.entries, self._saved = [], set()
            print(f"マッピング開始: {self.dir}", flush=True)
        now = time.monotonic()
        if index != self._index:
            self._index, self._since = index, now
            return
        if index in self._saved or now - self._since < SETTLE_S:
            return
        pan, tilt = float(cmd[3]), float(cmd[4])
        name = f"{index:02d}_pan{pan:+04.0f}_tilt{tilt:+03.0f}.jpg"
        (self.dir / name).write_bytes(jpeg)
        self.entries.append(
            {"file": name, "t_capture_ns": t_ns, "pan_deg": pan, "tilt_deg": tilt,
             "ultrasonic_cm": ultrasonic_cm}  # fmt: skip
        )
        self._saved.add(index)

    def finish(self) -> None:
        if self.dir is None:
            return
        # 2: チルトを較正した後（指令 0° が水平）。1 は指令 0° で実際は約 +26.5°
        scan = {"schema_version": 2, "camera": "picarx-ov5647", "tilt_offset_deg": 0.0,
                "frames": self.entries}  # fmt: skip
        (self.dir / "scan.json").write_text(json.dumps(scan, ensure_ascii=False, indent=1))
        print(f"マッピング終了: {len(self.entries)} 枚 → {self.dir}", flush=True)
        self.dir = None
        self._index = -1
