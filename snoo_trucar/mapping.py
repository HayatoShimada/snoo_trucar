"""map / explore のときの撮影。カメラが向きごとに落ち着いたら 1 枚ずつ保存し、向きと超音波を scan.json に書く。
explore で走っている間（scan_index = DRIVE_INDEX）は DRIVE_FPS で撮り続ける（画像から自分の動きを出すため）。

地図（パノラマ・奥行き・間取り）を組み立てるのは home-linux 側。ここは素材を集めるだけ。
"""

from __future__ import annotations

import json
import time
from pathlib import Path

DRIVE_FPS = 5.0
SETTLE_S = 1.2  # 向きを変えてから撮るまで待つ（サーボが止まり、ぶれが収まるまで）


class MapRecorder:
    def __init__(self, root: Path):
        self.root = root
        self.dir: Path | None = None
        self.entries: list[dict] = []
        self._index = -1
        self._since = 0.0
        self._saved: set[int] = set()
        self.mode = "map"
        self._last_drive = -1e9
        self._drive_n = 0

    def offer(self, t_ns: int, jpeg: bytes, cmd, ultrasonic_cm: float | None) -> None:
        """カメラの 1 フレームごとに呼ぶ。cmd は制御プロセスの出力（control_proc.CMD_N）"""
        from .controller import DRIVE_INDEX, MODES

        mode = MODES[int(cmd[14])] if cmd is not None else "idle"
        if mode not in ("map", "explore"):
            self.finish()
            return
        index = int(cmd[13])
        if index < 0 and index != DRIVE_INDEX:
            return  # snoo を見ている間などは撮らない（探索は続いている）
        if self.dir is None:
            self.mode = mode
            self.dir = self.root / time.strftime(f"{mode}_%Y%m%d_%H%M%S")
            self.dir.mkdir(parents=True, exist_ok=True)
            self.entries, self._saved = [], set()
            print(f"マッピング開始: {self.dir}", flush=True)
        now = time.monotonic()
        if index == DRIVE_INDEX:
            self._index = index
            if now - self._last_drive >= 1.0 / DRIVE_FPS:
                self._last_drive = now
                self._drive_n += 1
                self._save(f"d{self._drive_n:05d}", t_ns, jpeg, cmd, ultrasonic_cm, moving=True)
            return
        if index != self._index:
            self._index, self._since = index, now
            return
        if index in self._saved or now - self._since < SETTLE_S:
            return
        self._save(f"{index:02d}", t_ns, jpeg, cmd, ultrasonic_cm, moving=False)
        self._saved.add(index)

    def _save(self, stem: str, t_ns: int, jpeg: bytes, cmd, ultrasonic_cm, moving: bool) -> None:
        pan, tilt = float(cmd[3]), float(cmd[4])
        name = f"{stem}_pan{pan:+04.0f}_tilt{tilt:+03.0f}.jpg"
        (self.dir / name).write_bytes(jpeg)
        entry = {"file": name, "t_capture_ns": t_ns, "pan_deg": pan, "tilt_deg": tilt,
                 "ultrasonic_cm": ultrasonic_cm}  # fmt: skip
        if self.mode == "explore":
            # 自分の動きの手がかり（指令。車輪の回転計は無い）。station は撮った場所の番号
            # （走っている間の分は、着いた先の station に入れる。control_proc の ex_station）
            entry.update(steer_deg=float(cmd[1]), speed=float(cmd[2]), moving=moving)
            if len(cmd) > 21:
                entry["station"] = int(cmd[21])
        self.entries.append(entry)
        # 途中で止まっても（サービスの入れ替え・電池切れ）残るよう、1 枚ごとに足していく
        with open(self.dir / "frames.jsonl", "a") as f:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")

    def finish(self) -> None:
        if self.dir is None:
            return
        # 2: チルトを較正した後（指令 0° が水平）。1 は指令 0° で実際は約 +26.5°
        scan = {"schema_version": 2, "camera": "picarx-ov5647", "tilt_offset_deg": 0.0,
                "mode": self.mode, "frames": self.entries}  # fmt: skip
        (self.dir / "scan.json").write_text(json.dumps(scan, ensure_ascii=False, indent=1))
        print(f"マッピング終了: {len(self.entries)} 枚 → {self.dir}", flush=True)
        self.dir = None
        self._index = -1
        self._drive_n = 0
