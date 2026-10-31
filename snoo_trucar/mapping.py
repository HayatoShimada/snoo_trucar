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
        self._pose = (0.0, 0.0)  # 直前のフレームのときの向き（指令）
        self._settle = SETTLE_S
        # 撮れなかった訳を出すため（station.py）。最後にカメラのコマが来た時刻、撮る前に次の向きへ
        # 替わった向き（scan_index。時刻の窓に来なかった）、書き込みに失敗した回数
        self.last_offer_ns = 0
        self.missed: list[int] = []
        self.write_errors = 0
        self._resume: str | None = None  # 次に撮り始めるとき、このセッションの置き場に続けて足す

    def offer(self, t_ns: int, jpeg: bytes, cmd, ultrasonic_cm: float | None) -> None:
        """カメラの 1 フレームごとに呼ぶ。cmd は制御プロセスの出力（control_proc.CMD_N）"""
        from .controller import DRIVE_INDEX, MODES

        self.last_offer_ns = t_ns
        mode = MODES[int(cmd[14])] if cmd is not None else "idle"
        if mode not in ("map", "explore"):
            self.finish()
            return
        index = int(cmd[13])
        if index < 0 and index != DRIVE_INDEX:
            return  # snoo を見ている間などは撮らない（探索は続いている）
        if self.dir is None:
            self.mode = mode
            name, self._resume = (self._resume if mode == "explore" else None), None
            self.dir = self.root / (name or time.strftime(f"{mode}_%Y%m%d_%H%M%S"))
            self.dir.mkdir(parents=True, exist_ok=True)
            self.entries, self._saved = [], set()
            self.missed, self.write_errors = [], 0
            if name:
                self._load_entries()
            print(f"マッピング{'の続き' if name else '開始'}: {self.dir}", flush=True)
        now = time.monotonic()
        if index == DRIVE_INDEX:
            self._index = index
            self._pose = (float(cmd[3]), float(cmd[4]))
            if now - self._last_drive >= 1.0 / DRIVE_FPS:
                self._last_drive = now
                self._drive_n += 1
                self._save(f"d{self._drive_n:05d}", t_ns, jpeg, cmd, ultrasonic_cm, moving=True)
            return
        pose = (float(cmd[3]), float(cmd[4]))
        if index != self._index:
            if self._index >= 0 and self._index not in self._saved:
                self.missed = self.missed[-99:] + [self._index]
            settle = SETTLE_S
            if mode == "explore":
                # 向きの差に合わせて待つ（controller.scan_schedule と同じ）
                from .controller import scan_settle_s

                settle = scan_settle_s(max(abs(pose[0] - self._pose[0]), abs(pose[1] - self._pose[1])))
            self._index, self._since, self._settle = index, now, settle
            self._pose = pose
            return
        self._pose = pose
        if index in self._saved or now - self._since < self._settle:
            return
        if self._save(f"{index:02d}", t_ns, jpeg, cmd, ultrasonic_cm, moving=False):
            self._saved.add(index)

    def resume(self, session_id: str) -> None:
        """起動し直した後、explore の同じセッション（置き場の名前）に続けて撮る（resume.py）"""
        self._resume = session_id

    def _load_entries(self) -> None:
        """前のプロセスが frames.jsonl に足した分を読む。scan.json に全部を残し、走っている間の
        画像の名前（d00001…）を上書きしないため"""
        try:
            lines = (self.dir / "frames.jsonl").read_text().splitlines()
        except FileNotFoundError:
            return
        by_file = {}
        for line in lines:
            try:
                e = json.loads(line)
            except ValueError:
                continue  # 書いている途中で落ちた行
            by_file.pop(e.get("file"), None)
            by_file[e.get("file")] = e  # 撮り直した画像は後の行
        self.entries = list(by_file.values())
        names = [str(e.get("file", "")) for e in self.entries]
        drives = [int(n[1:6]) for n in names if n.startswith("d") and n[1:6].isdigit()]
        self._drive_n = max(drives, default=0)

    def _save(self, stem: str, t_ns: int, jpeg: bytes, cmd, ultrasonic_cm, moving: bool) -> bool:
        pan, tilt = float(cmd[3]), float(cmd[4])
        name = f"{stem}_pan{pan:+04.0f}_tilt{tilt:+03.0f}.jpg"
        try:
            (self.dir / name).write_bytes(jpeg)
        except OSError as e:  # SD がいっぱい・壊れた。entries に入れない（送れない画像を載せない）
            self.write_errors += 1
            if self.write_errors <= 3:
                print(f"画像を書けなかった（{name}）: {e}", flush=True)
            return False
        entry = {"file": name, "t_capture_ns": t_ns, "pan_deg": pan, "tilt_deg": tilt,
                 "ultrasonic_cm": ultrasonic_cm}  # fmt: skip
        if not moving:
            # 同じ向きを撮り直した（起動し直して同じ station を撮った）。同じ名前を 2 つ送らない
            self.entries = [e for e in self.entries if e.get("file") != name]
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
        return True

    def note_clear(self, t_ns: int, clear_cm, clear_conf, clear_skip, ultrasonic_cm) -> None:
        """サーバーが画像から出した通り道の空きを clear.jsonl に残す（t_capture_ns で frames と突き合わせる）"""
        d = self.dir
        if d is None:
            return
        row = {"t_capture_ns": t_ns, "clear_cm": clear_cm, "clear_conf": clear_conf,
               "clear_skip": clear_skip, "ultrasonic_cm": ultrasonic_cm}  # fmt: skip
        with open(d / "clear.jsonl", "a") as f:
            f.write(json.dumps(row) + "\n")

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
