"""起動し直しても explore を同じセッションで続ける（snoo_trucar#30）。

explore の間、explore.json に mode・session_id・次の station_id・始めた時刻・今の段階を
atomic に書く。起動したとき、それが MAX_AGE_S より新しければ、同じ session_id・続きの
station_id で explore に戻る。
戻ったら、まずその場で首を振って撮る（prev_step は null）。走るのはサーバーの返事が来てから。
explore をやめたら（USR の長押し・調べ終えた・時間切れ）消す。

段階（phase）は、入れ替えてよいかを見るため（cli の wait-scan）。scan（撮っている）・
plan（返事を待っている）・step（走っている）。入れ替えてよいのは scan の間だけ。
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path

MAX_AGE_S = 600.0  # これより古ければ戻らない（最後に書いてから）


class ExploreState:
    def __init__(self, path: Path | None):
        self.path = path
        self.started = 0.0  # explore を始めた時刻（UNIX 時刻。起動し直しても引き継ぐ）
        self._last: tuple | None = None  # 最後に書いた中身（同じものは書かない。SD を減らす）

    def load(self, now: float | None = None) -> dict | None:
        """戻ってよい中身。無い・壊れている・古い・explore でないなら None"""
        if self.path is None:
            return None
        now = time.time() if now is None else now
        try:
            d = json.loads(self.path.read_text())
            ok = (d.get("mode") == "explore" and isinstance(d.get("session_id"), str)
                  and int(d.get("next_station", -1)) >= 0
                  and -60.0 <= now - float(d["updated"]) <= MAX_AGE_S)  # fmt: skip
        except (OSError, ValueError, TypeError, KeyError):
            return None
        if not ok or "/" in d["session_id"] or d["session_id"].startswith("."):
            return None
        self.started = float(d.get("started", now))
        return d

    def write(self, session_id: str, next_station: int, phase: str,
              now: float | None = None) -> None:  # fmt: skip
        if self.path is None:
            return
        now = time.time() if now is None else now
        if not self.started:
            self.started = now
        key = (session_id, next_station, phase)
        if key == self._last:
            return
        d = {"mode": "explore", "session_id": session_id, "next_station": int(next_station),
             "started": round(self.started, 3), "updated": round(now, 3),
             "phase": phase}  # fmt: skip
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_name(self.path.name + ".tmp")
        with open(tmp, "w") as f:
            f.write(json.dumps(d))
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.path)  # 書いている途中で落ちても、前の中身か新しい中身のどちらか
        self._last = key

    def clear(self) -> None:
        self.started, self._last = 0.0, None
        if self.path is None:
            return
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass


def phase_of(cmd) -> str:
    """制御プロセスの出力（control_proc.CMD_N）から、explore の今の段階"""
    from .control_proc import C_STATION_WAIT
    from .controller import DRIVE_INDEX

    if int(cmd[C_STATION_WAIT]) >= 0:
        return "plan"
    if int(cmd[13]) == DRIVE_INDEX:
        return "step"
    return "scan"
