"""画像が動いているかを見て、走らせているのに進めていない（ぶつかった・詰まった・乗り上げた）ことに気づく。

超音波は正面の細い円錐しか見えないので、横や低い障害物に当たっても気づけない。車輪の回転計も無い。
そこで、走らせている間の画像を 1 秒前と比べ、止まっているときのざらつき（暗い部屋ではゲインが高く大きい）と
変わらなければ、進めていないとみなす。

画像は 80x60 の白黒に縮めて、平均を引いてから差の絶対値の平均を見る。比べるのは DT_S 離れた 2 枚
（0.2 s 離れた 2 枚では、速さ 15 で 3 cm しか動かず、ざらつきと区別できなかった。2026-10-29 の explore）。
"""

from __future__ import annotations

import threading
from collections import deque

import numpy as np

SAMPLE_HZ = 5.0  # 縮めて比べる頻度
DT_S = 1.0  # これだけ前の画像と比べる
NOISE_GAP_S = 0.2  # 止まっているときのざらつきは、この間をあけた 2 枚で測る
WARMUP_S = 1.2  # 走り始めてからこの間は判定しない（動き出し）
STUCK_HOLD_S = 1.0  # 動いていないのがこれだけ続いたら、進めていないとみなす
# 下がるのは 30 cm まで（速さ 15 で約 1.2 s）なので、上の値（最短 2.2 s）では終わるまで気づけない。
# 下がるときは短く見る（5 Hz で比べるので、走り出しから 0.8〜1.0 s で気づく）。後ろは見えず、
# ぶつかっても他に気づく手段が無いので、早めに止める方を取る
WARMUP_REVERSE_S = 0.4
STUCK_HOLD_REVERSE_S = 0.4
# 前へ走る短い 1 歩（20 cm で約 1 s）は、上の判定（最短 2.2 s）では終わるまで気づけない。
# そこで走ったひと続き（run）ごとに、走り出して RUN_REF_S（首が向きを変え終わる）後の画像を
# 基にして、それとの差の最大を覚える。止まったとき、その差がざらつきと変わらなければ
# 「動いていなかった」とする
# （station.py が prev_step の stop_reason を stuck にする。#21 のレビューの E）
RUN_REF_S = 0.4
RUN_MIN_SAMPLES = 2  # 基にした画像の後に、これだけ比べられたときだけ決める
FACTOR = 1.3  # 止まっているときのざらつきの何倍より小さければ、動いていないとみなす（走っている 1 秒差は 10 以上、止まっているときは 6〜7 以下）
NOISE_FLOOR = 2.0


def shrink(gray: np.ndarray) -> np.ndarray:
    h, w = gray.shape[:2]
    a = gray[:: max(1, h // 60), :: max(1, w // 80)].astype(np.float32)
    return a - a.mean()


class MotionWatch:
    def __init__(self):
        self._hist: deque[tuple[float, np.ndarray]] = deque(maxlen=int(SAMPLE_HZ * (DT_S + 1)) + 2)
        self._last = -1e9
        self.noise = 6.0  # 止まっているときのざらつき（初めは 2026-10-29 の暗い部屋の値）
        self.diff = 0.0  # いちばん新しい「1 秒前との差」
        self._moving_since: float | None = None
        self._still_since: float | None = None
        self._slow_since: float | None = None
        self._reverse = False
        self.stuck = False
        # 走ったひと続きごとの判定（moved: True 動いていた / False 動いていなかった /
        # None 短すぎて分からない）
        self.runs: list[dict] = []
        self._run: dict | None = None
        # feed はカメラのスレッド、take_runs は station のスレッド
        self._runs_lock = threading.Lock()

    def due(self, t: float) -> bool:
        """この時刻の画像を縮めて渡すべきか（重い JPEG の展開を間引くため）"""
        return t - self._last >= 1.0 / SAMPLE_HZ

    def feed(self, t: float, small: np.ndarray, moving: bool, reverse: bool = False) -> bool:
        """small は shrink() したもの。moving は前後に走らせているか（ハンドルだけ・首だけは含めない）。
        reverse は下がっているか"""
        self._last = t
        self._track_run(t, small, moving, reverse)
        if moving:
            self._still_since = None
            if self._moving_since is None or reverse != self._reverse:
                # 走り出した・向きが変わった。進んですぐ下がると、1 秒前と同じ所に戻って似た画像になるので
                # 動き出しから数え直す
                self._moving_since = t
                self._slow_since = None
                self._reverse = reverse
        else:
            self._moving_since = None
            self._slow_since = None
            self.stuck = False
            if self._still_since is None:
                self._still_since = t
            # 止まって落ち着いたら、ざらつきを測る
            prev = self._at(t - NOISE_GAP_S)
            if prev is not None and t - self._still_since >= 0.5:
                d = float(np.abs(small - prev).mean())
                self.noise = 0.8 * self.noise + 0.2 * d
        self._hist.append((t, small))
        warmup, hold = (WARMUP_REVERSE_S, STUCK_HOLD_REVERSE_S) if self._reverse else (WARMUP_S, STUCK_HOLD_S)
        if self._moving_since is None or t - self._moving_since < warmup:
            return self.stuck
        prev = self._at(t - DT_S)
        if prev is None:
            return self.stuck
        self.diff = float(np.abs(small - prev).mean())
        if self.diff < FACTOR * max(self.noise, NOISE_FLOOR):
            if self._slow_since is None:
                self._slow_since = t
            self.stuck = t - self._slow_since >= hold
        else:
            self._slow_since = None
            self.stuck = False
        return self.stuck

    def _track_run(self, t: float, small: np.ndarray, moving: bool, reverse: bool) -> None:
        run = self._run
        if run is not None and (not moving or reverse != run["reverse"]):
            self._end_run(t)
            run = None
        if not moving:
            return
        if run is None:
            run = self._run = {"t0": t, "reverse": reverse, "ref": None, "max": 0.0, "n": 0}
        if run["ref"] is None:
            if t - run["t0"] >= RUN_REF_S - 1e-6:
                run["ref"] = small
            return
        run["max"] = max(run["max"], float(np.abs(small - run["ref"]).mean()))
        run["n"] += 1

    def _end_run(self, t: float) -> None:
        run, self._run = self._run, None
        moved = None
        if run["n"] >= RUN_MIN_SAMPLES:
            moved = run["max"] >= FACTOR * max(self.noise, NOISE_FLOOR)
        row = {"seconds": round(t - run["t0"], 2), "reverse": run["reverse"],
               "diff": round(run["max"], 1), "noise": round(self.noise, 1),
               "moved": moved}  # fmt: skip
        with self._runs_lock:
            self.runs = self.runs[-19:] + [row]

    def take_runs(self) -> list[dict]:
        """前に取ってから走ったひと続きの判定（station.py が prev_step に付ける）"""
        with self._runs_lock:
            runs, self.runs = self.runs, []
        return runs

    def _at(self, t: float) -> np.ndarray | None:
        """t より前で、いちばん新しい画像"""
        best = None
        for ti, a in self._hist:
            if ti <= t:
                best = a
            else:
                break
        return best
