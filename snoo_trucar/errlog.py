"""スレッドのループの 1 回分の失敗を、ログが溢れないように間引いて出す。

受信・カメラ・超音波・電池・station のループは、1 回の失敗（壊れた返事・送れない・I2C が一瞬読めない）
では死なせずに続ける。これまでは例外でスレッドが静かに死に、止める仕組みも一緒に外れていた。
"""

from __future__ import annotations

import time
import traceback


class ErrorLog:
    def __init__(self, where: str, every_s: float = 5.0):
        self.where = where
        self.every_s = every_s
        self.count = 0
        self._last = -1e9
        self._skipped = 0

    def __call__(self, e: BaseException) -> None:
        self.count += 1
        now = time.monotonic()
        if now - self._last < self.every_s:
            self._skipped += 1
            return
        more = f"（ほかに {self._skipped} 回）" if self._skipped else ""
        print(f"{self.where}で失敗、続ける{more}: {type(e).__name__}: {e}", flush=True)
        if self.count == 1:
            traceback.print_exception(e)  # 初めの 1 回だけ、どこで起きたかを残す
        self._last, self._skipped = now, 0
