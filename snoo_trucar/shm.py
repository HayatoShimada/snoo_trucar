"""プロセス間で float64 の配列を受け渡す共有メモリ（書き手 1・読み手 1 の seqlock）。

先頭の要素が書き込み回数。書いている間は奇数になるので、読み手は偶数で前後が同じときだけ採る。
5 kHz の制御プロセスがロックを待たずに読めるようにするため、Lock は使わない。
"""

from __future__ import annotations

from multiprocessing import shared_memory

import numpy as np


class SeqBuffer:
    def __init__(self, n: int, name: str | None = None):
        self.n = n
        size = (n + 1) * 8
        if name is None:
            self._shm = shared_memory.SharedMemory(create=True, size=size)
            self._owner = True
        else:
            self._shm = shared_memory.SharedMemory(name=name)
            self._owner = False
        self._a = np.ndarray((n + 1,), dtype=np.float64, buffer=self._shm.buf)
        if self._owner:
            self._a[:] = 0.0

    @property
    def name(self) -> str:
        return self._shm.name

    def write(self, values) -> None:
        a = self._a
        a[0] += 1.0
        a[1:] = values
        a[0] += 1.0

    def read(self) -> tuple[int, np.ndarray] | None:
        """(書き込み回数, 値の写し)。書いている途中なら None"""
        a = self._a
        s1 = a[0]
        values = a[1:].copy()
        s2 = a[0]
        if s1 != s2 or int(s1) % 2:
            return None
        return int(s1) // 2, values

    def close(self) -> None:
        del self._a
        self._shm.close()
        if self._owner:
            self._shm.unlink()
