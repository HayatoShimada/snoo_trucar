"""explore で、止まって撮った分（station）を home-linux の地図のサーバーに送り、次の 1 歩をもらう。

制御プロセスが撮り終えると cmd の station_wait に station の番号が出る。ここでその station の画像
（前の station から走ってくる間の分も）と scan.json の形のメタデータを POST し、返事を plan に書く。
返事が来ない・失敗したら PLAN_FALLBACK を書き、制御プロセスは Pi だけで次を決める。仕様は docs/PROTOCOL.md。
"""

from __future__ import annotations

import json
import threading
import time

from .controller import PLAN_DONE, PLAN_FALLBACK, PLAN_MOVE, STOP_REASONS


class StationClient:
    def __init__(self, runner, plan_buf):
        self.runner = runner
        self.plan_buf = plan_buf
        self.cfg = runner.cfg
        self._posted = -1
        self._session = None
        self._last_move = None  # 直前に返ってきた 1 歩 (steer, speed)
        self.last = ""  # 状態の行に出す

    def run(self, stop: threading.Event) -> None:
        while not stop.is_set():
            v = self.runner.last_cmd
            rec = self.runner.recorder
            if v is not None and rec is not None and rec.dir is not None:
                if self._session != rec.dir.name:
                    self._session, self._posted, self._last_move = rec.dir.name, -1, None
                k = int(v[18])
                if k >= 0 and k != self._posted:
                    self._posted = k
                    self._post(k, v, rec)
            stop.wait(0.1)

    def _post(self, k: int, v, rec) -> None:
        import requests

        entries = [e for e in list(rec.entries) if e.get("station") == k]
        prev = None
        if k > 0:
            secs, reason = float(v[19]), STOP_REASONS[int(v[20])]
            if self._last_move is not None:
                steer, speed = self._last_move
                prev = {"steer_deg": steer, "speed": speed, "seconds": round(secs, 2),
                        "est_distance_cm": round(speed * self.cfg.car.cm_s_per_speed * secs, 1),
                        "stop_reason": reason}  # fmt: skip
            else:
                prev = {"stop_reason": "local"}  # 前の 1 歩は Pi だけで決めた
        meta = {"schema_version": 2, "camera": "picarx-ov5647", "tilt_offset_deg": 0.0,
                "mode": "explore", "frames": entries, "session_id": self._session, "station_id": k,
                "prev_step": prev, "battery_v": self.runner.battery_v,
                "ultrasonic_cm": self.runner.last_us}  # fmt: skip
        t0 = time.monotonic()
        plan = (k, PLAN_FALLBACK, 0.0, 0.0, 0.0)
        try:
            files = [("images", (e["file"], (rec.dir / e["file"]).read_bytes(), "image/jpeg"))
                     for e in entries]  # fmt: skip
            r = requests.post(self.cfg.server.map_url, data={"meta": json.dumps(meta)}, files=files,
                              timeout=self.cfg.server.map_timeout_s)  # fmt: skip
            if r.status_code != 200:
                raise RuntimeError(f"{r.status_code} {r.text[:200]}")
            ans = r.json()
            if ans.get("action") == "move":
                steer, speed = float(ans["steer_deg"]), float(ans["speed"])
                plan = (k, PLAN_MOVE, steer, speed, float(ans["distance_cm"]))
                self._last_move = (steer, speed)
                self.last = (f"station {k}: 次は {ans['distance_cm']:.0f}cm ハンドル{steer:+.0f}° "
                             f"位置={ans.get('pose')}")  # fmt: skip
            elif ans.get("action") == "done":
                plan = (k, PLAN_DONE, 0.0, 0.0, 0.0)
                self.last = f"station {k}: 終わり（{ans.get('reason')}）"
            else:
                raise RuntimeError(f"知らない返事: {ans}")
        except Exception as e:  # 届かない・失敗 → Pi だけで決める
            self._last_move = None
            self.last = f"station {k}: 地図のサーバーに届かない（{type(e).__name__}: {str(e)[:80]}）→ Pi だけで決める"
        self.plan_buf.write(plan)
        print(f"{self.last}（{len(entries)} 枚、{time.monotonic() - t0:.1f}s）", flush=True)
