"""explore で、止まって撮った分（station）を home-linux の地図のサーバーに送り、次の 1 歩をもらう。

制御プロセスが撮り終えると cmd の station_wait に station の番号が出る。ここでその station の画像
（前の station から走ってくる間の分も）と scan.json の形のメタデータを POST し、返事を plan に書く。
返事が来ない・失敗したら PLAN_FALLBACK を書き、制御プロセスは Pi だけで次を決める。仕様は docs/PROTOCOL.md。
"""

from __future__ import annotations

import json
import threading
import time

from .control_proc import PLAN_N
from .controller import MAX_SEGMENTS, PLAN_DONE, PLAN_FALLBACK, PLAN_MOVE, STOP_REASONS
from .errlog import ErrorLog


class StationClient:
    def __init__(self, runner, plan_buf):
        self.runner = runner
        self.plan_buf = plan_buf
        self.cfg = runner.cfg
        self._posted = -1
        self._session = None
        self._last_move = None  # 直前に返ってきた 1 歩 ([(steer, cm)], speed)
        self.last = ""  # 状態の行に出す

    def run(self, stop: threading.Event) -> None:
        errors = ErrorLog("station")
        while not stop.is_set():
            try:
                self._once()
            except Exception as e:  # 送れなかった station は、制御プロセスが 25 s 待って Pi だけで決める
                errors(e)
            stop.wait(0.1)

    def _once(self) -> None:
        v = self.runner.last_cmd
        rec = self.runner.recorder
        if v is not None and rec is not None and rec.dir is not None:
            if self._session != rec.dir.name:
                self._session, self._posted, self._last_move = rec.dir.name, -1, None
            k = int(v[18])
            if k >= 0 and k != self._posted:
                self._posted = k
                self._post(k, v, rec)

    def _post(self, k: int, v, rec) -> None:
        entries = [e for e in list(rec.entries) if e.get("station") == k]
        prev = None
        if k > 0:
            secs, reason = float(v[19]), STOP_REASONS[int(v[20])]
            if self._last_move is not None:
                segs, speed = self._last_move
                # steer_deg は最後に走った区切りのハンドル（途中で止めたときは、そこまで）
                prev = {"steer_deg": segs[-1][0], "speed": abs(speed), "reverse": speed < 0,
                        "segments": [{"steer_deg": s, "distance_cm": d} for s, d in segs],
                        "seconds": round(secs, 2),
                        "est_distance_cm": round(abs(speed) * self.cfg.car.cm_s_per_speed * secs, 1),
                        "stop_reason": reason,
                        "low_obstacle_passed": int(v[22]) if len(v) > 22 else 0}  # fmt: skip
            else:
                prev = {"stop_reason": "local"}  # 前の 1 歩は Pi だけで決めた
        meta = {"schema_version": 2, "camera": "picarx-ov5647", "tilt_offset_deg": 0.0,
                "mode": "explore", "frames": entries, "session_id": self._session, "station_id": k,
                "prev_step": prev, "battery_v": self.runner.battery_v,
                "ultrasonic_cm": self.runner.last_us,
                "camera_height_cm": self.cfg.camera.height_cm,
                "supports": ["reverse", "segments", "long_step"]}  # fmt: skip
        t0 = time.monotonic()
        plan = (k, PLAN_FALLBACK) + (0.0,) * (PLAN_N - 2)
        try:
            import requests  # 無ければ Pi だけで決める（下の except）

            files = [("images", (e["file"], (rec.dir / e["file"]).read_bytes(), "image/jpeg"))
                     for e in entries]  # fmt: skip
            r = requests.post(self.cfg.server.map_url, data={"meta": json.dumps(meta)}, files=files,
                              timeout=self.cfg.server.map_timeout_s)  # fmt: skip
            if r.status_code != 200:
                raise RuntimeError(f"{r.status_code} {r.text[:200]}")
            ans = r.json()
            if ans.get("action") == "move":
                speed = abs(float(ans["speed"]))
                if ans.get("reverse"):
                    speed = -speed  # 下がる（plan では速さの符号で伝える）
                segs = ans.get("segments") or [{"steer_deg": ans["steer_deg"], "distance_cm": ans["distance_cm"]}]
                segs = [(float(s["steer_deg"]), float(s["distance_cm"])) for s in segs[:MAX_SEGMENTS]]
                flat = [x for s in segs for x in s] + [0.0] * (2 * (MAX_SEGMENTS - len(segs)))
                plan = (k, PLAN_MOVE, speed, len(segs), *flat)
                self._last_move = (segs, speed)
                desc = " → ".join(f"ハンドル{s:+.0f}° {d:.0f}cm" for s, d in segs)
                self.last = (f"station {k}: 次は{'（下がる）' if speed < 0 else ''} {desc} "
                             f"位置={ans.get('pose')} 目標={ans.get('target')}")  # fmt: skip
            elif ans.get("action") == "done":
                plan = (k, PLAN_DONE) + (0.0,) * (PLAN_N - 2)
                self.last = f"station {k}: 終わり（{ans.get('reason')}）"
            else:
                raise RuntimeError(f"知らない返事: {ans}")
        except Exception as e:  # 届かない・失敗 → Pi だけで決める
            self._last_move = None
            self.last = f"station {k}: 地図のサーバーに届かない（{type(e).__name__}: {str(e)[:80]}）→ Pi だけで決める"
        if rec.dir is None or rec.dir.name != self._session:
            # 返事を待つ間に explore をやめた・始め直した。前の explore の 1 歩は渡さない
            print(f"station {k}: 返事の前に explore が替わったので捨てる", flush=True)
            return
        self.plan_buf.write(plan)
        print(f"{self.last}（{len(entries)} 枚、{time.monotonic() - t0:.1f}s）", flush=True)
