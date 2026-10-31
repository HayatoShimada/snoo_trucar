"""explore で、止まって撮った分（station）を home-linux の地図のサーバーに送り、次の 1 歩をもらう。

制御プロセスが撮り終えると cmd の station_wait に station の番号が出る。ここでその station の画像
（前の station から走ってくる間の分も）と scan.json の形のメタデータを POST し、返事を plan に書く。
道はサーバーがすべて決める（区切りごとに前・後ろ。"gear_segments"）。返事が来ない・失敗したら、
RETRY_S おきに送り直す（制御プロセスは止まって待つ。Pi では道を決めない）。仕様は docs/PROTOCOL.md。
"""

from __future__ import annotations

import json
import threading
import time

from .control_proc import PLAN_N
from .controller import MAX_SEGMENTS, PLAN_DONE, PLAN_MOVE, PLAN_SEG_N, STOP_REASONS
from .errlog import ErrorLog

# 地図のサーバーに届かない・失敗したとき、送り直すまでの秒
RETRY_S = 3.0
# サーバーに伝える、Pi ができること
SUPPORTS = ["reverse", "segments", "long_step", "gear_segments"]


def parse_move(ans: dict) -> tuple[list[tuple[float, float, bool, float | None]], float, float]:
    """move の返事から ([(ハンドル°, cm, 後ろへか, clear_cm の下限 or None)], 前の速さ, 後ろの速さ)。
    区切りの reverse が無ければ返事の reverse（前の版のサーバー）。"""
    back_all = bool(ans.get("reverse"))
    speed = abs(float(ans["speed"]))
    back_speed = abs(float(ans.get("reverse_speed", speed)))
    raw = ans.get("segments") or [{"steer_deg": ans["steer_deg"], "distance_cm": ans["distance_cm"]}]
    segs = []
    for x in raw[:MAX_SEGMENTS]:
        mc = x.get("min_clear_cm")
        segs.append((float(x["steer_deg"]), abs(float(x["distance_cm"])), bool(x.get("reverse", back_all)),
                     float(mc) if mc is not None else None))  # fmt: skip
    return segs, speed, back_speed


def plan_row(k: int, segs, speed: float, back_speed: float) -> tuple:
    """plan の共有メモリの 1 行（control_proc.PLAN_N）"""
    flat = []
    for steer, cm, back, mc in segs:
        flat += [steer, -cm if back else cm, mc if mc is not None else -1.0]
    flat += [0.0] * (PLAN_SEG_N * (MAX_SEGMENTS - len(segs)))
    return (k, PLAN_MOVE, speed, len(segs), back_speed, *flat)


class StationClient:
    def __init__(self, runner, plan_buf):
        self.runner = runner
        self.plan_buf = plan_buf
        self.cfg = runner.cfg
        self._posted = -1
        self._retry_at = 0.0
        self._session = None
        # 返ってきた 1 歩を station ごとに [(steer, cm, 後ろへか, min_clear)]。どれを走ったかは制御プロセスが
        # step_station で伝える
        self._moves: dict[int, list[tuple[float, float, bool, float | None]]] = {}
        self.last = ""  # 状態の行に出す

    def run(self, stop: threading.Event) -> None:
        errors = ErrorLog("station")
        while not stop.is_set():
            try:
                self._once()
            except Exception as e:  # 送れなかった station は、RETRY_S おきに送り直す
                errors(e)
            stop.wait(0.1)

    def _once(self) -> None:
        v = self.runner.last_cmd
        rec = self.runner.recorder
        if v is not None and rec is not None and rec.dir is not None:
            if self._session != rec.dir.name:
                self._session, self._posted, self._moves = rec.dir.name, -1, {}
            k = int(v[18])
            if k >= 0 and k != self._posted and time.monotonic() >= self._retry_at:
                self._posted = k
                if not self._post(k, v, rec):
                    # 届かない・失敗：制御プロセスは待ち続けるので、少しして同じ station を送り直す
                    self._posted, self._retry_at = -1, time.monotonic() + RETRY_S

    def _post(self, k: int, v, rec) -> bool:
        """送って返事を plan に書く。返事を書けたら True"""
        entries = [dict(e) for e in list(rec.entries) if e.get("station") == k]
        link = getattr(self.runner, "link", None)
        if link is not None:
            # 止まって撮った画像に、その画像の検出の猫の箱を付ける（無ければ付けない）
            for e in entries:
                if not e.get("moving"):
                    d = link.recent_dets.get(e.get("t_capture_ns"))
                    if d is not None:
                        e["dets"] = d
        prev = None
        if k > 0:
            secs, reason = float(v[19]), STOP_REASONS[int(v[20])]
            segs = self._moves.get(int(v[23])) if int(v[23]) >= 0 else None
            if segs is not None:
                speed = float(v[24])  # 上限（explore_speed）で抑えた後の、前へ使った速さ
                # 区切りは実際に走った所まで（途中で止めたら、その区切りは止まった所まで）。
                # 距離は guard・電池を通した後の速さで数えたもの（速さ × 秒では、待った間も進んだことになる）
                driven = [float(x) for x in v[26 : 26 + MAX_SEGMENTS]]
                done = [(s_, round(d, 1), b) for (s_, _, b, _), d in zip(segs, driven, strict=False)
                        if d > 0.05]  # fmt: skip
                done = done or [(segs[0][0], 0.0, segs[0][2])]
                # steer_deg は最後に走った区切りのハンドル。reverse は全部が後ろへのとき（前の版の形）
                prev = {"steer_deg": done[-1][0], "speed": abs(speed), "reverse": all(b for *_, b in done),
                        "segments": [{"steer_deg": s_, "distance_cm": d, "reverse": b} for s_, d, b in done],
                        "seconds": round(secs, 2),
                        "est_distance_cm": round(float(v[25]), 1),
                        "stop_reason": reason,
                        "low_obstacle_passed": int(v[22])}  # fmt: skip
        us = self.runner.last_us
        meta = {"schema_version": 2, "camera": "picarx-ov5647", "tilt_offset_deg": 0.0,
                "mode": "explore", "frames": entries, "session_id": self._session, "station_id": k,
                "prev_step": prev, "battery_v": self.runner.battery_v,
                "ultrasonic_cm": us,
                "camera_height_cm": self.cfg.camera.height_cm,
                "supports": SUPPORTS}  # fmt: skip
        if us is not None:
            # 測れたときだけ、測った時刻を付ける（地図の弱い証拠にだけ使う）
            meta["ultrasonic"] = {"cm": us, "t_ns": int(getattr(self.runner, "last_us_t_ns", 0))}
        t0 = time.monotonic()
        try:
            import requests

            files = [("images", (e["file"], (rec.dir / e["file"]).read_bytes(), "image/jpeg"))
                     for e in entries]  # fmt: skip
            r = requests.post(self.cfg.server.map_url, data={"meta": json.dumps(meta)}, files=files,
                              timeout=self.cfg.server.map_timeout_s)  # fmt: skip
            if r.status_code != 200:
                raise RuntimeError(f"{r.status_code} {r.text[:200]}")
            ans = r.json()
            if ans.get("action") == "move":
                segs, speed, back_speed = parse_move(ans)
                plan = plan_row(k, segs, speed, back_speed)
                self._moves[k] = segs
                desc = " → ".join(f"{'後ろ' if b else '前'} ハンドル{s_:+.0f}° {d:.0f}cm" for s_, d, b, _ in segs)
                self.last = f"station {k}: 次は {desc} 位置={ans.get('pose')} 目標={ans.get('target')}"
            elif ans.get("action") == "done":
                plan = (k, PLAN_DONE) + (0.0,) * (PLAN_N - 2)
                self.last = f"station {k}: 終わり（{ans.get('reason')}）"
            else:
                raise RuntimeError(f"知らない返事: {ans}")
        except Exception as e:  # 届かない・失敗 → 送り直して待つ（Pi では道を決めない）
            self.last = (f"station {k}: 地図のサーバーに届かない（{type(e).__name__}: {str(e)[:80]}）"
                         f"→ {RETRY_S:.0f} 秒後に送り直す")  # fmt: skip
            print(self.last, flush=True)
            return False
        if rec.dir is None or rec.dir.name != self._session:
            # 返事を待つ間に explore をやめた・始め直した。前の explore の 1 歩は渡さない
            print(f"station {k}: 返事の前に explore が替わったので捨てる", flush=True)
            return True
        self.plan_buf.write(plan)
        print(f"{self.last}（{len(entries)} 枚、{time.monotonic() - t0:.1f}s）", flush=True)
        return True
