"""explore で、止まって撮った分（station）を home-linux の地図のサーバーに送り、次の 1 歩をもらう。

制御プロセスが撮り終えると cmd の station_wait に station の番号が出る。ここでその station の画像
（前の station から走ってくる間の分も）と scan.json の形のメタデータを POST し、返事を plan に書く。
道はサーバーがすべて決める（区切りごとに前・後ろ。"gear_segments"）。返事が来ない・失敗したら、
RETRY_S おきに送り直す（制御プロセスは止まって待つ。Pi では道を決めない）。仕様は docs/PROTOCOL.md。

止まって撮ったコマが足りない（0 枚など）ときは送らず、その場で撮り直させる（PLAN_RESHOOT）。
サーバーが 4xx（中身がおかしい）を返したときも、同じものを送り直さず撮り直させる。撮り直しは
MAX_RESHOOTS 回まで。それでも 1 枚も無い・断られるなら、探索をやめる
（PLAN_DONE。止まって待ち続けない）。
"""

from __future__ import annotations

import json
import threading
import time

from .control_proc import PLAN_N
from .controller import (
    MAX_SEGMENTS,
    PLAN_DONE,
    PLAN_MOVE,
    PLAN_RESHOOT,
    PLAN_SEG_N,
    STOP_REASONS,
    scan_poses,
)
from .errlog import ErrorLog

# 地図のサーバーに届かない・失敗したとき、送り直すまでの秒
RETRY_S = 3.0
# サーバーに伝える、Pi ができること
SUPPORTS = ["reverse", "segments", "long_step", "gear_segments"]
# 1 つの station を撮り直す回数の上限
MAX_RESHOOTS = 2


class Rejected(RuntimeError):
    """サーバーが 4xx を返した（中身がおかしい）。同じものを送り直しても通らない"""


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
    def __init__(self, runner, plan_buf, explore_state=None):
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
        # 止まって撮る向きの数。これの半分より少なければ撮り直す
        # （撮り直しの上限の後は 1 枚でも送る）
        ctl = self.cfg.control
        self.n_poses = len(scan_poses(ctl.explore_scan_tilts, ctl.explore_scan_pans,
                                      self.cfg.car.pan_min, self.cfg.car.pan_max))  # fmt: skip
        self._reshoots: dict[int, int] = {}  # station ごとの撮り直した回数
        # 撮り直させた station。制御プロセスが撮り始める（待つのをやめる）まで送らない
        self._reshoot_k = -1
        # explore の続きを書く所（resume.ExploreState）。explore をやめたら消す
        self.explore_state = explore_state
        self._was_explore = False

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
        if v is not None and self.explore_state is not None:
            self._note_state(v, rec)
        if v is not None and rec is not None and rec.dir is not None:
            if self._session != rec.dir.name:
                self._session, self._posted, self._moves = rec.dir.name, -1, {}
                self._reshoots, self._reshoot_k = {}, -1
            k = int(v[18])
            if self._reshoot_k >= 0 and k != self._reshoot_k:
                # 制御プロセスが撮り直し始めた。撮り終えて同じ番号でまた待ったら送る
                self._reshoot_k, self._posted = -1, -1
            if k >= 0 and k != self._posted and time.monotonic() >= self._retry_at:
                self._posted = k
                stills = sum(1 for e in list(rec.entries)
                             if e.get("station") == k and not e.get("moving"))  # fmt: skip
                few = f"止まって撮ったコマが {stills}/{self.n_poses} 枚"
                if stills < max(1, self.n_poses // 2) and self._can_reshoot(k):
                    self._reshoot(k, f"{few}（{self._why_few(k, rec)}）")
                    return
                if stills == 0:  # 撮り直しても撮れない。送っても 400 になる
                    self._give_up(k, f"{MAX_RESHOOTS} 回撮り直しても{few}（{self._why_few(k, rec)}）")
                    return
                ok = self._post(k, v, rec)
                if ok is None:  # 4xx：同じものは送り直さない
                    if self._can_reshoot(k):
                        self._reshoot(k, "サーバーに断られた")
                    else:
                        self._give_up(k, "撮り直してもサーバーに断られた")
                elif not ok:
                    # 届かない・失敗：制御プロセスは待ち続けるので、少しして同じ station を送り直す
                    self._posted, self._retry_at = -1, time.monotonic() + RETRY_S

    def _note_state(self, v, rec) -> None:
        """explore の今を explore.json に書く（起動し直したら続きから）。explore をやめたら消す"""
        from .controller import MODES
        from .resume import phase_of

        explore = MODES[int(v[14])] == "explore"
        if self._was_explore and not explore:
            self.explore_state.clear()  # 調べ終えた・時間切れ・ボタン
        self._was_explore = explore
        if explore and rec is not None and rec.dir is not None:
            phase = phase_of(v)
            # 返事を待っている station はまだ終わっていない（戻ったら撮り直して送る）
            nxt = int(v[18]) if phase == "plan" else int(v[21])
            self.explore_state.write(rec.dir.name, nxt, phase)

    def _can_reshoot(self, k: int) -> bool:
        return self._reshoots.get(k, 0) < MAX_RESHOOTS

    def _reshoot(self, k: int, why: str) -> None:
        n = self._reshoots[k] = self._reshoots.get(k, 0) + 1
        self._reshoot_k = k
        self.last = f"station {k}: {why} → 撮り直す（{n}/{MAX_RESHOOTS} 回目）"
        print(self.last, flush=True)
        self.plan_buf.write((k, PLAN_RESHOOT) + (0.0,) * (PLAN_N - 2))

    def _give_up(self, k: int, why: str) -> None:
        """送っても通らない。止まって待ち続けないよう、探索をやめる（idle）。_posted は k のまま"""
        self.last = f"station {k}: {why}。探索をやめる"
        print(self.last, flush=True)
        self.plan_buf.write((k, PLAN_DONE) + (0.0,) * (PLAN_N - 2))

    def _why_few(self, k: int, rec) -> str:
        """撮れなかった訳の手がかり：カメラ（最後のコマの古さ）・書き込みの失敗・時刻の窓に来なかった向き"""
        n = max(1, self.n_poses)
        missed = [i for i in getattr(rec, "missed", []) if i // n == k]
        last = getattr(rec, "last_offer_ns", 0)
        if last:
            cam = f"カメラの最後のコマ {(time.monotonic_ns() - last) / 1e9:.1f}s 前"
        else:
            cam = "カメラのコマが来ていない"
        return (f"{cam}、書き込みの失敗 {getattr(rec, 'write_errors', 0)} 回、"
                f"撮る前に次の向きへ替わった {len(missed)} 向き、置き場 {rec.dir}")  # fmt: skip

    def _post(self, k: int, v, rec) -> bool | None:
        """送って返事を plan に書く。返事を書けたら True、届かない・失敗（送り直せば
        通りうる）は False、サーバーが 4xx で断った（送り直しても通らない）は None"""
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
            if 400 <= r.status_code < 500:
                raise Rejected(f"{r.status_code} {r.text[:200]}")
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
        except Rejected as e:
            self.last = (f"station {k}: 地図のサーバーに断られた（{str(e)[:120]}）"
                         "。同じものは送り直さない")  # fmt: skip
            print(self.last, flush=True)
            return None
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
