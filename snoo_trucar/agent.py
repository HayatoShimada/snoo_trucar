"""agent モード：エージェントの指令（look・move・scan・stop）を地図のサーバーに問い合わせて実行する
（snoo_trucar#28。仕様は rocm_opencv_server の docs/agent_robot.md）。

- POLL_S ごとに `GET /v1/snoo/agent/commands/next?after=<id>&mode=&battery_v=&sonar_cm=`
  `&running=&supports=` を問い合わせる（どのモードでも）。通るたびに、その時刻を agent の
  共有メモリに書く。制御プロセスは 3 s（AGENT_HEARTBEAT_NS）通らなければ、動いている指令を
  止める（heartbeat）
- 受けたらすぐ accepted を書き、実行するなら共有メモリに書く。実行するのは mode が agent の
  ときだけ。ほかは refused（not_agent_mode・battery_low・guard・busy・bad_params）
- 制御プロセスの結果（cmd の C_AGENT_ID から）を見て、running → done・failed・cancelled を書く。
  書けなかった結果は、次に通ったときに順に書き直す
- stop はいつでも：動いている指令を cancelled（stop）にして止める。USR の長押しも
  cancelled（button）
- scan の station の prev_step は、前の scan からの agent の move（同じ向きだけなら segments に
  並べ、向きが混じった・自分で動いた（guard で下がった）なら {"stop_reason": "agent"} だけ）
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque

from .control_proc import AGENT_N, C_AGENT_ID, C_EX_STATION
from .controller import (
    A_LOOK,
    A_MOVE,
    A_SCAN,
    A_STOP,
    AGENT_REASONS,
    AS_CANCELLED,
    AS_DONE,
    AS_FAILED,
    AS_RUNNING,
    MODES,
)
from .errlog import ErrorLog
from .guard import OK

POLL_S = 0.5
POLL_TIMEOUT_S = 1.0
DISPATCH_S = 1.0  # 書いてからこれだけ制御プロセスが受けなければ、受けなかったとみなす
SUPPORTS = ["look", "move", "scan", "stop"]
KINDS = {"look": A_LOOK, "move": A_MOVE, "scan": A_SCAN, "stop": A_STOP}


def _num(x) -> float | None:
    if isinstance(x, bool) or not isinstance(x, int | float) or not math.isfinite(x):
        return None
    return float(x)


class AgentClient:
    def __init__(self, runner, buf, base_url: str, supports=None, session=None):
        self.runner = runner
        self.buf = buf
        self.base = base_url.rstrip("/")
        self.supports = list(supports or SUPPORTS)
        self._session = session  # requests.Session か同じ形のもの（テストで差し替え）
        self.after = 0
        self.running: dict | None = None  # 制御プロセスに渡した指令
        self.unsent: deque[tuple[int, dict]] = deque(maxlen=50)  # 書けなかった結果
        self.last_ok_ns = 0
        self._now: tuple[float, float] | None = None  # (サーバーの now, そのときの monotonic)
        self.button_at: float | None = None  # USR の長押しをしたときのサーバーの時刻
        self.moves: list[dict] = []  # 前の scan からの move（prev_step）
        self.self_moved = False  # 指令の無い間に車が動いた（guard が下がった）
        self.last = ""  # 状態の行に出す
        self._row = [0.0] * AGENT_N
        self._lock = threading.Lock()
        self._mode: str | None = None
        self.errors = ErrorLog("agent", every_s=60.0)

    # --- 問い合わせ ---

    def run(self, stop: threading.Event) -> None:
        """agent のスレッド。失敗しても抜けない（抜けると runner が止まる）"""
        while not stop.is_set():
            t0 = time.monotonic()
            try:
                self.once()
            except Exception as e:  # 届かない：制御プロセスは heartbeat で止まる
                self.errors(e)
            stop.wait(max(0.05, POLL_S - (time.monotonic() - t0)))

    def http(self):
        if self._session is None:
            import requests

            self._session = requests.Session()
        return self._session

    def once(self) -> None:
        with self._lock:
            self._watch()
        r = self.http().get(f"{self.base}/v1/snoo/agent/commands/next", params=self._query(),
                            timeout=POLL_TIMEOUT_S)  # fmt: skip
        if r.status_code != 200:
            raise RuntimeError(f"commands/next {r.status_code}")
        ans = r.json()
        with self._lock:
            self.last_ok_ns = time.monotonic_ns()
            self._now = (float(ans.get("now", 0.0)), time.monotonic())
            self._write()  # 通った（heartbeat）
            self._flush()
            c = ans.get("command")
            if c:
                self._handle(c, float(ans.get("now", 0.0)))

    def _query(self) -> dict:
        r = self.runner
        q = {"after": self.after, "mode": r.current_mode(), "supports": ",".join(self.supports)}
        if r.battery_v is not None:
            q["battery_v"] = round(float(r.battery_v), 2)
        if r.last_us is not None:
            q["sonar_cm"] = round(float(r.last_us), 1)
        if self.running is not None:
            q["running"] = self.running["id"]
        return q

    def _write(self, cid: int | None = None, kind: int = 0, params=(math.nan,) * 4) -> None:
        """agent の共有メモリに書く。cid が None なら、問い合わせが通った時刻だけ新しくする"""
        if cid is not None:
            self._row[1:] = [float(cid), float(kind), *(float(x) for x in params)]
        self._row[0] = float(time.monotonic_ns())
        self.buf.write(self._row)

    # --- 結果 ---

    def result(self, cid: int, status: str, reason: str | None = None,
               detail: dict | None = None) -> None:  # fmt: skip
        self.unsent.append((cid, {"status": status, "reason": reason, "detail": detail or {}}))
        self._flush()

    def _flush(self) -> None:
        while self.unsent:
            cid, body = self.unsent[0]
            try:
                r = self.http().post(f"{self.base}/v1/snoo/agent/command/{cid}/result", json=body,
                                     timeout=POLL_TIMEOUT_S)  # fmt: skip
            except Exception as e:
                self.errors(e)
                return  # 次に通ったときに書き直す
            if r.status_code >= 500:
                return
            self.unsent.popleft()  # 4xx（無い指令など）は書き直さない

    # --- 指令 ---

    def _handle(self, c: dict, now: float) -> None:
        cid = int(c["id"])
        if cid <= self.after:
            return
        self.after = cid
        kind = str(c.get("kind"))
        self.result(cid, "accepted")
        if float(c.get("expires_at", math.inf)) < now:
            return self._refuse(cid, "expired", None)
        if kind == "stop":
            self._write(cid, A_STOP)
            if self.running is not None:
                self.result(self.running["id"], "cancelled", "stop", self._detail(self.running))
                self.running = None
            self.last = f"agent: stop {cid}"
            return self.result(cid, "done")
        v = self.runner.last_cmd
        if v is None or MODES[int(v[14])] != "agent":
            return self._refuse(cid, "refused", "not_agent_mode")
        if self.button_at is not None and float(c.get("t", 0.0)) < self.button_at:
            return self._refuse(cid, "cancelled", "button")  # 長押しより前に並んだもの
        if self.running is not None:
            return self._refuse(cid, "refused", "busy")
        if kind not in KINDS:
            return self._refuse(cid, "refused", "bad_params")
        params = (math.nan,) * 4
        if kind == "look":
            pan, tilt = c.get("pan_deg"), c.get("tilt_deg")
            if (pan is not None and _num(pan) is None) or (tilt is not None and _num(tilt) is None):
                return self._refuse(cid, "refused", "bad_params")
            params = (math.nan if pan is None else pan, math.nan if tilt is None else tilt,
                      math.nan, math.nan)  # fmt: skip
        elif kind == "move":
            steer, cm = _num(c.get("steer_deg", 0.0)), _num(c.get("distance_cm"))
            speed = _num(c.get("speed"))
            if steer is None or cm is None or speed is None or cm <= 0 or speed <= 0:
                return self._refuse(cid, "refused", "bad_params")
            if not self.runner.battery[1]:
                return self._refuse(cid, "refused", "battery_low")
            if int(v[17]) != OK:
                return self._refuse(cid, "refused", "guard")
            params = (steer, cm, speed, 1.0 if c.get("reverse") else 0.0)
        self._write(cid, KINDS[kind], params)
        self.running = {"id": cid, "kind": kind, "cmd": c, "t_send": time.monotonic(),
                        "t_start_ns": time.monotonic_ns(), "posted_running": False}  # fmt: skip
        self.last = f"agent: {kind} {cid}"

    def _refuse(self, cid: int, status: str, reason: str | None) -> None:
        self.last = f"agent: {cid} は {status}（{reason}）"
        self.result(cid, status, reason)

    def _watch(self) -> None:
        """制御プロセスの結果を見て書く"""
        v = self.runner.last_cmd
        if v is None:
            return
        mode = MODES[int(v[14])]
        if mode != self._mode and mode == "agent":
            self.prev_step_sent()  # agent に入った：新しいセッション（station 0 は prev_step 無し）
        self._mode = mode
        run = self.running
        if run is None:
            if abs(float(v[2])) > 0.5:
                self.self_moved = True  # guard が下がった など。prev_step の長さが分からない
            return
        aid, st, reason = int(v[C_AGENT_ID]), int(v[C_AGENT_ID + 1]), int(v[C_AGENT_ID + 2])
        if aid != run["id"]:
            if time.monotonic() - run["t_send"] > DISPATCH_S:
                # 制御プロセスが受けなかった（その間に agent をやめた など）
                self.running = None
                why = "not_agent_mode" if MODES[int(v[14])] != "agent" else "busy"
                self._refuse(run["id"], "refused", why)
            return
        if st == AS_RUNNING:
            if not run["posted_running"]:
                run["posted_running"] = True
                self.result(run["id"], "running")
            return
        status = {AS_DONE: "done", AS_FAILED: "failed", AS_CANCELLED: "cancelled"}.get(st)
        if status is None:
            return
        why = AGENT_REASONS[reason] if 0 <= reason < len(AGENT_REASONS) else ""
        detail = self._detail(run, v)
        if run["kind"] == "move":
            self.moves.append(detail)
        self.running = None
        self.last = f"agent: {run['kind']} {run['id']} は {status}（{why}）"
        self.result(run["id"], status, None if status == "done" else why, detail)

    def _detail(self, run: dict, v=None) -> dict:
        v = self.runner.last_cmd if v is None else v
        c = run["cmd"]
        if run["kind"] == "move":
            reason = int(v[C_AGENT_ID + 2])
            why = AGENT_REASONS[reason] if 0 <= reason < len(AGENT_REASONS) else ""
            return {"moved_cm": round(float(v[C_AGENT_ID + 3]), 1), "stop_reason": why,
                    "seconds": round(float(v[C_AGENT_ID + 4]), 2),
                    "steer_deg": c.get("steer_deg"), "speed": c.get("speed"),
                    "reverse": bool(c.get("reverse")), "t_start_ns": run["t_start_ns"],
                    "t_end_ns": time.monotonic_ns()}  # fmt: skip
        if run["kind"] == "look":
            return {"pan_deg": round(float(v[3]), 1), "tilt_deg": round(float(v[4]), 1)}
        if run["kind"] == "scan":
            st = getattr(self.runner, "station", None)
            k = int(v[C_EX_STATION]) - 1
            rec = getattr(self.runner, "recorder", None)
            return {"session_id": rec.dir.name if rec is not None and rec.dir is not None else None,
                    "station_id": k,
                    "reply_action": st.replies.get(k) if st is not None else None}  # fmt: skip
        return {}

    # --- ボタン・prev_step ---

    def on_button(self) -> None:
        """USR の長押し：受けている指令を取り消し、長押しより前に並んだものは実行しない"""
        with self._lock:
            if self._now is not None:
                srv, mono = self._now
                self.button_at = srv + (time.monotonic() - mono)
            if self.running is not None:
                self.result(self.running["id"], "cancelled", "button", self._detail(self.running))
                self.running = None

    def prev_step(self) -> dict | None:
        """agent の scan の station に付ける prev_step（前の scan からの move）"""
        moves = list(self.moves)
        if self.self_moved or len({m["reverse"] for m in moves}) > 1:
            return {"stop_reason": "agent"}  # 長さの分からない歩（地図のサーバーは地図に足さない）
        if not moves:
            return {"steer_deg": 0.0, "speed": 0.0, "reverse": False, "seconds": 0.0,
                    "segments": [{"steer_deg": 0.0, "distance_cm": 0.0, "reverse": False}],
                    "est_distance_cm": 0.0, "stop_reason": "done"}  # fmt: skip
        segs = [{"steer_deg": float(m["steer_deg"] or 0.0), "distance_cm": m["moved_cm"],
                 "reverse": m["reverse"]} for m in moves]  # fmt: skip
        return {"steer_deg": segs[-1]["steer_deg"], "speed": float(moves[-1]["speed"] or 0.0),
                "reverse": moves[0]["reverse"], "segments": segs,
                "seconds": round(sum(m["seconds"] for m in moves), 2),
                "est_distance_cm": round(sum(m["moved_cm"] for m in moves), 1),
                "stop_reason": moves[-1]["stop_reason"]}  # fmt: skip

    def prev_step_sent(self) -> None:
        self.moves, self.self_moved = [], False
