"""teleop モード：ブラウザ・ゲームパッドから手で操縦する（snoo_trucar#27。仕様は
rocm_opencv_server の docs/teleop.md）。

- teleop の間は POLL_S（10 Hz）、ほかのモードでは IDLE_POLL_S ごとに
  `GET /v1/snoo/teleop/latest` を問い合わせる（ほかのモードでも、画面にモードを出し、
  頼みを受けるため）
- 受けた指令を teleop の共有メモリに書く：(受けた時刻, seq, ハンドル, 速さ, パン, チルト, ok)。
  失敗したら書かない（制御プロセスが 0.3 s で止まる。deadman）
- 問い合わせには、実際にモーターへ書いている値と、弱めた理由（limit）を付けて返す
- 画面からのモードの頼み（request_id が増えたときだけ。teleop か idle）を受ける。
  起動して初めの返事の request_id は受けたことにする（前の頼みでモードを変えない）
"""

from __future__ import annotations

import math
import threading
import time

from .control_proc import C_CLEAR_CM, C_TELEOP_LIMIT
from .controller import MODES, limit_names
from .errlog import ErrorLog

POLL_S = 0.1
IDLE_POLL_S = 1.0
TIMEOUT_S = 0.25
DEADMAN_S = 0.3
SUPPORTS = "teleop,ui_mode"


def _num(x) -> float:
    if isinstance(x, bool) or not isinstance(x, int | float) or not math.isfinite(x):
        return 0.0
    return float(x)


def ok_value(ans: dict) -> float:
    """1: 使ってよい / 0: 古い・まだ無い（deadman） / -1: stop と言った"""
    age = ans.get("age_s")
    if age is None or ans.get("stale") or float(age) > DEADMAN_S:
        return 0.0
    if ans.get("stop"):
        return -1.0
    return 1.0


class TeleopClient:
    def __init__(self, runner, buf, base_url: str, session=None):
        self.runner = runner
        self.buf = buf
        self.base = base_url.rstrip("/")
        self._session = session
        self.after = 0
        self.request_id: int | None = None  # 受けた画面の頼みの番号
        self.last = ""
        self.errors = ErrorLog("teleop", every_s=60.0)

    def http(self):
        if self._session is None:
            import requests

            self._session = requests.Session()
        return self._session

    def run(self, stop: threading.Event) -> None:
        """teleop のスレッド。1 本で順に問い合わせ、重ねない。失敗しても抜けない"""
        while not stop.is_set():
            t0 = time.monotonic()
            try:
                self.once()
            except Exception as e:  # 届かない：書かないので、制御プロセスが deadman で止まる
                self.errors(e)
            wait = POLL_S if self.runner.current_mode() == "teleop" else IDLE_POLL_S
            stop.wait(max(0.0, wait - (time.monotonic() - t0)))

    def query(self) -> dict:
        r = self.runner
        q = {"after": self.after, "mode": r.current_mode(), "supports": SUPPORTS}
        if r.battery_v is not None:
            q["battery_v"] = round(float(r.battery_v), 2)
        if r.last_us is not None:
            q["sonar_cm"] = round(float(r.last_us), 1)
        v = r.last_cmd
        if v is not None:
            if float(v[C_CLEAR_CM]) >= 0:
                q["clear_cm"] = round(float(v[C_CLEAR_CM]), 1)
            # 実際にモーターへ書いている値（電池で走らないときは 0。runner._actuate_loop と同じ）
            q["steer_deg"] = round(float(v[1]), 1)
            q["speed"] = round(float(v[2]) if r.battery[1] else 0.0, 1)
            q["pan_deg"] = round(float(v[3]), 1)
            q["tilt_deg"] = round(float(v[4]), 1)
            names = limit_names(int(v[C_TELEOP_LIMIT]))
            if not r.battery[1] and abs(float(v[2])) > 0 and "battery" not in names:
                names.append("battery")
            if names:
                q["limit"] = ",".join(names)
        return q

    def once(self) -> None:
        r = self.http().get(f"{self.base}/v1/snoo/teleop/latest", params=self.query(),
                            timeout=TIMEOUT_S)  # fmt: skip
        if r.status_code != 200:
            raise RuntimeError(f"teleop/latest {r.status_code}")
        ans = r.json()
        t_recv = time.monotonic_ns()
        c = ans.get("command") or {}
        seq = int(ans.get("seq") or 0)
        ok = ok_value(ans)
        self.buf.write((t_recv, seq, _num(c.get("steer_deg")), _num(c.get("speed")),
                        _num(c.get("pan_deg")), _num(c.get("tilt_deg")), ok))  # fmt: skip
        self.after = max(self.after, seq)
        self.last = f"teleop: seq={seq} ok={ok:+.0f} {ans.get('reason') or ''}"
        self._request(ans)

    def _request(self, ans: dict) -> None:
        rid = int(ans.get("request_id") or 0)
        if self.request_id is None:
            self.request_id = rid  # 起動する前の頼みでは変えない
            return
        if rid <= self.request_id:
            return
        self.request_id = rid
        mode = ans.get("request_mode")
        if mode in ("teleop", "idle") and mode in MODES:
            print(f"画面から {mode} を頼まれた（{rid}）", flush=True)
            self.runner.set_mode(mode)
