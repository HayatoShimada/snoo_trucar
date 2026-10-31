"""声（snoo_trucar#29）。Robot HAT のスピーカーで、地図のサーバーの声（say/next）と、
出来事の短い知らせを鳴らす。

- サーバーの声：`GET /v1/snoo/agent/say/next?since=<id>` を SAY_POLL_S ごとに見て、
  wav を取って鳴らす（仕様は rocm_opencv_server の docs/agent_robot.md）
- 出来事：explore を始めた・終えた、詰まった、電池が少ない、snoo を見つけた。
  文は `POST say`（queue: false）で wav にしてもらう（Pi では声を作らない）
- 鳴らし方：GPIO 20 を High（アンプをオン）→ `aplay -q -D plughw:sndrpihifiberry,0 <wav>` → Low。
  カードは番号でなく名前で指す。aplay・pinctrl が無い・失敗したら黙って続ける
- このスレッドだけで鳴らす（aplay は別のプロセス）。制御のループ・止める仕組みは待たせない
- snoo を怖がらせない：guard が OK でない・snoo が quiet_near_cm より近くに見えるときは鳴らさない。
  音の大きさは gain で wav を小さくする（ALSA の音量は変えない）。1 分の回数に上限。
  `[voice] enabled = false` で切る
"""

from __future__ import annotations

import io
import subprocess
import threading
import time
import wave
from collections import deque
from dataclasses import dataclass
from pathlib import Path

from .errlog import ErrorLog

SAY_POLL_S = 1.5  # サーバーの声を見る間隔
OBSERVE_S = 0.2  # 出来事を見る間隔
EVENT_REPEAT_S = 60.0  # 同じ知らせはこの間に 2 回言わない
SNOO_AGAIN_S = 60.0  # snoo をこれだけ見失ってから見つけたら「見つけた」と言う


@dataclass
class VoiceConfig:
    enabled: bool = True
    # 出来事を話す（false ならサーバーの声だけ鳴らす）
    events: bool = True
    # 1 分に鳴らす回数の上限（サーバーの声と出来事を合わせて）
    max_per_min: int = 6
    # wav の大きさに掛ける（0..1）。ALSA の音量（amixer）は変えない
    gain: float = 0.6
    # snoo がこれより近くに見えるときは鳴らさない（cm）
    quiet_near_cm: float = 150.0
    device: str = "plughw:sndrpihifiberry,0"
    amp_gpio: int = 20
    # wav を置く所（tmpfs。SD に書かない）
    tmp_dir: str = "/dev/shm"


EXPLORE_DONE = {
    "no_frontier": "地図を全部埋めました。探索を終えます。",
    "low_battery": "電池が少ないので、探索を終えます。",
}


def scale_wav(data: bytes, gain: float) -> bytes:
    """16 ビットの wav の大きさを gain 倍にする。読めない形ならそのまま"""
    if gain >= 0.999:
        return data
    try:
        import numpy as np

        with wave.open(io.BytesIO(data)) as w:
            params, frames = w.getparams(), w.readframes(w.getnframes())
        if params.sampwidth != 2:
            return data
        a = (np.frombuffer(frames, dtype="<i2").astype(np.float32) * max(0.0, gain)).astype("<i2")
        out = io.BytesIO()
        with wave.open(out, "wb") as w:
            w.setparams(params)
            w.writeframes(a.tobytes())
        return out.getvalue()
    except Exception:
        return data


def aplay(path: Path, cfg: VoiceConfig) -> None:
    """アンプをオンにして鳴らし、オフに戻す。無い・失敗しても例外を出さない（errors に残す）"""
    gpio = str(cfg.amp_gpio)
    try:
        subprocess.run(["pinctrl", "set", gpio, "op", "dh"], capture_output=True, timeout=5)
        subprocess.run(["aplay", "-q", "-D", cfg.device, str(path)],
                       capture_output=True, timeout=60)  # fmt: skip
    finally:
        subprocess.run(["pinctrl", "set", gpio, "op", "dl"], capture_output=True, timeout=5)


class Voice:
    """runner の様子を見て知らせを決め、サーバーの声と一緒に順に鳴らす。

    play(wav_bytes) と tts(text) -> wav_bytes は差し替えられる（テスト）"""

    def __init__(self, runner, cfg: VoiceConfig, base_url: str, play=None, tts=None,
                 clock=time.monotonic):  # fmt: skip
        self.runner = runner
        self.cfg = cfg
        self.base = base_url.rstrip("/")
        self._play = play or self._aplay
        self._tts = tts or self._server_tts
        self.clock = clock
        self.since = 0  # 最後に鳴らした say の id
        self.pending: deque[str] = deque(maxlen=3)  # 話す出来事の文（古いものから落とす）
        self.played: deque[tuple[float, str]] = deque()  # (時刻, 文)。1 分の上限を見る
        self._said: dict[str, float] = {}  # 出来事の文ごとに、最後に言った時刻
        self._mode: str | None = None
        self._step: tuple | None = None
        self._battery_ok = True
        self._seen_at: float | None = None  # snoo を最後に見た時刻
        self._next_poll = 0.0
        self.done_reason: str | None = None  # 地図のサーバーの done の理由（station.py が入れる）
        # 届かない・aplay が無いのは続くので、たまにだけ出す
        self.errors = ErrorLog("声", every_s=300.0)

    # --- 出来事 ---

    def event(self, text: str) -> None:
        """出来事の知らせを並べる。同じ文は EVENT_REPEAT_S の間に 2 回言わない"""
        if not self.cfg.events:
            return
        now = self.clock()
        if now - self._said.get(text, -1e9) < EVENT_REPEAT_S:
            return
        self._said[text] = now
        self.pending.append(text)

    def observe(self) -> None:
        """runner の様子の変わり目から出来事を拾う"""
        from .controller import MODES, STOP_REASONS, TRACK

        v = self.runner.last_cmd
        if v is None:
            return
        now = self.clock()
        mode = MODES[int(v[14])]
        if self._mode is not None and mode != self._mode:
            if mode == "explore":
                self.done_reason = None
                self.event("探索を始めます。")
            elif self._mode == "explore":
                why = self.done_reason
                if why:
                    self.event(EXPLORE_DONE.get(why, "探索が終わりました。"))
                else:
                    self.event("探索をやめました。")
        self._mode = mode
        if len(v) > 20:
            step = (float(v[19]), int(v[20]))
            if self._step is not None and step != self._step and STOP_REASONS[step[1]] == "stuck":
                self.event("引っかかって止まりました。")
            self._step = step
        ok = bool(self.runner.battery[1])
        volts = getattr(self.runner, "battery_v", None)
        if self._battery_ok and not ok and volts is not None and volts >= 1.0:
            # 1 V より低いのは電池が入っていない（鳴らせない）
            self.event("電池が少ないので止まります。")
        self._battery_ok = ok
        if int(v[5]) == TRACK:
            if self._seen_at is None or now - self._seen_at >= SNOO_AGAIN_S:
                if float(v[8]) >= self.cfg.quiet_near_cm:
                    self.event("スヌーを見つけました。")
            self._seen_at = now

    # --- 鳴らす ---

    def quiet_now(self) -> bool:
        """snoo を驚かせないよう、今は鳴らさない"""
        from .guard import OK

        v = self.runner.last_cmd
        if v is None:
            return False
        near = bool(v[6]) and float(v[9]) < 1.0 and float(v[8]) < self.cfg.quiet_near_cm
        return int(v[17]) != OK or near

    def _room(self) -> bool:
        now = self.clock()
        while self.played and now - self.played[0][0] > 60.0:
            self.played.popleft()
        return len(self.played) < self.cfg.max_per_min

    def once(self) -> None:
        """出来事を拾い、鳴らせるなら 1 つ鳴らす（出来事を先に、次にサーバーの声）"""
        self.observe()
        if self.quiet_now() or not self._room():
            return
        if self.pending:
            text = self.pending.popleft()
            self._say(text, self._tts(text))
            return
        now = self.clock()
        if now >= self._next_poll:
            self._next_poll = now + SAY_POLL_S
            self._server_next()

    def _say(self, text: str, wav: bytes | None) -> None:
        if not wav:
            return
        self.played.append((self.clock(), text))
        self._play(scale_wav(wav, self.cfg.gain))

    def _server_next(self) -> None:
        import requests

        r = requests.get(f"{self.base}/v1/snoo/agent/say/next", params={"since": self.since},
                         timeout=2.0)  # fmt: skip
        if r.status_code != 200:
            return
        say = r.json().get("say")
        if not say:
            return
        sid = int(say["id"])
        url = str(say.get("url") or f"/v1/snoo/agent/say/{say['wav']}")
        w = requests.get(self.base + url, timeout=5.0)
        self.since = sid  # 取れなくても次へ（同じものを何度も取りに行かない）
        if w.status_code == 200:
            self._say(str(say.get("text", "")), w.content)

    def _server_tts(self, text: str) -> bytes | None:
        import requests

        r = requests.post(f"{self.base}/v1/snoo/agent/say", json={"text": text, "queue": False},
                          timeout=10.0)  # fmt: skip
        return r.content if r.status_code == 200 else None

    def _aplay(self, wav: bytes) -> None:
        path = Path(self.cfg.tmp_dir) / "snoo_say.wav"
        path.write_bytes(wav)
        try:
            aplay(path, self.cfg)
        finally:
            path.unlink(missing_ok=True)

    def run(self, stop: threading.Event) -> None:
        """声のスレッド。失敗しても抜けない（抜けると runner がスレッドが死んだとみなして止まる）"""
        while not stop.is_set():
            try:
                self.once()
            except Exception as e:  # サーバーに届かない・aplay が無い：黙って続ける
                self.errors(e)
            stop.wait(OBSERVE_S)


def base_url(map_url: str) -> str:
    """地図のサーバーの scheme・host・port（config.server.map_url から）"""
    from urllib.parse import urlsplit

    u = urlsplit(map_url)
    return f"{u.scheme}://{u.netloc}"
