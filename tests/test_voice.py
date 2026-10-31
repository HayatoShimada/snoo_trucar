"""声：サーバーの say/next と出来事の知らせを順に鳴らす・上限・鳴らさない時（snoo_trucar#29）"""

import io
import threading
import wave
from types import SimpleNamespace

import requests

from snoo_trucar.control_proc import CMD_N
from snoo_trucar.controller import MODES, STOP_REASONS, TRACK
from snoo_trucar.guard import HOLD
from snoo_trucar.voice import EVENT_REPEAT_S, Voice, VoiceConfig, aplay, base_url, scale_wav


def _wav(value=1000, n=160):
    out = io.BytesIO()
    with wave.open(out, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(16000)
        w.writeframes(int(value).to_bytes(2, "little", signed=True) * n)
    return out.getvalue()


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def _voice(monkeypatch, says=(), **cfg):
    v = [0.0] * CMD_N
    v[14] = MODES.index("idle")
    runner = SimpleNamespace(last_cmd=v, battery=("電池=7.8V", True), battery_v=7.8)
    played, spoken = [], []
    clock = Clock()
    queue = list(says)

    def get(url, params=None, timeout=None):
        if url.endswith("/say/next"):
            nxt = next((s for s in queue if s["id"] > params["since"]), None)
            return SimpleNamespace(status_code=200, json=lambda: {"say": nxt, "now": 0})
        return SimpleNamespace(status_code=200, content=_wav())

    monkeypatch.setattr(requests, "get", get)

    def tts(text):
        spoken.append(text)
        return _wav()

    voice = Voice(runner, VoiceConfig(**cfg), "http://srv:8720", play=played.append, tts=tts,
                  clock=clock)  # fmt: skip
    return voice, v, runner, played, spoken, clock


def _tick(voice, clock, n=1, dt=2.0):
    for _ in range(n):
        voice.once()
        clock.t += dt


def test_events_are_spoken_in_order_then_server_says(monkeypatch):
    says = [{"id": 1, "text": "ソファを見に行くよ", "url": "/v1/snoo/agent/say/a.wav"}]
    voice, v, runner, played, spoken, clock = _voice(monkeypatch, says)
    _tick(voice, clock)  # 初めの様子を覚える（ここでは知らせない）。サーバーの声を 1 つ鳴らす
    assert spoken == [] and len(played) == 1 and voice.since == 1
    v[14] = MODES.index("explore")
    _tick(voice, clock)
    v[19], v[20] = 1.2, STOP_REASONS.index("stuck")
    _tick(voice, clock)
    voice.done_reason = "no_frontier"
    v[14] = MODES.index("idle")
    _tick(voice, clock, 3)
    assert spoken == ["探索を始めます。", "引っかかって止まりました。",
                      "地図を全部埋めました。探索を終えます。"]  # fmt: skip
    assert len(played) == 4


def test_battery_low_and_snoo_found(monkeypatch):
    voice, v, runner, played, spoken, clock = _voice(monkeypatch)
    _tick(voice, clock)
    runner.battery, runner.battery_v = ("電池=6.5V（少ないので走らない）", False), 6.5
    _tick(voice, clock)
    v[5], v[8] = TRACK, 250.0  # 遠くに見つけた
    _tick(voice, clock)
    _tick(voice, clock)  # 見え続けている間は繰り返さない
    assert spoken == ["電池が少ないので止まります。", "スヌーを見つけました。"]


def test_no_sound_when_snoo_is_near_or_guard_is_not_ok(monkeypatch):
    says = [{"id": 1, "text": "こんにちは", "url": "/v1/snoo/agent/say/a.wav"}]
    voice, v, runner, played, spoken, clock = _voice(monkeypatch, says)
    v[17] = HOLD  # guard が待っている（snoo がストレス）
    _tick(voice, clock, 3)
    assert played == [] and voice.since == 0  # 飛ばさずに待つ（期限はサーバーが見る）
    v[17] = 0
    v[6], v[8], v[9] = 1.0, 80.0, 0.1  # snoo が 80 cm に見えている
    _tick(voice, clock, 3)
    assert played == []
    v[6] = 0.0
    _tick(voice, clock)
    assert len(played) == 1


def test_per_minute_limit_and_repeat(monkeypatch):
    voice, v, runner, played, spoken, clock = _voice(monkeypatch, max_per_min=2)
    for i in range(5):
        voice.event(f"知らせ {i}")
    voice.event("知らせ 0")  # 同じ文は繰り返さない
    _tick(voice, clock, 5, dt=1.0)
    assert len(played) == 2
    clock.t += 61.0
    _tick(voice, clock, 5, dt=1.0)
    assert len(played) == 3  # 並べるのは 3 つまで（古いものから落とす）
    clock.t += EVENT_REPEAT_S
    voice.event("知らせ 0")
    assert "知らせ 0" in voice.pending


def test_events_can_be_turned_off(monkeypatch):
    voice, v, runner, played, spoken, clock = _voice(monkeypatch, events=False)
    _tick(voice, clock)
    v[14] = MODES.index("explore")
    _tick(voice, clock, 2)
    assert spoken == []


def test_aplay_turns_the_amp_on_and_off_and_survives_missing_tools(monkeypatch, tmp_path):
    calls = []

    def run(cmd, **kw):
        calls.append(cmd[:2] + cmd[-2:] if cmd[0] == "aplay" else cmd)
        if cmd[0] == "aplay":
            raise FileNotFoundError("aplay")
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr("snoo_trucar.voice.subprocess.run", run)
    try:
        aplay(tmp_path / "x.wav", VoiceConfig())
    except FileNotFoundError:
        pass
    assert calls[0] == ["pinctrl", "set", "20", "op", "dh"]
    assert calls[1] == ["aplay", "-q", "plughw:sndrpihifiberry,0", str(tmp_path / "x.wav")]
    assert calls[2] == ["pinctrl", "set", "20", "op", "dl"]  # 失敗してもオフに戻す
    # 声のスレッドは失敗しても抜けない（抜けると runner が止まる）
    voice, v, runner, played, spoken, clock = _voice(monkeypatch)
    voice._play = lambda wav: aplay(tmp_path / "x.wav", VoiceConfig())
    voice.event("テスト")
    stop = threading.Event()
    t = threading.Thread(target=voice.run, args=(stop,))
    t.start()
    t.join(0.5)
    assert t.is_alive()
    stop.set()
    t.join(1.0)


def test_scale_wav_and_base_url():
    loud = _wav(10000)
    with wave.open(io.BytesIO(scale_wav(loud, 0.5))) as w:
        assert int.from_bytes(w.readframes(1), "little", signed=True) == 5000
    assert scale_wav(b"not a wav", 0.5) == b"not a wav"
    assert base_url("http://192.168.0.248:8720/v1/snoo/map/station") == "http://192.168.0.248:8720"
