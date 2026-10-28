"""Robot HAT の USR ボタン（GPIO25、押すと Low）と LED（GPIO26）。

USR ボタン:
  1 回 → watch（snoo を探して、カメラで追う）
  2 回 → keep（距離を保つ。近づいたり離れたりする）
  3 回 → map（その場でカメラを振って部屋を撮る）
  長押し（1.5 秒）→ idle（止まって待つ）

LED は今の行動を点滅の回数で出す（idle は 2 秒に 1 回短く光る）。
RST ボタンはマイコンのリセットなので使わない。
"""

from __future__ import annotations

import threading
import time

BUTTON_GPIO = 25
LED_GPIO = 26
CLICK_GAP_S = 0.6  # これだけ押されなければ、押した回数を確定する
LONG_PRESS_S = 1.5
CLICK_MODES = {1: "watch", 2: "keep", 3: "map"}
LED_BLINKS = {"watch": 1, "keep": 2, "map": 3, "follow": 4}


class ClickCounter:
    """押した・離したを時刻と一緒に入れると、確定したときに "click:n" か "long" を返す"""

    def __init__(self):
        self._down_at: float | None = None
        self._count = 0
        self._last_up = 0.0
        self._long_sent = False

    def feed(self, pressed: bool, t: float) -> str | None:
        if pressed:
            if self._down_at is None:
                self._down_at = t
                self._long_sent = False
            elif not self._long_sent and t - self._down_at >= LONG_PRESS_S:
                self._long_sent = True
                self._count = 0
                return "long"
            return None
        if self._down_at is not None:
            if not self._long_sent:
                self._count += 1
                self._last_up = t
            self._down_at = None
            return None
        if self._count and t - self._last_up >= CLICK_GAP_S:
            n, self._count = self._count, 0
            return f"click:{n}"
        return None


def event_to_mode(event: str) -> str | None:
    if event == "long":
        return "idle"
    n = int(event.split(":")[1])
    return CLICK_MODES.get(n)


class Panel:
    """ボタンを読んで on_mode(mode) を呼び、LED を光らせる"""

    def __init__(self, on_mode, get_mode):
        from gpiozero import LED, Button

        self.button = Button(BUTTON_GPIO, pull_up=True, bounce_time=None)
        self.led = LED(LED_GPIO)
        self.on_mode = on_mode
        self.get_mode = get_mode

    def run(self, stop: threading.Event) -> None:
        counter = ClickCounter()
        threading.Thread(target=self._led_loop, args=(stop,), daemon=True).start()
        while not stop.is_set():
            event = counter.feed(self.button.is_pressed, time.monotonic())
            if event:
                mode = event_to_mode(event)
                print(f"USR ボタン: {event} → {mode}", flush=True)
                if mode:
                    self.on_mode(mode)
            time.sleep(0.01)
        self.led.off()

    def _led_loop(self, stop: threading.Event) -> None:
        while not stop.is_set():
            n = LED_BLINKS.get(self.get_mode(), 0)
            if n == 0:
                self.led.on()
                time.sleep(0.05)
                self.led.off()
                stop.wait(2.0)
                continue
            for _ in range(n):
                self.led.on()
                time.sleep(0.15)
                self.led.off()
                time.sleep(0.2)
            stop.wait(1.0)
