"""全体の組み立て。

- 制御プロセス（control_proc、1 kHz）
- メインプロセス: カメラ → サーバー（link）、超音波、ハードへの書き込み（50 Hz）、USR ボタンと LED、
  map のときの撮影、状態の表示
"""

from __future__ import annotations

import multiprocessing as mp
import os
import threading
import time

from . import control_proc
from pathlib import Path

from .config import Config
from .controller import MODES, STATE_NAMES
from .guard import GUARD_NAMES, OK
from .shm import SeqBuffer
from .trace import Trace

CMD_STALE_NS = 100_000_000  # 制御プロセスの出力がこれより古ければ止める
BATTERY_NONE_V = 1.0  # これより低ければ電池が入っていない・スイッチが切れている


def battery_state(volts: float | None, low_v: float) -> tuple[str, bool]:
    """(状態の行に出す文字, 走ってよいか)"""
    if volts is None:
        return "電池=?", True  # 測れないときは止めない（測る仕組みの不具合で走れなくならないように）
    if volts < BATTERY_NONE_V:
        return f"電池={volts:.1f}V（入っていない・スイッチが切れている）", False
    if volts < low_v:
        return f"電池={volts:.1f}V（少ないので走らない）", False
    return f"電池={volts:.1f}V", True


class Runner:
    def __init__(self, cfg: Config, sim: bool = False, seconds: float | None = None,
                 quiet: bool = False, seed: int = 0, buttons: bool = True,
                 camera_file: str | None = None):  # fmt: skip
        self.cfg = cfg
        self.sim = sim
        self.seconds = seconds
        self.quiet = quiet
        self.seed = seed
        self.buttons = buttons
        self.camera_file = camera_file
        self.mode_buf: SeqBuffer | None = None
        self.last_us: float | None = None
        self.stop = threading.Event()
        self.history: list[tuple] = []  # (t, state, bearing_est, dist_est, bearing_true, dist_true)
        self.world = None
        self.link = None
        self.camera = None
        self.last_cmd = None
        self.battery: tuple[str, bool] = ("電池=?", True)
        self.trace: Trace | None = None

    def run(self) -> None:
        cfg = self.cfg
        meas = SeqBuffer(control_proc.MEAS_N)
        us = SeqBuffer(control_proc.US_N)
        cmd = SeqBuffer(control_proc.CMD_N)
        self.mode_buf = mode = SeqBuffer(control_proc.MODE_N)
        self.set_mode(cfg.control.mode)
        ctx = mp.get_context("fork")
        proc_stop = ctx.Event()
        proc = ctx.Process(
            target=control_proc.run,
            args=(cfg, meas.name, us.name, mode.name, cmd.name, proc_stop),
            name="snoo-control",
            daemon=True,
        )
        proc.start()  # スレッドを作る前に fork する
        if cfg.control.cpu >= 0:
            # 制御プロセスの CPU を空けておく。60 fps の JPEG 化やカメラのスレッドが同じ CPU に載ると、
            # 1 kHz の周期が 1 ms 以上遅れることがある。この後に作るスレッドもこれを引き継ぐ
            try:
                os.sched_setaffinity(0, set(range(os.cpu_count() or 1)) - {cfg.control.cpu})
            except OSError:
                pass

        threads: list[threading.Thread] = []
        camera = None
        if self.sim:
            from .hal import FakeCar
            from .sim import SimWorld

            car = FakeCar(cfg.car)
            self.world = SimWorld(cfg, car, meas, seed=self.seed)
            threads.append(threading.Thread(target=self.world.run, args=(self.stop,)))
        else:
            from .camera import Camera, FileCamera
            from .hal import Car
            from .link import Link, camera_loop
            from .mapping import MapRecorder

            car = Car(cfg.car)
            if self.camera_file:
                camera = FileCamera(cfg.camera, self.camera_file)
            else:
                camera = self.camera = Camera(cfg.camera)
            trace_path = None
            if cfg.trace_dir:
                trace_path = Path(cfg.trace_dir).expanduser() / time.strftime("trace_%Y%m%d_%H%M%S.jsonl")
            self.trace = Trace(trace_path)
            self.link = Link(cfg, meas, Path(cfg.seen_dir).expanduser(), self.trace)
            recorder = MapRecorder(Path(cfg.map_dir).expanduser())

            def on_frame(t_ns, jpeg):
                recorder.offer(t_ns, jpeg, self.last_cmd, self.last_us)

            threads.append(threading.Thread(target=self.link.receive_forever, args=(self.stop,)))
            threads.append(
                threading.Thread(target=camera_loop, args=(camera, self.link, self.stop, on_frame))
            )
            if self.buttons:
                from .buttons import Panel

                panel = Panel(self.set_mode, self.current_mode)
                threads.append(threading.Thread(target=panel.run, args=(self.stop,)))
        threads.append(threading.Thread(target=self._ultrasonic_loop, args=(car, us)))
        threads.append(threading.Thread(target=self._battery_loop, args=(car,)))
        for t in threads:
            t.daemon = True
            t.start()

        try:
            self._actuate_loop(car, cmd)
        finally:
            self.stop.set()
            proc_stop.set()
            car.stop()
            proc.join(timeout=1.0)
            for t in threads:
                t.join(timeout=1.0)
            car.close()
            if camera is not None:
                camera.close()
            if self.link is not None:
                self.link.close()
            if self.trace is not None:
                self.trace.close()
            for b in (meas, us, cmd, mode):
                b.close()

    def set_mode(self, mode: str) -> None:
        self.mode_buf.write((MODES.index(mode),))

    def current_mode(self) -> str:
        return MODES[int(self.last_cmd[14])] if self.last_cmd is not None else self.cfg.control.mode

    def _ultrasonic_loop(self, car, us: SeqBuffer) -> None:
        period = 1.0 / self.cfg.car.ultrasonic_hz
        fails = 0
        while not self.stop.is_set():
            t0 = time.monotonic()
            cm = car.read_ultrasonic()
            if self.trace is not None:
                self.trace.loop("超音波1回", (time.monotonic() - t0) * 1000)
            self.last_us = cm
            us.write((time.monotonic_ns(), cm if cm is not None else -1.0))
            fails = 0 if cm is not None else fails + 1
            # 何度も測れなければ（つながっていない・前に何も無い）間隔をあける
            wait = period if fails < 20 else self.cfg.car.ultrasonic_retry_s
            time.sleep(max(0.0, wait - (time.monotonic() - t0)))

    def _battery_loop(self, car) -> None:
        low_v = self.cfg.car.battery_low_v
        while not self.stop.is_set():
            label, ok = battery_state(car.read_battery(), low_v)
            if ok != self.battery[1] and not self.quiet:
                print(label, flush=True)
            self.battery = (label, ok)
            self.stop.wait(self.cfg.car.battery_check_s)

    def _actuate_loop(self, car, cmd: SeqBuffer) -> None:
        period = 1.0 / self.cfg.car.actuator_hz
        start = time.monotonic()
        nxt = start
        last_print = last_trace = 0.0
        prev = None
        applied_capture = 0
        trace = self.trace
        while not self.stop.is_set():
            now = time.monotonic()
            if self.seconds is not None and now - start >= self.seconds:
                break
            if trace is not None and prev is not None:
                trace.loop("書き込みの周期", (now - prev) * 1000)
            prev = now
            r = cmd.read()
            if r is not None:
                v = r[1]
                if time.monotonic_ns() - int(v[0]) > CMD_STALE_NS:
                    car.stop()  # 制御プロセスが止まった
                else:
                    # 電池が少ない・入っていないときは走らない（首は動かす）
                    car.apply(v[1], v[2] if self.battery[1] else 0.0, v[3], v[4])
                    self.last_cmd = v
                    self._record(now - start, v)
                    # 新しい検出を使ったコマンドを初めて書いた
                    if trace is not None and int(v[15]) and int(v[15]) != applied_capture:
                        applied_capture = int(v[15])
                        trace.mark(applied_capture, "used", int(v[16]))
                        trace.mark(applied_capture, "applied")
            if not self.quiet and now - last_print >= 0.5 and self.last_cmd is not None:
                last_print = now
                print(self._status(self.last_cmd), flush=True)
            if not self.quiet and trace is not None and now - last_trace >= 10.0:
                last_trace = now
                print(trace.summary(), flush=True)
            nxt += period
            time.sleep(max(0.0, nxt - time.monotonic()))

    def _record(self, t: float, v) -> None:
        truth = self.world.truth() if self.world is not None else (float("nan"),) * 2
        self.history.append((t, int(v[5]), v[7], v[8], *truth, v[3], v[2], v[6]))

    def _status(self, v) -> str:
        s = (
            f"{MODES[int(v[14])]:6s} {STATE_NAMES[int(v[5])]:6s} pan={v[3]:+5.1f} tilt={v[4]:+5.1f} "
            f"steer={v[1]:+5.1f} speed={v[2]:+5.1f}"
        )
        if v[6]:
            s += f" | 方位={v[7]:+6.1f}° 距離={v[8]:5.0f}cm 見えてから={v[9]:4.2f}s"
        if v[13] >= 0:
            s += f" | 撮影 {int(v[13])}"
        if int(v[17]) != OK:
            s += f" | 安全={GUARD_NAMES[int(v[17])]}"
        s += f" | 超音波={self.last_us or -1:.0f}cm loop={v[10]:6.0f}Hz p99遅れ={v[11]:5.1f}us 超過={int(v[12])}"
        s += f" | {self.battery[0]}"
        if self.link is not None:
            st = self.link.stats
            age = (time.monotonic_ns() - st.last_reply_ns) / 1e9 if st.last_reply_ns else -1
            s += (
                f" | 送信={st.sent} 返事={st.replies} snoo={st.snoo_hits} "
                f"rtt={st.last_rtt_ms:.0f}ms 推論={st.last_infer_ms:.0f}ms 最後の返事={age:.1f}s前 "
                f"宛先={self.link.route.host}"
            )
        if self.camera is not None:
            boost = "（暗いので上げている）" if self.camera.boost.gain is not None else ""
            cam = self.camera
            s += (
                f" | 露光={cam.exposure_us / 1000:.1f}ms ゲイン={cam.gain:.1f}{boost} "
                f"明るさ={cam.brightness:.0f}"
            )
        if self.world is not None:
            b, d = self.world.truth()
            s += f" | 真値 方位={b:+6.1f}° 距離={d:5.0f}cm"
        return s
