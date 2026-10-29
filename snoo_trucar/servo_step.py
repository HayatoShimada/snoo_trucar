"""パンのサーボの実際の遅れを、カメラ画像のずれから測る（車体は動かさない。首だけ振る）。

  .venv/bin/python -m snoo_trucar.servo_step [--step 10] [--repeat 4]

パンを 0° と step° の間で行き来させ、I2C に書いた時刻から、画像が動き始める（10%）・半分（50%）・
止まる（90%）までの時間を出す。画像のずれは、隣のフレームの Y を縮めて位相相関で求め、足し合わせる。
撮った時刻は camera.capture_time_ns（画面の中央の行の露光の真ん中）なので、readout_ms の見積もりの
誤差はそのまま入る。サービス（snoo-trucar）を止めてから動かす。
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np

from .camera import Camera
from .config import load

SHRINK = 4  # 位相相関は 160x120 で
RECORD_S = 0.6


def shift_px(a: np.ndarray, b: np.ndarray) -> float:
    """a から b への横のずれ（画素。右に動けば +）を位相相関で求める。"""
    fa, fb = np.fft.rfft2(a), np.fft.rfft2(b)
    r = fb * np.conj(fa)
    r /= np.abs(r) + 1e-9
    c = np.fft.irfft2(r, s=a.shape)
    col = c.sum(axis=0)
    i = int(np.argmax(col))
    # 隣との放物線で小数まで
    y0, y1, y2 = col[i - 1], col[i], col[(i + 1) % len(col)]
    d = 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2) if (y0 - 2 * y1 + y2) != 0 else 0.0
    x = i + d
    return x - len(col) if x > len(col) / 2 else x


def crossing(ts: list[float], ys: list[float], level: float) -> float | None:
    """ys が初めて level を越える時刻（隣の 2 点の間は直線で補う）。"""
    for k in range(1, len(ys)):
        if (ys[k - 1] - level) * (ys[k] - level) <= 0 and ys[k] != ys[k - 1]:
            return ts[k - 1] + (level - ys[k - 1]) / (ys[k] - ys[k - 1]) * (ts[k] - ts[k - 1])
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--step", type=float, default=10.0, help="振る角度（度）")
    parser.add_argument("--repeat", type=int, default=4, help="往復の回数")
    args = parser.parse_args()

    from .hal import Car

    cfg = load()
    car = Car(cfg.car)
    cam = Camera(cfg.camera)
    fx = 0.5 / math.tan(math.radians(cfg.camera.hfov_deg) / 2) * cfg.camera.width / SHRINK

    def small() -> tuple[int, np.ndarray]:
        cam.grab()
        t = cam.last_capture_ns
        y = cam.last_y[::SHRINK, ::SHRINK].astype(np.float32)
        return t, (y - y.mean()) * np.hanning(y.shape[0])[:, None] * np.hanning(y.shape[1])[None]

    results = []
    try:
        car.apply(0, 0, 0.0, 0.0)
        time.sleep(1.0)
        pan = 0.0
        for k in range(args.repeat * 2):
            goal = args.step if pan == 0.0 else 0.0
            for _ in range(3):  # 古いフレームを捨てる
                cam.grab()
            _, prev = small()
            car.apply(0, 0, goal, 0.0)
            t_write = time.monotonic_ns()
            ts, deg, total = [], [], 0.0
            while time.monotonic_ns() - t_write < RECORD_S * 1e9:
                t, cur = small()
                total += shift_px(prev, cur)
                prev = cur
                ts.append((t - t_write) / 1e6)
                deg.append(math.degrees(math.atan(total / fx)))
            pan = goal
            final = float(np.median(deg[-5:]))
            if abs(final) < 0.5 * args.step:
                print(f"{k}: 画像のずれが小さすぎる（{final:.1f}°）。模様の少ない所を向いているかもしれない")
                continue
            ys = [d / final for d in deg]
            t10, t50, t90 = (crossing(ts, ys, q) for q in (0.1, 0.5, 0.9))
            results.append((t10, t50, t90, abs(final)))
            fmt = lambda v: "?" if v is None else f"{v:.0f}"  # noqa: E731
            print(f"{k}: {goal:+.0f}° へ: 動き始め {fmt(t10)} ms・半分 {fmt(t50)} ms・"
                  f"90% {fmt(t90)} ms（画像で {abs(final):.1f}°）")  # fmt: skip
        ok = [r for r in results if None not in r[:3]]
        if ok:
            a = np.array([r[:3] for r in ok])
            print(f"中央値（{len(ok)} 回）: 動き始め {np.median(a[:, 0]):.0f} ms・半分 {np.median(a[:, 1]):.0f} ms"
                  f"・90% {np.median(a[:, 2]):.0f} ms・画像の角度 {np.median([r[3] for r in ok]):.1f}°"
                  f"（指令 {args.step:.0f}°）")  # fmt: skip
    finally:
        car.close()
        cam.close()


if __name__ == "__main__":
    main()
