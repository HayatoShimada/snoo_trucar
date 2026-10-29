"""map で撮った画像（scan.json）を、カメラの向きから 1 枚のパノラマ（正距円筒、横=方位・縦=仰角）に貼り合わせる。

画像の特徴点は使わず、撮ったときのパン・チルトの角度と画角だけで置く。サーボの角度がずれていれば継ぎ目もずれる。
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

from .config import CameraConfig


def _sample(img: np.ndarray, u: np.ndarray, v: np.ndarray) -> np.ndarray:
    """画素座標 (u, v) を双線形で引く"""
    h, w = img.shape[:2]
    u = np.clip(u, 0, w - 1.001)
    v = np.clip(v, 0, h - 1.001)
    u0, v0 = u.astype(int), v.astype(int)
    fu, fv = (u - u0)[..., None], (v - v0)[..., None]
    a, b = img[v0, u0], img[v0, u0 + 1]
    c, d = img[v0 + 1, u0], img[v0 + 1, u0 + 1]
    return (a * (1 - fu) + b * fu) * (1 - fv) + (c * (1 - fu) + d * fu) * fv


def build(scan_dir: Path, cam: CameraConfig, px_per_deg: float = 8.0, tilt_offset_deg: float = 0.0,
          pan_gain: float = 1.0) -> tuple[Image.Image, dict]:
    scan = json.loads((scan_dir / "scan.json").read_text())
    # チルトのサーボのずれ（指令 0° のとき実際は上を向いている分）を足す
    # パンは指令より少なく回る（pan_gain 倍）
    frames = [dict(f, pan_deg=f["pan_deg"] * pan_gain, tilt_deg=f["tilt_deg"] + tilt_offset_deg)
              for f in scan["frames"]]  # fmt: skip
    if not frames:
        raise ValueError(f"画像が無い: {scan_dir}")
    hh, vh = cam.hfov_deg / 2, cam.vfov_deg / 2
    pans = [f["pan_deg"] for f in frames]
    tilts = [f["tilt_deg"] for f in frames]
    yaw0, yaw1 = min(pans) - hh, max(pans) + hh
    pit0, pit1 = min(tilts) - vh, max(tilts) + vh
    W = int((yaw1 - yaw0) * px_per_deg)
    H = int((pit1 - pit0) * px_per_deg)
    yaw = np.radians(yaw0 + (np.arange(W) + 0.5) / px_per_deg)[None, :]
    pit = np.radians(pit1 - (np.arange(H) + 0.5) / px_per_deg)[:, None]
    # 向き（x 右、y 上、z 前）
    dx, dy, dz = np.sin(yaw) * np.cos(pit), np.sin(pit) + 0 * yaw, np.cos(yaw) * np.cos(pit)
    fx = 0.5 / math.tan(math.radians(hh))
    fy = 0.5 / math.tan(math.radians(vh))
    acc = np.zeros((H, W, 3))
    wsum = np.zeros((H, W, 1))
    for f in frames:
        img = np.asarray(Image.open(scan_dir / f["file"]).convert("RGB"), dtype=np.float32)
        ih, iw = img.shape[:2]
        p, t = math.radians(f["pan_deg"]), math.radians(f["tilt_deg"])
        # カメラの右・上・前
        r = (math.cos(p), 0.0, -math.sin(p))
        up = (-math.sin(p) * math.sin(t), math.cos(t), -math.cos(p) * math.sin(t))
        fw = (math.sin(p) * math.cos(t), math.sin(t), math.cos(p) * math.cos(t))
        cx = dx * r[0] + dy * r[1] + dz * r[2]
        cy = dx * up[0] + dy * up[1] + dz * up[2]
        cz = dx * fw[0] + dy * fw[1] + dz * fw[2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = 0.5 + fx * cx / cz
            v = 0.5 - fy * cy / cz
        inside = (cz > 0) & (u >= 0) & (u < 1) & (v >= 0) & (v < 1)
        if not inside.any():
            continue
        # 画面の端ほど軽くして、継ぎ目をぼかす
        wgt = np.where(inside, np.minimum(u, 1 - u) * np.minimum(v, 1 - v), 0.0)[..., None]
        ys, xs = np.nonzero(inside)
        pix = _sample(img, u[ys, xs] * iw, v[ys, xs] * ih)
        acc[ys, xs] += pix * wgt[ys, xs]
        wsum[ys, xs] += wgt[ys, xs]
    out = np.where(wsum > 0, acc / np.maximum(wsum, 1e-9), 0).clip(0, 255).astype(np.uint8)
    pano = Image.fromarray(out)
    _grid(pano, yaw0, pit1, px_per_deg)
    info = {"yaw_deg": [yaw0, yaw1], "pitch_deg": [pit0, pit1], "px_per_deg": px_per_deg,
            "frames": len(frames)}  # fmt: skip
    return pano, info


def _grid(pano: Image.Image, yaw0: float, pit1: float, ppd: float) -> None:
    """10 度ごとの目盛り。横は車体の正面からの方位（右が +）、縦は水平からの仰角"""
    d = ImageDraw.Draw(pano)
    W, H = pano.size
    for a in range(-180, 181, 10):
        x = (a - yaw0) * ppd
        if 0 <= x < W:
            d.line([(x, H - 8), (x, H)], fill=(255, 255, 0), width=1)
            if a % 30 == 0:
                d.line([(x, 0), (x, H)], fill=(255, 255, 0), width=1)
                d.text((x + 3, H - 20), f"{a:+d}", fill=(255, 255, 0))
    for e in range(-90, 91, 10):
        y = (pit1 - e) * ppd
        if 0 <= y < H:
            d.line([(0, y), (8, y)], fill=(255, 255, 0), width=1)
            if e == 0:
                d.line([(0, y), (W, y)], fill=(0, 255, 255), width=1)
            d.text((10, y - 6), f"{e:+d}", fill=(255, 255, 0))


def latest(root: Path) -> Path:
    dirs = sorted(p for p in root.glob("map_*") if (p / "scan.json").exists())
    if not dirs:
        raise FileNotFoundError(f"scan.json のある map_* が無い: {root}")
    return dirs[-1]
