"""home-linux の検出サーバーとのやりとり（UDP）。仕様は docs/PROTOCOL.md。"""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass

MAGIC = b"SNO1"
HEADER = struct.Struct("<4sIQHH")  # magic, seq, t_capture_ns, width, height
MAX_DATAGRAM = 65000


# 版 2: 撮った時刻に書いていた指令も付ける（走っている間の通り道の空き clear_cm をサーバーが出すため）
MAGIC2 = b"SNO2"
HEADER2 = struct.Struct("<4sIQHHffffB")  # + steer_deg, speed, pan_deg, tilt_deg, flags
FLAG_MOVING, FLAG_REVERSING = 1, 2


def pack_frame(seq: int, t_capture_ns: int, width: int, height: int, jpeg: bytes,
               pose: tuple[float, float, float, float] | None = None) -> bytes:  # fmt: skip
    """pose = (steer_deg, speed, pan_deg, tilt_deg) を渡すと版 2（SNO2）"""
    if pose is None:
        data = HEADER.pack(MAGIC, seq & 0xFFFFFFFF, t_capture_ns, width, height) + jpeg
    else:
        steer, speed, pan, tilt = pose
        flags = (FLAG_MOVING if abs(speed) > 0.5 else 0) | (FLAG_REVERSING if speed < -0.5 else 0)
        data = HEADER2.pack(MAGIC2, seq & 0xFFFFFFFF, t_capture_ns, width, height,
                            steer, speed, pan, tilt, flags) + jpeg  # fmt: skip
    if len(data) > MAX_DATAGRAM:
        raise ValueError(f"JPEG が大きすぎる: {len(data)} bytes")
    return data


def unpack_frame(data: bytes) -> tuple[int, int, int, int, bytes]:
    if data[:4] == MAGIC2:
        magic, seq, t_ns, w, h, *_ = HEADER2.unpack_from(data)
        return seq, t_ns, w, h, data[HEADER2.size :]
    magic, seq, t_ns, w, h = HEADER.unpack_from(data)
    if magic != MAGIC:
        raise ValueError("magic が違う")
    return seq, t_ns, w, h, data[HEADER.size :]


@dataclass
class Box:
    cx: float  # 画面比 0..1
    cy: float
    w: float
    h: float
    cat: float  # 猫らしさ
    snoo: float | None  # snoo らしさ（サーバーが判定しないときは None）
    stress: float | None = None  # 遊びの様子 play.p.stress（判定しないときは None）


@dataclass
class Reply:
    seq: int
    t_capture_ns: int
    infer_ms: float
    boxes: list[Box]
    # 走っている間の通り道の空き（cm）。欄が無い = サーバーが出していない（超音波だけで走る）。
    # 欄があって null = 出そうとしたが間に合わなかった
    clear_present: bool = False
    clear_cm: float | None = None
    clear_conf: float | None = None


def parse_reply(data: bytes) -> Reply:
    msg = json.loads(data)
    if msg.get("v") != 1:
        raise ValueError(f"知らない版: {msg.get('v')}")
    boxes = []
    for d in msg.get("dets", []):
        play = d.get("play") or msg.get("play") or {}
        stress = (play.get("p") or {}).get("stress")
        x1, y1, x2, y2 = d["box"]
        boxes.append(
            Box(
                cx=(x1 + x2) / 2,
                cy=(y1 + y2) / 2,
                w=x2 - x1,
                h=y2 - y1,
                cat=float(d.get("cat", 0.0)),
                snoo=None if d.get("snoo") is None else float(d["snoo"]),
                stress=None if stress is None else float(stress),
            )
        )
    clear = msg.get("clear_cm")
    conf = msg.get("clear_conf")
    return Reply(int(msg["seq"]), int(msg["t_capture_ns"]), float(msg.get("infer_ms", 0)), boxes,
                 "clear_cm" in msg, None if clear is None else float(clear),
                 None if conf is None else float(conf))  # fmt: skip


def make_reply(seq: int, t_capture_ns: int, infer_ms: float, dets: list[dict]) -> bytes:
    """サーバー側の返事を作る（テスト・偽サーバー用。サーバーの実装の見本にもなる）"""
    return json.dumps(
        {"v": 1, "seq": seq, "t_capture_ns": t_capture_ns, "infer_ms": infer_ms, "dets": dets}
    ).encode()


def pick_snoo(boxes: list[Box], cat_threshold: float, snoo_threshold: float, require_snoo: bool,
              max_box_area: float = 0.8):  # fmt: skip
    """snoo らしい箱を 1 つ選ぶ。無ければ None"""
    best, best_score = None, 0.0
    for b in boxes:
        if b.cat < cat_threshold or b.w * b.h > max_box_area:
            continue
        if b.snoo is None:
            if require_snoo:
                continue
            score = b.cat
        else:
            if b.snoo < snoo_threshold:
                continue
            score = b.snoo * b.cat
        if score > best_score:
            best, best_score = b, score
    return best, best_score
