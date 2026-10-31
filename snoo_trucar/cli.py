"""コマンド。

  run          snoo を探して追う（--sim でハード・サーバーなしのシミュレーション）
  fake-server  検出サーバーの偽物（決まった位置に snoo がいると返す）
  probe        画像を 1 枚サーバーに送って返事を見る
  panorama     map で撮った画像を 1 枚のパノラマにする
  stop         モーターだけを止める（systemd の ExecStopPost。run が動いていなくても使える）
  wait-scan    explore を止めずに入れ替えられる時（撮っている間）まで待つ
"""

from __future__ import annotations

import argparse
import json
import socket
import time
from pathlib import Path

from . import config as config_mod
from .controller import MODES
from .protocol import make_reply, pack_frame, unpack_frame


def _run(args) -> int:
    from .runner import Runner

    cfg = config_mod.load(args.config)
    if args.mode:
        cfg.control.mode = args.mode
    if args.max_speed is not None:
        cfg.control.max_speed = args.max_speed
    if args.search_drive:
        cfg.control.search_drive = True
    if args.server:
        host, _, port = args.server.rpartition(":")
        cfg.server.host, cfg.server.port = host, int(port)
    if args.any_cat:
        cfg.control.require_snoo = False
    print(f"mode={cfg.control.mode} max_speed={cfg.control.max_speed} sim={args.sim}", flush=True)
    from .runner import ControlDied, ThreadDied

    try:
        # --mode を決めずに起動したら、10 分以内の explore の続きに戻る（resume.py）
        Runner(cfg, sim=args.sim, seconds=args.seconds, buttons=not args.no_buttons,
               camera_file=args.camera_file, resume=not args.mode).run()
    except KeyboardInterrupt:
        pass
    except (ControlDied, ThreadDied) as e:
        print(f"{e}。止めて終わる", flush=True)
        return 1  # systemd の Restart=on-failure で起動し直す
    return 0


def _stop(args) -> int:
    from .hal import stop_motors

    try:
        stop_motors(config_mod.load(args.config).car)
    except Exception as e:  # 止められなかったことを残す（systemd のログ）。ここでは他にできることが無い
        print(f"モーターを止められなかった: {type(e).__name__}: {e}", flush=True)
        return 1
    print("モーターを止めた", flush=True)
    return 0


def _wait_scan(args) -> int:
    """explore の間なら、撮っている（scan）間になるまで待つ。走っている・返事を待っている間に
    起動し直すと、走った 1 歩が分からなくなる。地図のサーバーが返事を作っている間（busy）も待つ。
    explore でなければすぐ 0。待ちきれなければ 1"""
    from .resume import ExploreState

    cfg = config_mod.load(args.config)
    path = Path(cfg.explore_state).expanduser() if cfg.explore_state else None
    end = time.monotonic() + args.timeout
    busy_url = cfg.server.map_url.split("/v1/")[0] + "/v1/snoo/map/busy"
    while time.monotonic() < end:
        st = ExploreState(path).load()
        if st is None:
            print("explore ではない（続きが無い）", flush=True)
            return 0
        if st.get("phase") == "scan" and _map_idle(busy_url):
            print(f"撮っている（station {st['next_station']}）。今なら入れ替えてよい", flush=True)
            return 0
        time.sleep(0.2)
    print(f"{args.timeout:.0f} 秒待っても撮っている間にならなかった", flush=True)
    return 1


def _map_idle(url: str) -> bool:
    """地図のサーバーが返事を作っていない（in_flight 0）。聞けなければ見ない
    （Pi の段階だけで決める）"""
    try:
        import requests

        return int(requests.get(url, timeout=1.0).json().get("in_flight", 0)) == 0
    except Exception:
        return True


def _fake_server(args) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    print(f"偽サーバー {args.host}:{args.port}（box={args.box}）", flush=True)
    box = [float(x) for x in args.box.split(",")]
    while True:
        data, addr = sock.recvfrom(65535)
        t0 = time.monotonic()
        try:
            seq, t_ns, _w, _h, _jpeg = unpack_frame(data)
        except ValueError:
            continue
        dets = [{"box": box, "cat": 0.9, "snoo": 0.9}] if not args.empty else []
        sock.sendto(make_reply(seq, t_ns, (time.monotonic() - t0) * 1e3, dets), addr)


def _probe(args) -> None:
    cfg = config_mod.load(args.config)
    host, port = cfg.server.host, cfg.server.port
    if args.server:
        host, _, p = args.server.rpartition(":")
        port = int(p)
    jpeg = Path(args.image).read_bytes()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    for i in range(args.count):
        t = time.monotonic_ns()
        sock.sendto(pack_frame(i + 1, t, 640, 480, jpeg), (host, port))
        try:
            data, _ = sock.recvfrom(65535)
        except TimeoutError:
            print(f"#{i + 1}: 返事なし（2 秒）")
            continue
        rtt = (time.monotonic_ns() - t) / 1e6
        print(f"#{i + 1}: rtt={rtt:.1f}ms {json.dumps(json.loads(data), ensure_ascii=False)}")


def _panorama(args) -> None:
    from .panorama import build, latest

    cfg = config_mod.load(args.config)
    scan_dir = Path(args.dir).expanduser() if args.dir else latest(Path(cfg.map_dir).expanduser())
    pano, info = build(scan_dir, cfg.camera, args.px_per_deg, args.tilt_offset, args.pan_gain)
    out = scan_dir / "panorama.jpg"
    pano.save(out, quality=90)
    print(f"{out}  {pano.size[0]}x{pano.size[1]}  方位 {info['yaw_deg'][0]:+.0f}..{info['yaw_deg'][1]:+.0f}°"
          f"  仰角 {info['pitch_deg'][0]:+.0f}..{info['pitch_deg'][1]:+.0f}°  {info['frames']} 枚")


def main(argv=None) -> None:
    p = argparse.ArgumentParser(prog="snoo_trucar")
    p.add_argument("--config", type=Path, default=None)
    sub = p.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", help="snoo を探して追う")
    r.add_argument(
        "--mode",
        choices=MODES,
        help="起動したときの行動（既定 idle。USR ボタンで変わる）。watch: 探してカメラで追う / "
        "keep: 距離を保つ / follow: 追う / map: 部屋を撮る",
    )
    r.add_argument("--camera-file", help="カメラの代わりにこの JPEG を流す（試験用）")
    r.add_argument("--no-buttons", action="store_true", help="USR ボタンと LED を使わない")
    r.add_argument("--max-speed", type=float, help="picarx の速さの上限（0..100）")
    r.add_argument("--search-drive", action="store_true", help="探すときに車体も動かす")
    r.add_argument("--server", help="検出サーバー host:port")
    r.add_argument("--any-cat", action="store_true", help="snoo の判定なしで、猫なら追う")
    r.add_argument("--sim", action="store_true", help="シミュレーション（ハード・サーバーなし）")
    r.add_argument("--seconds", type=float, help="この秒数で止める")
    r.set_defaults(func=_run)

    f = sub.add_parser("fake-server", help="検出サーバーの偽物")
    f.add_argument("--host", default="127.0.0.1")
    f.add_argument("--port", type=int, default=8710)
    f.add_argument("--box", default="0.45,0.35,0.6,0.7", help="x1,y1,x2,y2（0..1）")
    f.add_argument("--empty", action="store_true", help="何も見つからないと返す")
    f.set_defaults(func=_fake_server)

    q = sub.add_parser("probe", help="画像を 1 枚サーバーに送って返事を見る")
    q.add_argument("--image", required=True)
    q.add_argument("--server", help="host:port（既定は設定の値）")
    q.add_argument("--count", type=int, default=5)
    q.set_defaults(func=_probe)

    m = sub.add_parser("panorama", help="map で撮った画像を 1 枚のパノラマにする")
    m.add_argument("dir", nargs="?", help="map_* のディレクトリ（既定は一番新しいもの）")
    m.add_argument("--px-per-deg", type=float, default=8.0)
    # 2026-10-29 に home-linux の VGGT で測った値（パンは指令の 0.96 倍）
    m.add_argument("--tilt-offset", type=float, default=None,
                   help="チルトのずれ（度）。既定は scan.json の値（較正前の版 1 は 26.5）")
    m.add_argument("--pan-gain", type=float, default=0.96, help="実際のパン / 指令のパン")
    m.set_defaults(func=_panorama)

    s = sub.add_parser("stop", help="モーターだけを止める（run が動いていなくても）")
    s.set_defaults(func=_stop)

    w = sub.add_parser("wait-scan", help="入れ替えてよい時（explore で撮っている間）まで待つ")
    w.add_argument("--timeout", type=float, default=120.0)
    w.set_defaults(func=_wait_scan)

    args = p.parse_args(argv)
    return args.func(args)
