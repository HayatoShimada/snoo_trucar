"""設定。~/.config/snoo_trucar/config.toml の値で既定値を上書きする（例は deploy/config.example.toml）。"""

from __future__ import annotations

import dataclasses
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

DEFAULT_PATH = Path.home() / ".config" / "snoo_trucar" / "config.toml"


@dataclass
class CameraConfig:
    width: int = 640
    height: int = 480
    # OV5647 の 640x480 のモードは 62.5 fps まで。60 fps で撮ってから届くまでと、結果が新しくなる間隔が
    # 30 fps の半分になる（間隔 16.7 ms、カメラの中の遅れ 22 → 19 ms）
    fps: float = 60.0
    # サーバーの検出は品質 50 でも見つかる割合が変わらなかった。小さいほど送るのが速い
    jpeg_quality: int = 60
    # 60 fps では露光が 16.6 ms までになる。暗いときはアナログゲインをここまで上げる（AE は 8 まで。センサーは 16 まで）
    max_gain: float = 16.0
    # 暗いときに目指す明るさ（画素の平均 0..255）。30 fps・ゲイン 8 で露光が上限のとき 55 だった
    brightness_target: float = 60.0
    # 1 行目から最後の行までの読み出しにかかる時間（ms）。撮った時刻を画面の中央の行に合わせるのに使う。
    # 640x480 のモードの最短フレーム（16 ms）からの見積もりで、測っていない
    readout_ms: float = 15.0
    # OV5647（PiCar-X 付属）の画角。度
    hfov_deg: float = 53.5
    vfov_deg: float = 41.4
    rotate180: bool = False


@dataclass
class ServerConfig:
    # home-linux の検出サーバー（docs/PROTOCOL.md）。家の中では tailscale を通さず LAN の名前に送る
    # （ping は tailscale 経由で平均 6.7 ms、LAN 宛てで 3.7 ms）。名前が引けない・返事が
    # switch_after_s 秒来ないときは fallback_host（tailscale）に替え、retry_host_s 秒ごとに host を試し直す
    host: str = "home-linux.local"
    fallback_host: str = "home-linux.taila713c8.ts.net"
    switch_after_s: float = 1.0
    retry_host_s: float = 30.0
    port: int = 8710
    # 返事を待たずに送ってよいフレーム数
    max_in_flight: int = 2
    # これより古い返事は捨てる
    reply_timeout_s: float = 0.5


@dataclass
class ControlConfig:
    # 推定と制御則の周期。ハードが受け取れるのは 50 Hz（サーボの PWM の周期 20 ms）なので、その 20 倍の 1 kHz。
    # 5 kHz でも回るが、1 コアを 100% 使う（1 kHz なら 2 割ほど）。推定は実際の dt で進めるので、周期の揺れは効かない
    rate_hz: float = 1000.0
    # 制御ループを固定する CPU（-1 で固定しない）
    cpu: int = 3
    # 起動したときの行動（controller.MODES）。USR ボタンで変わる
    mode: str = "idle"
    # snoo とみなす確率。サーバーが snoo を返さないとき（猫の検出だけ）は require_snoo=false にする
    snoo_threshold: float = 0.6
    require_snoo: bool = True
    # 暗い画像で、検出器が画面全体を猫（0.3〜0.4）と返すことがある。本物の猫は 0.8 前後
    cat_threshold: float = 0.45
    # これより大きい箱（画面に対する面積の比）は捨てる
    max_box_area: float = 0.8
    # 誤検出で追い始めないよう、confirm_window_s の間に confirm_hits 回見えたら追い始める
    confirm_hits: int = 3
    confirm_window_s: float = 0.5
    # 距離（cm）
    keep_distance_cm: float = 80.0
    follow_distance_cm: float = 50.0
    distance_deadband_cm: float = 15.0
    # 箱の高さ（画面比）から距離を出す係数: d = k / h。猫の高さ 25 cm、vfov 41.4° でおよそ 33
    distance_k: float = 33.0
    # 超音波がこの距離より近ければ前に進まない
    stop_distance_cm: float = 20.0
    # 速さ（picarx の -100..100）。picarx は 0 以外の速さ s を PWM のデューティ 50 + s/2 % にする（30 → 65 %）。
    # TT モーター（1:48、約 200 rpm）と後輪（径 約 6.5 cm）から、100 で 0.6〜0.7 m/s、30 で 0.3〜0.4 m/s くらい（実測していない）。
    # 室内で、超音波が使えないときも、見てから止まるまで（遅れ 0.1 s）に 10 cm ほどで済む速さにしている
    max_speed: float = 30.0
    # 探すときに車体も動かす（false ならカメラを振るだけ）
    search_drive: bool = False
    search_drive_speed: float = 20.0
    # カメラ・ハンドルは、今の推定ではなく lead_s 秒後の予測に向ける。書き込みは 50 Hz の 0 次ホールド
    # （平均 10 ms 遅れ）、サーボの PWM も 50 Hz（平均 10 ms）、サーボが動き出すまで 10 ms ほど
    lead_s: float = 0.03
    # 見失ってから予測だけで追う時間と、探し始めるまでの時間
    coast_s: float = 0.7
    lost_s: float = 2.0
    # 制御の利得
    pan_kp: float = 6.0  # 1/s。カメラの向きをこの速さで目標に寄せる
    steer_kp: float = 1.0  # 度/度
    speed_kp: float = 0.4  # 速さ/cm
    # 探すときのカメラの振り
    search_pan_deg: float = 35.0
    search_period_s: float = 6.0
    search_tilt_deg: float = 0.0


@dataclass
class CarConfig:
    # picar-x の較正値（servo のずれなど）。picarx が自分で読み書きする
    picarx_config: str = str(Path.home() / ".config" / "snoo_trucar" / "picar-x.conf")
    # サーボの PWM（robot_hat の Servo.FREQ）は 50 Hz で、20 ms ごとにしか新しい角度を受け取らない。
    # ただ書き込みも 50 Hz だと、PWM と揃っていないので 0 次ホールドが 2 段重なる。200 Hz にすると
    # 検出を使ってから I2C に書くまでが p50 8.9 → 2.4 ms、p95 16 → 4.4 ms（2026-10-29 に実測）。
    # 値が変わったときだけ書く（hal.Car.apply）。I2C は 1 回 0.45 ms
    actuator_hz: float = 200.0
    # 超音波（HC-SR04 型）は、前の反響が消えるまで 60 ms あけるのがよいとされるので 15 Hz
    ultrasonic_hz: float = 15.0
    # 測れない（つながっていない・前に何も無い）ことが続いたら、この間隔でだけ試す。
    # 1 回に 21 ms ほど Python で待ち続け（GIL を持ったまま）、その間はカメラ・送信・書き込みが止まる。
    # 1 s ごとだと撮影→返事の最大が 64 ms、30 s ごとなら 55 ms（2026-10-29 に実測）
    ultrasonic_retry_s: float = 30.0
    pan_min: float = -35.0
    pan_max: float = 35.0
    tilt_min: float = -35.0
    tilt_max: float = 35.0
    steer_max: float = 30.0


@dataclass
class Config:
    camera: CameraConfig = field(default_factory=CameraConfig)
    server: ServerConfig = field(default_factory=ServerConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    car: CarConfig = field(default_factory=CarConfig)
    # map で撮った画像の置き場所（map_日時/ に画像と scan.json）
    map_dir: str = "~/captures/snoo_map"
    seen_dir: str = "~/captures/snoo_seen"  # 追い始めたときの画像と箱
    # フレームごとの区間の時刻（trace.py）を JSON Lines で残すディレクトリ。空なら残さない（集計は出す）
    trace_dir: str = ""


def _merge(obj, values: dict, where: str):
    names = {f.name: f for f in dataclasses.fields(obj)}
    for key, value in values.items():
        if key not in names:
            raise ValueError(f"設定に無い項目: {where}{key}")
        current = getattr(obj, key)
        if dataclasses.is_dataclass(current):
            _merge(current, value, f"{where}{key}.")
        else:
            setattr(obj, key, type(current)(value))


def load(path: Path | None = None) -> Config:
    cfg = Config()
    path = path or DEFAULT_PATH
    if path.exists():
        with open(path, "rb") as f:
            _merge(cfg, tomllib.load(f), "")
    return cfg
