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
    fps: float = 30.0
    jpeg_quality: int = 70
    # OV5647（PiCar-X 付属）の画角。度
    hfov_deg: float = 53.5
    vfov_deg: float = 41.4
    rotate180: bool = False


@dataclass
class ServerConfig:
    # home-linux の検出サーバー（docs/PROTOCOL.md）。tailnet の中だけで受ける
    host: str = "home-linux.taila713c8.ts.net"
    port: int = 8710
    # 返事を待たずに送ってよいフレーム数
    max_in_flight: int = 2
    # これより古い返事は捨てる
    reply_timeout_s: float = 0.5


@dataclass
class ControlConfig:
    rate_hz: float = 5000.0
    # 制御ループを固定する CPU（-1 で固定しない）
    cpu: int = 3
    # 起動したときの行動（controller.MODES）。USR ボタンで変わる
    mode: str = "idle"
    # snoo とみなす確率。サーバーが snoo を返さないとき（猫の検出だけ）は require_snoo=false にする
    snoo_threshold: float = 0.6
    require_snoo: bool = True
    cat_threshold: float = 0.35
    # 距離（cm）
    keep_distance_cm: float = 80.0
    follow_distance_cm: float = 50.0
    distance_deadband_cm: float = 15.0
    # 箱の高さ（画面比）から距離を出す係数: d = k / h。猫の高さ 25 cm、vfov 41.4° でおよそ 33
    distance_k: float = 33.0
    # 超音波がこの距離より近ければ前に進まない
    stop_distance_cm: float = 20.0
    # 速さ（picarx の -100..100）。「大きく動かさない」ため小さめ
    max_speed: float = 20.0
    # 探すときに車体も動かす（false ならカメラを振るだけ）
    search_drive: bool = False
    search_drive_speed: float = 15.0
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
    actuator_hz: float = 50.0
    ultrasonic_hz: float = 15.0
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
