"""設定。~/.config/snoo_trucar/config.toml の値で既定値を上書きする（例は deploy/config.example.toml）。"""

from __future__ import annotations

import dataclasses
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

from .guard import GuardConfig
from .voice import VoiceConfig

DEFAULT_PATH = Path.home() / ".config" / "snoo_trucar" / "config.toml"


@dataclass
class CameraConfig:
    # 床からレンズの中心まで（cm）。2026-10-29 にユーザーが測った。地図の縮尺に使う（station の meta）
    height_cm: float = 8.0
    width: int = 640
    height: int = 480
    # OV5647 の 640x480 のモードは 62.5 fps まで。60 fps で撮ってから届くまでと、結果が新しくなる間隔が
    # 30 fps の半分になる（間隔 16.7 ms、カメラの中の遅れ 22 → 19 ms）
    fps: float = 60.0
    # サーバーの検出は品質 50 でも見つかる割合が変わらなかった。小さいほど送るのが速い
    jpeg_quality: int = 60
    # 60 fps では露光が 16.6 ms までになる。暗いときはアナログゲインをここまで上げる（AE は 8 まで。センサーは 16 まで）
    max_gain: float = 16.0
    # 露光の上限（ms）。0 ならフレームの長さいっぱい（60 fps で 16.6 ms）。短くすると、撮ってから
    # 受け取るまでが露光の半分だけ縮み、走りながらのブレも減るが、暗い部屋では画像が暗くなる（snoo_trucar#1）
    max_exposure_ms: float = 0.0
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
    # 画像に撮った時刻の指令を付ける版 2（SNO2）で送る（サーバーは 2026-10-29 19:52 から読める）
    header_v2: bool = True
    # 待機（idle、ボタン待ち）のときに送るフレーム数/s。サーバーの GPU の仕事はフレーム数に比例するので、
    # 待機中は減らして電力を抑える（2026-10-29、ユーザーの希望）。行動を選んだらすぐ全部（60 fps）送る
    idle_fps: float = 5.0
    # 探索の地図のサーバー（撮った分を送ると、次の 1 歩を返す。docs/PROTOCOL.md）
    map_url: str = "http://192.168.0.248:8720/v1/snoo/map/station"
    map_timeout_s: float = 20.0


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
    # 指令から出した車体の角速度を、推定に分かっている入力として入れる（snoo_trucar#17）
    ego_ff: bool = True
    # 見失ってから予測だけで追う時間と、探し始めるまでの時間
    coast_s: float = 0.7
    lost_s: float = 2.0
    # 制御の利得
    # 1/s。カメラの向きをこの速さで目標に寄せる（約 2 Hz）。bench で 6 → 12 にすると、横へ走る snoo への
    # 向きのずれが RMS 3.5 → 2.7°。サーボの値が仮なので、#2 で測るまでは 20 にはしない
    pan_kp: float = 12.0
    steer_kp: float = 1.0  # 度/度（cascade = false のとき）
    # 車体の向きのループ（snoo_trucar#18）。首（方位）を 0° に戻すように、この帯域で車体を回す。
    # 首のループ（pan_kp、約 1 Hz）より十分遅くする。前輪で曲がる車は止まると向きを変えられないので、
    # 距離が目標の内でも方位が turn_creep_deg を超えたら creep_speed でゆっくり動いて向きを変える
    cascade: bool = True
    heading_hz: float = 0.3
    turn_creep_deg: float = 15.0
    creep_speed: float = 12.0
    # 速さ/cm（距離のループ。約 0.2 Hz）。bench の円を歩く snoo で、0.4 だと近づかれて見失い（最短 10 cm）、
    # 1.2 なら見失わず最短 41 cm だった（箱の高さのぶれ 8% を入れて）
    speed_kp: float = 1.2
    # 探すときのカメラの振り
    search_pan_deg: float = 35.0
    search_period_s: float = 6.0
    search_tilt_deg: float = 0.0
    # explore（部屋を動きながら撮る）。止まって撮る → home-linux の地図のサーバー（server.map_url）が決めた
    # 1 歩を走る、を繰り返す。Pi では道を決めない（届かなければ送り直して待つ）
    explore_speed: float = 15.0
    explore_max_s: float = 600.0
    explore_snoo_near_cm: float = 150.0
    # 止まって撮る向き：チルトの段と、段ごとのパン（段ごとに折り返して撮る）。パンは car.pan_min..pan_max に
    # 収める。広く撮る案（tilt 0 で -70..70、-15 で -35..35）は、首の台が当たらないか実物で確かめてから、
    # pan_min・pan_max と一緒に変える
    explore_scan_tilts: list = field(default_factory=lambda: [-15.0, 0.0])
    explore_scan_pans: list = field(
        default_factory=lambda: [[-30.0, -15.0, 0.0, 15.0, 30.0], [-30.0, -15.0, 0.0, 15.0, 30.0]]
    )
    # サーバーの前向きの区切りの合計の上限。地図で見えている床が続けば、最後のまっすぐは 60 cm まで来る
    # （long_step）。止まる決まり（clear_cm・画像が動かない・guard・電池）はそのまま
    explore_server_step_cm: float = 80.0
    # サーバーの下がる区切りの合計の上限（後ろは見えない）
    explore_reverse_max_cm: float = 30.0
    # サーバーが画像から出す通り道の空き（clear_cm）がこれより近ければ、進むのをやめて向きを変える。
    # 止まるまでの距離（未測定）+ 余裕 15 cm
    explore_clear_stop_cm: float = 25.0  # snoo がこれより近くに見えたら止まって見守る
    # snoo を怖がらせない決まり（guard.py）。設定ファイルでは [control.guard]
    guard: GuardConfig = field(default_factory=GuardConfig)


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
    # 電池（2 セルのリチウムイオン、満充電 8.4 V）。battery_low_v より低いと走らない（首は動かす）。
    # 0 V 近くは電池が入っていない・スイッチが切れている（サーボもモーターも動かない）
    battery_low_v: float = 6.6
    battery_check_s: float = 5.0
    # 電池がこれより低いのが shutdown_hold_s 続いたら、Pi を shutdown する（Pi も車の電池から給電していると、
    # 切れたときに電源ごと落ちて SD カードが壊れる。2026-10-29 に 2 回）。0 で使わない。
    # sudo で shutdown をパスワードなしで呼べるようにしておく（deploy/sudoers-snoo-shutdown）
    shutdown_v: float = 6.4
    shutdown_hold_s: float = 10.0
    pan_min: float = -35.0
    pan_max: float = 35.0
    tilt_min: float = -35.0
    tilt_max: float = 35.0
    steer_max: float = 30.0
    # 車体の動きの換算（自転車モデル）。速さ 1 あたりの cm/s と、前後の車輪の間（cm）。
    # 速さの換算は見積もりで、測っていない（snoo_trucar#8）。車輪の半径は 3.26 cm、左右の車輪の間は 12 cm
    # 2026-10-29: 速さ 12 で 1.48 s 走って 28〜30 cm（home-linux の VGGT の位置、カメラの高さ 8.0 cm で縮尺）→ 約 1.7
    cm_s_per_speed: float = 1.7
    wheelbase_cm: float = 14.0  # PiCar-X の URDF（cau-se/ARCHES-PiCar-X）。最小回転半径は ±30° で 24 cm


@dataclass
class Config:
    camera: CameraConfig = field(default_factory=CameraConfig)
    server: ServerConfig = field(default_factory=ServerConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    car: CarConfig = field(default_factory=CarConfig)
    # 声（voice.py）。設定ファイルでは [voice]
    voice: VoiceConfig = field(default_factory=VoiceConfig)
    # map で撮った画像の置き場所（map_日時/ に画像と scan.json）
    map_dir: str = "~/captures/snoo_map"
    seen_dir: str = "~/captures/snoo_seen"  # 追い始めたときの画像と箱
    # フレームごとの区間の時刻（trace.py）を JSON Lines で残すディレクトリ。空なら残さない（集計は出す）
    trace_dir: str = ""
    # explore の続き（resume.py）。起動し直したとき、これが 10 分より新しければ
    # 同じセッションで続ける。空なら使わない
    explore_state: str = "~/.local/state/snoo_trucar/explore.json"


# 前の版にあった項目。Pi の config.toml に残っていても止めずに無視する（超音波で止まる・Pi だけで探る
# 決まりを消した。2026-10-31）
REMOVED = {
    "control.stop_distance_cm", "control.explore_step_cm", "control.explore_clear_cm",
    "control.explore_turn_s", "control.explore_server", "control.explore_step_stop_cm",
}  # fmt: skip


def _merge(obj, values: dict, where: str):
    names = {f.name: f for f in dataclasses.fields(obj)}
    for key, value in values.items():
        if f"{where}{key}" in REMOVED:
            print(f"設定の {where}{key} は使わなくなったので無視する", flush=True)
            continue
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
