"""行動と制御則。推定した snoo の方位・距離から、カメラの向き・ハンドル・速さを決める。

状態:
- SEARCH: 見つけていない。カメラを左右に振る（search_drive なら時々車体の向きも変える）
- TRACK: 見えている。カメラを向け、mode が keep / follow なら距離を保つ
- COAST: 少し前に見失った。予測した位置にカメラを向けたまま止まって待つ
- IDLE: 待機（mode = idle）。止まってカメラを正面に戻す
- MAP: 部屋を撮る（mode = map）。その場でカメラを決まった向きに順に向け、各向きで止まる。
  撮って保存するのはメインプロセス（scan_index が変わって落ち着いたら 1 枚）。一巡したら IDLE に戻る
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .config import CarConfig, ControlConfig
from .estimator import TargetEstimator
from .guard import OK, Guard

SEARCH, TRACK, COAST, IDLE, MAP = 0, 1, 2, 3, 4
STATE_NAMES = {SEARCH: "SEARCH", TRACK: "TRACK", COAST: "COAST", IDLE: "IDLE", MAP: "MAP"}
# idle: 待機 / watch: 探してカメラで追う / keep: 距離を保つ（近づく・離れる）/ follow: 追いかける /
# map: 部屋を撮る
MODES = ("idle", "watch", "keep", "follow", "map")
MAP_PANS = (-35.0, -25.0, -15.0, -5.0, 5.0, 15.0, 25.0, 35.0)
MAP_TILTS = (0.0, 25.0)
MAP_HOLD_S = 1.0


@dataclass
class Command:
    steer: float = 0.0
    speed: float = 0.0
    pan: float = 0.0
    tilt: float = 0.0
    state: int = SEARCH
    scan_index: int = -1  # MAP のとき、今どの向きか
    guard: int = OK  # 安全の層の状態（guard.py）


def clip(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def ego_yaw_rate(steer_deg: float, speed: float, car: CarConfig) -> float:
    """指令（ハンドル・速さ）から出した車体の角速度（度/秒、右回りが +）。自転車モデル。"""
    v = speed * car.cm_s_per_speed
    return math.degrees(v / car.wheelbase_cm * math.tan(math.radians(steer_deg)))


class Controller:
    PAN_RATE = 120.0  # 度/秒。サーボが追いつく速さに抑える

    def __init__(self, ctl: ControlConfig, car: CarConfig):
        if ctl.mode not in MODES:
            raise ValueError(f"mode は {MODES} のどれか: {ctl.mode}")
        self.ctl = ctl
        self.car = car
        self.mode = ctl.mode
        self.cmd = Command(tilt=ctl.search_tilt_deg)
        self._map_t = 0.0
        self._search_t = 0.0
        self._search_turns = 0
        self._arc_left = 0.0
        self.guard = Guard(ctl.guard, car.cm_s_per_speed)

    def observe_stress(self, stress: float | None) -> None:
        self.guard.observe_stress(stress)

    def set_mode(self, mode: str) -> None:
        if mode not in MODES:
            raise ValueError(mode)
        if mode != self.mode:
            self.mode = mode
            self._map_t = 0.0
            self._search_t = 0.0
            self._arc_left = 0.0

    def step(self, est: TargetEstimator, now_ns: int, dt: float, ultrasonic_cm: float | None):
        ctl, car, cmd = self.ctl, self.car, self.cmd
        since = est.since_seen_s(now_ns)
        cmd.scan_index = -1
        if self.mode == "idle":
            state = IDLE
        elif self.mode == "map":
            state = MAP
        elif est.valid and since < ctl.coast_s:
            state = TRACK
        elif est.valid and since < ctl.lost_s:
            state = COAST
        else:
            state = SEARCH
            if est.valid:
                est.forget()
        if state == TRACK and cmd.state == SEARCH:
            self._search_t = 0.0
            self._arc_left = 0.0

        steer = speed = 0.0
        if state == IDLE:
            pan_goal, tilt_goal = 0.0, 0.0
        elif state == MAP:
            pan_goal, tilt_goal = self._map(dt)
            if self.mode == "idle":  # 一巡した
                state = IDLE
        elif state == SEARCH:
            pan_goal, tilt_goal, steer, speed = self._search(dt)
        else:
            # 書いてから効くまでの遅れ（0 次ホールドとサーボ）の分だけ先を狙う。
            # 方位は snoo の動きと車体の回転の両方で変わる
            bearing = est.bearing.x[0] + (est.bearing.x[1] - est.yaw_rate) * ctl.lead_s
            pan_goal = bearing
            tilt_goal = est.elev.x[0] + est.elev.x[1] * ctl.lead_s
            if state == TRACK and self.mode in ("keep", "follow"):
                steer, speed = self._approach(bearing, est.dist.x[0], ultrasonic_cm)

        # 走る行動の出力は、最後に必ず安全の層を通す
        seen = est.valid and state == TRACK
        steer, speed = self.guard.apply(
            self.mode in ("keep", "follow"), dt, steer, speed,
            est.dist.x[0] if seen else None, est.dist.x[1] if seen else None,
        )
        cmd.guard = self.guard.state

        # カメラは目標に一次遅れで寄せ、速さも抑える
        max_step = self.PAN_RATE * dt
        cmd.pan = clip(
            cmd.pan + clip(ctl.pan_kp * (pan_goal - cmd.pan) * dt, -max_step, max_step),
            car.pan_min,
            car.pan_max,
        )
        cmd.tilt = clip(
            cmd.tilt + clip(ctl.pan_kp * (tilt_goal - cmd.tilt) * dt, -max_step, max_step),
            car.tilt_min,
            car.tilt_max,
        )
        cmd.steer = clip(steer, -car.steer_max, car.steer_max)
        cmd.speed = clip(speed, -ctl.max_speed, ctl.max_speed)
        if ultrasonic_cm is not None and ultrasonic_cm < ctl.stop_distance_cm:
            cmd.speed = min(cmd.speed, 0.0)
        cmd.state = state
        return cmd

    def _approach(self, bearing: float, dist: float, ultrasonic_cm: float | None):
        """距離のループ（速さ）と、車体の向きのループ（ハンドル）。

        cascade（snoo_trucar#18）では、車体の向きは「首（方位）を 0° に戻す」ループとして
        距離から切り離し、帯域を heading_hz に揃える。前輪で曲がる車は止まると
        向きを変えられないので、距離が目標の内でも方位が turn_creep_deg を超えたら、
        creep_speed でゆっくり進んで（近すぎれば下がって）向きを変える。
        """
        ctl = self.ctl
        goal = ctl.follow_distance_cm if self.mode == "follow" else ctl.keep_distance_cm
        err = dist - goal
        if abs(err) < ctl.distance_deadband_cm:
            speed = 0.0
        else:
            speed = ctl.speed_kp * (err - math.copysign(ctl.distance_deadband_cm, err))
        if self.mode == "follow":
            speed = max(speed, 0.0)  # 追いかけるときは下がらない
        if not ctl.cascade:
            if speed == 0.0:
                return 0.0, 0.0
            steer = ctl.steer_kp * bearing
            if speed > 0:
                # 横にいるときは曲がることを優先して遅くする
                speed *= max(0.3, math.cos(math.radians(min(abs(bearing), 90.0))))
            else:
                steer = -steer  # 下がるときは逆に切ると、車体が snoo の方を向く
            return steer, speed
        if abs(speed) < ctl.creep_speed and abs(bearing) > ctl.turn_creep_deg:
            # 首だけでは追いきれない角度。向きを変えるために、ゆっくり動く
            back = err < 0 and self.mode != "follow"
            speed = -ctl.creep_speed if back else ctl.creep_speed
        elif speed > 0:
            # 横にいるときは曲がることを優先して遅くする
            speed *= max(0.3, math.cos(math.radians(min(abs(bearing), 90.0))))
        v = abs(speed) * self.car.cm_s_per_speed
        if v < 1e-6:
            return 0.0, 0.0
        # 車体の向きを heading_hz の帯域で方位に寄せる。
        # 角速度 = 2π f × 方位 を、自転車モデルでハンドルの角度に直す
        yaw_rate = 2 * math.pi * ctl.heading_hz * math.radians(bearing)
        steer = math.degrees(math.atan(yaw_rate * self.car.wheelbase_cm / v))
        if speed < 0:
            steer = -steer  # 下がるときは逆に切ると、車体が snoo の方を向く
        return steer, speed

    def _search(self, dt: float):
        ctl = self.ctl
        steer = speed = 0.0
        if self._arc_left > 0:
            self._arc_left -= dt
            steer = self.car.steer_max
            speed = ctl.search_drive_speed
            return 0.0, ctl.search_tilt_deg, steer, speed
        self._search_t += dt
        phase = 2 * math.pi * self._search_t / ctl.search_period_s
        pan = ctl.search_pan_deg * math.sin(phase)
        if ctl.search_drive and self._search_t >= ctl.search_period_s:
            # 一往復しても見つからなければ、少し進みながら向きを変える
            self._search_t = 0.0
            self._search_turns += 1
            self._arc_left = 1.0
        return pan, ctl.search_tilt_deg, steer, speed

    def _map(self, dt: float):
        poses = [(p, t) for t in MAP_TILTS for p in MAP_PANS]
        self._map_t += dt
        i = int(self._map_t / MAP_HOLD_S)
        if i >= len(poses):
            self.set_mode("idle")
            return 0.0, 0.0
        self.cmd.scan_index = i
        return poses[i]
