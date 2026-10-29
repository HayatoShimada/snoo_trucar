"""行動と制御則。推定した snoo の方位・距離から、カメラの向き・ハンドル・速さを決める。

状態:
- SEARCH: 見つけていない。カメラを左右に振る（search_drive なら時々車体の向きも変える）
- TRACK: 見えている。カメラを向け、mode が keep / follow なら距離を保つ
- COAST: 少し前に見失った。予測した位置にカメラを向けたまま止まって待つ
- IDLE: 待機（mode = idle）。止まってカメラを正面に戻す
- MAP: 部屋を撮る（mode = map）。その場でカメラを決まった向きに順に向け、各向きで止まる。
  撮って保存するのはメインプロセス（scan_index が変わって落ち着いたら 1 枚）。一巡したら IDLE に戻る
- EXPLORE: 部屋を動きながら撮る（mode = explore）。止まって首を振って撮る → 超音波で前が空いていれば
  explore_step_cm 進む（その間も撮る）→ 塞がっていれば、ハンドルを切って下がって向きを変える、を繰り返す。
  超音波が測れない（-1）ときは前に進まない。snoo が explore_snoo_near_cm より近くに見えたら、
  止まってカメラで追うだけにする（臆病なので）
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass

from .config import CarConfig, ControlConfig
from .estimator import TargetEstimator
from .guard import OK, Guard

SEARCH, TRACK, COAST, IDLE, MAP, EXPLORE = 0, 1, 2, 3, 4, 5
STATE_NAMES = {SEARCH: "SEARCH", TRACK: "TRACK", COAST: "COAST", IDLE: "IDLE", MAP: "MAP",
               EXPLORE: "EXPLORE"}  # fmt: skip
# idle: 待機 / watch: 探してカメラで追う / keep: 距離を保つ（近づく・離れる）/ follow: 追いかける /
# map: 部屋を撮る / explore: 部屋を動きながら撮る（共有メモリでは番号なので、足すときは後ろに）
MODES = ("idle", "watch", "keep", "follow", "map", "explore")
MAP_PANS = (-35.0, -25.0, -15.0, -5.0, 5.0, 15.0, 25.0, 35.0)
# チルトは picar-x.conf の較正（picarx_cam_tilt_servo = 26.5）で 0° が水平。床・壁・家具の高さを撮る
MAP_TILTS = (-15.0, 0.0, 15.0)
# パンを 10° 回すのに要る時間より長く。1.0 s では回りきる前に撮っていた（2026-10-29、VGGT で 0.88 倍）
MAP_HOLD_S = 1.6
# explore で止まるたびに撮る向き（間取りには水平とやや下が要る）
EXPLORE_PANS = (-30.0, -15.0, 0.0, 15.0, 30.0)
EXPLORE_TILTS = (-15.0, 0.0)
# 段ごとに折り返す（2 段目は右から左）。隣の向きとの差がいつも 15° 以内になる
EXPLORE_POSES = tuple(
    (p, t) for k, t in enumerate(EXPLORE_TILTS) for p in (EXPLORE_PANS if k % 2 == 0 else EXPLORE_PANS[::-1])
)
# explore で向きを変えてから撮るまでの待ち = BASE + PER_DEG × 角度の差。指令は滑らかにせず、すぐ書く。
# 撮ったら（CAPTURE_S の間に 1 枚）次の向きへ。これまでは 1 向き 1.6 s 固定で、1 か所 16 s かかっていた
SCAN_SETTLE_BASE_S = 0.35
SCAN_SETTLE_PER_DEG_S = 0.01
SCAN_CAPTURE_S = 0.12


def scan_settle_s(delta_deg: float) -> float:
    return SCAN_SETTLE_BASE_S + SCAN_SETTLE_PER_DEG_S * abs(delta_deg)


def scan_schedule(pan0: float, tilt0: float) -> list[float]:
    """explore の 1 か所の撮影で、各向きを終える時刻（撮り始めからの秒）"""
    ends, t, prev = [], 0.0, (pan0, tilt0)
    for p, ti in EXPLORE_POSES:
        t += scan_settle_s(max(abs(p - prev[0]), abs(ti - prev[1]))) + SCAN_CAPTURE_S
        ends.append(t)
        prev = (p, ti)
    return ends
# 超音波の -1（反響なし）は、前が 3.4 m より遠くまで空いているときにも出る（robot_hat の待ちが 20 ms）。
# この間に 1 度でも測れていれば、センサーは生きているとみなし、-1 を遠い（EXPLORE_FAR_CM）として扱う。
# 1 度も測れていなければ（外れている・壊れている）、前にも後ろにも動かない。
# 60 s では、間に置かれた物を見落としたまま長く走れた（snoo_trucar#21 の B）。今は 1 s。
# 測った値が古い（us_stale: スレッドが止まった・読めない）のは -1 と違い、いつも分からない（進まない）
EXPLORE_US_TRUST_S = 1.0
EXPLORE_FAR_CM = 300.0
EXPLORE_CHECK_S = 0.3  # 切り返しのあと、止まって超音波を測り直す時間
EXPLORE_MAX_TURNS = 8  # 切り返しをこれだけしても前が空かなければ、その場で撮り直す
EXPLORE_PLAN_TIMEOUT_S = 25.0  # サーバーの返事をこれだけ待って来なければ、Pi だけで決める（サーバーは 20 s で返す）
PLAN_NONE, PLAN_MOVE, PLAN_DONE, PLAN_FALLBACK = 0, 1, 2, 3
MAX_SEGMENTS = 3  # サーバーの 1 歩の区切り（segments）の数の上限
STOP_REASONS = ("", "done", "ultrasonic", "clear", "clear_unknown", "stuck", "clear_skip")
CLEAR_NULL_STOP_NS = 300_000_000
# 画像の通り道の空き（clear_cm）の状態。ABSENT は返事に欄が無い（clear_cm を出さないサーバー）、
# SKIP は clear_skip（settling・out_of_view・reversing で出せない）、STALE は返事が途切れて古い・まだ来ていない
CLEAR_ABSENT, CLEAR_VALUE, CLEAR_NULL, CLEAR_SKIP, CLEAR_STALE = 0, 1, 2, 3, 4
# clear_skip が続く間は止めないが、見えていないので遅くし、続けて走る距離にも上限を付ける。
# 超音波が新しく CLEAR_SKIP_US_OK_CM より先を測れているときは、上限を付けない
CLEAR_SKIP_SPEED = 10.0
CLEAR_SKIP_MAX_CM = 20.0
CLEAR_SKIP_US_OK_CM = 60.0
# clear_cm が近くても、超音波がそれより LOW_OBSTACLE_GAP_CM 以上先なら低い物とみなし、
# LOW_OBSTACLE_SPEED に落として LOW_OBSTACLE_CM だけ越える（2026-10-29、ラグの縁で止まっていた）
LOW_OBSTACLE_GAP_CM = 25.0
LOW_OBSTACLE_SPEED = 10.0
LOW_OBSTACLE_CM = 20.0
# 越えるのは 1 歩に 1 回まで。越えている間に、画像の空きが（進んだ分を引いても）これより縮んだら止まる。
# ソファ・ベッドの下のように上にかぶさる物は、超音波（低い）では先まで空いて見えるので取り違えうる
LOW_OBSTACLE_SHRINK_CM = 5.0
DRIVE_INDEX = -2
# 走っている間のカメラ。サーバーが画像から通り道の空き（clear_cm）を出すので、少し下を向けて車の前を入れる
# （HFOV 54° だと車幅が画面に入るのは、tilt -15° で前端から約 13 cm 先から）。曲がるときは曲がる方へ首を向ける
DRIVE_TILT = -15.0
DRIVE_PAN_PER_STEER = 1.5  # 曲がる弧の通り道が画面に長く入るように（home-linux の clear_cm）
DRIVE_PAN_MAX = 30.0


def _drive_pan(steer_deg: float) -> float:
    return max(-DRIVE_PAN_MAX, min(DRIVE_PAN_MAX, DRIVE_PAN_PER_STEER * steer_deg))  # 走っている間の scan_index（メインプロセスが一定の間隔で撮る）


@dataclass
class Command:
    steer: float = 0.0
    speed: float = 0.0
    pan: float = 0.0
    tilt: float = 0.0
    state: int = SEARCH
    scan_index: int = -1  # MAP / EXPLORE のとき、今どの向きか（通し番号）。走っている間は DRIVE_INDEX
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
        self._ex_phase, self._ex_t, self._ex_stop = "scan", 0.0, 0  # explore の段階・経過・止まった回数
        self._ex_total = 0.0
        self._ex_moved = 0.0
        self._ex_turn_dir = 1.0
        self._ex_us_at = -1e9
        self._ex_turns = 0
        self.stuck = False  # 走らせているのに画像が動かない（メインプロセスの motion.py が見る）
        self.us_stale = False  # 超音波の新しい値（-1 を含む）が来ていない。control_proc が入れる
        # サーバーが画像から出す通り道の空き（protocol.Reply.clear_cm）
        self.clear_state, self.clear_cm = CLEAR_ABSENT, -1.0
        self._clear_null_since: int | None = None  # null・古いが始まった時刻
        self._clear_t = 0  # 今の clear を撮った時刻（古さを見る）
        self._skip_moved = 0.0  # clear_skip が続いている間に進んだ距離（cm）
        self._step_start_ns = 0  # サーバーの 1 歩を走り始めた時刻
        # サーバーが決める探索（explore_server）。station_wait は返事を待っている station（-1 で待っていない）。
        # plan は control_proc が入れる (station, action, steer_deg, speed, distance_cm)
        self.station_wait = -1
        self.plan = None
        self.last_step = (0.0, 0)  # 直前の 1 歩の (秒, STOP_REASONS の番号)
        self._ex_step: tuple[float, list[tuple[float, float]]] = (0.0, [])  # (速さ, [(ハンドル, cm)])
        self._ex_seg = 0
        self._ex_sched: list[float] = []
        self._ex_sched_stop = -1
        self._low_left = 0.0  # 低い物を越えている残りの距離（cm）
        self._low_clear_cm = 0.0  # 越え始めたときの clear_cm
        self.low_passed = 0  # この 1 歩で低い物を越えた回数（次の station で伝える）
        self._now_ns = 0  # 超音波が最後に測れた時刻（explore の経過時間）
        self.guard = Guard(ctl.guard, car.cm_s_per_speed)

    def set_clear(self, state: int, cm: float, now_ns: int, t_capture_ns: int = 0) -> None:
        """state は CLEAR_*。t_capture_ns はその返事の画像を撮った時刻（age_clear で古さを見る）"""
        if state in (CLEAR_NULL, CLEAR_STALE):
            if self._clear_null_since is None:
                self._clear_null_since = now_ns
        else:
            self._clear_null_since = None
        self.clear_state, self.clear_cm = state, cm
        self._clear_t = t_capture_ns

    def age_clear(self, now_ns: int, max_age_ns: int) -> None:
        """撮ってから max_age_ns より古い clear は、分からない（STALE）にする"""
        if self._clear_t and now_ns - self._clear_t > max_age_ns:
            self.set_clear(CLEAR_STALE, -1.0, now_ns)

    def clear_blocked(self, strict: bool = False) -> str | None:
        """走っている間に、画像の通り道の空きから止まるべきか。理由を返す。

        strict はサーバーの 1 歩を走っているとき。サーバーが画像を見ている前提なので、返事が途切れた
        （STALE）のも null と同じく分からないとする。Pi だけの探索では、サーバーに届かないのが普通なので
        STALE は欄が無いのと同じ（超音波だけで走る）"""
        ctl = self.ctl
        if self.clear_state == CLEAR_VALUE and self.clear_cm < ctl.explore_clear_stop_cm:
            return "near"
        unknown = self.clear_state == CLEAR_NULL or (strict and self.clear_state == CLEAR_STALE)
        if unknown and self._clear_null_since is not None:
            since = self._clear_null_since
            if strict:
                since = max(since, self._step_start_ns)  # 走り始めてから 0.3 s は待つ（撮ったばかりなので）
            if self._now_ns - since >= CLEAR_NULL_STOP_NS:
                return "unknown"
        return None

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
            self._reset_explore()

    def _reset_explore(self) -> None:
        """explore の状態をすべて始めに戻す。前の explore の 1 歩・待っている station・撮る予定が
        残っていると、次の explore の station 0 で、サーバーに聞かずに前の 1 歩を走ってしまう"""
        self._ex_phase, self._ex_t, self._ex_stop = "scan", 0.0, 0
        self._ex_total = 0.0
        self._ex_moved = 0.0
        self._ex_turns = 0
        self._ex_us_at = -1e9  # _ex_total で数えるので、始め直したら前の時刻は使えない
        self.station_wait = -1
        self.plan = None
        self.last_step = (0.0, 0)
        self._ex_step = (0.0, [])
        self._ex_seg = 0
        self._ex_sched, self._ex_sched_stop = [], -1
        self._low_left, self.low_passed, self._low_clear_cm = 0.0, 0, 0.0

    def set_plan(self, plan) -> None:
        """サーバーの 1 歩（control_proc が plan の共有メモリから入れる）。返事を待っている station の分
        だけ受ける。待っていないときに来たもの（前の explore の遅れた返事など）は捨てる"""
        if self.station_wait >= 0 and int(plan[0]) == self.station_wait:
            self.plan = tuple(plan)

    def step(self, est: TargetEstimator, now_ns: int, dt: float, ultrasonic_cm: float | None):
        ctl, car, cmd = self.ctl, self.car, self.cmd
        self._now_ns = now_ns
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
        if self.mode == "explore":
            # snoo が近いときだけ止まって見守る。遠くにいるなら探索を続ける
            far = state != SEARCH and est.dist.x[0] > ctl.explore_snoo_near_cm
            if state == SEARCH or far:
                state = EXPLORE
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
        elif state == EXPLORE:
            pan_goal, tilt_goal, steer, speed = self._explore(dt, ultrasonic_cm)
            if self.mode == "idle":  # 時間切れ
                state = IDLE
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

        if state == EXPLORE and self._ex_phase == "scan":
            # 撮るために止まって向きを変えるときは、滑らかにせずすぐ書く（サーボは自分の速さで回る）
            cmd.pan = clip(pan_goal, car.pan_min, car.pan_max)
            cmd.tilt = clip(tilt_goal, car.tilt_min, car.tilt_max)
            pan_goal, tilt_goal = cmd.pan, cmd.tilt
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

    def _explore(self, dt: float, us_cm: float | None):
        ctl, car = self.ctl, self.car
        self._ex_total += dt
        if self._ex_total >= ctl.explore_max_s:
            self.set_mode("idle")
            return 0.0, 0.0, 0.0, 0.0
        self._ex_t += dt
        us_real = us_cm  # 実際に測れた値（-1 を遠いとみなす前）
        if us_cm is not None:
            self._ex_us_at = self._ex_total
        elif not self.us_stale and self._ex_total - self._ex_us_at <= EXPLORE_US_TRUST_S:
            us_cm = EXPLORE_FAR_CM
        if self._ex_phase == "scan":
            poses = EXPLORE_POSES
            if self._ex_sched_stop != self._ex_stop:
                self._ex_sched_stop = self._ex_stop
                self._ex_sched = scan_schedule(self.cmd.pan, self.cmd.tilt)
            i = bisect.bisect_right(self._ex_sched, self._ex_t)
            if i < len(poses):
                self.cmd.scan_index = self._ex_stop * len(poses) + i
                return (*poses[i], 0.0, 0.0)
            self._ex_stop += 1
            self._ex_t, self._ex_moved = 0.0, 0.0
            if ctl.explore_server:
                # 撮った分をメインプロセスがサーバーに送り、次の 1 歩をもらうまで止まって待つ
                self._ex_phase, self.station_wait = "plan", self._ex_stop - 1
                self.cmd.scan_index = -1
                return 0.0, 0.0, 0.0, 0.0
            if not self._ex_local_next(us_cm):
                return 0.0, 0.0, 0.0, 0.0
        if self._ex_phase == "plan":
            self.cmd.scan_index = -1
            p = self.plan
            if p is not None and int(p[0]) == self.station_wait:
                self.station_wait = -1
                action = int(p[1])
                if action == PLAN_DONE:
                    self.set_mode("idle")  # 見ていない所が無い・電池が少ない など（理由はメインプロセスが出す）
                    return 0.0, 0.0, 0.0, 0.0
                if action == PLAN_MOVE:
                    # 速さが負なら下がる（前に行ける所が無いとき、来た弧を同じハンドルで戻る。後ろは見えないので短く）
                    back = p[2] < 0
                    speed = abs(p[2]) if 0 < abs(p[2]) <= ctl.explore_speed else ctl.explore_speed
                    # 区切りごとの距離は、合わせて 1 歩の上限まで（後ろは見えないので、下がるときはもっと短く）
                    left = ctl.explore_reverse_max_cm if back else ctl.explore_server_step_cm
                    segs = []
                    for i in range(min(int(p[3]), MAX_SEGMENTS)):
                        steer = max(-car.steer_max, min(car.steer_max, p[4 + 2 * i]))
                        d = min(max(0.0, p[5 + 2 * i]), left)
                        left -= d
                        if d > 0:
                            segs.append((steer, d))
                    self._ex_step = (-speed if back else speed, segs)
                    self._ex_phase, self._ex_t, self._ex_moved, self._ex_seg = "step", 0.0, 0.0, 0
                    self._low_left, self.low_passed = 0.0, 0
                    self._step_start_ns, self._skip_moved = self._now_ns, 0.0
                elif not self._ex_local_next(us_cm):  # サーバーに届かない → Pi だけの探索
                    return 0.0, 0.0, 0.0, 0.0
            elif self._ex_t >= EXPLORE_PLAN_TIMEOUT_S:
                self.station_wait = -1
                if not self._ex_local_next(us_cm):
                    return 0.0, 0.0, 0.0, 0.0
            else:
                return 0.0, 0.0, 0.0, 0.0
        self.cmd.scan_index = DRIVE_INDEX
        if self._ex_phase == "step":
            # サーバーが決めた 1 歩（弧）。止まったら、理由を次の station で伝える。
            # 前の超音波と画像の空きは、前に進むときだけ見る（下がるときは、来た所を戻るだけ）
            speed, segs = self._ex_step
            while self._ex_seg < len(segs) and self._ex_moved >= segs[self._ex_seg][1]:
                self._ex_seg, self._ex_moved = self._ex_seg + 1, 0.0  # 次の区切りへ
            steer = segs[self._ex_seg][0] if self._ex_seg < len(segs) else 0.0
            fwd = speed > 0
            blocked = self.clear_blocked(strict=True)
            skip = fwd and self.clear_state == CLEAR_SKIP
            if not skip:
                self._skip_moved = 0.0
            # clear_skip の間は画像で前を見ていない。超音波が遠くまで空いていなければ遅く・短く
            skip_capped = skip and not (us_real is not None and us_real > CLEAR_SKIP_US_OK_CM)
            reason = None
            if self.stuck:
                reason, self.stuck = "stuck", False
            elif fwd and (us_cm is None or us_cm < ctl.explore_step_stop_cm):
                reason = "ultrasonic"
            elif fwd and blocked == "near" and self._low_left <= 0:
                low = us_real is not None and us_real >= self.clear_cm + LOW_OBSTACLE_GAP_CM
                if low and self.low_passed == 0:
                    # 画像では近いのに、超音波（床から 4.5 cm）はずっと先を見ている → 低い物（ラグの縁・
                    # ケーブル・敷居）。止まらずに遅くして越える。引っかかれば画像が動かなくなって止まる
                    self._low_left = LOW_OBSTACLE_CM
                    self._low_clear_cm = self.clear_cm
                    self.low_passed += 1
                else:
                    reason = "clear"
            elif (fwd and self._low_left > 0 and self.clear_state == CLEAR_VALUE
                  and self.clear_cm < self._low_clear_cm - (LOW_OBSTACLE_CM - self._low_left)
                  - LOW_OBSTACLE_SHRINK_CM):  # fmt: skip
                reason = "clear"  # 越えている物より手前に、別の物が見えた
            elif fwd and blocked == "unknown":
                reason = "clear_unknown"
            elif skip_capped and self._skip_moved >= CLEAR_SKIP_MAX_CM:
                reason = "clear_skip"
            elif self._ex_seg >= len(segs):
                reason = "done"
            if reason is not None:
                self.last_step = (self._ex_t, STOP_REASONS.index(reason))
                self._ex_phase, self._ex_t = "scan", 0.0
                return 0.0, 0.0, 0.0, 0.0
            if self._low_left > 0:
                speed = min(speed, LOW_OBSTACLE_SPEED)
                self._low_left -= speed * car.cm_s_per_speed * dt
            if skip_capped:
                speed = min(speed, CLEAR_SKIP_SPEED)
            if skip:
                self._skip_moved += speed * car.cm_s_per_speed * dt
            self._ex_moved += abs(speed) * car.cm_s_per_speed * dt
            return (_drive_pan(steer) if fwd else 0.0), DRIVE_TILT, steer, speed
        if self.stuck and self._ex_phase in ("drive", "fwd", "back"):
            # ぶつかった・詰まった。進んでいたなら下がって向きを変え、下がっていたなら止まって測り直す
            self._ex_phase = "check" if self._ex_phase == "back" else "back"
            self._ex_t = 0.0
            self.stuck = False  # 次の判定はメインプロセスが改めて出す
        blocked = self.clear_blocked() if self._ex_phase in ("drive", "fwd") else None
        if blocked == "near":
            self._ex_phase, self._ex_t, self._ex_turns = "back", 0.0, 0  # 画像で前が近い
        elif blocked == "unknown":
            self._ex_phase, self._ex_t = "scan", 0.0  # 画像の判定が届かない。止まってその場で撮り直す
            return 0.0, 0.0, 0.0, 0.0
        if self._ex_phase == "drive":
            if us_cm is None and self.us_stale:
                self._ex_phase, self._ex_t = "scan", 0.0  # 超音波が届かない。前が分からないので止まる
                return 0.0, 0.0, 0.0, 0.0
            if us_cm is None or us_cm < ctl.explore_clear_cm:
                self._ex_phase, self._ex_t, self._ex_turns = "back", 0.0, 0
            else:
                self._ex_moved += ctl.explore_speed * car.cm_s_per_speed * dt
                if self._ex_moved >= ctl.explore_step_cm:
                    # 新しい場所に着いた。次に向きを変えるときは逆回りにして、片寄らないようにする
                    self._ex_turn_dir = -self._ex_turn_dir
                    self._ex_phase, self._ex_t = "scan", 0.0
                    return 0.0, 0.0, 0.0, 0.0
                return 0.0, DRIVE_TILT, 0.0, ctl.explore_speed
        # 前が塞がっている。切り返しで同じ向きに回り続ける（ハンドルを切って下がる → 逆に切って進む）。
        # 下がるのは後ろが見えないので短く。進む間は超音波が近ければやめる
        d = self._ex_turn_dir * car.steer_max
        if self._ex_phase == "back":
            # 下がるのはサーバーの 1 歩と同じく explore_reverse_max_cm まで（1.5 s では 38 cm 下がっていた）
            back_s = min(ctl.explore_turn_s,
                         ctl.explore_reverse_max_cm / (ctl.explore_speed * car.cm_s_per_speed))  # fmt: skip
            if self._ex_t < back_s:
                return 0.0, DRIVE_TILT, d, -ctl.explore_speed
            self._ex_phase, self._ex_t = "fwd", 0.0
        if self._ex_phase == "fwd":
            if self._ex_t < ctl.explore_turn_s and us_cm is not None and us_cm >= ctl.explore_clear_cm:
                return _drive_pan(-d), DRIVE_TILT, -d, ctl.explore_speed
            self._ex_phase, self._ex_t = "check", 0.0
        if self._ex_phase == "check":
            if self._ex_t < EXPLORE_CHECK_S:  # 止まって超音波を測り直す
                return 0.0, 0.0, 0.0, 0.0
            self._ex_turns += 1
            self._ex_t = 0.0
            if us_cm is not None and us_cm >= ctl.explore_clear_cm + ctl.explore_step_cm:
                self._ex_phase, self._ex_moved = "drive", 0.0
            elif us_cm is not None and self._ex_turns < EXPLORE_MAX_TURNS:
                self._ex_phase = "back"
            else:
                # 一回りしても空いていない・超音波が分からない。ここで撮り直す
                self._ex_phase = "scan"
        return 0.0, 0.0, 0.0, 0.0

    def _ex_local_next(self, us_cm: float | None) -> bool:
        """Pi だけで次を決める（サーバーを使わない・届かないとき）。動くなら True"""
        ctl = self.ctl
        self._ex_t, self._ex_moved = 0.0, 0.0
        if us_cm is not None and us_cm >= ctl.explore_clear_cm + ctl.explore_step_cm:
            self._ex_phase = "drive"
        elif us_cm is not None:
            self._ex_phase, self._ex_turns = "back", 0
        else:
            # 超音波がずっと測れない（壊れている・つながっていない）。前も見えないので動かず、その場で撮り直す
            self._ex_phase = "scan"
            return False
        return True

    def _map(self, dt: float):
        poses = [(p, t) for t in MAP_TILTS for p in MAP_PANS]
        self._map_t += dt
        i = int(self._map_t / MAP_HOLD_S)
        if i >= len(poses):
            self.set_mode("idle")
            return 0.0, 0.0
        self.cmd.scan_index = i
        return poses[i]
