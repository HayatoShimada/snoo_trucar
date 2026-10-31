"""行動と制御則。推定した snoo の方位・距離から、カメラの向き・ハンドル・速さを決める。

状態:
- SEARCH: 見つけていない。カメラを左右に振る（search_drive なら時々車体の向きも変える）
- TRACK: 見えている。カメラを向け、mode が keep / follow なら距離を保つ
- COAST: 少し前に見失った。予測した位置にカメラを向けたまま止まって待つ
- IDLE: 待機（mode = idle）。止まってカメラを正面に戻す
- MAP: 部屋を撮る（mode = map）。その場でカメラを決まった向きに順に向け、各向きで止まる。
  撮って保存するのはメインプロセス（scan_index が変わって落ち着いたら 1 枚）。一巡したら IDLE に戻る
- EXPLORE: 部屋を動きながら撮る（mode = explore）。止まって首を振って撮る → home-linux の地図のサーバーに
  送り、返ってきた 1 歩（区切りの列。区切りごとに前・後ろ）をそのとおり走る、を繰り返す。道はサーバーが
  すべて決め、Pi は決めない。Pi に残すのは反射の安全だけ：前へ走る区切りの clear_cm（カメラ）、guard、
  走ったのに画像が動かない（stuck）、電池・USR。超音波では止まらない（当てにならないので、地図の弱い
  証拠として送るだけ）。止まったら自分で別の道を考えず、走った分を次の station の prev_step で返して撮る。
  サーバーに届かないときは送り直して待つ（StationClient）。snoo が explore_snoo_near_cm より近くに
  見えたら、止まってカメラで追うだけにする（臆病なので）
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
# explore で止まるたびに撮る向きの既定（間取りには水平とやや下が要る）。ControlConfig の
# explore_scan_tilts・explore_scan_pans で変えられる（段ごとに折り返して撮る）
EXPLORE_PANS = (-30.0, -15.0, 0.0, 15.0, 30.0)
EXPLORE_TILTS = (-15.0, 0.0)


def scan_poses(tilts=EXPLORE_TILTS, pans=None, pan_min=-90.0, pan_max=90.0) -> tuple:
    """撮る向きの並び（パン, チルト）。段ごとに折り返す（2 段目は右から左）。pans は段ごとのパンの
    並び（None なら EXPLORE_PANS）。パンは pan_min..pan_max に収める（首の台が当たらない範囲）"""
    rows = pans if pans else [EXPLORE_PANS] * len(tilts)
    out = []
    for k, t in enumerate(tilts):
        row = sorted({min(pan_max, max(pan_min, float(p))) for p in rows[min(k, len(rows) - 1)]})
        out += [(p, float(t)) for p in (row if k % 2 == 0 else row[::-1])]
    return tuple(out)


EXPLORE_POSES = scan_poses()
# explore で向きを変えてから撮るまでの待ち = BASE + PER_DEG × 角度の差。指令は滑らかにせず、すぐ書く。
# 撮ったら（CAPTURE_S の間に 1 枚）次の向きへ。これまでは 1 向き 1.6 s 固定で、1 か所 16 s かかっていた
SCAN_SETTLE_BASE_S = 0.35
SCAN_SETTLE_PER_DEG_S = 0.01
SCAN_CAPTURE_S = 0.12


def scan_settle_s(delta_deg: float) -> float:
    return SCAN_SETTLE_BASE_S + SCAN_SETTLE_PER_DEG_S * abs(delta_deg)


def scan_schedule(pan0: float, tilt0: float, poses=EXPLORE_POSES) -> list[float]:
    """explore の 1 か所の撮影で、各向きを終える時刻（撮り始めからの秒）"""
    ends, t, prev = [], 0.0, (pan0, tilt0)
    for p, ti in poses:
        t += scan_settle_s(max(abs(p - prev[0]), abs(ti - prev[1]))) + SCAN_CAPTURE_S
        ends.append(t)
        prev = (p, ti)
    return ends
# 検出のサーバーは、走っている（flags に MOVING）フレームにだけ clear_cm（か clear_skip）を返す。
# 1 歩を始めた瞬間は止まっていたフレームの返事しか無く、clear の欄が無い（CLEAR_ABSENT）。
# 走り出してからこの間は、欄が無くても止めずに待つ（ふつう 0.1 s で skip が来る）。
# それより長く欄が無い（clear_cm を出さないサーバー）なら、見えていないので clear_skip と同じく遅く・短く
CLEAR_ABSENT_GRACE_S = 0.5
EXPLORE_PLAN_TIMEOUT_S = 25.0  # サーバーの返事をこれだけ待っても来なければ、待ち続けていると表に出す
# PLAN_RESHOOT: 止まって撮ったコマが足りない（0 枚など）ので、送らずに
# 同じ station をその場で撮り直す
PLAN_NONE, PLAN_MOVE, PLAN_DONE, PLAN_FALLBACK, PLAN_RESHOOT = 0, 1, 2, 3, 4
MAX_SEGMENTS = 5  # サーバーの 1 歩の区切り（segments）の数の上限
# plan の共有メモリの 1 区切りの数（ハンドル、符号付きの cm（負は後ろへ）、clear_cm の下限（無ければ -1））
PLAN_SEG_N = 3
# "ultrasonic" は前の版の止まる理由（超音波では止まらない。番号は変えない）
STOP_REASONS = ("", "done", "ultrasonic", "clear", "clear_unknown", "stuck", "clear_skip")
CLEAR_NULL_STOP_NS = 300_000_000
# 画像の通り道の空き（clear_cm）の状態。ABSENT は返事に欄が無い（clear_cm を出さないサーバー）、
# SKIP は clear_skip（settling・out_of_view・reversing で出せない）、STALE は返事が途切れて古い・まだ来ていない
CLEAR_ABSENT, CLEAR_VALUE, CLEAR_NULL, CLEAR_SKIP, CLEAR_STALE = 0, 1, 2, 3, 4
# clear_skip（と欄が無い）が続く間は止めないが、見えていないので遅くし、続けて走る距離にも上限を付ける
CLEAR_SKIP_SPEED = 10.0
CLEAR_SKIP_MAX_CM = 20.0
# 前と後ろを切り替えるときに、止まって待つ時間（モーターを急に逆に回さない）
GEAR_PAUSE_S = 0.3
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
        self.stuck = False  # 走らせているのに画像が動かない（メインプロセスの motion.py が見る）
        # 撮る向きの並び（設定 explore_scan_tilts・explore_scan_pans。パンは首の範囲に収める）
        self.scan_poses = scan_poses(ctl.explore_scan_tilts, ctl.explore_scan_pans, car.pan_min, car.pan_max)
        # メインプロセスが速さを書いてよいか（電池が少ない・入っていないと 0 で書く）。control_proc が入れる
        self.drive_ok = True
        # 進んだ距離は、guard と電池を通した後の速さで数える（step の最後）。_explore が走らせた回だけ
        self._drive_count: str | None = None  # "step" / "drive"
        self._skip_now = False
        # 直前のサーバーの 1 歩: どの station の返事か（-1 は走っていない）、区切りごとに走った距離（cm）
        self.step_station = -1
        self.step_driven = [0.0] * MAX_SEGMENTS
        # サーバーが画像から出す通り道の空き（protocol.Reply.clear_cm）
        self.clear_state, self.clear_cm = CLEAR_ABSENT, -1.0
        self._clear_null_since: int | None = None  # null・古いが始まった時刻
        self._clear_t = 0  # 今の clear を撮った時刻（古さを見る）
        self._skip_moved = 0.0  # clear_skip が続いている間に進んだ距離（cm）
        self._step_start_ns = 0  # サーバーの 1 歩を走り始めた時刻
        # サーバーが決める探索。station_wait は返事を待っている station（-1 で待っていない）。
        # plan は control_proc が入れる (station, action, steer_deg, speed, distance_cm)
        self.station_wait = -1
        self.plan = None
        self.last_step = (0.0, 0)  # 直前の 1 歩の (秒, STOP_REASONS の番号)
        # (前の速さ, 後ろの速さ, [(ハンドル, cm, 後ろへか, clear_cm の下限 or None)])
        self._ex_step: tuple[float, float, list[tuple[float, float, bool, float | None]]] = (0.0, 0.0, [])
        self._ex_seg = 0
        self._gear_wait = 0.0  # 前と後ろを切り替える前に止まって待つ残り（秒）
        self._last_dt = 0.0
        self._ex_sched: list[float] = []
        self._ex_sched_stop = -1
        self.low_passed = 0  # 前の版の「低い物を越えた回数」（今は越えない。prev_step では 0）
        self._now_ns = 0
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

    def clear_blocked(self, stop_cm: float | None = None) -> str | None:
        """前へ走っている間に、画像の通り道の空きから止まるべきか。理由を返す。stop_cm は止まる近さ
        （既定 explore_clear_stop_cm。サーバーの区切りの min_clear_cm がそれより大きければそれ）。
        返事が途切れた（STALE）のも null と同じく分からないとする。走り始めてから 0.3 s は待つ"""
        stop_cm = max(self.ctl.explore_clear_stop_cm, stop_cm or 0.0)
        if self.clear_state == CLEAR_VALUE and self.clear_cm < stop_cm:
            return "near"
        unknown = self.clear_state in (CLEAR_NULL, CLEAR_STALE)
        if unknown and self._clear_null_since is not None:
            since = max(self._clear_null_since, self._step_start_ns)
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

    def resume_explore(self, station: int, elapsed_s: float = 0.0) -> None:
        """起動し直した後、同じセッションの続き（station 番号）から explore を始める（resume.py）。
        まずその場で撮る。直前の 1 歩は分からないので、prev_step は無し（step_station = -1）。
        elapsed_s は前のプロセスで探った時間（explore_max_s を延ばさない）"""
        self.set_mode("explore")
        self._reset_explore()
        self._ex_stop = max(0, int(station))
        self._ex_total = max(0.0, float(elapsed_s))

    def _reset_explore(self) -> None:
        """explore の状態をすべて始めに戻す。前の explore の 1 歩・待っている station・撮る予定が
        残っていると、次の explore の station 0 で、サーバーに聞かずに前の 1 歩を走ってしまう"""
        self._ex_phase, self._ex_t, self._ex_stop = "scan", 0.0, 0
        self._ex_total = 0.0
        self._ex_moved = 0.0
        self.station_wait = -1
        self.plan = None
        self.last_step = (0.0, 0)
        self._ex_step = (0.0, 0.0, [])
        self._ex_seg = 0
        self._gear_wait = 0.0
        self._ex_sched, self._ex_sched_stop = [], -1
        self.low_passed = 0
        self.step_station, self.step_driven = -1, [0.0] * MAX_SEGMENTS

    def set_plan(self, plan) -> None:
        """サーバーの 1 歩（control_proc が plan の共有メモリから入れる）。返事を待っている station の分
        だけ受ける。待っていないときに来たもの（前の explore の遅れた返事など）は捨てる"""
        if self.station_wait >= 0 and int(plan[0]) == self.station_wait:
            self.plan = tuple(plan)

    def step(self, est: TargetEstimator, now_ns: int, dt: float, ultrasonic_cm: float | None = None):
        """ultrasonic_cm は snoo の距離の推定（estimator）にだけ使う。止まる判断には使わない"""
        ctl, car, cmd = self.ctl, self.car, self.cmd
        self._now_ns = now_ns
        self._last_dt = dt
        since = est.since_seen_s(now_ns)
        cmd.scan_index = -1
        self._drive_count = None
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
            pan_goal, tilt_goal, steer, speed = self._explore(dt)
            if self.mode == "idle":  # 時間切れ
                state = IDLE
        else:
            # 書いてから効くまでの遅れ（0 次ホールドとサーボ）の分だけ先を狙う。
            # 方位は snoo の動きと車体の回転の両方で変わる
            bearing = est.bearing.x[0] + (est.bearing.x[1] - est.yaw_rate) * ctl.lead_s
            pan_goal = bearing
            tilt_goal = est.elev.x[0] + est.elev.x[1] * ctl.lead_s
            if state == TRACK and self.mode in ("keep", "follow"):
                steer, speed = self._approach(bearing, est.dist.x[0])

        # 走る行動の出力は、最後に必ず安全の層を通す。snoo の距離は、見えている間はいつも渡す
        # （explore で遠くに見えて探索を続けている間も、30 cm の決まりと「速く離れた」が効くように）。
        # 遊びの時間は keep / follow と、explore で snoo が近くにいる（見失って lost_s まで）間だけ数える。
        # snoo のいない探索まで遊びに数えると、3 分で 10 分休んでしまう
        seen = est.valid and since < ctl.coast_s
        playing = self.mode in ("keep", "follow") or (
            self.mode == "explore" and est.valid and since < ctl.lost_s
        )
        steer, speed = self.guard.apply(
            playing, dt, steer, speed,
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
        # 超音波では止めない（当てにならない。2026-10-31 ユーザーの判断）。前を見るのは clear_cm と guard
        cmd.state = state
        if self._drive_count is not None:
            self._count_driven(cmd.speed, dt)
        return cmd

    def _count_driven(self, speed: float, dt: float) -> None:
        """サーバーの 1 歩で進んだ距離を、実際に書く速さで数える（区切りの向きに進んだ分だけ）。
        guard が待たせた（HOLD・REST）・電池が少ないときは 0 なので進まず、1 歩を終えたことにならない。
        区切りと逆の向き（guard の BACKOFF）は数えない"""
        if not self.drive_ok:
            speed = 0.0
        segs = self._ex_step[2]
        back = self._ex_seg < len(segs) and segs[self._ex_seg][2]
        d = max(0.0, -speed if back else speed) * self.car.cm_s_per_speed * dt
        if self._ex_seg < MAX_SEGMENTS:
            self.step_driven[self._ex_seg] += d
        if self._skip_now:
            self._skip_moved += d
        self._ex_moved += d

    def _approach(self, bearing: float, dist: float):
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

    def _explore(self, dt: float):
        ctl = self.ctl
        self._ex_total += dt
        if self._ex_total >= ctl.explore_max_s:
            self.set_mode("idle")
            return 0.0, 0.0, 0.0, 0.0
        self._ex_t += dt
        if self._ex_phase == "scan":
            poses = self.scan_poses
            if self._ex_sched_stop != self._ex_stop:
                self._ex_sched_stop = self._ex_stop
                self._ex_sched = scan_schedule(self.cmd.pan, self.cmd.tilt, poses)
            i = bisect.bisect_right(self._ex_sched, self._ex_t)
            if i < len(poses):
                self.cmd.scan_index = self._ex_stop * len(poses) + i
                return (*poses[i], 0.0, 0.0)
            self._ex_stop += 1
            self._ex_t, self._ex_moved = 0.0, 0.0
            # 撮った分をメインプロセスがサーバーに送り、次の 1 歩をもらうまで止まって待つ
            self._ex_phase, self.station_wait = "plan", self._ex_stop - 1
            self.cmd.scan_index = -1
            return 0.0, 0.0, 0.0, 0.0
        if self._ex_phase == "plan":
            self.cmd.scan_index = -1
            p = self.plan
            if p is None or int(p[0]) != self.station_wait:
                # 返事を待つ。届かなければ StationClient が送り直す（Pi では道を決めない）
                return 0.0, 0.0, 0.0, 0.0
            action = int(p[1])
            if action == PLAN_DONE:
                self.station_wait = -1
                self.set_mode("idle")  # 見ていない所が無い・電池が少ない など（理由はメインプロセスが出す）
                return 0.0, 0.0, 0.0, 0.0
            if action == PLAN_RESHOOT:
                self._reshoot()
                return 0.0, 0.0, 0.0, 0.0
            if action != PLAN_MOVE:
                return 0.0, 0.0, 0.0, 0.0  # 前の版の PLAN_FALLBACK などは待つのと同じ
            self.station_wait = -1
            self._start_step(p)
        self.cmd.scan_index = DRIVE_INDEX
        return self._step()

    def _reshoot(self) -> None:
        """待っている station を同じ番号のまま撮り直す。scan_index も同じなので、撮れた向きは
        メインプロセスが撮り直さず、足りない向きだけ足す。直前の 1 歩（prev_step の元）はそのまま"""
        self._ex_stop, self.station_wait, self.plan = self.station_wait, -1, None
        self._ex_phase, self._ex_t, self._ex_sched_stop = "scan", 0.0, -1

    def _start_step(self, p) -> None:
        """plan（control_proc.PLAN_N）から、サーバーの 1 歩を走り始める。区切りの距離は、前へは合わせて
        explore_server_step_cm、後ろへは合わせて explore_reverse_max_cm まで（後ろは見えない）"""
        ctl, car = self.ctl, self.car
        fwd_speed = abs(p[2]) if 0 < abs(p[2]) <= ctl.explore_speed else ctl.explore_speed
        back_speed = abs(p[4]) if 0 < abs(p[4]) <= ctl.explore_speed else min(fwd_speed, ctl.explore_speed)
        left_fwd, left_back = ctl.explore_server_step_cm, ctl.explore_reverse_max_cm
        segs = []
        for i in range(min(int(p[3]), MAX_SEGMENTS)):
            k = 5 + PLAN_SEG_N * i
            steer = clip(p[k], -car.steer_max, car.steer_max)
            back = p[k + 1] < 0
            d = abs(p[k + 1])
            if back:
                d = min(d, left_back)
                left_back -= d
            else:
                d = min(d, left_fwd)
                left_fwd -= d
            if d > 0:
                segs.append((steer, d, back, p[k + 2] if p[k + 2] > 0 else None))
        self._ex_step = (fwd_speed, back_speed, segs)
        self._ex_phase, self._ex_t, self._ex_moved, self._ex_seg = "step", 0.0, 0.0, 0
        self._gear_wait, self.low_passed = 0.0, 0
        self._step_start_ns, self._skip_moved = self._now_ns, 0.0
        self.step_station, self.step_driven = int(p[0]), [0.0] * MAX_SEGMENTS

    def _step(self):
        """サーバーが決めた 1 歩（区切りの列）を走る。止まったら、理由を次の station で伝える。
        前へ走る区切りだけ画像の空き（clear_cm）を見る。下がる区切りは、サーバーが確かな床だけを通した道で、
        Pi の上限（explore_reverse_max_cm）と、画像が動かないとき（stuck）・guard・電池で止まる"""
        fwd_speed, back_speed, segs = self._ex_step
        while self._ex_seg < len(segs) and self._ex_moved >= segs[self._ex_seg][1]:
            nxt = self._ex_seg + 1
            if nxt < len(segs) and segs[nxt][2] != segs[self._ex_seg][2]:
                self._gear_wait = GEAR_PAUSE_S  # 前と後ろを切り替える：止まって待つ
            self._ex_seg, self._ex_moved = nxt, 0.0  # 次の区切りへ
            self._skip_moved = 0.0
        if self._ex_seg >= len(segs):
            return self._end_step("done")
        steer, _, back, min_clear = segs[self._ex_seg]
        if self._gear_wait > 0:
            self._gear_wait -= self._last_dt
            # 前へ走り出すときは、clear_cm を走り始めと同じく待つ（下がる間は出ていない）
            self._step_start_ns = self._now_ns
            self._clear_null_since = None if self.clear_state not in (CLEAR_NULL, CLEAR_STALE) else self._now_ns
            return 0.0, DRIVE_TILT, steer, 0.0
        if self.stuck:
            self.stuck = False
            return self._end_step("stuck")
        speed = -back_speed if back else fwd_speed
        skip = False
        if not back:
            blocked = self.clear_blocked(min_clear)
            if blocked == "near":
                return self._end_step("clear")
            if blocked == "unknown":
                return self._end_step("clear_unknown")
            absent = self.clear_state == CLEAR_ABSENT and self._ex_t >= CLEAR_ABSENT_GRACE_S
            skip = self.clear_state == CLEAR_SKIP or absent
            if not skip:
                self._skip_moved = 0.0
            elif self._skip_moved >= CLEAR_SKIP_MAX_CM:
                # 画像で前を見ていない（大きく曲がっている・clear_cm を出さないサーバー）。遅く・短く
                return self._end_step("clear_skip")
            if skip:
                speed = min(speed, CLEAR_SKIP_SPEED)
        self._drive_count, self._skip_now = "step", skip  # 進んだ距離は step の最後に数える
        return (_drive_pan(steer) if not back else 0.0), DRIVE_TILT, steer, speed

    def _end_step(self, reason: str):
        self.last_step = (self._ex_t, STOP_REASONS.index(reason))
        self._ex_phase, self._ex_t = "scan", 0.0
        return 0.0, 0.0, 0.0, 0.0

    def _map(self, dt: float):
        poses = [(p, t) for t in MAP_TILTS for p in MAP_PANS]
        self._map_t += dt
        i = int(self._map_t / MAP_HOLD_S)
        if i >= len(poses):
            self.set_mode("idle")
            return 0.0, 0.0
        self.cmd.scan_index = i
        return poses[i]
