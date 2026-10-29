"""snoo を怖がらせない・しつこくしないための安全の層。走る行動（keep / follow、のちの遊びの技）の出力に、
最後にかぶせる。どの技を足しても、ここを通らないと車体は動かない。

決まり（2026-10-29 に ROCm 側と決めた初めの値。実際の snoo のデータで詰める）:
- ストレス: サーバーの play.p.stress が 0.8 以上を 1 回、または 0.5 以上を 3 回続けて見たら引き金。
  引き金 → 止まる → ゆっくり 20〜30 cm 下がる → snoo の方を向いたまま止まって待つ。
  0.2 未満が 5 s 続いたら解く（ヒステリシス）。1 回の遊びで 3 回引いたら、遊びを終えて休む
- play が null（サーバーがまだ判定しない）でも効く決まり:
  1. 自分が動いて 30 cm より近づかない（snoo から来るのはよい）
  2. 車が近づいた直後に snoo が速く離れる（50 cm/s 以上）のはストレスとみなす
  3. 1 回の遊びは 3 分まで。そのあと 10 分は休む（走らない。首は動かしてよい）
"""

from __future__ import annotations

from dataclasses import dataclass

OK, BACKOFF, HOLD, REST = 0, 1, 2, 3
GUARD_NAMES = {OK: "OK", BACKOFF: "下がる", HOLD: "待つ", REST: "休む"}


@dataclass
class GuardConfig:
    stress_hi: float = 0.8  # 1 回で引き金
    stress_mid: float = 0.5  # stress_mid_frames 回続けて引き金
    stress_mid_frames: int = 3
    stress_release: float = 0.2  # これ未満が release_s 続いたら解く
    release_s: float = 5.0
    max_triggers: int = 3  # 1 回の遊びでこれだけ引いたら終える
    backoff_cm: float = 25.0
    backoff_speed: float = 12.0
    min_gap_cm: float = 30.0  # 自分が動いてこれより近づかない
    gap_lookahead_s: float = 0.5  # 今の速さでこの先に近づく分も見る
    flee_cm_s: float = 50.0  # これ以上の速さで離れたらストレスとみなす
    flee_after_approach_s: float = 1.0  # 車が前に進んでからこの間だけ見る
    flee_hold_s: float = 0.2  # 速く離れるのがこれだけ続いたら
    session_s: float = 180.0
    rest_s: float = 600.0


class Guard:
    def __init__(self, cfg: GuardConfig, cm_s_per_speed: float):
        self.cfg = cfg
        self.cm_s = cm_s_per_speed
        self.state = OK
        self.triggers = 0
        self._mid_run = 0
        self._calm_since: float | None = None
        self._backoff_left_cm = 0.0
        self._session_t: float | None = None  # 走る行動に入ってからの時間
        self._rest_left = 0.0
        self._last_forward: float | None = None
        self._flee_t = 0.0
        self._stress: float | None = None
        self.t = 0.0

    # 入力
    def observe_stress(self, stress: float | None) -> None:
        """検出 1 回ごとに呼ぶ。None はサーバーが判定していない"""
        self._stress = stress
        if stress is None:
            return
        c = self.cfg
        self._mid_run = self._mid_run + 1 if stress >= c.stress_mid else 0
        if stress >= c.stress_hi or self._mid_run >= c.stress_mid_frames:
            self._mid_run = 0
            self.trigger()

    def trigger(self) -> None:
        if self.state in (BACKOFF, HOLD, REST):
            self._calm_since = None  # 待っている間にまた引いたら、待ち直す
            return
        self.triggers += 1
        if self.triggers >= self.cfg.max_triggers:
            self._start_rest()
            return
        self.state = BACKOFF
        self._backoff_left_cm = self.cfg.backoff_cm
        self._calm_since = None

    def _start_rest(self) -> None:
        self.state = REST
        self._rest_left = self.cfg.rest_s
        self._session_t = None
        self.triggers = 0

    def apply(self, driving: bool, dt: float, steer: float, speed: float,
              dist_cm: float | None, dist_rate_cm_s: float | None) -> tuple[float, float]:  # fmt: skip
        """行動が出したハンドル・速さを、決まりに合わせて直して返す。driving は走る行動のとき True"""
        c = self.cfg
        self.t += dt
        if self.state == REST:
            self._rest_left -= dt
            if self._rest_left <= 0:
                self.state = OK
            return 0.0, 0.0
        if not driving:
            self._session_t = None  # 走らない行動に戻ったら、遊びは終わり
            if self.state == BACKOFF:
                self.state = HOLD
        else:
            self._session_t = (self._session_t or 0.0) + dt
            if self._session_t >= c.session_s:
                self._start_rest()
                return 0.0, 0.0

        # 決まり 2: 車が近づいた直後に snoo が速く離れた
        if speed > 0:
            self._last_forward = self.t
        recent = self._last_forward is not None and self.t - self._last_forward <= c.flee_after_approach_s
        if recent and dist_rate_cm_s is not None and dist_rate_cm_s >= c.flee_cm_s:
            self._flee_t += dt
            if self._flee_t >= c.flee_hold_s:
                self._flee_t = 0.0
                self._last_forward = None
                self.trigger()
        else:
            self._flee_t = 0.0

        if self.state == BACKOFF:
            self._backoff_left_cm -= c.backoff_speed * self.cm_s * dt
            if self._backoff_left_cm <= 0:
                self.state = HOLD
            return 0.0, -c.backoff_speed
        if self.state == HOLD:
            calm = self._stress is None or self._stress < c.stress_release
            if not calm:
                self._calm_since = None
            elif self._calm_since is None:
                self._calm_since = self.t
            elif self.t - self._calm_since >= c.release_s:
                self.state = OK
            return 0.0, 0.0

        # 決まり 1: 自分が動いて min_gap_cm より近づかない
        if speed > 0 and dist_cm is not None:
            if dist_cm - speed * self.cm_s * c.gap_lookahead_s < c.min_gap_cm:
                speed = 0.0
        return steer, speed
