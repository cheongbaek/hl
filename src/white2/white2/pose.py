#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pose.py ― GPS 만으로 뒷차축 위치 · 헤딩 내기 (IMU · 바퀴 속도 없음)
════════════════════════════════════════════════════════════════════════════════
white1(단일 GPS — 전방 안테나) · white2(듀얼 GPS — 전방 + 후방 안테나)가 같은 파일을 쓴다.
ROS 를 쓰지 않는 순수 계산이다 — 좌표는 로컬 평면 [m] (x 동, y 북), 헤딩은 도(동 0°, 반시계 +).
driving 의 경로추종(black 제어)은 ★뒷차축★ 을 기준점으로 삼으므로 여기서 뒷차축 자리와 헤딩을 만든다.

────────────────────────────────────────────────────────────────────────────────
 FrontTracker — 전방 안테나 궤적만으로 (white1, white2 의 '후방 GPS 를 못 믿을 때')
────────────────────────────────────────────────────────────────────────────────
  뒷차축은 ★차체 방향으로만★ 움직인다(옆으로 미끄러지지 않는다). 그래서 차체 축 위 d 만큼 앞에 있는
  안테나가 끌고 가는 '끌려가는 점' 으로 볼 수 있다 — 트레일러 뒷바퀴가 견인고리를 따라가는 것과 같다.
  안테나의 새 자리 F 가 오면 :  방향 = (F − 뒷차축의 지난 자리),  뒷차축 = F − d·방향.
  · 조향각 · 자이로 없이 헤딩이 나온다. 첫 헤딩이 조금 틀려도 앞으로 가면서 d 의 몇 배 거리 안에 줄어든다.
  · 서 있을 때 GPS 잡음(1~2 cm)은 헤딩을 흔들 뿐 쌓이지 않는다(뒷차축 자리는 축 방향으로만 조금 움직인다).
    순간 흔들림은 잡음 ÷ d — d 1.0 m, 잡음 1.5 cm 면 약 0.9°.
  · 한 걸음(10 Hz) 동안 뒷차축이 간 만큼 방향이 늦게 잡히므로 가운데 점으로 한 번 고친다(set 주석).
    남는 지연은 급선회에서 1° 안쪽이다.
  · ★전진 전용★ 이다. 후진하면 끌려가는 점이 미는 점이 되어 헤딩이 서서히 뒤집힌다.
  · 위치가 튀면(Float) 헤딩도 튄다 — max_turn_deg 로 한 번에 돌 수 있는 양을 차가 실제로 돌 수 있는
    만큼으로 막는다(turn_limit_deg).

────────────────────────────────────────────────────────────────────────────────
 DualPose — 전방 + 후방 안테나 (white2)
────────────────────────────────────────────────────────────────────────────────
  기준점 = 후방 안테나(뒷차축 위). 안테나 간격 B(base_m)는 실측해서 넣는다.
  · 둘 다 RTK Fixed 이고 두 점 거리가 B ± tol : ★DUAL★ — 헤딩 = 후방 → 전방 방향, 위치 = 후방. 서 있어도 된다.
    ★첫 헤딩은 이때만 잡는다★ (사용자 지정: Fixed 에서만 헤딩을 잡는다).
  · 한쪽만 믿을 수 있을 때(한쪽이 Float · 두절, 또는 둘 다 Fixed 인데 간격이 안 맞는 오고정) :
      후방만 → ★REAR★  위치 = 후방, 헤딩 = 후방 안테나의 이동 방향으로 잇는다(CourseHeading).
      전방만 → ★FRONT★ 위 FrontTracker 로 후방 자리와 헤딩을 잇는다.
    믿는 쪽에서 헤딩 방향으로 B 만큼 떨어진 곳이 다른 안테나 자리다(other_antenna) — 사용자 표현 '1 m 보간'.
  · 어느 쪽을 믿나 : Fixed 가 Float 보다 먼저. 등급이 같으면(둘 다 Float, 또는 간격이 틀어진 Fixed 둘)
    ★직전 자세에서 예측한 자리에 더 가까운 쪽★ — 튄 쪽이 걸러진다.
  · 간격이 맞아도 곧바로 믿지 않는다 : 틀린 점이 우연히 간격만 맞는 자리에 올 수 있다(차가 돌면서 생긴다).
    듀얼 해가 ★지금 자세에서 갑자기 벗어나면★(헤딩 REJOIN_DEG, 위치 REJOIN_M 넘게) 받아들이지 않고 한쪽만 쓴다.
    그 상태가 REJOIN_WAIT_N 번(2초) 이어지면 지금 자세가 틀렸다고 보고 듀얼 해로 다시 맞춘다.
  ⚠️ Fixed 라도 틀릴 수 있다(tool/info.md · AGENTS.md: 10분간 1.3 m 틀린 Fixed). 간격 검사가 그 방어다.
"""

import math

Q_NONE, Q_SPS, Q_DGPS, Q_FLOAT, Q_FIXED = 0, 1, 2, 3, 4      # broon.gpsdev 와 같다
MODE_NONE, MODE_FRONT, MODE_REAR, MODE_DUAL = 0, 1, 2, 3     # /ego_state 의 ego_mode 열
MODE_LABEL = {MODE_NONE: "없음", MODE_FRONT: "전방만", MODE_REAR: "후방만", MODE_DUAL: "듀얼"}
MAX_STEER_DEG = 25.0         # 차가 낼 수 있는 조향 (아두이노 규약 상한) — 헤딩 변화 한계 계산용
NAN = float('nan')


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def turn_limit_deg(v_ms, dt_s, wheelbase_m):
    """dt 동안 이 차가 실제로 돌 수 있는 헤딩 변화의 상한 [deg] (1.5배 여유 + 잡음 몫 1°)."""
    yaw = abs(v_ms) * math.tan(math.radians(MAX_STEER_DEG)) / wheelbase_m
    return 1.0 + 1.5 * math.degrees(yaw) * max(0.0, dt_s)


class FrontTracker:
    """앞쪽 안테나(끌려가는 점에서 d 앞) 궤적 → 끌려가는 점(뒷차축) 위치 · 헤딩."""

    INIT_DIST_M = 0.5        # auto_init : 첫 점에서 이만큼 움직이면 그 방향을 첫 헤딩으로 삼는다

    def __init__(self, d):
        self.d = float(d)
        self.reset()

    def reset(self):
        self.r = None            # 끌려가는 점 (x, y)
        self.heading = None      # [deg]
        self._f0 = None

    @property
    def ready(self):
        return self.heading is not None

    def set(self, fx, fy, heading_deg):
        """안테나 자리와 헤딩을 알 때 — 끌려가는 점을 그 뒤 d 에 둔다."""
        h = math.radians(heading_deg)
        self.heading = wrap180(heading_deg)
        self.r = (fx - self.d * math.cos(h), fy - self.d * math.sin(h))

    def set_rear(self, rx, ry, heading_deg):
        """끌려가는 점 자리와 헤딩을 알 때 (DualPose 가 다른 방법으로 낸 자세를 넘겨받는다)."""
        self.heading = wrap180(heading_deg)
        self.r = (rx, ry)

    def rear(self):
        return self.r

    def feed(self, fx, fy, max_turn_deg=None, auto_init=False):
        """안테나의 새 자리. 자세가 있으면(갱신했으면) True."""
        if self.heading is None:
            if not auto_init:
                return False
            if self._f0 is None:
                self._f0 = (fx, fy)
                return False
            dx, dy = fx - self._f0[0], fy - self._f0[1]
            if math.hypot(dx, dy) < self.INIT_DIST_M:
                return False
            self.set(fx, fy, math.degrees(math.atan2(dy, dx)))   # 곧게 움직였다고 본다 — 틀려도 가면서 줄어든다
            return True
        rx, ry = self.r
        if math.hypot(fx - rx, fy - ry) < 1e-6:
            return True
        # ① 지난 자리에서 본 방향 → 임시 새 자리  ② 지난 자리와 임시 자리의 가운데에서 다시 본다
        #    (뒷차축이 이번 걸음에 간 만큼 ①은 방향을 늦게 본다 — 가운데 점이 그 지연을 절반으로 줄인다)
        h1 = math.atan2(fy - ry, fx - rx)
        mx = 0.5 * (rx + fx - self.d * math.cos(h1))
        my = 0.5 * (ry + fy - self.d * math.sin(h1))
        h = math.degrees(math.atan2(fy - my, fx - mx))
        if max_turn_deg is not None:
            dh = wrap180(h - self.heading)
            h = self.heading + max(-max_turn_deg, min(max_turn_deg, dh))
        self.set(fx, fy, h)
        return True


class CourseHeading:
    """뒷차축 위 안테나의 이동 방향으로 헤딩을 잇는다 — 뒷차축은 헤딩 방향으로만 움직인다(전진 전용)."""

    STEP_M = 0.30            # 이만큼 움직일 때마다 한 번 본다 (GPS 잡음 1~2 cm 가 방향을 흔들지 않게)
    GAIN = 0.7

    def __init__(self, wheelbase_m):
        self.wheelbase = wheelbase_m
        self.anchor = None

    def reset(self, x, y):
        self.anchor = (x, y)

    def feed(self, x, y, heading_deg):
        if self.anchor is None:
            self.anchor = (x, y)
            return heading_deg
        dx, dy = x - self.anchor[0], y - self.anchor[1]
        d = math.hypot(dx, dy)
        if d < self.STEP_M:
            return heading_deg
        self.anchor = (x, y)
        # 거리 d 를 가는 동안 차가 돌 수 있는 양(d ÷ 최소회전반경)보다 크게는 고치지 않는다
        lim = 3.0 + math.degrees(d * math.tan(math.radians(MAX_STEER_DEG)) / self.wheelbase)
        dh = self.GAIN * wrap180(math.degrees(math.atan2(dy, dx)) - heading_deg)
        return wrap180(heading_deg + max(-lim, min(lim, dh)))


class DualPose:
    """전방 · 후방 안테나 → 후방 안테나 자리(x, y) · 헤딩. update() 에 같은 측위 시각의 두 점을 넣는다."""

    REJOIN_DEG = 15.0        # 듀얼 해의 헤딩이 지금 헤딩에서 이보다 벗어나면 의심한다 (한 표본에 돌 수 있는 양의 두 배)
    REJOIN_M = 0.5           # 듀얼 해의 위치가 예측 자리에서 이보다 벗어나면 의심한다
    REJOIN_WAIT_N = 20       # 의심스러운 듀얼 해가 이만큼 이어지면(2초) 받아들인다 — 지금 자세가 틀린 것이다

    def __init__(self, base_m, wheelbase_m, base_tol_m=0.20, dual_alpha=0.7):
        self.B = float(base_m)
        self.L = float(wheelbase_m)
        self.tol = float(base_tol_m)
        self.alpha = float(dual_alpha)       # DUAL 헤딩 평활(1 = 그대로). 두 점 잡음 1.5 cm · B 1 m 면 한 표본 ±1.2°
        self.front = FrontTracker(self.B)
        self.course = CourseHeading(self.L)
        self.reset()

    def reset(self):
        self.x = self.y = None
        self.heading = None
        self.mode = MODE_NONE
        self.base = NAN              # 이번에 잰 두 안테나 거리
        self.base_bad = False        # 둘 다 Fixed 인데 간격이 안 맞는다 (한쪽 오고정 의심)
        self.suspect = False         # 간격은 맞는데 듀얼 해가 지금 자세에서 갑자기 벗어났다
        self._suspect_n = 0
        self.front.reset()
        self.course.anchor = None

    @property
    def ready(self):
        return self.heading is not None

    def other_antenna(self):
        """지금 자세에서 전방 안테나가 있을 자리 (후방에서 헤딩 방향으로 B)."""
        h = math.radians(self.heading)
        return self.x + self.B * math.cos(h), self.y + self.B * math.sin(h)

    def update(self, f, r, v_ms, dt_s, min_q=Q_FLOAT):
        """f, r = (x, y, quality) 또는 None (그 시각에 측위 없음). 자세를 갱신했으면 True."""
        f = f if f is not None and f[2] >= min_q else None
        r = r if r is not None and r[2] >= min_q else None
        f_fix = f is not None and f[2] >= Q_FIXED
        r_fix = r is not None and r[2] >= Q_FIXED
        self.base = math.hypot(f[0] - r[0], f[1] - r[1]) if f is not None and r is not None else NAN
        self.base_bad = f_fix and r_fix and abs(self.base - self.B) > self.tol
        dual = f_fix and r_fix and not self.base_bad
        self.suspect = False
        if dual and self.heading is not None and dt_s < 0.5:      # 오래 끊겼다 돌아온 첫 해는 그대로 받는다
            h = math.degrees(math.atan2(f[1] - r[1], f[0] - r[0]))
            hr = math.radians(self.heading)
            jump = math.hypot(r[0] - self.x - v_ms * dt_s * math.cos(hr), r[1] - self.y - v_ms * dt_s * math.sin(hr))
            if abs(wrap180(h - self.heading)) > self.REJOIN_DEG or jump > self.REJOIN_M:
                self._suspect_n += 1
                if self._suspect_n < self.REJOIN_WAIT_N:
                    dual, self.suspect = False, True
        if dual:
            self._suspect_n = 0
            h = math.degrees(math.atan2(f[1] - r[1], f[0] - r[0]))
            if self.heading is not None and self.mode == MODE_DUAL:
                h = self.heading + self.alpha * wrap180(h - self.heading)
            self.heading = wrap180(h)
            self.x, self.y = r[0], r[1]
            self.front.set_rear(self.x, self.y, self.heading)
            self.course.reset(self.x, self.y)
            self.mode = MODE_DUAL
            return True
        if self.heading is None:
            return False                     # 첫 헤딩은 둘 다 Fixed(간격 정상)일 때만 잡는다
        use = self._choose(f, r, f_fix, r_fix, v_ms, dt_s)
        if not self.suspect:
            self._suspect_n = 0
        if use == 'rear':
            self.heading = self.course.feed(r[0], r[1], self.heading)
            self.x, self.y = r[0], r[1]
            self.front.set_rear(self.x, self.y, self.heading)
            self.mode = MODE_REAR
        elif use == 'front':
            self.front.feed(f[0], f[1], max_turn_deg=turn_limit_deg(v_ms, dt_s, self.L))
            self.x, self.y = self.front.rear()
            self.heading = self.front.heading
            self.course.reset(self.x, self.y)
            self.mode = MODE_FRONT
        else:
            return False
        return True

    def _choose(self, f, r, f_fix, r_fix, v_ms, dt_s):
        if f is None and r is None:
            return None
        if f is None:
            return 'rear'
        if r is None:
            return 'front'
        if f_fix != r_fix:
            return 'rear' if r_fix else 'front'
        # 등급이 같다 — 직전 자세에서 예측한 자리에 더 가까운 쪽을 믿는다
        h = math.radians(self.heading)
        px = self.x + v_ms * dt_s * math.cos(h)
        py = self.y + v_ms * dt_s * math.sin(h)
        err_r = math.hypot(r[0] - px, r[1] - py)
        err_f = math.hypot(f[0] - (px + self.B * math.cos(h)), f[1] - (py + self.B * math.sin(h)))
        return 'rear' if err_r <= err_f else 'front'
