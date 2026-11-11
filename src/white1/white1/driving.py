#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
driving.py ― 1/5카 GPS 경로추종, ★단일 GPS(전방 안테나) · IMU 없음★ [white1]
════════════════════════════════════════════════════════════════════════════════
    ros2 run white1 driving         (one_launch.py 가 띄운다)

hl_ws/src/white 의 driving.py(black_ws22 v6.7.3 제어 + gold white1 상태기계)를 바탕으로, 센서 쪽만 바꿨다.
경로추종 제어(순수추종 + PID · 지연예측 · 곡률 선행감속 · 종점 접근 …)와 경로 CSV · 순환(return) 처리는
white 와 같다 — 그 설명은 white/driving.py 머리말과 아래 각 절 주석에 있다.

white 대비 달라진 것
  · ★헤딩 · 위치를 GPS 만으로★ — IMU(/imu)를 쓰지 않는다.
      출발 : 조향 0 으로 곧게 굴려 전방 안테나의 변위로 첫 헤딩을 잡는다(DRIVE_HEADING — white 와 같은 방법).
      이후 : 전방 안테나 궤적으로 뒷차축 자리와 헤딩을 잇는다(pose.FrontTracker — 견인 기하, pose.py 머리말).
    GPS 안테나는 ★뒷차축에서 antenna_base_m 앞★(앞차축 쪽)에 있다. 제어 기준점은 white 와 같이 뒷차축이다.
  · ★바퀴 속도가 없다★ — 새 아두이노 규약은 PC 로 아무것도 보내지 않는다. 속도는 GPS 지면속도(NAV-PVT)로 본다.
    전방 안테나의 속도라 선회 중에는 뒷차축보다 조금 크다(조향 20° 에서 6 %).
    지연 예측의 요레이트는 직전 조향 명령으로 추정한다(black 의 'IMU 두절' 경로).
  · ★출발은 RTK Fixed 에서만, 그 뒤로는 Float 여도 계속 간다★ (사용자 지정 2026-11-11).
    Fixed 가 아니면 출발을 거절한다. 주행 중 Float 미만(단독 · DGNSS)이나 두절이 GPS_LOST_S 이어지면
    서서 기다렸다가 Float 이상이 GPS_RESUME_S 이어지면 그 자리에서 재개한다. 헤딩을 잡는 직진 구간은 Fixed 만 쓴다.
    ⚠️ Float 는 실제 오차가 0.6~2.7 m 일 수 있다(AGENTS.md GPS 시험) — 위치가 튀면 헤딩도 흔들린다.
       한 번에 돌 수 있는 양만 막아 두었다(pose.turn_limit_deg). 경로에서 2 m 벗어나면 선다.
  · ★명령은 km/h · 도★ — /cmd_vel_raw 의 linear.x 가 km/h 다(white 는 m/s). broon/arduino 가 정수로
    반올림해 4바이트 프레임으로 보낸다(최대 16 km/h · 25°). 안에서는 m/s 로 계산한다.
    속도 인자도 km/h : cruise_kmh 6 · min_kmh 4 · heading_kmh 4 (정수 km/h 로 나가므로 속도는 4 · 5 · 6 단계다).
  · 리모컨 · IMU · 아두이노 응답은 보지 않는다.

토픽
  구독  /gps_pvt (전방 GPS — broon/gps) · /drive_cmd
  발행  /cmd_vel_raw (Twist: linear.x ★km/h★, angular.z deg ★+ 좌 / − 우★)   /control_state (Bool)
        /drive_state · /drive_event · /drive_status (String)   /driving_debug (Float64MultiArray, black 0~26)
        /ego_state (Float64MultiArray: 뒷차축 x, y [m], 헤딩 [deg], 속도 [m/s], 뒷차축 위도, 경도, 자세 방식)
        /mapping_cmd (Bool)   /route_dir (String)

상태기계 (white 와 같다)
      IDLE ──(MAP_START)──▶ MAP ──(MAP_STOP)──▶ IDLE
      IDLE ──(경로 + DRIVE_START, ★Fixed 일 때만★)──▶ DRIVE_HEADING ──(헤딩 확정)──▶ DRIVE_RUN
                                  DRIVE_DONE ◀──(도착 · 이탈 · STOP · 헤딩 실패)   ※ 순환 경로는 도착 없음
                                   속도 0, 조향 0 → GPS 속도가 2 s 동안 0 이면 IDLE (prompt 메뉴 복귀)

⚠️ 실차 미검증 — 가상 차량 시뮬레이션으로만 확인했다. 실측해서 넣을 값 : antenna_base_m(뒷차축 → 안테나),
   WHEELBASE_M 0.73(레거시 코드 값), 조향 부호 · 눈금(새 아두이노), black 의 조향 트림 · 게인이 그대로 맞는지.
"""

import csv
import math
import os
import signal
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64MultiArray, String

from broon.gpsdev import GPS_PVT_FIELDS, GPS_PVT_TOPIC, Q_FIXED, Q_FLOAT, Q_LABEL, Q_NONE
from white1 import paths
from white1.pose import MODE_FRONT, MODE_NONE, FrontTracker, turn_limit_deg

# ══════════════════════════════════════════════════════════════════════════════
#  차량 · 속도
# ══════════════════════════════════════════════════════════════════════════════
WHEELBASE_M = 0.73           # 휠베이스 [m] (black)
STEER_MAX_DEG = 20.0         # ROS측 조향 클램프 (black, v6 눈금)
CRUISE_KMH = 6.0             # ★순항 상한 [km/h]★ (white 1.8 m/s = 6.5 km/h → 정수 6)
MIN_KMH = 4.0                # 주행 중 최저 속도 (black 1.0 m/s = 3.6 km/h → 정수 4)
HEADING_KMH = 4.0            # 헤딩 초기화 직진 속도
MAX_KMH = 16.0               # 아두이노 규약 상한 (broon/arduino.py)
ANTENNA_BASE_M = 1.0         # ★뒷차축 → 전방 안테나 거리★ (사용자 지정 2026-11-11 — 실측해서 런치 인자로 고칠 것)
HEADING_RAMP_S = 1.0         # 헤딩 초기화 출발 램프
SAFE_STOP_SPEED = 0.08       # 정지로 간주하는 미세속도 (black)
GPS_STOP_SPEED = 0.12        # [m/s] GPS 지면속도가 이 밑이면 '섰다' (정지 잡음 0.01~0.06 m/s)

# ══════════════════════════════════════════════════════════════════════════════
#  black v6.7.3 제어 상수 (값 그대로 — 근거는 black_ws22 driving.py 주석)
# ══════════════════════════════════════════════════════════════════════════════
SPD_REASON_CODE = {
    "MAX": 0, "CURVE": 1, "BRAKE": 2, "CTE_RECOVERY": 3, "STEER": 4, "RAMP": 7,
    "SPEED_PI": 8, "ACCEL_LIMIT": 9, "DECEL_LIMIT": 10, "APPROACH": 11, "OMEGA": 12,
}
GAIN_TABLE = [               # v(m/s), Kp, Ki, Kd — 구간 사이 선형보간
    (1.0, 1.7, 0.12, 0.11),
    (1.4, 1.9, 0.14, 0.14),
    (1.8, 2.1, 0.16, 0.17),
    (2.2, 2.3, 0.20, 0.21),
    (2.5, 2.5, 0.23, 0.25),
    (2.8, 2.6, 0.25, 0.30),
]
LFD_TABLE = [                # v(m/s), LFD(m) — ω_n = v·√2/LFD ≈ 1.0 설계
    (1.0, 2.3),
    (1.4, 2.5),
    (1.8, 2.8),
    (2.2, 3.2),
    (2.5, 3.6),
    (2.8, 4.0),
]
STEER_AUTHORITY_FULL_SPEED = 0.5
RAMP_UP_DURATION = 3.0
MIN_LFD, MAX_LFD = 2.3, 4.0
LFD_CURVE_CAP_K = 1.5
LFD_LPF_ALPHA = 0.40
NEAR_DEMAND_LPF_ALPHA = 0.30
STEER_FULL_SLOWDOWN_DEG = 15.0
CURVE_PREVIEW_NEAR_K = 1.6
CURVE_PREVIEW_FAR_MAX = 14.0
CURVE_WINDOW_M = 3.0
CURVE_WINDOW_PEAK_M = 1.2
CURVE_SCAN_STEP_M = 0.4
PEAK_SPEED_BLEND = 0.50
BRAKE_GATE_DECEL = 2.0
BRAKE_GATE_MARGIN = 1.5
GATE_HOLD_DEMAND_DEG = 12.0
GATE_HOLD_DIST_M = 9.5
GATE_HOLD_SPEED = 1.6
MAX_ACCEL_PER_S = 1.7
MAX_DECEL_PER_S = 3.5
CORNER_DECEL_PER_S = 4.5
CTE_RECOVER_START = 0.38
CTE_RECOVER_CAPS = [(0.48, 2.2), (0.62, 1.7), (0.80, 1.2)]
STEER_SPEED_START_DEG = 5.0
STEER_SPEED_CAPS = [(7.0, 2.3), (10.0, 1.8), (13.0, 1.3)]
PREDICT_HORIZON_S = 0.50     # 조향지연 실측 0.45~0.55 s (black 0703 · 0714)
PREDICT_MAX_YAW_RAD = 0.60
OMEGA_N_MAX = 0.95
OMEGA_N_MAX_NO_IMU = 0.80
STEER_PLANT_GAIN_L = 0.85    # 좌회전 실효게인
STEER_PLANT_GAIN_R = 0.82    # 우회전 실효게인
STEER_TRIM_DEG = -1.5        # 직진 쏠림 트림 (black 0703 실측)
V_SCHED_TAU = 0.30
LFD_RATE_UP = 0.9
LFD_RATE_DOWN = 2.0
KI_CLAMP = 6.0
CTE_DEADBAND = 0.02
CTE_DEADBAND_STRAIGHT = 0.06
CTE_LPF_ALPHA = 0.80
D_TERM_LPF_ALPHA = 0.16
D_TERM_SCALE = 0.20
LEAD_WHEELBASE_MULT = 1.2
LEAD_CLAMP = 0.45
LEAD_RATE_LPF_ALPHA = 0.40
HEADING_ALIGN_CTE_REF = 0.45
HEADING_ALIGN_BOOST = 1.0
PID_STEER_LIMIT = 9.0
INTEGRAL_DECAY_INTERVAL = 1.0
JITTER_DEMAND_DEG = 6.0
JITTER_CTE_M = 0.08
JITTER_STEER_DEG = 3.0
JITTER_ZERO_DEG = 0.7
JITTER_LEAD_M = 0.05
SPD_KP, SPD_KI, MAX_SPD_INTEG = 0.08, 0.0005, 0.08
STEER_SMOOTH_ALPHA = 0.5
STEER_STEP_LOW_DEG = 2.5
STEER_STEP_HIGH_DEG = 3.2
STEER_STEP_RECOVER_DEG = 3.5
STEER_STEP_STRAIGHT_DEG = 1.3
APPROACH_START_DIST = 9.0
APPROACH_END_SPEED = 0.10
FINAL_APPROACH_WP_WINDOW = 90
FINAL_APPROACH_ENTRY_SPEED = 2.6
FINAL_HEADING_ALIGN_DIST = 2.0
FINAL_BLEND_BAND = 2.0
FINAL_HEADING_TOL_DEG = 22.0
FINAL_CREEP_DIST = 2.0
FINAL_CREEP_SPEED = 0.85
APPROACH_LFD_MIN = 2.20
APPROACH_LFD_RATIO = 0.45
APPROACH_LFD_OFFSET = 1.30
WP_SEARCH_WINDOW = 45
START_ALIGN_DURATION = 2.0
MAX_WP_ADVANCE_START = 35
MAX_WP_ADVANCE_NORMAL = 5
GPS_JUMP_M = 1.0             # 위치 점프 → WP 재동기화 창 확대
POS_DR_MIN_S = 0.15          # 위치가 이보다 낡으면 GPS 속도 · 헤딩으로 메운다 (10 Hz 정상 간격은 그대로)
POS_DR_MAX_S = 1.0           # 메우는 상한 — 그 전에 GPS 대기(0.4 s)가 차를 세운다
GPS_SPEED_LPF_TAU = 0.05     # GPS 속도(10 Hz) 저역통과

# ══════════════════════════════════════════════════════════════════════════════
#  헤딩 초기화 (gold white1 — 곧게 굴려 GPS 변위로)
# ══════════════════════════════════════════════════════════════════════════════
HEAD_MIN_DIST_M = 1.0        # 이보다 짧으면 판정하지 않는다
HEAD_MAX_DIST_M = 5.0        # 여기까지 가면 최선의 추정으로 확정한다
HEAD_MIN_SAMPLES = 4
HEAD_TARGET_SIGMA = 3.0      # [deg] 추정 오차가 이 밑이면 확정
HEAD_SIGMA_FLOOR = 0.02      # [m] RTK Fixed 수평 노이즈 하한
HEAD_FORCED_MIN_SAMPLES = 3
HEAD_MAX_RESID_M = 0.15      # 직선 잔차 RMS 상한(곧게 갔는가)
HEAD_ABORT_DIST_M = 8.0      # 이만큼(GPS 속도 적분) 곧게 갔는데도 못 잡으면 세운다
MODE_SETTLE_S = 0.7          # 진입 직후 이만큼은 굴리지 않는다

# ── 출발 · 도착 · 이탈 (gold) ──
WP_REACH_M = 0.9             # 도착 허용반경 — 마지막 WP 를 이 안에 들면 도착
GOAL_PASS_MAX_M = 2.5        # 통과 판정을 인정하는 최대 이격
GOAL_ZONE_M = 6.0            # 남은 호길이가 이 안일 때만 도착 판정 (순환코스 출발점 오검출 방지)
CTE_DEVIATION_M = 2.0        # 경로이탈 안전정지
DRIVE_DONE_RELEASE_S = 2.0   # GPS 속도가 이만큼 0 이면 IDLE (prompt 메뉴 복귀)
DRIVE_DONE_MAX_S = 6.0       # 속도를 몰라도 이만큼 지나면 IDLE

# ── 순환 경로 (mod 의 return) ──
LOOP_REBASE_BACK = 12        # WP 번호가 한 바퀴를 넘고도 이만큼 더 간 뒤 한 바퀴 되돌린다 (이탈 판정의 back=10 보다 크게)
LOOP_AHEAD_WP = 160          # 이어 붙인 경로가 되돌리기 직전에도 앞에 남겨야 할 WP 수 (출발 탐색 창 120 · CTE ahead 40)
LOOP_AHEAD_M = 30.0          # 같은 이유의 거리 (곡률 미리보기 14 m + LFD 4 m 보다 넉넉히)
LOOP_GAP_WARN_M = 1.0        # 끝 → 처음 이음매가 이보다 길고 평소 간격의 4배를 넘으면 알린다

# ── 경로 CSV (매핑 CSV 양식) ──
MOD_SEP, MOD_RETURN = ';', 'return'

# ── GPS 품질 · 일시정지 ──
RUN_MIN_QUALITY = Q_FLOAT    # ★주행 중★ 이 품질 이상이면 계속 간다 (출발 · 헤딩 초기화는 언제나 Fixed)
GPS_LOST_S = 0.4             # 품질 미달 · 두절이 이만큼 이어지면 선다
GPS_RESUME_S = 2.0           # 다시 좋아진 채 이만큼 이어지면 재개
GPS_STALE_S = 0.6            # fix 가 이만큼 안 오면 두절
RESUME_GRACE_S = 1.5         # 재개 전 속도 0 유지
GPS_TIMEOUT_S = 2.0          # 출발 판정 — 이만큼 fix 가 없으면 'GPS 가 안 들어온다'

CONTROL_HZ = 20.0
EARTH_R = 6378137.0

S_IDLE = 'IDLE'
S_MAP = 'MAP'
S_DRIVE_HEADING = 'DRIVE_HEADING'
S_DRIVE_RUN = 'DRIVE_RUN'
S_DRIVE_DONE = 'DRIVE_DONE'
DRIVE_STATES = (S_DRIVE_HEADING, S_DRIVE_RUN, S_DRIVE_DONE)


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def latlon_to_xy(lat, lon, lat0, lon0):
    """위경도 → 원점 기준 로컬 평면 [m]. x=동, y=북."""
    x = EARTH_R * math.radians(lon - lon0) * math.cos(math.radians(lat0))
    y = EARTH_R * math.radians(lat - lat0)
    return x, y


def xy_to_latlon(x, y, lat0, lon0):
    """latlon_to_xy 의 역."""
    return (lat0 + math.degrees(y / EARTH_R),
            lon0 + math.degrees(x / (EARTH_R * math.cos(math.radians(lat0)))))


def table_lookup(table, v):
    """속도 → 표 값 선형보간. 행 = (v, a, b, ...) → (a, b, ...)."""
    if v <= table[0][0]:
        return table[0][1:]
    if v >= table[-1][0]:
        return table[-1][1:]
    for k in range(len(table) - 1):
        r0, r1 = table[k], table[k + 1]
        if r0[0] <= v <= r1[0]:
            t = (v - r0[0]) / (r1[0] - r0[0])
            return tuple(a + (b - a) * t for a, b in zip(r0[1:], r1[1:]))
    return table[-1][1:]


def parse_mod(cell):
    """mod 칸 → (구역 단어들, return 여부). 사이트 csv.ts 의 parseMod 와 같다."""
    words, ret = [], False
    for part in (cell or '').split(MOD_SEP):
        w = part.strip()
        if not w:
            continue
        if w.lower() == MOD_RETURN:
            ret = True
        elif w not in words:
            words.append(w)
    return words, ret


def read_route_csv(path):
    """매핑 CSV 양식 → (점 [(lat, lon)], 순환 여부, 후진(dir 1) 줄 수, 구역 {단어: 줄 수}).
    lat · lon 열이 없으면(옛 양식 등) ValueError."""
    with open(path, newline='', encoding='utf-8-sig') as f:      # 양식은 BOM 없음 — 있어도 뗀다
        reader = csv.reader(f)
        head = [h.strip().lower() for h in next(reader, [])]
        if 'lat' not in head or 'lon' not in head:
            raise ValueError("lat · lon 열이 없다 — 매핑 CSV 양식(lat,lon,alt,dir,mod,…)이 아니다 (옛 양식?)")
        ilat, ilon = head.index('lat'), head.index('lon')
        idir = head.index('dir') if 'dir' in head else None
        imod = head.index('mod') if 'mod' in head else None
        pts, loop, rev, zones = [], False, 0, {}
        for row in reader:
            try:
                lat, lon = float(row[ilat]), float(row[ilon])
            except (IndexError, ValueError):
                continue
            if not (math.isfinite(lat) and math.isfinite(lon)) or abs(lat) > 90.0 or abs(lon) > 180.0 \
                    or (abs(lat) < 1e-6 and abs(lon) < 1e-6):          # (0,0) = 미수신
                continue
            words, loop = parse_mod(row[imod] if imod is not None and imod < len(row) else '')
            # ↑ 위경도가 읽힌 마지막 줄의 return 이 순환을 정한다 (사이트와 같다)
            for w in words:
                zones[w] = zones.get(w, 0) + 1
            if idir is not None and idir < len(row):
                try:
                    if int(float(row[idir] or 0)) == 1:
                        rev += 1
                except ValueError:
                    pass
            pts.append((lat, lon))
    return pts, loop, rev, zones


class HeadingEstimator:
    """곧게 굴러간 구간의 GPS 점들로 진행 방위를 낸다 (gold white1).

    방향은 첫 점 → 마지막 점 벡터, 중간 점들은 직진성 검사(두 끝점 직선에서의 수직거리 RMS)에 쓴다.
    """

    def __init__(self):
        self.pts = []

    def reset(self):
        self.pts = []

    def add(self, x, y):
        if self.pts:
            px, py = self.pts[-1]
            if math.hypot(x - px, y - py) < 0.02:
                return
        self.pts.append((x, y))

    def distance(self):
        if len(self.pts) < 2:
            return 0.0
        (x0, y0), (x1, y1) = self.pts[0], self.pts[-1]
        return math.hypot(x1 - x0, y1 - y0)

    def solve(self, sigma_floor_m=HEAD_SIGMA_FLOOR):
        """→ (heading_deg, sigma_deg, resid_rms, n, dist). 표본 부족이면 None."""
        n = len(self.pts)
        if n < 2:
            return None
        (x0, y0), (x1, y1) = self.pts[0], self.pts[-1]
        dx, dy = x1 - x0, y1 - y0
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return None
        heading = math.degrees(math.atan2(dy, dx))
        ux, uy = dx / d, dy / d
        resid_sq = 0.0
        for (px, py) in self.pts:
            cross = (px - x0) * uy - (py - y0) * ux
            resid_sq += cross * cross
        resid_rms = math.sqrt(resid_sq / n)
        sigma = math.degrees(math.atan2(max(resid_rms, sigma_floor_m), d))
        return heading, sigma, resid_rms, n, d


class DrivingNode(Node):
    def __init__(self):
        super().__init__('driving_node')
        self.declare_parameter('data_dir', '')
        self.declare_parameter('cruise_kmh', CRUISE_KMH)
        self.declare_parameter('min_kmh', MIN_KMH)
        self.declare_parameter('heading_kmh', HEADING_KMH)
        self.declare_parameter('run_min_quality', RUN_MIN_QUALITY)   # 3 = Float 여도 계속, 4 = Fixed 에서만 주행
        self.declare_parameter('antenna_base_m', ANTENNA_BASE_M)
        self.declare_parameter('wp_reach_m', WP_REACH_M)
        self.declare_parameter('cte_deviation_m', CTE_DEVIATION_M)
        self.data_dir = paths.data_dir(str(self.get_parameter('data_dir').value))
        self.max_speed = min(MAX_KMH, float(self.get_parameter('cruise_kmh').value)) / 3.6
        self.min_speed = min(float(self.get_parameter('min_kmh').value) / 3.6, self.max_speed)
        self.heading_speed = min(MAX_KMH, float(self.get_parameter('heading_kmh').value)) / 3.6
        self.run_min_q = max(1, min(Q_FIXED, int(self.get_parameter('run_min_quality').value)))
        self.ant_base = float(self.get_parameter('antenna_base_m').value)
        self.wp_reach = float(self.get_parameter('wp_reach_m').value)
        self.cte_dev = float(self.get_parameter('cte_deviation_m').value)

        # ── 상태 ──
        self.state = S_IDLE
        self.state_t0 = time.time()
        self.route_name = ''
        self.raw_wps = []
        self.route_loop = False       # mod 의 return — 순환 경로
        self.wps = []                 # [(x, y)] — 순환이면 몇 바퀴 이어 붙인 것
        self.wp_s = []                # 누적 호길이
        self.wp_idx = 0
        self.lap_wps = 0              # 순환이면 한 바퀴 WP 수, 아니면 0
        self.laps = 0                 # 순환 — 마친 바퀴 수

        # ── 위치 · 헤딩 (GPS 만) ──
        self.lat0 = self.lon0 = None
        self.fx = self.fy = 0.0       # 전방 안테나 [m]
        self.x = self.y = 0.0         # ★뒷차축★ [m] — 헤딩을 잡기 전에는 안테나 자리 그대로
        self.heading = None           # [deg] ENU, 동 0° 반시계 +
        self.tracker = FrontTracker(self.ant_base)
        self.pose_mode = MODE_NONE
        self.fix_time = 0.0           # 마지막 fix (품질 무관) — 두절 판정
        self.pos_time = 0.0           # 마지막으로 위치에 반영한 fix (run_min_quality 이상)
        self.gps_q = Q_NONE
        self.gps_q_logged = None
        self.gps_hacc = float('nan')
        self.gps_speed = float('nan')
        self.gps_speed_t = 0.0
        self.speed_ms = 0.0           # 제어용 속도 (GPS 지면속도 저역통과)
        self._v_at_pos = 0.0          # 마지막으로 위치에 반영한 fix 때의 속도
        self.fix_ok = False           # 지금 RTK Fixed 인가
        self.jump_recover = 0
        self.head_est = HeadingEstimator()
        self.head_odo = 0.0
        self._head_go_t = 0.0

        # ── 일시정지 ──
        self._gps_wait = False
        self._gps_bad_t = 0.0
        self._gps_good_t = 0.0
        self._resume_t = 0.0
        self._done_zero_t = None

        self._last_evt = {}
        self.reset_controller()

        qos_latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel_raw', 10)
        self.pub_ctrl = self.create_publisher(Bool, '/control_state', 10)
        self.pub_state = self.create_publisher(String, '/drive_state', 10)
        self.pub_event = self.create_publisher(String, '/drive_event', 50)
        self.pub_status = self.create_publisher(String, '/drive_status', 10)
        self.pub_debug = self.create_publisher(Float64MultiArray, '/driving_debug', 10)
        self.pub_ego = self.create_publisher(Float64MultiArray, '/ego_state', 20)
        self.pub_map = self.create_publisher(Bool, '/mapping_cmd', 10)
        self.pub_dir = self.create_publisher(String, '/route_dir', qos_latched)
        self.create_subscription(Float64MultiArray, GPS_PVT_TOPIC, self.cb_pvt, 20)
        self.create_subscription(String, '/drive_cmd', self.cb_drive_cmd, 10)
        self.create_timer(1.0 / CONTROL_HZ, self.loop)
        self.create_timer(0.2, self.publish_state)
        self.pub_dir.publish(String(data=self.data_dir))

        self.get_logger().info(
            f"🚀 driving [white1 — 단일 GPS(전방 안테나), IMU 없음] 순항 {self.max_speed * 3.6:.0f} km/h · "
            f"최저 {self.min_speed * 3.6:.0f} · 헤딩 직진 {self.heading_speed * 3.6:.0f} km/h · "
            f"출발은 RTK Fixed 에서만, 주행 중 {Q_LABEL[self.run_min_q]} 이상이면 계속 · "
            f"안테나 = 뒷차축 앞 {self.ant_base:.2f} m · 도착 {self.wp_reach} m · 이탈 {self.cte_dev} m | "
            f"경로 폴더 {self.data_dir}")

    def reset_controller(self):
        """black load_waypoints 가 주행마다 지우던 제어 상태."""
        now = time.time()
        self.cte_integral = 0.0
        self.prev_cte = 0.0
        self.cte_lpf = 0.0
        self.cte_lpf_ready = False
        self.prev_cte_seg_idx = 0
        self.d_term_lpf = 0.0
        self.spd_integral = 0.0
        self.last_published_speed = 0.0
        self._lfd_lpf = MIN_LFD
        self._lfd_out = MIN_LFD
        self._near_demand_lpf = 0.0
        self._near_peak_lpf = 0.0
        self._cte_rate_lpf = 0.0
        self.v_sched = 0.0
        self.prev_steer = 0.0
        self.approach_reset_done = False
        self.ramp_up_active = False
        self.ramp_v0 = 0.0
        self.drive_start_time = now
        self.last_integral_decay = now
        self.last_control_time = None
        self.last_status_publish = 0.0

    # ══════════════════════════════════════════════════════════════════════════
    #  이벤트
    # ══════════════════════════════════════════════════════════════════════════
    def event(self, text, level='info'):
        self.pub_event.publish(String(data=text))
        getattr(self.get_logger(), {'warn': 'warning'}.get(level, level))(text)

    def throttle_event(self, text, period=5.0, key=None):
        k = key or text[:12]
        now = time.time()
        if now - self._last_evt.get(k, 0.0) >= period:
            self._last_evt[k] = now
            self.event(text)

    # ══════════════════════════════════════════════════════════════════════════
    #  센서 콜백 — 위치 · 헤딩 · 속도
    # ══════════════════════════════════════════════════════════════════════════
    def cb_pvt(self, msg: Float64MultiArray):
        """전방 GPS 한 점 → 속도 · 품질, 그리고 (헤딩을 잡은 뒤에는) 뒷차축 자리 · 헤딩."""
        d = msg.data
        if len(d) < GPS_PVT_FIELDS:
            self.throttle_event(f"⚠️ {GPS_PVT_TOPIC} 길이 {len(d)} — broon/gps 규약 불일치")
            return
        lat, lon, q = float(d[0]), float(d[1]), int(d[2])
        if q == Q_NONE or not (math.isfinite(lat) and math.isfinite(lon)):
            return
        now = time.time()
        self.fix_time = now
        self.gps_q, self.gps_hacc = q, float(d[3])
        self.fix_ok = q >= Q_FIXED
        if math.isfinite(d[4]):
            self._on_speed(float(d[4]), now)
        self._log_quality(q)
        if q < self.run_min_q:
            return                        # 단독 · DGNSS 좌표(오차 수 m)는 위치로 쓰지 않는다 — _hold 가 세운다
        if self.lat0 is None:
            self.lat0, self.lon0 = lat, lon
        fx, fy = latlon_to_xy(lat, lon, self.lat0, self.lon0)
        dt = min(1.0, now - self.pos_time) if self.pos_time > 0.0 else 0.1
        if self.pos_time > 0.0 and math.hypot(fx - self.fx, fy - self.fy) > GPS_JUMP_M \
                and now - self.pos_time < 0.5 and self.state == S_DRIVE_RUN:
            self.jump_recover = 4
            self.event(f"📍 위치 점프 {math.hypot(fx - self.fx, fy - self.fy):.2f}m → WP 재동기화")
        self.fx, self.fy, self.pos_time = fx, fy, now
        if self.state == S_DRIVE_HEADING and self.heading is None and self.fix_ok:
            self.head_est.add(fx, fy)     # 헤딩을 잡는 직진 구간은 Fixed 점만
        if self.tracker.ready:
            # 한 번에 돌 수 있는 양을 막는다(위치 튐 방어). 좌표가 끊겼다 돌아온 첫 점은 끊기기 전 속도로 본다 —
            # 그동안(서면서) 돈 만큼은 받아들여야 헤딩이 곧바로 맞는다
            lim = turn_limit_deg(max(self.speed_ms, self._v_at_pos), dt, WHEELBASE_M)
            self.tracker.feed(fx, fy, max_turn_deg=lim)
            self.x, self.y = self.tracker.rear()
            self.heading = self.tracker.heading
            self.pose_mode = MODE_FRONT
        else:
            self.x, self.y = fx, fy       # 헤딩을 잡기 전 — 안테나 자리 그대로
            self.pose_mode = MODE_NONE
        self._v_at_pos = self.speed_ms
        self._publish_ego()

    def _on_speed(self, raw, now):
        """GPS 지면속도 → 제어용 속도 (white 의 바퀴 속도 저역통과와 같은 식)."""
        if self.gps_speed_t <= 0.0 or now - self.gps_speed_t > 0.5:
            self.speed_ms = raw
        else:
            dt = max(0.001, now - self.gps_speed_t)
            self.speed_ms += dt / (GPS_SPEED_LPF_TAU + dt) * (raw - self.speed_ms)
        self.gps_speed, self.gps_speed_t = raw, now

    def _publish_ego(self):
        la, lo = xy_to_latlon(self.x, self.y, self.lat0, self.lon0)
        self.pub_ego.publish(Float64MultiArray(data=[
            self.x, self.y, self.heading if self.heading is not None else float('nan'),
            self.speed_ms, la, lo, float(self.pose_mode)]))

    def _log_quality(self, q):
        if q != self.gps_q_logged:
            if self.gps_q_logged is not None:
                arrow = "↘ 저하" if q < self.gps_q_logged else "↗ 회복"
                self.event(f"📶 GPS 품질 {arrow} : {Q_LABEL.get(self.gps_q_logged, '?')} → "
                           f"{Q_LABEL.get(q, '?')} (hAcc {self.gps_hacc:.3f} m)")
            self.gps_q_logged = q

    def pose_now(self, now, v):
        """제어용 위치(뒷차축). 좌표가 POS_DR_MIN_S 넘게 끊겼으면 속도 · 헤딩으로 메운다."""
        age = now - self.pos_time
        if self.heading is None or age <= POS_DR_MIN_S or age > POS_DR_MAX_S:
            return self.x, self.y
        h = math.radians(self.heading)
        return self.x + v * age * math.cos(h), self.y + v * age * math.sin(h)

    def gps_speed_now(self, now):
        return self.gps_speed if now - self.gps_speed_t < 0.5 else None

    def current_speed(self, now):
        """제어용 속도 = GPS 지면속도. 0.5 s 넘게 안 오면 0."""
        return self.speed_ms if now - self.gps_speed_t < 0.5 else 0.0

    # ══════════════════════════════════════════════════════════════════════════
    #  명령 (prompt)
    # ══════════════════════════════════════════════════════════════════════════
    def cb_drive_cmd(self, msg: String):
        cmd = str(msg.data).strip()
        if not cmd:
            return
        up = cmd.upper()
        if up == 'STOP':
            if self.state in (S_DRIVE_HEADING, S_DRIVE_RUN):
                self.enter(S_DRIVE_DONE, "🛑 정지 명령")
            elif self.state == S_MAP:
                self.enter(S_IDLE, "🗺️ 매핑 종료")
            elif self.state == S_DRIVE_DONE:
                pass
            else:
                self.enter(S_IDLE)
            return
        if up == 'MAP_START':
            if self.state != S_IDLE:
                self.event(f"⚠️ 매핑 시작 실패 — 현재 상태 {self.state}(IDLE 에서만 가능)")
            else:
                self.enter(S_MAP, "🗺️ 매핑 시작 — 손으로 전진으로 몰 것")
            return
        if up == 'MAP_STOP':
            if self.state == S_MAP:
                self.enter(S_IDLE, "🗺️ 매핑 종료")
            return
        if up == 'DRIVE_START':
            why = self.start_refusal()
            if why:
                self.event(f"⚠️ 주행 시작 실패 — {why}")
                return
            self.enter(S_DRIVE_HEADING,
                       f"▶ 주행 시작 [{self.route_name}] — 조향 0 으로 곧게 {self.heading_speed * 3.6:.0f} km/h, "
                       f"헤딩을 잡을 때까지")
            self.event(f"📶 출발 GPS : {Q_LABEL.get(self.gps_q, '?')} hAcc {self.gps_hacc:.3f} m")
            return
        if self.state != S_IDLE:
            self.event(f"⚠️ 경로 선택 무시 — 현재 상태 {self.state}")
            return
        self.select_route(cmd)

    def start_refusal(self):
        now = time.time()
        if self.state != S_IDLE:
            return f"현재 상태 {self.state} (IDLE 에서만 가능)"
        if not self.raw_wps:
            return "경로가 선택되지 않았다"
        if now - self.fix_time > GPS_TIMEOUT_S:
            return "GPS 위치가 안 들어온다 (broon/gps 노드 · 전방 GPS 확인)"
        if self.gps_q < Q_FIXED or self.lat0 is None:
            return f"RTK Fixed 가 아니다 (지금 {Q_LABEL.get(self.gps_q, '?')}) — 출발은 Fixed 에서만 한다"
        return ""

    def select_route(self, name):
        path = os.path.join(self.data_dir, name)
        if not os.path.isfile(path):
            self.event(f"❌ 경로 파일 없음: {path}")
            return
        try:
            pts, loop, rev, zones = read_route_csv(path)
        except (OSError, UnicodeError, csv.Error, ValueError) as e:   # 어떤 이유든 경로를 못 읽은 것이다
            self.event(f"❌ 경로 읽기 실패 {name}: {e}")
            return
        # 같은 자리 점(2 cm 이내) 제거 — 길이 0 구간은 방위가 없다
        clean = []
        for p in pts:
            if clean:
                dx, dy = latlon_to_xy(p[0], p[1], clean[-1][0], clean[-1][1])
                if math.hypot(dx, dy) < 0.02:
                    continue
            clean.append(p)
        if loop and len(clean) > 2 and math.hypot(*latlon_to_xy(
                clean[-1][0], clean[-1][1], clean[0][0], clean[0][1])) < 0.02:
            clean.pop()          # 첫 점을 끝에 되풀이한 파일 — 이음매 길이 0 구간을 만들지 않게
        need = 3 if loop else 2
        if len(clean) < need:
            self.event(f"❌ 웨이포인트가 부족하다({len(clean)}개): {name}")
            return
        self.route_name, self.raw_wps, self.route_loop = name, clean, loop
        steps = [math.hypot(*latlon_to_xy(b[0], b[1], a[0], a[1])) for a, b in zip(clean, clean[1:])]
        length = sum(steps)
        if loop:
            gap = math.hypot(*latlon_to_xy(clean[0][0], clean[0][1], clean[-1][0], clean[-1][1]))
            self.event(f"📁 경로 선택: {name} (WP {len(clean)}개, 한 바퀴 {length + gap:.1f} m) "
                       f"🔁 순환 — 도착 없이 계속 돈다. Enter 로 멈춘다")
            typical = sorted(steps)[len(steps) // 2]
            if gap > max(LOOP_GAP_WARN_M, 4.0 * typical):
                self.event(f"⚠️ 순환 이음매(끝 → 처음) {gap:.2f} m — 평소 간격 {typical:.2f} m 보다 길다. "
                           f"그 사이는 직선으로 잇는다")
        else:
            self.event(f"📁 경로 선택: {name} (WP {len(clean)}개, {length:.1f} m)")
        if rev:
            self.event(f"⚠️ 후진 표시(dir 1) {rev}개 — 후진은 지원하지 않는다. 전진으로 따른다")
        if zones:
            words = " · ".join(f"{w}({n}줄)" for w, n in zones.items())
            self.event(f"ℹ️ 구역 표시 {words} — white1 은 구역 동작이 없어 무시하고 GPS 추종만 한다")

    def build_waypoints(self):
        if self.lat0 is None or not self.raw_wps:
            return False
        lap = [latlon_to_xy(la, lo, self.lat0, self.lon0) for (la, lo) in self.raw_wps]
        self.lap_wps = len(lap) if self.route_loop else 0
        if self.route_loop:
            # 몇 바퀴 이어 붙인다 — 되돌리기(LOOP_REBASE_BACK) 직전에도 앞쪽 탐색 · 미리보기가 끝에 닿지 않게
            n = len(lap)
            lap_m = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(lap, lap[1:] + lap[:1]))
            copies = 2 + max(math.ceil((LOOP_REBASE_BACK + LOOP_AHEAD_WP) / n),
                             math.ceil(LOOP_AHEAD_M / max(lap_m, 0.1)))
            self.wps = lap * copies
        else:
            self.wps = lap
        self.wp_s = [0.0]
        for (x0, y0), (x1, y1) in zip(self.wps, self.wps[1:]):
            self.wp_s.append(self.wp_s[-1] + math.hypot(x1 - x0, y1 - y0))
        self.wp_idx = 0
        self.laps = 0
        return True

    # ══════════════════════════════════════════════════════════════════════════
    #  상태 전이
    # ══════════════════════════════════════════════════════════════════════════
    def enter(self, new_state, msg=""):
        old = self.state
        now = time.time()
        self.state = new_state
        self.state_t0 = now
        self._gps_wait = False
        self._gps_bad_t = self._gps_good_t = 0.0
        self._resume_t = 0.0
        if msg:
            self.event(msg)

        if new_state == S_MAP:
            self.pub_map.publish(Bool(data=True))
        elif old == S_MAP:
            self.pub_map.publish(Bool(data=False))

        if new_state == S_DRIVE_HEADING:
            self.head_est.reset()
            self.heading = None
            self.tracker.reset()
            self.head_odo = 0.0
            self._head_go_t = now + MODE_SETTLE_S   # 진입 직후는 굴리지 않는다
            self.reset_controller()
            if not self.build_waypoints():
                self.enter(S_IDLE, "❌ GPS 원점이 없어 경로를 세울 수 없다")
                return
        elif new_state == S_DRIVE_RUN:
            v0 = max(SAFE_STOP_SPEED, abs(self.current_speed(now)))
            self.reset_controller()
            self.ramp_up_active = True
            self.ramp_v0 = min(v0, self.max_speed)
            self.last_published_speed = self.ramp_v0
            self.v_sched = self.ramp_v0
            self.drive_start_time = now
        elif new_state == S_DRIVE_DONE:
            self._done_zero_t = None
            self.send(0.0, 0.0, raw=True)

        self.pub_ctrl.publish(Bool(data=new_state in DRIVE_STATES))
        self.publish_state()

    def publish_state(self):
        self.pub_state.publish(String(data=self.state))
        self.pub_ctrl.publish(Bool(data=self.state in DRIVE_STATES))

    # ══════════════════════════════════════════════════════════════════════════
    #  메인 루프 (20 Hz)
    # ══════════════════════════════════════════════════════════════════════════
    def loop(self):
        if self.state in (S_IDLE, S_MAP):
            return
        now = time.time()
        if self.state == S_DRIVE_DONE:
            self.send(0.0, 0.0, raw=True)
            self._check_done_release(now)
            return

        # ── 일시정지 게이트 (gold loop 순서) ──
        if self._hold(now):
            return
        if now < self._resume_t:
            self.send(0.0, 0.0 if self.state == S_DRIVE_HEADING else self.prev_steer)
            return

        if self.state == S_DRIVE_HEADING:
            self.run_heading_init(now)
        elif self.state == S_DRIVE_RUN:
            self.run_follow(now)

    def _hold(self, now):
        """GPS 품질이 모자라면 선다 → 이번 틱을 여기서 끝냈으면 True.

        헤딩을 잡는 직진 구간은 Fixed, 그 밖(경로추종)은 run_min_quality(기본 Float) 이상이어야 한다.
        white 의 RTK 대기와 같은 틀이다 — 경로 · 헤딩은 그대로 두고 서서 기다렸다 그 자리에서 재개한다."""
        need = Q_FIXED if (self.state == S_DRIVE_HEADING and self.heading is None) else self.run_min_q
        lost = now - self.fix_time > GPS_STALE_S
        good = (not lost) and self.gps_q >= need
        if not self._gps_wait:
            if good:
                self._gps_bad_t = 0.0
                return False
            if self._gps_bad_t == 0.0:
                self._gps_bad_t = now
            if now - self._gps_bad_t < GPS_LOST_S:
                return False
            self._gps_wait = True
            self._gps_good_t = 0.0
            what = f"fix {now - self.fix_time:.1f}s 없음" if lost else \
                f"{Q_LABEL.get(self.gps_q, '?')} hAcc {self.gps_hacc:.2f}m"
            self.event(f"🛰️ GPS 품질 부족({what} — {Q_LABEL[need]} 이상이어야 한다) — 서서 기다린다 "
                       f"(경로 · 헤딩 유지, {GPS_RESUME_S:.0f}s 이어지면 그 자리에서 재개)")
        self.send(0.0, self.prev_steer if self.state == S_DRIVE_RUN else 0.0)
        if good:
            if self._gps_good_t == 0.0:
                self._gps_good_t = now
            elif now - self._gps_good_t >= GPS_RESUME_S:
                self._gps_wait = False
                self._gps_bad_t = 0.0
                self._resume_t = now + RESUME_GRACE_S
                self._arm_resume_ramp()
                self.event(f"🟢 GPS {Q_LABEL.get(self.gps_q, '?')} {GPS_RESUME_S:.0f}s — "
                           f"{RESUME_GRACE_S:.1f}s 뒤 그 자리에서 재개")
        else:
            self._gps_good_t = 0.0
            self.throttle_event(f"🛰️ GPS 대기 중 — 지금 {'두절' if lost else Q_LABEL.get(self.gps_q, '?')}",
                                key='gpswait')
        return True

    def _arm_resume_ramp(self):
        """재개는 정지에서 다시 출발이다 — 램프를 다시 걸어 한 번에 튀어 나가지 않게."""
        if self.state == S_DRIVE_HEADING:
            self._head_go_t = self._resume_t
        elif self.state == S_DRIVE_RUN:
            self.ramp_up_active = True
            self.ramp_v0 = SAFE_STOP_SPEED
            self.drive_start_time = self._resume_t
            self.last_published_speed = 0.0

    def _check_done_release(self, now):
        g = self.gps_speed_now(now)
        if g is not None:
            if g > GPS_STOP_SPEED:
                self._done_zero_t = None
            elif self._done_zero_t is None:
                self._done_zero_t = now
            elif now - self._done_zero_t >= DRIVE_DONE_RELEASE_S:
                self.enter(S_IDLE, "🏁 정지 확인 — 메뉴로")
                return
        if now - self.state_t0 > DRIVE_DONE_MAX_S:
            self.enter(S_IDLE, "🏁 정지 대기 시간 초과 — 메뉴로")

    # ══════════════════════════════════════════════════════════════════════════
    #  헤딩 초기화 — 조향 0 으로 곧게 굴려 GPS 변위로 첫 헤딩을 잡는다 (gold)
    # ══════════════════════════════════════════════════════════════════════════
    def run_heading_init(self, now):
        if now < self._head_go_t:
            self.send(0.0, 0.0)
            return
        ramp = min(1.0, (now - self._head_go_t) / HEADING_RAMP_S)
        v_cmd = max(SAFE_STOP_SPEED, self.heading_speed * ramp)
        # ★조향 0 = 트림 보정된 직진★ (δ_ctrl 0 → 발행식이 black 실측 트림 −1.5° 를 상쇄)
        self.send(v_cmd, 0.0)
        self.head_odo += abs(self.current_speed(now)) / CONTROL_HZ

        if not self.fix_ok:
            self.throttle_event(f"⚠️ 헤딩 표본이 쌓이지 않는다 — GPS {Q_LABEL.get(self.gps_q, '?')}",
                                key='headq')
        sol = None
        if self.fix_ok:
            floor = HEAD_SIGMA_FLOOR
            if math.isfinite(self.gps_hacc):
                floor = max(floor, self.gps_hacc)
            sol = self.head_est.solve(floor)
        if sol is not None and sol[4] >= HEAD_MIN_DIST_M:
            heading, sigma, resid, n, dist = sol
            good = n >= HEAD_MIN_SAMPLES and sigma <= HEAD_TARGET_SIGMA and resid <= HEAD_MAX_RESID_M
            forced = dist >= HEAD_MAX_DIST_M and n >= HEAD_FORCED_MIN_SAMPLES
            if good or forced:
                self._lock_heading(heading, sigma, resid, n, dist, good)
                return
        if self.head_odo >= HEAD_ABORT_DIST_M:
            self.enter(S_DRIVE_DONE,
                       f"⛔ 헤딩 미확정 — {self.head_odo:.1f} m 곧게 갔는데(GPS 속도 적분) GPS 변위가 "
                       f"{self.head_est.distance():.2f} m · 표본 {len(self.head_est.pts)}개. 정지")

    def _lock_heading(self, heading, sigma, resid, n, dist, good):
        """직진 변위로 낸 헤딩으로 전방 안테나 추적을 시작한다 — 이때부터 self.x · y 는 뒷차축이다."""
        self.tracker.set(self.fx, self.fy, heading)
        self.heading = heading
        self.x, self.y = self.tracker.rear()
        self.pose_mode = MODE_FRONT
        mark = "" if good else "  ⚠️(최대거리 도달 — 정확도 미달인 채 확정)"
        self.event(f"🧭 헤딩 확정 {heading:+.1f}° (±{sigma:.1f}°, {dist:.2f}m, {n}점, "
                   f"잔차 {resid*100:.1f}cm){mark} — 이후는 전방 안테나 궤적으로 잇는다 "
                   f"(뒷차축 = 안테나 뒤 {self.ant_base:.2f} m)")
        best_i, best_d = self.align_start_wp()
        if best_d > self.cte_dev:
            self.enter(S_DRIVE_DONE, f"⛔ 출발 위치가 경로에서 {best_d:.1f} m 떨어져 있다 "
                                     f"(한계 {self.cte_dev:.1f} m) — 경로 시작점 근처에 세우고 다시")
            return
        total = self.lap_wps or len(self.wps)
        self.enter(S_DRIVE_RUN, f"📍 출발 WP {best_i % total}/{total} (최근접 {best_d:.2f} m) → 경로추종"
                                + (" 🔁 순환" if self.lap_wps else ""))

    def align_start_wp(self):
        """출발 WP 를 현재 위치에 맞춘다 (gold + black).

        경로 ★앞쪽 절반★ 에서 최근접 WP 를 찾고(black — 처음과 끝이 맞닿은 열린 경로에서 종점을 출발점으로
        잡지 않게), 그것이 등 뒤(방위차 90° 초과)면 하나 앞을 잡는다(gold — 헤딩 초기화로 이미 지나온 점).
        순환 경로(return)는 끝이 없으므로 ★한 바퀴 전체★ 에서 찾는다.
        """
        n = len(self.wps)
        limit = self.lap_wps if self.lap_wps else max(20, n // 2)
        best_i, best_d = 0, float('inf')
        for i in range(min(limit, n)):
            wx, wy = self.wps[i]
            d = math.hypot(wx - self.x, wy - self.y)
            if d < best_d:
                best_i, best_d = i, d
        if self.heading is not None and best_i < n - 1:
            wx, wy = self.wps[best_i]
            ang = wrap180(math.degrees(math.atan2(wy - self.y, wx - self.x)) - self.heading)
            if abs(ang) > 90.0:
                best_i += 1
        self.wp_idx = best_i
        return best_i, best_d

    # ══════════════════════════════════════════════════════════════════════════
    #  black 인지 함수 (x, y 판)
    # ══════════════════════════════════════════════════════════════════════════
    def scan_curve_demand(self, from_idx, preview_dist):
        """전방 곡률 → 요구 조향각. (창 평균 최대, 그 거리, 짧은 창 Menger 피크, 그 거리)."""
        wps = self.wps
        if len(wps) < 3:
            return 0.0, float('inf'), 0.0, float('inf')
        start = max(0, min(from_idx, len(wps) - 3))
        pts = [wps[start]]
        s = [0.0]
        for i in range(start + 1, len(wps)):
            x, y = wps[i]
            s.append(s[-1] + math.hypot(x - pts[-1][0], y - pts[-1][1]))
            pts.append((x, y))
            if s[-1] > preview_dist:
                break
        if len(pts) < 3:
            return 0.0, float('inf'), 0.0, float('inf')

        def idx_at(dist_m):
            for k in range(1, len(s)):
                if s[k] >= dist_m:
                    return k
            return len(s) - 1

        def heading_between(i0, i1):
            return math.atan2(pts[i1][1] - pts[i0][1], pts[i1][0] - pts[i0][0])

        win, win_pk = CURVE_WINDOW_M, CURVE_WINDOW_PEAK_M
        max_steer_deg, dist_at_max = 0.0, float('inf')
        peak_steer_deg, peak_dist = 0.0, float('inf')
        d = win_pk
        scan_end = min(preview_dist, s[-1])
        while d <= scan_end:
            if d >= win:
                i_p0 = idx_at(max(0.0, d - win))
                i_p1 = idx_at(d)
                i_c1 = idx_at(min(d + win, s[-1]))
                if i_p1 > i_p0 and i_c1 > i_p1:
                    diff = wrap_pi(heading_between(i_p1, i_c1) - heading_between(i_p0, i_p1))
                    req = math.degrees(math.atan(WHEELBASE_M * abs(diff) / win))
                    if req > max_steer_deg:
                        max_steer_deg, dist_at_max = req, d
            i_pm = idx_at(d)
            j0 = idx_at(max(0.0, d - win_pk))
            j2 = idx_at(min(d + win_pk, s[-1]))
            if j0 < i_pm < j2:
                (ax, ay), (bx, by), (cx, cy) = pts[j0], pts[i_pm], pts[j2]
                la = math.hypot(bx - ax, by - ay)
                lb = math.hypot(cx - bx, cy - by)
                lc = math.hypot(cx - ax, cy - ay)
                area2 = abs((bx - ax) * (cy - ay) - (cx - ax) * (by - ay))
                if area2 > 1e-6 and la * lb * lc > 1e-9:
                    r_menger = (la * lb * lc) / (2.0 * area2)
                    req_pk = math.degrees(math.atan(WHEELBASE_M / r_menger))
                    if req_pk > peak_steer_deg:
                        peak_steer_deg, peak_dist = req_pk, d
            d += CURVE_SCAN_STEP_M
        return max_steer_deg, dist_at_max, peak_steer_deg, peak_dist

    def segment_cte(self, from_idx, px, py, psi, back=3, ahead=25):
        """(px, py) 의 경로 횡오차. → (cte 차체좌표 횡성분 [+ = 경로가 왼쪽], 구간 idx, 최근접 거리)."""
        wps = self.wps
        if len(wps) < 2:
            return 0.0, from_idx, 0.0
        start = max(0, from_idx - back)
        end = min(len(wps) - 2, from_idx + ahead)
        best_dist, best_x, best_y, best_i = float('inf'), 0.0, 0.0, from_idx
        for i in range(start, end + 1):
            x0, y0 = wps[i][0] - px, wps[i][1] - py
            x1, y1 = wps[i + 1][0] - px, wps[i + 1][1] - py
            vx, vy = x1 - x0, y1 - y0
            denom = vx * vx + vy * vy
            if denom < 1e-6:
                continue
            t = max(0.0, min(1.0, -(x0 * vx + y0 * vy) / denom))
            qx, qy = x0 + t * vx, y0 + t * vy
            dist = math.hypot(qx, qy)
            if dist < best_dist:
                best_dist, best_x, best_y, best_i = dist, qx, qy, i
        cte = -best_x * math.sin(psi) + best_y * math.cos(psi)
        return cte, best_i, best_dist

    def final_path_heading(self):
        n = len(self.wps)
        (ax, ay), (bx, by) = self.wps[max(0, n - 9)], self.wps[-1]
        if math.hypot(bx - ax, by - ay) < 1e-6:
            ax, ay = self.wps[max(0, n - 2)]
        return math.atan2(by - ay, bx - ax)

    def final_line_target(self, x, y, lookahead_m, overshoot_max_m=1.0):
        dx_, dy_ = self.wps[-1]
        h = self.final_path_heading()
        ux, uy = math.cos(h), math.sin(h)
        t = (x - dx_) * ux + (y - dy_) * uy
        t_tgt = min(overshoot_max_m, t + lookahead_m)
        return dx_ + t_tgt * ux, dy_ + t_tgt * uy

    def demand_to_speed(self, demand_deg):
        r = max(0.0, min(1.0, demand_deg / STEER_FULL_SLOWDOWN_DEG))
        return self.max_speed - (self.max_speed - self.min_speed) * r

    # ══════════════════════════════════════════════════════════════════════════
    #  경로추종 — black v6.7.3 control_loop (인지 → 판단 → 제어)
    #  출발 · 도착 · 이탈 판정만 gold. 주석의 [v6.x]·[07xx] 는 black 쪽 이력 표기.
    # ══════════════════════════════════════════════════════════════════════════
    def run_follow(self, now):
        loop_dt = 0.05 if self.last_control_time is None else \
            max(0.01, min(0.2, now - self.last_control_time))
        self.last_control_time = now
        loop = bool(self.lap_wps)
        if loop and self.wp_idx >= self.lap_wps + LOOP_REBASE_BACK:
            # 순환 — 한 바퀴 되돌린다. 이어 붙인 경로는 바퀴마다 같으므로 위치 · 제어 상태는 그대로다
            self.wp_idx -= self.lap_wps
            self.prev_cte_seg_idx -= self.lap_wps
            self.laps += 1
            self.event(f"🔁 {self.laps}바퀴 — 계속 돈다 (Enter 로 정지)")
        wps = self.wps
        wp_total = len(wps)
        v_signed = self.current_speed(now)
        v_now = abs(v_signed)
        x, y = self.pose_now(now, v_signed)
        psi_raw = math.radians(self.heading)

        # ── [v6.7] 지연 보상: τ초 뒤 자세 예측 → 이 자세로 WP탐색·CTE·PP ──
        #   (도착 · 이탈 · d2dest 는 원시좌표 — 예측으로 일찍 서지 않게)
        tau = PREDICT_HORIZON_S
        imu_fresh = False            # IMU 가 없다 — 요레이트는 직전 조향 명령으로 추정한다 (black 의 IMU 두절 경로)
        omega_z = v_signed * math.tan(math.radians(self.prev_steer)) / WHEELBASE_M
        dpsi = max(-PREDICT_MAX_YAW_RAD, min(PREDICT_MAX_YAW_RAD, omega_z * tau))
        psi_mid = psi_raw + 0.5 * dpsi
        adv = v_signed * tau
        cx = x + adv * math.cos(psi_mid)
        cy = y + adv * math.sin(psi_mid)
        psi = psi_raw + dpsi
        cos_p, sin_p = math.cos(psi), math.sin(psi)
        elapsed_drive = max(0.0, now - self.drive_start_time)

        # ── 인지 1. WP 탐색 ──
        search_window, max_advance = WP_SEARCH_WINDOW, MAX_WP_ADVANCE_NORMAL
        if self.wp_idx < 5 and elapsed_drive < START_ALIGN_DURATION:
            search_window, max_advance = max(search_window, 120), MAX_WP_ADVANCE_START
        elif self.jump_recover > 0:
            search_window, max_advance = max(search_window, 70), max(max_advance, 12)
            self.jump_recover -= 1
        search_end = min(self.wp_idx + search_window, wp_total)
        min_ahead, best_ahead = float('inf'), None
        min_any, best_any = float('inf'), self.wp_idx
        for i in range(self.wp_idx, search_end):
            dx, dy = wps[i][0] - cx, wps[i][1] - cy
            lx = dx * cos_p + dy * sin_p
            d = math.hypot(dx, dy)
            if d < min_any:
                min_any, best_any = d, i
            if lx > -2.0 and d < min_ahead:
                min_ahead, best_ahead = d, i
        best = best_any if (best_ahead is None or min_ahead > min_any + 1.2) else best_ahead
        self.wp_idx = max(self.wp_idx, min(best, self.wp_idx + max_advance))

        # ── 경로이탈 안전정지 (gold) — 원시 위치에서 경로까지 거리 ──
        _, _, dev = self.segment_cte(self.wp_idx, x, y, psi_raw, back=10, ahead=40)
        if dev > self.cte_dev:
            self.enter(S_DRIVE_DONE, f"⛔ 경로이탈 안전정지 — 경로에서 {dev:.2f} m (한계 {self.cte_dev:.1f} m)")
            return

        # ── 인지 2. 도착 판정 (gold: 반경 or 통과, 종점 부근에서만) — 순환 경로는 도착이 없다 ──
        gx, gy = wps[-1]
        d2dest = 99.0 if loop else math.hypot(gx - x, gy - y)
        s_left = self.wp_s[-1] - self.wp_s[min(self.wp_idx, wp_total - 1)]
        if not loop and s_left <= GOAL_ZONE_M:
            if d2dest <= self.wp_reach:
                self.enter(S_DRIVE_DONE, f"🎯 도착 — 마지막 WP {d2dest:.2f} m, v {v_now:.2f} m/s")
                return
            if (gx - x) * math.cos(psi_raw) + (gy - y) * math.sin(psi_raw) < 0.0 \
                    and d2dest <= GOAL_PASS_MAX_M:
                self.enter(S_DRIVE_DONE, f"🎯 도착(통과) — 마지막 WP 를 {d2dest:.2f} m 지났다")
                return

        # ── 종점 접근 감속 (black APPROACH — 브레이크가 없어 속도로 줄인다) ──
        final_zone_idx = max(0, wp_total - FINAL_APPROACH_WP_WINDOW)
        approach_active = (not loop) and self.wp_idx >= final_zone_idx and d2dest <= APPROACH_START_DIST
        approach_speed_cap = self.max_speed
        final_heading_error = math.degrees(wrap_pi(self.final_path_heading() - psi_raw))
        if approach_active:
            ratio = max(0.0, min(1.0, d2dest / APPROACH_START_DIST))
            approach_speed_cap = APPROACH_END_SPEED + \
                (FINAL_APPROACH_ENTRY_SPEED - APPROACH_END_SPEED) * (ratio ** 1.3)
            if d2dest <= FINAL_CREEP_DIST:
                creep_cap = APPROACH_END_SPEED + \
                    (FINAL_CREEP_SPEED - APPROACH_END_SPEED) * (d2dest / FINAL_CREEP_DIST)
                approach_speed_cap = min(approach_speed_cap, creep_cap)
            if abs(final_heading_error) > FINAL_HEADING_TOL_DEG:
                approach_speed_cap = min(approach_speed_cap, max(0.35, 0.18 + 0.18 * d2dest))
            approach_speed_cap = max(APPROACH_END_SPEED, min(FINAL_APPROACH_ENTRY_SPEED, approach_speed_cap))
            if not self.approach_reset_done and d2dest < APPROACH_START_DIST * 0.6:
                self.cte_integral = 0.0
                self.d_term_lpf = 0.0
                self.approach_reset_done = True
                self.event(f"🎯 종점 접근 (d2dest={d2dest:.2f}m)")
        else:
            self.approach_reset_done = False

        # ── 인지 3. 전방 곡률 → 요구 조향각 ──
        self.v_sched += loop_dt / (V_SCHED_TAU + loop_dt) * (v_now - self.v_sched)
        near_preview = max(MIN_LFD * CURVE_PREVIEW_NEAR_K, self._lfd_lpf * CURVE_PREVIEW_NEAR_K)
        near_demand_raw, _, near_peak_raw, _ = self.scan_curve_demand(self.wp_idx, near_preview)
        self._near_demand_lpf += NEAR_DEMAND_LPF_ALPHA * (near_demand_raw - self._near_demand_lpf)
        near_demand_deg = self._near_demand_lpf
        self._near_peak_lpf += NEAR_DEMAND_LPF_ALPHA * (
            max(near_demand_raw, near_peak_raw) - self._near_peak_lpf)
        near_peak_deg = self._near_peak_lpf
        far_demand_deg, far_dist, far_peak, far_peak_dist = self.scan_curve_demand(
            self.wp_idx, CURVE_PREVIEW_FAR_MAX)
        far_peak_deg = max(far_demand_deg, far_peak)

        # ── 인지 4. CTE (예측 자세) ──
        cte_raw, cte_seg_idx, _ = self.segment_cte(self.wp_idx, cx, cy, psi)
        if (not self.cte_lpf_ready or abs(cte_seg_idx - self.prev_cte_seg_idx) > 20
                or abs(cte_raw - self.cte_lpf) > 6.0):
            self.cte_lpf = cte_raw
            self.cte_integral = 0.0
            self.d_term_lpf = 0.0
            self.cte_lpf_ready = True
        else:
            self.cte_lpf += CTE_LPF_ALPHA * (cte_raw - self.cte_lpf)
        self.prev_cte_seg_idx = cte_seg_idx
        cte = max(-4.0, min(4.0, self.cte_lpf))
        self.wp_idx = max(self.wp_idx, cte_seg_idx)
        cte_abs = abs(cte)

        # ── [v6.6.1] 휠베이스 기하 lead: cte + L·sin(θe) ──
        si = max(0, min(cte_seg_idx, wp_total - 2))
        (w0x, w0y), (w1x, w1y) = wps[si], wps[si + 1]
        if math.hypot(w1x - w0x, w1y - w0y) > 1e-6:
            he = wrap_pi(math.atan2(w1y - w0y, w1x - w0x) - psi)
            lead_raw = LEAD_WHEELBASE_MULT * WHEELBASE_M * math.sin(he)
        else:
            lead_raw = 0.0
        self._cte_rate_lpf += LEAD_RATE_LPF_ALPHA * (lead_raw - self._cte_rate_lpf)
        align_boost = 1.0 + HEADING_ALIGN_BOOST * (1.0 - min(1.0, cte_abs / HEADING_ALIGN_CTE_REF))
        cte_lead = max(-LEAD_CLAMP, min(LEAD_CLAMP, self._cte_rate_lpf * align_boost))
        cte_ctrl = max(-4.0, min(4.0, cte + cte_lead))
        is_straight = far_peak_deg < JITTER_DEMAND_DEG

        # ── 판단 1. LFD = 속도표 ∩ 곡률캡 ──
        lfd_speed = table_lookup(LFD_TABLE, self.v_sched)[0]
        if near_peak_deg > 2.5:
            r_ahead = WHEELBASE_M / math.tan(math.radians(min(near_peak_deg, 25.0)))
            lfd_curve_cap = math.sqrt(LFD_CURVE_CAP_K * r_ahead)
        else:
            lfd_curve_cap = MAX_LFD
        if near_demand_deg > 2.5:
            r_win = WHEELBASE_M / math.tan(math.radians(min(near_demand_deg, 25.0)))
            lfd_cap_win = math.sqrt(LFD_CURVE_CAP_K * r_win)
        else:
            lfd_cap_win = MAX_LFD
        lfd_win_only = min(MAX_LFD, max(MIN_LFD, min(lfd_speed, lfd_cap_win)))
        lfd_target = min(MAX_LFD, max(MIN_LFD, min(lfd_speed, lfd_curve_cap)))
        self._lfd_lpf += LFD_LPF_ALPHA * (lfd_target - self._lfd_lpf)
        self._lfd_out = min(self._lfd_out + LFD_RATE_UP * loop_dt,
                            max(self._lfd_out - LFD_RATE_DOWN * loop_dt, self._lfd_lpf))
        dynamic_lfd = max(MIN_LFD, min(MAX_LFD, self._lfd_out))
        if approach_active:
            lfd_cap = max(APPROACH_LFD_MIN, d2dest * APPROACH_LFD_RATIO + APPROACH_LFD_OFFSET)
            dynamic_lfd = min(self._lfd_lpf, lfd_cap)

        # ── 판단 2. 목표 속도 — 선행 가감속 ──
        near_for_speed = near_demand_deg + PEAK_SPEED_BLEND * max(0.0, near_peak_deg - near_demand_deg)
        v_curve = self.demand_to_speed(near_for_speed)
        reason = "MAX" if v_curve >= self.max_speed - 0.02 else "CURVE"
        omega_n_max = OMEGA_N_MAX if (tau > 1e-3 and imu_fresh) else OMEGA_N_MAX_NO_IMU

        far_for_gate = far_demand_deg + PEAK_SPEED_BLEND * max(0.0, far_peak_deg - far_demand_deg)
        gate_dist = far_dist
        if far_peak_dist != float('inf') and far_peak > far_demand_deg + 1.0:
            gate_dist = min(gate_dist, far_peak_dist)
        v_target = v_curve
        if gate_dist != float('inf') and far_for_gate > near_demand_deg + 1.0:
            v_corner = self.demand_to_speed(far_for_gate)
            if far_peak_deg > 2.5:
                r_far = WHEELBASE_M / math.tan(math.radians(min(far_peak_deg, 25.0)))
                cap_far = min(MAX_LFD, max(MIN_LFD, math.sqrt(LFD_CURVE_CAP_K * r_far)))
                v_corner = min(v_corner, omega_n_max * cap_far / 1.41421356)
            brake_d = max(0.0, gate_dist - BRAKE_GATE_MARGIN)
            v_brake = math.sqrt(max(0.0, v_corner * v_corner + 2.0 * BRAKE_GATE_DECEL * brake_d))
            if v_brake < v_target:
                v_target, reason = v_brake, "BRAKE"
        if (gate_dist != float('inf') and far_peak_deg > GATE_HOLD_DEMAND_DEG
                and gate_dist < GATE_HOLD_DIST_M):
            v_hold = max(GATE_HOLD_SPEED, self.demand_to_speed(far_peak_deg))
            if v_hold < v_target:
                v_target, reason = v_hold, "BRAKE"
        if not approach_active and lfd_win_only < (lfd_speed - 0.05):
            v_wn = omega_n_max * lfd_win_only / 1.41421356
            if v_wn < v_target:
                v_target, reason = v_wn, "OMEGA"
        if cte_abs > CTE_RECOVER_START:
            cte_cap = self.max_speed
            for thr, cap in CTE_RECOVER_CAPS:
                if cte_abs >= thr:
                    cte_cap = cap
            if cte_cap < v_target:
                v_target, reason = cte_cap, "CTE_RECOVERY"
        if is_straight and not approach_active:
            steer_mag = abs(self.prev_steer)
            if steer_mag > STEER_SPEED_START_DEG:
                steer_cap = self.max_speed
                for thr, cap in STEER_SPEED_CAPS:
                    if steer_mag >= thr:
                        steer_cap = cap
                if steer_cap < v_target:
                    v_target, reason = steer_cap, "STEER"
        v_target = max(self.min_speed, min(self.max_speed, v_target))

        # 속도 PI(미세보정)
        spd_err = v_target - v_now
        self.spd_integral += spd_err * (0.1 if v_now / max(v_target, 0.01) < 0.30 else 1.0)
        self.spd_integral = max(-MAX_SPD_INTEG, min(MAX_SPD_INTEG, self.spd_integral))
        spd_corr = SPD_KP * spd_err + SPD_KI * self.spd_integral
        pre_pi = v_target
        v_target += spd_corr * 0.22 if spd_corr >= 0 else spd_corr * 0.06
        v_target = max(SAFE_STOP_SPEED, v_target)
        if v_target < pre_pi - 0.12:
            reason = "SPEED_PI"

        # 출발 점진 가속 (헤딩 초기화 속도 · 재개 시 0 에서 이어 붙인다)
        if self.ramp_up_active:
            ramp_ratio = min(1.0, max(0.0, now - self.drive_start_time) / RAMP_UP_DURATION)
            ramp_max = self.ramp_v0 + (self.max_speed - self.ramp_v0) * ramp_ratio
            if ramp_max < v_target:
                reason = "RAMP"
            v_target = min(v_target, max(SAFE_STOP_SPEED, ramp_max))
            if ramp_ratio >= 1.0:
                self.ramp_up_active = False

        normal_floor = not self.ramp_up_active and not approach_active
        if normal_floor:
            v_target = max(self.min_speed, v_target)

        # 속도 슬루레이트 (가속 부드럽게 / 감속 빠르게)
        max_accel = MAX_ACCEL_PER_S * loop_dt
        decel_rate = CORNER_DECEL_PER_S if reason in ("BRAKE", "CTE_RECOVERY") else MAX_DECEL_PER_S
        if approach_active:
            decel_rate = max(decel_rate, 5.0)
        max_decel = decel_rate * loop_dt
        prev_v = abs(self.last_published_speed)
        if v_target > prev_v + max_accel:
            v_target = prev_v + max_accel
            if reason == "MAX":
                reason = "ACCEL_LIMIT"
        elif prev_v - v_target > max_decel:
            v_target = max(SAFE_STOP_SPEED, prev_v - max_decel)
            reason = "DECEL_LIMIT"
        if normal_floor:
            v_target = max(self.min_speed, v_target)
        v_target = max(SAFE_STOP_SPEED, v_target)
        if approach_active and approach_speed_cap < v_target:
            v_target, reason = max(SAFE_STOP_SPEED, approach_speed_cap), "APPROACH"

        # ── 제어 1. Pure Pursuit ──
        pp_idx = min(self.wp_idx + 1, wp_total - 1)
        for i in range(self.wp_idx, wp_total):
            dx, dy = wps[i][0] - cx, wps[i][1] - cy
            if math.hypot(dx, dy) >= dynamic_lfd and dx * cos_p + dy * sin_p > 0:
                pp_idx = i
                break
        tgt_x, tgt_y = wps[pp_idx]
        blend_start = FINAL_HEADING_ALIGN_DIST + FINAL_BLEND_BAND
        if approach_active and d2dest <= blend_start:
            fx, fy = self.final_line_target(x, y, dynamic_lfd, 1.0)
            if d2dest <= FINAL_HEADING_ALIGN_DIST:
                tgt_x, tgt_y = fx, fy
            else:
                blend = (d2dest - FINAL_HEADING_ALIGN_DIST) / FINAL_BLEND_BAND
                tgt_x = tgt_x * blend + fx * (1.0 - blend)
                tgt_y = tgt_y * blend + fy * (1.0 - blend)
        dx, dy = tgt_x - cx, tgt_y - cy
        lx = dx * cos_p + dy * sin_p
        ly = -dx * sin_p + dy * cos_p
        if lx <= 0.1:
            fbx, fby = wps[min(self.wp_idx + 1, wp_total - 1)]
            dxf, dyf = fbx - cx, fby - cy
            lxf = dxf * cos_p + dyf * sin_p
            lyf = -dxf * sin_p + dyf * cos_p
            if lxf > 0.05:
                alpha = math.atan2(lyf, lxf)
                fb_dist = max(dynamic_lfd * 0.5, math.hypot(dxf, dyf))
                pp_steer = math.degrees(math.atan2(2.0 * WHEELBASE_M * math.sin(alpha), fb_dist))
                pp_steer = max(-STEER_MAX_DEG, min(STEER_MAX_DEG, pp_steer))
            else:
                pp_steer = 0.0
        else:
            alpha = math.atan2(ly, lx)
            pp_steer = math.degrees(math.atan2(2.0 * WHEELBASE_M * math.sin(alpha), dynamic_lfd))

        # ── 제어 2. PID (게인표, PP 보조 + 정상상태 오차 제거) ──
        curr_kp, curr_ki, curr_kd = table_lookup(GAIN_TABLE, self.v_sched)
        active_deadband = CTE_DEADBAND_STRAIGHT if is_straight else CTE_DEADBAND
        if (cte * self.prev_cte) < 0 and cte_abs > active_deadband and abs(self.prev_cte) > active_deadband:
            self.cte_integral *= 0.35
        if cte_abs < active_deadband:
            self.cte_integral *= 0.80
        else:
            self.cte_integral += cte * loop_dt
        self.cte_integral = max(-KI_CLAMP, min(KI_CLAMP, self.cte_integral))
        if now - self.last_integral_decay > INTEGRAL_DECAY_INTERVAL:
            if abs(self.cte_integral) > KI_CLAMP * 0.90:
                self.cte_integral *= 0.85
            self.last_integral_decay = now
        d_cte_per_sec = (cte - self.prev_cte) / max(1e-3, loop_dt)
        self.prev_cte = cte
        self.d_term_lpf += D_TERM_LPF_ALPHA * (d_cte_per_sec - self.d_term_lpf)
        p_term = curr_kp * cte_ctrl
        i_term = curr_ki * self.cte_integral
        d_term = curr_kd * self.d_term_lpf * D_TERM_SCALE
        pid_steer = max(-PID_STEER_LIMIT, min(PID_STEER_LIMIT, p_term + i_term + d_term))
        raw_steer = pp_steer + pid_steer

        # ── 제어 3. 직선 잔떨림 억제 ──
        if (is_straight and not approach_active and cte_abs < JITTER_CTE_M
                and abs(raw_steer) < JITTER_STEER_DEG and abs(cte_lead) < JITTER_LEAD_M):
            if abs(raw_steer) < JITTER_ZERO_DEG:
                raw_steer = 0.0
            else:
                raw_steer = math.copysign((abs(raw_steer) - 0.35) * 0.72, raw_steer)
        raw_steer = max(-STEER_MAX_DEG - 4.0, min(STEER_MAX_DEG + 4.0, raw_steer))

        # ── 제어 4. 조향 슬루레이트 + 클램프 + 저속 권한 램프 ──
        speed_ratio = max(0.0, min(1.0, (v_now - self.min_speed) / max(1e-6, self.max_speed - self.min_speed)))
        smoothed = raw_steer * STEER_SMOOTH_ALPHA + self.prev_steer * (1.0 - STEER_SMOOTH_ALPHA)
        if is_straight and not approach_active:
            step_limit = STEER_STEP_STRAIGHT_DEG
        else:
            step_limit = STEER_STEP_LOW_DEG + (STEER_STEP_HIGH_DEG - STEER_STEP_LOW_DEG) * speed_ratio
        if cte_abs > 0.55 or abs(raw_steer) > 12.0:
            step_limit = max(step_limit, STEER_STEP_RECOVER_DEG)
        delta = smoothed - self.prev_steer
        if delta > step_limit:
            smoothed = self.prev_steer + step_limit
        elif delta < -step_limit:
            smoothed = self.prev_steer - step_limit
        self.prev_steer = smoothed
        clamped_steer = max(-STEER_MAX_DEG, min(STEER_MAX_DEG, smoothed))
        clamped_steer *= min(1.0, max(0.15, v_now / STEER_AUTHORITY_FULL_SPEED))

        self.send(v_target, clamped_steer)

        self.pub_debug.publish(Float64MultiArray(data=[float(v) for v in (
            self.wp_idx, wp_total, d2dest, cte, cte_raw,
            self.cte_integral, dynamic_lfd, speed_ratio, raw_steer, clamped_steer,
            step_limit, v_target, v_target, v_signed, near_demand_deg,
            (far_dist if far_dist != float('inf') else 99.0), far_demand_deg, p_term, i_term, d_term,
            self.d_term_lpf, loop_dt, 0.0, near_peak_deg, 1.0 if approach_active else 0.0,
            SPD_REASON_CODE.get(reason, -1), far_peak_deg)]))
        if now - self.last_status_publish >= 1.0:
            self.last_status_publish = now
            curve = "curve=none" if far_dist == float('inf') else f"curve={far_demand_deg:.0f}°@{far_dist:.1f}m"
            if loop:
                k = self.wp_idx % self.lap_wps
                where = f"WP={k}/{self.lap_wps} ({k / self.lap_wps * 100.0:.0f}%) 🔁{self.laps}바퀴"
                goal = "순환"
            else:
                where = f"WP={self.wp_idx}/{wp_total} ({self.wp_idx / max(wp_total - 1, 1) * 100.0:.0f}%)"
                goal = f"d2dest={d2dest:.1f}m"
            self.pub_status.publish(String(data=(
                f"[{Q_LABEL.get(self.gps_q, '?')}] {where} | "
                f"tgt={v_target:.2f} real={v_signed:+.2f}m/s | CTE={cte:+.2f}m | {reason} | "
                f"{curve} | {goal} | 헤딩 {self.heading:+.0f}°")))

    # ══════════════════════════════════════════════════════════════════════════
    #  발행
    # ══════════════════════════════════════════════════════════════════════════
    def send(self, speed_ms, steer_deg, raw=False):
        """black publish_cmd — δ_pub = (δ_ctrl − TRIM)/GAIN (좌우 게인 분리). raw 면 보정 없이 그대로.
        ★속도는 km/h 로 내보낸다★ — broon/arduino 가 정수 km/h · 정수 도로 반올림해 보낸다."""
        speed_ms = max(-self.max_speed, min(self.max_speed, float(speed_ms)))
        if raw:
            steer_out = float(steer_deg)
        else:
            gain = STEER_PLANT_GAIN_R if steer_deg < 0.0 else STEER_PLANT_GAIN_L
            steer_out = (float(steer_deg) - STEER_TRIM_DEG) / max(0.5, gain)
        steer_out = max(-STEER_MAX_DEG, min(STEER_MAX_DEG, steer_out))
        msg = Twist()
        msg.linear.x = speed_ms * 3.6
        msg.angular.z = steer_out
        self.pub_cmd.publish(msg)
        self.last_published_speed = speed_ms

    def shutdown(self):
        if self.state in DRIVE_STATES:
            self.send(0.0, 0.0, raw=True)
        self.pub_ctrl.publish(Bool(data=False))
        if self.state == S_MAP:
            self.pub_map.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = DrivingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # Ctrl-C 두 번째(launch 가 다시 보낸다)가 정리를 끊지 않게
        try:
            node.shutdown()
        except Exception:      # noqa: BLE001 — 내려가는 중 컨텍스트가 이미 닫혔을 수 있다
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
