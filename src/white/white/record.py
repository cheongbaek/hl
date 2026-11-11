#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record.py ― 주행 기록 [gold white1/record.py 계승]
════════════════════════════════════════════════════════════════════════════════
    ros2 run white record            (one_launch.py 가 함께 띄운다 — use_record:=false 로 끈다)
    ros2 run white record --force    driving 없이 지금부터 계속 기록 (리모컨 수동 계측 등)

자율주행 구간에 주고받는 토픽을 ★CSV 파일 하나★ 에 담는다. 구독만 하는 노드라 제어에 끼어들지 않는다.
틀(기록 구간 · 파일 이름 · 스냅샷 · hold/one-shot · 열 이름 중복 검사)은 gold white1 의 것 그대로이고,
토픽 표(RECORD_TOPICS)만 white 에 있는 토픽으로 바꿨다. gold 에만 있던 것(브레이크 · 라이다 · 신호등 ·
B보드 · .cones.csv)은 뺐다.

════════════════════════════════════════════════════════════════════════════════
 기록 구간 — driving 의 상태를 그대로 따른다
════════════════════════════════════════════════════════════════════════════════
  /drive_state 가 DRIVE_HEADING · DRIVE_RUN · DRIVE_DONE 일 때 기록한다 = prompt 에서 주행을 시작한
  순간부터 서서 IDLE(메뉴)로 돌아올 때까지. 헤딩 초기화 구간도 넣는다 — 출발 헤딩이 틀렸을 때 원인을
  찾으려면 그 구간의 GPS 가 남아 있어야 한다. 매핑(MAP)은 기록하지 않는다(그쪽 산출물은 경로 CSV).
  --force (= -p force_record:=true) 면 상태와 무관하게 노드가 뜨는 즉시 시작해 내릴 때까지 적는다.

════════════════════════════════════════════════════════════════════════════════
 출력 — 한 주행에 ★파일 하나★
════════════════════════════════════════════════════════════════════════════════
  <ws>/src/white/ros2bag/<주행한 경로 CSV 이름>-<날짜>_<시각>.csv     (paths.record_dir)
      gps_data/route_20261109_203222.csv 로 주행 → ros2bag/route_20261109_203222-20261109_214501.csv
  경로 이름은 prompt 가 /drive_cmd 로 보내는 파일명(못 들었으면 driving 의 '경로 선택 · 주행 시작'
  이벤트)에서 줍는다. 끝내 못 들으면 unknown, --force 면 manual 이 된다.
  1행 열 이름 / 2행~ 데이터. UTF-8 BOM(엑셀이 한글을 깨뜨리지 않게) — 궤적 사이트는 BOM 을 떼고 읽는다.

  ★한 행 = 한 시점의 차량 전체 상태★ — SAMPLE_HZ(20 Hz = driving 제어 주기)마다 각 토픽의 그 순간
  최신값을 한 줄에 나란히 적는다. 더 빠른 토픽(IMU · 아두이노)은 주기 안의 마지막 값만 남는다.
  · 숫자 토픽은 다음 값이 올 때까지 ★값을 유지★ 한다(hold).
  · 이벤트 · 명령 · 상태 문자열은 ★새로 온 행에만★ 적고 비운다(one-shot). 한 주기에 둘 이상 오면
    " | " 로 이어 붙인다. 콤마가 든 문자열은 csv 모듈이 큰따옴표로 감싸므로 열이 밀리지 않는다.
    기록 전에 온 것은 ★시작 직전 PRE_EVENT_S 초 안의 것만★ 첫 줄에 싣는다(경로 선택 · 주행 시작) —
    gold 는 대기 중 이벤트를 전부 첫 줄에 붙였다(white 는 대기 중에도 GPS 품질 이벤트가 나온다).
  · 앞 두 열은 t_wall(UNIX epoch) · t_rel(기록 시작 기준 경과 초). 20행마다 flush.

  궤적 사이트는 fix_lat · fix_lon 열로 '주행 기록' 을 알아보고, gps_sigma_m · fix_cov_xx · gps_quality
  로 구간 품질을, cte_m · target_dist_m · cte_integral · cte_i_term_deg · gps_kmh 등을 그래프로 그린다.

════════════════════════════════════════════════════════════════════════════════
 열 — gold 와 이름이 같으면 뜻도 같다. white 에서 뜻이 다른 값은 ★다른 이름★ 을 썼다
════════════════════════════════════════════════════════════════════════════════
  · /cmd_vel_raw : cmd_speed_ms (목표 속도 m/s — gold 의 cmd_pulse 가 아니다),
                   cmd_steer_deg (발행 조향각, black v6 눈금 ★+ 좌 / − 우★ — gold 의 pot 규약과 부호가 반대)
  · /driving_debug : driving.py 의 black 배열 27칸(DRIVING_DEBUG_COLUMNS, 순서 1:1). cte_m 은 +가 경로가
                   왼쪽(gold 와 같다), target_dist_m 은 순수추종 선행거리(LFD).
  · /gps_pvt : gps.py 의 GPS_PVT 규약 10칸. gps_quality 0 없음 · 1 단독 · 2 DGNSS · 3 Float · 4 Fixed,
               gps_sigma_m = hAcc, gps_kmh = 지면속도 × 3.6.
  · /encoder : encoder_ticks = 아두이노 PID 한 주기 동안의 틱(gold 의 encoder_sum 과 다르다),
    /wheel_speed : wheel_speed_ms.
  토픽을 늘리려면 RECORD_TOPICS 에 TopicSpec 한 줄을 더한다(열 이름은 파일 전체에서 유일해야 한다).
"""

import csv
import os
import re
import signal
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Callable, Dict, List, Tuple

import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, Float32, Float64MultiArray, Int32, String

from white import paths
from white.gps import GPS_PVT_FIELDS, GPS_PVT_TOPIC

# 이 상태들에서만 기록한다 (driving.py 의 상태 이름과 같아야 한다)
RECORD_STATES = ('DRIVE_HEADING', 'DRIVE_RUN', 'DRIVE_DONE')
SAMPLE_HZ = 20.0
PRE_EVENT_S = 5.0     # 기록 시작 직전 이만큼 안에 온 one-shot 문자열만 첫 줄에 싣는다 (경로 선택 · 주행 시작)
PRE_EVENT_MAX = 20    # 기록하지 않는 동안 열마다 들고 있는 one-shot 상한

# ── 파일명 앞부분(= 주행한 경로)을 알아내는 두 길 (gold 와 같다) ──
ROUTE_CMD_WORDS = ('STOP', 'MAP_START', 'MAP_STOP', 'DRIVE_START')
ROUTE_EVENT_HINTS = ('경로 선택', '주행 시작')
ROUTE_IN_TEXT = re.compile(r'([^\s\[\]/\\]+\.csv)')
UNKNOWN_ROUTE = 'unknown'
MANUAL_ROUTE = 'manual'


# ══════════════════════════════════════════════════════════════════════════════
#  추출 헬퍼
# ══════════════════════════════════════════════════════════════════════════════
def _scalar(msg) -> List[Any]:
    return [msg.data]


def _twist(msg) -> List[Any]:
    """white 는 linear.x(목표 m/s)와 angular.z(조향각)만 쓴다."""
    return [msg.linear.x, msg.angular.z]


def _array(n: int) -> Callable[[Any], List[Any]]:
    """MultiArray → 고정 길이 n. 발행측이 늘려도 열이 밀리지 않게 강제한다."""
    def _f(msg) -> List[Any]:
        d = list(msg.data)
        if len(d) < n:
            return d + [''] * (n - len(d))
        return d[:n]
    return _f


def _gps_pvt(msg) -> List[Any]:
    """GPS_PVT 규약 → 열. 지면속도만 km/h 로 바꾼다(사이트의 gps_kmh)."""
    d = _array(GPS_PVT_FIELDS)(msg)
    if d[4] != '':
        d[4] = d[4] * 3.6
    return d


def _navsat(msg) -> List[Any]:
    return [msg.latitude, msg.longitude, msg.altitude,
            msg.status.status, msg.position_covariance[0]]


def _imu(msg) -> List[Any]:
    q, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
    return [q.x, q.y, q.z, q.w, w.x, w.y, w.z, a.x, a.y, a.z]


# ══════════════════════════════════════════════════════════════════════════════
#  구독 토픽 표
# ══════════════════════════════════════════════════════════════════════════════
@dataclass(frozen=True)
class TopicSpec:
    topic: str
    msg_type: type
    columns: Tuple[str, ...]
    extract: Callable[[Any], List[Any]]
    hold: bool = True
    note: str = ""


#  ★/driving_debug 열 이름 — driving.py run_follow 끝 pub_debug 배열(black 필드 순서 0~26)과 1:1★
#  열을 더할 때는 driving.py 의 배열 끝에 붙이고 여기 끝에 이름을 붙인다.
DRIVING_DEBUG_COLUMNS: Tuple[str, ...] = (
    'wp_idx', 'wp_total', 'goal_dist_m', 'cte_m', 'cte_raw_m',
    'cte_integral', 'target_dist_m', 'speed_ratio', 'raw_steer_deg', 'steer_ctrl_deg',
    'steer_step_deg', 'v_target_ms', 'v_target_out_ms', 'v_meas_ms', 'near_demand_deg',
    'far_dist_m', 'far_demand_deg', 'p_term_deg', 'cte_i_term_deg', 'd_term_deg',
    'd_term_lpf', 'loop_dt_s', 'dbg_reserved', 'near_peak_deg', 'approach_active',
    'speed_reason', 'far_peak_deg',
)

#  /gps_pvt 열 이름 — gps.py 의 GPS_PVT 규약 순서
GPS_PVT_COLUMNS: Tuple[str, ...] = (
    'gps_lat', 'gps_lon', 'gps_quality', 'gps_sigma_m', 'gps_kmh',
    'gps_course_deg', 'gps_num_sv', 'gps_itow_s', 'gps_rtcm_age_s', 'gps_hmsl_m',
)
assert len(GPS_PVT_COLUMNS) == GPS_PVT_FIELDS, "record.GPS_PVT_COLUMNS 와 gps.GPS_PVT_FIELDS 가 다르다"

RECORD_TOPICS: Tuple[TopicSpec, ...] = (
    # ── 제어 명령 ──
    TopicSpec('/cmd_vel_raw', Twist, ('cmd_speed_ms', 'cmd_steer_deg'), _twist,
              note='아두이노로 나가는 최종 명령 — 목표 속도 m/s, 조향각(v6 눈금, + 좌 / − 우)'),
    TopicSpec('/control_state', Bool, ('control_state',), _scalar,
              note='True = 자율주행이 모터를 쥔다(arduino 가 C 명령 송신)'),

    # ── 추종 진단 ★주행 성패를 판정하는 열들★ ──
    TopicSpec('/driving_debug', Float64MultiArray, DRIVING_DEBUG_COLUMNS,
              _array(len(DRIVING_DEBUG_COLUMNS)),
              note='★cte_m 이 핵심★ 경로이탈(+ 경로가 왼쪽). DRIVE_RUN 에서만 나온다 — 헤딩 초기화 · '
                   '정지 구간은 직전 값이 유지된다(drive_state 로 가를 것). speed_reason 은 driving.py '
                   'SPD_REASON_CODE (0 MAX · 1 CURVE · 2 BRAKE · 3 CTE_RECOVERY · 4 STEER · 7 RAMP · '
                   '8 SPEED_PI · 9 ACCEL_LIMIT · 10 DECEL_LIMIT · 11 APPROACH · 12 OMEGA). '
                   '순환 경로면 goal_dist_m 은 99'),
    TopicSpec('/drive_state', String, ('drive_state',), _scalar,
              note='driving 상태기계 ★기록 구간을 정하는 신호★'),
    TopicSpec('/drive_cmd', String, ('drive_cmd',), _scalar, hold=False,
              note='prompt 하달 (경로 선택 / DRIVE_START / STOP)'),
    TopicSpec('/drive_event', String, ('drive_event',), _scalar, hold=False,
              note='헤딩 확정 · 도착 · 이탈 · RTK 대기 · 바퀴 수 등 이벤트'),
    TopicSpec('/drive_status', String, ('drive_status',), _scalar, hold=False,
              note='driving 1 Hz 한 줄 상태'),

    # ── 구동 (아두이노) ──
    TopicSpec('/encoder', Int32, ('encoder_ticks',), _scalar,
              note='아두이노 PID 한 주기 동안의 엔코더 틱(d_val)'),
    TopicSpec('/wheel_speed', Float32, ('wheel_speed_ms',), _scalar,
              note='위를 m/s 로 (arduino.py 환산 — 300틱/회전 미검증)'),
    TopicSpec('/arduino_status', String, ('arduino_status',), _scalar, hold=False,
              note='arduino 1 Hz 한 줄 상태 — 펌웨어 형식 · 주기 · 리모컨 표시(white_fw)'),

    # ── GPS ──
    TopicSpec(GPS_PVT_TOPIC, Float64MultiArray, GPS_PVT_COLUMNS, _gps_pvt,
              note='gps.py 가 NAV-PVT 에서 뽑은 값(driving 이 쓰는 원값). gps_quality 3 Float · 4 Fixed, '
                   'gps_sigma_m = hAcc, gps_course_deg 는 동 0° 반시계 +(0.3 m/s 미만 NaN), '
                   'gps_rtcm_age_s = 마지막 보정 수신 후'),
    TopicSpec('/fix', NavSatFix, ('fix_lat', 'fix_lon', 'fix_alt_m', 'fix_status', 'fix_cov_xx'),
              _navsat, note='GPS 원시 (status: −1 없음 · 0 단독 · 1 DGNSS · 2 RTK Float 이상)'),
    TopicSpec('/gps_status', String, ('gps_status',), _scalar, hold=False,
              note='gps 1 Hz 한 줄 상태 — RTK · NTRIP · IMU'),

    # ── IMU ──
    TopicSpec('/imu', Imu,
              ('imu_quat_x', 'imu_quat_y', 'imu_quat_z', 'imu_quat_w',
               'imu_gyro_x', 'imu_gyro_y', 'imu_gyro_z',
               'imu_acc_x', 'imu_acc_y', 'imu_acc_z'),
              _imu, note='iAHRS 원시 6축 + 자세'),
)

COMMON_COLUMNS = ('t_wall', 't_rel')
ALL_COLUMNS: Tuple[str, ...] = COMMON_COLUMNS + tuple(
    c for spec in RECORD_TOPICS for c in spec.columns)

# ★열 이름 중복 검사 — import 시점에 터뜨린다 (gold [2026-08-18])★
#  CSV 헤더에 같은 이름이 두 번 들어가면 csv.DictReader 가 뒤엣것만 남겨 앞 열이 조용히 사라진다.
_dups = sorted({c for c in ALL_COLUMNS if ALL_COLUMNS.count(c) > 1})
if _dups:
    raise RuntimeError(f"record.py 열 이름 중복: {_dups} — RECORD_TOPICS 에서 이름을 바꿀 것")
del _dups


# ══════════════════════════════════════════════════════════════════════════════
#  노드
# ══════════════════════════════════════════════════════════════════════════════
class RecordNode(Node):

    STATUS_PERIOD_S = 10.0
    FLUSH_EVERY_ROWS = 20

    def __init__(self, **kwargs):
        super().__init__('record_node', **kwargs)
        self.declare_parameter('output_dir', '')
        self.declare_parameter('sample_hz', SAMPLE_HZ)
        self.declare_parameter('force_record', False)

        self.out_root = paths.record_dir(str(self.get_parameter('output_dir').value or ''))
        self.sample_hz = max(1.0, float(self.get_parameter('sample_hz').value))
        self.force_record = bool(self.get_parameter('force_record').value)

        self.drive_state = 'IDLE'
        self.route_name = ''          # 주행 중인 경로 CSV 이름 — 기록 파일명 앞부분

        self._hold: Dict[str, Any] = {}
        self._pending: Dict[str, List[Tuple[float, str]]] = {}   # 열 → [(받은 시각, 문자열)]
        self._rx: Dict[str, int] = {s.topic: 0 for s in RECORD_TOPICS}
        self._warned: set = set()

        self.recording = False
        self.csv_path = ''
        self.session_t0 = 0.0
        self._fp = None
        self._writer = None
        self._rows = 0
        self._last_status_log = 0.0

        for spec in RECORD_TOPICS:
            self.create_subscription(spec.msg_type, spec.topic,
                                     (lambda msg, s=spec: self._on_msg(s, msg)), 10)
        self.create_timer(1.0 / self.sample_hz, self._tick)
        self.create_timer(1.0, self._status_tick)

        self.get_logger().info(
            f"📼 record 대기 — 기록 위치: {self.out_root} | 토픽 {len(RECORD_TOPICS)}개 → "
            f"CSV {len(ALL_COLUMNS)}열, {self.sample_hz:.0f}Hz 스냅샷 | 기록 구간: "
            + ('★force_record — 지금부터 계속★' if self.force_record else ', '.join(RECORD_STATES)))
        if self.force_record:
            self.route_name = MANUAL_ROUTE
            self._start_session()

    # ── 수신 ───────────────────────────────────────────────────────────────────
    def _on_msg(self, spec: TopicSpec, msg):
        was = self.recording
        if spec.topic == '/drive_state':
            self.drive_state = str(msg.data)
        elif spec.topic == '/drive_cmd':
            self._note_route_cmd(str(msg.data))
        elif spec.topic == '/drive_event':
            self._note_route_event(str(msg.data))
        self._stash(spec, msg)

        now_on = self.force_record or self.drive_state in RECORD_STATES
        if now_on and not was:
            self._start_session()
        elif was and not now_on:
            self._write_row()          # 종료 사유가 담긴 마지막 한 줄
            self._stop_session()

    def _note_route_cmd(self, text: str):
        """prompt 가 /drive_cmd 로 보낸 것이 경로 파일명이면 붙든다."""
        name = text.strip()
        if not name or name.upper() in ROUTE_CMD_WORDS:
            return
        if name.lower().endswith('.csv'):
            self.route_name = os.path.basename(name)

    def _note_route_event(self, text: str):
        """driving 의 '경로 선택 / 주행 시작' 이벤트에서 이름을 줍는다(보조)."""
        if not any(h in text for h in ROUTE_EVENT_HINTS):
            return
        m = ROUTE_IN_TEXT.search(text)
        if m:
            self.route_name = m.group(1)

    def _stash(self, spec: TopicSpec, msg):
        try:
            values = spec.extract(msg)
        except Exception as e:      # noqa: BLE001 — 한 토픽이 이상해도 기록은 이어 간다
            self.get_logger().warning(f"{spec.topic} 추출 실패: {e}")
            return
        self._rx[spec.topic] += 1
        # 메시지 ★원래★ 길이로 본다 — _array(n) 는 늘 n 개를 돌려주므로 발행측이 열을 늘린 것이 안 보인다
        raw = getattr(msg, 'data', None)
        n_raw = (len(raw) if raw is not None and hasattr(raw, '__len__')
                 and not isinstance(raw, (str, bytes)) else len(values))
        if n_raw != len(spec.columns) and spec.topic not in self._warned:
            self._warned.add(spec.topic)
            self.get_logger().warning(
                f"{spec.topic} 필드 수 {n_raw} 가 표({len(spec.columns)}열)와 다르다 — "
                f"record.py 의 columns 와 발행측 배열을 함께 확인할 것")
        if spec.hold:
            for col, val in zip(spec.columns, values):
                self._hold[col] = val
        else:
            now = time.time()
            for col, val in zip(spec.columns, values):
                q = self._pending.setdefault(col, [])
                q.append((now, str(val)))
                if not self.recording and len(q) > PRE_EVENT_MAX:
                    del q[0]

    # ── 세션 ───────────────────────────────────────────────────────────────────
    def _start_session(self):
        os.makedirs(self.out_root, exist_ok=True)
        base = os.path.splitext(os.path.basename(self.route_name))[0] or UNKNOWN_ROUTE
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.out_root, f"{base}-{stamp}.csv")
        n = 2
        while os.path.exists(self.csv_path):   # 같은 초에 두 번 시작한 경우만
            self.csv_path = os.path.join(self.out_root, f"{base}-{stamp}_{n}.csv")
            n += 1
        self._fp = open(self.csv_path, 'w', newline='', encoding='utf-8-sig')
        self._writer = csv.writer(self._fp)
        self._writer.writerow(ALL_COLUMNS)
        self.session_t0 = time.time()
        self._rows = 0
        self._rx = {s.topic: 0 for s in RECORD_TOPICS}
        self._last_status_log = self.session_t0      # 첫 상태 줄은 STATUS_PERIOD_S 뒤에
        cut = self.session_t0 - PRE_EVENT_S
        self._pending = {c: kept for c, q in self._pending.items() if (kept := [e for e in q if e[0] >= cut])}
        self.recording = True
        if not self.route_name:
            self.get_logger().warning(
                f"경로 이름을 못 들었다(record 를 주행 도중에 띄웠는가?) — 파일명 앞부분이 {UNKNOWN_ROUTE}")
        self.get_logger().info(f"🔴 기록 시작 → {self.csv_path}")

    def _tick(self):
        if self.recording:
            self._write_row()

    def _write_row(self):
        if self._writer is None:
            return
        now = time.time()
        row = [f"{now:.6f}", f"{now - self.session_t0:.3f}"]
        for col in ALL_COLUMNS[len(COMMON_COLUMNS):]:
            if col in self._pending:
                row.append(" | ".join(text for _, text in self._pending.pop(col)))
            else:
                row.append(self._hold.get(col, ''))
        self._writer.writerow(row)
        self._rows += 1
        if self._rows % self.FLUSH_EVERY_ROWS == 0:
            try:
                self._fp.flush()
            except Exception:      # noqa: BLE001
                pass

    def _stop_session(self):
        if not self.recording:
            return
        self.recording = False
        dur = time.time() - self.session_t0
        try:
            self._fp.close()
        except Exception:          # noqa: BLE001
            pass
        self._fp = self._writer = None
        self.get_logger().info(f"⏹️ 기록 종료 ({self.drive_state}) — {dur:.1f}초, {self._rows}행 → {self.csv_path}")

    def _status_tick(self):
        now = time.time()
        if not self.recording or now - self._last_status_log < self.STATUS_PERIOD_S:
            return
        self._last_status_log = now
        live = sum(1 for n in self._rx.values() if n > 0)
        silent = [t for t, n in self._rx.items() if n == 0]
        self.get_logger().info(
            f"📼 기록 중 {now - self.session_t0:.0f}초 | {self._rows}행 | 수신 토픽 {live}/{len(RECORD_TOPICS)}"
            + (f" (안 옴: {', '.join(silent)})" if silent else ""))

    def destroy_node(self):
        if self.recording:
            self._write_row()
            self._stop_session()
        super().destroy_node()


def main(args=None):
    # `ros2 run white record --force` — rclpy.init 은 모르는 인자를 싫어하므로 먼저 꺼낸다
    argv = sys.argv[1:] if args is None else list(args)
    flag_force = '--force' in argv
    argv = [a for a in argv if a != '--force']
    if args is None:
        sys.argv = [sys.argv[0]] + argv
    else:
        args = argv
    rclpy.init(args=args)
    from rclpy.parameter import Parameter
    overrides = [Parameter('force_record', Parameter.Type.BOOL, True)] if flag_force else []
    node = RecordNode(parameter_overrides=overrides)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # Ctrl-C 두 번째(launch 가 다시 보낸다)가 파일 닫기를 끊지 않게
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
