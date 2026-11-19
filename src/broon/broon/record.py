#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
record.py ― 주행 기록 CSV [broon]  (2026-11-19 — white1/record.py 를 옮겨 고쳤다. 틀은 gold white1/record.py)
════════════════════════════════════════════════════════════════════════════════
    ros2 run broon record                    ★수동★ — 띄우는 즉시 기록, Ctrl+C 또는 아무 키나 누르면 저장하고 끝
    ros2 launch broon broon.launch.py        (위와 같다 — 런치에서는 키 입력이 없으므로 Ctrl+C 로 끝)
    ros2 launch white1 one.launch.py         ★자율주행★ 기록 노드(mode:=drive)를 따로 띄운다 — 아래

두 방식 — 사용자 지정 2026-11-19
  manual  (기본, 노드 이름 record)       뜨는 즉시 시작해 끝날 때까지 한 파일.
          rec_<연도월일>_<시분초>.csv            예) rec_20261119_142012.csv
  drive   (-p mode:=drive, 노드 이름 record_drive — white1 one.launch.py 가 띄운다)
          driving 의 자율주행 구간(/drive_state 가 DRIVE_HEADING · DRIVE_RUN · DRIVE_DONE)마다 한 파일.
          route_<매핑 파일 이름>_<연도월일>_<시분초>.csv   예) map_421.csv 로 주행 → route_map_421_20261119_142012.csv
          매핑 이름은 prompt 가 /drive_cmd 로 보내는 파일명(못 들으면 driving 의 '경로 선택 · 주행 시작' 이벤트)에서
          줍는다. 끝내 못 들으면 unknown. ★수동 기록이 따로 돌고 있어도 상관없이 따로 기록한다★(파일이 둘 생긴다).
          /joy318 과 driving 의 진단 토픽도 적는다.
  저장 폴더 : -p output_dir:=… > 환경변수 BROON_RECORD_DIR > <ws>/src/broon/ros2bag > ~/broon/ros2bag
              (★공개 리포★ — src/*/ros2bag 은 .gitignore 로 막혀 있다)

════════════════════════════════════════════════════════════════════════════════
 행 — ★한 행 = 한 시점의 전체 상태★ (gold 와 같다)
════════════════════════════════════════════════════════════════════════════════
  SAMPLE_HZ(20 Hz)마다 각 토픽의 그 순간 최신값을 한 줄에 적는다. UTF-8 BOM, 1행 열 이름, 20행마다 flush.
  · 숫자 토픽은 다음 값이 올 때까지 ★값을 유지★(hold). 단 GPS · 우노 출력은 stale_s 동안 안 오면 칸을 비운다
    (끊긴 뒤에도 마지막 자리에 서 있는 것처럼 보이지 않게). ★GPS 를 안 꽂았으면 GPS 칸은 처음부터 비어 있다★.
  · 문자열 · 이벤트 · /joy318 은 ★새로 온 행에만★ 적는다(one-shot). 한 주기에 둘 이상이면 " | " 로 잇는다.
  · 앞 세 열 t_wall(UNIX epoch) · t_rel(기록 시작 기준 초) · vehicle(늘 broon — 궤적 사이트가 hl 차량으로 읽는다).

════════════════════════════════════════════════════════════════════════════════
 열 — 궤적 사이트(domiman.duckdns.org/gps)와 맞춘 이름 (2026-11-19 xeon 세션과 확인)
════════════════════════════════════════════════════════════════════════════════
  ★lat · lon · latitude · longitude 라는 열은 쓰지 않는다★ — 사이트가 매핑 파일로 읽는다. 주행 기록은 fix_lat · fix_lon 으로 알아본다.
  /cmd_vel  (Int32MultiArray)   cmd_pulse · cmd_steer_deg        PC(joy · driving)가 낸 명령. 펄스 부호 = 방향, 조향 − 좌 · + 우
  /enc_pot  (Float64MultiArray) ref_pulse · ref_steer_deg        ★우노가 받았다고 되돌려 준 명령★ — cmd_* 와 나란히 보면
                                                                  우노가 제대로 받는지 안다
                                meas_pulse · steer_measured_deg  실측 — 엔코더 펄스(최근 100 ms A+B 에지 수, 부호 = DIR) ·
                                                                  가변저항 조향각 [도]
  /fix      (NavSatFix)         fix_lat · fix_lon · fix_alt_m · fix_status · fix_cov_xx    전방 GPS (사이트의 기준 궤적)
  /gps_pvt  (Float64MultiArray) gps_lat · gps_lon · gps_quality(0 없음 · 1 단독 · 2 DGNSS · 3 Float · 4 Fixed) · gps_sigma_m(hAcc) ·
                                gps_kmh · gps_course_deg · gps_num_sv · gps_itow_s · gps_rtcm_age_s · gps_hmsl_m   전방 NAV-PVT
  /gps_pvt_rear                 fix_lat2 · fix_lon2 · gps_quality2 · gps_sigma_m2 · gps_kmh2 · … · gps_hmsl_m2
                                ★후방 GPS — 듀얼(broon gps dual:=true)일 때만 값이 있다★. 사이트가 '듀얼 GPS 궤적 비교' 로 겹쳐 그린다
  /gps_status · /arduino_status (String, one-shot)
  /estop    (Int32, 래치)       estop = 1 발동 · 0 해제 (broon/estop — 세 방식 모두. estop 노드가 없으면 빈칸)
  ── drive 방식만 ──
  /joy318 (Int32) joy318 = 1 (조이스틱 SWA ON — 자율주행 정지 신호, one-shot)
  /control_state · /drive_state · /drive_cmd · /drive_event · /drive_status   driving 상태기계
  /driving_debug  white1 driving 의 black 배열 27칸 (DRIVING_DEBUG_COLUMNS — 순서는 white1/driving.py 의 pub_debug 와 1:1)
  /ego_state      ego_x_m · ego_y_m · ego_heading_deg · ego_speed_ms · ego_lat · ego_lon · ego_mode (뒷차축 자세)
  토픽을 늘리려면 표에 TopicSpec 한 줄을 더한다(열 이름은 파일 전체에서 유일해야 한다 — import 때 검사).
"""

import csv
import os
import re
import select
import signal
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Callable, Dict, List, Tuple

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float64MultiArray, Int32, Int32MultiArray, String

from broon.boards import CMD_TOPIC, ENC_POT_COLUMNS, ENC_POT_TOPIC, ESTOP_TOPIC, JOY318_TOPIC
from broon.gpsdev import GPS_PVT_FIELDS, GPS_PVT_REAR_TOPIC, GPS_PVT_TOPIC

SAMPLE_HZ = 20.0
VEHICLE = 'broon'
RECORD_STATES = ('DRIVE_HEADING', 'DRIVE_RUN', 'DRIVE_DONE')    # white1 driving 의 상태 이름
PRE_EVENT_S = 5.0     # 자율 기록 시작 직전 이만큼 안에 온 one-shot 만 첫 줄에 싣는다
PRE_EVENT_MAX = 20

ROUTE_CMD_WORDS = ('STOP', 'MAP_START', 'MAP_STOP', 'DRIVE_START')
ROUTE_EVENT_HINTS = ('경로 선택', '주행 시작')
ROUTE_IN_TEXT = re.compile(r'([^\s\[\]/\\]+\.csv)')
UNKNOWN_ROUTE = 'unknown'
SAFE_NAME = re.compile(r'[^0-9A-Za-z._-]')     # 사이트 이름 규칙 [0-9A-Za-z._-]


# ── 저장 폴더 ─────────────────────────────────────────────────────────────────
def source_root():
    """<ws>/src/broon — 복사설치여도 설치본 위치에서 조상 폴더를 거슬러 올라가 찾는다 (white1 paths.py 와 같은 처리)."""
    here = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    if os.path.isfile(os.path.join(here, "package.xml")) and os.path.basename(here) == 'broon':
        return here
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        cand = os.path.join(d, "src", "broon")
        if os.path.isfile(os.path.join(cand, "package.xml")):
            return cand
        parent = os.path.dirname(d)
        if parent == d:
            return ""
        d = parent


def record_dir(explicit=""):
    if explicit:
        return os.path.expanduser(explicit)
    env = os.environ.get("BROON_RECORD_DIR", "")
    if env:
        return os.path.expanduser(env)
    root = source_root()
    return os.path.join(root, "ros2bag") if root else os.path.expanduser("~/broon/ros2bag")


# ── 추출 ──────────────────────────────────────────────────────────────────────
def _scalar(msg) -> List[Any]:
    return [msg.data]


def _array(n: int) -> Callable[[Any], List[Any]]:
    def _f(msg) -> List[Any]:
        d = list(msg.data)
        return d + [''] * (n - len(d)) if len(d) < n else d[:n]
    return _f


def _enc_pot(msg) -> List[Any]:
    d = _array(4)(msg)
    return [int(v) if v != '' and i < 3 else v for i, v in enumerate(d)]


def _gps_pvt(msg) -> List[Any]:
    d = _array(GPS_PVT_FIELDS)(msg)
    if d[4] != '':
        d[4] = d[4] * 3.6       # 지면속도 m/s → km/h (사이트의 gps_kmh)
    return d


def _navsat(msg) -> List[Any]:
    return [msg.latitude, msg.longitude, msg.altitude, msg.status.status, msg.position_covariance[0]]


@dataclass(frozen=True)
class TopicSpec:
    topic: str
    msg_type: type
    columns: Tuple[str, ...]
    extract: Callable[[Any], List[Any]]
    hold: bool = True
    stale_s: float = 0.0        # hold 값을 이만큼 새로 안 받으면 비운다 (0 = 계속 유지)
    latched: bool = False       # 래치 토픽 — 늦게 떠도 마지막 값을 받는다


GPS_PVT_COLUMNS = ('gps_lat', 'gps_lon', 'gps_quality', 'gps_sigma_m', 'gps_kmh',
                   'gps_course_deg', 'gps_num_sv', 'gps_itow_s', 'gps_rtcm_age_s', 'gps_hmsl_m')
GPS_REAR_COLUMNS = ('fix_lat2', 'fix_lon2') + tuple(c + '2' for c in GPS_PVT_COLUMNS[2:])
assert len(GPS_PVT_COLUMNS) == GPS_PVT_FIELDS == len(GPS_REAR_COLUMNS)

# white1/driving.py run_follow 끝 pub_debug 배열(black 필드 순서 0~26)과 1:1
DRIVING_DEBUG_COLUMNS: Tuple[str, ...] = (
    'wp_idx', 'wp_total', 'goal_dist_m', 'cte_m', 'cte_raw_m',
    'cte_integral', 'target_dist_m', 'speed_ratio', 'raw_steer_deg', 'steer_ctrl_deg',
    'steer_step_deg', 'v_target_ms', 'v_target_out_ms', 'v_meas_ms', 'near_demand_deg',
    'far_dist_m', 'far_demand_deg', 'p_term_deg', 'cte_i_term_deg', 'd_term_deg',
    'd_term_lpf', 'loop_dt_s', 'dbg_reserved', 'near_peak_deg', 'approach_active',
    'speed_reason', 'far_peak_deg',
)
EGO_COLUMNS = ('ego_x_m', 'ego_y_m', 'ego_heading_deg', 'ego_speed_ms', 'ego_lat', 'ego_lon', 'ego_mode')

BASE_TOPICS: Tuple[TopicSpec, ...] = (
    TopicSpec(CMD_TOPIC, Int32MultiArray, ('cmd_pulse', 'cmd_steer_deg'), _array(2)),
    TopicSpec(ENC_POT_TOPIC, Float64MultiArray, ENC_POT_COLUMNS, _enc_pot, stale_s=0.5),
    TopicSpec('/fix', NavSatFix, ('fix_lat', 'fix_lon', 'fix_alt_m', 'fix_status', 'fix_cov_xx'), _navsat,
              stale_s=1.0),
    TopicSpec(GPS_PVT_TOPIC, Float64MultiArray, GPS_PVT_COLUMNS, _gps_pvt, stale_s=1.0),
    TopicSpec(GPS_PVT_REAR_TOPIC, Float64MultiArray, GPS_REAR_COLUMNS, _gps_pvt, stale_s=1.0),
    TopicSpec('/gps_status', String, ('gps_status',), _scalar, hold=False),
    TopicSpec('/arduino_status', String, ('arduino_status',), _scalar, hold=False),
    TopicSpec(ESTOP_TOPIC, Int32, ('estop',), _scalar, latched=True),
)
DRIVE_TOPICS: Tuple[TopicSpec, ...] = (
    TopicSpec(JOY318_TOPIC, Int32, ('joy318',), _scalar, hold=False),
    TopicSpec('/control_state', Bool, ('control_state',), _scalar),
    TopicSpec('/drive_state', String, ('drive_state',), _scalar),
    TopicSpec('/drive_cmd', String, ('drive_cmd',), _scalar, hold=False),
    TopicSpec('/drive_event', String, ('drive_event',), _scalar, hold=False),
    TopicSpec('/drive_status', String, ('drive_status',), _scalar, hold=False),
    TopicSpec('/driving_debug', Float64MultiArray, DRIVING_DEBUG_COLUMNS, _array(len(DRIVING_DEBUG_COLUMNS))),
    TopicSpec('/ego_state', Float64MultiArray, EGO_COLUMNS, _array(len(EGO_COLUMNS))),
)
COMMON_COLUMNS = ('t_wall', 't_rel', 'vehicle')


def columns_of(topics):
    cols = COMMON_COLUMNS + tuple(c for s in topics for c in s.columns)
    dups = sorted({c for c in cols if cols.count(c) > 1})
    if dups:
        raise RuntimeError(f"record.py 열 이름 중복: {dups}")
    bad = {'lat', 'lon', 'latitude', 'longitude'} & {c.lower() for c in cols}
    if bad:
        raise RuntimeError(f"record.py — {bad} 열은 사이트가 매핑으로 읽는다. 이름을 바꿀 것")
    return cols


columns_of(BASE_TOPICS + DRIVE_TOPICS)      # import 때 검사


class RecordNode(Node):
    FLUSH_EVERY_ROWS = 20
    STATUS_PERIOD_S = 10.0

    def __init__(self, mode, **kwargs):
        super().__init__('record_drive' if mode == 'drive' else 'record', **kwargs)
        self.declare_parameter('output_dir', '')
        self.declare_parameter('sample_hz', SAMPLE_HZ)
        self.mode = mode
        self.out_root = record_dir(str(self.get_parameter('output_dir').value or ''))
        self.sample_hz = max(1.0, float(self.get_parameter('sample_hz').value))
        self.topics = BASE_TOPICS + (DRIVE_TOPICS if mode == 'drive' else ())
        self.columns = columns_of(self.topics)
        self.stale = {c: s.stale_s for s in self.topics if s.hold and s.stale_s > 0 for c in s.columns}

        self.drive_state = 'IDLE'
        self.route_name = ''
        self._hold: Dict[str, Any] = {}
        self._hold_t: Dict[str, float] = {}
        self._pending: Dict[str, List[Tuple[float, str]]] = {}
        self._rx: Dict[str, int] = {s.topic: 0 for s in self.topics}
        self.recording = False
        self.csv_path = ''
        self.session_t0 = 0.0
        self._fp = self._writer = None
        self._rows = 0
        self._last_status_log = 0.0
        self.sessions = []           # [(경로, 행 수, 초)]

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for spec in self.topics:
            self.create_subscription(spec.msg_type, spec.topic, (lambda msg, s=spec: self._on_msg(s, msg)),
                                     latched if spec.latched else 20)
        self.create_timer(1.0 / self.sample_hz, self._tick)
        self.create_timer(1.0, self._status_tick)
        self.get_logger().info(
            f"📼 record [{'자율주행 — ' + ', '.join(RECORD_STATES) + ' 구간마다' if mode == 'drive' else '수동 — 지금부터 끝날 때까지'}]"
            f" → {self.out_root} | 토픽 {len(self.topics)}개 → {len(self.columns)}열, {self.sample_hz:.0f} Hz")
        if mode != 'drive':
            self._start_session(f"rec_{datetime.now().strftime('%Y%m%d_%H%M%S')}")

    # ── 수신 ───────────────────────────────────────────────────────────────────
    def _on_msg(self, spec: TopicSpec, msg):
        if self.mode == 'drive':
            if spec.topic == '/drive_state':
                self.drive_state = str(msg.data)
            elif spec.topic == '/drive_cmd':
                self._note_route_cmd(str(msg.data))
            elif spec.topic == '/drive_event':
                self._note_route_event(str(msg.data))
        self._stash(spec, msg)
        if self.mode != 'drive':
            return
        on = self.drive_state in RECORD_STATES
        if on and not self.recording:
            base = os.path.splitext(os.path.basename(self.route_name))[0] or UNKNOWN_ROUTE
            if not self.route_name:
                self.get_logger().warning(f"매핑 이름을 못 들었다(record 를 주행 도중에 띄웠는가?) — {UNKNOWN_ROUTE}")
            self._start_session(f"route_{SAFE_NAME.sub('_', base)}_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
        elif self.recording and not on:
            self._write_row()          # 끝난 사유가 담긴 마지막 줄
            self._stop_session()

    def _note_route_cmd(self, text):
        name = text.strip()
        if name and name.upper() not in ROUTE_CMD_WORDS and name.lower().endswith('.csv'):
            self.route_name = os.path.basename(name)

    def _note_route_event(self, text):
        if any(h in text for h in ROUTE_EVENT_HINTS):
            m = ROUTE_IN_TEXT.search(text)
            if m:
                self.route_name = m.group(1)

    def _stash(self, spec: TopicSpec, msg):
        try:
            values = spec.extract(msg)
        except Exception as e:      # noqa: BLE001 — 한 토픽이 이상해도 기록은 이어 간다
            self.get_logger().warning(f"{spec.topic} 추출 실패: {e}", throttle_duration_sec=5.0)
            return
        self._rx[spec.topic] += 1
        now = time.time()
        if spec.hold:
            for col, val in zip(spec.columns, values):
                self._hold[col] = val
                self._hold_t[col] = now
        else:
            for col, val in zip(spec.columns, values):
                q = self._pending.setdefault(col, [])
                q.append((now, str(val)))
                if not self.recording and len(q) > PRE_EVENT_MAX:
                    del q[0]

    # ── 세션 ───────────────────────────────────────────────────────────────────
    def _start_session(self, stem):
        os.makedirs(self.out_root, exist_ok=True)
        self.csv_path = os.path.join(self.out_root, f"{stem}.csv")
        n = 2
        while os.path.exists(self.csv_path):           # 같은 초에 두 번 시작한 경우만
            self.csv_path = os.path.join(self.out_root, f"{stem}_{n}.csv")
            n += 1
        self._fp = open(self.csv_path, 'w', newline='', encoding='utf-8-sig')
        self._writer = csv.writer(self._fp)
        self._writer.writerow(self.columns)
        self.session_t0 = time.time()
        self._rows = 0
        self._rx = {s.topic: 0 for s in self.topics}
        self._last_status_log = self.session_t0
        cut = self.session_t0 - PRE_EVENT_S
        self._pending = {c: kept for c, q in self._pending.items() if (kept := [e for e in q if e[0] >= cut])}
        self.recording = True
        self.get_logger().info(f"🔴 기록 시작 → {self.csv_path}")

    def _tick(self):
        if self.recording:
            self._write_row()

    def _write_row(self):
        if self._writer is None:
            return
        now = time.time()
        row = [f"{now:.6f}", f"{now - self.session_t0:.3f}", VEHICLE]
        for col in self.columns[len(COMMON_COLUMNS):]:
            if col in self._pending:
                row.append(" | ".join(text for _, text in self._pending.pop(col)))
                continue
            lim = self.stale.get(col, 0.0)
            if lim and now - self._hold_t.get(col, 0.0) > lim:
                row.append('')
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
        self.sessions.append((self.csv_path, self._rows, dur))
        self.get_logger().info(f"⏹️ 기록 저장 — {dur:.1f}초, {self._rows}행 → {self.csv_path}")

    def _status_tick(self):
        now = time.time()
        if not self.recording or now - self._last_status_log < self.STATUS_PERIOD_S:
            return
        self._last_status_log = now
        silent = [t for t, n in self._rx.items() if n == 0]
        self.get_logger().info(
            f"📼 기록 중 {now - self.session_t0:.0f}초 | {self._rows}행"
            + (f" | 안 오는 토픽: {', '.join(silent)}" if silent else ""))

    def finish(self):
        if self.recording:
            self._write_row()
            self._stop_session()


class KeyWatcher:
    """터미널에서 아무 키나 누르면 stop 을 켠다. 표준입력이 터미널이 아니면(런치) 아무것도 안 한다."""

    def __init__(self, stop):
        self.stop = stop
        self.saved = None
        if not sys.stdin.isatty():
            return
        try:
            fd = sys.stdin.fileno()
            self.saved = termios.tcgetattr(fd)
            tty.setcbreak(fd)                # 줄 단위가 아니라 한 글자로 · 화면에 찍지 않는다
        except (termios.error, OSError, ValueError):
            self.saved = None
            return
        threading.Thread(target=self._run, args=(fd,), daemon=True).start()

    def _run(self, fd):
        while not self.stop.is_set():
            r, _, _ = select.select([fd], [], [], 0.2)
            if r:
                try:
                    os.read(fd, 64)
                except OSError:
                    pass
                self.stop.set()

    def restore(self):
        if self.saved is not None:
            try:
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.saved)
            except (termios.error, OSError, ValueError):
                pass
            self.saved = None


def main(args=None):
    rclpy.init(args=args)
    # mode 는 노드 이름을 정하므로 노드를 만들기 전에 읽는다 (--ros-args -p mode:=drive)
    probe = rclpy.create_node('_record_mode_probe')
    probe.declare_parameter('mode', 'manual')
    mode = 'drive' if str(probe.get_parameter('mode').value).strip().lower() == 'drive' else 'manual'
    probe.destroy_node()
    node = RecordNode(mode)
    stop = threading.Event()
    keys = KeyWatcher(stop) if mode == 'manual' else None
    if keys is not None and keys.saved is not None:
        node.get_logger().info("⌨️  아무 키나 누르면(또는 Ctrl+C) 저장하고 끝낸다")
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not stop.is_set():
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # Ctrl-C 두 번째(launch 가 다시 보낸다)가 파일 닫기를 끊지 않게
        if keys is not None:
            keys.restore()
        node.finish()
        for path, rows, dur in node.sessions:
            print(f"💾 {path}  ({rows}행, {dur:.1f}초)", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
