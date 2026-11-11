#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gps.py ― GPS(ZED-F9P + NTRIP RTK) · IMU(iAHRS) 포트 담당
════════════════════════════════════════════════════════════════════════════════
    ros2 run white gps              (one_launch.py 가 띄운다)

GPS : hl_ws/tool/gps_mapper.py 의 수신부를 그대로 옮겼다 (GUI·기록·기지국 해독은 뺐다).
  · u-blox VID(0x1546) 포트만 찾아 연다. USB 가 빠지면 다시 찾는다.
  · 접속할 때마다 수신기 ★RAM 에만★ 설정을 쓴다 — NAV-PVT 10 Hz, USB 로는 GGA · RMC 만,
    UART1 · I2C 의 NMEA 는 끈다(공장 설정은 출력 버퍼가 넘쳐 RTCM 을 띄엄띄엄 써서 Fix 가 안 된다).
  · NGII VRS(VRS-RTCM34) 보정을 ★PC 인터넷으로★ 받아 같은 포트로 수신기에 넣는다.
    캐스터에는 수신기 GGA 를 10 초마다 올린다. 접속 정보는 hl_ws/tool/info.md · tool/gps_mapper.py 와 같다.
  · 위치 · 품질 · 정확도 · 속도를 ★NAV-PVT 한 메시지에서★ 뽑는다 — carrSoln 으로 RTK Fixed /
    Float 이 바로 갈린다(nmea_navsat_driver 의 status 로는 둘이 안 갈렸다 — gold gps.py ①절).
  ⚠️ Fixed 라도 틀릴 수 있다 — 2026-10-13 시험에서 1번 수신기가 10분간 1.3 m 틀린 Fixed 를 냈다
     (CLAUDE.md 5.4). 이 노드는 수신기가 말한 품질을 그대로 전한다.
  · F9P 두 대가 USB 상으로 똑같이 보인다(info.md) — 한 대만 꽂는다. 둘 다 꽂혀 있으면 번호가
    작은 쪽을 연다.

IMU : black · gold 의 iahrs.py 드라이버 로직 그대로 (so=1, sp=주기, sd=가속도|각속도|오일러).
  CP210x(10c4:ea60) 포트를 찾고, 못 열거나 빠지면 2 초마다 다시 찾는다. TF 는 내지 않는다.

발행
  /gps_pvt    (Float64MultiArray)  ★driving · mapping 이 쓰는 것★ — 아래 GPS_PVT 규약
  /fix        (NavSatFix)          확인 · rosbag 용 (status: −1 없음 · 0 단독 · 1 DGNSS · 2 RTK Float 이상)
  /imu        (Imu)                iAHRS 원값 (가속도 m/s², 각속도 rad/s, 자세 쿼터니언)
  /gps_status (String)             1 Hz 한 줄 상태 (prompt 메인 화면)
"""

import base64
import math
import signal
import socket
import struct
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from std_msgs.msg import Float64MultiArray, String

import serial

from white import ports

# ── /gps_pvt 배열 규약 ★순서를 바꾸면 driving · mapping 이 어긋난다★ ──
#  [0] lat_deg   [1] lon_deg   [2] quality (Q_*)   [3] hacc_m (수신기 수평 정확도 추정)
#  [4] speed_mps (지면속도)    [5] course_deg (진행방위, 동 0° 반시계 + — driving 헤딩과 같은 규약,
#      0.3 m/s 미만이면 NaN)   [6] num_sv   [7] itow_s (GPS 주 시각)   [8] rtcm_age_s (마지막 보정 수신 후, NaN=없음)
#  [9] hmsl_m (해발고도 — 매핑 CSV 의 alt 열)
GPS_PVT_TOPIC = '/gps_pvt'
GPS_PVT_FIELDS = 10
Q_NONE, Q_SPS, Q_DGPS, Q_FLOAT, Q_FIXED = 0, 1, 2, 3, 4
Q_LABEL = {Q_NONE: "측위없음", Q_SPS: "단독", Q_DGPS: "DGNSS", Q_FLOAT: "RTK Float", Q_FIXED: "RTK Fixed"}
COURSE_MIN_SPEED = 0.3

NTRIP = {
    "host": "rts1.ngii.go.kr",
    "port": 2101,
    "mountpoint": "VRS-RTCM34",
    "user": "knut_gps",
    "password": "ngii",
}

GPS_BAUD = 38400             # USB 가상 시리얼(CDC-ACM)이라 실제 속도와 무관
GGA_UPLOAD_SEC = 10          # VRS 캐스터로 현재 위치(GGA)를 올리는 주기
NTRIP_RETRY_SEC = 5          # 연결이 끊기면 이만큼 쉬고 다시 접속
NTRIP_FATAL_RETRY_SEC = 30   # 인증 실패 등은 길게 쉬고 다시 시도
NTRIP_NO_DATA_SEC = 20       # 접속 중 이 시간 동안 보정이 안 오면 끊고 다시 접속
PVT_STALE_SEC = 2.0          # NAV-PVT 가 이보다 오래 안 오면 'GPS 데이터 없음'

# 접속할 때마다 RAM 레이어에만 쓰는 수신기 설정: (키 이름, 키 ID, 바이트 수, 값)  — gps_mapper.py 와 같다
RECEIVER_RAM_CONFIG = [
    ("CFG-RATE-MEAS", 0x30210001, 2, 100),              # 10 Hz
    ("CFG-UART1OUTPROT-NMEA", 0x10740002, 1, 0),
    ("CFG-I2COUTPROT-NMEA", 0x10720002, 1, 0),
    ("CFG-USBOUTPROT-NMEA", 0x10780002, 1, 1),
    ("CFG-USBOUTPROT-UBX", 0x10780001, 1, 1),
    ("CFG-USBINPROT-RTCM3X", 0x10770004, 1, 1),
    ("CFG-MSGOUT-NMEA_ID_GGA_USB", 0x209100BD, 1, 1),
    ("CFG-MSGOUT-NMEA_ID_RMC_USB", 0x209100AE, 1, 1),
    ("CFG-MSGOUT-NMEA_ID_GSV_USB", 0x209100C7, 1, 0),
    ("CFG-MSGOUT-NMEA_ID_GSA_USB", 0x209100C2, 1, 0),
    ("CFG-MSGOUT-NMEA_ID_VTG_USB", 0x209100B3, 1, 0),
    ("CFG-MSGOUT-NMEA_ID_GLL_USB", 0x209100CC, 1, 0),
    ("CFG-MSGOUT-UBX_NAV_PVT_USB", 0x20910009, 1, 1),   # 매 측위마다 NAV-PVT
]

# ── iAHRS ──
IMU_BAUD = 115200
IMU_PERIOD_MS = 50           # 20 Hz (driving 제어 주기와 같다). 20 = 50 Hz
IMU_SYNC_DATA = 0x0004 | 0x0008 | 0x0040   # 가속도(g) | 각속도(deg/s) | 오일러(deg)
IMU_RETRY_SEC = 2.0
IMU_STALE_SEC = 1.0
G = 9.80665


# ══════════════════════════════════════════════════════════════════════════════
#  u-blox 프로토콜 (gps_mapper.py)
# ══════════════════════════════════════════════════════════════════════════════
def ubx(cls, id_, payload=b""):
    body = bytes([cls, id_]) + struct.pack("<H", len(payload)) + payload
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return b"\xb5\x62" + body + bytes([a, b])


def ubx_ck_ok(frame):
    a = b = 0
    for x in frame[2:-2]:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return frame[-2] == a and frame[-1] == b


def nmea_ok(line):
    star = line.rfind("*")
    if not line.startswith("$") or star < 0 or len(line) < star + 3:
        return False
    c = 0
    for ch in line[1:star].encode("ascii", "replace"):
        c ^= ch
    try:
        return c == int(line[star + 1:star + 3], 16)
    except ValueError:
        return False


def parse_gga(line):
    f = line[:line.rfind("*")].split(",")
    if len(f) < 15:
        return None
    try:
        q = int(f[6] or 0)
    except ValueError:
        return None
    return dict(raw=line, q=q)


def parse_pvt(p):
    itow = struct.unpack_from("<I", p, 0)[0]
    flags = p[21]
    lon, lat, height, hmsl, hacc, vacc = struct.unpack_from("<iiiiII", p, 24)
    gspeed = struct.unpack_from("<i", p, 60)[0]
    headmot = struct.unpack_from("<i", p, 64)[0]
    return dict(itow=itow, fix=p[20], ok=bool(flags & 1), diff=bool(flags & 2),
                carr=(flags >> 6) & 3, sv=p[23], lat=lat * 1e-7, lon=lon * 1e-7,
                hmsl=hmsl / 1000, hacc=hacc / 1000, speed=gspeed / 1000,
                headmot=headmot * 1e-5)


def pvt_quality(pvt):
    """NAV-PVT 상태 → Q_* (순서가 곧 품질 순서다 — Float < Fixed)."""
    if not pvt["ok"] or pvt["fix"] == 0:
        return Q_NONE
    if pvt["carr"] == 2:
        return Q_FIXED
    if pvt["carr"] == 1:
        return Q_FLOAT
    return Q_DGPS if pvt["diff"] else Q_SPS


def apply_receiver_config(ser, ser_lock):
    """RECEIVER_RAM_CONFIG 를 CFG-VALSET 으로 RAM 에만 쓴다. ACK 를 받으면 True."""
    payload = bytes([0, 0x01, 0, 0])  # version 0, layer RAM
    for _, key, size, value in RECEIVER_RAM_CONFIG:
        payload += struct.pack("<I", key) + value.to_bytes(size, "little")
    msg = ubx(0x06, 0x8A, payload)
    ack = b"\xb5\x62\x05\x01\x02\x00\x06\x8a"
    nak = b"\xb5\x62\x05\x00\x02\x00\x06\x8a"
    for _ in range(5):  # 공장 설정(과부하) 상태에서는 ACK 가 빠질 수 있어 재시도
        ser.reset_input_buffer()
        with ser_lock:
            ser.write(msg)
        buf, t0 = b"", time.time()
        while time.time() - t0 < 1.0:
            buf += ser.read(4096)
            if ack in buf:
                return True
            if nak in buf:
                return False
    return False


class F9PStream:
    """시리얼 바이트를 NMEA GGA 와 UBX NAV-PVT 로 나눈다."""

    def __init__(self):
        self.buf = b""

    def feed(self, data):
        b, i = self.buf + data, 0
        ggas, pvts = [], []
        while i < len(b):
            if b[i] == 0xB5:
                if len(b) - i < 8:
                    break
                if b[i + 1] == 0x62:
                    n = struct.unpack_from("<H", b, i + 4)[0]
                    if n <= 1024:
                        if len(b) - i < 8 + n:
                            break
                        if b[i + 2:i + 4] == b"\x01\x07" and n == 92 and ubx_ck_ok(b[i:i + 8 + n]):
                            pvts.append(parse_pvt(b[i + 6:i + 6 + n]))
                        i += 8 + n
                        continue
            elif b[i] == 0x24:  # '$'
                e = b.find(b"\r\n", i)
                if e < 0:
                    if len(b) - i > 200:      # 줄 끝이 영영 안 오면(깨진 줄) 건너뛴다
                        i += 1
                        continue
                    break
                line = b[i:e].decode("ascii", "replace")
                if line[3:6] == "GGA" and nmea_ok(line):
                    g = parse_gga(line)
                    if g:
                        ggas.append(g)
                i = e + 2
                continue
            i += 1
        self.buf = b[i:]
        return ggas, pvts


# ══════════════════════════════════════════════════════════════════════════════
#  NTRIP (gps_mapper.py — 기지국 해독 RtcmMonitor 는 뺐다)
# ══════════════════════════════════════════════════════════════════════════════
class NtripFatal(Exception):
    pass


class NtripClient(threading.Thread):
    """NGII 캐스터 접속을 유지한다. 받은 RTCM3 는 sink(bytes) 로 넘긴다(시리얼 쓰기).

    status : off / wait_pos(위치 대기) → connecting → streaming, 실패하면 retry / fatal 후 다시 시도.
    """

    def __init__(self, cfg, sink, log):
        super().__init__(daemon=True)
        self.cfg, self.sink, self.log = cfg, sink, log
        self.sock = None
        self.gga_raw = None
        self.enabled = True
        self.status = "wait_pos"
        self.detail = ""
        self.bytes_in = 0
        self.last_data_t = 0.0
        self.stop_evt = threading.Event()
        self.wake = threading.Event()

    def stop(self):
        self.stop_evt.set()
        self.wake.set()
        self._close()

    def _sleep(self, sec):
        self.wake.wait(sec)
        self.wake.clear()

    def _set(self, status, detail=""):
        if status != self.status or detail != self.detail:
            if status in ("streaming", "retry", "fatal"):
                self.log(f"NTRIP {status} {detail}".rstrip())
        self.status, self.detail = status, detail

    def run(self):
        while not self.stop_evt.is_set():
            if not self.enabled:
                self._set("off")
                self._sleep(1.0)
                continue
            if not self.gga_raw:
                self._set("wait_pos")
                self._sleep(0.5)
                continue
            self._set("connecting")
            wait = NTRIP_RETRY_SEC
            try:
                self._stream(self._connect())
            except NtripFatal as e:
                self._set("fatal", str(e))
                wait = NTRIP_FATAL_RETRY_SEC
            except OSError as e:
                self._set("retry", str(e) or type(e).__name__)
            self._close()
            if self.stop_evt.is_set():
                return
            self._sleep(wait)

    def _connect(self):
        c = self.cfg
        auth = base64.b64encode(f"{c['user']}:{c['password']}".encode()).decode()
        req = (f"GET /{c['mountpoint']} HTTP/1.0\r\n"
               f"Host: {c['host']}\r\n"
               f"User-Agent: NTRIP white/1.0\r\n"
               f"Authorization: Basic {auth}\r\n"
               f"Accept: */*\r\n"
               f"Connection: close\r\n\r\n")
        sock = socket.create_connection((c["host"], c["port"]), timeout=10)
        self.sock = sock
        sock.sendall(req.encode())
        head = b""
        while b"\r\n" not in head:
            chunk = sock.recv(1024)
            if not chunk:
                raise OSError("응답 없이 연결 종료")
            head += chunk
        first, _, rest = head.partition(b"\r\n")
        first = first.decode("latin1").strip()
        if first.startswith("ICY 200"):
            pass
        elif first.startswith("HTTP/") and " 200" in first:
            rest = b"\r\n" + rest
            while b"\r\n\r\n" not in rest:
                chunk = sock.recv(1024)
                if not chunk:
                    raise OSError("헤더 수신 중 연결 종료")
                rest += chunk
            rest = rest.split(b"\r\n\r\n", 1)[1]
        elif "SOURCETABLE" in first:
            raise NtripFatal(f"마운트포인트 '{c['mountpoint']}' 없음")
        elif "401" in first:
            raise NtripFatal("인증 실패(401) — ID 확인, 다른 곳에서 같은 ID로 접속 중인지 확인")
        else:
            raise NtripFatal(f"예상 밖 응답: {first}")
        self._set("streaming", c["mountpoint"])
        self.last_data_t = time.time()
        self._send_gga(sock)
        if rest:
            self._feed(rest)
        return sock

    def _stream(self, sock):
        sock.settimeout(1.0)
        last_gga = time.time()
        while not self.stop_evt.is_set() and self.enabled:
            now = time.time()
            if now - last_gga >= GGA_UPLOAD_SEC:
                self._send_gga(sock)
                last_gga = now
            if now - self.last_data_t > NTRIP_NO_DATA_SEC:
                raise OSError(f"{NTRIP_NO_DATA_SEC}초 동안 보정 데이터 없음")
            try:
                data = sock.recv(4096)
            except socket.timeout:
                continue
            if not data:
                raise OSError("캐스터가 연결을 닫음")
            self._feed(data)

    def _send_gga(self, sock):
        if self.gga_raw:
            sock.sendall((self.gga_raw + "\r\n").encode("ascii"))

    def _feed(self, data):
        self.bytes_in += len(data)
        self.last_data_t = time.time()
        self.sink(data)

    def _close(self):
        s, self.sock = self.sock, None
        if s:
            try:
                s.close()
            except OSError:
                pass


# ══════════════════════════════════════════════════════════════════════════════
#  노드
# ══════════════════════════════════════════════════════════════════════════════
def euler_to_quaternion(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


class GpsNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.declare_parameter('gps_port', '')        # 비우면 VID 로 찾는다
        self.declare_parameter('imu_port', '')
        self.declare_parameter('ntrip', True)         # false = 보정 안 받음 (단독/DGNSS)
        self.declare_parameter('imu_period_ms', IMU_PERIOD_MS)
        self.gps_port_param = str(self.get_parameter('gps_port').value)
        self.imu_port_param = str(self.get_parameter('imu_port').value)
        self.imu_period_ms = int(self.get_parameter('imu_period_ms').value)

        self.stop_evt = threading.Event()
        self.gps_ser = None
        self.gps_lock = threading.Lock()
        self.gps_conn = "search"
        self.gps_port = ""
        self.cfg_ok = None
        self.pvt = None
        self.pvt_t = 0.0
        self.quality = Q_NONE
        self.imu_conn = "search"
        self.imu_port = ""
        self.imu_t = 0.0
        self.imu_n = 0

        self.pub_pvt = self.create_publisher(Float64MultiArray, GPS_PVT_TOPIC, 20)
        self.pub_fix = self.create_publisher(NavSatFix, '/fix', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu', 20)
        self.pub_status = self.create_publisher(String, '/gps_status', 10)
        self.create_timer(1.0, self.publish_status)

        self.ntrip = NtripClient(NTRIP, self._write_rtcm, self.get_logger().info)
        self.ntrip.enabled = bool(self.get_parameter('ntrip').value)
        self.ntrip.start()
        threading.Thread(target=self.gps_worker, daemon=True).start()
        threading.Thread(target=self.imu_worker, daemon=True).start()
        self.get_logger().info(
            f"GPS(F9P {'+ NTRIP ' + NTRIP['mountpoint'] if self.ntrip.enabled else '보정 없음'}) · "
            f"IMU(iAHRS {1000 // max(1, self.imu_period_ms)} Hz) 시작")

    # ── GPS ──────────────────────────────────────────────────────────────────
    def _write_rtcm(self, data):
        with self.gps_lock:
            if self.gps_ser:
                try:
                    self.gps_ser.write(data)
                except (serial.SerialException, OSError):
                    pass

    def gps_worker(self):
        while not self.stop_evt.is_set():
            dev = self.gps_port_param or ports.find_gps()
            if not dev:
                self.gps_conn, self.gps_port = "search", ""
                self.stop_evt.wait(1.0)
                continue
            self.gps_conn, self.gps_port = "opening", dev
            try:
                ser = serial.Serial(dev, GPS_BAUD, timeout=0.05, exclusive=True)
            except (serial.SerialException, OSError) as e:
                self.gps_conn = "busy"
                self.get_logger().warning(f"GPS 포트 열기 실패 {dev}: {e}", throttle_duration_sec=10.0)
                self.stop_evt.wait(2.0)
                continue
            with self.gps_lock:
                self.gps_ser = ser
            try:
                self.cfg_ok = apply_receiver_config(ser, self.gps_lock)
                self.gps_conn = "open"
                self.get_logger().info(
                    f"GPS 연결 {dev} — 수신기 설정 {'ACK' if self.cfg_ok else '★응답 없음★'}")
                ser.reset_input_buffer()
                stream = F9PStream()
                while not self.stop_evt.is_set():
                    ggas, pvts = stream.feed(ser.read(4096))
                    for g in ggas:
                        if g["q"] > 0:
                            self.ntrip.gga_raw = g["raw"]
                    for p in pvts:
                        self._on_pvt(p)
            except (serial.SerialException, OSError) as e:
                self.gps_conn = "lost"
                self.get_logger().error(f"GPS 단절: {e} — 다시 찾는다")
            finally:
                with self.gps_lock:
                    self.gps_ser = None
                try:
                    ser.close()
                except (serial.SerialException, OSError):
                    pass
            self.stop_evt.wait(1.0)

    def _on_pvt(self, p):
        now = time.time()
        q = pvt_quality(p)
        if q != self.quality:
            self.get_logger().info(f"GPS 품질 {Q_LABEL[self.quality]} → {Q_LABEL[q]} (hAcc {p['hacc']:.3f} m)")
        self.quality, self.pvt, self.pvt_t = q, p, now
        if q == Q_NONE:
            return                       # 좌표가 없다 — 내보내지 않는다(driving 은 두절로 본다)
        course = (90.0 - p["headmot"] + 180.0) % 360.0 - 180.0 \
            if p["speed"] >= COURSE_MIN_SPEED else float('nan')
        age = now - self.ntrip.last_data_t if self.ntrip.last_data_t > 0 else float('nan')
        self.pub_pvt.publish(Float64MultiArray(data=[
            p["lat"], p["lon"], float(q), p["hacc"], p["speed"], course,
            float(p["sv"]), p["itow"] / 1000.0, age, p["hmsl"]]))
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps'
        fix.status.status = {Q_SPS: NavSatStatus.STATUS_FIX, Q_DGPS: NavSatStatus.STATUS_SBAS_FIX}.get(
            q, NavSatStatus.STATUS_GBAS_FIX)
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude, fix.longitude, fix.altitude = p["lat"], p["lon"], p["hmsl"]
        var = p["hacc"] ** 2
        fix.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, 4.0 * var]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.pub_fix.publish(fix)

    # ── IMU ──────────────────────────────────────────────────────────────────
    def imu_worker(self):
        while not self.stop_evt.is_set():
            dev = self.imu_port_param or ports.find_imu(exclude=(self.gps_port,))
            if not dev:
                self.imu_conn, self.imu_port = "search", ""
                self.stop_evt.wait(IMU_RETRY_SEC)
                continue
            try:
                ser = serial.Serial(dev, IMU_BAUD, timeout=0.05, exclusive=True)
            except (serial.SerialException, OSError) as e:
                self.imu_conn = "busy"
                self.get_logger().warning(f"IMU 포트 열기 실패 {dev}: {e}", throttle_duration_sec=15.0)
                self.stop_evt.wait(IMU_RETRY_SEC)
                continue
            self.imu_conn, self.imu_port = "open", dev
            try:
                ser.reset_input_buffer()
                self._imu_cmd(ser, "so=1")
                self._imu_cmd(ser, f"sp={self.imu_period_ms}")
                self._imu_cmd(ser, f"sd=0x{IMU_SYNC_DATA:X}")
                self.get_logger().info(f"IMU 연결 {dev} @ {IMU_BAUD}")
                msg = Imu()
                msg.header.frame_id = 'imu_link'
                msg.linear_acceleration_covariance[0] = 0.0064
                msg.linear_acceleration_covariance[4] = 0.0063
                msg.linear_acceleration_covariance[8] = 0.0064
                msg.angular_velocity_covariance[0] = math.radians(0.032)
                msg.angular_velocity_covariance[4] = math.radians(0.028)
                msg.angular_velocity_covariance[8] = math.radians(0.006)
                msg.orientation_covariance[0] = math.radians(0.013)
                msg.orientation_covariance[4] = math.radians(0.011)
                msg.orientation_covariance[8] = math.radians(0.006)
                while not self.stop_evt.is_set():
                    line = ser.readline()
                    if line:
                        self._on_imu_line(line, msg)
            except (serial.SerialException, OSError) as e:
                self.imu_conn = "lost"
                self.get_logger().error(f"IMU 단절: {e} — 다시 찾는다")
            finally:
                try:
                    ser.close()
                except (serial.SerialException, OSError):
                    pass
            self.stop_evt.wait(IMU_RETRY_SEC)

    @staticmethod
    def _imu_cmd(ser, cmd, timeout=0.5):
        """명령을 쓰고 같은 줄이 되돌아오길 잠깐 기다린다(안 와도 넘어간다 — 부팅 충돌 방지)."""
        ser.write((cmd + "\n").encode())
        t0 = time.time()
        while time.time() - t0 < timeout:
            line = ser.readline()
            if line and line.split(b"\r\n")[0].decode("utf-8", "ignore").strip() == cmd:
                return True
        return False

    def _on_imu_line(self, line, msg):
        s = line.split(b"\r\n")[0].decode("utf-8", "ignore").strip()
        if not s or "=" in s:
            return
        f = s.split(",")
        if len(f) != 9:
            return
        try:
            d = [float(x) for x in f]
        except ValueError:
            return
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = \
            d[0] * G, d[1] * G, d[2] * G
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = \
            math.radians(d[3]), math.radians(d[4]), math.radians(d[5])
        w, x, y, z = euler_to_quaternion(math.radians(d[6]), math.radians(d[7]), math.radians(d[8]))
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z = w, x, y, z
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_imu.publish(msg)
        self.imu_t = time.time()
        self.imu_n += 1

    # ── 상태 ─────────────────────────────────────────────────────────────────
    def publish_status(self):
        now = time.time()
        if self.gps_conn != "open":
            g = {"search": "GPS 찾는 중(VID 1546)", "busy": f"GPS 포트 사용 중 {self.gps_port}",
                 "lost": "GPS 단절", "opening": f"GPS 여는 중 {self.gps_port}"}.get(self.gps_conn, self.gps_conn)
        elif self.pvt is None or now - self.pvt_t > PVT_STALE_SEC:
            g = f"GPS {self.gps_port} 데이터 없음" + ("" if self.cfg_ok else " (수신기 설정 무응답)")
        else:
            p = self.pvt
            g = f"GPS {Q_LABEL[self.quality]} hAcc {p['hacc']*100:.1f}cm 위성 {p['sv']}"
        nt = self.ntrip.status
        if nt == "streaming":
            age = now - self.ntrip.last_data_t
            n = f"NTRIP 수신 {self.ntrip.bytes_in // 1024}KB (최근 {age:.0f}s)"
        else:
            n = f"NTRIP {nt} {self.ntrip.detail}".rstrip()
        if self.imu_conn != "open":
            i = "IMU 찾는 중(10c4:ea60)" if self.imu_conn == "search" else f"IMU {self.imu_conn}"
        elif now - self.imu_t > IMU_STALE_SEC:
            i = f"IMU {self.imu_port} 데이터 없음"
        else:
            i = f"IMU {self.imu_port} 정상"
        self.pub_status.publish(String(data=f"{g} | {n} | {i}"))

    def shutdown(self):
        self.stop_evt.set()
        self.ntrip.stop()


def main(args=None):
    rclpy.init(args=args)
    node = GpsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # Ctrl-C 두 번째(launch 가 다시 보낸다)가 정리를 끊지 않게
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
