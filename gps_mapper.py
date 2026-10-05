#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gps_mapper.py ― GPS 궤적 수집 (Windows 11 · tkinter)
════════════════════════════════════════════════════════════════════════════════
USB 로 꽂힌 u-blox ZED-F9P 를 자동으로 찾아 접속하고, 국토지리정보원(NGII) NTRIP VRS 로
RTK 보정을 넣으면서 사람이 몰고 다닌 궤적을 CSV 로 남긴다. 저장 위치는 이 파일 옆의
gps_data/route_YYYYmmdd_HHMMSS.csv.

CSV 형식 : 앞 15열은 white1/mapping.py 와 ★열 이름·순서가 같다★ (driving·find_wc 호환).
  값이 있는 열은 latitude · longitude · speed 뿐이고 나머지는 전부 0 이다
  (조향·엔코더·페달 같은 차량 신호가 없는 단독 매핑 도구이므로).
  ★끝에 2열을 더 붙인다★ — find_wc 가 이 열을 보고 구간별 RTK 품질을 색으로 그린다.
    gps_quality : NMEA GGA 품질 번호 (0 없음 · 1 단독 · 2 DGNSS · 4 RTK Fixed · 5 RTK Float)
    gps_sigma_m : 수신기 수평 정확도 추정 hAcc [m]
  둘 다 위치와 ★같은 NAV-PVT★ 에서 뽑으므로 점마다 시점이 정확히 맞는다.

기록 규칙 : ★일정 거리마다 한 점★ (시간 간격이 아니라) — 기본 0.25 m, 창에서 조절.
  신호 대기처럼 멈춰 있는 동안 같은 자리가 수백 줄 쌓이면 경로가 아니라 점 뭉치가 되기
  때문이다(mapping.py 와 같은 규칙). 측위가 안 된(위성 없음) 동안은 기록하지 않는다.

GPS 자동 감지 : u-blox USB VID(0x1546)인 COM 포트만 고른다. COM 번호가 매번 달라도 된다.
  ★다른 COM 포트는 열지 않는다★ — 아두이노는 포트를 여는 순간 리셋되므로, 아무 포트나
  열어 보고 확인하는 방식은 쓰지 않는다. USB 가 빠지면 다시 찾아 자동으로 붙는다.

수신기 설정 : 접속할 때마다 ★RAM 에만★ 쓴다(전원을 뺐다 꽂으면 공장 설정으로 돌아간다).
  공장 설정은 NMEA 6종을 USB·UART1(38400bps)·I2C 로 똑같이 10 Hz 출력해 출력 버퍼가
  넘치고(`$GNTXT txbuf alloc`), 그러면 RTCM 보정을 띄엄띄엄 써서 Fix 가 안 된다
  (gpstest.py 끝 주석에 측정 근거). 그래서 UART1·I2C 의 NMEA 를 끄고 USB 는 GGA·RMC 만 둔다.
  여기에 ★NAV-PVT 10 Hz★ 를 켠다 — 위치(1e-7° ≈ 1.1 cm), 지면속도, hAcc, RTK 상태를
  한 메시지로 준다(GGA 위도는 1.9 cm 단위로 끊긴다). GGA 는 VRS 캐스터에 현재 위치를
  올리는 데만 쓴다.

경로 확인 : 목록에서 CSV 를 누르면 find_wc(카카오 위성지도)가 그 경로를 바로 그린다.
  CSV 를 gzip → base64url 로 URL 프래그먼트(#csvgz=…&name=…)에 싣는다 — find_wc 의
  src/lib/inbound.ts 가 받는 형식. 프래그먼트는 서버로 전송되지 않으므로 좌표가 GitHub
  Pages 로그에 남지 않는다. 경로가 길면 주소가 Windows 명령줄 길이 제한을 넘으므로,
  임시 HTML 을 열어 그 안에서 location.replace 로 넘어간다.
"""

import base64
import csv
import ctypes
import gzip
import json
import math
import os
import socket
import struct
import sys
import tempfile
import threading
import time
import tkinter as tk
import urllib.parse
import webbrowser
from collections import deque
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk

import serial
import serial.tools.list_ports


APP_DIR = Path(__file__).resolve().parent
DATA_DIR = APP_DIR / "gps_data"
FIND_WC_URL = "https://cheongbaek.github.io/find_wc/"

NTRIP = {
    "host": "rts1.ngii.go.kr",
    "port": 2101,
    "mountpoint": "VRS-RTCM34",
    "user": "knut_gps",
    "password": "ngii",
}

UBLOX_VID = 0x1546
SERIAL_BAUD = 38400          # USB 가상 COM 이라 실제 속도와 무관
SPACING_M = 0.25             # 이 거리마다 한 점 기록 (mapping.py 와 같은 기본값)
SPACING_MIN, SPACING_MAX = 0.05, 10.0
GGA_UPLOAD_SEC = 10          # VRS 캐스터로 현재 위치(GGA)를 올리는 주기
NTRIP_RETRY_SEC = 5          # 연결이 끊기면 이만큼 쉬고 다시 접속
NTRIP_FATAL_RETRY_SEC = 30   # 인증 실패 등은 길게 쉬고 다시 시도
NTRIP_NO_DATA_SEC = 20       # 접속 중 이 시간 동안 보정이 안 오면 끊고 다시 접속
PVT_STALE_SEC = 2.0          # NAV-PVT 가 이보다 오래 안 오면 'GPS 데이터 없음'
EARTH_R = 6378137.0

CSV_HEADER = [
    # ── white1/mapping.py 와 같은 열 (앞 8열은 구 white 와도 같다) ──
    "latitude", "longitude", "heading", "speed", "steer",
    "direction", "pitch", "terrain",
    "throttle_pulse", "wheel_pulse", "wheel_speed", "steer_measured",
    "throttle_raw", "auto_mode", "estop",
    # ── find_wc 품질 표시용 (mapping.py 에는 없는 열) ──
    "gps_quality", "gps_sigma_m",
]

# 접속할 때마다 RAM 레이어에만 쓰는 수신기 설정: (키 이름, 키 ID, 바이트 수, 값)
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

GREEN, AMBER, ORANGE, RED, GRAY, BLUE = "#1b8a3a", "#c98a00", "#d9480f", "#c92a2a", "#6c757d", "#1c7ed6"
FONT = "Malgun Gothic"


# ── 프로토콜 도우미 ────────────────────────────────────────────────────────────────
def ubx(cls, id_, payload=b""):
    body = bytes([cls, id_]) + struct.pack("<H", len(payload)) + payload
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return b"\xb5\x62" + body + bytes([a, b])


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
    return dict(raw=line, q=int(f[6] or 0), age=f[13])


def parse_pvt(p):
    flags = p[21]
    lon, lat, height, hmsl, hacc, vacc = struct.unpack_from("<iiiiII", p, 24)
    gspeed = struct.unpack_from("<i", p, 60)[0]
    pdop = struct.unpack_from("<H", p, 76)[0]
    return dict(t=time.time(), fix=p[20], ok=bool(flags & 1), diff=bool(flags & 2),
                carr=(flags >> 6) & 3, sv=p[23], lat=lat * 1e-7, lon=lon * 1e-7,
                hmsl=hmsl / 1000, hacc=hacc / 1000, vacc=vacc / 1000,
                speed=gspeed / 1000, pdop=pdop * 0.01)


def gga_quality(pvt):
    """NAV-PVT 상태 → NMEA GGA 품질 번호 (find_wc 가 4=Fixed, 5=Float 로 읽는다)."""
    if not pvt["ok"] or pvt["fix"] == 0:
        return 0
    if pvt["carr"] == 2:
        return 4
    if pvt["carr"] == 1:
        return 5
    return 2 if pvt["diff"] else 1


def delta_m(lat0, lon0, lat1, lon1):
    """mapping.py 의 _delta 와 같은 평면 근사(동, 북) [m]."""
    x = EARTH_R * math.radians(lon1 - lon0) * math.cos(math.radians(lat0))
    y = EARTH_R * math.radians(lat1 - lat0)
    return x, y


def find_gps_port():
    """u-blox(VID 0x1546) COM 포트. 다른 포트는 열어 보지 않는다(아두이노 리셋 방지)."""
    ports = sorted((p for p in serial.tools.list_ports.comports() if p.vid == UBLOX_VID),
                   key=lambda p: p.device)
    return ports[0] if ports else None


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
                        if b[i + 2:i + 4] == b"\x01\x07" and n == 92:
                            pvts.append(parse_pvt(b[i + 6:i + 6 + n]))
                        i += 8 + n
                        continue
            elif b[i] == 0x24:  # '$'
                e = b.find(b"\r\n", i)
                if e < 0:
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


# ── NTRIP ─────────────────────────────────────────────────────────────────────────
class NtripFatal(Exception):
    pass


class NtripClient(threading.Thread):
    """NGII 캐스터 접속을 계속 유지한다. 받은 RTCM3 는 sink(bytes) 로 넘긴다(시리얼 쓰기).

    status : wait_pos(위치 대기) → connecting → streaming, 실패하면 retry / fatal 후 다시 시도.
    """

    def __init__(self, cfg, sink):
        super().__init__(daemon=True)
        self.cfg, self.sink = cfg, sink
        self.sock = None
        self.gga_raw = None
        self.status = "wait_pos"
        self.detail = ""
        self.attempts = 0
        self.retry_at = 0.0
        self.bytes_in = 0
        self.session_bytes = 0
        self.last_data_t = 0.0
        self.stop_evt = threading.Event()

    def stop(self):
        self.stop_evt.set()
        self._close()

    def run(self):
        while not self.stop_evt.is_set():
            if not self.gga_raw:
                self.status = "wait_pos"
                self.stop_evt.wait(0.5)
                continue
            self.attempts += 1
            self.status, self.detail = "connecting", ""
            wait = NTRIP_RETRY_SEC
            try:
                self._connect()
                self._stream()
            except NtripFatal as e:
                self.status, self.detail, wait = "fatal", str(e), NTRIP_FATAL_RETRY_SEC
            except OSError as e:
                self.status, self.detail = "retry", str(e) or type(e).__name__
            self._close()
            if self.stop_evt.is_set():
                return
            self.retry_at = time.time() + wait
            self.stop_evt.wait(wait)

    def _connect(self):
        c = self.cfg
        auth = base64.b64encode(f"{c['user']}:{c['password']}".encode()).decode()
        req = (f"GET /{c['mountpoint']} HTTP/1.0\r\n"
               f"Host: {c['host']}\r\n"
               f"User-Agent: NTRIP gps_mapper/1.0\r\n"
               f"Authorization: Basic {auth}\r\n"
               f"Accept: */*\r\n"
               f"Connection: close\r\n\r\n")
        self.sock = socket.create_connection((c["host"], c["port"]), timeout=10)
        self.sock.sendall(req.encode())
        head = b""
        while b"\r\n" not in head:
            chunk = self.sock.recv(1024)
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
                chunk = self.sock.recv(1024)
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
        self.status, self.detail = "streaming", first
        self.session_bytes = 0
        self.last_data_t = time.time()
        self._send_gga()
        if rest:
            self._feed(rest)

    def _stream(self):
        self.sock.settimeout(1.0)
        last_gga = time.time()
        while not self.stop_evt.is_set():
            now = time.time()
            if now - last_gga >= GGA_UPLOAD_SEC:
                self._send_gga()
                last_gga = now
            if now - self.last_data_t > NTRIP_NO_DATA_SEC:
                raise OSError(f"{NTRIP_NO_DATA_SEC}초 동안 보정 데이터 없음")
            try:
                data = self.sock.recv(4096)
            except socket.timeout:
                continue
            if not data:
                raise OSError("캐스터가 연결을 닫음")
            self._feed(data)

    def _send_gga(self):
        if self.gga_raw and self.sock:
            self.sock.sendall((self.gga_raw + "\r\n").encode("ascii"))

    def _feed(self, data):
        self.bytes_in += len(data)
        self.session_bytes += len(data)
        self.last_data_t = time.time()
        self.sink(data)

    def _close(self):
        s, self.sock = self.sock, None
        if s:
            try:
                s.close()
            except OSError:
                pass


# ── CSV 기록 ──────────────────────────────────────────────────────────────────────
class RouteRecorder:
    """매핑 시작/종료와 일정 거리 기록. GPS 스레드가 on_fix 를, 창이 start/stop 을 부른다."""

    def __init__(self):
        self.lock = threading.Lock()
        self.spacing = SPACING_M
        self.fp = self.writer = None
        self.path = None
        self.rows = 0
        self.length = 0.0
        self.started = 0.0
        self.last = None    # 마지막으로 '기록한' 점

    @property
    def active(self):
        return self.fp is not None

    def start(self):
        with self.lock:
            DATA_DIR.mkdir(exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.path = DATA_DIR / f"route_{stamp}.csv"
            self.fp = open(self.path, "w", newline="", encoding="utf-8")
            self.writer = csv.writer(self.fp)
            self.writer.writerow(CSV_HEADER)
            self.fp.flush()
            self.rows, self.length, self.last = 0, 0.0, None
            self.started = time.time()
            return self.path

    def stop(self):
        with self.lock:
            if self.fp:
                try:
                    self.fp.close()
                except OSError:
                    pass
            self.fp = self.writer = None
            return self.path, self.rows, self.length

    def on_fix(self, lat, lon, speed, quality, sigma):
        with self.lock:
            if not self.fp:
                return
            if self.last is not None:
                d = math.hypot(*delta_m(self.last[0], self.last[1], lat, lon))
                if d < self.spacing:
                    return
                self.length += d
            self.writer.writerow([
                f"{lat:.8f}", f"{lon:.8f}",
                "0.00",              # heading
                f"{speed:.3f}",      # speed — NAV-PVT 지면속도 [m/s]
                "0", "0", "0.00", "0",                   # steer / direction / pitch / terrain
                "0", "0.0", "0.000", "0", "0", "0", "0",  # 차량 실계측 열 — 없음
                quality,             # gps_quality — NMEA GGA 품질 번호
                f"{sigma:.3f}",      # gps_sigma_m — hAcc [m]
            ])
            self.fp.flush()      # 기록 중인 파일도 목록에서 바로 열어 볼 수 있게
            self.rows += 1
            self.last = (lat, lon)


# ── GPS 스레드 ────────────────────────────────────────────────────────────────────
class GpsWorker(threading.Thread):
    """포트 감지 → 접속 → 수신기 설정 → 읽기. USB 가 빠지면 다시 찾는다."""

    def __init__(self, recorder):
        super().__init__(daemon=True)
        self.recorder = recorder
        self.lock = threading.Lock()       # self.state 보호
        self.ser_lock = threading.Lock()   # 시리얼 열기·닫기·쓰기 보호
        self.ser = None
        self.stop_evt = threading.Event()
        self.state = dict(conn="search", port="", cfg_ok=None, err="",
                          pvt=None, gga=None)
        self.ntrip = NtripClient(NTRIP, self._write_rtcm)

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def stop(self):
        self.stop_evt.set()
        self.ntrip.stop()

    def _set(self, **kw):
        with self.lock:
            self.state.update(kw)

    def _write_rtcm(self, data):
        with self.ser_lock:
            if self.ser:
                try:
                    self.ser.write(data)
                except (serial.SerialException, OSError):
                    pass

    def run(self):
        self.ntrip.start()
        while not self.stop_evt.is_set():
            port = find_gps_port()
            if not port:
                self._set(conn="search", port="", pvt=None, gga=None)
                self.stop_evt.wait(1.0)
                continue
            self._set(conn="opening", port=port.device, err="")
            try:
                ser = serial.Serial(port.device, SERIAL_BAUD, timeout=0.05)
            except (serial.SerialException, OSError) as e:
                self._set(conn="busy", err=str(e))
                self.stop_evt.wait(2.0)
                continue
            with self.ser_lock:
                self.ser = ser
            try:
                self._session(ser)
            except (serial.SerialException, OSError) as e:
                self._set(conn="lost", err=str(e), pvt=None, gga=None)
            finally:
                with self.ser_lock:
                    self.ser = None
                try:
                    ser.close()
                except (serial.SerialException, OSError):
                    pass
            self.stop_evt.wait(1.0)

    def _session(self, ser):
        ok = apply_receiver_config(ser, self.ser_lock)
        self._set(conn="open", cfg_ok=ok)
        ser.reset_input_buffer()
        stream = F9PStream()
        while not self.stop_evt.is_set():
            ggas, pvts = stream.feed(ser.read(4096))
            for g in ggas:
                if g["q"] > 0:
                    self.ntrip.gga_raw = g["raw"]
            for p in pvts:
                if p["ok"] and p["fix"] in (2, 3, 4):
                    self.recorder.on_fix(p["lat"], p["lon"], p["speed"], gga_quality(p), p["hacc"])
            if ggas or pvts:
                with self.lock:
                    if ggas:
                        self.state["gga"] = ggas[-1]
                    if pvts:
                        self.state["pvt"] = pvts[-1]


# ── find_wc 연계 ──────────────────────────────────────────────────────────────────
LAUNCHER_HTML = """<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><title>find_wc 로 이동</title></head>
<body style="font-family:'Malgun Gothic',sans-serif">
<p>find_wc 에서 경로를 여는 중… <a id="go" href="#">자동으로 안 넘어가면 누르세요</a></p>
<script>
var url = __URL__;
document.getElementById("go").href = url;
location.replace(url);
</script>
</body></html>
"""


def open_in_find_wc(path):
    """CSV 를 find_wc 링크(#csvgz=…&name=…)로 연다."""
    payload = base64.urlsafe_b64encode(gzip.compress(path.read_bytes(), mtime=0)).rstrip(b"=").decode("ascii")
    url = f"{FIND_WC_URL}#csvgz={payload}&name={urllib.parse.quote(path.name, safe='')}"
    if len(url) <= 2000:
        webbrowser.open(url)
        return
    launcher = Path(tempfile.gettempdir()) / "gps_mapper_find_wc.html"
    launcher.write_text(LAUNCHER_HTML.replace("__URL__", json.dumps(url)), encoding="utf-8")
    webbrowser.open(launcher.as_uri())


def summarize_csv(path):
    """(점 수, 경로 길이 m) — 위경도 열이 없으면 길이는 None."""
    rows, length, prev = 0, 0.0, None
    try:
        with open(path, newline="", encoding="utf-8") as f:
            reader = csv.reader(f)
            header = [h.strip().lower() for h in next(reader, [])]
            ilat = header.index("latitude") if "latitude" in header else None
            ilon = header.index("longitude") if "longitude" in header else None
            for r in reader:
                if not r:
                    continue
                rows += 1
                if ilat is None or ilon is None:
                    continue
                try:
                    cur = (float(r[ilat]), float(r[ilon]))
                except (ValueError, IndexError):
                    continue
                if prev:
                    length += math.hypot(*delta_m(prev[0], prev[1], cur[0], cur[1]))
                prev = cur
    except OSError:
        return 0, None
    return rows, (length if ilat is not None and ilon is not None else None)


# ── 창 ────────────────────────────────────────────────────────────────────────────
def fmt_acc(m):
    return f"±{m * 100:.0f} cm" if m < 0.995 else f"±{m:.1f} m"


class App:
    def __init__(self, root, worker, recorder):
        self.root, self.worker, self.recorder = root, worker, recorder
        self.rate_hist = deque(maxlen=40)
        self.file_cache = {}
        self.last_note = ("", GRAY)

        root.title("GPS 매퍼 — gps_mapper.py")
        root.geometry("720x700")
        root.minsize(620, 560)
        style = ttk.Style(root)
        style.configure(".", font=(FONT, 10))
        style.configure("Treeview.Heading", font=(FONT, 10, "bold"))
        style.configure("Treeview", rowheight=26)

        top = ttk.Frame(root, padding=(14, 12, 14, 4))
        top.pack(fill="x")
        self.btn = tk.Button(top, text="매핑 시작", font=(FONT, 14, "bold"), width=12,
                             bg=GREEN, fg="white", activebackground=GREEN, activeforeground="white",
                             relief="flat", cursor="hand2", command=self.toggle_mapping)
        self.btn.grid(row=0, column=0, rowspan=2, sticky="w", padx=(0, 14))
        self.map_lbl = ttk.Label(top, text="대기 중", font=(FONT, 10), wraplength=500, justify="left")
        self.map_lbl.grid(row=0, column=1, sticky="w")

        sp = ttk.Frame(top)
        sp.grid(row=1, column=1, sticky="w", pady=(6, 0))
        ttk.Label(sp, text="기록 간격 :").pack(side="left")
        self.spacing_var = tk.StringVar(value=f"{SPACING_M:.2f}")
        self.spin = ttk.Spinbox(sp, from_=SPACING_MIN, to=SPACING_MAX, increment=0.05, width=6,
                                format="%.2f", textvariable=self.spacing_var, command=self.apply_spacing)
        self.spin.pack(side="left", padx=4)
        ttk.Label(sp, text="m").pack(side="left")
        for ev in ("<Return>", "<FocusOut>", "<KeyRelease>"):
            self.spin.bind(ev, lambda e: self.apply_spacing())
        self.spacing_msg = ttk.Label(sp, text="", foreground=RED)
        self.spacing_msg.pack(side="left", padx=8)

        st = ttk.Frame(root, padding=(14, 10, 14, 0))
        st.pack(fill="x")
        line = ttk.Frame(st)
        line.pack(fill="x")
        ttk.Label(line, text="상태 :", font=(FONT, 16, "bold")).pack(side="left")
        self.status_lbl = tk.Label(line, text="", font=(FONT, 18, "bold"))
        self.status_lbl.pack(side="left", padx=(8, 0))
        self.acc_lbl = tk.Label(line, text="", font=(FONT, 18, "bold"))
        self.acc_lbl.pack(side="left", padx=(14, 0))
        self.sub_lbl = tk.Label(st, text="", font=(FONT, 10), anchor="w", justify="left", wraplength=680)
        self.sub_lbl.pack(fill="x", pady=(2, 0))

        det = ttk.LabelFrame(root, text="상세", padding=(10, 6))
        det.pack(fill="x", padx=14, pady=(10, 0))
        self.detail = {}
        for r, key in enumerate(("GPS", "위성", "RTK", "위치")):
            ttk.Label(det, text=key, width=5, font=(FONT, 10, "bold")).grid(row=r, column=0, sticky="nw")
            lbl = ttk.Label(det, text="-", wraplength=600, justify="left")
            lbl.grid(row=r, column=1, sticky="w", pady=1)
            self.detail[key] = lbl

        data = ttk.LabelFrame(root, text="매핑 데이터 — gps_data", padding=(10, 6))
        data.pack(fill="both", expand=True, padx=14, pady=(10, 14))
        bar = ttk.Frame(data)
        bar.pack(fill="x")
        ttk.Label(bar, text="파일을 누르면 find_wc 에서 경로를 지도로 엽니다", foreground=GRAY).pack(side="left")
        ttk.Button(bar, text="폴더 열기", command=lambda: os.startfile(DATA_DIR)).pack(side="right")
        body = ttk.Frame(data)
        body.pack(fill="both", expand=True, pady=(6, 0))
        cols = ("pts", "len", "time")
        self.tree = ttk.Treeview(body, columns=cols, show="tree headings", selectmode="none")
        self.tree.heading("#0", text="파일", anchor="w")
        self.tree.heading("pts", text="점")
        self.tree.heading("len", text="길이")
        self.tree.heading("time", text="수정 시각")
        self.tree.column("#0", width=300, anchor="w")
        self.tree.column("pts", width=70, anchor="e")
        self.tree.column("len", width=90, anchor="e")
        self.tree.column("time", width=130, anchor="center")
        vsb = ttk.Scrollbar(body, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=vsb.set)
        self.tree.pack(side="left", fill="both", expand=True)
        vsb.pack(side="right", fill="y")
        self.tree.tag_configure("live", foreground=RED)
        self.tree.bind("<ButtonRelease-1>", self.on_tree_click)
        self.tree.configure(cursor="hand2")

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.tick()
        self.refresh_files()

    # ── 매핑 ─────────────────────────────────────────────────────────────────────
    def toggle_mapping(self):
        if self.recorder.active:
            path, rows, length = self.recorder.stop()
            if rows < 2:
                self.last_note = (f"⚠ {path.name} — 점이 {rows}개뿐이라 경로로 쓸 수 없음", RED)
            else:
                self.last_note = (f"저장됨: {path.name} · {rows}점 · {length:.1f} m", GREEN)
            self.btn.configure(text="매핑 시작", bg=GREEN, activebackground=GREEN)
        else:
            self.apply_spacing()
            try:
                self.recorder.start()
            except OSError as e:
                messagebox.showerror("매핑 시작 실패", f"CSV 파일을 만들 수 없습니다.\n{e}")
                return
            self.btn.configure(text="매핑 종료", bg=RED, activebackground=RED)
        self.refresh_files(force=True)

    def apply_spacing(self):
        try:
            v = float(self.spacing_var.get())
        except ValueError:
            v = None
        if v is None or not (SPACING_MIN <= v <= SPACING_MAX):
            self.spacing_msg.configure(text=f"{SPACING_MIN}~{SPACING_MAX} m 사이로 입력")
            return
        self.spacing_msg.configure(text="")
        self.recorder.spacing = v

    # ── 상태 표시 ────────────────────────────────────────────────────────────────
    def rtk_phase(self, now):
        nt = self.worker.ntrip
        if nt.status == "wait_pos":
            return "RTK 대기 — GPS 측위가 되면 접속 (VRS 는 현재 위치가 필요)"
        if nt.status == "connecting":
            return f"RTK 연결 시도 중… ({nt.attempts}번째)"
        if nt.status == "streaming":
            if nt.session_bytes == 0:
                return "RTK 접속됨 — 보정 데이터 대기"
            if now - nt.last_data_t > 3:
                return f"RTK 보정 끊김 ({now - nt.last_data_t:.0f}초째)"
            return "RTK 보정 수신 중"
        if nt.status == "retry":
            return f"RTK 연결 끊김 — {max(0, nt.retry_at - now):.0f}초 후 재접속"
        if nt.status == "fatal":
            return f"RTK 오류 — {max(0, nt.retry_at - now):.0f}초 후 다시 시도"
        return ""

    def describe(self, s, now):
        """(상태, 색, 오차, 부가 설명)"""
        conn, pvt = s["conn"], s["pvt"]
        if conn == "search":
            return "GPS 없음", GRAY, "", "u-blox GPS(USB)를 찾는 중 — 수신기를 USB 에 꽂아 주세요"
        if conn == "busy":
            return "GPS 사용 중", RED, "", f"{s['port']} 를 다른 프로그램이 쓰고 있음 (u-center, gpstest.py 등을 닫아 주세요)"
        if conn == "opening":
            return "GPS 연결 중", BLUE, "", f"{s['port']} 여는 중 · 수신기 설정 적용 중"
        if conn == "lost":
            return "GPS 연결 끊김", RED, "", "USB 가 빠졌거나 포트 오류 — 다시 찾는 중"
        if not pvt or now - pvt["t"] > PVT_STALE_SEC:
            return "GPS 데이터 없음", RED, "", f"{s['port']} 열림 · 수신기 응답 대기"
        rtk = self.rtk_phase(now)
        if not pvt["ok"] or pvt["fix"] == 0:
            return "위성 신호 없음", RED, "", f"측위 안 됨 (사용 위성 {pvt['sv']}개) — 하늘이 트인 곳에 안테나를 두세요 · {rtk}"
        acc = fmt_acc(pvt["hacc"])
        if pvt["carr"] == 2:
            return "RTK Fixed", GREEN, acc, f"cm급 측위 · {rtk}"
        if pvt["carr"] == 1:
            return "RTK Float", AMBER, acc, f"보정 적용 중, Fix 대기 (보통 수십 초) · {rtk}"
        if pvt["diff"]:
            return "DGNSS", ORANGE, acc, f"위성(SBAS) 또는 RTK 보정 적용 · m급 · {rtk}"
        return "단독 측위", ORANGE, acc, f"보정 없음 · m급 · {rtk}"

    def tick(self):
        now = time.time()
        s = self.worker.snapshot()
        nt = self.worker.ntrip
        status, color, acc, sub = self.describe(s, now)
        self.status_lbl.configure(text=status, fg=color)
        self.acc_lbl.configure(text=acc, fg=color)
        self.sub_lbl.configure(text=sub, fg=GRAY if status == "RTK Fixed" else color)

        # GPS
        if s["port"]:
            cfg = {True: "수신기 설정 적용됨", False: "⚠ 수신기 설정 실패 — Fix 가 늦을 수 있음",
                   None: "수신기 설정 전"}[s["cfg_ok"]]
            gps = f"{s['port']} · u-blox 수신기 자동 감지 · {cfg}"
            if s["conn"] in ("busy", "lost") and s["err"]:
                gps += f" · {s['err']}"
        else:
            gps = "연결된 u-blox 수신기 없음"
        self.detail["GPS"].configure(text=gps)

        pvt = s["pvt"] if s["pvt"] and now - s["pvt"]["t"] <= PVT_STALE_SEC else None
        if pvt and pvt["ok"]:
            sats = f"사용 {pvt['sv']}개 · PDOP {pvt['pdop']:.1f} · 수직 오차 {fmt_acc(pvt['vacc'])}"
        else:
            sats = f"사용 {pvt['sv']}개" if pvt else "-"
        self.detail["위성"].configure(text=sats)
        self.detail["위치"].configure(
            text=(f"{pvt['lat']:.8f}, {pvt['lon']:.8f} · 해발 {pvt['hmsl']:.2f} m · 속도 {pvt['speed']:.2f} m/s"
                  if pvt and pvt["ok"] else "-"))

        # RTK — 수신 속도는 최근 몇 초의 바이트 증가로 낸다
        self.rate_hist.append((now, nt.bytes_in))
        t0, b0 = self.rate_hist[0]
        rate = (nt.bytes_in - b0) / (now - t0) if now - t0 > 0.5 else 0.0
        rtk = f"{NTRIP['host']}:{NTRIP['port']}/{NTRIP['mountpoint']} (ID {NTRIP['user']}) · {self.rtk_phase(now)}"
        if nt.status == "streaming" and nt.session_bytes:
            rtk += f" · {rate / 1000:.2f} KB/s"
        gga = s["gga"]
        if gga and gga["age"]:
            rtk += f" · 보정 나이 {gga['age']} s"
        if nt.status in ("retry", "fatal") and nt.detail:
            rtk += f" · 원인: {nt.detail}"
        self.detail["RTK"].configure(text=rtk)

        # 매핑 진행
        rec = self.recorder
        if rec.active:
            el = int(now - rec.started)
            msg = f"● 기록 중  {rec.path.name} · {rec.rows}점 · {rec.length:.1f} m · {el // 60:02d}:{el % 60:02d}"
            if rec.rows == 0:
                msg += "  (측위 대기 — 아직 기록된 점 없음)"
            self.map_lbl.configure(text=msg, foreground=RED)
        else:
            text, col = self.last_note
            self.map_lbl.configure(text=text or "대기 중", foreground=col)

        self.root.after(200, self.tick)

    # ── 파일 목록 ────────────────────────────────────────────────────────────────
    def refresh_files(self, force=False):
        try:
            files = sorted(DATA_DIR.glob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
        except OSError:
            files = []
        live = self.recorder.path if self.recorder.active else None
        names = [p.name for p in files]
        for iid in self.tree.get_children():
            if iid not in names:
                self.tree.delete(iid)
        for idx, p in enumerate(files):
            try:
                stt = p.stat()
            except OSError:
                continue
            key = (stt.st_mtime_ns, stt.st_size)
            cached = self.file_cache.get(p.name)
            if force or not cached or cached[0] != key:
                rows, length = summarize_csv(p)
                cached = (key, rows, length)
                self.file_cache[p.name] = cached
            _, rows, length = cached
            values = (f"{rows}", "-" if length is None else f"{length:.1f} m",
                      datetime.fromtimestamp(stt.st_mtime).strftime("%m-%d %H:%M:%S"))
            tags = ("live",) if live and p == live else ()
            text = p.name + ("  ●" if tags else "")
            if self.tree.exists(p.name):
                self.tree.item(p.name, text=text, values=values, tags=tags)
                if self.tree.index(p.name) != idx:
                    self.tree.move(p.name, "", idx)
            else:
                self.tree.insert("", idx, iid=p.name, text=text, values=values, tags=tags)
        if not force:
            self.root.after(2000, self.refresh_files)

    def on_tree_click(self, event):
        iid = self.tree.identify_row(event.y)
        if not iid or self.tree.identify_region(event.x, event.y) not in ("tree", "cell"):
            return
        path = DATA_DIR / iid
        rows, _ = summarize_csv(path)
        if rows < 2:
            messagebox.showinfo("find_wc", f"{iid}\n점이 {rows}개라 경로를 그릴 수 없습니다.")
            return
        try:
            open_in_find_wc(path)
        except OSError as e:
            messagebox.showerror("find_wc 열기 실패", str(e))

    def on_close(self):
        if self.recorder.active:
            self.recorder.stop()
        self.worker.stop()
        self.root.after(150, self.root.destroy)


def main():
    if sys.platform == "win32":
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(1)   # 고해상도 화면에서 글자가 번지지 않게
        except (AttributeError, OSError):
            pass
    DATA_DIR.mkdir(exist_ok=True)
    recorder = RouteRecorder()
    worker = GpsWorker(recorder)
    worker.start()
    root = tk.Tk()
    App(root, worker, recorder)
    root.mainloop()


if __name__ == "__main__":
    main()
