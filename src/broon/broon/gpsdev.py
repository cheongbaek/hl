#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gpsdev.py ― GPS(u-blox ZED-F9P) 공용 모듈 : 수신기 식별 · 전/후방 배치 설정 · UBX 해석
════════════════════════════════════════════════════════════════════════════════
broon 의 gps(노드) · gps_setting(GUI) · check(점검)와 white1 · white2 가 같이 쓴다.

수신기 식별 : F9P 두 대는 USB 상으로 똑같이 보인다(시리얼 번호 없음, ttyACM 번호는 꽂은 순서).
  그래서 포트를 열어 ★칩 고유 ID (UBX-SEC-UNIQID 5바이트, 16진수)★ 를 읽어 가른다 (tool/info.md).

배치 설정 파일 gps_setting.json : 전방 · 후방에 어느 ID 의 수신기를 달았는지.
    {"front": {"id": "54401a3a98", "name": "…"}, "rear": {"id": "55408a3e9f", "name": "…"}}
  ros2 run broon gps_setting (GUI)이 쓰고, gps 노드 · check 가 읽는다. 찾는 순서(config_path):
    1) 환경변수 BROON_GPS_SETTING
    2) ★소스 트리★ <ws>/src/broon/config/gps_setting.json  — 리포에 올라가는 정본
    3) ~/.config/broon/gps_setting.json
  파일이 없으면 DEFAULT_SETTING(1번 GPS = 전방, 2번 GPS = 후방 — tool/info.md)을 쓴다.

UBX 해석 · 수신기 RAM 설정 · NMEA/UBX 분리(F9PStream)는 tool/gps_mapper.py → white/gps.py 의 것 그대로다.
"""

import errno
import json
import os
import struct
import subprocess
import time
from collections import namedtuple

import serial
import serial.tools.list_ports

PKG = "broon"
UBLOX_VID = 0x1546
GPS_BAUD = 38400             # USB 가상 시리얼(CDC-ACM)이라 실제 속도와 무관
PVT_STALE_SEC = 2.0          # NAV-PVT 가 이보다 오래 안 오면 'GPS 데이터 없음'
ID_TIMEOUT_S = 4.0           # 칩 ID 를 읽는 데 쓸 최대 시간 (공장 설정의 NMEA 폭주 속에서도 읽히게)

ROLES = ("front", "rear")
ROLE_KO = {"front": "전방", "rear": "후방"}
SETTING_NAME = "gps_setting.json"
SETTING_ENV = "BROON_GPS_SETTING"
# tool/info.md — 1번 GPS(HPG 1.32) = 전방, 2번 GPS(HPG 1.13) = 후방
DEFAULT_SETTING = {
    "front": {"id": "54401a3a98", "name": "U-Blox AG u-blox GNSS receiver"},
    "rear": {"id": "55408a3e9f", "name": "U-Blox AG u-blox GNSS receiver"},
}

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
GPS_PVT_REAR_TOPIC = '/gps_pvt_rear'

# ── /gps_dual 배열 규약 (듀얼 — 같은 측위 시각 iTOW 끼리 짝지은 한 줄) ★순서를 바꾸면 white2 가 어긋난다★ ──
#  [0] itow_s
#  [1] f_lat  [2] f_lon  [3] f_quality  [4] f_hacc_m  [5] f_speed_mps  [6] f_hmsl_m      ← 전방
#  [7] r_lat  [8] r_lon  [9] r_quality  [10] r_hacc_m [11] r_speed_mps [12] r_hmsl_m     ← 후방
#  [13] baseline_m (두 안테나 수평 거리 — 둘 다 측위됐을 때만, 아니면 NaN)
#  한쪽이 그 시각에 측위를 못 냈으면 그쪽 quality = 0, 나머지는 NaN.
GPS_DUAL_TOPIC = '/gps_dual'
GPS_DUAL_FIELDS = 14

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
#  배치 설정 파일
# ══════════════════════════════════════════════════════════════════════════════
def source_root():
    """<ws>/src/broon 경로. 못 찾으면 ''. (복사설치여도 찾는다 — white/paths.py 와 같은 방법)"""
    here = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    if os.path.isfile(os.path.join(here, "package.xml")):
        return here
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        cand = os.path.join(d, "src", PKG)
        if os.path.isfile(os.path.join(cand, "package.xml")):
            return cand
        parent = os.path.dirname(d)
        if parent == d:
            return ""
        d = parent


def config_path():
    env = os.environ.get(SETTING_ENV, "")
    if env:
        return os.path.expanduser(env)
    root = source_root()
    if root:
        return os.path.join(root, "config", SETTING_NAME)
    return os.path.expanduser(os.path.join("~", ".config", PKG, SETTING_NAME))


def _clean(setting):
    out = {}
    for role in ROLES:
        e = (setting or {}).get(role) or {}
        out[role] = {"id": str(e.get("id") or "").strip().lower(), "name": str(e.get("name") or "")}
    return out


def load_setting(path=None):
    """→ {'front': {'id', 'name'}, 'rear': {…}}. 파일이 없거나 못 읽으면 DEFAULT_SETTING."""
    path = path or config_path()
    try:
        with open(path, encoding="utf-8") as f:
            return _clean(json.load(f))
    except (OSError, ValueError):
        return _clean(DEFAULT_SETTING)


def save_setting(setting, path=None):
    """임시 파일에 쓴 뒤 바꿔치기한다 — 쓰다 죽어도 옛 파일이 남는다. 저장한 경로를 돌려준다."""
    path = path or config_path()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    data = {"_comment": "전방 · 후방 GPS 배치 (칩 고유 ID). ros2 run broon gps_setting 으로 고친다."}
    data.update(_clean(setting))
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")
    os.replace(tmp, path)
    return path


# ══════════════════════════════════════════════════════════════════════════════
#  포트 찾기 · 수신기 식별
# ══════════════════════════════════════════════════════════════════════════════
PortInfo = namedtuple("PortInfo", "device short name vid pid")


def lsusb_names():
    """{(vid, pid): 'lsusb 가 보이는 이름'} — lsusb 가 없으면(Windows 등) 빈 dict."""
    out = {}
    try:
        text = subprocess.run(["lsusb"], capture_output=True, text=True, timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return out
    for line in text.splitlines():
        head, sep, rest = line.partition(" ID ")
        if not sep:
            continue
        ids, _, name = rest.partition(" ")
        try:
            vid, pid = (int(x, 16) for x in ids.split(":"))
        except ValueError:
            continue
        out[(vid, pid)] = name.strip()
    return out


def _sort_key(dev):
    i = len(dev)
    while i > 0 and dev[i - 1].isdigit():
        i -= 1
    return (dev[:i], int(dev[i:]) if i < len(dev) else -1)


def list_ublox_ports():
    """꽂혀 있는 u-blox 시리얼 포트 [PortInfo] (번호순). 이름은 lsusb 에 나오는 그대로."""
    names = None
    out = []
    for p in serial.tools.list_ports.comports():
        if p.vid != UBLOX_VID:
            continue
        if names is None:
            names = lsusb_names()
        name = names.get((p.vid, p.pid)) or " ".join(x for x in (p.manufacturer, p.product) if x) or "u-blox"
        out.append(PortInfo(p.device, os.path.basename(p.device), name, p.vid, p.pid))
    out.sort(key=lambda t: _sort_key(t.device))
    return out


def open_port(dev, baud=GPS_BAUD, timeout=0.05):
    """→ (Serial, '') 또는 (None, 까닭). Linux 는 exclusive(flock)로 연다 — 다른 프로그램이 쥐고 있으면 실패."""
    kw = {} if os.name == "nt" else {"exclusive": True}
    try:
        return serial.Serial(dev, baud, timeout=timeout, **kw), ""
    except (serial.SerialException, OSError) as e:
        text = str(e)
        if getattr(e, "errno", None) == errno.EACCES or "Permission denied" in text:
            return None, "권한 없음 (dialout 그룹?)"
        if "lock" in text.lower() or getattr(e, "errno", None) in (errno.EAGAIN, errno.EBUSY):
            return None, "사용 중 (다른 프로그램이 열고 있다)"
        return None, f"열기 실패: {text}"


def poll_ubx(ser, cls, id_, timeout, sink=None):
    """UBX 조회(poll)를 보내고 응답 payload 를 돌려준다(설정은 바꾸지 않는다). 못 받으면 None.
    공장 설정이면 NMEA 가 쏟아지므로 0.5 초마다 다시 묻는다. 그동안 읽은 바이트는 sink(list)에 모은다."""
    want = b"\xb5\x62" + bytes([cls, id_])
    buf, t0, last_ask = b"", time.time(), 0.0
    while time.time() - t0 < timeout:
        if time.time() - last_ask > 0.5:
            ser.write(ubx(cls, id_))
            last_ask = time.time()
        data = ser.read(4096)
        if sink is not None:
            sink.append(data)
        buf += data
        i = buf.find(want)
        while i >= 0:
            if len(buf) < i + 6:
                break
            n = int.from_bytes(buf[i + 4:i + 6], "little")
            if len(buf) < i + 8 + n:
                break
            frame = buf[i:i + 8 + n]
            if ubx(cls, id_, frame[6:6 + n]) == frame:
                return frame[6:6 + n]
            i = buf.find(want, i + 2)
        buf = buf[-2048:]
    return None


def read_chip_id(ser, timeout=ID_TIMEOUT_S, sink=None):
    """칩 고유 ID (16진수 10자, 소문자). 못 읽으면 ''."""
    uid = poll_ubx(ser, 0x27, 0x03, timeout, sink)
    return uid[4:9].hex() if uid and len(uid) >= 9 else ""


def read_firmware(ser, timeout=2.0, sink=None):
    """펌웨어 문자열 'HPG 1.32'. 못 읽으면 ''."""
    ver = poll_ubx(ser, 0x0A, 0x04, timeout, sink)
    if ver:
        for k in range(40, len(ver), 30):
            s = ver[k:k + 30].split(b"\0")[0].decode("ascii", "replace")
            if s.startswith("FWVER="):
                return s[6:]
    return ""
