#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gps_mapper.py ― GPS 궤적 매핑 · 고정점 검증 (Windows 11 · Ubuntu · tkinter)
════════════════════════════════════════════════════════════════════════════════
USB 로 꽂힌 u-blox ZED-F9P 를 자동으로 찾아 접속하고, 국토지리정보원(NGII) NTRIP VRS 로
RTK 보정을 넣으면서 위치를 CSV 로 남긴다. 창 오른쪽 위에서 모드를 고른다(열면 매핑 모드).
기록 중에는 모드를 바꿀 수 없다.

  매핑 모드 : 사람이 몰고 다닌 궤적을 ★일정 거리마다★ 기록한다.
              → gps_data/route_YYYYmmdd_HHMMSS.csv
  검증 모드 : 수신기를 ★한 자리에 고정해 둔 채★ ★일정 시간마다★ 기록한다(예전 gps_check.py) —
              같은 자리에 두어도 위치가 옮겨 가는(오프셋) 증상이 규칙을 띠는지 보려고 데이터를
              모으는 용도. → gps_check_data/point_YYYYmmdd_HHMMSS_<간격>s.csv
              (매핑 경로 목록과 섞이지 않게 폴더를 나눴다)

CSV 형식 : 두 모드가 같다. 앞 15열은 white1/mapping.py 와 ★열 이름·순서가 같다★ (driving·find_wc 호환).
  값이 있는 열은 latitude · longitude · speed 뿐이고 나머지는 전부 0 이다
  (조향·엔코더·페달 같은 차량 신호가 없는 단독 매핑 도구이므로).
  ★끝에 2열을 더 붙인다★ — find_wc 가 이 열을 보고 구간별 RTK 품질을 색으로 그린다.
    gps_quality : NMEA GGA 품질 번호 (0 없음 · 1 단독 · 2 DGNSS · 4 RTK Fixed · 5 RTK Float)
    gps_sigma_m : 수신기 수평 정확도 추정 hAcc [m]
  둘 다 위치와 ★같은 NAV-PVT★ 에서 뽑으므로 점마다 시점이 정확히 맞는다.
  검증 모드 파일은 고정점이라 지도에서 한 점 뭉치로 보인다. 시각 열은 없다 — 파일 이름의 시작
  시각과 간격으로 역산하되, 측위가 안 된 동안은 기록을 건너뛰므로 그 뒤 줄부터는 행 번호와
  시각이 어긋난다.

기록 규칙 — 두 모드 모두 측위가 안 된(위성 없음) 동안은 기록하지 않는다.
  매핑 : ★일정 거리마다 한 점★ (시간 간격이 아니라) — 기본 0.25 m, 창에서 조절.
    신호 대기처럼 멈춰 있는 동안 같은 자리가 수백 줄 쌓이면 경로가 아니라 점 뭉치가 되기
    때문이다(mapping.py 와 같은 규칙). 매핑 중에도 간격을 바꿀 수 있다.
  검증 : ★일정 시간마다 한 점★ — 기본 0.1 초(= 출력 주기 그대로, 모든 측위를 남긴다), 0.1~60 초, 0.1 초 단위.
    수신기가 NAV-PVT 를 10 Hz 로 내므로 0.1 초가 최소 단위다. 간격은 PC 시계가 아니라 NAV-PVT 의
    iTOW(수신기 GPS 시각, ms)로 잰다 — USB 로 들어오는 시점이 들쭉날쭉해도 기록 간격은 흔들리지
    않는다. 기록 중에는 간격을 바꿀 수 없다(한 파일 안에서 간격이 섞이면 시각 열이 없어 나중에
    구분할 방법이 없으므로).

RTK : 창의 체크박스로 켜고 끈다(기본 켬). 끄면 NTRIP 접속을 끊어 보정을 더 넣지 않는다. 수신기는
  이미 받은 보정을 만료될 때까지 쓰므로 RTK Fixed/Float 가 곧바로 풀리지는 않는다 —
  gps_quality 열에는 수신기가 실제로 낸 품질이 그대로 남는다. 기록 중에도 바꿀 수 있다.

GPS 자동 감지 : u-blox USB VID(0x1546)인 시리얼 포트만 고른다(Windows COM*, Ubuntu /dev/tty*).
  포트 번호가 매번 달라도 된다. ★다른 포트는 열지 않는다★ — 아두이노는 포트를 여는 순간
  리셋되므로, 아무 포트나 열어 보고 확인하는 방식은 쓰지 않는다. USB 가 빠지면 다시 찾아
  자동으로 붙는다.

수신기 설정 : 접속할 때마다 ★RAM 에만★ 쓴다(전원을 뺐다 꽂으면 공장 설정으로 돌아간다).
  공장 설정은 NMEA 6종을 USB·UART1(38400bps)·I2C 로 똑같이 10 Hz 출력해 출력 버퍼가
  넘치고(`$GNTXT txbuf alloc`), 그러면 RTCM 보정을 띄엄띄엄 써서 Fix 가 안 된다
  (gpstest.py 끝 주석에 측정 근거). 그래서 UART1·I2C 의 NMEA 를 끄고 USB 는 GGA·RMC 만 둔다.
  여기에 ★NAV-PVT 10 Hz★ 를 켠다 — 위치(1e-7° ≈ 1.1 cm), 지면속도, hAcc, RTK 상태를
  한 메시지로 준다(GGA 위도는 1.9 cm 단위로 끊긴다). GGA 는 VRS 캐스터에 현재 위치를
  올리는 데만 쓴다.

기지국 : 받은 RTCM 을 수신기에 넣으면서 따로 엿봐 창의 '기지국' 줄에 보인다(RtcmMonitor).
  · 1005/1006 — 기준국 ID · 위치. VRS 는 '계산된 기준국'으로 표시돼 오고(DF141), 접속할 때마다 ID 가 바뀌며
    올린 GGA 자리에 생긴다 — 그래서 '여기서 몇 m'가 0 에 가깝다.
  · 1032 — 그 VRS 를 만든 ★실제 기준국★ ID · 위치. 국토지리정보원 위성기준점 표(NGII_STATIONS)와 맞춰
    이름을 붙인다(2026-10-13 충주 대소원면: ID 10 = GSAN 괴산, 18.5 km).
  · 1033 — 기준국 수신기 · 펌웨어 · 일련번호, 1014 — 네트워크 주 · 보조 기준국(오면).
  · MSM 메시지 번호(107x GPS, 108x GLONASS, 109x Galileo, 111x QZSS, 112x BeiDou)로 보정에 든 위성계를 안다.
  해독이 실패해도 보정은 이미 수신기로 간 뒤라 RTK 에는 영향이 없다.

경로 확인 : 목록에서 CSV 를 누르면 find_wc(카카오 위성지도)가 그 기록을 바로 그린다.
  CSV 를 gzip → base64url 로 URL 프래그먼트(#csvgz=…&name=…)에 싣는다 — find_wc 의
  src/lib/inbound.ts 가 받는 형식. 프래그먼트는 서버로 전송되지 않으므로 좌표가 GitHub
  Pages 로그에 남지 않는다. 주소가 너무 길면 임시 HTML 을 열어 그 안에서 location.replace 로
  넘어간다(길이 기준과 HTML 위치는 아래 OS 분기 참고).

Windows · Ubuntu : 같은 파일이 두 OS 에서 돈다. OS 에 따라 갈리는 값은 IS_WINDOWS 분기 한곳에 모았다.
  · 시리얼 포트 — Windows 는 COM*, Ubuntu 는 /dev/tty* (ZED-F9P 는 USB CDC-ACM 이라 /dev/ttyACM*).
  · 포트 독점 — Windows 는 OS 가 COM 포트를 한 프로그램에만 열어 준다. Linux 의 tty 는 여러
    프로그램이 동시에 열 수 있어 데이터가 나뉘어 들어가므로 exclusive(flock)로 연다.
  · Ubuntu 준비 — /dev/ttyACM* 는 dialout 그룹만 열 수 있다. 처음 한 번
    `sudo usermod -aG dialout $USER` 후 다시 로그인. 패키지: `sudo apt install python3-tk python3-serial`.
  · find_wc 주소 — Windows 는 명령줄 길이 제한 때문에 2000자를 넘으면 임시 폴더의 HTML 을 거친다.
    Ubuntu 는 한 인자를 128 KiB 까지 넘길 수 있어 웬만한 기록은 주소를 바로 열고, 그보다 길 때만
    HTML 을 이 파일 옆에 만든다 — Ubuntu 기본 Firefox(snap)는 /tmp 를 따로 써서 임시 폴더의
    파일을 못 연다.
  · 글꼴 — 맑은 고딕이 없으면(Ubuntu) Noto Sans CJK KR · 나눔고딕 순으로 찾는다.
  · 폴더 열기 — Windows 는 탐색기(os.startfile), Ubuntu 는 xdg-open.
"""

import base64
import csv
import ctypes
import errno
import gzip
import json
import math
import os
import re
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
import urllib.parse
import webbrowser
from collections import deque
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk

import serial
import serial.tools.list_ports


APP_DIR = Path(__file__).resolve().parent
MAP_DIR = APP_DIR / "gps_data"            # 매핑 모드
CHECK_DIR = APP_DIR / "gps_check_data"    # 검증 모드
FIND_WC_URL = "https://cheongbaek.github.io/find_wc/"

# ── OS 분기 ───────────────────────────────────────────────────────────────────────
IS_WINDOWS = sys.platform == "win32"
if IS_WINDOWS:
    SERIAL_OPEN_KW = {}                        # COM 포트는 OS 가 한 프로그램에만 열어 준다
    URL_DIRECT_MAX = 2000                      # 넘으면 Windows 명령줄 길이 제한에 걸린다
    LAUNCHER_DIR = Path(tempfile.gettempdir())
else:
    SERIAL_OPEN_KW = {"exclusive": True}       # tty 는 여럿이 동시에 열 수 있어 flock 으로 막는다
    URL_DIRECT_MAX = 100_000                   # 한 인자 한도 128 KiB(MAX_ARG_STRLEN) 안쪽
    LAUNCHER_DIR = APP_DIR                     # snap Firefox 는 /tmp 를 따로 써서 못 연다


def is_serial_device(dev):
    """이 OS 의 시리얼 포트 이름인가 — Windows 는 COM*, Ubuntu 는 /dev/tty*."""
    if IS_WINDOWS:
        return dev.upper().startswith("COM")
    return dev.startswith("/dev/tty")


NTRIP = {
    "host": "rts1.ngii.go.kr",
    "port": 2101,
    "mountpoint": "VRS-RTCM34",
    "user": "knut_gps",
    "password": "ngii",
}

# 기지국 정보 (RtcmMonitor)
RTCM_RECENT_SEC = 15         # 이보다 오래전에 마지막으로 받은 메시지는 '지금 받는 중'에서 뺀다
NGII_MATCH_M = 50            # 1032 의 실제 기준국이 이 거리 안이면 그 위성기준점으로 본다
# 국토지리정보원 위성기준점 92곳 (코드, 위도, 경도). ITRF2020 성과(KGRF20_NGII.CRD, 2025-02 공개)의 지구중심 좌표를
# GRS80 위경도로 바꾼 값 — 1032 로 받은 실제 기준국에 이름을 붙이는 데만 쓴다. 코드의 한글 이름은 성과에 없다.
NGII_STATIONS = (
    ("ANSG", 37.01215, 127.26579), ("BOEN", 36.48837, 127.73027), ("BONH", 36.94401, 128.91452), ("BSNG", 34.89295, 127.13498),
    ("CCSB", 37.94757, 127.75493), ("CHEN", 36.87801, 127.15526), ("CHJU", 33.51393, 126.52982), ("CHLW", 38.16448, 127.41525),
    ("CHNG", 35.53331, 128.47806), ("CHSG", 36.43563, 129.05635), ("CHUL", 38.27327, 127.14522), ("CHWN", 35.23612, 128.69300),
    ("CHYG", 36.45890, 126.80150), ("CNJU", 36.62689, 127.46123), ("DANJ", 36.89242, 126.82305), ("DOND", 37.90205, 127.06074),
    ("DONH", 37.50698, 129.12421), ("GANH", 37.71938, 126.39024), ("GHDG", 35.27054, 128.98509), ("GOCH", 35.66790, 127.94334),
    ("GOHG", 34.56579, 127.25801), ("GOJE", 34.72179, 128.59087), ("GOSG", 34.98066, 128.30771), ("GSAN", 36.81616, 127.78679),
    ("GYJU", 35.75545, 129.03908), ("GYOI", 35.67360, 129.28862), ("HADG", 35.16242, 127.70924), ("HCHN", 38.30809, 127.67955),
    ("HCSN", 38.06291, 127.52425), ("HONC", 37.70916, 128.19433), ("ICHN", 37.11740, 127.61304), ("IMJA", 35.08702, 126.11996),
    ("INCH", 37.41986, 126.68644), ("INJE", 38.06946, 128.17080), ("JAHG", 34.67535, 126.89958), ("JANS", 35.64163, 127.52224),
    ("JECH", 36.90977, 128.16592), ("JIND", 34.47199, 126.32374), ("JINJ", 35.17309, 128.04968), ("JSUN", 37.36938, 128.68464),
    ("JUNG", 35.62277, 126.97403), ("JUNJ", 35.84345, 127.13513), ("KANR", 37.77093, 128.86823), ("KIMC", 36.13740, 128.14230),
    ("KUNW", 36.23348, 128.57424), ("KUSN", 36.00547, 126.76155), ("KWNJ", 35.17837, 126.91024), ("MUAN", 34.81743, 126.46623),
    ("MUJU", 36.00329, 127.66118), ("MYSO", 35.50179, 128.81577), ("NAMH", 34.72388, 127.98167), ("NAMW", 35.42305, 127.39618),
    ("NAMY", 37.71999, 127.30004), ("NSAN", 36.19416, 127.11233), ("OJBU", 37.75487, 127.10667), ("PAJU", 37.74641, 126.73790),
    ("PCHN", 37.53467, 128.44605), ("PJMS", 37.88604, 126.76622), ("POCN", 38.01858, 127.19097), ("POHC", 35.91014, 129.40295),
    ("PSJA", 35.38219, 129.24874), ("PUSN", 35.23394, 129.07485), ("SEJN", 36.52221, 127.30319), ("SEOS", 36.77641, 126.49421),
    ("SIAN", 34.78689, 126.14815), ("SMAN", 35.73027, 126.52794), ("SNJU", 36.37916, 128.14448), ("SONC", 34.95748, 127.48614),
    ("SOUL", 37.61855, 127.09835), ("SSAN", 33.46780, 126.90472), ("SUNJ", 35.91408, 128.25613), ("SUWN", 37.27551, 127.05424),
    ("TABK", 37.16364, 128.98608), ("TEGN", 35.90631, 128.80197), ("WNDO", 34.33537, 126.74283), ("WNJU", 37.33721, 127.94709),
    ("WOLS", 35.50395, 129.41602), ("WSJG", 35.56568, 129.31705), ("WSSN", 35.54542, 129.11897), ("WULJ", 36.99198, 129.41300),
    ("YANJ", 37.69689, 126.93163), ("YANP", 37.45419, 127.50563), ("YCHG", 35.98087, 129.10643), ("YCIG", 36.05278, 129.04002),
    ("YCMP", 35.97294, 128.92693), ("YDKG", 36.36295, 129.39044), ("YECH", 36.65146, 128.44634), ("YEOJ", 37.27857, 127.57831),
    ("YODK", 36.53346, 129.40905), ("YONK", 35.27858, 126.51647), ("YOWL", 37.18252, 128.46182), ("YYAN", 38.10510, 128.64040),
)

UBLOX_VID = 0x1546
SERIAL_BAUD = 38400          # USB 가상 시리얼(CDC-ACM)이라 실제 속도와 무관
SPACING_M = 0.25             # 매핑: 이 거리마다 한 점 기록 (mapping.py 와 같은 기본값)
SPACING_MIN, SPACING_MAX = 0.05, 10.0
INTERVAL_S = 0.1             # 검증: 이 시간마다 한 점 기록 — 0.1 초 = NAV-PVT 를 하나도 버리지 않는다
INTERVAL_MIN, INTERVAL_MAX = 0.1, 60.0
EPOCH_MS = 100               # NAV-PVT 출력 주기 (CFG-RATE-MEAS 10 Hz)
WEEK_MS = 7 * 24 * 3600 * 1000   # iTOW 는 매주 일요일 0시(GPS 시각)에 0 으로 돌아간다
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
SEG_OFF_BG, SEG_OFF_FG, SEG_EDGE = "#f1f3f5", "#495057", "#ced4da"   # 모드 선택에서 안 고른 쪽
FONT_CANDIDATES = ("Malgun Gothic", "Noto Sans CJK KR", "NanumGothic", "UnDotum")

# 모드별 창 문구와 기록 간격 입력 범위
MODES = {
    "map": dict(name="매핑 모드", start="매핑 시작", stop="매핑 종료", data="매핑 데이터",
                unit="m", lo=SPACING_MIN, hi=SPACING_MAX, step=0.05, fmt="%.2f",
                bad=f"{SPACING_MIN}~{SPACING_MAX} m 사이로 입력",
                hint="파일을 누르면 find_wc 에서 경로를 지도로 엽니다",
                cols=("pts", "len", "time"), name_w=300),
    "check": dict(name="검증 모드", start="기록 시작", stop="기록 중지", data="검증 데이터",
                  unit="초", lo=INTERVAL_MIN, hi=INTERVAL_MAX, step=0.1, fmt="%.1f",
                  bad=f"{INTERVAL_MIN}~{INTERVAL_MAX:.0f} 초, 0.1 초 단위로 입력",
                  hint="파일을 누르면 find_wc 에서 지도로 엽니다 · 퍼짐은 평균 위치 기준 RMS",
                  cols=("pts", "rms", "ext", "time"), name_w=270),
}
# 파일 목록 열: (ID, 제목, 기본 폭, 정렬) — 모드마다 MODES 의 cols 만 보인다
TREE_COLS = (("pts", "점", 60, "e"), ("len", "길이", 90, "e"), ("rms", "퍼짐", 80, "e"),
             ("ext", "범위 동서×남북", 130, "e"), ("time", "수정 시각", 120, "center"))


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
    itow = struct.unpack_from("<I", p, 0)[0]
    flags = p[21]
    lon, lat, height, hmsl, hacc, vacc = struct.unpack_from("<iiiiII", p, 24)
    gspeed = struct.unpack_from("<i", p, 60)[0]
    pdop = struct.unpack_from("<H", p, 76)[0]
    return dict(t=time.time(), itow=itow, fix=p[20], ok=bool(flags & 1), diff=bool(flags & 2),
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


def port_sort_key(dev):
    """COM3 이 COM10 보다, ttyACM2 가 ttyACM10 보다 앞에 오게 끝 번호를 숫자로 비교."""
    m = re.search(r"(\d+)$", dev)
    return (dev[:m.start()], int(m.group(1))) if m else (dev, -1)


def find_gps_port():
    """u-blox(VID 0x1546) 시리얼 포트. 다른 포트는 열어 보지 않는다(아두이노 리셋 방지)."""
    ports = sorted((p for p in serial.tools.list_ports.comports()
                    if p.vid == UBLOX_VID and is_serial_device(p.device)),
                   key=lambda p: port_sort_key(p.device))
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


# ── RTCM 기지국 정보 ──────────────────────────────────────────────────────────────
MSM_SYSTEM = {107: "GPS", 108: "GLONASS", 109: "Galileo", 110: "SBAS", 111: "QZSS", 112: "BeiDou", 113: "NavIC"}


def crc24q(data):
    """RTCM3 프레임 검사값 (CRC-24Q)."""
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def ecef_to_llh(x, y, z):
    """지구중심 좌표 [m] → (위도, 경도, 타원체고 m) — GRS80."""
    a, f = 6378137.0, 1 / 298.257222101
    e2 = f * (2 - f)
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1 - e2))
    h = 0.0
    for _ in range(6):
        n = a / math.sqrt(1 - e2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1 - e2 * n / (n + h)))
    return math.degrees(lat), math.degrees(lon), h


class RtcmMonitor:
    """NTRIP 으로 받은 RTCM3 를 엿봐 기지국 정보를 모은다. NtripClient 스레드가 feed, 창이 snapshot 을 부른다.
    보정은 이와 상관없이 먼저 수신기로 간다."""

    def __init__(self):
        self.lock = threading.Lock()
        self.buf = b""
        self.crc_bad = 0
        self.last = {}           # 메시지 번호 → 마지막으로 받은 시각
        self.station = None      # 1005/1006 : dict(id, lat, lon, h, virtual, ant_h, t)
        self.physical = {}       # 1032 : 실제 기준국 ID → dict(lat, lon, h, vrs, t)
        self.receiver = None     # 1033 : dict(id, antenna, rx, fw, sn)
        self.network = None      # 1014 : dict(net, sub, master, aux, t)

    def new_stream(self):
        """다시 접속하면 앞 접속의 끊긴 조각만 버린다 — 기준국 정보는 새 메시지가 덮는다(VRS ID 는 접속마다 바뀐다)."""
        with self.lock:
            self.buf = b""

    def snapshot(self):
        with self.lock:
            return dict(last=dict(self.last), station=self.station, physical=dict(self.physical),
                        receiver=self.receiver, network=self.network, crc_bad=self.crc_bad)

    def feed(self, data):
        with self.lock:
            b, i = self.buf + data, 0
            while True:
                j = b.find(b"\xd3", i)
                if j < 0:
                    i = len(b)
                    break
                if len(b) - j < 3:
                    i = j
                    break
                if b[j + 1] & 0xFC:      # 머리의 예약 6비트는 늘 0 — 본문 속 0xD3 를 길이로 믿고 기다리지 않게
                    i = j + 1
                    continue
                n = ((b[j + 1] & 0x03) << 8) | b[j + 2]
                if len(b) - j < n + 6:
                    i = j
                    break
                frame = b[j:j + n + 6]
                if crc24q(frame[:-3]) != int.from_bytes(frame[-3:], "big"):
                    self.crc_bad += 1     # 0xD3 는 본문에도 나온다 — 한 바이트 넘겨 다시 찾는다
                    i = j + 1
                    continue
                self._message(frame[3:3 + n])
                i = j + n + 6
            self.buf = b[i:]

    def _message(self, p):
        if len(p) < 3:
            return
        v, size = int.from_bytes(p, "big"), len(p) * 8

        def u(pos, k):
            return (v >> (size - pos - k)) & ((1 << k) - 1) if pos + k <= size else 0

        def sgn(pos, k):
            x = u(pos, k)
            return x - (1 << k) if x >> (k - 1) else x
        msg, sid, now = u(0, 12), u(12, 12), time.time()
        self.last[msg] = now
        if msg in (1005, 1006) and size >= 152:
            lat, lon, h = ecef_to_llh(sgn(34, 38) * 1e-4, sgn(74, 38) * 1e-4, sgn(114, 38) * 1e-4)
            self.station = dict(id=sid, lat=lat, lon=lon, h=h, virtual=bool(u(33, 1)), t=now,
                                ant_h=u(152, 16) * 1e-4 if msg == 1006 and size >= 168 else None)
        elif msg == 1032 and size >= 156:
            lat, lon, h = ecef_to_llh(sgn(42, 38) * 1e-4, sgn(80, 38) * 1e-4, sgn(118, 38) * 1e-4)
            self.physical[u(24, 12)] = dict(lat=lat, lon=lon, h=h, vrs=sid, t=now)
        elif msg == 1033:
            texts, pos = [], 24
            for k in range(5):     # 안테나 이름 · (설정 ID) · 안테나 일련번호 · 수신기 · 펌웨어 · 수신기 일련번호
                c = u(pos, 8)
                pos += 8
                texts.append(bytes(u(pos + 8 * q, 8) for q in range(c)).decode("ascii", "replace").strip())
                pos += 8 * c + (8 if k == 0 else 0)
            self.receiver = dict(id=sid, antenna=texts[0], rx=texts[2], fw=texts[3], sn=texts[4])
        elif msg == 1014 and size >= 53:
            self.network = dict(net=u(12, 8), sub=u(20, 4), master=u(29, 12), aux=u(41, 12), t=now)


def nearest_ngii(lat, lon):
    """가장 가까운 NGII 위성기준점 (코드, 거리 m)."""
    d, code = min((math.hypot(*delta_m(lat, lon, a, b)), c) for c, a, b in NGII_STATIONS)
    return code, d


def fmt_dist(m):
    return f"{m:.0f} m" if m < 1000 else f"{m / 1000:.1f} km"


def describe_base(rt, pos, now):
    """창의 '기지국' 줄. rt 는 RtcmMonitor.snapshot(), pos 는 지금 위치 (위도, 경도) 또는 None."""
    recent = sorted(m for m, t in rt["last"].items() if now - t <= RTCM_RECENT_SEC)
    if not recent:
        return "받는 RTCM 없음"
    parts = []
    st = rt["station"]
    if st:
        line = f"{'VRS 가상 기준국' if st['virtual'] else '기준국'} ID {st['id']}"
        if pos:
            line += f" (여기서 {fmt_dist(math.hypot(*delta_m(pos[0], pos[1], st['lat'], st['lon']))) })"
        if now - st["t"] > 4 * RTCM_RECENT_SEC:
            line += f" ({now - st['t']:.0f}초 전)"
        parts.append(line)
    for pid, ph in sorted(rt["physical"].items()):
        if now - ph["t"] > 8 * RTCM_RECENT_SEC:
            continue                       # 앞 접속에서 쓰던 기준국
        code, d = nearest_ngii(ph["lat"], ph["lon"])
        line = f"실제 기준국 ID {pid}" + (f" = NGII 위성기준점 {code}" if d <= NGII_MATCH_M
                                        else f" ({ph['lat']:.5f}, {ph['lon']:.5f})")
        if pos:
            line += f" · {fmt_dist(math.hypot(*delta_m(pos[0], pos[1], ph['lat'], ph['lon'])))}"
        parts.append(line)
    nw = rt["network"]
    if nw and now - nw["t"] <= 8 * RTCM_RECENT_SEC:
        parts.append(f"네트워크 주 기준국 {nw['master']} · 보조 {nw['aux']}")
    rx = rt["receiver"]
    if rx and rx["rx"]:
        parts.append(rx["rx"] + (f" #{rx['sn']}" if rx["sn"] else ""))
    systems = [MSM_SYSTEM[k] for k in sorted({m // 10 for m in recent if 1070 <= m <= 1139}) if k in MSM_SYSTEM]
    if systems:
        parts.append("보정 위성 " + "·".join(systems))
    parts.append("메시지 " + ",".join(str(m) for m in recent))
    return " · ".join(parts)


class NtripClient(threading.Thread):
    """NGII 캐스터 접속을 유지한다. 받은 RTCM3 는 sink(bytes) 로 넘긴다(시리얼 쓰기).

    status : off(체크 해제) / wait_pos(위치 대기) → connecting → streaming,
             실패하면 retry / fatal 후 다시 시도.
    set_enabled(False) 는 소켓을 바로 닫으므로 접속·수신 중이어도 곧장 끊긴다.
    """

    def __init__(self, cfg, sink):
        super().__init__(daemon=True)
        self.cfg, self.sink = cfg, sink
        self.sock = None
        self.gga_raw = None
        self.enabled = True
        self.status = "wait_pos"
        self.detail = ""
        self.attempts = 0
        self.retry_at = 0.0
        self.bytes_in = 0
        self.session_bytes = 0
        self.last_data_t = 0.0
        self.rtcm = RtcmMonitor()       # 받은 보정에서 기지국 정보를 엿본다
        self.stop_evt = threading.Event()
        self.wake = threading.Event()   # 켬/끔이 바뀌면 대기 중인 루프를 바로 깨운다

    def set_enabled(self, on):
        self.enabled = on
        if not on:
            self._close()
        self.wake.set()

    def stop(self):
        self.stop_evt.set()
        self.wake.set()
        self._close()

    def _sleep(self, sec):
        self.wake.wait(sec)
        self.wake.clear()

    def run(self):
        while not self.stop_evt.is_set():
            if not self.enabled:
                self.status, self.detail = "off", ""
                self._sleep(1.0)
                continue
            if not self.gga_raw:
                self.status = "wait_pos"
                self._sleep(0.5)
                continue
            self.attempts += 1
            self.status, self.detail = "connecting", ""
            wait = NTRIP_RETRY_SEC
            try:
                self._stream(self._connect())
            except NtripFatal as e:
                self.status, self.detail, wait = "fatal", str(e), NTRIP_FATAL_RETRY_SEC
            except OSError as e:
                self.status, self.detail = "retry", str(e) or type(e).__name__
            self._close()
            if self.stop_evt.is_set():
                return
            if not self.enabled:   # 체크를 해제해서 끊긴 것 — 재접속 대기 없이 off 로
                continue
            self.retry_at = time.time() + wait
            self._sleep(wait)

    def _connect(self):
        """접속해서 헤더까지 읽고 소켓을 돌려준다. 다른 스레드가 self.sock 을 닫을 수 있으므로
        여기서는 지역 변수 sock 만 쓴다(닫히면 OSError 로 빠져나간다)."""
        c = self.cfg
        auth = base64.b64encode(f"{c['user']}:{c['password']}".encode()).decode()
        req = (f"GET /{c['mountpoint']} HTTP/1.0\r\n"
               f"Host: {c['host']}\r\n"
               f"User-Agent: NTRIP gps_mapper/1.0\r\n"
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
        self.status, self.detail = "streaming", first
        self.session_bytes = 0
        self.rtcm.new_stream()
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
        self.session_bytes += len(data)
        self.last_data_t = time.time()
        self.sink(data)
        try:
            self.rtcm.feed(data)        # 기지국 정보 — 보정은 이미 수신기로 갔으니 여기서 실패해도 RTK 는 그대로
        except (IndexError, ValueError):
            pass

    def _close(self):
        s, self.sock = self.sock, None
        if s:
            try:
                s.close()
            except OSError:
                pass


# ── 퍼짐 통계 (검증 모드) ──────────────────────────────────────────────────────────
class Spread:
    """고정점 퍼짐. 첫 점 기준 동·북 거리 [m] 의 누적합만 들고 있어 점이 많아도 O(1) 로 갱신된다."""

    def __init__(self):
        self.count = 0
        self.origin = None
        self.se = self.sn = self.see = self.snn = 0.0
        self.box = None    # (동 최소, 동 최대, 북 최소, 북 최대)

    def add(self, lat, lon):
        if self.origin is None:
            self.origin = (lat, lon)
        e, n = delta_m(self.origin[0], self.origin[1], lat, lon)
        self.count += 1
        self.se += e
        self.sn += n
        self.see += e * e
        self.snn += n * n
        b = self.box
        self.box = (e, e, n, n) if b is None else (min(b[0], e), max(b[1], e), min(b[2], n), max(b[3], n))

    def rms(self):
        """평균 위치에서 각 점까지 거리의 RMS [m]. 점이 2개 미만이면 None."""
        if self.count < 2:
            return None
        c = self.count
        var = self.see / c - (self.se / c) ** 2 + self.snn / c - (self.sn / c) ** 2
        return math.sqrt(max(var, 0.0))

    def extent(self):
        """(동서 폭, 남북 폭) [m]. 점이 2개 미만이면 None."""
        if self.count < 2:
            return None
        b = self.box
        return b[1] - b[0], b[3] - b[2]


# ── CSV 기록 ──────────────────────────────────────────────────────────────────────
def csv_row(pvt):
    """NAV-PVT 한 개 → CSV_HEADER 순서의 한 줄."""
    return [
        f"{pvt['lat']:.8f}", f"{pvt['lon']:.8f}",
        "0.00",                  # heading
        f"{pvt['speed']:.3f}",   # speed — NAV-PVT 지면속도 [m/s]
        "0", "0", "0.00", "0",                    # steer / direction / pitch / terrain
        "0", "0.0", "0.000", "0", "0", "0", "0",  # 차량 실계측 열 — 없음
        gga_quality(pvt),        # gps_quality — NMEA GGA 품질 번호
        f"{pvt['hacc']:.3f}",    # gps_sigma_m — hAcc [m]
    ]


class Recorder:
    """기록 시작/종료와 한 줄 쓰기. GPS 스레드가 on_fix 를, 창이 start/stop 을 부른다.
    어느 점을 남길지는 모드별 하위 클래스의 on_fix 가 정한다."""

    def __init__(self, data_dir):
        self.data_dir = data_dir
        self.lock = threading.Lock()
        self.fp = self.writer = None
        self.path = None
        self.rows = 0
        self.started = 0.0

    @property
    def active(self):
        return self.fp is not None

    def file_name(self, stamp):
        raise NotImplementedError

    def reset(self):
        """새 파일을 열 때 모드별 누적값을 비운다."""

    def start(self):
        with self.lock:
            self.data_dir.mkdir(exist_ok=True)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.path = self.data_dir / self.file_name(stamp)
            self.fp = open(self.path, "w", newline="", encoding="utf-8")
            self.writer = csv.writer(self.fp)
            self.writer.writerow(CSV_HEADER)
            self.fp.flush()
            self.rows = 0
            self.reset()
            self.started = time.time()
            return self.path

    def stop(self):
        """(경로, 점 수, 경과 초)"""
        with self.lock:
            if self.fp:
                try:
                    self.fp.close()
                except OSError:
                    pass
            self.fp = self.writer = None
            return self.path, self.rows, time.time() - self.started

    def _write(self, pvt):
        """self.lock 을 쥔 채로 부른다."""
        self.writer.writerow(csv_row(pvt))
        self.fp.flush()      # 기록 중인 파일도 목록에서 바로 열어 볼 수 있게
        self.rows += 1


class RouteRecorder(Recorder):
    """매핑 모드 — 일정 거리마다 한 점."""

    def __init__(self, data_dir):
        super().__init__(data_dir)
        self.spacing = SPACING_M
        self.length = 0.0
        self.last = None    # 마지막으로 '기록한' 점

    def file_name(self, stamp):
        return f"route_{stamp}.csv"

    def reset(self):
        self.length, self.last = 0.0, None

    def summary(self):
        """summarize_csv 와 같은 (점 수, 길이, 퍼짐, 범위) — 기록 중인 파일을 다시 읽지 않으려고."""
        with self.lock:
            return self.rows, self.length, None, None

    def on_fix(self, pvt):
        with self.lock:
            if not self.fp:
                return
            lat, lon = pvt["lat"], pvt["lon"]
            if self.last is not None:
                d = math.hypot(*delta_m(self.last[0], self.last[1], lat, lon))
                if d < self.spacing:
                    return
                self.length += d
            self._write(pvt)
            self.last = (lat, lon)


class FixRecorder(Recorder):
    """검증 모드 — 일정 시간(수신기 iTOW 기준)마다 한 점."""

    def __init__(self, data_dir):
        super().__init__(data_dir)
        self.interval = INTERVAL_S
        self.last_itow = None    # 마지막으로 '기록한' 점의 iTOW [ms]
        self.spread = Spread()

    def file_name(self, stamp):
        return f"point_{stamp}_{self.interval:.1f}s.csv"

    def reset(self):
        self.last_itow, self.spread = None, Spread()

    def summary(self):
        """summarize_csv 와 같은 (점 수, 길이, 퍼짐, 범위) — 기록 중인 파일을 다시 읽지 않으려고."""
        with self.lock:
            return self.rows, None, self.spread.rms(), self.spread.extent()

    def on_fix(self, pvt):
        with self.lock:
            if not self.fp:
                return
            if self.last_itow is not None:
                gap = (pvt["itow"] - self.last_itow) % WEEK_MS
                # iTOW 는 측위 주기(100 ms) 격자에서 몇 ms 어긋날 수 있어 반 주기만큼 여유를 둔다
                if gap < round(self.interval * 1000) - EPOCH_MS // 2:
                    return
            self._write(pvt)
            self.last_itow = pvt["itow"]
            self.spread.add(pvt["lat"], pvt["lon"])


# ── GPS 스레드 ────────────────────────────────────────────────────────────────────
class GpsWorker(threading.Thread):
    """포트 감지 → 접속 → 수신기 설정 → 읽기. USB 가 빠지면 다시 찾는다.
    측위가 된 NAV-PVT 는 모든 recorder 에 넘긴다 — 기록 중이 아닌 쪽은 그냥 버린다."""

    def __init__(self, recorders):
        super().__init__(daemon=True)
        self.recorders = recorders
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
                ser = serial.Serial(port.device, SERIAL_BAUD, timeout=0.05, **SERIAL_OPEN_KW)
            except (serial.SerialException, OSError) as e:
                # Ubuntu 에서 dialout 그룹이 아니면 EACCES. Windows 는 다른 프로그램이 쓰는 중일 때도
                # '액세스 거부'라 권한 문제와 구분하지 않는다.
                denied = not IS_WINDOWS and e.errno == errno.EACCES
                self._set(conn="perm" if denied else "busy", err=str(e))
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
                    for rec in self.recorders:
                        rec.on_fix(p)
            if ggas or pvts:
                with self.lock:
                    if ggas:
                        self.state["gga"] = ggas[-1]
                    if pvts:
                        self.state["pvt"] = pvts[-1]


# ── find_wc 연계 ──────────────────────────────────────────────────────────────────
LAUNCHER_HTML = """<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><title>find_wc 로 이동</title></head>
<body style="font-family:'Malgun Gothic','Noto Sans CJK KR',sans-serif">
<p>find_wc 에서 기록을 여는 중… <a id="go" href="#">자동으로 안 넘어가면 누르세요</a></p>
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
    if len(url) <= URL_DIRECT_MAX:
        webbrowser.open(url)
        return
    launcher = LAUNCHER_DIR / "gps_mapper_find_wc.html"
    launcher.write_text(LAUNCHER_HTML.replace("__URL__", json.dumps(url)), encoding="utf-8")
    webbrowser.open(launcher.as_uri())


def open_folder(path):
    path.mkdir(exist_ok=True)
    if IS_WINDOWS:
        os.startfile(path)
    else:
        subprocess.Popen(["xdg-open", str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def summarize_csv(path):
    """(점 수, 경로 길이 m, RMS 퍼짐 m, (동서, 남북) 범위 m).
    위경도 열이 없으면 길이는 None, 퍼짐·범위는 점이 2개 미만이어도 None."""
    rows, length, prev, sp = 0, 0.0, None, Spread()
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
                sp.add(*cur)
    except (OSError, UnicodeError, csv.Error):
        return 0, None, None, None
    return rows, (length if ilat is not None and ilon is not None else None), sp.rms(), sp.extent()


# ── 창 ────────────────────────────────────────────────────────────────────────────
def pick_font(root):
    """한글이 나오는 글꼴 — Windows 는 맑은 고딕, Ubuntu 는 Noto Sans CJK KR 등. 없으면 Tk 기본 글꼴."""
    have = set(tkfont.families(root))
    for name in FONT_CANDIDATES:
        if name in have:
            return name
    return tkfont.nametofont("TkDefaultFont").actual("family")


def fmt_acc(m):
    return f"±{m * 100:.0f} cm" if m < 0.995 else f"±{m:.1f} m"


def fmt_cm(m):
    if m is None:
        return "-"
    return f"{m * 100:.1f} cm" if m < 0.995 else f"{m:.2f} m"


def fmt_extent(ext):
    """(동서, 남북) → '3.1×2.4 cm'"""
    if ext is None:
        return "-"
    w, h = ext
    if max(w, h) < 0.995:
        return f"{w * 100:.1f}×{h * 100:.1f} cm"
    return f"{w:.2f}×{h:.2f} m"


def fmt_elapsed(sec):
    sec = int(sec)
    return f"{sec // 3600}:{sec % 3600 // 60:02d}:{sec % 60:02d}"


class App:
    def __init__(self, root, worker, recorders):
        self.root, self.worker, self.recorders = root, worker, recorders
        self.mode = "map"
        self.rate_hist = deque(maxlen=40)
        self.file_cache = {}
        self.notes = {m: ("", GRAY) for m in MODES}   # 모드별 마지막 저장 결과

        F = pick_font(root)
        root.geometry("760x700")
        root.minsize(660, 560)
        style = ttk.Style(root)
        if not IS_WINDOWS:
            style.theme_use("clam")    # Linux 기본 테마(default)는 너무 투박하다
        style.configure(".", font=(F, 10))
        style.configure("Treeview.Heading", font=(F, 10, "bold"))
        style.configure("Treeview", rowheight=26)

        self.top = top = ttk.Frame(root, padding=(14, 12, 14, 4))
        top.pack(fill="x")
        top.columnconfigure(1, weight=1)
        top.bind("<Configure>", self.fit_rec_label)
        self.btn = tk.Button(top, font=(F, 14, "bold"), width=12,
                             bg=GREEN, fg="white", activebackground=GREEN, activeforeground="white",
                             relief="flat", cursor="hand2", command=self.toggle_recording)
        self.btn.grid(row=0, column=0, rowspan=2, sticky="w", padx=(0, 14))
        self.rec_lbl = ttk.Label(top, text="대기 중", wraplength=380, justify="left")
        self.rec_lbl.grid(row=0, column=1, sticky="w")

        # 오른쪽 위 모드 선택 — 둘 중 하나. 기록 중에는 잠근다.
        seg = tk.Frame(top, bg=SEG_EDGE, padx=1, pady=1)
        seg.grid(row=0, column=2, sticky="ne", padx=(10, 0))
        self.mode_var = tk.StringVar(value=self.mode)
        self.mode_btns = {}
        for i, (key, spec) in enumerate(MODES.items()):
            b = tk.Radiobutton(seg, text=spec["name"], value=key, variable=self.mode_var,
                               indicatoron=False, font=(F, 10, "bold"), width=9, pady=3,
                               relief="flat", offrelief="flat", bd=0, highlightthickness=0,
                               command=self.on_mode)
            b.pack(side="left", padx=(1 if i else 0, 0))
            self.mode_btns[key] = b

        sp = ttk.Frame(top)
        sp.grid(row=1, column=1, columnspan=2, sticky="w", pady=(6, 0))
        ttk.Label(sp, text="기록 간격 :").pack(side="left")
        self.step_vars = {"map": tk.StringVar(value=f"{SPACING_M:.2f}"),
                          "check": tk.StringVar(value=f"{INTERVAL_S:.1f}")}
        self.spin = ttk.Spinbox(sp, width=6, command=self.apply_step)
        self.spin.pack(side="left", padx=4)
        self.unit_lbl = ttk.Label(sp)
        self.unit_lbl.pack(side="left")
        for ev in ("<Return>", "<FocusOut>", "<KeyRelease>"):
            self.spin.bind(ev, lambda e: self.apply_step())
        self.rtk_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(sp, text="RTK 보정 사용", variable=self.rtk_var,
                        command=self.apply_rtk).pack(side="left", padx=(18, 0))
        self.step_msg = ttk.Label(sp, text="", foreground=RED)
        self.step_msg.pack(side="left", padx=8)

        st = ttk.Frame(root, padding=(14, 10, 14, 0))
        st.pack(fill="x")
        line = ttk.Frame(st)
        line.pack(fill="x")
        ttk.Label(line, text="상태 :", font=(F, 16, "bold")).pack(side="left")
        self.status_lbl = tk.Label(line, text="", font=(F, 18, "bold"))
        self.status_lbl.pack(side="left", padx=(8, 0))
        self.acc_lbl = tk.Label(line, text="", font=(F, 18, "bold"))
        self.acc_lbl.pack(side="left", padx=(14, 0))
        self.sub_lbl = tk.Label(st, text="", font=(F, 10), anchor="w", justify="left", wraplength=720)
        self.sub_lbl.pack(fill="x", pady=(2, 0))

        det = ttk.LabelFrame(root, text="상세", padding=(10, 6))
        det.pack(fill="x", padx=14, pady=(10, 0))
        self.detail = {}
        for r, key in enumerate(("GPS", "위성", "RTK", "기지국", "위치")):
            ttk.Label(det, text=key, width=5, font=(F, 10, "bold")).grid(row=r, column=0, sticky="nw")
            lbl = ttk.Label(det, text="-", wraplength=640, justify="left")
            lbl.grid(row=r, column=1, sticky="w", pady=1)
            self.detail[key] = lbl

        self.data_frame = ttk.LabelFrame(root, padding=(10, 6))
        self.data_frame.pack(fill="both", expand=True, padx=14, pady=(10, 14))
        bar = ttk.Frame(self.data_frame)
        bar.pack(fill="x")
        self.hint_lbl = ttk.Label(bar, foreground=GRAY)
        self.hint_lbl.pack(side="left")
        ttk.Button(bar, text="폴더 열기", command=self.open_data_folder).pack(side="right")
        body = ttk.Frame(self.data_frame)
        body.pack(fill="both", expand=True, pady=(6, 0))
        # 두 모드의 열을 다 만들어 두고 displaycolumns 로 모드에 맞는 것만 보인다
        self.tree = ttk.Treeview(body, columns=[c[0] for c in TREE_COLS],
                                 show="tree headings", selectmode="none")
        self.tree.heading("#0", text="파일", anchor="w")
        for col, text, width, anchor in TREE_COLS:
            self.tree.heading(col, text=text)
            self.tree.column(col, width=width, anchor=anchor)
        vsb = ttk.Scrollbar(body, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")     # 표보다 먼저 놓아야 표가 넓어져도 밀려나지 않는다
        self.tree.pack(side="left", fill="both", expand=True)
        self.tree.tag_configure("live", foreground=RED)
        self.tree.bind("<ButtonRelease-1>", self.on_tree_click)
        self.tree.configure(cursor="hand2")

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.apply_mode()
        self.tick()
        self.refresh_files()

    @property
    def rec(self):
        """지금 모드의 recorder."""
        return self.recorders[self.mode]

    # ── 모드 ─────────────────────────────────────────────────────────────────────
    def on_mode(self):
        new = self.mode_var.get()
        if new == self.mode:
            return
        if self.rec.active:    # 기록 중에는 버튼을 잠그므로 보통 여기 오지 않는다
            self.mode_var.set(self.mode)
            return
        self.mode = new
        self.apply_mode()

    def apply_mode(self):
        """창을 지금 모드에 맞춘다. 기록 중이 아닐 때만 부른다."""
        spec = MODES[self.mode]
        self.root.title(f"GPS 매퍼 — {spec['name']}")
        self.btn.configure(text=spec["start"], bg=GREEN, activebackground=GREEN)
        self.spin.configure(textvariable=self.step_vars[self.mode], from_=spec["lo"], to=spec["hi"],
                            increment=spec["step"], format=spec["fmt"])
        self.spin.state(["!disabled"])
        self.unit_lbl.configure(text=spec["unit"])
        self.apply_step()
        self.data_frame.configure(text=f"{spec['data']} — {self.rec.data_dir.name}")
        self.hint_lbl.configure(text=spec["hint"])
        self.tree.configure(displaycolumns=spec["cols"])
        self.tree.column("#0", width=spec["name_w"])
        for col, _, width, _ in TREE_COLS:   # 앞 모드에서 창 폭에 맞춰 늘어난 열 폭을 되돌린다
            self.tree.column(col, width=width)
        self.tree.delete(*self.tree.get_children())
        self.paint_mode_buttons()
        self.show_progress()
        self.refresh_files(force=True)

    def paint_mode_buttons(self):
        locked = self.rec.active
        for key, b in self.mode_btns.items():
            on = key == self.mode
            bg, fg = (BLUE, "white") if on else (SEG_OFF_BG, SEG_OFF_FG)
            b.configure(bg=bg, fg=fg, selectcolor=bg, activebackground=bg, activeforeground=fg,
                        disabledforeground="white" if on else "#adb5bd",
                        state="disabled" if locked else "normal",
                        cursor="arrow" if locked or on else "hand2")

    # ── 기록 ─────────────────────────────────────────────────────────────────────
    def toggle_recording(self):
        rec, spec = self.rec, MODES[self.mode]
        if rec.active:
            path, rows, el = rec.stop()
            _, length, rms, ext = rec.summary()
            if self.mode == "map":
                note = (f"⚠ {path.name} — 점이 {rows}개뿐이라 경로로 쓸 수 없음", RED) if rows < 2 else \
                       (f"저장됨: {path.name} · {rows}점 · {length:.1f} m", GREEN)
            else:
                note = (f"⚠ {path.name} — 점이 {rows}개뿐", RED) if rows < 2 else \
                       (f"저장됨: {path.name} · {rows}점 · {fmt_elapsed(el)} · "
                        f"퍼짐 {fmt_cm(rms)} · 범위 {fmt_extent(ext)}", GREEN)
            self.notes[self.mode] = note
            self.btn.configure(text=spec["start"], bg=GREEN, activebackground=GREEN)
            self.spin.state(["!disabled"])
        else:
            if not self.apply_step():
                return
            try:
                rec.start()
            except OSError as e:
                messagebox.showerror(f"{spec['start']} 실패", f"CSV 파일을 만들 수 없습니다.\n{e}")
                return
            self.btn.configure(text=spec["stop"], bg=RED, activebackground=RED)
            if self.mode == "check":
                self.spin.state(["disabled"])   # 한 파일 안에서 간격이 섞이지 않게
        self.paint_mode_buttons()
        self.show_progress()
        self.refresh_files(force=True)

    def apply_step(self):
        """입력한 기록 간격이 맞으면 recorder 에 넣고 True.
        매핑은 기록 중에도 바로 반영하고, 검증은 기록 중에는 바꾸지 않는다."""
        if self.mode == "check" and self.rec.active:
            return True
        spec = MODES[self.mode]
        try:
            v = float(self.step_vars[self.mode].get())
        except ValueError:
            v = None
        ok = v is not None and spec["lo"] <= v <= spec["hi"]
        if ok and self.mode == "check":
            ok = abs(v * 10 - round(v * 10)) <= 1e-6     # 0.1 초 단위 (NAV-PVT 10 Hz)
        if not ok:
            self.step_msg.configure(text=spec["bad"])
            return False
        self.step_msg.configure(text="")
        if self.mode == "map":
            self.rec.spacing = v
        else:
            self.rec.interval = round(v, 1)
        return True

    def apply_rtk(self):
        self.worker.ntrip.set_enabled(self.rtk_var.get())

    # ── 상태 표시 ────────────────────────────────────────────────────────────────
    def rtk_phase(self, now):
        nt = self.worker.ntrip
        if not nt.enabled:
            return "RTK 꺼짐"
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
            return "GPS 사용 중", RED, "", (f"{s['port']} 를 다른 프로그램이 쓰고 있음 "
                                           f"(다른 gps_mapper 창, u-center, gpstest.py 등을 닫아 주세요)")
        if conn == "perm":
            return "GPS 권한 없음", RED, "", (f"{s['port']} 를 열 권한이 없음 — 터미널에서 "
                                             f"sudo usermod -aG dialout $USER 실행 후 다시 로그인")
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
        if not self.worker.ntrip.enabled and pvt["carr"]:
            rtk += " — 수신기에 남은 보정이 만료되면 풀림"
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
            if s["conn"] in ("busy", "perm", "lost") and s["err"]:
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
        if not nt.enabled:
            rtk += " — 체크하면 접속"
        elif nt.status == "streaming" and nt.session_bytes:
            rtk += f" · {rate / 1000:.2f} KB/s"
        gga = s["gga"]
        if gga and gga["age"]:
            rtk += f" · 보정 나이 {gga['age']} s"
        if nt.enabled and nt.status in ("retry", "fatal") and nt.detail:
            rtk += f" · 원인: {nt.detail}"
        self.detail["RTK"].configure(text=rtk)
        here = (pvt["lat"], pvt["lon"]) if pvt and pvt["ok"] else None
        self.detail["기지국"].configure(text=describe_base(nt.rtcm.snapshot(), here, now) if nt.enabled else "-")

        self.show_progress()
        self.root.after(200, self.tick)

    def show_progress(self):
        """버튼 옆 글줄 — 기록 중이면 진행 상황, 아니면 이 모드의 마지막 저장 결과."""
        rec = self.rec
        if rec.active:
            rows, length, rms, ext = rec.summary()
            msg = f"● 기록 중  {rec.path.name} · {rows}점"
            if length is not None:
                msg += f" · {length:.1f} m"
            msg += f" · {fmt_elapsed(time.time() - rec.started)}"
            if rows == 0:
                msg += "  (측위 대기 — 아직 기록된 점 없음)"
            elif rms is not None:
                msg += f" · 퍼짐 {fmt_cm(rms)} · 범위 {fmt_extent(ext)}"
            self.rec_lbl.configure(text=msg, foreground=RED)
        else:
            text, col = self.notes[self.mode]
            self.rec_lbl.configure(text=text or "대기 중", foreground=col)

    def fit_rec_label(self, _event=None):
        """버튼 옆 글줄을 모드 선택 왼쪽 칸 폭에서 줄바꿈한다(칸 폭은 창 폭·글꼴·DPI 따라 다르다)."""
        width = self.top.grid_bbox(1, 0)[2]
        if width > 50:
            self.rec_lbl.configure(wraplength=width - 4)

    # ── 파일 목록 ────────────────────────────────────────────────────────────────
    def refresh_files(self, force=False):
        rec = self.rec
        try:
            files = sorted(rec.data_dir.glob("*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
        except OSError:
            files = []
        live = rec.path if rec.active else None
        names = {p.name for p in files}
        for iid in self.tree.get_children():
            if iid not in names:
                self.tree.delete(iid)
        for idx, p in enumerate(files):
            try:
                stt = p.stat()
            except OSError:
                continue
            if p == live:
                # 기록 중인 파일은 매번 다시 읽지 않고 recorder 의 누적값을 쓴다(검증 0.1 초 간격이면 금방 커진다)
                rows, length, rms, ext = rec.summary()
            else:
                key = (stt.st_mtime_ns, stt.st_size)
                cached = self.file_cache.get(p)
                if force or not cached or cached[0] != key:
                    cached = (key, *summarize_csv(p))
                    self.file_cache[p] = cached
                _, rows, length, rms, ext = cached
            values = (f"{rows}", "-" if length is None else f"{length:.1f} m", fmt_cm(rms), fmt_extent(ext),
                      datetime.fromtimestamp(stt.st_mtime).strftime("%m-%d %H:%M:%S"))
            tags = ("live",) if p == live else ()
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
        path = self.rec.data_dir / iid
        rows = summarize_csv(path)[0]
        if rows < 2:
            messagebox.showinfo("find_wc", f"{iid}\n점이 {rows}개라 지도에 그릴 수 없습니다.")
            return
        try:
            open_in_find_wc(path)
        except OSError as e:
            messagebox.showerror("find_wc 열기 실패", str(e))

    def open_data_folder(self):
        try:
            open_folder(self.rec.data_dir)
        except OSError as e:
            messagebox.showerror("폴더 열기 실패", str(e))

    def on_close(self):
        for rec in self.recorders.values():
            if rec.active:
                rec.stop()
        self.worker.stop()
        self.root.after(150, self.root.destroy)


def main():
    if IS_WINDOWS:
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(1)   # 고해상도 화면에서 글자가 번지지 않게
        except (AttributeError, OSError):
            pass
    recorders = {"map": RouteRecorder(MAP_DIR), "check": FixRecorder(CHECK_DIR)}
    for rec in recorders.values():
        rec.data_dir.mkdir(exist_ok=True)
    worker = GpsWorker(list(recorders.values()))
    worker.start()
    root = tk.Tk()
    App(root, worker, recorders)
    root.mainloop()


if __name__ == "__main__":
    main()
