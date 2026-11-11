#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ports.py ― USB 시리얼 장치 찾기 (GPS · IMU · 아두이노 공용)
════════════════════════════════════════════════════════════════════════════════
USB VID/PID 로만 고른다. ★맞는 포트만 열고 다른 포트는 열어 보지 않는다★ —
아두이노 Mega 는 포트를 여는 순간 리셋되므로 '아무 포트나 열어 확인' 은 쓰지 않는다
(gps_mapper.py 와 같은 규칙).

  GPS     u-blox ZED-F9P        VID 1546  (두 대가 USB 상으로 똑같이 보인다 — info.md)
  IMU     iAHRS (CP210x)        10c4:ea60
  아두이노 Mega 2560 / 호환     VID 2341 · 2a03 · 1a86(CH340)

ttyACM / ttyUSB 번호는 꽂는 순서로 바뀌므로 번호를 고정해 쓰지 않는다. 여러 개가 맞으면
번호가 작은 것부터 고르고, 이미 다른 노드가 쓰는 경로는 exclude 로 뺀다.
"""

import re

import serial.tools.list_ports

GPS_VIDS = {0x1546}
IMU_VIDPIDS = {(0x10C4, 0xEA60)}
ARDUINO_VIDS = {0x2341, 0x2A03, 0x1A86}

# F9P 칩 고유 ID (UBX-SEC-UNIQID 5바이트, 16진수) → 이름. 두 대가 USB 상으로 똑같아 이것으로만 가른다.
# 정본은 hl_ws/tool/info.md · tool/gps_mapper_dual.py 의 GPS1_ID / GPS2_ID. check.py 가 쓴다.
GPS_IDS = {
    "54401a3a98": "1번 GPS (HPG 1.32)",
    "55408a3e9f": "2번 GPS (HPG 1.13)",
}


def _sort_key(dev):
    """ttyACM2 가 ttyACM10 보다 앞에 오게 끝 번호를 숫자로 비교."""
    m = re.search(r"(\d+)$", dev)
    return (dev[:m.start()], int(m.group(1))) if m else (dev, -1)


def find(vids=(), vidpids=(), exclude=()):
    """조건에 맞는 첫 포트 경로. 없으면 ''."""
    hits = []
    for p in serial.tools.list_ports.comports():
        if p.vid is None or not p.device.startswith("/dev/tty") or p.device in exclude:
            continue
        if p.vid in vids or (p.vid, p.pid) in vidpids:
            hits.append(p.device)
    hits.sort(key=_sort_key)
    return hits[0] if hits else ""


def find_gps(exclude=()):
    return find(vids=GPS_VIDS, exclude=exclude)


def find_imu(exclude=()):
    return find(vidpids=IMU_VIDPIDS, exclude=exclude)


def find_arduino(exclude=()):
    return find(vids=ARDUINO_VIDS, exclude=exclude)
