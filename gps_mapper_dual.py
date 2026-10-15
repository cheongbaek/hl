#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gps_mapper_dual.py ― GPS 2대 동시 기록 (매핑 · 검증, Windows 11 · Ubuntu · tkinter)
════════════════════════════════════════════════════════════════════════════════
gps_mapper.py 의 듀얼 판. 같은 u-blox ZED-F9P 두 대를 USB 로 동시에 꽂고, 각자 RTK 로 측위한 위치를
★같은 측위 시각(iTOW)끼리 짝지어 한 줄에★ 남긴다. 창 왼쪽이 1번, 오른쪽이 2번 GPS 다.

★gps_mapper.py 와 같은 폴더에 둔다★ — 수신기 설정(RAM) · NTRIP · CSV 앞 열 · find_wc 연계 · 글꼴 · 색은
그 파일의 것을 그대로 가져다 쓴다(한쪽만 고쳐 두 프로그램이 어긋나는 일이 없게).

수신기 구분 : ttyACM 번호는 인식 순서, 포트 경로는 꽂은 자리를 따를 뿐 기기를 가리키지 않는다. 두 대는 USB
  시리얼 번호도 없어 USB 상으로 똑같다(info.md '포트 · tty 관찰'). 그래서 접속할 때 ★칩 고유 ID
  (UBX-SEC-UNIQID)★를 읽어 1번 · 2번을 정한다 — 아래 GPS1_ID / GPS2_ID. 등록 안 된 수신기는 쓰지 않고
  화면에 그 ID 만 알린다. 다른 기기를 쓰려면 그 두 줄만 고친다.

기지국 : gps_mapper.py 의 RtcmMonitor 로 받은 RTCM 에서 VRS · 실제 기준국(NGII 위성기준점 이름)을 '두 GPS' 칸에 보인다.

RTK : NTRIP 접속은 ★하나★다. 받은 RTCM 을 두 수신기에 똑같이 넣는다. VRS 는 GGA 로 올린 위치 근처에 만든
  가상 기준국이라 1~2 m 떨어진 두 안테나가 같은 보정을 쓸 수 있고, 같은 ID 로 두 번 접속할 일도 없다.
  캐스터에 올리는 GGA 는 1번이 측위 중이면 1번 것, 아니면 2번 것.

짝짓기 : 두 수신기 모두 10 Hz 로 GPS 시각 0.1 초 경계에서 측위한다 → 같은 순간의 NAV-PVT 는 iTOW 가 같다
  (실내 시험 300/300 짝, PC 도착 시각 차 2~10 ms). ★도착 시각이 아니라 iTOW 로★ 짝을 짓는다.
  한쪽이 PAIR_TIMEOUT_S 안에 안 오면 그쪽은 빈칸으로 넘긴다. 한쪽 수신기가 아예 없으면 기다리지 않는다.

기록 — 둘 중 하나라도 측위가 됐으면 한 줄을 쓴다(측위가 안 된 쪽 열은 빈칸).
  매핑 : gps_data/route_dual_YYYYmmdd_HHMMSS.csv — ★일정 거리마다★(기본 0.25 m, gps_mapper.py 와 같다).
         기준은 1번(측위가 안 되면 2번)이 마지막으로 기록한 자리에서 그만큼 움직였을 때.
  검증 : gps_check_data/point_dual_YYYYmmdd_HHMMSS_<간격>s.csv — ★일정 시간마다★, 기본 0.1 초 = NAV-PVT 를
         하나도 버리지 않는다. 간격은 iTOW 로 잰다. 기록 중에는 간격을 바꿀 수 없다.
  이름이 route_ · point_ 로 시작하지만 _dual_ 로 단일 GPS 기록과 갈린다. 목록에는 _dual_ 파일만 보인다.

CSV : 앞 17열은 gps_mapper.py 와 같고 ★1번 GPS★ 값이다 — find_wc · reminding.py 의 옛 판도 1번 궤적으로는
  그대로 읽는다. 1번이 측위 안 된 줄은 latitude/longitude 가 빈칸이다. 그 뒤에 붙는 열:
    latitude2, longitude2, speed2, gps_quality2, gps_sigma_m2 — 2번 GPS (뜻은 1번 열과 같다)
    num_sv, num_sv2          — 측위에 쓴 위성 수 (1번, 2번)
    hmsl_m, hmsl2_m          — 해발고도 [m]
    itow_ms                  — 측위 시각 (GPS 주 시작부터 ms, 짝지은 두 GPS 공통)
    t_rel                    — 기록 시작부터 [s]. iTOW 로 계산하므로 행 번호로 역산할 필요가 없다
    baseline_m, baseline_deg — 1번 → 2번 수평 거리 [m] 와 방위(북 0°, 동 90°). 둘 다 측위됐을 때만
    rx_dt_ms                 — 같은 측위가 PC 에 도착한 시각 차 (2번 − 1번) [ms]. 둘 다 왔을 때만
  find_wc 는 latitude2/longitude2 가 있으면 한 파일을 1번 · 2번 두 궤적으로 그린다.

필요한 것 : gps_mapper.py 와 같다(Ubuntu: python3-tk python3-serial, dialout 그룹).
"""

import csv
import errno
import math
import sys
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import messagebox, ttk

import serial
import serial.tools.list_ports

import gps_mapper as gm


# ── 수신기 등록 — 칩 고유 ID (UBX-SEC-UNIQID 5바이트, 16진수). 다른 기기를 쓰면 여기만 고친다 ──
GPS1_ID = "54401a3a98"     # 1번 GPS — ZED-F9P, HPG 1.32 (info.md)
GPS2_ID = "55408a3e9f"     # 2번 GPS — ZED-F9P, HPG 1.13 (info.md)
GPS_IDS = {GPS1_ID.lower(): 1, GPS2_ID.lower(): 2}

INTERVAL_S = 0.1           # 검증 기본 간격 = 출력 주기(10 Hz) 그대로
INTERVAL_MIN = 0.1
PAIR_TIMEOUT_S = 0.5       # 한쪽 NAV-PVT 를 이만큼 기다려도 안 오면 빈칸으로 넘긴다
IDENT_TIMEOUT_S = 4.0      # 칩 ID 를 읽는 데 쓸 최대 시간 (공장 설정의 NMEA 폭주 속에서도 읽히게)
UNKNOWN_RETRY_S = 10.0     # 등록 안 된 수신기는 이만큼 지나야 다시 열어 본다
SCAN_SEC = 1.0             # 새로 꽂힌 포트를 찾는 주기
GGA_FRESH_S = 3.0          # 이보다 오래된 GGA 는 캐스터에 올리지 않는다
STATS_WINDOW_S = 10.0      # 화면의 짝 맞춤 · 기선 흔들림 통계 구간
SIDES = (1, 2)

DUAL_HEADER = gm.CSV_HEADER + [
    "latitude2", "longitude2", "speed2", "gps_quality2", "gps_sigma_m2",
    "num_sv", "num_sv2", "hmsl_m", "hmsl2_m",
    "itow_ms", "t_rel", "baseline_m", "baseline_deg", "rx_dt_ms",
]
# 1번이 측위 안 된 줄의 앞 17열 — 위경도 · 속도 · 품질은 빈칸, 차량 실계측 열은 단일판과 같이 0
EMPTY_FRONT = ["", "", "0.00", "", "0", "0", "0.00", "0",
               "0", "0.0", "0.000", "0", "0", "0", "0", "", ""]

MODES = {
    "map": dict(gm.MODES["map"], hint="파일을 누르면 find_wc 에서 1번 · 2번 경로를 함께 엽니다",
                cols=("pts", "len", "bl", "time"), name_w=300),
    "check": dict(gm.MODES["check"], lo=INTERVAL_MIN,
                  bad=f"{INTERVAL_MIN}~{gm.INTERVAL_MAX:.0f} 초, 0.1 초 단위로 입력",
                  hint="파일을 누르면 find_wc 에서 엽니다 · 퍼짐은 평균 위치 기준 RMS · 기선은 1번→2번 거리",
                  cols=("pts", "rms1", "rms2", "bl", "time"), name_w=270),
}
TREE_COLS = (("pts", "줄", 60, "e"), ("len", "길이(1번)", 90, "e"), ("rms1", "퍼짐 1번", 80, "e"),
             ("rms2", "퍼짐 2번", 80, "e"), ("bl", "기선 평균 ± σ", 140, "e"),
             ("time", "수정 시각", 120, "center"))
FILE_GLOB = {"map": "route_dual_*.csv", "check": "point_dual_*.csv"}


def has_fix(p):
    """기록할 만한 측위인가 — gps_mapper.py 와 같은 기준."""
    return bool(p) and p["ok"] and p["fix"] in (2, 3, 4)


def baseline(a, b):
    """1번 → 2번 (수평 거리 m, 방위 °) — 북 0°, 동 90°."""
    e, n = gm.delta_m(a["lat"], a["lon"], b["lat"], b["lon"])
    return math.hypot(e, n), math.degrees(math.atan2(e, n)) % 360


# ── 수신기 식별 ────────────────────────────────────────────────────────────────────
def poll_ubx(ser, cls, id_, timeout):
    """UBX 조회(poll)를 보내고 응답 payload 를 돌려준다. 설정은 바꾸지 않는다.
    공장 설정이면 NMEA 가 쏟아지므로 0.5 초마다 다시 묻고, 체크섬이 맞는 프레임만 받는다."""
    want = b"\xb5\x62" + bytes([cls, id_])
    buf, t0, last_ask = b"", time.time(), 0.0
    while time.time() - t0 < timeout:
        if time.time() - last_ask > 0.5:
            ser.write(gm.ubx(cls, id_))
            last_ask = time.time()
        buf += ser.read(4096)
        i = buf.find(want)
        while i >= 0:
            if len(buf) < i + 6:
                break
            n = int.from_bytes(buf[i + 4:i + 6], "little")
            if len(buf) < i + 8 + n:
                break
            frame = buf[i:i + 8 + n]
            if gm.ubx(cls, id_, frame[6:6 + n]) == frame:
                return frame[6:6 + n]
            i = buf.find(want, i + 2)
        buf = buf[-2048:]
    return None


def identify(ser):
    """(칩 ID 16진수, 펌웨어 'HPG 1.32') — 못 읽으면 빈 문자열."""
    uid = poll_ubx(ser, 0x27, 0x03, IDENT_TIMEOUT_S)
    ver = poll_ubx(ser, 0x0A, 0x04, 2.0)
    fw = ""
    if ver:
        for k in range(40, len(ver), 30):
            s = ver[k:k + 30].split(b"\0")[0].decode("ascii", "replace")
            if s.startswith("FWVER="):
                fw = s[6:]
    return (uid[4:9].hex() if uid and len(uid) >= 9 else ""), fw


# ── 짝짓기 ────────────────────────────────────────────────────────────────────────
class Pairer:
    """두 수신기의 NAV-PVT 를 iTOW 로 짝지어 ★iTOW 순서대로★ sink(pvt1, pvt2) 로 넘긴다.
    한쪽이 PAIR_TIMEOUT_S 안에 안 오면 None 으로 넘긴다. 상대 수신기가 연결돼 있지 않으면(is_active)
    기다리지 않는다 — 한 대만 꽂았을 때 줄마다 0.5 초씩 늦지 않게."""

    def __init__(self, sink, is_active):
        self.sink = sink
        self.is_active = is_active
        self.lock = threading.Lock()
        self.wait = {}                      # iTOW → [pvt1, pvt2, 처음 온 시각]

    def feed(self, no, pvt):
        # sink 까지 잠금 안에서 부른다 — 두 읽기 스레드가 동시에 내보내도 CSV 줄이 iTOW 순서를 지키게
        with self.lock:
            now = time.monotonic()
            e = self.wait.setdefault(pvt["itow"], [None, None, now])
            e[no - 1] = pvt
            for a, b in self._ready(now):
                self.sink(a, b)

    def flush(self):
        """주기적으로 불러 시간이 지난 것을 내보낸다(두 수신기가 다 멈췄을 때도 남은 것이 나가게)."""
        with self.lock:
            for a, b in self._ready(time.monotonic()):
                self.sink(a, b)

    def _ready(self, now):
        out = []
        for itow in sorted(self.wait):
            a, b, t0 = self.wait[itow]
            done = a is not None and b is not None
            absent = not done and not self.is_active(2 if b is None else 1)
            if not (done or absent or now - t0 > PAIR_TIMEOUT_S):
                break                       # 이것을 기다리는 동안 뒤의 것도 내보내지 않는다 — 순서 유지
            del self.wait[itow]
            out.append((a, b))
        return out


# ── GPS 스레드 ────────────────────────────────────────────────────────────────────
class Slot:
    """1번 · 2번 수신기 한 대. 읽기 세션이 state 를 쓰고 창이 snapshot 으로 읽는다."""

    def __init__(self, no):
        self.no = no
        self.lock = threading.Lock()
        self.ser_lock = threading.Lock()    # 시리얼 쓰기(설정 · 보정) 보호
        self.ser = None
        self.busy = False                   # 이 번호를 붙잡은 세션이 있다 (DualWorker.lock 으로 보호)
        self.state = dict(conn="search", port="", uid="", fw="", cfg_ok=None, err="",
                          pvt=None, gga=None, gga_t=0.0)

    def snapshot(self):
        with self.lock:
            return dict(self.state)

    def set(self, **kw):
        with self.lock:
            self.state.update(kw)

    def write(self, data):
        with self.ser_lock:
            if self.ser:
                try:
                    self.ser.write(data)
                except (serial.SerialException, OSError):
                    pass


class DualWorker(threading.Thread):
    """u-blox 포트를 찾아 칩 ID 로 1번 · 2번을 가리고, 포트마다 읽기 세션 스레드를 돌린다.
    NTRIP 은 하나 — 받은 보정을 열려 있는 수신기 모두에 쓴다."""

    def __init__(self, recorders):
        super().__init__(daemon=True)
        self.recorders = recorders
        self.slots = {n: Slot(n) for n in SIDES}
        self.lock = threading.Lock()
        self.claimed = set()                # 세션이 붙잡은 포트
        self.retry_at = {}                  # 포트 → 다시 열어 볼 시각 (등록 안 됨 · 열기 실패)
        self.port_notes = {}                # 포트 → 화면에 알릴 말
        self.stop_evt = threading.Event()
        self.pairer = Pairer(self._on_pair, self._is_active)
        self.ntrip = gm.NtripClient(gm.NTRIP, self._write_rtcm)
        self.stats_lock = threading.Lock()
        self.recent = deque()               # (시각, 둘 다 왔나, 도착 차 ms, 기선 m | None)
        self.last_pair = None               # 가장 최근 짝 (pvt1, pvt2)

    def _is_active(self, no):
        """그 번호의 수신기가 열려 측위를 내보내는 중인가."""
        slot = self.slots[no]
        return slot.busy and slot.snapshot()["conn"] == "open"

    # ── 보정 · GGA ──
    def _write_rtcm(self, data):
        for slot in self.slots.values():
            slot.write(data)

    def _pick_gga(self):
        """캐스터에 올릴 GGA — 1번이 측위 중이면 1번, 아니면 2번."""
        now = time.time()
        for n in SIDES:
            s = self.slots[n].snapshot()
            g = s["gga"]
            if g and g["q"] > 0 and now - s["gga_t"] <= GGA_FRESH_S:
                self.ntrip.gga_raw = g["raw"]
                self.ntrip_gga_from = n
                return

    ntrip_gga_from = None

    # ── 짝 ──
    def _on_pair(self, p1, p2):
        now = time.time()
        both = p1 is not None and p2 is not None
        bl = baseline(p1, p2)[0] if has_fix(p1) and has_fix(p2) else None
        dt = (p2["t"] - p1["t"]) * 1000 if both else None
        with self.stats_lock:
            self.recent.append((now, both, dt, bl))
            while self.recent and now - self.recent[0][0] > STATS_WINDOW_S:
                self.recent.popleft()
            self.last_pair = (p1, p2)
        for rec in self.recorders:
            rec.on_pair(p1, p2)

    def pair_stats(self):
        """최근 STATS_WINDOW_S 동안 (짝 수, 짝 맞은 비율, 도착 차 평균 ms, 기선 평균 m, 기선 표준편차 m)."""
        with self.stats_lock:
            rows = list(self.recent)
        if not rows:
            return 0, None, None, None, None
        both = [r for r in rows if r[1]]
        dts = [r[2] for r in both]
        bls = [r[3] for r in rows if r[3] is not None]
        mean_bl = sum(bls) / len(bls) if bls else None
        sd_bl = math.sqrt(max(0.0, sum(b * b for b in bls) / len(bls) - mean_bl ** 2)) if bls else None
        return (len(rows), len(both) / len(rows), (sum(dts) / len(dts)) if dts else None, mean_bl, sd_bl)

    # ── 포트 ──
    def stop(self):
        self.stop_evt.set()
        self.ntrip.stop()

    def notes(self):
        with self.lock:
            return dict(self.port_notes)

    def _note(self, dev, text, retry_s):
        with self.lock:
            self.port_notes[dev] = text
            self.retry_at[dev] = time.time() + retry_s

    def run(self):
        self.ntrip.start()
        next_scan = 0.0
        while not self.stop_evt.is_set():
            self.pairer.flush()
            if time.time() >= next_scan:
                next_scan = time.time() + SCAN_SEC
                self._scan()
            self.stop_evt.wait(0.2)

    def _scan(self):
        try:
            ports = [p.device for p in serial.tools.list_ports.comports()
                     if p.vid == gm.UBLOX_VID and gm.is_serial_device(p.device)]
        except OSError:
            ports = []
        now = time.time()
        with self.lock:
            for dev in list(self.port_notes):        # 빠진 포트의 알림은 지운다
                if dev not in ports:
                    self.port_notes.pop(dev, None)
                    self.retry_at.pop(dev, None)
            for slot in self.slots.values():         # 빠진 수신기는 '끊김' 에서 '찾는 중' 으로
                s = slot.snapshot()
                if not slot.busy and s["conn"] == "lost" and s["port"] not in ports:
                    slot.set(conn="search", port="", pvt=None, gga=None)
            for dev in sorted(ports, key=gm.port_sort_key):
                if dev in self.claimed or now < self.retry_at.get(dev, 0):
                    continue
                self.claimed.add(dev)
                threading.Thread(target=self._port_session, args=(dev,), daemon=True).start()

    def _port_session(self, dev):
        """포트 하나 — 열고, 칩 ID 로 번호를 정하고, 빠질 때까지 읽는다."""
        slot = None
        try:
            try:
                ser = serial.Serial(dev, gm.SERIAL_BAUD, timeout=0.05, **gm.SERIAL_OPEN_KW)
            except (serial.SerialException, OSError) as e:
                denied = not gm.IS_WINDOWS and getattr(e, "errno", None) == errno.EACCES
                self._note(dev, "열 권한 없음 — sudo usermod -aG dialout $USER 후 다시 로그인" if denied
                           else f"열 수 없음 — 다른 프로그램(gps_mapper.py · u-center 등)이 쓰는 중? ({e})", 3.0)
                return
            try:
                uid, fw = identify(ser)
                no = GPS_IDS.get(uid)
                if no is None:
                    self._note(dev, (f"등록 안 된 수신기 (칩 ID {uid}) — gps_mapper_dual.py 의 GPS1_ID/GPS2_ID 를 확인"
                                     if uid else "칩 ID 를 읽지 못함 — 잠시 뒤 다시 시도"),
                               UNKNOWN_RETRY_S if uid else 3.0)
                    return
                with self.lock:
                    if self.slots[no].busy:
                        self.port_notes[dev] = f"{no}번 칩 ID 가 이미 다른 포트에 연결돼 있음 (ID {uid})"
                        self.retry_at[dev] = time.time() + UNKNOWN_RETRY_S
                        return
                    slot = self.slots[no]
                    slot.busy = True
                    self.port_notes.pop(dev, None)
                slot.set(conn="opening", port=dev, uid=uid, fw=fw, cfg_ok=None, err="", pvt=None, gga=None)
                with slot.ser_lock:
                    slot.ser = ser
                self._session(slot, ser)
            except (serial.SerialException, OSError) as e:
                if slot:
                    slot.set(conn="lost", err=str(e), pvt=None, gga=None)
            finally:
                if slot:
                    with slot.ser_lock:
                        slot.ser = None
                try:
                    ser.close()
                except (serial.SerialException, OSError):
                    pass
        finally:
            with self.lock:
                if slot:
                    slot.busy = False
                self.claimed.discard(dev)

    def _session(self, slot, ser):
        ok = gm.apply_receiver_config(ser, slot.ser_lock)
        slot.set(conn="open", cfg_ok=ok)
        ser.reset_input_buffer()
        stream = gm.F9PStream()
        while not self.stop_evt.is_set():
            # ★들어온 만큼 바로 읽는다★ read(4096) 은 시간 제한(50 ms)까지 기다려 도착 시각이 뭉개진다
            data = ser.read(max(1, ser.in_waiting))
            if not data:
                continue
            ggas, pvts = stream.feed(data)
            if ggas:
                slot.set(gga=ggas[-1], gga_t=time.time())
                self._pick_gga()
            for p in pvts:
                slot.set(pvt=p)
                self.pairer.feed(slot.no, p)


# ── CSV 기록 ──────────────────────────────────────────────────────────────────────
def side_cols(p):
    """2번 쪽 열 (위도, 경도, 속도, 품질, σ) — 측위가 안 됐으면 빈칸."""
    if not has_fix(p):
        return ["", "", "", "", ""]
    return [f"{p['lat']:.8f}", f"{p['lon']:.8f}", f"{p['speed']:.3f}", gm.gga_quality(p), f"{p['hacc']:.3f}"]


def dual_row(p1, p2, itow0):
    """짝 하나 → DUAL_HEADER 순서의 한 줄. itow0 는 기록 시작 iTOW (t_rel 기준)."""
    a = p1 if has_fix(p1) else None
    b = p2 if has_fix(p2) else None
    ref = a or b
    row = gm.csv_row(a) if a else list(EMPTY_FRONT)
    row += side_cols(b)
    row += [str(a["sv"]) if a else "", str(b["sv"]) if b else "",
            f"{a['hmsl']:.3f}" if a else "", f"{b['hmsl']:.3f}" if b else ""]
    row += [str(ref["itow"]), f"{((ref['itow'] - itow0) % gm.WEEK_MS) / 1000:.1f}"]
    if a and b:
        d, deg = baseline(a, b)
        row += [f"{d:.3f}", f"{deg:.1f}"]
    else:
        row += ["", ""]
    row.append(f"{(p2['t'] - p1['t']) * 1000:.1f}" if p1 is not None and p2 is not None else "")
    return row


class Stat:
    """평균 · 표준편차 누적 (O(1))."""

    def __init__(self):
        self.n, self.s, self.ss = 0, 0.0, 0.0

    def add(self, v):
        self.n += 1
        self.s += v
        self.ss += v * v

    def mean(self):
        return self.s / self.n if self.n else None

    def sd(self):
        if self.n < 2:
            return None
        m = self.s / self.n
        return math.sqrt(max(0.0, self.ss / self.n - m * m))


class DualRecorder:
    """기록 시작/종료와 한 줄 쓰기. GPS 스레드가 on_pair 를, 창이 start/stop 을 부른다."""

    def __init__(self, data_dir):
        self.data_dir = data_dir
        self.lock = threading.Lock()
        self.fp = self.writer = None
        self.path = None
        self.rows = 0
        self.started = 0.0
        self.itow0 = None
        self.bl = Stat()

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
            self.writer.writerow(DUAL_HEADER)
            self.fp.flush()
            self.rows, self.itow0, self.bl = 0, None, Stat()
            self.reset()
            self.started = time.time()
            return self.path

    def stop(self):
        """(경로, 줄 수, 경과 초)"""
        with self.lock:
            if self.fp:
                try:
                    self.fp.close()
                except OSError:
                    pass
            self.fp = self.writer = None
            return self.path, self.rows, time.time() - self.started

    def _write(self, p1, p2):
        """self.lock 을 쥔 채로 부른다."""
        ref = p1 if has_fix(p1) else p2
        if self.itow0 is None:
            self.itow0 = ref["itow"]
        self.writer.writerow(dual_row(p1, p2, self.itow0))
        self.fp.flush()      # 기록 중인 파일도 목록에서 바로 열어 볼 수 있게
        self.rows += 1
        if has_fix(p1) and has_fix(p2):
            self.bl.add(baseline(p1, p2)[0])


class DualRouteRecorder(DualRecorder):
    """매핑 — 일정 거리마다 한 줄. 기준은 1번, 1번이 측위가 안 되면 2번."""

    def __init__(self, data_dir):
        super().__init__(data_dir)
        self.spacing = gm.SPACING_M
        self.length = 0.0                   # 1번 경로 길이 — 목록(summarize_dual_csv)과 같은 셈
        self.last = {1: None, 2: None}      # 번호마다 마지막으로 '기록한' 자리

    def file_name(self, stamp):
        return f"route_dual_{stamp}.csv"

    def reset(self):
        self.length, self.last = 0.0, {1: None, 2: None}

    def summary(self):
        with self.lock:
            return dict(rows=self.rows, length=self.length, bl=self.bl.mean(), bl_sd=self.bl.sd())

    def on_pair(self, p1, p2):
        a = p1 if has_fix(p1) else None
        b = p2 if has_fix(p2) else None
        if not (a or b):
            return
        with self.lock:
            if not self.fp:
                return
            no, ref = (1, a) if a else (2, b)
            last = self.last[no]
            if last is not None:
                d = math.hypot(*gm.delta_m(last[0], last[1], ref["lat"], ref["lon"]))
                if d < self.spacing:
                    return
            self._write(p1, p2)
            if a:
                # ★길이는 1번 경로로만 센다★ 1번이 끊긴 동안 2번으로 잰 거리를 1번이 돌아올 때 또 더하지 않게
                if self.last[1] is not None:
                    self.length += math.hypot(*gm.delta_m(self.last[1][0], self.last[1][1], a["lat"], a["lon"]))
                self.last[1] = (a["lat"], a["lon"])
            if b:
                self.last[2] = (b["lat"], b["lon"])


class DualFixRecorder(DualRecorder):
    """검증 — 일정 시간(iTOW)마다 한 줄."""

    def __init__(self, data_dir):
        super().__init__(data_dir)
        self.interval = INTERVAL_S
        self.last_itow = None
        self.spread = {1: gm.Spread(), 2: gm.Spread()}

    def file_name(self, stamp):
        return f"point_dual_{stamp}_{self.interval:.1f}s.csv"

    def reset(self):
        self.last_itow, self.spread = None, {1: gm.Spread(), 2: gm.Spread()}

    def summary(self):
        with self.lock:
            return dict(rows=self.rows, rms1=self.spread[1].rms(), rms2=self.spread[2].rms(),
                        bl=self.bl.mean(), bl_sd=self.bl.sd())

    def on_pair(self, p1, p2):
        a = p1 if has_fix(p1) else None
        b = p2 if has_fix(p2) else None
        if not (a or b):
            return
        with self.lock:
            if not self.fp:
                return
            itow = (a or b)["itow"]
            if self.last_itow is not None:
                gap = (itow - self.last_itow) % gm.WEEK_MS
                if gap < round(self.interval * 1000) - gm.EPOCH_MS // 2:
                    return
            self._write(p1, p2)
            self.last_itow = itow
            for n, p in ((1, a), (2, b)):
                if p:
                    self.spread[n].add(p["lat"], p["lon"])


def summarize_dual_csv(path):
    """목록용 — 줄 수, 1번 경로 길이, 퍼짐(1번 · 2번), 기선 평균 · 표준편차. 못 읽으면 rows 0."""
    out = dict(rows=0, length=None, rms1=None, rms2=None, bl=None, bl_sd=None)
    sp, bl, length, prev = {1: gm.Spread(), 2: gm.Spread()}, Stat(), 0.0, None
    try:
        with open(path, newline="", encoding="utf-8") as f:
            reader = csv.reader(f)
            header = [h.strip().lower() for h in next(reader, [])]
            col = {name: (header.index(name) if name in header else None)
                   for name in ("latitude", "longitude", "latitude2", "longitude2", "baseline_m")}

            def num(r, name):
                i = col[name]
                try:
                    return float(r[i]) if i is not None and r[i].strip() else None
                except (ValueError, IndexError):
                    return None
            for r in reader:
                if not r:
                    continue
                out["rows"] += 1
                p1 = (num(r, "latitude"), num(r, "longitude"))
                p2 = (num(r, "latitude2"), num(r, "longitude2"))
                if None not in p1:
                    sp[1].add(*p1)
                    if prev:
                        length += math.hypot(*gm.delta_m(prev[0], prev[1], p1[0], p1[1]))
                    prev = p1
                if None not in p2:
                    sp[2].add(*p2)
                b = num(r, "baseline_m")
                if b is not None:
                    bl.add(b)
    except (OSError, UnicodeError, csv.Error):
        return dict(out, rows=0)
    out.update(length=length if prev else None, rms1=sp[1].rms(), rms2=sp[2].rms(), bl=bl.mean(), bl_sd=bl.sd())
    return out


def fmt_bl(mean, sd):
    if mean is None:
        return "-"
    return f"{mean:.3f} m" + (f" ± {sd * 100:.1f} cm" if sd is not None else "")


# ── 창 ────────────────────────────────────────────────────────────────────────────
def rtk_phase(nt, now):
    """NTRIP 상태 한 줄 — gps_mapper.py 의 App.rtk_phase 와 같은 문구."""
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


def describe(s, nt, now):
    """수신기 한 대의 (상태, 색, 오차, 부가 설명) — gps_mapper.py App.describe 와 같은 기준."""
    conn, pvt = s["conn"], s["pvt"]
    if conn == "search":
        return "GPS 없음", gm.GRAY, "", "이 번호의 수신기를 찾는 중 — USB 에 꽂아 주세요"
    if conn == "opening":
        return "GPS 연결 중", gm.BLUE, "", f"{s['port']} 여는 중 · 수신기 설정 적용 중"
    if conn == "lost":
        return "GPS 연결 끊김", gm.RED, "", "USB 가 빠졌거나 포트 오류 — 다시 찾는 중"
    if not pvt or now - pvt["t"] > gm.PVT_STALE_SEC:
        return "GPS 데이터 없음", gm.RED, "", f"{s['port']} 열림 · 수신기 응답 대기"
    if not pvt["ok"] or pvt["fix"] == 0:
        return "위성 신호 없음", gm.RED, "", f"측위 안 됨 (사용 위성 {pvt['sv']}개) — 하늘이 트인 곳에 안테나를 두세요"
    acc = gm.fmt_acc(pvt["hacc"])
    tail = "" if nt.enabled or not pvt["carr"] else " — RTK 꺼짐, 남은 보정이 만료되면 풀림"
    if pvt["carr"] == 2:
        return "RTK Fixed", gm.GREEN, acc, "cm급 측위" + tail
    if pvt["carr"] == 1:
        return "RTK Float", gm.AMBER, acc, "보정 적용 중, Fix 대기 (보통 수십 초)" + tail
    if pvt["diff"]:
        return "DGNSS", gm.ORANGE, acc, "위성(SBAS) 또는 RTK 보정 적용 · m급"
    return "단독 측위", gm.ORANGE, acc, "보정 없음 · m급"


class GpsPanel:
    """수신기 한 대의 상태 칸 — 1번은 왼쪽, 2번은 오른쪽."""

    def __init__(self, parent, no, F):
        self.frame = ttk.LabelFrame(parent, text=f"{no}번 GPS", padding=(10, 6))
        line = ttk.Frame(self.frame)
        line.pack(fill="x")
        self.status = tk.Label(line, text="", font=(F, 17, "bold"))
        self.status.pack(side="left")
        self.acc = tk.Label(line, text="", font=(F, 17, "bold"))
        self.acc.pack(side="left", padx=(12, 0))
        self.sub = tk.Label(self.frame, text="", font=(F, 10), anchor="w", justify="left", wraplength=400)
        self.sub.pack(fill="x", pady=(2, 6))
        grid = ttk.Frame(self.frame)
        grid.pack(fill="x")
        grid.columnconfigure(1, weight=1)
        self.rows = {}
        for r, key in enumerate(("연결", "위성", "위치", "보정")):
            ttk.Label(grid, text=key, width=4, font=(F, 10, "bold")).grid(row=r, column=0, sticky="nw")
            lbl = ttk.Label(grid, text="-", wraplength=380, justify="left")
            lbl.grid(row=r, column=1, sticky="w", pady=1)
            self.rows[key] = lbl
        self.frame.bind("<Configure>", self._wrap)

    def _wrap(self, event):
        w = max(200, event.width - 30)
        self.sub.configure(wraplength=w)
        for lbl in self.rows.values():
            lbl.configure(wraplength=w - 50)

    def show(self, s, nt, now):
        status, color, acc, sub = describe(s, nt, now)
        self.status.configure(text=status, fg=color)
        self.acc.configure(text=acc, fg=color)
        self.sub.configure(text=sub, fg=gm.GRAY if status == "RTK Fixed" else color)
        if s["port"]:
            cfg = {True: "설정 적용됨", False: "⚠ 설정 실패 — Fix 가 늦을 수 있음", None: "설정 전"}[s["cfg_ok"]]
            conn = f"{s['port']} · 칩 ID {s['uid']} · {s['fw'] or '펌웨어 ?'} · {cfg}"
            if s["conn"] == "lost" and s["err"]:
                conn += f" · {s['err']}"
        else:
            conn = "연결된 수신기 없음"
        self.rows["연결"].configure(text=conn)
        pvt = s["pvt"] if s["pvt"] and now - s["pvt"]["t"] <= gm.PVT_STALE_SEC else None
        if pvt and pvt["ok"]:
            sats = f"사용 {pvt['sv']}개 · PDOP {pvt['pdop']:.1f} · 수직 오차 {gm.fmt_acc(pvt['vacc'])}"
        else:
            sats = f"사용 {pvt['sv']}개" if pvt else "-"
        self.rows["위성"].configure(text=sats)
        self.rows["위치"].configure(
            text=(f"{pvt['lat']:.8f}, {pvt['lon']:.8f} · 해발 {pvt['hmsl']:.2f} m · 속도 {pvt['speed']:.2f} m/s"
                  if pvt and pvt["ok"] else "-"))
        gga = s["gga"]
        age = f"보정 나이 {gga['age']} s" if gga and gga["age"] else "보정 나이 -"
        q = {0: "측위 없음", 1: "단독", 2: "DGNSS", 4: "RTK Fixed", 5: "RTK Float"}.get(gga["q"], str(gga["q"])) \
            if gga else "-"
        self.rows["보정"].configure(text=f"GGA 품질 {q} · {age}")


class App:
    def __init__(self, root, worker, recorders):
        self.root, self.worker, self.recorders = root, worker, recorders
        self.mode = "map"
        self.rate_hist = deque(maxlen=40)
        self.file_cache = {}
        self.notes = {m: ("", gm.GRAY) for m in MODES}

        F = gm.pick_font(root)
        root.geometry("980x800")
        root.minsize(820, 640)
        style = ttk.Style(root)
        if not gm.IS_WINDOWS:
            style.theme_use("clam")
        style.configure(".", font=(F, 10))
        style.configure("Treeview.Heading", font=(F, 10, "bold"))
        style.configure("Treeview", rowheight=26)

        # ── 위: 기록 버튼 · 모드 · 간격 (gps_mapper.py 와 같은 자리) ──
        self.top = top = ttk.Frame(root, padding=(14, 12, 14, 4))
        top.pack(fill="x")
        top.columnconfigure(1, weight=1)
        top.bind("<Configure>", self.fit_rec_label)
        self.btn = tk.Button(top, font=(F, 14, "bold"), width=12, bg=gm.GREEN, fg="white",
                             activebackground=gm.GREEN, activeforeground="white",
                             relief="flat", cursor="hand2", command=self.toggle_recording)
        self.btn.grid(row=0, column=0, rowspan=2, sticky="w", padx=(0, 14))
        self.rec_lbl = ttk.Label(top, text="대기 중", wraplength=480, justify="left")
        self.rec_lbl.grid(row=0, column=1, sticky="w")
        seg = tk.Frame(top, bg=gm.SEG_EDGE, padx=1, pady=1)
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
        self.step_vars = {"map": tk.StringVar(value=f"{gm.SPACING_M:.2f}"),
                          "check": tk.StringVar(value=f"{INTERVAL_S:.1f}")}
        self.spin = ttk.Spinbox(sp, width=6, command=self.apply_step)
        self.spin.pack(side="left", padx=4)
        self.unit_lbl = ttk.Label(sp)
        self.unit_lbl.pack(side="left")
        for ev in ("<Return>", "<FocusOut>", "<KeyRelease>"):
            self.spin.bind(ev, lambda e: self.apply_step())
        self.rtk_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(sp, text="RTK 보정 사용 (두 수신기 공통)", variable=self.rtk_var,
                        command=self.apply_rtk).pack(side="left", padx=(18, 0))
        self.step_msg = ttk.Label(sp, text="", foreground=gm.RED)
        self.step_msg.pack(side="left", padx=8)

        # ── 가운데: 1번(왼쪽) · 2번(오른쪽) ──
        pair = ttk.Frame(root, padding=(14, 8, 14, 0))
        pair.pack(fill="x")
        pair.columnconfigure(0, weight=1, uniform="gps")
        pair.columnconfigure(1, weight=1, uniform="gps")
        self.panels = {n: GpsPanel(pair, n, F) for n in SIDES}
        self.panels[1].frame.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self.panels[2].frame.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        # ── 두 GPS 를 함께 보는 줄 · RTK · 기타 포트 ──
        det = ttk.LabelFrame(root, text="두 GPS", padding=(10, 6))
        det.pack(fill="x", padx=14, pady=(10, 0))
        det.columnconfigure(1, weight=1)
        self.detail = {}
        for r, key in enumerate(("기선", "짝", "RTK", "기지국", "포트")):
            ttk.Label(det, text=key, width=5, font=(F, 10, "bold")).grid(row=r, column=0, sticky="nw")
            lbl = ttk.Label(det, text="-", wraplength=820, justify="left")
            lbl.grid(row=r, column=1, sticky="w", pady=1)
            self.detail[key] = lbl

        # ── 아래: 파일 목록 (_dual_ 파일만) ──
        self.data_frame = ttk.LabelFrame(root, padding=(10, 6))
        self.data_frame.pack(fill="both", expand=True, padx=14, pady=(10, 14))
        bar = ttk.Frame(self.data_frame)
        bar.pack(fill="x")
        self.hint_lbl = ttk.Label(bar, foreground=gm.GRAY)
        self.hint_lbl.pack(side="left")
        ttk.Button(bar, text="폴더 열기", command=self.open_data_folder).pack(side="right")
        body = ttk.Frame(self.data_frame)
        body.pack(fill="both", expand=True, pady=(6, 0))
        self.tree = ttk.Treeview(body, columns=[c[0] for c in TREE_COLS], show="tree headings", selectmode="none")
        self.tree.heading("#0", text="파일", anchor="w")
        for col, text, width, anchor in TREE_COLS:
            self.tree.heading(col, text=text)
            self.tree.column(col, width=width, anchor=anchor)
        vsb = ttk.Scrollbar(body, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        self.tree.pack(side="left", fill="both", expand=True)
        self.tree.tag_configure("live", foreground=gm.RED)
        self.tree.bind("<ButtonRelease-1>", self.on_tree_click)
        self.tree.configure(cursor="hand2")

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.apply_mode()
        self.tick()
        self.refresh_files()

    @property
    def rec(self):
        return self.recorders[self.mode]

    # ── 모드 ──
    def on_mode(self):
        new = self.mode_var.get()
        if new == self.mode:
            return
        if self.rec.active:
            self.mode_var.set(self.mode)
            return
        self.mode = new
        self.apply_mode()

    def apply_mode(self):
        spec = MODES[self.mode]
        self.root.title(f"GPS 매퍼 (듀얼) — {spec['name']}")
        self.btn.configure(text=spec["start"], bg=gm.GREEN, activebackground=gm.GREEN)
        self.spin.configure(textvariable=self.step_vars[self.mode], from_=spec["lo"], to=spec["hi"],
                            increment=spec["step"], format=spec["fmt"])
        self.spin.state(["!disabled"])
        self.unit_lbl.configure(text=spec["unit"])
        self.apply_step()
        self.data_frame.configure(text=f"{spec['data']} (듀얼) — {self.rec.data_dir.name}/{FILE_GLOB[self.mode]}")
        self.hint_lbl.configure(text=spec["hint"])
        self.tree.configure(displaycolumns=spec["cols"])
        self.tree.column("#0", width=spec["name_w"])
        for col, _, width, _ in TREE_COLS:
            self.tree.column(col, width=width)
        self.tree.delete(*self.tree.get_children())
        self.paint_mode_buttons()
        self.show_progress()
        self.refresh_files(force=True)

    def paint_mode_buttons(self):
        locked = self.rec.active
        for key, b in self.mode_btns.items():
            on = key == self.mode
            bg, fg = (gm.BLUE, "white") if on else (gm.SEG_OFF_BG, gm.SEG_OFF_FG)
            b.configure(bg=bg, fg=fg, selectcolor=bg, activebackground=bg, activeforeground=fg,
                        disabledforeground="white" if on else "#adb5bd",
                        state="disabled" if locked else "normal",
                        cursor="arrow" if locked or on else "hand2")

    # ── 기록 ──
    def toggle_recording(self):
        rec, spec = self.rec, MODES[self.mode]
        if rec.active:
            path, rows, el = rec.stop()
            s = rec.summary()
            if rows < 2:
                note = (f"⚠ {path.name} — {rows}줄뿐", gm.RED)
            elif self.mode == "map":
                note = (f"저장됨: {path.name} · {rows}줄 · {s['length']:.1f} m · 기선 {fmt_bl(s['bl'], s['bl_sd'])}",
                        gm.GREEN)
            else:
                note = (f"저장됨: {path.name} · {rows}줄 · {gm.fmt_elapsed(el)} · 퍼짐 1번 {gm.fmt_cm(s['rms1'])} · "
                        f"2번 {gm.fmt_cm(s['rms2'])} · 기선 {fmt_bl(s['bl'], s['bl_sd'])}", gm.GREEN)
            self.notes[self.mode] = note
            self.btn.configure(text=spec["start"], bg=gm.GREEN, activebackground=gm.GREEN)
            self.spin.state(["!disabled"])
        else:
            if not self.apply_step():
                return
            try:
                rec.start()
            except OSError as e:
                messagebox.showerror(f"{spec['start']} 실패", f"CSV 파일을 만들 수 없습니다.\n{e}")
                return
            self.btn.configure(text=spec["stop"], bg=gm.RED, activebackground=gm.RED)
            if self.mode == "check":
                self.spin.state(["disabled"])
        self.paint_mode_buttons()
        self.show_progress()
        self.refresh_files(force=True)

    def apply_step(self):
        if self.mode == "check" and self.rec.active:
            return True
        spec = MODES[self.mode]
        try:
            v = float(self.step_vars[self.mode].get())
        except ValueError:
            v = None
        ok = v is not None and spec["lo"] <= v <= spec["hi"]
        if ok and self.mode == "check":
            ok = abs(v * 10 - round(v * 10)) <= 1e-6
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

    # ── 상태 표시 ──
    def tick(self):
        now = time.time()
        nt = self.worker.ntrip
        snaps = {n: self.worker.slots[n].snapshot() for n in SIDES}
        for n in SIDES:
            self.panels[n].show(snaps[n], nt, now)

        # 기선 — 가장 최근 짝, 그리고 최근 10 초 흔들림
        count, paired, dt, bl_mean, bl_sd = self.worker.pair_stats()
        last = self.worker.last_pair
        if last and has_fix(last[0]) and has_fix(last[1]) and now - last[0]["t"] <= gm.PVT_STALE_SEC:
            d, deg = baseline(last[0], last[1])
            dz = last[1]["hmsl"] - last[0]["hmsl"]
            line = f"1번 → 2번  {d:.3f} m · 방위 {deg:.1f}° · 높이차 {dz:+.3f} m"
            if bl_mean is not None:
                line += f"   (최근 {STATS_WINDOW_S:.0f}초 평균 {fmt_bl(bl_mean, bl_sd)})"
        else:
            line = "두 GPS 가 모두 측위돼야 잴 수 있음"
        self.detail["기선"].configure(text=line)
        live = [n for n in SIDES if snaps[n]["conn"] == "open"]
        if len(live) < 2:
            pline = (f"{live[0]}번만 연결됨 — 짝 없이 {live[0]}번 값만 기록된다" if live
                     else "연결된 수신기 없음")
        elif count:
            pline = (f"최근 {STATS_WINDOW_S:.0f}초 {count}개 측위 중 같은 iTOW 짝 {paired * 100:.0f}%"
                     + (f" · PC 도착 시각 차(2번−1번) 평균 {dt:+.1f} ms" if dt is not None else ""))
        else:
            pline = "아직 받은 측위 없음"
        self.detail["짝"].configure(text=pline)

        self.rate_hist.append((now, nt.bytes_in))
        t0, b0 = self.rate_hist[0]
        rate = (nt.bytes_in - b0) / (now - t0) if now - t0 > 0.5 else 0.0
        rtk = f"{gm.NTRIP['host']}:{gm.NTRIP['port']}/{gm.NTRIP['mountpoint']} (ID {gm.NTRIP['user']}) · {rtk_phase(nt, now)}"
        if not nt.enabled:
            rtk += " — 체크하면 접속"
        elif nt.status == "streaming" and nt.session_bytes:
            on = [str(n) for n in SIDES if snaps[n]["conn"] == "open"]
            rtk += f" · {rate / 1000:.2f} KB/s · {', '.join(on) or '-'}번에 넣는 중"
        if self.worker.ntrip_gga_from:
            rtk += f" · 위치(GGA)는 {self.worker.ntrip_gga_from}번"
        if nt.enabled and nt.status in ("retry", "fatal") and nt.detail:
            rtk += f" · 원인: {nt.detail}"
        self.detail["RTK"].configure(text=rtk)
        here = None                          # 기지국까지 거리 — 1번이 측위 중이면 1번, 아니면 2번 자리에서
        for n in SIDES:
            p = snaps[n]["pvt"]
            if p and p["ok"] and now - p["t"] <= gm.PVT_STALE_SEC:
                here = (p["lat"], p["lon"])
                break
        self.detail["기지국"].configure(text=gm.describe_base(nt.rtcm.snapshot(), here, now) if nt.enabled else "-")

        notes = self.worker.notes()
        self.detail["포트"].configure(
            text=" / ".join(f"{dev}: {msg}" for dev, msg in sorted(notes.items())) if notes
            else f"등록된 칩 ID — 1번 {GPS1_ID} · 2번 {GPS2_ID}")

        self.show_progress()
        self.root.after(200, self.tick)

    def show_progress(self):
        rec = self.rec
        if rec.active:
            s = rec.summary()
            msg = f"● 기록 중  {rec.path.name} · {s['rows']}줄"
            if self.mode == "map":
                msg += f" · {s['length']:.1f} m"
            msg += f" · {gm.fmt_elapsed(time.time() - rec.started)}"
            if s["rows"] == 0:
                msg += "  (측위 대기 — 아직 기록된 줄 없음)"
            elif self.mode == "check":
                msg += f" · 퍼짐 1번 {gm.fmt_cm(s['rms1'])} · 2번 {gm.fmt_cm(s['rms2'])}"
            if s["rows"]:
                msg += f" · 기선 {fmt_bl(s['bl'], s['bl_sd'])}"
            missing = [str(n) for n in SIDES if self.worker.slots[n].snapshot()["conn"] != "open"]
            if missing:
                msg += f"  ⚠ {', '.join(missing)}번 GPS 없음 — 그쪽 열은 빈칸"
            self.rec_lbl.configure(text=msg, foreground=gm.RED)
        else:
            text, col = self.notes[self.mode]
            self.rec_lbl.configure(text=text or "대기 중", foreground=col)

    def fit_rec_label(self, _event=None):
        width = self.top.grid_bbox(1, 0)[2]
        if width > 50:
            self.rec_lbl.configure(wraplength=width - 4)

    # ── 파일 목록 ──
    def refresh_files(self, force=False):
        rec = self.rec
        try:
            files = sorted(rec.data_dir.glob(FILE_GLOB[self.mode]), key=lambda p: p.stat().st_mtime, reverse=True)
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
                s = dict(rec.summary())
                s.setdefault("length", None)
                s.setdefault("rms1", None)
                s.setdefault("rms2", None)
            else:
                key = (stt.st_mtime_ns, stt.st_size)
                cached = self.file_cache.get(p)
                if force or not cached or cached[0] != key:
                    cached = (key, summarize_dual_csv(p))
                    self.file_cache[p] = cached
                s = cached[1]
            values = (f"{s['rows']}", "-" if s["length"] is None else f"{s['length']:.1f} m",
                      gm.fmt_cm(s["rms1"]), gm.fmt_cm(s["rms2"]), fmt_bl(s["bl"], s["bl_sd"]),
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
        rows = summarize_dual_csv(path)["rows"]
        if rows < 2:
            messagebox.showinfo("find_wc", f"{iid}\n{rows}줄이라 지도에 그릴 수 없습니다.")
            return
        try:
            gm.open_in_find_wc(path)
        except OSError as e:
            messagebox.showerror("find_wc 열기 실패", str(e))

    def open_data_folder(self):
        try:
            gm.open_folder(self.rec.data_dir)
        except OSError as e:
            messagebox.showerror("폴더 열기 실패", str(e))

    def on_close(self):
        for rec in self.recorders.values():
            if rec.active:
                rec.stop()
        self.worker.stop()
        self.root.after(150, self.root.destroy)


def main():
    if gm.IS_WINDOWS:
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)   # 고해상도 화면에서 글자가 번지지 않게
        except (AttributeError, OSError):
            pass
    recorders = {"map": DualRouteRecorder(gm.MAP_DIR), "check": DualFixRecorder(gm.CHECK_DIR)}
    for rec in recorders.values():
        rec.data_dir.mkdir(exist_ok=True)
    worker = DualWorker(list(recorders.values()))
    worker.start()
    root = tk.Tk()
    App(root, worker, recorders)
    root.mainloop()


if __name__ == "__main__":
    main()
