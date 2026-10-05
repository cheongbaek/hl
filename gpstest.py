#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import base64
import math
import socket
import statistics as st
import struct
import sys
import threading
import time

import serial

CONFIG = {
    "serial_port": "COM10",
    "baud": 38400,
    "host": "rts1.ngii.go.kr",
    "port": 2101,
    "mountpoint": "VRS-RTCM34",
    "user": "knut_gps",
    "password": "ngii",
}
GGA_UPLOAD_SEC = 10    # 캐스터로 GGA 올리는 주기
WAIT_FIX_SEC = 180     # RTK Fix 대기 상한
MEASURE_SEC = 20       # 흔들림 측정 시간
NO_DATA_SEC = 10       # 접속 후 이 시간 동안 RTCM이 없으면 $GPGGA로 바꿔 다시 보냄

# 실행할 때마다 RAM 레이어에만 쓰는 수신기 설정: (키 이름, 키 ID, 바이트 수, 값). 이유는 파일 끝 주석.
RECEIVER_RAM_CONFIG = [
    ("CFG-RATE-MEAS", 0x30210001, 2, 100),
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
]

QUALITY = {0: "무효", 1: "단독", 2: "DGNSS/SBAS", 4: "RTK Fix", 5: "RTK Float", 6: "추측항법"}
CARR = {0: "없음", 1: "Float", 2: "Fix"}
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


# ---------------------------------------------------------------- 프로토콜 도우미
def ubx(cls, id_, payload=b""):
    body = bytes([cls, id_]) + struct.pack("<H", len(payload)) + payload
    a = b = 0
    for x in body:
        a = (a + x) & 0xFF
        b = (b + a) & 0xFF
    return b"\xb5\x62" + body + bytes([a, b])


def nmea_checksum(body):
    c = 0
    for ch in body.encode("ascii", "replace"):
        c ^= ch
    return c


def nmea_ok(line):
    star = line.rfind("*")
    if not line.startswith("$") or star < 0 or len(line) < star + 3:
        return False
    try:
        return nmea_checksum(line[1:star]) == int(line[star + 1:star + 3], 16)
    except ValueError:
        return False


def to_gp_talker(line):
    """$GNGGA → $GPGGA (구형 캐스터가 GN 토커를 못 받는 경우 대비)."""
    body = "GP" + line[3:line.rfind("*")]
    return f"${body}*{nmea_checksum(body):02X}"


def nmea_deg(v, hemi):
    d = int(float(v) / 100)
    deg = d + (float(v) - d * 100) / 60
    return -deg if hemi in "SW" else deg


def parse_gga(line):
    f = line[:line.rfind("*")].split(",")
    if len(f) < 15 or not f[2] or not f[4] or not f[9]:
        return None
    return dict(raw=line, t=time.time(), utc=f[1],
                lat=nmea_deg(f[2], f[3]), lon=nmea_deg(f[4], f[5]),
                q=int(f[6] or 0), sats=int(f[7] or 0), hdop=float(f[8] or 99.99),
                alt=float(f[9]), sep=float(f[11] or 0), age=f[13], sid=f[14])


def parse_pvt(p):
    flags = p[21]
    return dict(fix=p[20], carr=(flags >> 6) & 3, sv=p[23])


def parse_hppos(p):
    if p[3] & 1:  # invalidLlh
        return None
    lon, lat, h, hmsl = struct.unpack_from("<iiii", p, 8)
    lon_hp, lat_hp, h_hp, hmsl_hp = struct.unpack_from("<bbbb", p, 24)
    hacc, vacc = struct.unpack_from("<II", p, 28)
    return dict(t=time.time(),
                lat=lat * 1e-7 + lat_hp * 1e-9, lon=lon * 1e-7 + lon_hp * 1e-9,
                h=(h + h_hp * 0.1) / 1000, hmsl=(hmsl + hmsl_hp * 0.1) / 1000,
                hacc=hacc / 1e4, vacc=vacc / 1e4)


def crc24q(data):
    crc = 0
    for byte in data:
        crc ^= byte << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def getbits(val, total, pos, n, signed=False):
    v = (val >> (total - pos - n)) & ((1 << n) - 1)
    if signed and v & (1 << (n - 1)):
        v -= 1 << n
    return v


def parse_1005(payload):
    """RTCM 1005/1006 → 기준국(VRS) ECEF 좌표(m)."""
    val, total = int.from_bytes(payload, "big"), len(payload) * 8
    x = getbits(val, total, 34, 38, True) * 1e-4
    y = getbits(val, total, 74, 38, True) * 1e-4
    z = getbits(val, total, 114, 38, True) * 1e-4
    return x, y, z


def ecef_to_llh(x, y, z):
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1 - WGS84_E2))
    h = 0.0
    for _ in range(6):
        n = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1 - WGS84_E2 * n / (n + h)))
    return math.degrees(lat), math.degrees(lon), h


def local_radii(lat_deg):
    s = math.sin(math.radians(lat_deg))
    rn = WGS84_A / math.sqrt(1 - WGS84_E2 * s * s)
    rm = rn * (1 - WGS84_E2) / (1 - WGS84_E2 * s * s)
    return rn, rm


def horiz_dist(lat1, lon1, lat2, lon2):
    rn, rm = local_radii((lat1 + lat2) / 2)
    de = math.radians(lon2 - lon1) * rn * math.cos(math.radians((lat1 + lat2) / 2))
    dn = math.radians(lat2 - lat1) * rm
    return math.hypot(de, dn)


# ---------------------------------------------------------------- 수신기 설정
def apply_receiver_config(ser):
    """RECEIVER_RAM_CONFIG를 CFG-VALSET으로 RAM 레이어에만 쓴다. ACK를 받으면 True."""
    payload = bytes([0, 0x01, 0, 0])  # version 0, layer RAM
    for _, key, size, value in RECEIVER_RAM_CONFIG:
        payload += struct.pack("<I", key) + value.to_bytes(size, "little")
    msg = ubx(0x06, 0x8A, payload)
    ack = b"\xb5\x62\x05\x01\x02\x00\x06\x8a"
    nak = b"\xb5\x62\x05\x00\x02\x00\x06\x8a"
    for _ in range(5):  # 과부하 상태에서는 ACK가 빠질 수 있어 재시도
        ser.reset_input_buffer()
        ser.write(msg)
        buf, t0 = b"", time.time()
        while time.time() - t0 < 1.0:
            buf += ser.read(4096)
            if ack in buf:
                return True
            if nak in buf:
                return False
    return False


# ---------------------------------------------------------------- F9P 수신
class F9PReader:
    """시리얼 바이트를 NMEA GGA 줄과 UBX(NAV-PVT, NAV-HPPOSLLH) 프레임으로 나눈다."""

    def __init__(self, ser):
        self.ser = ser
        self.buf = b""

    def read(self):
        self.buf += self.ser.read(4096)
        b, i = self.buf, 0
        ggas, pvts, hps = [], [], []
        while i < len(b):
            if b[i] == 0xB5:
                if len(b) - i < 8:
                    break
                if b[i + 1] == 0x62:
                    n = struct.unpack_from("<H", b, i + 4)[0]
                    if n <= 1024:
                        if len(b) - i < 8 + n:
                            break
                        cls_id, p = b[i + 2:i + 4], b[i + 6:i + 6 + n]
                        if cls_id == b"\x01\x07" and n == 92:
                            pvts.append(parse_pvt(p))
                        elif cls_id == b"\x01\x14" and n == 36:
                            hp = parse_hppos(p)
                            if hp:
                                hps.append(hp)
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
        return ggas, pvts, hps


# ---------------------------------------------------------------- NTRIP 클라이언트
class NtripFatal(Exception):
    pass


class NtripClient(threading.Thread):
    """캐스터에서 받은 RTCM3를 그대로 시리얼에 쓰는 스레드. gga_raw에 최신 GGA를 넣어 두면 주기적으로 올린다."""

    def __init__(self, cfg, ser, ser_lock):
        super().__init__(daemon=True)
        self.cfg, self.ser, self.ser_lock = cfg, ser, ser_lock
        self.sock = None
        self.gga_raw = None
        self.gp_talker = False
        self.status = "접속 전"
        self.fatal = None
        self.bytes_in = 0
        self.gga_sent = 0
        self.connected_at = None
        self.msg_count = {}
        self.base_ecef = None
        self.rbuf = b""
        self.stop_evt = threading.Event()

    def stop(self):
        self.stop_evt.set()
        self._close()

    def run(self):
        fails = 0
        while not self.stop_evt.is_set():
            try:
                self._connect()
                fails = 0
                self._stream()
            except NtripFatal as e:
                self.fatal = str(e)
                self.status = "중단"
                return
            except OSError as e:
                if self.stop_evt.is_set():
                    return
                self.status = f"연결 끊김({e})"
                print(f"  [NTRIP] {self.status} — 3초 후 재접속", flush=True)
            self._close()
            fails += 1
            if fails > 5:
                self.fatal = "재접속 5회 실패"
                return
            self.stop_evt.wait(3)

    def _connect(self):
        c = self.cfg
        auth = base64.b64encode(f"{c['user']}:{c['password']}".encode()).decode()
        req = (f"GET /{c['mountpoint']} HTTP/1.0\r\n"
               f"Host: {c['host']}\r\n"
               f"User-Agent: NTRIP gpstest/1.0\r\n"
               f"Authorization: Basic {auth}\r\n"
               f"Accept: */*\r\n"
               f"Connection: close\r\n\r\n")
        self.status = "접속 중"
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
            raise NtripFatal(f"마운트포인트 '{c['mountpoint']}'가 없음 (sourcetable 응답)")
        elif "401" in first:
            raise NtripFatal("인증 실패(401) — ID/비밀번호, 계정 반영 대기(다음 날 9시), 다른 곳 동시접속 여부 확인")
        else:
            raise NtripFatal(f"예상 밖 응답: {first}")
        self.status = f"접속됨({first})"
        self.connected_at = time.time()
        print(f"  [NTRIP] {c['host']}:{c['port']}/{c['mountpoint']} → {first}", flush=True)
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
            if not self.gp_talker and self.bytes_in == 0 and now - self.connected_at > NO_DATA_SEC:
                self.gp_talker = True
                print(f"  [NTRIP] {NO_DATA_SEC}초간 RTCM 없음 → $GPGGA 형식으로 다시 보냄", flush=True)
                self._send_gga()
            try:
                data = self.sock.recv(4096)
            except socket.timeout:
                continue
            if not data:
                raise OSError("캐스터가 연결을 닫음")
            self._feed(data)

    def _send_gga(self):
        line = self.gga_raw
        if not line or not self.sock:
            return
        if self.gp_talker:
            line = to_gp_talker(line)
        self.sock.sendall((line + "\r\n").encode("ascii"))
        self.gga_sent += 1

    def _feed(self, data):
        self.bytes_in += len(data)
        with self.ser_lock:
            self.ser.write(data)
        self._count_rtcm(data)

    def _count_rtcm(self, data):
        b = self.rbuf + data
        i = 0
        while True:
            j = b.find(b"\xd3", i)
            if j < 0:
                i = len(b)
                break
            if len(b) - j < 6:
                i = j
                break
            n = ((b[j + 1] & 0x03) << 8) | b[j + 2]
            if len(b) - j < n + 6:
                i = j
                break
            frame = b[j:j + 3 + n]
            if n < 2 or crc24q(frame) != int.from_bytes(b[j + 3 + n:j + 6 + n], "big"):
                i = j + 1
                continue
            payload = frame[3:]
            mtype = (payload[0] << 4) | (payload[1] >> 4)
            self.msg_count[mtype] = self.msg_count.get(mtype, 0) + 1
            if mtype in (1005, 1006) and n >= 19:
                self.base_ecef = parse_1005(payload)
            i = j + 6 + n
        self.rbuf = b[i:]

    def _close(self):
        s, self.sock = self.sock, None
        if s:
            try:
                s.close()
            except OSError:
                pass


# ---------------------------------------------------------------- 통계·출력
def enu_stats(pts):
    """(lat, lon, h) 목록 → 평균 위치와 평균 기준 동/북/높이 흔들림(m)."""
    lat0 = st.fmean(p[0] for p in pts)
    lon0 = st.fmean(p[1] for p in pts)
    h0 = st.fmean(p[2] for p in pts)
    rn, rm = local_radii(lat0)
    e = [math.radians(p[1] - lon0) * rn * math.cos(math.radians(lat0)) for p in pts]
    n = [math.radians(p[0] - lat0) * rm for p in pts]
    u = [p[2] - h0 for p in pts]
    hor = sorted(math.hypot(a, b) for a, b in zip(e, n))
    return dict(n=len(pts), lat=lat0, lon=lon0, h=h0,
                sd=(st.pstdev(e), st.pstdev(n), st.pstdev(u)),
                p2p=(max(e) - min(e), max(n) - min(n), max(u) - min(u)),
                drift=(e[-1] - e[0], n[-1] - n[0], u[-1] - u[0]),
                cep50=hor[len(hor) // 2], r95=hor[min(len(hor) - 1, int(len(hor) * 0.95))], rmax=hor[-1])


def cm(v):
    return f"{v * 100:6.1f}"


def print_stats(title, s):
    print(f"\n[{title}] 샘플 {s['n']}개")
    print(f"  평균 위치 : {s['lat']:.8f}, {s['lon']:.8f}, 높이 {s['h']:.3f} m")
    print("              동(E)   북(N)  높이(U)   [cm]")
    print(f"  표준편차  : {cm(s['sd'][0])}  {cm(s['sd'][1])}  {cm(s['sd'][2])}")
    print(f"  최대폭    : {cm(s['p2p'][0])}  {cm(s['p2p'][1])}  {cm(s['p2p'][2])}")
    print(f"  처음→끝   : {cm(s['drift'][0])}  {cm(s['drift'][1])}  {cm(s['drift'][2])}")
    print(f"  수평 반경 : 50% {cm(s['cep50'])} / 95% {cm(s['r95'])} / 최대 {cm(s['rmax'])} cm")


def status_line(elapsed, gga, pvt, hp, ntrip):
    q = gga["q"] if gga else 0
    s = f"[{elapsed:5.1f}s] {QUALITY.get(q, q):10s}"
    if gga:
        s += f" 위성 {gga['sats']:2d}  HDOP {gga['hdop']:.2f}  보정나이 {gga['age'] or '-':>4}s"
    if pvt:
        s += f"  carr={CARR.get(pvt['carr'])}"
    if hp:
        s += f"  hAcc {hp['hacc'] * 100:.1f}cm vAcc {hp['vacc'] * 100:.1f}cm"
    s += f"  RTCM {ntrip.bytes_in}B"
    return s


# ---------------------------------------------------------------- 메인
def main():
    cfg = CONFIG
    ser = serial.Serial(cfg["serial_port"], cfg["baud"], timeout=0.05)
    if apply_receiver_config(ser):
        print("수신기 설정(RAM) 적용: UART1·I2C NMEA 끔, USB는 GGA·RMC만, 10 Hz", flush=True)
    else:
        print("경고: 수신기 설정 적용 실패(ACK 없음) — 출력 과부하로 Fix가 늦거나 안 될 수 있음", flush=True)
    ser.reset_input_buffer()
    ser_lock = threading.Lock()
    reader = F9PReader(ser)
    ntrip = NtripClient(cfg, ser, ser_lock)
    gga = pvt = hp = None
    last_poll = 0.0

    def pump():
        """시리얼을 한 번 읽고, 1초마다 UBX를 조회하고, 최신 GGA를 NTRIP 쪽에 넘긴다."""
        nonlocal gga, pvt, hp, last_poll
        now = time.time()
        if now - last_poll >= 1.0:
            with ser_lock:
                ser.write(ubx(0x01, 0x07) + ubx(0x01, 0x14))
            last_poll = now
        ggas, pvts, hps = reader.read()
        for g in ggas:
            gga = g
            if g["q"] > 0:
                ntrip.gga_raw = g["raw"]
        if pvts:
            pvt = pvts[-1]
        if hps:
            hp = hps[-1]
        return ggas, hps

    try:
        # 1) F9P 자체 측위가 될 때까지 대기 (VRS 생성에 위치가 필요)
        print(f"F9P({cfg['serial_port']}) 위치 대기...", flush=True)
        t0 = time.time()
        while not ntrip.gga_raw:
            pump()
            if time.time() - t0 > 60:
                sys.exit("60초 동안 유효한 GGA가 없음 — 안테나/위성 상태 확인")
        print(f"  시작 위치: {gga['raw']}", flush=True)

        # 2) NTRIP 접속 → RTK Fix 대기
        ntrip.start()
        t_start = time.time()
        first_seen = {}
        last_print = 0.0
        while True:
            pump()
            now = time.time()
            if ntrip.fatal:
                sys.exit(f"NTRIP 오류: {ntrip.fatal}")
            if gga:
                first_seen.setdefault(gga["q"], now - t_start)
            if now - last_print >= 2:
                print(status_line(now - t_start, gga, pvt, hp, ntrip), flush=True)
                last_print = now
            if gga and gga["q"] == 4:
                print(f"\n★ RTK Fix — NTRIP 접속 후 {now - t_start:.1f}초", flush=True)
                break
            if now - t_start > WAIT_FIX_SEC:
                print(f"\n{WAIT_FIX_SEC}초 안에 Fix 안 됨 → 현재 상태로 측정", flush=True)
                break

        # 3) 20초 측정
        print(f"{MEASURE_SEC}초 측정 중...", flush=True)
        rec_gga, rec_hp = [], []
        t_meas = time.time()
        last_print = 0.0
        while time.time() - t_meas < MEASURE_SEC:
            ggas, hps = pump()
            rec_gga += [g for g in ggas if g["q"] > 0]
            rec_hp += hps
            if time.time() - last_print >= 5:
                print(status_line(time.time() - t_start, gga, pvt, hp, ntrip), flush=True)
                last_print = time.time()
    finally:
        ntrip.stop()
        ser.close()

    # 4) 결과
    print("\n================ 결과 ================")
    print(f"캐스터     : {cfg['host']}:{cfg['port']}/{cfg['mountpoint']} (GGA {ntrip.gga_sent}회 전송"
          f"{', $GPGGA 사용' if ntrip.gp_talker else ''})")
    dur = max(time.time() - (ntrip.connected_at or t_start), 1)
    print(f"RTCM 수신  : {ntrip.bytes_in} B ({ntrip.bytes_in / dur:.0f} B/s)  메시지별 개수: "
          + ", ".join(f"{k}×{v}" for k, v in sorted(ntrip.msg_count.items())))
    for q in (2, 5, 4):
        if q in first_seen:
            print(f"처음 {QUALITY[q]:9s}: 접속 후 {first_seen[q]:.1f}초")
    qs = [g["q"] for g in rec_gga]
    print("측정 중 품질: " + ", ".join(f"{QUALITY.get(q, q)} {qs.count(q) / len(qs) * 100:.0f}%"
                                   for q in sorted(set(qs))) if qs else "측정 중 품질: 없음")
    if rec_gga:
        print(f"위성/HDOP  : {min(g['sats'] for g in rec_gga)}~{max(g['sats'] for g in rec_gga)}개 / "
              f"{min(g['hdop'] for g in rec_gga):.2f}~{max(g['hdop'] for g in rec_gga):.2f}"
              f"   보정나이 {rec_gga[-1]['age'] or '-'}s, 기준국 ID {rec_gga[-1]['sid'] or '-'}")
    if rec_hp:
        print(f"수신기 추정: hAcc {min(h['hacc'] for h in rec_hp) * 100:.1f}~{max(h['hacc'] for h in rec_hp) * 100:.1f} cm, "
              f"vAcc {min(h['vacc'] for h in rec_hp) * 100:.1f}~{max(h['vacc'] for h in rec_hp) * 100:.1f} cm")

    best = None
    if len(rec_gga) >= 2:
        best = s = enu_stats([(g["lat"], g["lon"], g["alt"]) for g in rec_gga])
        print_stats("GGA 10Hz, 높이=해발", s)
    if len(rec_hp) >= 2:
        best = s = enu_stats([(h["lat"], h["lon"], h["hmsl"]) for h in rec_hp])
        print_stats("NAV-HPPOSLLH 1Hz, 0.1mm 해상도, 높이=해발", s)
        print(f"  타원체고 평균: {st.fmean(h['h'] for h in rec_hp):.3f} m")

    if ntrip.base_ecef and best:
        blat, blon, bh = ecef_to_llh(*ntrip.base_ecef)
        print(f"\nVRS 기준점 : {blat:.8f}, {blon:.8f}, 타원체고 {bh:.3f} m"
              f"  → 수평거리 {horiz_dist(best['lat'], best['lon'], blat, blon):.2f} m")

    if best:
        print(f"\n구글지도   : https://www.google.com/maps/search/?api=1&query={best['lat']:.8f},{best['lon']:.8f}")
    if rec_gga:
        print("\n--- 원본 GGA (처음 3 / 마지막 3) ---")
        print("\n".join(g["raw"] for g in rec_gga[:3]))
        print("...")
        print("\n".join(g["raw"] for g in rec_gga[-3:]))


if __name__ == "__main__":
    main()


# =====================================================================================
# 설명 (2026-10-05 측정 기준)
# =====================================================================================
#
# ■ gpstest.py — ZED-F9P + 국토지리정보원(NGII) NTRIP VRS로 RTK 측위를 하고 흔들림을 잰다.
#   1) 수신기 출력 설정을 RAM에 쓴다(RECEIVER_RAM_CONFIG, 아래 "수신기 설정" 참고).
#   2) F9P(COM10)의 NMEA GGA로 현재 위치를 얻는다.
#   3) NGII 캐스터(CONFIG)에 NTRIP v1로 접속해 GGA를 10초마다 올려 보내고
#      (VRS는 이 위치에 가상기준점을 만든다), 내려오는 RTCM3 보정정보를 그대로 F9P에 써 넣는다.
#   4) RTK Fix(GGA 품질 4)가 되면 20초 동안 위치를 기록해 흔들림을 계산하고 구글지도 링크를 출력한다.
#      제한 시간(WAIT_FIX_SEC) 안에 Fix가 안 되면 그 상태 그대로 측정한다.
#   UBX는 1초마다 '조회(poll)'만 한다:
#     NAV-PVT      → RTK 상태(carrSoln), 사용 위성 수
#     NAV-HPPOSLLH → 0.1mm 해상도 위치·hAcc/vAcc (GGA는 소수 5자리 분이라 위도 약 1.9cm 단위로 끊긴다)
#
# ■ 수신기 설정을 실행할 때마다 RAM에만 쓰는 이유
#   - 플래시/BBR에는 저장하지 않는다. F9P 전원을 뺐다 꽂으면 공장 설정으로 돌아가므로 매번 다시 쓴다.
#   - 공장 설정은 NMEA 6종(RMC·VTG·GGA·GSA·GSV·GLL)을 10 Hz로 USB·UART1(38400bps)·I2C에 똑같이 내보낸다.
#     아무것도 연결 안 된 UART1이 가장 느린 출구라 출력 버퍼가 가득 차서:
#       · `$GNTXT,01,01,00,txbuf alloc` 경고가 1초에 1번 나온다.
#       · GSV·GLL 문장과 UBX 조회 응답이 대부분 빠진다 (12초 동안 NAV-SAT 0/12, MON-COMMS 1/12, GLL 0개).
#       · RTCM 보정을 띄엄띄엄 쓴다 — 보정나이 0~13.6초, 품질이 RTK Float와 SBAS 사이를 오간다.
#         이 상태로는 180초 동안 Fix가 안 됐고, Float 결과도 나중 Fix 위치와 약 3.1m 어긋났다
#         (수신기가 추정한 hAcc는 30cm였다 — Float의 hAcc는 믿을 수 없다).
#   - UART1 속도만 115200으로 올려 봤지만 USB 출력이 초당 약 11.7KB(= 115200bps 한계)에서 막히고
#     경고도 그대로였다. 가장 느린 포트가 전체 출력 속도를 붙잡는다.
#   - UART1·I2C의 NMEA를 끄고 USB는 GGA·RMC만 남기자(10 Hz 유지):
#     경고 0회, UBX 조회 응답 12/12, 보정나이 0.2~1.1초, Float 2.6초 · Fix 39.6초, hAcc 2~3cm.
#     RTCM 33,176바이트를 보내 수신기가 33,192바이트(UBX 조회 16바이트 포함)를 받고, RTCM3 277개 해석, 버린 바이트 0.
#   - USB의 UBX 출력·RTCM3 입력과 10 Hz(CFG-RATE-MEAS=100ms)는 기본값과 같지만, 이 스크립트가 기대는
#     값이라 같이 써 둔다. GSV(위성 목록)가 필요하면 NMEA_ID_GSV_USB만 1로 바꾸고 UART1·I2C는 계속 끈다.
#
# ■ NGII 실시간 측위보정정보(RTS) 메모
#   - 계정: GNSS 서비스포털(geodesy.ngii.go.kr) → 마이페이지 → 통합회원 연계 → 신규등록으로 만든 ID.
#     비밀번호는 RTS1·RTS2 공통 고정값 'ngii'. ID 하나로 동시 접속 1개만 된다.
#   - RTS1 VRS-RTCM34(Trimble)는 목록에는 GPS+GLO+GAL+BDS+QZS라고 나오지만, 실제로 받은 관측 메시지는
#     1075·1095·1115(GPS·Galileo·QZSS MSM5)뿐이었다. 그래서 반송파 보정을 쓰는 위성이 7개 정도(GPS 4, GAL 3)라
#     Fix가 쉽게 풀린다. 그 밖에 1005·1007·1030·1032·1033·1230·1304·4094가 온다.
#   - 대안: rts2.ngii.go.kr:2101 VRS-RTCM32(Geo++, GPS+GLONASS),
#           rts3.ngii.go.kr:2101 G-VRS 격자(GPS+Galileo+BeiDou MSM7, GGA 불필요, 시범 서비스).
#           측정 위치에서 가장 가까운 격자점은 'Cheongju-Judeok'(약 3km).
#   - 옛 주소 vrs.ngii.go.kr, fkp.ngii.go.kr:2201은 더 이상 접속되지 않는다.
#   - GGA의 위성 수는 최대 12까지만 표시된다. 실제 사용 위성 수는 NAV-PVT/NAV-SAT로 본다(측정 때 21개).
#
# ■ 측정 환경 메모
#   - 안테나(난간 부착)는 북서쪽이 가려져 있었다: W/N 방향 위성 C/N0 14~28 dB-Hz, E/S 방향 34~47 dB-Hz.
#     하늘이 트인 곳과 안테나 아래 접지판(금속판)이 Fix 안정에 도움이 된다.
