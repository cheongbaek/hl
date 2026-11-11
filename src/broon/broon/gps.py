#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gps.py ― GPS 노드 (ZED-F9P 1대 또는 2대 + NTRIP RTK) — white1 · white2 런치가 띄운다
════════════════════════════════════════════════════════════════════════════════
    ros2 run broon gps                              단일 : 전방 GPS 한 대 (white1)
    ros2 run broon gps --ros-args -p dual:=true     듀얼 : 전방 + 후방 (white2)

어느 수신기가 전방 · 후방인지는 ★배치 설정 파일★(config/gps_setting.json — ros2 run broon gps_setting)의
칩 고유 ID 로 정한다. u-blox 포트를 하나씩 열어 UBX-SEC-UNIQID 를 읽고, 설정의 ID 와 같은 것만 쓴다.
ttyACM 번호 · 꽂은 순서와 무관하다. 설정에 없는 수신기는 쓰지 않고 그 ID 를 로그로 알린다.
IMU 는 쓰지 않는다 (white1 · white2 는 헤딩을 GPS 만으로 잡는다).

수신부는 white/gps.py(= tool/gps_mapper.py)와 같다 :
  · 접속할 때마다 수신기 ★RAM 에만★ 설정을 쓴다 — NAV-PVT 10 Hz, USB 로는 GGA · RMC 만.
  · NGII VRS 보정(NTRIP)은 접속 ★하나★ 로 받아 연결된 수신기 모두에 똑같이 넣는다(tool/gps_mapper_dual.py 와 같다).
    캐스터에 올리는 GGA 는 전방이 측위 중이면 전방 것, 아니면 후방 것.
  · 위치 · 품질 · 정확도 · 속도를 NAV-PVT 한 메시지에서 뽑는다(carrSoln 으로 RTK Fixed / Float 이 갈린다).
  ⚠️ Fixed 라도 틀릴 수 있다(2026-10-13 시험: 10분간 1.3 m 틀린 Fixed). 듀얼이면 white2 가 두 안테나 간격으로 검사한다.

발행
  /gps_pvt       (Float64MultiArray)  전방 — gpsdev 의 GPS_PVT 규약 10칸
  /gps_pvt_rear  (Float64MultiArray)  후방 — 같은 규약 (dual 일 때만)
  /gps_dual      (Float64MultiArray)  ★같은 측위 시각(iTOW)끼리 짝지은 한 줄★ GPS_DUAL 규약 14칸 (dual 일 때만)
                                      두 수신기 모두 10 Hz 로 GPS 시각 0.1 초 경계에서 측위하므로 iTOW 가 같다.
                                      한쪽이 PAIR_TIMEOUT_S 안에 안 오면 그쪽은 quality 0 으로 내보낸다.
  /fix           (NavSatFix)          전방 (확인 · 기록용. status: −1 없음 · 0 단독 · 1 DGNSS · 2 RTK Float 이상)
  /gps_status    (String)             1 Hz 한 줄 상태 (prompt 메인 화면)
"""

import math
import signal
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64MultiArray, String

import serial

from broon import gpsdev
from broon.gpsdev import (COURSE_MIN_SPEED, F9PStream, GPS_DUAL_FIELDS, GPS_DUAL_TOPIC, GPS_PVT_REAR_TOPIC,
                          GPS_PVT_TOPIC, PVT_STALE_SEC, Q_DGPS, Q_LABEL, Q_NONE, Q_SPS, ROLE_KO,
                          apply_receiver_config, pvt_quality)
from broon.ntrip import NTRIP, NtripClient

SCAN_SEC = 1.0               # 새로 꽂힌 포트를 찾는 주기
UNKNOWN_RETRY_S = 10.0       # 쓰지 않는(다른 ID · ID 못 읽은) 포트는 이만큼 지나야 다시 열어 본다
PAIR_TIMEOUT_S = 0.15        # 듀얼 — 한쪽 NAV-PVT 를 이만큼 기다려도 안 오면 한쪽만으로 내보낸다
GGA_FRESH_S = 3.0            # 이보다 오래된 GGA 는 캐스터에 올리지 않는다
EARTH_R = 6378137.0
NAN = float('nan')


class Receiver:
    """수신기 한 대(전방 또는 후방)의 상태."""

    def __init__(self, role, want_id):
        self.role, self.want_id = role, want_id
        self.ser = None
        self.lock = threading.Lock()       # 시리얼 쓰기(RTCM · 설정) 보호
        self.port = ""
        self.cfg_ok = None
        self.pvt = None
        self.pvt_t = 0.0
        self.quality = Q_NONE
        self.gga = None                    # (raw, 받은 시각)

    @property
    def connected(self):
        return self.ser is not None

    def fresh(self, now):
        return self.pvt is not None and now - self.pvt_t <= PVT_STALE_SEC


class GpsNode(Node):
    def __init__(self):
        super().__init__('gps_node')
        self.declare_parameter('dual', False)         # true = 전방 + 후방, false = 전방만
        self.declare_parameter('ntrip', True)         # false = 보정 안 받음 (단독/DGNSS)
        self.declare_parameter('setting', '')         # 배치 설정 파일. 비우면 gpsdev.config_path()
        self.dual = bool(self.get_parameter('dual').value)
        path = str(self.get_parameter('setting').value) or gpsdev.config_path()
        setting = gpsdev.load_setting(path)
        roles = gpsdev.ROLES if self.dual else ('front',)
        self.rx = {r: Receiver(r, setting[r]['id']) for r in roles}

        self.stop_evt = threading.Event()
        self.known = {}                    # 포트 → (칩 ID, 열어 본 시각) — 쓰지 않는 포트를 매초 열지 않게
        self.pair_lock = threading.Lock()
        self.pairs = {}                    # iTOW → {'front': pvt, 'rear': pvt, 't': 처음 온 시각}

        self.pub = {'front': self.create_publisher(Float64MultiArray, GPS_PVT_TOPIC, 20)}
        self.pub_fix = self.create_publisher(NavSatFix, '/fix', 10)
        self.pub_status = self.create_publisher(String, '/gps_status', 10)
        if self.dual:
            self.pub['rear'] = self.create_publisher(Float64MultiArray, GPS_PVT_REAR_TOPIC, 20)
            self.pub_dual = self.create_publisher(Float64MultiArray, GPS_DUAL_TOPIC, 20)
            self.create_timer(0.02, self._flush_pairs)
        self.create_timer(1.0, self.publish_status)

        self.ntrip = NtripClient(NTRIP, self._write_rtcm, self.get_logger().info)
        self.ntrip.enabled = bool(self.get_parameter('ntrip').value)
        self.ntrip.start()
        threading.Thread(target=self.scan_worker, daemon=True).start()
        want = " · ".join(f"{ROLE_KO[r]} {x.want_id or '★ID 미지정★'}" for r, x in self.rx.items())
        self.get_logger().info(
            f"GPS {'듀얼' if self.dual else '단일'} ({want}) — 설정 {path} | "
            f"{'NTRIP ' + NTRIP['mountpoint'] if self.ntrip.enabled else '보정 없음'}")
        for r, x in self.rx.items():
            if not x.want_id:
                self.get_logger().error(f"{ROLE_KO[r]} GPS 의 ID 가 설정에 없다 — ros2 run broon gps_setting 으로 지정할 것")

    # ── 보정 · 포트 찾기 ─────────────────────────────────────────────────────
    def _write_rtcm(self, data):
        for x in self.rx.values():
            with x.lock:
                if x.ser:
                    try:
                        x.ser.write(data)
                    except (serial.SerialException, OSError):
                        pass

    def scan_worker(self):
        """u-blox 포트를 열어 칩 ID 를 읽고, 설정의 전방 · 후방과 같으면 그 수신기의 읽기 스레드를 띄운다."""
        while not self.stop_evt.is_set():
            missing = [x for x in self.rx.values() if not x.connected and x.want_id]
            if missing:
                now = time.time()
                ports = gpsdev.list_ublox_ports()
                alive = {p.device for p in ports}
                self.known = {d: v for d, v in self.known.items() if d in alive}
                used = {x.port for x in self.rx.values() if x.connected}
                for p in ports:
                    if p.device in used:
                        continue
                    seen = self.known.get(p.device)
                    if seen and now - seen[1] < UNKNOWN_RETRY_S:
                        continue
                    self._try_port(p.device)
            self.stop_evt.wait(SCAN_SEC)

    def _try_port(self, dev):
        ser, why = gpsdev.open_port(dev)
        if ser is None:
            self.known[dev] = ("", time.time())
            self.get_logger().warning(f"GPS 포트 {dev} : {why}", throttle_duration_sec=15.0)
            return
        try:
            chip = gpsdev.read_chip_id(ser)
        except (serial.SerialException, OSError):
            chip = ""
        x = next((x for x in self.rx.values() if chip and x.want_id == chip and not x.connected), None)
        if x is None:
            self.known[dev] = (chip, time.time())
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass
            if not chip:
                self.get_logger().warning(f"GPS 포트 {dev} : 칩 ID 를 못 읽었다", throttle_duration_sec=15.0)
            elif not any(v.want_id == chip for v in self.rx.values()):
                self.get_logger().warning(
                    f"GPS 포트 {dev} : 칩 ID {chip} 는 이 구성에서 쓰지 않는다 "
                    f"({'설정에 없는 수신기' if self.dual else '단일 구성은 전방 GPS 만 쓴다'} — ros2 run broon gps_setting)",
                    throttle_duration_sec=30.0)
            return
        self.known.pop(dev, None)
        with x.lock:
            x.ser, x.port = ser, dev
        threading.Thread(target=self.read_worker, args=(x, ser, dev), daemon=True).start()

    def read_worker(self, x, ser, dev):
        try:
            x.cfg_ok = apply_receiver_config(ser, x.lock)
            self.get_logger().info(
                f"{ROLE_KO[x.role]} GPS 연결 {dev} (칩 ID {x.want_id}) — 수신기 설정 {'ACK' if x.cfg_ok else '★응답 없음★'}")
            ser.reset_input_buffer()
            stream = F9PStream()
            while not self.stop_evt.is_set():
                ggas, pvts = stream.feed(ser.read(4096))
                for g in ggas:
                    if g["q"] > 0:
                        x.gga = (g["raw"], time.time())
                        self._pick_gga()
                for p in pvts:
                    self._on_pvt(x, p)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"{ROLE_KO[x.role]} GPS 단절({dev}): {e} — 다시 찾는다")
        finally:
            with x.lock:
                x.ser, x.port = None, ""
            x.pvt = None
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass

    def _pick_gga(self):
        """캐스터에 올릴 GGA — 전방이 신선하면 전방, 아니면 후방."""
        now = time.time()
        for role in ('front', 'rear'):
            x = self.rx.get(role)
            if x and x.gga and now - x.gga[1] < GGA_FRESH_S:
                self.ntrip.gga_raw = x.gga[0]
                return

    # ── 측위 ─────────────────────────────────────────────────────────────────
    def _on_pvt(self, x, p):
        now = time.time()
        q = pvt_quality(p)
        if q != x.quality:
            self.get_logger().info(
                f"{ROLE_KO[x.role]} GPS 품질 {Q_LABEL[x.quality]} → {Q_LABEL[q]} (hAcc {p['hacc']:.3f} m)")
        x.quality, x.pvt, x.pvt_t = q, p, now
        if q != Q_NONE:                    # 좌표가 없으면 내보내지 않는다(driving 은 두절로 본다)
            course = (90.0 - p["headmot"] + 180.0) % 360.0 - 180.0 if p["speed"] >= COURSE_MIN_SPEED else NAN
            age = now - self.ntrip.last_data_t if self.ntrip.last_data_t > 0 else NAN
            self.pub[x.role].publish(Float64MultiArray(data=[
                p["lat"], p["lon"], float(q), p["hacc"], p["speed"], course,
                float(p["sv"]), p["itow"] / 1000.0, age, p["hmsl"]]))
            if x.role == 'front':
                self._publish_fix(p, q)
        if self.dual:
            self._pair(x.role, p, now)

    def _publish_fix(self, p, q):
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

    # ── 듀얼 짝짓기 ──────────────────────────────────────────────────────────
    def _pair(self, role, p, now):
        with self.pair_lock:
            e = self.pairs.setdefault(p["itow"], {'t': now})
            e[role] = p
            self._emit_ready(now)

    def _flush_pairs(self):
        with self.pair_lock:
            self._emit_ready(time.time())

    def _emit_ready(self, now):
        """짝이 찼거나 기다릴 만큼 기다린 것을 ★iTOW 순서대로★ 내보낸다 (pair_lock 을 쥔 채)."""
        other_alive = all(x.connected for x in self.rx.values())
        for itow in sorted(self.pairs):
            e = self.pairs[itow]
            full = 'front' in e and 'rear' in e
            if not (full or not other_alive or now - e['t'] >= PAIR_TIMEOUT_S):
                break                      # 앞의 것이 아직 기다리는 중이면 뒤의 것도 내보내지 않는다(순서 유지)
            del self.pairs[itow]
            self._publish_dual(itow, e.get('front'), e.get('rear'))

    def _publish_dual(self, itow, f, r):
        def side(p):
            if p is None or pvt_quality(p) == Q_NONE:
                return [NAN, NAN, 0.0, NAN, NAN, NAN], None
            return [p["lat"], p["lon"], float(pvt_quality(p)), p["hacc"], p["speed"], p["hmsl"]], p
        fd, fp = side(f)
        rd, rp = side(r)
        base = NAN
        if fp and rp:
            dx = EARTH_R * math.radians(fp["lon"] - rp["lon"]) * math.cos(math.radians(rp["lat"]))
            dy = EARTH_R * math.radians(fp["lat"] - rp["lat"])
            base = math.hypot(dx, dy)
        data = [itow / 1000.0] + fd + rd + [base]
        assert len(data) == GPS_DUAL_FIELDS
        self.pub_dual.publish(Float64MultiArray(data=data))

    # ── 상태 ─────────────────────────────────────────────────────────────────
    def publish_status(self):
        now = time.time()
        parts = []
        for role, x in self.rx.items():
            name = ROLE_KO[role] if self.dual else "GPS"
            if not x.want_id:
                parts.append(f"{name} ID 미지정(gps_setting)")
            elif not x.connected:
                parts.append(f"{name} 찾는 중({x.want_id})")
            elif not x.fresh(now):
                parts.append(f"{name} {x.port} 데이터 없음" + ("" if x.cfg_ok else " (수신기 설정 무응답)"))
            else:
                p = x.pvt
                parts.append(f"{name} {Q_LABEL[x.quality]} hAcc {p['hacc']*100:.1f}cm 위성 {p['sv']}")
        nt = self.ntrip.status
        if nt == "streaming":
            parts.append(f"NTRIP 수신 {self.ntrip.bytes_in // 1024}KB (최근 {now - self.ntrip.last_data_t:.0f}s)")
        else:
            parts.append(f"NTRIP {nt} {self.ntrip.detail}".rstrip())
        self.pub_status.publish(String(data=" | ".join(parts)))

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
