#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mapping.py ― 경로 수집 [white1 — 단일 GPS(전방 안테나)]  (2026-11-19 다시 씀)
════════════════════════════════════════════════════════════════════════════════
    ros2 run white1 mapping         (one.launch.py 가 띄운다)

사람이 몰고 다닌(조이스틱) ★GPS 궤적만★ CSV 로 남긴다 — 사용자 지정 2026-11-19.
시작 · 종료는 driving 의 상태기계가 /mapping_cmd 로 지시한다(prompt 1) 매핑 → MAP_START / Enter → MAP_STOP).

★기록하는 점 = 전방 GPS 안테나 자리 그대로★ (/gps_pvt). 헤딩을 잡지 않는다.
  뒷차축으로 바꾸는 일은 driving 이 경로를 고를 때 한다(driving.antenna_path_to_rear). 그래서 경로 파일은
  궤적 사이트에서 주행 기록의 fix_lat · fix_lon(같은 안테나)과 바로 겹쳐 볼 수 있다.
  ※ 2026-11-11 판은 뒷차축으로 바꿔 적었다 — 그 파일을 지금 driving 으로 달리면 안테나 거리만큼 한 번 더 옮겨진다.

기록 규칙 : ★마지막으로 적은 점에서 SPACING_M(0.25 m) 넘게 움직이면 한 점★. 측위가 없으면 적지 않는다.
  RTK Fixed 가 아닌 점도 적되 개수를 세어 끝에 알린다(CSV 에는 품질이 남지 않는다 — 사용자 결정).
dir (전진 0 · 후진 1) : /cmd_vel 의 ★목표 펄스 부호만★ 쓴다 — + 이면 0, − 이면 1, 0 이면 앞의 값을 그대로.
  (/cmd_vel = Int32MultiArray [펄스, 조향] — 조이스틱 · driving 이 낸다. 아무도 안 냈으면 0)

CSV : ★매핑 CSV 양식(2026-11)★ lat,lon,alt,dir,mod,olt,oon,oat 8열 (궤적 사이트 정본 — xeon C:\\web\\apps\\gps\\CLAUDE.md).
  alt = 안테나 해발고도 hMSL, mod · olt · oon · oat 빈칸. 숫자는 고정소수점만(위경도 소수 8자리), 줄바꿈 LF, BOM 없음.

파일 이름 — map_<일련번호>.csv (사용자 지정 2026-11-19)
  매핑 중에는 map_<시작 시각 연도월일_시분초>.csv 에 적는다(예 map_20261119_142012.csv).
  끝낼 때 prompt 가 일련번호를 묻는다 → /mapping_save 로 받는다:
    빈 문자열(그냥 Enter)  그 이름 그대로 둔다
    421                   map_421.csv 로 이름을 바꾼다. 이미 있으면 바꾸지 않고 /mapping_done 에 ❌ 로 알린다(다시 묻게)
  번호에 쓸 수 있는 글자는 영문 · 숫자 · . _ - (궤적 사이트 서버 이름 규칙과 같다), 80자까지.
"""

import csv
import math
import os
import re
import signal
import time
from datetime import datetime

import rclpy
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, Int32MultiArray, String

from broon.boards import CMD_TOPIC
from broon.gpsdev import GPS_PVT_FIELDS, GPS_PVT_TOPIC, Q_FIXED, Q_LABEL, Q_NONE
from white1 import paths

SPACING_M = 0.25        # 이 거리마다 한 점 (gps_mapper · 사이트 궤적 생성과 같은 기본값)
EARTH_R = 6378137.0
NAME_OK = re.compile(r'[0-9A-Za-z._-]{1,80}')

# 매핑 CSV 양식 8열 — 사이트 csv.ts ROUTE_COLUMNS · tool/gps_mapper.py 와 같다. driving.py 가 이 이름으로 읽는다.
ROUTE_COLUMNS = ["lat", "lon", "alt", "dir", "mod", "olt", "oon", "oat"]


def delta_m(lat0, lon0, lat1, lon1):
    x = EARTH_R * math.radians(lon1 - lon0) * math.cos(math.radians(lat0))
    y = EARTH_R * math.radians(lat1 - lat0)
    return x, y


def route_row(lat, lon, alt, direction):
    """한 점 → 8열. 양식은 지수 표기 · nan 을 받지 않는다 — 고도가 없으면 빈칸."""
    return [f"{lat:.8f}", f"{lon:.8f}", f"{alt:.3f}" if math.isfinite(alt) else "", str(direction), "", "", "", ""]


def map_name(serial):
    """일련번호 → map_<번호>.csv. 쓸 수 없는 글자가 있으면 None."""
    serial = serial.strip()
    if serial.lower().startswith('map_'):
        serial = serial[4:]
    if serial.lower().endswith('.csv'):
        serial = serial[:-4]
    return f"map_{serial}.csv" if NAME_OK.fullmatch(serial) else None


class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        self.declare_parameter('data_dir', '')
        self.declare_parameter('spacing_m', SPACING_M)
        self.data_dir = paths.data_dir(str(self.get_parameter('data_dir').value))
        self.spacing = float(self.get_parameter('spacing_m').value)
        os.makedirs(self.data_dir, exist_ok=True)

        self.active = False
        self.fp = self.writer = None
        self.path = ''
        self.saved_path = ''         # 방금 끝낸 매핑 파일 (일련번호를 기다린다)
        self.rows = 0
        self.rows_not_fixed = 0
        self.rows_rev = 0
        self.length = 0.0
        self.t0 = 0.0
        self.last = None             # 마지막으로 적은 점 (lat, lon)
        self.direction = 0           # 0 전진 · 1 후진 — /cmd_vel 펄스 부호

        self.pub_point = self.create_publisher(String, '/mapping_point', 10)
        self.pub_done = self.create_publisher(String, '/mapping_done', 10)
        self.create_subscription(Float64MultiArray, GPS_PVT_TOPIC, self.cb_pvt, 20)
        self.create_subscription(Bool, '/mapping_cmd', self.cb_cmd, 10)
        self.create_subscription(String, '/mapping_save', self.cb_save, 10)
        self.create_subscription(Int32MultiArray, CMD_TOPIC, self.cb_vel, 50)
        self.get_logger().info(f"🗺️ mapping 대기 — 저장 폴더 {self.data_dir} | 전방 안테나 GPS 궤적 그대로, "
                               f"{self.spacing:g} m 마다, dir = {CMD_TOPIC} 펄스 부호")

    def cb_vel(self, msg: Int32MultiArray):
        if len(msg.data) >= 1 and msg.data[0] != 0:
            self.direction = 0 if msg.data[0] > 0 else 1

    def cb_cmd(self, msg: Bool):
        if msg.data and not self.active:
            self.start()
        elif not msg.data and self.active:
            self.stop()

    def start(self):
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(self.data_dir, f"map_{stamp}.csv")
        self.fp = open(self.path, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.fp, lineterminator='\n')     # 양식: 줄바꿈 LF
        self.writer.writerow(ROUTE_COLUMNS)
        self.rows = self.rows_not_fixed = self.rows_rev = 0
        self.length = 0.0
        self.t0 = time.time()
        self.last = None
        self.direction = 0
        self.saved_path = ''
        self.active = True
        self.get_logger().info(f"🗺️ 매핑 시작 → {self.path}")

    def stop(self):
        self.active = False
        try:
            self.fp.close()
        except Exception:      # noqa: BLE001 — 닫다 실패해도 이미 쓴 줄은 남는다
            pass
        self.fp = self.writer = None
        name = os.path.basename(self.path)
        if self.rows < 2:
            text = f"⚠️ 수집된 점이 {self.rows}개뿐이다 — 경로로 쓸 수 없다: {name}"
            if self.rows == 0:
                try:
                    os.remove(self.path)
                    text += " (빈 파일 삭제)"
                except OSError:
                    pass
            else:
                self.saved_path = self.path
            self.get_logger().warning(text)
        else:
            self.saved_path = self.path
            text = (f"💾 매핑 저장 {name} — {self.rows}점, {self.length:.1f} m, {time.time() - self.t0:.0f} s "
                    f"(전방 안테나 궤적)")
            if self.rows_rev:
                text += f" · 후진(dir 1) {self.rows_rev}점"
            if self.rows_not_fixed:
                text += (f"  ⚠️ RTK Fixed 아닌 점 {self.rows_not_fixed}개 — 그 구간 좌표를 믿지 말 것 "
                         f"(CSV 에는 품질이 남지 않는다)")
            self.get_logger().info(text)
        self.pub_done.publish(String(data=text))

    def cb_save(self, msg: String):
        """prompt 가 보낸 일련번호로 방금 끝낸 매핑 파일 이름을 바꾼다. 빈 문자열이면 그대로 둔다."""
        serial = str(msg.data).strip()
        if not self.saved_path or not os.path.isfile(self.saved_path):
            self.pub_done.publish(String(data="❌ 이름을 바꿀 매핑 파일이 없다"))
            return
        old = os.path.basename(self.saved_path)
        if not serial:
            self.pub_done.publish(String(data=f"✅ 저장 이름 {old}"))
            self.saved_path = ''
            return
        new = map_name(serial)
        if new is None:
            self.pub_done.publish(String(data=f"❌ '{serial}' — 영문 · 숫자 · . _ - 만 쓸 수 있다"))
            return
        target = os.path.join(self.data_dir, new)
        if os.path.exists(target):
            self.pub_done.publish(String(data=f"❌ {new} 가 이미 있다 — 다른 번호를 넣을 것"))
            return
        try:
            os.rename(self.saved_path, target)
        except OSError as e:
            self.pub_done.publish(String(data=f"❌ 이름 바꾸기 실패: {e}"))
            return
        self.saved_path = ''
        self.get_logger().info(f"💾 {old} → {new}")
        self.pub_done.publish(String(data=f"✅ 저장 이름 {new}"))

    def cb_pvt(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) < GPS_PVT_FIELDS or not self.active:
            return
        lat, lon, q, hacc, speed, alt = d[0], d[1], int(d[2]), d[3], d[4], d[9]
        if q == Q_NONE or not (math.isfinite(lat) and math.isfinite(lon)):
            return
        if self.last is not None:
            step = math.hypot(*delta_m(self.last[0], self.last[1], lat, lon))
            if step < self.spacing:
                return
            self.length += step
        self.writer.writerow(route_row(lat, lon, alt, self.direction))
        self.last = (lat, lon)
        self.rows += 1
        if q != Q_FIXED:
            self.rows_not_fixed += 1
        if self.direction:
            self.rows_rev += 1
        self.pub_point.publish(String(
            data=f"#{self.rows:04d}  {self.length:6.1f} m  {'후진' if self.direction else '전진'}  "
                 f"{Q_LABEL.get(q, '?')} hAcc {hacc*100:.1f}cm  v {speed:.2f} m/s"))
        if self.rows % 20 == 0:
            try:
                self.fp.flush()
            except Exception:  # noqa: BLE001
                pass

    def destroy_node(self):
        if self.active:
            self.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # Ctrl-C 두 번째(launch 가 다시 보낸다)가 정리를 끊지 않게
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
