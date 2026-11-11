#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mapping.py ― 경로 수집 [gold white1/mapping.py 계승]
════════════════════════════════════════════════════════════════════════════════
    ros2 run white mapping          (one_launch.py 가 띄운다)

사람이 리모컨으로 몰고 다닌 궤적을 CSV 로 남긴다. driving 이 나중에 이 파일의
lat · lon 을 읽어 그대로 따라간다. 리모델링(black route_remodeler)은 하지 않는다.

★GPS 원값만 쓴다★ (/gps_pvt — gps.py 가 NAV-PVT 에서 뽑은 좌표 그대로)
  GPS 안테나가 ★뒷차축 바로 위★ 라 원값이 곧 뒷차축 궤적이다 — driving 이 추종하는 기준점과
  같다(black 은 안테나가 뒷차축 앞 0.865 m 라 헤딩으로 투영해서 적었다). 안테나를 옮기면
  매핑 · 추종 기준점이 함께 옮겨지므로 driving 의 순수추종 기하(뒷차축 기준)를 다시 봐야 한다.

시작 · 종료는 driving 의 상태기계가 /mapping_cmd 로 지시한다(prompt 1) 매핑 → MAP_START).
기록 규칙 : ★일정 거리마다 한 점★ (기본 0.25 m). 측위가 없으면(품질 0) 적지 않는다.
  RTK Fixed 가 아닌 점도 적되 개수를 세어 종료 때 알린다 — 주행은 Fixed 에서만 하므로
  Float 로 딴 구간은 경로 자체가 수십 cm~수 m 틀려 있을 수 있다. 품질은 CSV 에 남지 않으므로
  ★종료 알림을 볼 것★(사후 분석은 주행 기록 쪽).

CSV 형식 : ★매핑 CSV 양식(2026-11)★ — 궤적 사이트(domiman.duckdns.org/gps) · gps_mapper.py 와
  ★열 이름 · 순서가 같다★. 정본은 사이트 PC 의 C:\\web\\apps\\gps\\CLAUDE.md '매핑 CSV 양식' 절.
    lat,lon,alt,dir,mod,olt,oon,oat          (8열만 — 덧붙임 열 없음)
    lat · lon : 소수 8자리    alt : 해발고도 hMSL [m] 소수 3자리
    dir : 0(전진 — 이 차는 후진 매핑이 없다)    mod · olt · oon · oat : 빈칸
  구역 단어(mod) · 순환(return) · 점 옮기기(olt…)는 사이트에서 붙인다. 숫자는 고정소수점만, 줄바꿈 LF.
"""

import csv
import math
import os
import signal
import time
from datetime import datetime

import rclpy
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64MultiArray

from white import paths
from white.gps import GPS_PVT_FIELDS, GPS_PVT_TOPIC, Q_FIXED, Q_LABEL, Q_NONE

SPACING_M = 0.25        # 이 거리마다 한 점 (gold · gps_mapper · 사이트 궤적 생성과 같은 기본값)
EARTH_R = 6378137.0

# 매핑 CSV 양식 8열 — 사이트 csv.ts ROUTE_COLUMNS · gps_mapper.py 와 같다. driving.py 가 이 이름으로 읽는다.
ROUTE_COLUMNS = ["lat", "lon", "alt", "dir", "mod", "olt", "oon", "oat"]


def delta_m(lat0, lon0, lat1, lon1):
    x = EARTH_R * math.radians(lon1 - lon0) * math.cos(math.radians(lat0))
    y = EARTH_R * math.radians(lat1 - lat0)
    return x, y


def route_row(lat, lon, alt):
    """한 점 → 8열. 양식은 지수 표기 · nan 을 받지 않는다 — 고도가 없으면 빈칸."""
    return [f"{lat:.8f}", f"{lon:.8f}", f"{alt:.3f}" if math.isfinite(alt) else "", "0", "", "", "", ""]


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
        self.rows = 0
        self.rows_not_fixed = 0
        self.length = 0.0
        self.t0 = 0.0
        self.last = None          # 마지막으로 '기록한' 점

        self.pub_point = self.create_publisher(String, '/mapping_point', 10)
        self.pub_done = self.create_publisher(String, '/mapping_done', 10)
        self.create_subscription(Float64MultiArray, GPS_PVT_TOPIC, self.cb_pvt, 20)
        self.create_subscription(Bool, '/mapping_cmd', self.cb_cmd, 10)
        self.get_logger().info(f"🗺️ mapping 대기 — 저장 폴더 {self.data_dir}")

    def cb_cmd(self, msg: Bool):
        if msg.data and not self.active:
            self.start()
        elif not msg.data and self.active:
            self.stop()

    def start(self):
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path = os.path.join(self.data_dir, f"route_{stamp}.csv")
        self.fp = open(self.path, 'w', newline='', encoding='utf-8')
        self.writer = csv.writer(self.fp, lineterminator='\n')     # 양식: 줄바꿈 LF
        self.writer.writerow(ROUTE_COLUMNS)
        self.rows = self.rows_not_fixed = 0
        self.length = 0.0
        self.t0 = time.time()
        self.last = None
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
            self.get_logger().warning(text)
        else:
            text = (f"💾 매핑 저장 {name} — {self.rows}점, {self.length:.1f} m, "
                    f"{time.time() - self.t0:.0f} s")
            if self.rows_not_fixed:
                text += (f"  ⚠️ RTK Fixed 아닌 점 {self.rows_not_fixed}개 — 그 구간 좌표를 믿지 말 것 "
                         f"(CSV 에는 품질이 남지 않는다)")
            self.get_logger().info(text)
        self.pub_done.publish(String(data=text))

    def cb_pvt(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) < GPS_PVT_FIELDS:
            return
        lat, lon, q, hacc, speed, alt = d[0], d[1], int(d[2]), d[3], d[4], d[9]
        if q == Q_NONE or not (math.isfinite(lat) and math.isfinite(lon)):
            return
        if not self.active:
            return
        if self.last is not None:
            dx, dy = delta_m(self.last[0], self.last[1], lat, lon)
            step = math.hypot(dx, dy)
            if step < self.spacing:
                return
            self.length += step
        self.writer.writerow(route_row(lat, lon, alt))
        self.last = (lat, lon)
        self.rows += 1
        if q != Q_FIXED:
            self.rows_not_fixed += 1
        self.pub_point.publish(String(
            data=f"#{self.rows:04d}  {self.length:6.1f} m  {Q_LABEL.get(q, '?')} "
                 f"hAcc {hacc*100:.1f}cm  v {speed:.2f} m/s"))
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
