#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mapping.py ― 경로 수집 [white1 — 단일 GPS(전방 안테나)]
════════════════════════════════════════════════════════════════════════════════
    ros2 run white1 mapping         (one_launch.py 가 띄운다)

사람이 몰고 다닌 궤적을 CSV 로 남긴다. driving 이 나중에 이 파일의 lat · lon 을 읽어 따라간다.
시작 · 종료는 driving 의 상태기계가 /mapping_cmd 로 지시한다(prompt 1) 매핑 → MAP_START).

★기록하는 점은 뒷차축이다★ — 안테나 자리가 아니다.
  GPS 안테나는 뒷차축에서 antenna_base_m 앞(앞차축 쪽)에 있고, driving 의 경로추종은 뒷차축을 기준점으로
  삼는다. 그래서 전방 안테나 궤적에서 뒷차축 자리를 내어 적는다(pose.FrontTracker — driving 과 같은 계산).
  이렇게 하면 경로 파일의 뜻이 white · white1 · white2 모두 '뒷차축이 지나간 길' 로 같다.
  · 첫 헤딩은 매핑을 시작하고 ★0.5 m 움직인 방향★ 으로 잡는다(곧게 출발했다고 본다). 그때까지의 점은
    그 헤딩으로 한꺼번에 환산해 적는다. 출발이 휘었어도 d 의 몇 배(2~3 m)를 가는 동안 바로잡힌다.
  · ★전진으로만 몰 것★ — 후진하면 헤딩 추정이 틀어진다(pose.py 머리말).

기록 규칙 : ★일정 거리마다 한 점★ (기본 0.25 m, 뒷차축 기준). 측위가 없으면 적지 않는다.
  RTK Fixed 가 아닌 점도 적되 개수를 세어 종료 때 알린다 — 품질은 CSV 에 남지 않으므로 ★종료 알림을 볼 것★.

CSV 형식 : ★매핑 CSV 양식(2026-11)★ lat,lon,alt,dir,mod,olt,oon,oat 8열 — white/mapping.py 와 같다.
  alt = 안테나의 해발고도 hMSL, dir = 0, 나머지는 빈칸. 숫자는 고정소수점만, 줄바꿈 LF.
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

from broon.gpsdev import GPS_PVT_FIELDS, GPS_PVT_TOPIC, Q_FIXED, Q_LABEL, Q_NONE
from white1 import paths
from white1.pose import FrontTracker

SPACING_M = 0.25        # 이 거리마다 한 점 (white · gps_mapper · 사이트 궤적 생성과 같은 기본값)
ANTENNA_BASE_M = 1.0    # 뒷차축 → 전방 안테나 거리 (driving.py 와 같은 값이어야 한다 — 런치가 같이 넘긴다)
EARTH_R = 6378137.0

# 매핑 CSV 양식 8열 — 사이트 csv.ts ROUTE_COLUMNS · tool/gps_mapper.py 와 같다. driving.py 가 이 이름으로 읽는다.
ROUTE_COLUMNS = ["lat", "lon", "alt", "dir", "mod", "olt", "oon", "oat"]


def delta_m(lat0, lon0, lat1, lon1):
    x = EARTH_R * math.radians(lon1 - lon0) * math.cos(math.radians(lat0))
    y = EARTH_R * math.radians(lat1 - lat0)
    return x, y


def offset_latlon(lat0, lon0, x, y):
    return (lat0 + math.degrees(y / EARTH_R),
            lon0 + math.degrees(x / (EARTH_R * math.cos(math.radians(lat0)))))


def route_row(lat, lon, alt):
    """한 점 → 8열. 양식은 지수 표기 · nan 을 받지 않는다 — 고도가 없으면 빈칸."""
    return [f"{lat:.8f}", f"{lon:.8f}", f"{alt:.3f}" if math.isfinite(alt) else "", "0", "", "", "", ""]


class MappingNode(Node):
    def __init__(self):
        super().__init__('mapping_node')
        self.declare_parameter('data_dir', '')
        self.declare_parameter('spacing_m', SPACING_M)
        self.declare_parameter('antenna_base_m', ANTENNA_BASE_M)
        self.data_dir = paths.data_dir(str(self.get_parameter('data_dir').value))
        self.spacing = float(self.get_parameter('spacing_m').value)
        self.ant_base = float(self.get_parameter('antenna_base_m').value)
        os.makedirs(self.data_dir, exist_ok=True)

        self.active = False
        self.fp = self.writer = None
        self.path = ''
        self.rows = 0
        self.rows_not_fixed = 0
        self.length = 0.0
        self.t0 = 0.0
        self.origin = None        # (lat0, lon0) — 이번 매핑의 로컬 평면 원점
        self.tracker = FrontTracker(self.ant_base)
        self.pending = []         # 헤딩을 잡기 전 안테나 점 [(fx, fy, alt, fixed)]
        self.last = None          # 마지막으로 '기록한' 뒷차축 점 (x, y)

        self.pub_point = self.create_publisher(String, '/mapping_point', 10)
        self.pub_done = self.create_publisher(String, '/mapping_done', 10)
        self.create_subscription(Float64MultiArray, GPS_PVT_TOPIC, self.cb_pvt, 20)
        self.create_subscription(Bool, '/mapping_cmd', self.cb_cmd, 10)
        self.get_logger().info(f"🗺️ mapping 대기 — 저장 폴더 {self.data_dir} | 뒷차축 = 안테나 뒤 {self.ant_base:.2f} m")

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
        self.origin = None
        self.tracker.reset()
        self.pending = []
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
            if not self.tracker.ready:
                text += f" (헤딩을 잡으려면 {FrontTracker.INIT_DIST_M} m 는 움직여야 한다)"
            if self.rows == 0:
                try:
                    os.remove(self.path)
                    text += " (빈 파일 삭제)"
                except OSError:
                    pass
            self.get_logger().warning(text)
        else:
            text = (f"💾 매핑 저장 {name} — {self.rows}점, {self.length:.1f} m, "
                    f"{time.time() - self.t0:.0f} s (뒷차축 기준)")
            if self.rows_not_fixed:
                text += (f"  ⚠️ RTK Fixed 아닌 점 {self.rows_not_fixed}개 — 그 구간 좌표를 믿지 말 것 "
                         f"(CSV 에는 품질이 남지 않는다)")
            self.get_logger().info(text)
        self.pub_done.publish(String(data=text))

    def cb_pvt(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) < GPS_PVT_FIELDS or not self.active:
            return
        lat, lon, q, hacc, speed, alt = d[0], d[1], int(d[2]), d[3], d[4], d[9]
        if q == Q_NONE or not (math.isfinite(lat) and math.isfinite(lon)):
            return
        if self.origin is None:
            self.origin = (lat, lon)
        fx, fy = delta_m(self.origin[0], self.origin[1], lat, lon)
        fixed = q == Q_FIXED
        if not self.tracker.ready:
            # 헤딩을 아직 모른다 — 점을 모아 두었다가 헤딩이 잡히면 그 헤딩으로 한꺼번에 뒷차축으로 옮긴다
            self.pending.append((fx, fy, alt, fixed))
            if not self.tracker.feed(fx, fy, auto_init=True):
                return
            h = math.radians(self.tracker.heading)
            ox, oy = self.ant_base * math.cos(h), self.ant_base * math.sin(h)
            for px, py, palt, pfix in self.pending:
                self._point(px - ox, py - oy, palt, pfix, q, hacc, speed)
            self.pending = []
            return
        self.tracker.feed(fx, fy)
        rx, ry = self.tracker.rear()
        self._point(rx, ry, alt, fixed, q, hacc, speed)

    def _point(self, x, y, alt, fixed, q, hacc, speed):
        """뒷차축 한 점 — 마지막 기록에서 spacing 넘게 왔으면 적는다."""
        if self.last is not None:
            step = math.hypot(x - self.last[0], y - self.last[1])
            if step < self.spacing:
                return
            self.length += step
        lat, lon = offset_latlon(self.origin[0], self.origin[1], x, y)
        self.writer.writerow(route_row(lat, lon, alt))
        self.last = (x, y)
        self.rows += 1
        if not fixed:
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
