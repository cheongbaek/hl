#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
prompt.py ― CLI 메인 화면 [black prompt.py + gold white1 prompt.py]
════════════════════════════════════════════════════════════════════════════════
    ros2 run white1 prompt          (★별 터미널★ — one_launch.py 를 먼저 띄운다)

  1) 매핑   — 손으로(리모컨 등) 몰면 0.25 m 마다 기록된다. Enter 를 누르면 저장하고 메뉴로.
  2) 주행   — 경로 CSV 를 고르면 바로 출발한다: 조향 0 으로 곧게 굴러 헤딩을 잡은 뒤 경로추종.
              Enter = 정지. 도착 · 경로이탈 · 정지 뒤 바퀴가 서면 메뉴로 돌아온다.
              ★출발은 RTK Fixed 일 때만 된다★ — 아니면 거절되고 그 이유가 나온다.
              순환 경로(마지막 줄 mod 에 return — 목록에 🔁)는 도착 없이 계속 돈다 — Enter 로 멈춘다.
  3) 경로 목록 — 매핑 CSV 양식(lat,lon,…)만 쓸 수 있다. 옛 양식 파일은 '옛 양식' 으로 표시된다.
  4) 종료

시작은 이 화면이 /drive_cmd 로 보내는 명령뿐이다(MAP_START · MAP_STOP · 경로 이름 + DRIVE_START · STOP).
출발 가능 여부(GPS · IMU · 아두이노)는 driving 이 판정하고, 거절하면 그 이유가 이 화면에 나온다.
경로 폴더는 driving 이 /route_dir 로 알려 준 곳을 쓴다(못 받으면 paths.py 기본값).
"""

import csv
import os
import select
import signal
import sys
import threading
import time
from collections import deque

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from white1 import paths
from white1.driving import read_route_csv

BANNER = "═" * 72
START_WAIT_S = 3.0        # DRIVE_START · MAP_START 를 보낸 뒤 상태가 바뀌길 기다리는 시간
STOP_WAIT_S = 12.0        # STOP 뒤 IDLE 로 돌아오길 기다리는 상한
STATUS_EVERY_S = 2.0      # 주행 중 상태 줄 출력 주기


class PromptNode(Node):
    def __init__(self):
        super().__init__('prompt_node')
        self.lock = threading.Lock()
        self.events = deque(maxlen=200)
        self.state = '?'
        self.state_t = 0.0
        self.drive_status = ''
        self.gps_status = ''
        self.arduino_status = ''
        self.map_points = deque(maxlen=200)
        self.map_done = ''
        self.route_dir = ''
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_cmd = self.create_publisher(String, '/drive_cmd', 10)
        self.create_subscription(String, '/drive_state', self.cb_state, 10)
        self.create_subscription(String, '/drive_event', self.cb_event, 50)
        self.create_subscription(String, '/drive_status', self.cb_dstatus, 10)
        self.create_subscription(String, '/gps_status', self.cb_gstatus, 10)
        self.create_subscription(String, '/arduino_status', self.cb_astatus, 10)
        self.create_subscription(String, '/mapping_point', self.cb_mpoint, 50)
        self.create_subscription(String, '/mapping_done', self.cb_mdone, 10)
        self.create_subscription(String, '/route_dir', self.cb_dir, latched)

    def cb_state(self, m):
        with self.lock:
            self.state, self.state_t = m.data, time.time()

    def cb_event(self, m):
        with self.lock:
            self.events.append(m.data)

    def cb_dstatus(self, m):
        self.drive_status = m.data

    def cb_gstatus(self, m):
        self.gps_status = m.data

    def cb_astatus(self, m):
        self.arduino_status = m.data

    def cb_mpoint(self, m):
        with self.lock:
            self.map_points.append(m.data)

    def cb_mdone(self, m):
        self.map_done = m.data

    def cb_dir(self, m):
        self.route_dir = m.data

    def send(self, text):
        self.pub_cmd.publish(String(data=text))

    def get_state(self):
        with self.lock:
            return self.state, self.state_t

    def drain_events(self):
        with self.lock:
            out = list(self.events)
            self.events.clear()
        return out

    def drain_points(self):
        with self.lock:
            out = list(self.map_points)
            self.map_points.clear()
        return out


# ══════════════════════════════════════════════════════════════════════════════
#  화면
# ══════════════════════════════════════════════════════════════════════════════
class Screen:
    def __init__(self, node):
        self.node = node
        self.eof = False

    def data_dir(self):
        return self.node.route_dir or paths.data_dir()

    def key_pressed(self, timeout):
        """Enter 가 눌렸으면 True (줄 단위 입력). 입력이 닫혔으면(EOF) 더 보지 않는다."""
        if self.eof:
            time.sleep(timeout)
            return False
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        if not r:
            return False
        line = sys.stdin.readline()
        if line == '':
            self.eof = True
            return False
        return True

    def ask(self, text):
        try:
            return input(text).strip()
        except EOFError:
            self.eof = True
            return None

    def print_events(self):
        for e in self.node.drain_events():
            print(f"   {e}")

    def show_menu(self):
        st, _ = self.node.get_state()
        print(f"\n{BANNER}")
        print("  white1 — 1/5카 GPS 경로추종 (단일 GPS · 전방 안테나)")
        print(f"  {self.node.gps_status or 'GPS 상태 없음 (gps 노드?)'}")
        print(f"  {self.node.arduino_status or '아두이노 상태 없음 (arduino 노드?)'}")
        print(f"  driving : {st if st != '?' else '응답 없음 (driving 노드?)'}   경로 폴더 {self.data_dir()}")
        print(BANNER)
        print("  1) 매핑     2) 주행     3) 경로 목록     4) 종료")
        print(BANNER)

    def list_routes(self, numbered=True):
        d = self.data_dir()
        files = paths.list_routes(d)
        if not files:
            print(f"   (경로 CSV 없음: {d})")
            return []
        print(f"\n   📂 {d}")
        for i, f in enumerate(files, 1):
            try:
                pts, loop, _, _ = read_route_csv(os.path.join(d, f))
                info = f"{len(pts)}점" + (", 🔁 순환" if loop else "")
            except ValueError:
                info = "옛 양식 — 못 씀"
            except (OSError, UnicodeError, csv.Error):
                info = "?"
            print(f"   {i:3d}. {f}  ({info})" if numbered else f"   - {f} ({info})")
        return files

    # ── 1) 매핑 ──────────────────────────────────────────────────────────────
    def run_mapping(self):
        self.node.drain_events()
        self.node.drain_points()
        self.node.map_done = ''
        self.node.send('MAP_START')
        if not self.wait_state(lambda s: s == 'MAP', START_WAIT_S):
            print("   ⚠️ 매핑이 시작되지 않았다")
            self.print_events()
            return
        self.print_events()
        print("   🗺️ 매핑 중 — 손으로(리모컨 등) ★전진으로★ 몰 것.  [Enter] 저장하고 메뉴로")
        while True:
            for p in self.node.drain_points():
                print(f"   {p}")
            self.print_events()
            st, _ = self.node.get_state()
            if st != 'MAP':
                print(f"   ⚠️ 매핑이 끝났다 (driving 상태 {st})")
                break
            if self.key_pressed(0.2):
                self.node.send('MAP_STOP')
                break
        t0 = time.time()
        while not self.node.map_done and time.time() - t0 < START_WAIT_S:
            time.sleep(0.1)
        for p in self.node.drain_points():
            print(f"   {p}")
        self.print_events()
        print(f"   {self.node.map_done or '⚠️ 저장 결과를 받지 못했다 (mapping 노드?)'}")

    # ── 2) 주행 ──────────────────────────────────────────────────────────────
    def run_drive(self):
        files = self.list_routes()
        if not files:
            return
        ans = self.ask("\n   주행할 번호 (Enter = 취소): ")
        if not ans:
            return
        try:
            idx = int(ans) - 1
        except ValueError:
            print("   ⚠️ 숫자를 입력하세요")
            return
        if not 0 <= idx < len(files):
            print("   ⚠️ 잘못된 번호")
            return
        st, _ = self.node.get_state()
        if st not in ('IDLE', '?'):
            print(f"   ⚠️ driving 이 {st} 상태다 — IDLE 이 되면 다시")
            return
        self.node.drain_events()
        self.node.send(files[idx])
        time.sleep(0.2)
        self.node.send('DRIVE_START')
        if not self.wait_state(lambda s: s.startswith('DRIVE'), START_WAIT_S):
            self.print_events()
            print("   ⚠️ 주행이 시작되지 않았다 — 위 이유 확인")
            return
        print(f"   🚀 자율주행 [{files[idx]}]  — [Enter] 정지")
        stopping = False
        last_status = 0.0
        while True:
            self.print_events()
            st, _ = self.node.get_state()
            if st == 'IDLE':
                break
            now = time.time()
            if now - last_status >= STATUS_EVERY_S and self.node.drive_status and st == 'DRIVE_RUN':
                last_status = now
                print(f"   · {self.node.drive_status}")
            if not stopping and self.key_pressed(0.1):
                self.node.send('STOP')
                stopping = True
                stop_t = time.time()
                print("   🛑 정지 명령 보냄 — 바퀴가 서면 메뉴로")
            elif stopping:
                time.sleep(0.1)
                if time.time() - stop_t > STOP_WAIT_S:
                    print("   ⚠️ 정지 확인을 못 받았다 — 메뉴로 (driving 상태 확인)")
                    break
        time.sleep(0.2)
        self.print_events()
        print("   ↩︎ 메인 화면")

    def wait_state(self, pred, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            st, _ = self.node.get_state()
            if pred(st):
                return True
            time.sleep(0.05)
        return False

    # ── 메인 루프 ────────────────────────────────────────────────────────────
    def run(self):
        time.sleep(0.8)          # 상태 토픽을 한 번은 받고 시작한다
        while rclpy.ok():
            self.print_events()
            self.show_menu()
            choice = self.ask("  메뉴 선택 (1/2/3/4): ")
            if choice is None:
                return
            if choice == '1':
                self.run_mapping()
            elif choice == '2':
                self.run_drive()
            elif choice == '3':
                self.list_routes(numbered=False)
            elif choice == '4':
                print("  👋 종료")
                return
            elif choice:
                print("  ⚠️ 1~4 중에서 고르세요")


def _spin(executor):
    try:
        executor.spin()
    except (rclpy.executors.ExternalShutdownException, rclpy.executors.ShutdownException):
        pass


def main(args=None):
    sys.stdout.reconfigure(line_buffering=True)     # tee 로 남겨도 줄마다 바로 나오게
    rclpy.init(args=args)
    node = PromptNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=_spin, args=(executor,), daemon=True)
    spin.start()
    screen = Screen(node)
    try:
        screen.run()
    except KeyboardInterrupt:
        st, _ = node.get_state()
        if st.startswith('DRIVE') or st == 'MAP':
            node.send('STOP')
            time.sleep(0.2)
        print("\n  👋 종료 (Ctrl-C)")
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # Ctrl-C 를 또 눌러도 정리를 끊지 않게
        # ★실행기 스레드를 먼저 끝낸다★ spin 중에 노드를 지우면 프로세스가 abort 한다
        rclpy.try_shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
