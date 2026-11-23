#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arduino.py ― 아두이노 판별 · 구동 '우노' 브리지 [broon]  (2026-11-19 다시 씀)
════════════════════════════════════════════════════════════════════════════════
    ros2 run broon arduino          (broon.launch.py · white1 one.launch.py 가 띄운다)

아두이노 포트를 판별하고, 구동계(UNO R4 Minima — tool/arduino_code/broon_1119.ino)와 문자열로 주고받는다.
규약 · 판별의 정본은 boards.py 머리말이다.

판별 — ★보드 종류와 상관없이★ "J," 줄을 보내는 아두이노 = 조이스틱, ★나머지 아두이노 = 구동계★ (tool/joy.py 와 같다)
  · 조이스틱 포트는 열어서 확인만 하고 ★닫는다★ — /joy_port (래치)로 알리면 joy 노드가 연다.
    (메가는 포트를 열 때마다 리셋된다 — 여기서 한 번, joy 가 열 때 한 번. joy 는 "J,RESET" 을 받고 영점을 다시 잡는다)
  · 구동 포트는 계속 쥐고 있다.
  · 처음에 못 찾은 보드는 RESCAN_S 마다 ★새로 나타난 포트만★ 열어 본다 — 이미 조이스틱으로 판별한 포트는
    다시 열지 않는다(joy 가 쓰는 중에 열면 메가가 리셋된다). 포트가 목록에서 사라졌다 다시 생기면 새 포트로 본다.

재연결 (2026-11-23 사용자 지정 — gold nxde/arduino.py 의 _link_loop · _drop_board 방식)
  · 처음 찾기와 재연결을 worker 스레드 하나가 맡는다. 보드가 없어도 노드는 바로 뜨고, 꽂으면 그때 붙는다.
  · 구동 우노 : 읽기 · 쓰기 예외(USB 빠짐) → 그 포트만 닫고 놓는다(_drop_drive) → RESCAN_S 마다 다시 찾는다.
    끊긴 동안 온 /cmd_vel 은 버린다(상태의 '구동 없어 못 보냄'). 다시 붙어도 ★옛 명령을 다시 보내지 않는다★ —
    joy · driving 이 내는 다음 /cmd_vel 부터 따른다. E-STOP 중에 다시 붙으면 곧바로 's' · '0 0'.
  · 조이스틱 : 포트를 여는 것은 joy 노드다. 여기서는 RESCAN_S 마다 ★포트 목록만★ 보고(열지 않는다) 조이스틱 포트가
    사라졌으면 /joy_port 를 '' 로 내리고, 새 포트가 생기면 판별해서 다시 알린다. 같은 이름으로 금방 돌아오면 joy 가
    스스로 다시 연다. 영점은 joy 노드가 첫 실행 때 잡은 값을 그대로 쓴다(재연결 때 다시 잡지 않는다).
  · 포트는 열려 있는데 줄만 안 오는 것은 끊김으로 보지 않는다 — 상태에 '응답 없음' 만 띄운다 (gold 와 같다).
  · 포트를 직접 정하려면 -p drive_port:=/dev/ttyACM0 (그러면 그 포트를 판별 없이 구동으로 쓴다) ·
    -p joy_port:=/dev/ttyACM1 (판별 없이 /joy_port 로 알린다).

토픽
  구독  /cmd_vel  (std_msgs/Int32MultiArray)  data = [목표 펄스, 목표 조향각] 두 정수
                  펄스 −92 ~ 92 (부호 = 방향), 조향 −21 ~ 21 (− 좌 · + 우). ★길이 · 범위가 틀린 메시지는 버린다★.
                  받는 즉시 우노로 "<펄스> <조향>\\n" 한 줄을 보낸다 — ★스스로 되풀이해 보내지 않는다★.
                  그래서 /cmd_vel 을 내는 노드(joy · driving)가 죽으면 우노의 명령 시한(2 초)이 구동을 세운다.
  발행  /enc_pot  (std_msgs/Float64MultiArray) data = [받은 펄스, 받은 조향각, 실측 펄스, 실측 조향각]
                  우노가 50 ms 마다 내는 네 숫자 그대로 — 앞 둘 = 우노가 '받았다고' 되돌려 준 명령(자르기 뒤),
                  뒤 둘 = 엔코더 실측 펄스(최근 100 ms A+B 에지 수, 부호 = 지금 DIR) · 가변저항 실측 조향각 [도]
        /joy_port (std_msgs/String, 래치)        조이스틱 포트 경로 ('' = 아직 없음)
        /arduino_status (std_msgs/String)       1 Hz 한 줄 상태 (prompt 가 띄운다)
  구독  /estop    (std_msgs/Int32, 래치)       ★1 = E-STOP★ — 곧바로 우노에 's'(구동 · 조향 PWM 즉시 0) 뒤 '0 0'(조향 0° 로)을
                  보내고, 0 이 올 때까지 /cmd_vel 을 ★막고★ '0 0' 을 ESTOP_RESEND_S 마다 보낸다. 0 이면 다시 /cmd_vel 을 넘긴다
                  (해제했다고 스스로 움직이지는 않는다 — 다음 /cmd_vel 부터 따른다). (2026-11-19 사용자 지정, broon/estop.py)

끝낼 때 우노에 "0 0" 을 한 번 보낸다(구동은 하강 제한을 따라 서고 조향은 0° 로).
⚠️ 실제 보드 두 대로 함께 돌려 본 적은 아직 없다 — 가짜 시리얼 포트로만 확인했다 (재연결도 가짜 포트로만, 2026-11-23).
"""

import signal
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64MultiArray, Int32, Int32MultiArray, String

import serial

from broon import boards
from broon.boards import CMD_TOPIC, ENC_POT_TOPIC, ESTOP_TOPIC, JOY_PORT_TOPIC

RESCAN_S = 3.0               # 이 간격으로 포트 목록을 보고, 못 찾은 보드가 있으면 새 포트를 열어 본다 (gold DETECT_RETRY_S)
STATUS_S = 1.0
DRIVE_STALE_S = 0.5          # 우노 출력이 이만큼 없으면 상태에 '응답 없음'
ESTOP_RESEND_S = 0.5         # E-STOP 동안 '0 0' 을 다시 보내는 간격


class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino')
        self.declare_parameter('drive_port', '')     # 비우면 판별
        self.declare_parameter('joy_port', '')       # 비우면 판별
        self.fixed_drive = str(self.get_parameter('drive_port').value)
        self.fixed_joy = str(self.get_parameter('joy_port').value)

        self.stop_evt = threading.Event()
        self.ser = None              # 구동 포트
        self.ser_lock = threading.Lock()
        self.drive_dev = ''
        self.joy_dev = ''
        self.probed = set()          # 이미 열어 본 포트 (목록에서 사라지면 지운다)

        self.rx_n = 0                # 우노에서 받은 올바른 줄
        self.rx_t = 0.0
        self.last_fb = None
        self.tx_n = 0
        self.drop_n = 0              # 범위 · 길이가 틀려 버린 /cmd_vel
        self.nodrive_n = 0           # 구동 포트가 없어 못 보낸 /cmd_vel
        self.last_cmd = None
        self.last_cmd_t = 0.0
        self.estop = False           # /estop 1 — /cmd_vel 을 막는다
        self.estop_n = 0             # E-STOP 으로 막은 /cmd_vel
        self.drive_drops = 0         # 구동 우노가 끊긴 횟수 (재연결 횟수)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_enc = self.create_publisher(Float64MultiArray, ENC_POT_TOPIC, 20)
        self.pub_joy = self.create_publisher(String, JOY_PORT_TOPIC, latched)
        self.pub_status = self.create_publisher(String, '/arduino_status', 10)
        self.create_subscription(Int32MultiArray, CMD_TOPIC, self.cb_cmd, 50)
        self.create_subscription(Int32, ESTOP_TOPIC, self.cb_estop, latched)
        self.create_timer(ESTOP_RESEND_S, self.estop_tick)
        self.create_timer(STATUS_S, self.publish_status)
        self.pub_joy.publish(String(data=''))
        if self.fixed_joy:
            self._set_joy(self.fixed_joy)
        threading.Thread(target=self.worker, daemon=True).start()
        self.get_logger().info(
            f"아두이노 — 구동 {self.fixed_drive or '판별'} · 조이스틱 {self.fixed_joy or '판별'} "
            f"('J,' 줄 = 조이스틱, 나머지 = 구동). {CMD_TOPIC} [펄스 ±{boards.PULSE_MAX}, 조향 ±{boards.STEER_MAX}] → 우노, "
            f"우노 출력 → {ENC_POT_TOPIC}")

    # ── /cmd_vel → 우노 ──────────────────────────────────────────────────────
    def cb_cmd(self, msg: Int32MultiArray):
        cmd = boards.valid_cmd(msg.data)
        if cmd is None:
            self.drop_n += 1
            self.get_logger().warning(f"{CMD_TOPIC} 버림 {list(msg.data)} — [펄스 ±{boards.PULSE_MAX}, "
                                      f"조향 ±{boards.STEER_MAX}] 두 정수만 받는다", throttle_duration_sec=2.0)
            return
        if self.estop:
            self.estop_n += 1
            return                            # E-STOP 중 — 넘기지 않는다
        self.last_cmd, self.last_cmd_t = cmd, time.time()
        self.write_line(f"{cmd[0]} {cmd[1]}\n")

    # ── E-STOP ───────────────────────────────────────────────────────────────
    def cb_estop(self, msg: Int32):
        on = int(msg.data) == 1
        if on == self.estop:
            return
        self.estop = on
        if on:
            self.write_line("s\n")            # 구동 · 조향 PWM 즉시 0 (하강 제한 없이)
            self.write_line("0 0\n")          # 목표 펄스 0 · 조향 0° (조향 PID 를 다시 켠다)
            self.get_logger().warning("🛑 E-STOP — 우노에 's' · '0 0', 해제될 때까지 /cmd_vel 을 막는다")
        else:
            self.get_logger().info("✅ E-STOP 해제 — 다음 /cmd_vel 부터 넘긴다")

    def estop_tick(self):
        if self.estop:
            self.write_line("0 0\n")

    def write_line(self, text):
        err = None
        with self.ser_lock:
            ser = self.ser
            if ser is None:
                self.nodrive_n += 1           # 끊긴 동안 — 조용히 버린다 (gold 와 같다)
                return False
            try:
                ser.write(text.encode('ascii'))
            except serial.SerialTimeoutException:
                return False                  # 이번 것은 건너뛴다 — 다음 명령이 곧 온다
            except (serial.SerialException, OSError) as e:
                err = e
        if err is not None:
            self._drop_drive(ser, f"송신 실패: {err}")   # 쓰기 오류 = 포트가 죽었다 → 놓고 다시 찾는다
            return False
        self.tx_n += 1
        return True

    # ── 판별 · 수신 (스레드) ──────────────────────────────────────────────────
    def _set_joy(self, dev):
        if dev != self.joy_dev:
            self.joy_dev = dev
            self.pub_joy.publish(String(data=dev))
            self.get_logger().info(f"🎮 조이스틱 = {dev or '없음'}")

    def _forget_gone(self):
        """포트 목록만 본다(열지 않는다). 사라진 포트는 다음에 다시 열어 본다 (다시 꽂으면 새 장치다).
        조이스틱 포트가 사라졌으면 /joy_port 를 '' 로 — 다시 나타나면 _scan 이 판별해 다시 알린다."""
        present = {d for d, *_ in boards.candidate_ports()}
        self.probed &= present
        if self.joy_dev and self.joy_dev not in present and not self.fixed_joy:
            self.get_logger().warning(f"조이스틱 포트 {self.joy_dev} 가 사라졌다 — 다시 꽂으면 판별해 붙인다 (영점은 joy 가 유지)")
            self._set_joy('')
        return present

    def worker(self):
        """처음 찾기와 재연결을 함께 맡는다 (gold _link_loop). RESCAN_S 마다 포트 목록을 보고(사라진 조이스틱 포트를 놓는다),
        구동 · 조이스틱 중 못 찾은 것이 있으면 새 포트를 열어 본다. 구동 수신은 따로 도는 스레드(_read_drive)."""
        next_scan = 0.0
        while not self.stop_evt.is_set():
            now = time.monotonic()
            if now >= next_scan:
                next_scan = now + RESCAN_S
                try:
                    present = self._forget_gone()
                    if self.ser is None or not self.joy_dev:
                        self._scan(present)
                except Exception as e:        # 한 번의 오류가 재연결 스레드를 죽이지 않게 (gold 와 같다)
                    self.get_logger().error(f"포트 찾기 오류(다음에 다시): {e}", throttle_duration_sec=10.0)
            self.stop_evt.wait(0.2)

    def _scan(self, present):
        if self.ser is None and self.fixed_drive:
            try:
                ser = serial.Serial(self.fixed_drive, boards.BAUD, timeout=0.1,
                                    write_timeout=boards.WRITE_TIMEOUT_S, exclusive=True)
            except (serial.SerialException, OSError) as e:
                self.get_logger().warning(f"구동 포트 열기 실패 {self.fixed_drive}: {e}", throttle_duration_sec=10.0)
            else:
                self._attach(self.fixed_drive, ser)
        skip = {self.joy_dev, self.drive_dev, self.fixed_joy, self.fixed_drive} - {''}
        fresh = [d for d in sorted(present) if d not in self.probed and d not in skip]
        if not fresh:
            if self.ser is None:
                self.get_logger().warning("구동 아두이노를 찾는 중 — 새 아두이노 포트가 없다", throttle_duration_sec=15.0)
            return
        want_drive = self.ser is None
        self.get_logger().info(f"아두이노 판별 {fresh} — 최대 {boards.DETECT_WINDOW_S:.1f}s")
        res = boards.probe(fresh, keep_drive=want_drive, stop=self.stop_evt)
        self.probed |= set(fresh) - set(res['busy'])     # 열지 못한 포트는 다음에 다시
        for dev, why in res['busy'].items():
            self.get_logger().warning(f"{dev} 열기 실패(사용 중?): {why}", throttle_duration_sec=10.0)
        if res['joy'] and not self.fixed_joy and not self.joy_dev:
            self._set_joy(res['joy'])
        extra = res['extra'] + ([] if want_drive or not res['drive'] else [res['drive']])
        for dev in extra:
            self.get_logger().warning(f"구동 후보가 하나 더 있다 {dev} — 쓰지 않는다")
        if res['drive_ser'] is not None:
            self._attach(res['drive'], res['drive_ser'])

    def _attach(self, dev, ser):
        self.drive_dev = dev
        self.rx_n = 0
        with self.ser_lock:
            self.ser = ser
        again = self.drive_drops > 0
        self.get_logger().info(f"🚗 구동 우노 {'재연결' if again else '연결'} {dev}"
                               + (f" (끊김 {self.drive_drops}회째 복구)" if again else ""))
        threading.Thread(target=self._read_drive, args=(ser,), daemon=True).start()
        if self.estop:                        # E-STOP 중에 다시 붙었다 — 우노는 막 켜졌을 수도 있으니 곧바로 세운다
            self.write_line("s\n")
            self.write_line("0 0\n")

    def _read_drive(self, ser):
        buf = b""
        reason = "수신 스레드 끝"
        try:
            while not self.stop_evt.is_set() and self.ser is ser:
                data = ser.read(ser.in_waiting or 1)
                if not data:
                    continue
                buf += data
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    fb = boards.parse_drive_line(raw.decode('ascii', errors='ignore').strip())
                    if fb is None:
                        continue
                    self.rx_n += 1
                    self.rx_t = time.time()
                    self.last_fb = fb             # ★끊겨도 지우지 않는다★ (gold 와 같다 — 상태는 '구동 미연결' 로 알린다)
                    self.pub_enc.publish(Float64MultiArray(data=[float(v) for v in fb]))
        except (serial.SerialException, OSError, TypeError, AttributeError) as e:
            reason = f"수신 실패: {e}"          # TypeError · AttributeError = 다른 스레드가 포트를 닫은 순간의 read
        finally:
            if not self.stop_evt.is_set():
                self._drop_drive(ser, reason)

    def _drop_drive(self, ser, reason):
        """끊긴 구동 포트를 닫고 놓는다 → worker 가 RESCAN_S 마다 다시 찾는다 (gold _drop_board).
        읽기 · 쓰기 스레드가 함께 불러도 ★지금 쥔 포트일 때 한 번만★ 처리한다."""
        with self.ser_lock:
            if self.ser is not ser:
                return
            self.ser = None
        dev, self.drive_dev = self.drive_dev, ''
        self.probed.discard(dev)              # 같은 이름으로 돌아와도 다시 열어 본다
        self.drive_drops += 1
        boards.close_quietly(ser)
        self.get_logger().error(f"구동 우노 단절 {dev}: {reason} → 재연결 시도 ({RESCAN_S:.0f}s 마다 찾는다)")

    # ── 상태 ─────────────────────────────────────────────────────────────────
    def publish_status(self):
        now = time.time()
        if self.ser is None:
            drv = "구동 미연결" + (f" (끊김 {self.drive_drops}회 — 다시 찾는 중)" if self.drive_drops else "")
        elif self.rx_n == 0 or now - self.rx_t > DRIVE_STALE_S:
            drv = f"구동 {self.drive_dev} (응답 없음)"
        else:
            r, a, m, ma = self.last_fb
            drv = (f"구동 {self.drive_dev} 받은 {r:+d} {a:+d}° · 실측 {m:+d} {ma:+.1f}°"
                   + (f" (재연결 {self.drive_drops}회)" if self.drive_drops else ""))
        cmd = (f"마지막 명령 {self.last_cmd[0]:+d} {self.last_cmd[1]:+d}° ({now - self.last_cmd_t:.1f}s 전)"
               if self.last_cmd else "명령 없음")
        s = (("🛑 E-STOP 중 | " if self.estop else "")
             + f"아두이노 | {drv} | 조이스틱 {self.joy_dev or '없음'} | {cmd} | 송신 {self.tx_n}"
             + (f" · 버림 {self.drop_n}" if self.drop_n else "")
             + (f" · 구동 없어 못 보냄 {self.nodrive_n}" if self.nodrive_n else "")
             + (f" · E-STOP 으로 막음 {self.estop_n}" if self.estop_n else ""))
        self.pub_status.publish(String(data=s))

    def shutdown(self):
        self.stop_evt.set()
        self.write_line("0 0\n")


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNode()
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
