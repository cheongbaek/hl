#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arduino.py ― 아두이노(UNO R4 Minima) 브리지 — 새 바이너리 규약 ★가작성★ (2026-11-11)
════════════════════════════════════════════════════════════════════════════════
    ros2 run broon arduino          (white1 · white2 의 one_launch.py 가 띄운다)

  구독  /cmd_vel_raw (Twist)     linear.x = 목표 속도 ★km/h★, angular.z = 조향각 ★도★ (+ 좌 / − 우)
        /control_state (Bool)    True = 자율주행이 차를 쥔다. False 면 속도 0 · 조향 0 을 보낸다
  발행  /arduino_status (String) 1 Hz 한 줄 상태

⚠️ 아두이노 쪽 펌웨어가 아직 없다 — 아래 규약은 사용자가 정한 틀(시작바이트 · 속도 · 조향 · 체크섬)에
   PC 쪽에서 ★임시로★ 값을 채운 것이다. 펌웨어가 나오면 '규약' 절의 상수 · 함수만 맞추면 된다.
   실제 보드로 검증한 적이 없다. 옛 Mega 용 문자열 규약(C,vel,steer)은 white/arduino.py 에 있다.

════════════════════════════════════════════════════════════════════════════════
 규약 — PC → 아두이노, 한 프레임 4바이트
════════════════════════════════════════════════════════════════════════════════
    [0] 시작바이트  0xAA
    [1] 속도   int8  km/h   −16 … +16   (음수 = 후진. white1 · white2 는 후진 명령을 내지 않는다)
    [2] 조향   int8  도     −25 … +25   (+ 좌 / − 우 — steer_sign:=-1.0 으로 뒤집는다)
    [3] 체크섬 uint8        ([1] + [2]) 를 256 으로 나눈 나머지 (바이트 값 그대로 더한다)

  · 사용자 지정은 속도 · 조향 각각 int16 이었고 "더 줄일 수 있으면 줄인다" 였다 — 범위가 ±16 · ±25 라
    ★int8(1바이트)★ 로 충분하다. 프레임이 6바이트 → 4바이트.
  · 시작바이트 0xAA 는 유효한 속도(0x00~0x10, 0xF0~0xFF) · 조향(0x00~0x19, 0xE7~0xFF) · 체크섬 값과 겹치지
    않는다 — 받는 쪽이 0xAA 를 만나면 언제나 프레임 처음이다(중간부터 읽어도 바로 맞춘다).
  · 아두이노 → PC 는 정해진 것이 없다. 오는 바이트는 세기만 하고 처음 몇 개를 로그에 남긴다.
    ★바퀴 속도 피드백이 없으므로 white1 · white2 는 속도를 GPS 로만 본다.★

  받는 쪽(아두이노) 참고 — 4바이트를 모아 검사한다:
      if (Serial.available() >= 4 && Serial.read() == 0xAA) {
        int8_t speed = Serial.read(), steer = Serial.read();  uint8_t chk = Serial.read();
        if ((uint8_t)((uint8_t)speed + (uint8_t)steer) == chk) { /* 적용, 마지막 수신 시각 갱신 */ }
      }

 보내는 때 — ★주기가 없다★
  · 값(정수 km/h · 정수 도)이 ★바뀌면 곧바로★ 보낸다. driving 이 20 Hz 로 내므로 많아야 초당 20프레임.
  · 값이 그대로여도 ★1초에 한 번★ 은 같은 값을 다시 보낸다(HEARTBEAT_S) — 아두이노가 PC 생존을 알 수 있게.
    아두이노에는 3초 동안 아무것도 안 오면 정지하는 안전 코드가 들어갈 예정이다.
  · 자율주행이 아닐 때(/control_state False)는 0 · 0 을 그 주기로 보낸다. 리모컨 등으로 손수 몰 때 PC 의
    0 · 0 이 방해된다면 idle_heartbeat:=false — 0 · 0 을 몇 번 보낸 뒤 조용히 있는다(그러면 3초 뒤
    아두이노 안전 정지가 걸린다는 뜻이기도 하다 — 펌웨어가 나오면 다시 정할 것).
  · 자율주행 중 /cmd_vel_raw 가 CMD_TIMEOUT_S 넘게 끊기면 속도 0 을 보낸다. 내려갈 때도 0 · 0 을 보낸다.

 반올림 — 속도는 정수 km/h 로 반올림한다. 0 보다 큰 명령이 0 으로 깎이면 1 km/h 로 올린다
  (driving 의 '아주 느리게' 가 '정지' 로 바뀌지 않게 — white/arduino.py 의 최소 1틱과 같은 처리).

 포트 — USB VID 로 찾는다(ARDUINO_VIDS). UNO R4 Minima = 2341:0069. 여럿이면 R4 Minima 를 먼저 고른다.
  R4 Minima 는 USB 직결(CDC)이라 baud 값은 의미가 없고, Mega 와 달리 포트를 열어도 리셋되지 않는다.
"""

import signal
import struct
import threading
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

import serial
import serial.tools.list_ports

# ══════════════════════════════════════════════════════════════════════════════
#  규약 ★펌웨어가 나오면 여기만 맞춘다★
# ══════════════════════════════════════════════════════════════════════════════
START_BYTE = 0xAA
FRAME_LEN = 4
MAX_SPEED_KMH = 16
MAX_STEER_DEG = 25


def checksum(speed_byte, steer_byte):
    """데이터 두 바이트(0~255 값)의 합을 256 으로 나눈 나머지."""
    return (speed_byte + steer_byte) & 0xFF


def pack_frame(speed_kmh, steer_deg):
    """정수 속도 · 조향 → 4바이트 프레임. 범위를 넘으면 잘라 넣는다."""
    speed = max(-MAX_SPEED_KMH, min(MAX_SPEED_KMH, int(speed_kmh)))
    steer = max(-MAX_STEER_DEG, min(MAX_STEER_DEG, int(steer_deg)))
    body = struct.pack('<bb', speed, steer)
    return bytes([START_BYTE]) + body + bytes([checksum(body[0], body[1])])


def unpack_frame(frame):
    """4바이트 프레임 → (속도, 조향). 시작바이트 · 체크섬이 틀리면 None (시험 · 펌웨어 대조용)."""
    if len(frame) != FRAME_LEN or frame[0] != START_BYTE or checksum(frame[1], frame[2]) != frame[3]:
        return None
    return struct.unpack('<bb', frame[1:3])


def to_int_speed(v_kmh):
    """목표 속도 [km/h] → 정수. 0 보다 큰(작은) 명령은 최소 ±1 로."""
    n = int(round(v_kmh))
    if n == 0 and v_kmh > 0.05:
        n = 1
    elif n == 0 and v_kmh < -0.05:
        n = -1
    return max(-MAX_SPEED_KMH, min(MAX_SPEED_KMH, n))


# ── 포트 ──
ARDUINO_VIDS = {0x2341, 0x2A03, 0x1A86}    # Arduino SA · 호환(arduino.org) · CH340
R4_MINIMA = (0x2341, 0x0069)
BAUD = 115200                # R4 Minima(USB CDC)에서는 의미 없음

# ── 송신 ──
HEARTBEAT_S = 1.0            # 값이 그대로여도 이 간격으로 다시 보낸다
CMD_TIMEOUT_S = 0.5          # 자율주행 중 /cmd_vel_raw 가 이만큼 끊기면 속도 0
IDLE_STOP_N = 3              # idle_heartbeat:=false 일 때, 손을 떼면서 0 · 0 을 이만큼 보내고 조용히 한다
CHECK_PERIOD_S = 0.05        # 하트비트 · 시한을 살피는 주기
RX_LOG_N = 5                 # 연결마다 아두이노에서 온 바이트를 이만큼까지 로그에 남긴다


def find_port(exclude=()):
    """아두이노 포트 경로. R4 Minima 를 먼저, 없으면 ARDUINO_VIDS 중 번호가 작은 것. 없으면 ''."""
    hits = []
    for p in serial.tools.list_ports.comports():
        if p.vid in ARDUINO_VIDS and p.device not in exclude:
            hits.append((0 if (p.vid, p.pid) == R4_MINIMA else 1, p.device))
    hits.sort()
    return hits[0][1] if hits else ""


class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_node')
        self.declare_parameter('port', '')              # 비우면 VID 로 찾는다
        self.declare_parameter('baud', BAUD)
        self.declare_parameter('steer_sign', 1.0)       # −1.0 = 조향 부호 뒤집기 (펌웨어가 + 우 일 때)
        self.declare_parameter('idle_heartbeat', True)  # 자율주행이 아닐 때도 0 · 0 을 1초마다 보낸다
        self.port_param = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.steer_sign = -1.0 if float(self.get_parameter('steer_sign').value) < 0 else 1.0
        self.idle_heartbeat = bool(self.get_parameter('idle_heartbeat').value)

        self.ser = None
        self.ser_lock = threading.Lock()
        self.port = ''
        self.stop_evt = threading.Event()

        self.enabled = False
        self.cmd_kmh = 0.0
        self.cmd_steer = 0.0
        self.cmd_t = 0.0
        self.sent = None             # 마지막으로 보낸 (속도, 조향)
        self.sent_t = 0.0
        self.idle_left = 0
        self.tx_n = 0
        self.rx_n = 0
        self.rx_logged = 0

        self.pub_status = self.create_publisher(String, '/arduino_status', 10)
        self.create_subscription(Twist, '/cmd_vel_raw', self.cb_cmd, 10)
        self.create_subscription(Bool, '/control_state', self.cb_state, 10)
        self.create_timer(CHECK_PERIOD_S, self.flush)
        self.create_timer(1.0, self.publish_status)
        threading.Thread(target=self.port_worker, daemon=True).start()
        self.get_logger().info(
            f"아두이노 브리지(가작성 규약: 0x{START_BYTE:02X} · 속도 int8 km/h ±{MAX_SPEED_KMH} · 조향 int8 도 "
            f"±{MAX_STEER_DEG} · 체크섬) — 바뀌면 즉시 + {HEARTBEAT_S:.0f}초마다 재송신, "
            f"조향 부호 {'+ 좌' if self.steer_sign > 0 else '+ 우(뒤집음)'}, "
            f"대기 중 {'0·0 계속 송신' if self.idle_heartbeat else '송신 안 함'}")

    # ── ROS 콜백 ─────────────────────────────────────────────────────────────
    def cb_cmd(self, msg: Twist):
        self.cmd_kmh = float(msg.linear.x)
        self.cmd_steer = float(msg.angular.z)
        self.cmd_t = time.time()
        self.flush()                 # 값이 바뀌었으면 곧바로 나간다

    def cb_state(self, msg: Bool):
        on = bool(msg.data)
        if on == self.enabled:
            return
        self.enabled = on
        if not on:
            self.idle_left = IDLE_STOP_N
        self.get_logger().info(f"제어권 {'자율주행' if on else '해제 — 0 · 0 송신'}")
        self.flush()

    # ── 송신 ─────────────────────────────────────────────────────────────────
    def target(self, now):
        """지금 보내야 할 (속도, 조향) 정수 쌍. 보낼 것이 없으면 None."""
        if not self.enabled:
            if self.idle_heartbeat or self.idle_left > 0:
                return (0, 0)
            return None
        v = self.cmd_kmh
        if now - self.cmd_t > CMD_TIMEOUT_S:
            v = 0.0
            if self.cmd_t > 0.0:
                self.get_logger().warning(f"/cmd_vel_raw {now - self.cmd_t:.1f}s 끊김 — 속도 0 송신",
                                          throttle_duration_sec=2.0)
        steer = int(round(self.cmd_steer * self.steer_sign))
        return (to_int_speed(v), max(-MAX_STEER_DEG, min(MAX_STEER_DEG, steer)))

    def flush(self):
        """값이 바뀌었거나 HEARTBEAT_S 가 지났으면 한 프레임 보낸다."""
        now = time.time()
        want = self.target(now)
        if want is None:
            return
        if want == self.sent and now - self.sent_t < HEARTBEAT_S:
            return
        with self.ser_lock:
            if self.ser is None:
                return
            try:
                self.ser.write(pack_frame(*want))
            except (serial.SerialException, OSError) as e:
                self.get_logger().warning(f"시리얼 송신 오류: {e}", throttle_duration_sec=2.0)
                return
        self.sent, self.sent_t = want, now
        self.tx_n += 1
        if not self.enabled and self.idle_left > 0:
            self.idle_left -= 1
            self.sent_t = now - HEARTBEAT_S + CHECK_PERIOD_S     # 손 떼는 0 · 0 은 연달아 보낸다

    # ── 포트 (스레드) — 찾기 → 열기 → 읽기, 빠지면 다시 찾는다 ───────────────
    def port_worker(self):
        while not self.stop_evt.is_set():
            dev = self.port_param or find_port()
            if not dev:
                self.port = ''
                self.get_logger().warning("아두이노 포트를 찾는 중 (VID 2341/2a03/1a86)", throttle_duration_sec=10.0)
                self.stop_evt.wait(1.0)
                continue
            try:
                ser = serial.Serial(dev, self.baud, timeout=0.1, exclusive=True)
            except (serial.SerialException, OSError) as e:
                self.get_logger().warning(f"아두이노 포트 열기 실패 {dev}: {e}", throttle_duration_sec=10.0)
                self.stop_evt.wait(2.0)
                continue
            self.port = dev
            self.sent, self.rx_n, self.rx_logged = None, 0, 0
            self.get_logger().info(f"아두이노 연결 {dev}")
            with self.ser_lock:
                self.ser = ser
            try:
                while not self.stop_evt.is_set():
                    data = ser.read(64)
                    if data:
                        self._on_rx(data)
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"아두이노 단절: {e} — 다시 찾는다")
            finally:
                with self.ser_lock:
                    self.ser = None
                try:
                    ser.close()
                except (serial.SerialException, OSError):
                    pass
            self.stop_evt.wait(1.0)

    def _on_rx(self, data):
        """아두이노 → PC 는 규약이 없다 — 세기만 하고 처음 몇 번만 보여 준다."""
        self.rx_n += len(data)
        if self.rx_logged < RX_LOG_N:
            self.rx_logged += 1
            self.get_logger().info(f"아두이노에서 {len(data)}바이트: {data[:24].hex(' ')} {data[:24]!r}")

    def publish_status(self):
        now = time.time()
        if self.ser is None:
            s = f"아두이노 미연결 ({self.port_param or '자동 탐색'})"
        else:
            own = "자율" if self.enabled else "대기"
            last = f"{self.sent[0]}km/h {self.sent[1]}° ({now - self.sent_t:.1f}s 전)" if self.sent else "보낸 것 없음"
            s = f"아두이노 {self.port} | {own} | 마지막 송신 {last} | 송신 {self.tx_n}프레임 · 수신 {self.rx_n}바이트"
        self.pub_status.publish(String(data=s))

    def shutdown(self):
        self.stop_evt.set()
        with self.ser_lock:
            if self.ser is not None:
                try:
                    for _ in range(3):
                        self.ser.write(pack_frame(0, 0))
                    self.ser.flush()
                except (serial.SerialException, OSError):
                    pass


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
