#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
arduino.py ― 아두이노 Mega 브리지 (1/5카 구동 · 조향) [black motor.py 계승]
════════════════════════════════════════════════════════════════════════════════
    ros2 run white arduino          (one_launch.py 가 띄운다)

  구독  /cmd_vel_raw (Twist)    linear.x = 목표속도 m/s, angular.z = 조향각 deg (★+ 좌 / − 우★, black 규약)
        /control_state (Bool)   True = 자율주행이 모터를 쥔다 → 20 Hz 로 'C,틱,조향' 송신
                                False = 'C,0,0' 을 몇 번 보내고 송신을 멈춘다
  발행  /encoder (Int32)        E 줄의 d_val 원값 [틱/PID 주기]
        /wheel_speed (Float32)  위를 m/s 로 (전진 +)
        /arduino_status (String) 1 Hz 한 줄 상태

────────────────────────────────────────────────────────────────────────────────
 펌웨어 — ★115200 bps★ (2026-11-09 사용자 지정). 짝은 hl_ws/tool/arduino_code.ino (white_fw)
────────────────────────────────────────────────────────────────────────────────
  PC → 아두이노  'C,vel,steer\n'  vel = 틱/PID주기 정수(부호 = 방향), steer = 정수 도
                 'S,0|1\n'        제어 상태 표시 — 펌웨어는 E 줄로 되돌려 보낼 뿐이다
  아두이노 → PC  E 줄의 필드 수로 펌웨어를 알아본다(self.fmt). 그 밖의 줄은 처음 몇 개만 로그에 남긴다.
    white_fw  'H,white_fw,<버전>,<baud>,ps2=<n>'  부팅 때 한 번
              'E,d_val,sensorValue,PWM,Steering_Angle,state,remote,dt_ms'   20 ms 마다
              → d_val 을 잰 시간 dt_ms 가 줄에 있어 속도 환산에 그대로 쓴다. ±19 눈금.
    v6        'E,d_val' 또는 'E,d_val,dt_us' (black 계열). 2026-11-09 차에 올라가 있던 스케치가 이 꼴이다 —
              부팅 줄 "Arduino Ready. PS2 priority + comms watchdog 500ms failsafe …", E 사이에 P · A · D 줄
              (뜻 모름, 읽지 않는다). 원본 소스는 이 노트북 · GitHub 어디에도 없다. ±21 눈금.
    Vr.1      'E,d_val,sensorValue,PWM,Steering_Angle,state' (~/Arduino/arduino_backup, 57600 bps — baud:=57600)

 속도 환산 — tick = v · T · 틱수 / 둘레  (black motor.py 와 같은 식, 둘레 0.8482 m · 300틱/회전)
  T(PID 주기) : white_fw 는 E 줄의 dt_ms(평활), 그 밖에는 ★E 줄이 오는 간격★ 으로 잰다.
  enc_period_s 로 고정할 수 있다.
  ⚠️ 300틱/회전은 black 코드 값이고 이 저장소에서 다시 재지 않았다 — driving 이 GPS 속도와의
     비를 로그로 낸다(driving.py '엔코더/GPS 속도비').

 조향 환산 — 두 계열은 같은 가변저항 범위(AD −460~423)를 v6 은 ±21°, Vr.1 · white_fw 는 ±19° 로 눈금을
  매겼다. driving 의 조향각(black 튜닝값: 트림 −1.5°, 게인 0.85/0.82)은 black 이 돌던 v6 눈금이므로
  steer_scale 을 0(기본)으로 두면 ★알아본 펌웨어에 맞춰★ 고른다 — v6 1.0 · ±21, 그 밖 19/21 · ±19.

────────────────────────────────────────────────────────────────────────────────
 리모컨 — ★ROS 는 판정하지 않는다★ (2026-11-09 사용자 결정: 아두이노에게 모든 권한)
────────────────────────────────────────────────────────────────────────────────
  PS2 무선 리모컨은 아두이노에 직결돼 있고, L1/R1 을 누르는 동안 펌웨어가 구동 · 조향을 모두 쥔다
  (white_fw 는 그동안 C 명령을 받기만 하고 쓰지 않는다). 이 노드는 그 사실로 아무것도 하지 않는다 —
  자율주행 중이면 명령을 계속 보내고, 리모컨을 놓으면 펌웨어가 다시 PC 명령을 따른다.
  white_fw 의 E 줄 remote 필드는 /arduino_status 에 ★표시만★ 한다.

 명령 시한 — 이 노드는 /cmd_vel_raw 가 CMD_TIMEOUT_S 넘게 끊기면 0 을 보내고, 내려갈 때 C,0,0 을 보낸다.
  white_fw 는 C 가 500 ms 넘게 안 오면 스스로 속도 0 으로 선다(PC 가 통째로 죽어도).
"""

import signal
import threading
import time
from collections import deque

import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32, String

import serial

from white import ports

# ── 하드웨어 ──
BAUD = 115200                # white_fw · 지금 차의 v6 계열 스케치. Vr.1(arduino_backup) 이면 57600
WHEEL_CIRC_M = 0.8482        # 바퀴 둘레 (black motor.py)
TICKS_PER_REV = 300.0        # 엔코더 1회전 틱 (black motor.py — 미검증, 위 헤더)
FW_SCALE = {                 # 펌웨어 → (driving 도 → 펌웨어 눈금 배율, 펌웨어 조향 상한)
    'v6': (1.0, 21),
    'white_fw': (19.0 / 21.0, 19),
    'Vr.1': (19.0 / 21.0, 19),
}
SCALE_UNKNOWN = (19.0 / 21.0, 19)   # 펌웨어를 알아보기 전 (작은 쪽)
MAX_SPEED_MS = 3.0           # 최후 클램프. 실제 상한은 driving 이 정한다

# ── 송신 ──
TX_PERIOD_S = 0.05           # 20 Hz
CMD_TIMEOUT_S = 0.5          # /cmd_vel_raw 가 이만큼 끊기면 0 을 보낸다
STOP_REPEAT_N = 5            # 손을 뗄 때 C,0,0 을 이만큼 보내고 멈춘다 (0.25 s)
BOOT_WAIT_S = 2.5            # Mega 는 포트를 열면 리셋된다 — 부트로더가 지나갈 때까지 ★보내지 않는다★ (받기는 곧바로)

# ── PID 주기 ──
PERIOD_DEFAULT_S = 0.020     # 알기 전 가정값 (white_fw 20 ms)
PERIOD_MIN_S, PERIOD_MAX_S = 0.005, 0.06
PERIOD_MIN_N = 20            # 도착 간격으로 잴 때 이만큼 받아야 잰 값을 쓴다
PERIOD_GAP_S = 0.25          # E 줄 사이가 이보다 벌어지면 끊긴 것 — 창을 비운다
PERIOD_LPF = 0.1             # white_fw dt_ms 평활

RX_STALE_S = 1.0             # E 줄이 이만큼 없으면 '응답 없음'
TEXT_LOG_N = 5               # 연결마다 E 아닌 줄을 이만큼까지 로그에 남긴다 (부팅 줄 · 엉뚱한 스케치 알아보기)
SILENT_TAGS = ('P', 'A', 'D')   # v6 계열 스케치의 E 사이 줄 — 로그에 남기지 않는다


class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_node')
        self.declare_parameter('port', '')            # 비우면 VID 로 찾는다
        self.declare_parameter('baud', BAUD)
        self.declare_parameter('wheel_circ_m', WHEEL_CIRC_M)
        self.declare_parameter('ticks_per_rev', TICKS_PER_REV)
        self.declare_parameter('enc_period_s', 0.0)   # 0 = 펌웨어 dt · E 줄 간격으로 잰다
        self.declare_parameter('steer_scale', 0.0)    # 0 = 알아본 펌웨어에 맞춘다 (FW_SCALE)
        self.declare_parameter('fw_steer_max', 0)     # 0 = 알아본 펌웨어에 맞춘다
        self.port_param = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.circ = float(self.get_parameter('wheel_circ_m').value)
        self.tpr = float(self.get_parameter('ticks_per_rev').value)
        self.period_fixed = float(self.get_parameter('enc_period_s').value)
        self.steer_scale_param = float(self.get_parameter('steer_scale').value)
        self.fw_steer_max_param = int(self.get_parameter('fw_steer_max').value)

        self.ser = None
        self.ser_lock = threading.Lock()
        self.port = ''
        self.open_t = 0.0
        self.stop_evt = threading.Event()

        # 명령
        self.enabled = False
        self.cmd_v = 0.0
        self.cmd_steer = 0.0
        self.cmd_t = 0.0
        self.stop_left = 0
        self.tick_sent = 0
        self.fw_steer_sent = 0

        # 수신
        self.rx_t = 0.0
        self.rx_times = deque(maxlen=120)
        self.period = PERIOD_DEFAULT_S
        self.fmt = '?'
        self.fw_banner = ''           # H 줄 또는 처음 본 글자 줄
        self.text_logged = 0
        self.last_pwm = None
        self.fw_remote = None         # white_fw 의 remote 필드 (표시만)
        self.d_val = 0
        self.speed = 0.0

        self.pub_enc = self.create_publisher(Int32, '/encoder', 10)
        self.pub_speed = self.create_publisher(Float32, '/wheel_speed', 10)
        self.pub_status = self.create_publisher(String, '/arduino_status', 10)
        self.create_subscription(Twist, '/cmd_vel_raw', self.cb_cmd, 10)
        self.create_subscription(Bool, '/control_state', self.cb_state, 10)
        self.create_timer(TX_PERIOD_S, self.tx_loop)
        self.create_timer(1.0, self.publish_status)

        self.worker = threading.Thread(target=self.rx_worker, daemon=True)
        self.worker.start()
        scale = "펌웨어에 맞춰 자동" if self.steer_scale_param <= 0 else f"×{self.steer_scale_param:.4f}"
        self.get_logger().info(
            f"아두이노 브리지 — baud {self.baud}, 둘레 {self.circ} m, {self.tpr:.0f}틱/회전, "
            f"PID 주기 {'펌웨어 dt · E 줄 간격으로 측정' if self.period_fixed <= 0 else f'{self.period_fixed*1000:.1f} ms 고정'}, "
            f"조향 {scale} · 리모컨 판정 없음(아두이노가 쥔다)")

    # ══════════════════════════════════════════════════════════════════════════
    #  ROS 콜백
    # ══════════════════════════════════════════════════════════════════════════
    def cb_cmd(self, msg: Twist):
        self.cmd_v = max(-MAX_SPEED_MS, min(MAX_SPEED_MS, float(msg.linear.x)))
        self.cmd_steer = float(msg.angular.z)
        self.cmd_t = time.time()

    def cb_state(self, msg: Bool):
        on = bool(msg.data)
        if on == self.enabled:
            return
        self.enabled = on
        if on:
            self.stop_left = 0
            self._write('S,1\n')
        else:
            self.stop_left = STOP_REPEAT_N
            self._write('S,0\n')
        self.get_logger().info(f"제어권 {'자율주행' if on else '해제(송신 중지)'}")

    # ══════════════════════════════════════════════════════════════════════════
    #  송신
    # ══════════════════════════════════════════════════════════════════════════
    def steer_scale(self):
        """(배율, 펌웨어 조향 상한) — 파라미터가 0 이면 알아본 펌웨어에 맞춘다."""
        scale, lim = FW_SCALE.get(self.fmt, SCALE_UNKNOWN)
        if self.steer_scale_param > 0:
            scale = self.steer_scale_param
        if self.fw_steer_max_param > 0:
            lim = self.fw_steer_max_param
        return scale, lim

    def speed_to_tick(self, v):
        t = v * self.period_now() * self.tpr / self.circ
        if v > 0.0 and t < 1.0:
            return 1                   # 아주 느린 명령도 0 이 되지 않게 (black motor.py)
        if v < 0.0 and t > -1.0:
            return -1
        return int(round(t))

    def tx_loop(self):
        if self.ser is None:
            return
        now = time.time()
        if now - self.open_t < BOOT_WAIT_S:
            return                     # 부트로더가 지나가는 중 — 아무것도 보내지 않는다
        if self.enabled:
            v, st = self.cmd_v, self.cmd_steer
            if now - self.cmd_t > CMD_TIMEOUT_S:
                v = 0.0
                if self.cmd_t > 0.0:
                    self.get_logger().warning(
                        f"/cmd_vel_raw {now - self.cmd_t:.1f}s 끊김 — 0 송신",
                        throttle_duration_sec=2.0)
            tick = self.speed_to_tick(v) if abs(v) > 1e-6 else 0
            scale, lim = self.steer_scale()
            fw = max(-lim, min(lim, int(round(st * scale))))
            self._send_c(tick, fw)
        elif self.stop_left > 0:
            self.stop_left -= 1
            self._send_c(0, 0)
        # 그 밖에는 보내지 않는다 — 매핑(리모컨 주행) 중에 PC 명령이 끼어들 일이 없게

    def _send_c(self, tick, fw):
        self.tick_sent, self.fw_steer_sent = tick, fw
        self._write(f'C,{tick},{fw}\n')

    def _write(self, line):
        with self.ser_lock:
            if self.ser is None:
                return
            try:
                self.ser.write(line.encode('ascii'))
            except (serial.SerialException, OSError) as e:
                self.get_logger().warning(f"시리얼 송신 오류: {e}", throttle_duration_sec=2.0)

    # ══════════════════════════════════════════════════════════════════════════
    #  수신 (스레드) — 포트 찾기 → 열기 → 읽기, 빠지면 다시 찾는다
    # ══════════════════════════════════════════════════════════════════════════
    def rx_worker(self):
        while not self.stop_evt.is_set():
            dev = self.port_param or ports.find_arduino()
            if not dev:
                self.port = ''
                self.get_logger().warning("아두이노 포트를 찾는 중 (VID 2341/2a03/1a86)",
                                          throttle_duration_sec=10.0)
                self.stop_evt.wait(1.0)
                continue
            try:
                ser = serial.Serial(dev, self.baud, timeout=0.1, exclusive=True)
            except (serial.SerialException, OSError) as e:
                self.get_logger().warning(f"아두이노 포트 열기 실패 {dev}: {e}",
                                          throttle_duration_sec=10.0)
                self.stop_evt.wait(2.0)
                continue
            self.port = dev
            self.open_t = time.time()
            self.fmt, self.fw_banner, self.text_logged = '?', '', 0
            self.get_logger().info(f"아두이노 연결 {dev} @ {self.baud} — 리셋 · 부팅 {BOOT_WAIT_S:.1f}s 동안 송신 보류")
            self.rx_times.clear()
            with self.ser_lock:
                self.ser = ser
            try:
                self._read_loop(ser)
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

    def _read_loop(self, ser):
        buf = b''
        while not self.stop_evt.is_set():
            data = ser.read(256)
            if not data:
                continue
            buf += data
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                self._on_line(line.strip().decode('ascii', errors='replace'))
            if len(buf) > 512:
                buf = b''

    def _on_line(self, line):
        if not line:
            return
        f = line.split(',')
        if f[0] != 'E' or len(f) < 2:
            self._on_text(line, f[0])
            return
        try:
            vals = [int(float(x)) for x in f[1:]]
        except ValueError:
            return
        now = time.time()
        n = len(vals)
        dt = None
        if n >= 7:
            fmt = 'white_fw'             # E,d,sensor,PWM,steer,state,remote,dt_ms
            pwm, self.fw_remote = vals[2], vals[5]
            dt = vals[6] / 1000.0
        elif n == 5:
            fmt, pwm = 'Vr.1', vals[2]   # E,d,sensor,PWM,steer,state
        elif n in (1, 2):
            fmt, pwm = 'v6', None        # E,d_val[,dt_us]
            if n == 2 and vals[1] > 0:
                dt = vals[1] / 1e6
        else:
            return
        if fmt != self.fmt:
            scale, lim = FW_SCALE.get(fmt, SCALE_UNKNOWN)
            self.get_logger().info(f"펌웨어 형식 {fmt} — 조향 ×{scale:.4f} (±{lim})"
                                   + ("" if self.steer_scale_param <= 0 else " ※ steer_scale 파라미터가 우선"))
            self.fmt = fmt
        self._track_period(now, dt)
        if dt is None or not PERIOD_MIN_S <= dt <= PERIOD_MAX_S:
            dt = self.period_now()
        self.d_val = vals[0]
        self.speed = self.d_val * self.circ / self.tpr / dt
        self.rx_t = now
        if pwm is not None:
            self.last_pwm = pwm
        self.pub_enc.publish(Int32(data=self.d_val))
        self.pub_speed.publish(Float32(data=float(self.speed)))

    def _on_text(self, line, tag):
        """E 가 아닌 줄 — 부팅 줄(H) · 엉뚱한 스케치의 출력을 처음 몇 개만 남긴다(깨진 글자면 baud 를 의심)."""
        if tag == 'H':
            self.fw_banner = line[2:]
            self.get_logger().info(f"펌웨어 부팅 줄: {line}")
            return
        if tag in SILENT_TAGS:
            return
        letters = sum(1 for c in line if c.isalpha())
        junk = sum(1 for c in line if c == '�' or not c.isprintable())
        if letters < 3 and junk == 0:
            return                     # 줄이 끊긴 숫자 조각 ('0' 등) — 남길 것이 없다
        if not self.fw_banner and letters >= 3 and junk == 0:
            self.fw_banner = line[:60]
        if self.text_logged < TEXT_LOG_N:
            self.text_logged += 1
            hint = f"  ← 깨진 글자 {junk}개: baud({self.baud})가 스케치와 다른가?" if junk > len(line) // 3 else ""
            self.get_logger().info(f"아두이노 글자 줄: {line[:80]!r}{hint}")

    def _track_period(self, now, dt):
        if dt is not None and PERIOD_MIN_S <= dt <= PERIOD_MAX_S:
            self.period += PERIOD_LPF * (dt - self.period)      # 펌웨어가 잰 값
            return
        if self.rx_times and now - self.rx_times[-1] > PERIOD_GAP_S:
            self.rx_times.clear()
        self.rx_times.append(now)
        if len(self.rx_times) >= PERIOD_MIN_N:
            p = (self.rx_times[-1] - self.rx_times[0]) / (len(self.rx_times) - 1)
            if PERIOD_MIN_S <= p <= PERIOD_MAX_S:
                self.period = p

    def period_now(self):
        return self.period_fixed if self.period_fixed > 0.0 else self.period

    def publish_status(self):
        now = time.time()
        if self.ser is None:
            s = f"아두이노 미연결 ({self.port_param or '자동 탐색'})"
        elif now - self.rx_t > RX_STALE_S:
            seen = f" — 받은 줄: {self.fw_banner!r}" if self.fw_banner else ""
            s = (f"아두이노 {self.port} 응답 없음 ({now - self.rx_t:.0f}s)" if self.rx_t else
                 f"아두이노 {self.port} 응답 대기 @ {self.baud}") + seen
        else:
            hz = 1.0 / self.period if self.period > 0 else 0.0
            own = "자율" if self.enabled else "대기"
            if self.fw_remote:
                own += " · 리모컨이 쥐는 중(아두이노 우선)"
            s = (f"아두이노 {self.port} {self.fmt} {hz:.0f}Hz T={self.period_now()*1000:.1f}ms | "
                 f"{own} | 명령 {self.tick_sent}틱 조향 {self.fw_steer_sent}° | "
                 f"실측 {self.speed:+.2f}m/s PWM {self.last_pwm if self.last_pwm is not None else '-'}")
        self.pub_status.publish(String(data=s))

    # ══════════════════════════════════════════════════════════════════════════
    def shutdown(self):
        self.stop_evt.set()
        with self.ser_lock:
            if self.ser is not None:
                try:
                    for _ in range(3):
                        self.ser.write(b'C,0,0\n')
                    self.ser.write(b'S,0\n')
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
