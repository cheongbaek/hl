#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
joy.py ― 조이스틱 보드 → /cmd_vel · /joy318 + 화면 [broon]  (2026-11-19, tool/joy.py 를 ROS 노드로)
════════════════════════════════════════════════════════════════════════════════
    ros2 run broon joy              화면(tkinter)과 함께
    ros2 run broon joy --nogui      화면 없이 (1 초마다 상태 한 줄)
    ros2 run broon joy --ros-args -p port:=/dev/ttyACM1     포트를 직접 정할 때

조이스틱 보드 = 메가(tool/joy_nolcd.ino) 또는 나노(tool/joy_nano.ino) — 핀만 다르고 줄은 똑같다.
★보드에는 아무것도 쓰지 않고 읽기만 한다★ (2026-11-19).

★포트는 스스로 찾지 않는다★ — arduino 노드가 판별해 /joy_port (래치)로 알려 준 포트를 연다(사용자 지정).
arduino 노드가 없으면 기다리기만 한다(화면에 '포트 기다리는 중'). 포트를 열면 보드가 리셋되고 "J,RESET" 을 보낸다.

SWA (ON/OFF — 조이스틱 보드 안에서 바뀐다. 켜면 OFF) — 2026-11-19 사용자 지정
  ON  = 조이스틱 신호를 보낸다.   OFF = 스틱을 움직여도 무시한다(아무것도 안 낸다).
  ★자율주행 정지는 SWA 가 ON 으로 바뀌는 순간에만★ 일어난다 — 아래 /joy318.

발행
  /cmd_vel (std_msgs/Int32MultiArray) data = [목표 펄스, 목표 조향각]  — SWA ON 일 때만
      L 스틱 세로 → 펄스 : 위 + (전진) · 아래 − (후진), 최대 ±92.   R 스틱 가로 → 조향 : 왼쪽 − · 오른쪽 +, 최대 ±21°.
      ★스틱이 0 이 아니면★ 값이 바뀌는 즉시 + 그대로여도 RESEND_S 마다 보낸다 (메가 250 Hz 줄마다 다시 계산).
      ★스틱이 0 으로 돌아오면★ "0 0" 을 ZERO_TAIL_S 동안 RESEND_S 마다 보낸 뒤 ★조용해진다★ —
        계속 "0 0" 을 내면 자율주행(driving)의 /cmd_vel 과 섞이기 때문이다. 조용하면 우노는 명령 시한(2 초)이
        지나도 이미 서 있으므로 그대로다.
      SWA 가 바뀔 때(ON ↔ OFF) · 메가 끊김 · 노드 종료 때도 "0 0" 마무리를 보낸다 — ON 으로 바꾼 순간
        자율주행이 마지막으로 보낸 속도로 차가 계속 가지 않게(그러면 우노 시한 2 초까지 간다).
      영점 보정 중에는 스틱을 0 으로 본다.
  /joy318 (std_msgs/Int32, 늘 1) — ★SWA ON 동안★ JOY318_S 마다(ON 으로 바뀌는 순간 곧바로 한 번).
      white1 driving 은 이것을 들으면 자율주행을 멈추고 /cmd_vel 발행을 그친다. ON 인 동안은 자율주행 출발을 거절한다
      → 자율주행은 SWA OFF 에서만 돌고, 조이스틱이 자율을 멈추는 때는 ON 으로 바꾸는 순간뿐이다.

구독
  /joy_port (String, 래치) — 열 포트.
  /enc_pot (Float64MultiArray) — 우노가 받은 명령 · 실측. ★화면에 띄우기만 한다★ (보드로 넘기지 않는다).
  보드에 쓰지 않으니 부트로더 문제도 없다 — 예전에 PC 가 보드에 줄을 쓰다가 메가 2560 부트로더(stk500v2)가
  스케치로 못 넘어간 일이 있었다(2026-11-19 실물). 다시 보드에 쓰는 기능을 넣으면 이 점을 먼저 볼 것.

스틱 → 값 · 영점 (tool/joy.py 와 같다)
  영점에서 ADC 120 안은 0 (데드존), 그 밖은 1 ~ 최댓값에 직선으로 잇는다.
  영점 : ★파이썬이 보드에서 받은 값으로 잡는다★ — 연결하면 처음 0.3 s (75 줄)의 중앙값.
         보드 리셋 버튼 → 부팅 끝에 "J,RESET" → ★받는 즉시 같은 방법(0.3 s 중앙값)으로 다시 잡는다★ (기다리지 않는다 —
         2026-11-19 사용자 지정. 옛 joy_gold · tool/joy.py 는 3 초 뒤 값 한 벌이었다). 보정하는 0.3 s 동안 스틱은 0 으로 본다.
  보드 줄이 2.5 s 넘게 안 오면 화면 상태 줄에 '보드에서 줄이 안 온다' 를 띄운다(다른 스케치 · 배선).
⚠️ joy_nolcd · joy_nano 와 이 노드는 아직 실물로 함께 돌려 보지 않았다.
"""

import os
import signal
import statistics
import sys
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64MultiArray, Int32, Int32MultiArray, String

import serial

from broon import boards
from broon.boards import CMD_TOPIC, ENC_POT_TOPIC, ESTOP_TOPIC, JOY318_TOPIC, JOY_PORT_TOPIC

RESEND_S = 0.1               # 스틱이 그대로여도 이 간격으로 /cmd_vel 을 다시 낸다
ZERO_TAIL_S = 0.5            # 0 으로 돌아오면 "0 0" 을 이만큼 내고 조용해진다
JOY318_S = 0.1               # SWA ON 동안 /joy318 간격
TICK_S = 0.02                # 재발행 · 마무리 · /joy318 를 살피는 주기

CALIBRATION_SAMPLES = 75     # 영점에 쓰는 보드 줄 수 (250 Hz → 0.3 s) — 연결 때 · 리셋 때 같다
BOARD_SILENT_S = 2.5         # 이만큼 줄이 없으면 화면에 '보드에서 줄이 안 온다' (열 때 부트로더 ~1.7 s 를 넘게)
ADC_MAX = 1023
DEADZONE_RAW = 120
DEFAULT_CENTER = ADC_MAX // 2
FB_STALE_S = 0.5             # /enc_pot 이 이만큼 없으면 화면에 '---'
REOPEN_S = 1.0

GUI_HZ = 60
PAD = 260
DOT_R = 9
CENTER_R = 12
SW_R = 16
BG = "#1e1e1e"
PAD_BG = "#2b2b2b"
LINE = "#4a4a4a"
DOT_COLOR = "#4fc3f7"
PRESS_COLOR = "#ef5350"
IDLE_COLOR = "#3a3a3a"
OK_COLOR = "#66bb6a"
WARN_COLOR = "#ffb74d"
TEXT = "#dddddd"
COLORS = {"text": TEXT, "ok": OK_COLOR, "bad": PRESS_COLOR, "warn": WARN_COLOR}


# ── 스틱 값 → 명령 (tool/joy.py 그대로) ──────────────────────────────────────
def normalize(raw, center):
    center = max(1, min(ADC_MAX - 1, center))
    if raw >= center:
        return (raw - center) / (ADC_MAX - center)
    return (raw - center) / center


def scaled_with_deadzone(raw, center, deadzone_raw, max_out):
    center = max(1, min(ADC_MAX - 1, center))
    diff = raw - center
    span = (ADC_MAX - center) if diff >= 0 else center
    mag = abs(diff)
    if mag < deadzone_raw:
        return 0.0
    remain = span - deadzone_raw
    magnitude = max_out if remain <= 0 else 1 + (mag - deadzone_raw) / remain * (max_out - 1)
    return magnitude if diff >= 0 else -magnitude


def stick_to_command(y1, x2, center_y1, center_x2):
    """L 세로 → 목표 펄스 (위 +), R 가로 → 조향각 (왼쪽 −). 둘 다 정수."""
    ref = int(round(scaled_with_deadzone(y1, center_y1, DEADZONE_RAW, boards.PULSE_MAX)))
    ang = int(round(-scaled_with_deadzone(x2, center_x2, DEADZONE_RAW, boards.STEER_MAX)))
    return ref, ang


def signed2(v):
    v = int(round(v))
    return f"{'-' if v < 0 else '+'}{min(abs(v), 99):02d}"


class JoyNode(Node):
    def __init__(self):
        super().__init__('joy')
        self.declare_parameter('port', '')
        self.fixed_port = str(self.get_parameter('port').value)

        self.lock = threading.Lock()
        self.stop_evt = threading.Event()
        self.want_port = self.fixed_port
        self.ser = None
        self.ser_lock = threading.Lock()
        self.port = ''

        # x1, y1, k1, x2, y2, k2, swa, swb (버튼 active-low), on
        self.data = (DEFAULT_CENTER, DEFAULT_CENTER, 1, DEFAULT_CENTER, DEFAULT_CENTER, 1, 1, 1, 0)
        self.joy_lines = 0
        self.center = {"x1": DEFAULT_CENTER, "y1": DEFAULT_CENTER, "x2": DEFAULT_CENTER, "y2": DEFAULT_CENTER}
        self.calib_buf = None
        self.line_t = 0.0            # 보드에서 마지막으로 올바른 줄을 받은 때 (monotonic)
        self.status = ("조이스틱 포트 기다리는 중 — arduino 노드가 /joy_port 로 알려 준다", "warn")

        self.on = False              # SWA ON (보드가 알려 준 값)
        self.cmd = (0, 0)            # 지금 스틱이 뜻하는 값 (OFF · 보정 중이면 0 0)
        self.engaged = False         # 0 이 아닌 값을 내는 중
        self.tail_until = None       # "0 0" 마무리 끝 시각
        self.last_pub = None         # 마지막으로 낸 /cmd_vel
        self.pub_t = 0.0
        self.pub_n = 0
        self.joy318_t = 0.0
        self.joy318_n = 0
        self.fb = None               # /enc_pot
        self.fb_t = 0.0
        self.estop = False           # /estop 1 — arduino 가 /cmd_vel 을 막는다 (화면에 알리기만)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_cmd = self.create_publisher(Int32MultiArray, CMD_TOPIC, 50)
        self.pub_318 = self.create_publisher(Int32, JOY318_TOPIC, 10)
        self.create_subscription(String, JOY_PORT_TOPIC, self.cb_port, latched)
        self.create_subscription(Float64MultiArray, ENC_POT_TOPIC, self.cb_enc, 20)
        self.create_subscription(Int32, ESTOP_TOPIC, lambda m: setattr(self, 'estop', int(m.data) == 1), latched)
        self.create_timer(TICK_S, self.tick)
        threading.Thread(target=self.serial_worker, daemon=True).start()
        self.get_logger().info(
            f"🎮 joy — 포트 {self.fixed_port or '/joy_port 대기'} | SWA ON 일 때만 {CMD_TOPIC} "
            f"[펄스 ±{boards.PULSE_MAX}, 조향 ±{boards.STEER_MAX}] (스틱 0 이면 '0 0' {ZERO_TAIL_S}s 뒤 조용) · "
            f"ON 동안 {JOY318_TOPIC}")

    # ── 포트 ─────────────────────────────────────────────────────────────────
    def cb_port(self, msg: String):
        if self.fixed_port:
            return
        dev = str(msg.data)
        if dev and dev != self.want_port:
            self.get_logger().info(f"조이스틱 포트 받음 {dev}")
        self.want_port = dev

    def serial_worker(self):
        while not self.stop_evt.is_set():
            dev = self.want_port
            if not dev:
                self.stop_evt.wait(0.2)
                continue
            try:
                ser = serial.Serial(dev, boards.BAUD, timeout=0.1, write_timeout=boards.WRITE_TIMEOUT_S,
                                    exclusive=True)
            except (serial.SerialException, OSError) as e:
                self.status = (f"조이스틱 포트 {dev} 열기 실패: {e}", "bad")
                self.stop_evt.wait(REOPEN_S)
                continue
            with self.lock:
                self.line_t = time.monotonic()          # 여는 순간부터 BOARD_SILENT_S 를 센다
                self.calib_buf = {"x1": [], "y1": [], "x2": [], "y2": []}   # 연결마다 영점을 새로 잡는다
                self.status = ("영점 보정 중... (보드 리셋 버튼으로 재보정)", "text")
            with self.ser_lock:
                self.ser, self.port = ser, dev
            self.get_logger().info(f"🎮 조이스틱 연결 {dev}")
            try:
                self._read_loop(ser, dev)
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f"조이스틱 단절 {dev}: {e}")
            except TypeError:
                if not self.stop_evt.is_set():
                    raise                       # 끝낼 때만 무시 — shutdown() 이 포트를 닫으면 읽던 read() 가 TypeError
            finally:
                with self.ser_lock:
                    self.ser, self.port = None, ''
                boards.close_quietly(ser)
                self._lost()
            self.stop_evt.wait(REOPEN_S)

    def _read_loop(self, ser, dev):
        buf = b""
        while not self.stop_evt.is_set() and self.want_port == dev:
            data = ser.read(ser.in_waiting or 1)
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                v = boards.parse_joy_line(raw.decode('ascii', errors='ignore').strip())
                if v is None:
                    continue
                with self.lock:
                    self.line_t = time.monotonic()
                    if v == 'RESET':
                        self._on_reset()
                    else:
                        self.data = v
                        self.joy_lines += 1
                        self._step_calibration(v)
                        self._update(time.monotonic())

    def _lost(self):
        with self.lock:
            self.status = ("조이스틱 끊김 — 다시 연결을 기다린다", "bad")
            self._set_on(False, time.monotonic())
            self._update(time.monotonic())

    # ── 영점 ─────────────────────────────────────────────────────────────────
    def _on_reset(self):
        """보드가 막 부팅했다(리셋 버튼 · 포트 열기) — 곧바로 영점을 다시 잡는다(0.3 s 중앙값)."""
        self.calib_buf = {"x1": [], "y1": [], "x2": [], "y2": []}
        self.status = ("리셋 감지 — 영점을 다시 잡는다", "text")
        self.get_logger().info("조이스틱 보드 리셋 감지(J,RESET) — 영점을 다시 잡는다")

    def _step_calibration(self, d):
        x1, y1, _, x2, y2 = d[:5]
        if self.calib_buf is None:
            return
        for k, v in (("x1", x1), ("y1", y1), ("x2", x2), ("y2", y2)):
            self.calib_buf[k].append(v)
        if len(self.calib_buf["y1"]) >= CALIBRATION_SAMPLES:
            self.center = {k: statistics.median(v) for k, v in self.calib_buf.items()}
            self.calib_buf = None
            c = self.center
            self.status = (f"보정 완료  L center=({c['x1']},{c['y1']})  R center=({c['x2']},{c['y2']})", "ok")

    def calibrating(self):
        return self.calib_buf is not None

    def current_status(self):
        """화면 · 콘솔용 상태 — 연결됐는데 보드 줄이 끊겼으면 그것을 먼저 알린다."""
        if self.ser is not None and time.monotonic() - self.line_t > BOARD_SILENT_S:
            return (f"보드에서 줄이 안 온다 ({time.monotonic() - self.line_t:.0f}s) — 다른 스케치? 배선? "
                    f"보드 리셋 버튼을 눌러 볼 것", "bad")
        return self.status

    # ── 발행 규칙 (self.lock 안에서 부른다) ─────────────────────────────────────
    def _set_on(self, on, now):
        if on == self.on:
            return
        self.on = on
        self.get_logger().info(f"SWA {'ON — 조이스틱 신호를 보낸다 · /joy318' if on else 'OFF — 스틱 무시'}")
        self._start_tail(now)                # 바뀔 때마다 "0 0" 마무리 (스틱이 0 이 아니면 _update 가 덮는다)
        if on:
            self._publish_318(now)

    def _start_tail(self, now):
        self.engaged = False
        self.tail_until = now + ZERO_TAIL_S
        self._publish((0, 0), now)

    def _update(self, now):
        """메가 한 줄마다 · 주기마다. 스틱 → 명령, 그리고 낼지 말지."""
        connected = self.ser is not None
        if connected:
            self._set_on(bool(self.data[8]), now)
        active = connected and self.on and not self.calibrating()
        cmd = stick_to_command(self.data[1], self.data[3], self.center["y1"], self.center["x2"]) if active else (0, 0)
        self.cmd = cmd
        if cmd != (0, 0):
            if not self.engaged or cmd != self.last_pub or now - self.pub_t >= RESEND_S:
                self._publish(cmd, now)
            self.engaged = True
            self.tail_until = None
            return
        if self.engaged:
            self._start_tail(now)            # 방금 0 으로 돌아왔다
            return
        if self.tail_until is not None:
            if now >= self.tail_until:
                self.tail_until = None       # 마무리 끝 — 조용해진다
            elif now - self.pub_t >= RESEND_S:
                self._publish((0, 0), now)

    def _publish(self, cmd, now):
        self.pub_cmd.publish(Int32MultiArray(data=[int(cmd[0]), int(cmd[1])]))
        self.last_pub, self.pub_t = cmd, now
        self.pub_n += 1

    def _publish_318(self, now):
        self.pub_318.publish(Int32(data=1))
        self.joy318_t = now
        self.joy318_n += 1

    def tick(self):
        now = time.monotonic()
        with self.lock:
            self._update(now)
            if self.on and self.ser is not None and now - self.joy318_t >= JOY318_S:
                self._publish_318(now)

    # ── 우노 출력 → 화면 ─────────────────────────────────────────────────────
    def cb_enc(self, msg: Float64MultiArray):
        d = list(msg.data)
        if len(d) < 4:
            return
        self.fb, self.fb_t = d[:4], time.monotonic()

    def fb_fresh(self):
        return self.fb is not None and time.monotonic() - self.fb_t <= FB_STALE_S

    def describe_cmd(self):
        """화면 · 콘솔용 한 줄 — 지금 /cmd_vel 을 어떻게 내고 있나."""
        if self.ser is None:
            return "발행 안 함 (조이스틱 미연결)"
        if not self.on:
            return "발행 안 함 — SWA OFF (스틱 무시)"
        if self.calibrating():
            return "영점 보정 중 — 0 으로 본다"
        if self.engaged:
            return f"[{self.cmd[0]}, {self.cmd[1]}]  (REF {signed2(self.cmd[0])} · 조향 {self.cmd[1]:+d}°)"
        if self.tail_until is not None:
            return "[0, 0] 마무리"
        return "발행 안 함 — 스틱 가운데"

    def shutdown(self):
        self.stop_evt.set()
        with self.lock:
            if self.engaged or self.tail_until is not None:
                self._publish((0, 0), time.monotonic())
        with self.ser_lock:
            if self.ser is not None:
                boards.close_quietly(self.ser)


# ══════════════════════════════════════════════════════════════════════════════
#  화면 (tool/joy.py 와 같은 모양)
# ══════════════════════════════════════════════════════════════════════════════
def axis_to_screen(n):
    return (1 - n) / 2 * PAD


class JoyWindow:
    def __init__(self, root, node, on_close):
        import tkinter as tk
        self.tk = tk
        self.root = root
        self.node = node
        self.on_close = on_close
        root.title("broon joy — 조이스틱 → /cmd_vel")
        root.configure(bg=BG)

        main = tk.Frame(root, bg=BG)
        main.pack(padx=16, pady=16)
        self.pad_l = self._make_pad(main, "L", 0, "h")
        self._make_switch_column(main)
        self.pad_r = self._make_pad(main, "R", 2, "v")

        info = tk.Frame(root, bg=BG)
        info.pack(fill=tk.X, padx=16)
        self.cmd_label = tk.Label(info, text="", bg=BG, fg=DOT_COLOR, font=("Consolas", 12, "bold"), anchor="w")
        self.cmd_label.pack(fill=tk.X)
        self.j318_label = tk.Label(info, text="", bg=BG, fg=TEXT, font=("Consolas", 11), anchor="w")
        self.j318_label.pack(fill=tk.X)
        self.fb_label = tk.Label(info, text="", bg=BG, fg=TEXT, font=("Consolas", 12), anchor="w")
        self.fb_label.pack(fill=tk.X)
        bottom = tk.Frame(root, bg=BG)
        bottom.pack(fill=tk.X, padx=16, pady=(4, 10))
        self.status = tk.Label(bottom, text="", bg=BG, fg=TEXT, font=("Consolas", 10))
        self.status.pack(side=tk.LEFT)
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._update()

    def _make_pad(self, parent, label, column, highlight):
        tk = self.tk
        frame = tk.Frame(parent, bg=BG)
        frame.grid(row=0, column=column, padx=12)
        tk.Label(frame, text=label, bg=BG, fg=TEXT, font=("Consolas", 16, "bold")).pack()
        canvas = tk.Canvas(frame, width=PAD, height=PAD, bg=PAD_BG, highlightthickness=1, highlightbackground=LINE)
        canvas.pack()
        if highlight == "h":
            canvas.create_line(PAD / 2, 0, PAD / 2, PAD, fill=LINE)
            hi_line = canvas.create_line(0, PAD / 2, PAD, PAD / 2, fill=DOT_COLOR)
        else:
            hi_line = canvas.create_line(PAD / 2, 0, PAD / 2, PAD, fill=DOT_COLOR)
            canvas.create_line(0, PAD / 2, PAD, PAD / 2, fill=LINE)
        k_id = canvas.create_oval(PAD / 2 - CENTER_R, PAD / 2 - CENTER_R, PAD / 2 + CENTER_R, PAD / 2 + CENTER_R,
                                  fill="", outline=LINE)
        dot_id = canvas.create_oval(0, 0, 0, 0, fill=DOT_COLOR, outline="")
        value_label = tk.Label(frame, text="X:---- Y:----", bg=BG, fg=TEXT, font=("Consolas", 10))
        value_label.pack()
        return {"canvas": canvas, "dot": dot_id, "k": k_id, "label": value_label, "hi_line": hi_line,
                "highlight": highlight}

    def _make_switch_column(self, parent):
        tk = self.tk
        frame = tk.Frame(parent, bg=BG)
        frame.grid(row=0, column=1, padx=12)
        self.on_label = tk.Label(frame, text="OFF", bg=BG, fg=PRESS_COLOR, font=("Consolas", 22, "bold"))
        self.on_label.pack(pady=(0, 10))
        row = tk.Frame(frame, bg=BG)
        row.pack()
        self.sw_ids = {}
        for name in ("SWA", "SWB"):
            box = tk.Frame(row, bg=BG)
            box.pack(side=tk.LEFT, padx=8)
            c = tk.Canvas(box, width=SW_R * 2 + 6, height=SW_R * 2 + 6, bg=BG, highlightthickness=0)
            c.pack()
            oid = c.create_oval(3, 3, SW_R * 2 + 3, SW_R * 2 + 3, fill=IDLE_COLOR, outline=LINE)
            tk.Label(box, text=name, bg=BG, fg=TEXT, font=("Consolas", 10)).pack()
            self.sw_ids[name] = (c, oid)

    def _update_pad(self, pad, x_raw, y_raw, k, cx, cy):
        sx = axis_to_screen(normalize(x_raw, cx))
        sy = axis_to_screen(normalize(y_raw, cy))
        c = pad["canvas"]
        c.coords(pad["dot"], sx - DOT_R, sy - DOT_R, sx + DOT_R, sy + DOT_R)
        if pad["highlight"] == "h":
            c.coords(pad["hi_line"], 0, sy, PAD, sy)
        else:
            c.coords(pad["hi_line"], sx, 0, sx, PAD)
        c.itemconfig(pad["k"], fill=PRESS_COLOR if k == 0 else "")
        pad["label"].config(text=f"X:{x_raw:4d} Y:{y_raw:4d}")

    def _update(self):
        n = self.node
        with n.lock:
            x1, y1, k1, x2, y2, k2, swa, swb, _ = n.data
            c = dict(n.center)
            on, cmd_text, status = n.on, n.describe_cmd(), n.current_status()
            j318 = n.on and n.ser is not None
            port = n.port
        self._update_pad(self.pad_l, x1, y1, k1, c["x1"], c["y1"])
        self._update_pad(self.pad_r, x2, y2, k2, c["x2"], c["y2"])
        for name, value in (("SWA", swa), ("SWB", swb)):
            cv, oid = self.sw_ids[name]
            cv.itemconfig(oid, fill=PRESS_COLOR if value == 0 else IDLE_COLOR)
        self.on_label.config(text="ON" if on else "OFF", fg=OK_COLOR if on else PRESS_COLOR)
        self.cmd_label.config(text=f"/cmd_vel : {cmd_text}")
        self.j318_label.config(text="/joy318 : 발행 중 (SWA ON — 자율주행 정지 · 출발 거절)" if j318
                               else "/joy318 : 안 냄", fg=WARN_COLOR if j318 else TEXT)
        if n.estop:
            self.cmd_label.config(text="🛑 E-STOP 중 — arduino 가 /cmd_vel 을 막는다 (웹 · 채팅방에서 GO 로 해제)",
                                  fg=PRESS_COLOR)
        else:
            self.cmd_label.config(fg=DOT_COLOR)
        if n.fb_fresh():
            r, a, m, ma = n.fb
            self.fb_label.config(text=f"/enc_pot : 우노가 받은 {signed2(r)} · {int(a):+d}°    실측 {signed2(m)} · {ma:+.1f}°")
        else:
            self.fb_label.config(text="/enc_pot : --- (arduino 노드 · 우노 확인)")
        text, color = status
        self.status.config(text=f"[{port or '포트 없음'}] {text}", fg=COLORS[color])
        if not n.stop_evt.is_set():
            self.root.after(round(1000 / GUI_HZ), self._update)


def run_nogui(node, stop):
    last_lines = 0
    while not stop.is_set():
        stop.wait(1.0)
        with node.lock:
            lines = node.joy_lines - last_lines
            last_lines = node.joy_lines
            text = (("🛑 E-STOP | " if node.estop else "")
                    + f"{'ON ' if node.on else 'OFF'} /cmd_vel {node.describe_cmd()} | 보드 {lines}줄/s | "
                    f"{node.current_status()[0]}")
        fb = node.fb if node.fb_fresh() else None
        text += f" | /enc_pot {fb[0]:+.0f} {fb[1]:+.0f}° 실측 {fb[2]:+.0f} {fb[3]:+.1f}°" if fb else " | /enc_pot ---"
        print(text, flush=True)


def main(args=None):
    argv = sys.argv[1:] if args is None else list(args)
    nogui = '--nogui' in argv
    argv = [a for a in argv if a != '--nogui']
    rclpy.init(args=argv)
    node = JoyNode()
    if not nogui and not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        node.get_logger().warning("화면(DISPLAY)이 없다 — --nogui 로 돈다")
        nogui = True
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    stop = threading.Event()

    def spin():
        try:
            executor.spin()
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        stop.set()

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()
    try:
        if nogui:
            run_nogui(node, stop)
        else:
            import tkinter as tk
            root = tk.Tk()

            def close():
                stop.set()
                root.destroy()

            JoyWindow(root, node, close)

            def watch():                     # Ctrl-C · launch 종료로 spin 이 끝나면 창도 닫는다
                if stop.is_set():
                    root.destroy()
                else:
                    root.after(200, watch)
            root.after(200, watch)
            signal.signal(signal.SIGINT, lambda *_: stop.set())
            root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.shutdown()
        time.sleep(0.05)                     # 마지막 "0 0" 이 나가게
        executor.shutdown(timeout_sec=0.5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
