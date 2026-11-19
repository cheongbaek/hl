"""joy.py ― 조이스틱 '메가'(tool/joy.ino) → 차량 '우노'(tool/arduino_code/dualtest_1118.ino) 중계 + 화면.

2026-11-18. mad PC 의 joy_gold.py (336줄, 화면 · 영점 보정 · 포트 찾기)를 가져와 고쳤다.
보드에 올려 함께 돌려 보지는 않았다 (PC 에서 가짜 포트로만 확인).

보드 찾기
  보드 종류와 상관없이, 아두이노 후보 포트를 모두 열어 "J," 로 시작하는 줄을 보내는 쪽 = 조이스틱(메가),
  나머지 아두이노 = 구동계(우노). 둘 다 직접 정하려면  python3 joy.py --joy /dev/ttyACM1 --drive /dev/ttyACM0
  포트를 열면 메가는 리셋된다 (부트로더 1 ~ 2 초) — 찾는 데 최대 3.5 초.

메가 → PC   "J,x1,y1,k1,x2,y2,k2,swa,swb,on" 을 4 ms(250 Hz)마다, 부팅 때 "J,RESET".  on = SWA 로 바꾼 ON/OFF.
PC → 우노   "<목표 펄스> <조향각>"  (dualtest_1118 입력 규격). 받은 메가 줄마다 다시 계산해 ★값이 바뀌면 곧바로★ 보낸다.
            값이 그대로여도 0.2 s 마다 한 번 다시 보낸다. 펄스 −92 ~ 92, 조향 −21 ~ 21 — 둘 다 정수만 (dualtest 는 소수점이 든 줄을 버린다).
            OFF · 영점 보정 중 · 메가 끊김 · 창 닫기 → "0 0" (주행은 우노의 하강 제한으로 부드럽게 서고 조향은 0° 를 잡는다).
우노 → PC   "<목표 펄스> <목표 조향각> <실제 펄스> <실제 조향각>" 50 ms 마다 (dualtest 출력).
PC → 메가   "U,<목표 펄스>,<조향각>" — 우노 출력을 받을 때마다 넘긴다 (메가 LCD 둘째 줄). 정수.
            ★메가 줄이 0.05 s 안에 왔을 때만 쓴다★ — 메가 2560 부트로더는 바이트가 계속 들어오면 스케치로 넘어가지 않는다
            (리셋 버튼을 누른 동안 쓰면 LCD 가 흰 블록으로 남고 "J," 줄이 끊긴다. 2026-11-19 broon/joy.py 실물에서 겪음).
            조향각은 기본으로 우노의 '목표' 조향각 — 실제 각을 띄우려면 LCD_ACTUAL_ANGLE = True.
            타이머 · ON/OFF 는 메가 안에서만 돈다 (시리얼로 오가지 않는다).

스틱 → 값 (joy_gold 와 같은 방식, 범위만 바꿈)
  L 스틱 세로 → 목표 펄스 : 위 + (전진) · 아래 − (후진), 최대 ±92.   R 스틱 가로 → 조향 : 왼쪽 − · 오른쪽 +, 최대 ±21°.
  영점에서 ADC 120 안은 0 (데드존), 그 밖은 1 ~ 최댓값에 직선으로 잇는다.
  영점 : 시작하면 처음 0.3 s (75 줄)의 중앙값.  메가 리셋 버튼 → "J,RESET" → 3 초 뒤 그때 값 한 벌 (joy_gold 와 같다).
"""

import argparse
import statistics
import sys
import threading
import time
import tkinter as tk

import serial
from serial.tools import list_ports

BAUD = 115200

# Uno/Mega 정품(2341)과 CH340 클론(1A86)이 흔히 쓰는 VID
KNOWN_VIDS = {0x2341, 0x1A86, 0x2A03}

JOY_PREFIX = "J,"      # 메가(joy.ino)가 보내는 모든 줄의 접두어 — 이 줄을 보내는 보드 = 조이스틱
DETECT_WINDOW_S = 3.5  # 포트 판별에 기다리는 최대 시간 (메가는 포트를 열면 리셋된다)

JOY_FIELDS = 10        # "J" + x1,y1,k1,x2,y2,k2,swa,swb,on
DRIVE_FIELDS = 4       # 우노 출력 "목표펄스 목표조향각 실제펄스 실제조향각"

CALIBRATION_SAMPLES = 75      # 첫 영점에 쓰는 메가 줄 수 (250 Hz → 0.3 s)
RESET_CALIB_DELAY = 3.0       # 메가 RESET 신호 수신 후, 몇 초 뒤 값을 영점으로 삼을지

PULSE_MAX = 92                # L(Y1) 최대 목표 펄스 (위 + · 아래 −), 우노 dualtest 의 제한 속도와 같다
STEER_MAX = 21                # R(X2) 최대 조향각 (좌 − · 우 +), 우노 dualtest 의 제한각과 같다
ADC_MAX = 1023

DEADZONE_RAW = 120            # 원점 기준 raw ADC 편차가 이 값 미달이면 0으로 처리

DEFAULT_CENTER = ADC_MAX // 2  # 보정 전 임시 중앙값

DRIVE_RESEND_S = 0.2          # 값이 그대로여도 이 간격으로 우노에 다시 보낸다
WRITE_TIMEOUT_S = 0.05        # 보드가 안 받아도 이 이상 막히지 않는다
READ_TIMEOUT_S = 0.1
FB_STALE_S = 0.5              # 우노 출력이 이만큼 없으면 화면에 '---'
LCD_ACTUAL_ANGLE = False      # 메가 LCD 에 띄울 조향각 : False = 우노의 목표각, True = 실제각

GUI_HZ = 60
UPDATE_MS = round(1000 / GUI_HZ)

PAD = 260          # XY 패드 한 변 (px)
DOT_R = 9          # 스틱 위치 점 반지름
CENTER_R = 12      # K1/K2 눌림 표시 원 반지름
SW_R = 16          # SWA/SWB 표시 원 반지름

BG = "#1e1e1e"
PAD_BG = "#2b2b2b"
LINE = "#4a4a4a"
DOT_COLOR = "#4fc3f7"          # 유효 입력(L의 Y, R의 X) 강조에도 동일하게 사용
PRESS_COLOR = "#ef5350"
IDLE_COLOR = "#3a3a3a"
CALIB_COLOR = "#66bb6a"
ON_COLOR = "#66bb6a"
TEXT = "#dddddd"
COLORS = {"text": TEXT, "ok": CALIB_COLOR, "bad": PRESS_COLOR}


# ---------- 포트 찾기 ----------

def _candidate_ports():
    for port in list_ports.comports():
        if port.vid in KNOWN_VIDS:
            yield port.device
            continue
        desc = (port.description or "").lower()
        if "arduino" in desc or "ch340" in desc:
            yield port.device


def _close(ser):
    try:
        ser.close()
    except (serial.SerialException, OSError):
        pass


def find_boards(window_s=DETECT_WINDOW_S):
    """후보 포트를 모두 열어 "J," 줄을 보내는 쪽 = 조이스틱, 나머지 = 구동계. (joy_ser, drive_ser) — 없으면 None."""
    sers = []
    for device in _candidate_ports():
        try:
            sers.append(serial.Serial(device, BAUD, timeout=0, write_timeout=WRITE_TIMEOUT_S))
        except (serial.SerialException, OSError):
            pass
    bufs = {id(s): b"" for s in sers}
    joy = None
    talked = set()                      # "J," 가 아닌 줄을 보낸 포트
    deadline = time.monotonic() + window_s
    while sers and time.monotonic() < deadline:
        for s in sers:
            try:
                data = s.read(s.in_waiting or 1)
            except (serial.SerialException, OSError):
                continue
            if not data:
                continue
            bufs[id(s)] += data
            while b"\n" in bufs[id(s)]:
                raw, bufs[id(s)] = bufs[id(s)].split(b"\n", 1)
                text = raw.decode("ascii", errors="ignore").strip()
                if not text:
                    continue
                if text.startswith(JOY_PREFIX):
                    if joy is None:
                        joy = s
                else:
                    talked.add(id(s))
        others = [s for s in sers if s is not joy]
        if joy is not None and all(id(s) in talked for s in others):
            break                       # 다 가렸다
        time.sleep(0.01)

    others = [s for s in sers if s is not joy]
    # 나머지 아두이노 = 구동계. 여럿이면 줄을 보낸 쪽을 먼저, 그다음 아무 줄도 없던 쪽
    others.sort(key=lambda s: 0 if id(s) in talked else 1)
    drive = others[0] if others else None
    for s in others[1:]:
        _close(s)
    for s in (joy, drive):
        if s is not None:
            s.timeout = READ_TIMEOUT_S
    return joy, drive


def open_port(device):
    return serial.Serial(device, BAUD, timeout=READ_TIMEOUT_S, write_timeout=WRITE_TIMEOUT_S)


# ---------- 스틱 값 → 명령 ----------

def normalize(raw, center):
    # center 기준으로 좌/우(또는 상/하) 구간을 각각 독립 비율로 -1..1에 매핑
    # (원점이 정확히 512가 아니므로 양쪽 구간 길이가 다름)
    center = max(1, min(ADC_MAX - 1, center))
    if raw >= center:
        return (raw - center) / (ADC_MAX - center)
    return (raw - center) / center


def axis_to_screen(n):
    # n=1(1023쪽) -> 0, n=0(중앙) -> PAD/2, n=-1(0쪽) -> PAD
    return (1 - n) / 2 * PAD


def scaled_with_deadzone(raw, center, deadzone_raw, max_out):
    # center 기준 raw 편차가 deadzone_raw 미달이면 0.
    # 그 외에는 (deadzone_raw..편측 raw 끝) 구간을 (1..max_out)으로 재매핑
    # (raw 기준 공통 데드존이므로 L/R 어느 쪽에도 동일하게 적용 가능, 부호는 diff 방향을 따름)
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
    """L 세로 → 목표 펄스 (정수, 위 +), R 가로 → 조향각 (정수 도, 왼쪽 −). 우노는 정수만 받는다."""
    ref = int(round(scaled_with_deadzone(y1, center_y1, DEADZONE_RAW, PULSE_MAX)))
    ang = int(round(-scaled_with_deadzone(x2, center_x2, DEADZONE_RAW, STEER_MAX)))
    return ref, ang


def signed2(v):
    """+35 · -07 처럼 부호 + 두 자리 (메가 LCD 와 같은 모양)."""
    v = int(round(v))
    return f"{'-' if v < 0 else '+'}{min(abs(v), 99):02d}"


# ---------- 중계 (화면과 따로 돈다) ----------

class Relay:
    def __init__(self, joy_ser, drive_ser):
        self.joy = joy_ser
        self.drive = drive_ser
        self.running = True
        self.joy_ok = True
        self.drive_ok = drive_ser is not None
        self.drive_lock = threading.Lock()
        self.joy_lock = threading.Lock()

        # x1, y1, k1, x2, y2, k2, swa, swb (버튼은 active-low), on
        self.data = (DEFAULT_CENTER, DEFAULT_CENTER, 1,
                     DEFAULT_CENTER, DEFAULT_CENTER, 1, 1, 1, 0)
        self.joy_lines = 0
        self.joy_line_t = 0.0        # 메가에서 마지막으로 올바른 줄을 받은 때 (부트로더 보호)
        self.center_x1 = DEFAULT_CENTER
        self.center_y1 = DEFAULT_CENTER
        self.center_x2 = DEFAULT_CENTER
        self.center_y2 = DEFAULT_CENTER
        self.calib_buf = {"x1": [], "y1": [], "x2": [], "y2": []}   # 시작 시 자동으로 원점 보정 시작
        self.reset_calib_deadline = None   # "J,RESET" 수신 후 보정을 적용할 monotonic 시각
        self.status = ("원점 보정 중... (메가 리셋 버튼으로 재보정)", "text")

        self.cmd = (0, 0)          # 지금 우노에 보내야 할 값
        self.sent = None             # 마지막으로 보낸 값
        self.sent_t = 0.0
        self.sent_count = 0
        self.fb = None               # 우노 출력 (목표 펄스, 목표각, 실제 펄스, 실제각)
        self.fb_t = 0.0

        self.threads = [threading.Thread(target=self._joy_loop, daemon=True)]
        if drive_ser is not None:
            self.threads.append(threading.Thread(target=self._drive_loop, daemon=True))
        for t in self.threads:
            t.start()

    # ---------- 메가 수신 → 영점 · 명령 → 우노 ----------

    def _joy_loop(self):
        while self.running:
            try:
                raw = self.joy.readline()
            except (serial.SerialException, OSError):
                self._joy_lost()
                return
            if not raw:
                continue
            line = raw.decode("ascii", errors="ignore").strip()
            if not line:
                continue

            if line.startswith("J,"):
                self.joy_line_t = time.monotonic()
            if line == "J,RESET":
                if self.calib_buf is not None:
                    # 아직 첫 보정 중 (포트를 열어 메가가 리셋된 경우) -> 리셋 뒤 값으로 처음부터 다시
                    self.calib_buf = {"x1": [], "y1": [], "x2": [], "y2": []}
                else:
                    # 아두이노가 방금 (재)부팅함 -> 스틱이 제자리로 돌아올 시간을 준 뒤 그 값으로 영점 보정
                    self.reset_calib_deadline = time.monotonic() + RESET_CALIB_DELAY
                    self.status = ("리셋 감지 - 원점 재보정 대기 중... 조이스틱을 가만히 두세요.", "text")
                continue

            parts = line.split(",")
            # 접두어 "J," 확인 후 뒤 9필드만 사용
            if len(parts) != JOY_FIELDS or parts[0] != "J":
                continue
            try:
                data = tuple(map(int, parts[1:]))
            except ValueError:
                continue
            self.data = data
            self.joy_lines += 1
            self._step_calibration(data)
            self._update_command(data)

    def _step_calibration(self, data):
        x1, y1, _, x2, y2 = data[:5]
        if self.reset_calib_deadline is not None:
            if time.monotonic() >= self.reset_calib_deadline:
                # 리셋 직후 값 한 벌을 그대로 원점으로 삼는다 (첫 부팅 보정과 달리 즉시 확정)
                self.center_x1, self.center_y1 = x1, y1
                self.center_x2, self.center_y2 = x2, y2
                self.reset_calib_deadline = None
                self.calib_buf = None
                self.status = (f"리셋 보정 완료  L center=({x1},{y1})  R center=({x2},{y2})", "ok")
            return
        if self.calib_buf is None:
            return
        for key, v in (("x1", x1), ("y1", y1), ("x2", x2), ("y2", y2)):
            self.calib_buf[key].append(v)
        if len(self.calib_buf["y1"]) >= CALIBRATION_SAMPLES:
            self.center_x1 = statistics.median(self.calib_buf["x1"])
            self.center_y1 = statistics.median(self.calib_buf["y1"])
            self.center_x2 = statistics.median(self.calib_buf["x2"])
            self.center_y2 = statistics.median(self.calib_buf["y2"])
            self.calib_buf = None
            self.status = (f"보정 완료  L center=({self.center_x1},{self.center_y1})  "
                           f"R center=({self.center_x2},{self.center_y2})", "ok")

    def calibrating(self):
        return self.calib_buf is not None or self.reset_calib_deadline is not None

    def _update_command(self, data):
        on = data[8]
        if self.calibrating() or not on:
            # 보정 중/대기 중에는 옛 원점 기준값이 튀는 것을 막기 위해, OFF 면 늘 "0 0"
            cmd = (0, 0)
        else:
            cmd = stick_to_command(data[1], data[3], self.center_y1, self.center_x2)
        self.cmd = cmd
        self._send_drive(cmd)

    def _send_drive(self, cmd, force=False):
        if self.drive is None or not self.drive_ok:
            return
        now = time.monotonic()
        if not force and cmd == self.sent and now - self.sent_t < DRIVE_RESEND_S:
            return
        ref, ang = cmd
        with self.drive_lock:
            try:
                self.drive.write(f"{ref} {ang}\n".encode("ascii"))
            except serial.SerialTimeoutException:
                return                      # 이번 것은 건너뛴다 — 다음 줄에서 다시 보낸다
            except (serial.SerialException, OSError):
                self._drive_lost()
                return
        self.sent = cmd
        self.sent_t = now
        self.sent_count += 1

    def _joy_lost(self):
        self.joy_ok = False
        self.status = ("메가(조이스틱) 끊김 - 우노에 0 0 을 보냈다", "bad")
        self.cmd = (0, 0)
        self._send_drive((0, 0), force=True)

    # ---------- 우노 수신 → 메가 LCD ----------

    def _drive_loop(self):
        while self.running and self.drive_ok:
            try:
                raw = self.drive.readline()
            except (serial.SerialException, OSError):
                self._drive_lost()
                return
            if not raw:
                continue
            parts = raw.decode("ascii", errors="ignore").split()
            if len(parts) != DRIVE_FIELDS:
                continue
            try:
                fb = (int(parts[0]), int(parts[1]), int(parts[2]), float(parts[3]))
            except ValueError:
                continue
            self.fb = fb
            self.fb_t = time.monotonic()
            ang = fb[3] if LCD_ACTUAL_ANGLE else fb[1]
            self._send_joy(f"U,{fb[0]},{int(round(ang))}\n")

    def _send_joy(self, text):
        if not self.joy_ok or time.monotonic() - self.joy_line_t > 0.05:
            return                          # 메가가 부트로더일 수 있다 — 쓰면 스케치로 못 넘어간다
        with self.joy_lock:
            try:
                self.joy.write(text.encode("ascii"))
            except serial.SerialTimeoutException:
                pass
            except (serial.SerialException, OSError):
                pass                        # 읽기 쪽이 곧 끊김을 알아챈다

    def _drive_lost(self):
        self.drive_ok = False

    def fb_fresh(self):
        return self.fb is not None and time.monotonic() - self.fb_t <= FB_STALE_S

    # ---------- 끝내기 ----------

    def close(self):
        self._send_drive((0, 0), force=True)   # 부드럽게 세운다
        self.running = False
        time.sleep(0.05)
        for s in (self.joy, self.drive):
            if s is not None:
                _close(s)


# ---------- 화면 ----------

class JoyWindow:
    def __init__(self, root, relay):
        self.root = root
        self.relay = relay

        drive_port = relay.drive.port if relay.drive is not None else "없음"
        root.title(f"Joystick → Drive  (조이스틱 {relay.joy.port} · 구동 {drive_port})")
        root.configure(bg=BG)

        main = tk.Frame(root, bg=BG)
        main.pack(padx=16, pady=16)

        self.pad_l = self._make_pad(main, "L", column=0, highlight="h")
        self._make_switch_column(main)
        self.pad_r = self._make_pad(main, "R", column=2, highlight="v")

        info = tk.Frame(root, bg=BG)
        info.pack(fill=tk.X, padx=16)
        self.cmd_label = tk.Label(info, text="", bg=BG, fg=DOT_COLOR, font=("Consolas", 12, "bold"), anchor="w")
        self.cmd_label.pack(fill=tk.X)
        self.fb_label = tk.Label(info, text="", bg=BG, fg=TEXT, font=("Consolas", 12), anchor="w")
        self.fb_label.pack(fill=tk.X)
        self.lcd_label = tk.Label(info, text="", bg=BG, fg=TEXT, font=("Consolas", 11), anchor="w")
        self.lcd_label.pack(fill=tk.X)

        bottom = tk.Frame(root, bg=BG)
        bottom.pack(fill=tk.X, padx=16, pady=(4, 10))
        self.status = tk.Label(bottom, text="", bg=BG, fg=TEXT, font=("Consolas", 10))
        self.status.pack(side=tk.LEFT)

        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._update_gui()

    # ---------- UI 구성 ----------

    def _make_pad(self, parent, label, column, highlight):
        frame = tk.Frame(parent, bg=BG)
        frame.grid(row=0, column=column, padx=12)

        tk.Label(frame, text=label, bg=BG, fg=TEXT,
                 font=("Consolas", 16, "bold")).pack()

        canvas = tk.Canvas(frame, width=PAD, height=PAD,
                           bg=PAD_BG, highlightthickness=1,
                           highlightbackground=LINE)
        canvas.pack()

        # 십자선 : 유효 입력 쪽(L=Y축 값을 보여주는 X축평행선, R=X축 값을 보여주는
        # Y축평행선)은 하늘색으로 강조하고 실제 값을 따라 움직임, 나머지 한쪽은
        # 원래대로 회색으로 중앙에 고정
        if highlight == "h":
            canvas.create_line(PAD / 2, 0, PAD / 2, PAD, fill=LINE)             # 세로선(X1, 미사용) - 고정
            hi_line = canvas.create_line(0, PAD / 2, PAD, PAD / 2, fill=DOT_COLOR)  # 가로선(Y1) - 동적
        else:
            hi_line = canvas.create_line(PAD / 2, 0, PAD / 2, PAD, fill=DOT_COLOR)  # 세로선(X2) - 동적
            canvas.create_line(0, PAD / 2, PAD, PAD / 2, fill=LINE)             # 가로선(Y2, 미사용) - 고정

        # K 버튼 눌림 표시 (정가운데)
        k_id = canvas.create_oval(PAD / 2 - CENTER_R, PAD / 2 - CENTER_R,
                                  PAD / 2 + CENTER_R, PAD / 2 + CENTER_R,
                                  fill="", outline=LINE)
        # 스틱 위치 점
        dot_id = canvas.create_oval(0, 0, 0, 0, fill=DOT_COLOR, outline="")

        value_label = tk.Label(frame, text="X:---- Y:----",
                               bg=BG, fg=TEXT, font=("Consolas", 10))
        value_label.pack()

        return {"canvas": canvas, "dot": dot_id, "k": k_id, "label": value_label,
                "hi_line": hi_line, "highlight": highlight}

    def _make_switch_column(self, parent):
        frame = tk.Frame(parent, bg=BG)
        frame.grid(row=0, column=1, padx=12)

        self.on_label = tk.Label(frame, text="OFF", bg=BG, fg=PRESS_COLOR,
                                 font=("Consolas", 22, "bold"))
        self.on_label.pack(pady=(0, 10))

        row = tk.Frame(frame, bg=BG)
        row.pack()

        self.sw_ids = {}
        for name in ("SWA", "SWB"):
            box = tk.Frame(row, bg=BG)
            box.pack(side=tk.LEFT, padx=8)
            c = tk.Canvas(box, width=SW_R * 2 + 6, height=SW_R * 2 + 6,
                          bg=BG, highlightthickness=0)
            c.pack()
            oid = c.create_oval(3, 3, SW_R * 2 + 3, SW_R * 2 + 3,
                                fill=IDLE_COLOR, outline=LINE)
            tk.Label(box, text=name, bg=BG, fg=TEXT,
                     font=("Consolas", 10)).pack()
            self.sw_ids[name] = (c, oid)

    # ---------- 화면 갱신 ----------

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

    def _update_gui(self):
        r = self.relay
        x1, y1, k1, x2, y2, k2, swa, swb, on = r.data

        self._update_pad(self.pad_l, x1, y1, k1, r.center_x1, r.center_y1)
        self._update_pad(self.pad_r, x2, y2, k2, r.center_x2, r.center_y2)
        for name, value in (("SWA", swa), ("SWB", swb)):
            c, oid = self.sw_ids[name]
            c.itemconfig(oid, fill=PRESS_COLOR if value == 0 else IDLE_COLOR)
        self.on_label.config(text="ON" if on else "OFF", fg=ON_COLOR if on else PRESS_COLOR)

        ref, ang = r.cmd
        if r.drive is None:
            self.cmd_label.config(text=f'우노로 : (구동 보드 없음)   계산값 "{ref} {ang}"')
        elif not r.drive_ok:
            self.cmd_label.config(text="우노로 : 연결 끊김")
        else:
            self.cmd_label.config(text=f'우노로 : "{ref} {ang}"   (REF {signed2(ref)} · 조향 {ang:+d}°)')

        if r.fb_fresh():
            t_ref, t_ang, m_ref, m_ang = r.fb
            self.fb_label.config(text=f"우노 출력 : 목표 REF {signed2(t_ref)} · 조향 {t_ang:+d}°    "
                                      f"실제 {signed2(m_ref)} · {m_ang:+.1f}°")
            lcd_ang = m_ang if LCD_ACTUAL_ANGLE else t_ang
            self.lcd_label.config(text=f"메가 LCD 둘째 줄 : REF:{signed2(t_ref)} {signed2(lcd_ang)} 00km")
        else:
            self.fb_label.config(text="우노 출력 : ---")
            self.lcd_label.config(text="메가 LCD 둘째 줄 : REF:--- --- 00km")

        text, color = r.status
        self.status.config(text=text, fg=COLORS[color])

        if r.joy_ok:
            self.root.after(UPDATE_MS, self._update_gui)

    def _on_close(self):
        self.relay.close()
        self.root.destroy()


def run_nogui(relay):
    """화면 없이 돌린다 (SSH 등). 1 초마다 한 줄. Ctrl+C 로 끝 — 우노에 0 0 을 보내고 닫는다."""
    try:
        while relay.joy_ok:
            time.sleep(1.0)
            ref, ang = relay.cmd
            fb = relay.fb if relay.fb_fresh() else None
            on = "ON " if relay.data[8] else "OFF"
            fb_txt = (f"우노 목표 {signed2(fb[0])} {fb[1]:+d}  실제 {signed2(fb[2])} {fb[3]:+.1f}"
                      if fb else "우노 출력 ---")
            print(f"{on}  보냄 \"{ref} {ang}\"  {fb_txt}  메가 {relay.joy_lines}줄  ({relay.status[0]})", flush=True)
            relay.joy_lines = 0
    except KeyboardInterrupt:
        pass
    relay.close()


def main():
    ap = argparse.ArgumentParser(description="조이스틱(메가) → 구동(우노) 중계")
    ap.add_argument("--joy", help="조이스틱(메가) 포트를 직접 정한다 (예: /dev/ttyACM1, COM5)")
    ap.add_argument("--drive", help="구동(우노) 포트를 직접 정한다 (예: /dev/ttyACM0, COM4)")
    ap.add_argument("--nogui", action="store_true", help="창 없이 돌린다 (1 초마다 상태 한 줄)")
    args = ap.parse_args()

    if args.joy or args.drive:
        if not args.joy:
            print("--drive 를 쓰면 --joy 도 정해야 합니다.")
            sys.exit(1)
        joy = open_port(args.joy)
        drive = open_port(args.drive) if args.drive else None
    else:
        joy, drive = find_boards()
    if joy is None:
        if drive is not None:
            _close(drive)
        print('"J,"를 보내는 조이스틱 보드를 찾지 못했습니다.')
        sys.exit(1)
    print(f"조이스틱: {joy.port}   구동: {drive.port if drive is not None else '없음'}")

    relay = Relay(joy, drive)
    if args.nogui:
        run_nogui(relay)
        return
    root = tk.Tk()
    JoyWindow(root, relay)
    root.mainloop()


if __name__ == "__main__":
    main()
