#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
boards.py ― 아두이노 두 대(구동 '우노' · 조이스틱 보드 메가 또는 나노)의 규약과 판별 — arduino · joy · check 가 같이 쓴다
════════════════════════════════════════════════════════════════════════════════
ROS 를 import 하지 않는다 (check.py 처럼 노드가 아닌 도구도 쓴다).

판별 (tool/joy.py 와 같은 방법, 2026-11-18 사용자 지정)
  보드 종류(USB VID · PID)와 상관없이 아두이노 후보 포트를 모두 열어 줄을 읽는다.
  ★"J," 로 시작하는 줄을 보내는 쪽 = 조이스틱★, ★나머지 아두이노 = 구동계★.
  나머지가 여럿이면 무슨 줄이든 보낸 쪽을 먼저 고른다. 메가 · 나노는 포트를 열면 리셋된다(부트로더 1 ~ 2 초) —
  그래서 최대 DETECT_WINDOW_S 동안 기다린다. GPS(u-blox VID 1546)는 후보가 아니다.
  FTDI(VID 0403, FT232R — 정품 구형 나노)는 ★조이스틱으로만★ 잡는다 — "J," 줄을 보내면 조이스틱, 아니면 버린다.
  FTDI 칩은 아두이노가 아닌 장치도 많이 써서 구동 후보로 삼으면 엉뚱한 장치에 명령을 쓸 수 있다 (2026-11-19).

구동 '우노' (tool/arduino_code/broon_1119.ino) — 115200 bps (R4 Minima 는 USB 라 실제로는 무관)
  PC → 우노  "<목표 펄스> <목표 조향각>\\n"   둘 다 정수. 펄스 −92 ~ 92 (부호 = 방향), 조향 −21 ~ 21 (− 좌 · + 우).
             소수점이 든 줄은 우노가 통째로 버린다. 올바른 줄이 2 초 동안 없으면 우노가 구동 PWM 을 0 으로 한다.
  우노 → PC  50 ms 마다 "<받은 목표 펄스> <받은 목표 조향각> <실측 펄스> <실측 조향각>"
             앞 셋은 정수, 실측 조향각은 소수 둘째 자리. 실측 펄스 = 최근 100 ms 엔코더 A+B 에지 수, 부호 = 지금 DIR.
조이스틱 보드 — 메가(tool/joy_nolcd.ino) 또는 나노(tool/joy_nano.ino). 핀만 다르고 줄은 똑같다.
  보드 → PC  4 ms(250 Hz)마다 "J,x1,y1,k1,x2,y2,k2,swa,swb,on", 부팅 때 "J,RESET".  on = SWA 로 바꾼 ON(1)/OFF(0).
  PC → 보드  보내지 않는다 (2026-11-19 부터).
"""

import time

import serial
from serial.tools import list_ports

BAUD = 115200
KNOWN_VIDS = {0x2341, 0x1A86, 0x2A03}       # Arduino SA · CH340 클론 · arduino.org
JOY_ONLY_VIDS = {0x0403}                    # FTDI FT232R (정품 구형 나노) — 조이스틱으로만, 구동 후보는 아니다
R4_MINIMA = (0x2341, 0x0069)
DETECT_WINDOW_S = 3.5                       # 판별에 기다리는 최대 시간 (메가는 열면 리셋된다)
WRITE_TIMEOUT_S = 0.05                      # 보드가 안 받아도 이 이상 막히지 않는다

# ── 구동 규약 ──
PULSE_MAX = 92                              # |목표 펄스| 상한 (broon_1119 의 REF_ABS_MAX)
STEER_MAX = 21                              # |목표 조향각| 상한 [도] (broon_1119 의 ANG_ABS_MAX)
DRIVE_FIELDS = 4

# ── 조이스틱 규약 ──
JOY_PREFIX = "J,"
JOY_RESET = "J,RESET"
JOY_FIELDS = 10                             # "J" + x1,y1,k1,x2,y2,k2,swa,swb,on

# ── ROS 토픽 (broon 노드끼리 · white1 이 쓴다) ──
CMD_TOPIC = '/cmd_vel'          # Int32MultiArray [목표 펄스, 목표 조향각] — joy · driving 이 내고 arduino 가 받는다
ENC_POT_TOPIC = '/enc_pot'      # Float64MultiArray [받은 펄스, 받은 조향각, 실측 펄스, 실측 조향각] — arduino 가 낸다
JOY318_TOPIC = '/joy318'        # Int32 (늘 1) — 조이스틱 SWA ON 동안 joy 가 낸다. driving 은 들으면 자율주행을 멈춘다
JOY_PORT_TOPIC = '/joy_port'    # String (래치) — arduino 가 판별한 조이스틱 포트. joy 가 이것을 연다
ESTOP_TOPIC = '/estop'          # Int32 (래치) 1 = E-STOP 발동 · 0 = 해제 — estop 이 낸다(domichat 방 domi_fishing_broon)
ENC_POT_COLUMNS = ('ref_pulse', 'ref_steer_deg', 'meas_pulse', 'steer_measured_deg')


def valid_cmd(data):
    """/cmd_vel 값 검사 — [펄스, 조향] 두 정수이고 범위 안이면 (펄스, 조향), 아니면 None."""
    if len(data) != 2:
        return None
    p, s = int(data[0]), int(data[1])
    if -PULSE_MAX <= p <= PULSE_MAX and -STEER_MAX <= s <= STEER_MAX:
        return p, s
    return None


def parse_drive_line(text):
    """우노 한 줄 → (받은 펄스, 받은 조향, 실측 펄스, 실측 조향) 또는 None."""
    parts = text.split()
    if len(parts) != DRIVE_FIELDS:
        return None
    try:
        return int(parts[0]), int(parts[1]), int(parts[2]), float(parts[3])
    except ValueError:
        return None


def parse_joy_line(text):
    """메가 한 줄 → 'RESET' · 9칸 정수 튜플(x1,y1,k1,x2,y2,k2,swa,swb,on) · None."""
    if text == JOY_RESET:
        return 'RESET'
    parts = text.split(",")
    if len(parts) != JOY_FIELDS or parts[0] != "J":
        return None
    try:
        return tuple(int(v) for v in parts[1:])
    except ValueError:
        return None


def candidate_ports(exclude=()):
    """아두이노 후보 포트 [(경로, vid, pid, 설명)] — VID 또는 설명으로 고른다."""
    out = []
    for p in list_ports.comports():
        if p.device in exclude:
            continue
        desc = (p.description or "").lower()
        if p.vid in KNOWN_VIDS or p.vid in JOY_ONLY_VIDS or "arduino" in desc or "ch340" in desc:
            out.append((p.device, p.vid, p.pid, p.product or p.description or ""))
    return sorted(out)


def close_quietly(ser):
    try:
        ser.close()
    except (serial.SerialException, OSError):
        pass


def probe(devices, window_s=DETECT_WINDOW_S, keep_drive=True, stop=None):
    """포트들을 한꺼번에 열어 줄을 읽고 가른다.

    돌려주는 것 dict:
      joy        조이스틱으로 판별된 포트 경로 ('' = 없음) — ★닫고★ 돌려준다 (joy 노드가 다시 연다)
      drive      구동계 포트 경로 ('' = 없음)
      drive_ser  keep_drive 면 열린 채로 둔 구동 포트 (timeout 0.1), 아니면 None
      lines      {경로: 받은 줄 몇 개} — check 가 보여 준다
      busy       {경로: 열기 실패 이유}
    "J," 를 보낸 포트가 여럿이면 처음 것만 조이스틱이고 나머지는 버린다(닫는다).
    JOY_ONLY_VIDS(FTDI) 포트는 조이스틱이 아니면 구동 · extra 에도 넣지 않는다.
    stop 이 주어지면(threading.Event) 켜지는 즉시 그만둔다.
    모든 포트가 줄을 보내 가려지면 window_s 전이라도 끝낸다 (2026-11-23 — 재연결 때 조이스틱을 이미 알면 기다릴 것이 없다).
    """
    joy_only = {p.device for p in list_ports.comports() if p.vid in JOY_ONLY_VIDS}
    sers, busy = {}, {}
    for dev in devices:
        try:
            sers[dev] = serial.Serial(dev, BAUD, timeout=0, write_timeout=WRITE_TIMEOUT_S, exclusive=True)
        except (serial.SerialException, OSError) as e:
            busy[dev] = str(e)
    bufs = {d: b"" for d in sers}
    lines = {d: [] for d in sers}
    joy = ''
    talked = set()                       # "J," 가 아닌 줄을 보낸 포트
    jsent = set()                        # "J," 줄을 보낸 포트
    deadline = time.monotonic() + window_s
    while sers and time.monotonic() < deadline:
        if stop is not None and stop.is_set():
            break
        for dev, s in sers.items():
            try:
                data = s.read(s.in_waiting or 1)
            except (serial.SerialException, OSError):
                continue
            if not data:
                continue
            bufs[dev] += data
            while b"\n" in bufs[dev]:
                raw, bufs[dev] = bufs[dev].split(b"\n", 1)
                text = raw.decode("ascii", errors="ignore").strip()
                if not text:
                    continue
                if len(lines[dev]) < 5:
                    lines[dev].append(text)
                if text.startswith(JOY_PREFIX):
                    jsent.add(dev)
                    if not joy:
                        joy = dev
                elif dev != joy:
                    talked.add(dev)
        if all(d in talked or d in jsent for d in sers):
            break                        # 다 가렸다 — 조이스틱이 없어도(이미 알거나 안 꽂았으면) 창을 다 기다리지 않는다
        time.sleep(0.01)

    others = sorted((d for d in sers if d != joy and d not in joy_only
                     and not any(t.startswith(JOY_PREFIX) for t in lines[d])),
                    key=lambda d: (0 if d in talked else 1, d))
    drive = others[0] if others else ''
    drive_ser = None
    for dev, s in sers.items():
        if keep_drive and dev == drive:
            s.timeout = 0.1
            drive_ser = s
        else:
            close_quietly(s)
    return {'joy': joy, 'drive': drive, 'drive_ser': drive_ser, 'lines': lines, 'busy': busy,
            'extra': [d for d in others[1:]]}
