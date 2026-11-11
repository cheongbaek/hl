#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check.py ― 장치 연결 점검 (GPS 2대 · IMU · 아두이노)
════════════════════════════════════════════════════════════════════════════════
    ros2 run white check              GPS · IMU 는 열어서 확인, 아두이노는 USB 목록으로만
    ros2 run white check --arduino    아두이노도 열어 E 줄을 확인한다 (★열면 리셋된다★ — 바퀴를 띄우고)

★one_launch 를 끄고★ 돌린다 — 노드가 포트를 쥐고 있으면(exclusive) 열 수 없어 '사용 중' 으로 나온다.
ROS 노드가 아니다. 포트는 ports.py 와 같은 규칙(USB VID/PID)으로만 고르고 다른 포트는 열지 않는다.

  GPS     u-blox VID 1546 포트마다 칩 고유 ID(UBX-SEC-UNIQID)와 펌웨어(MON-VER)를 ★조회만★ 하고
          (설정은 바꾸지 않는다) 2 초 동안 들어오는 NAV-PVT · GGA 로 측위 상태를 본다.
          ID 는 ports.GPS_IDS 로 1번 · 2번을 가른다. 두 대가 USB 상으로 똑같아 이것 말고는 구분할 수 없다.
          white gps 노드는 한 대만 쓴다 — 둘 다 꽂혀 있으면 번호가 작은 포트를 연다(어느 쪽인지 알려 준다).
  IMU     CP210x(10c4:ea60). 열어서 1 초 들어 보고, 데이터가 없으면 gps 노드가 여는 때와 같은 명령
          (so=1 · sp=50 · sd=0x4C)을 보내 다시 1.5 초 듣는다. 9필드 줄(가속도 g · 각속도 · 오일러)을 센다.
  아두이노 VID 2341 · 2a03 · 1a86. 기본은 USB 목록에 있는지만 본다 — Mega 는 포트를 여는 순간 리셋된다.
          --arduino 면 열고(리셋) 4 초 동안 들어 부팅 줄 · E 줄 형식을 본다(115200 bps). E 줄이 안 오면 57600 으로
          한 번 더 들어 baud 가 어긋났는지 알려 준다. 형식 : white_fw(E 뒤 7필드, hl_ws/tool/arduino_code.ino) ·
          v6 계열(E 뒤 1~2필드) · Vr.1(5필드, 57600).

끝 상태 : 1번 · 2번 GPS 중 하나 이상 + IMU + 아두이노가 다 확인되면 0, 아니면 1.
"""

import argparse
import errno
import math
import sys
import time

import serial
import serial.tools.list_ports

from white import ports
from white.gps import F9PStream, Q_LABEL, pvt_quality, ubx

GPS_BAUD = 38400             # USB CDC-ACM 이라 실제 속도와 무관 (gps.py 와 같다)
IMU_BAUD = 115200            # gps.py 와 같다
IMU_SETUP = ("so=1", "sp=50", "sd=0x4C")   # gps.py 가 열 때 보내는 것과 같다 (20 Hz, 가속도|각속도|오일러)
ARDUINO_BAUD = 115200        # arduino.py BAUD — white_fw(arduino_code.ino) · 지금 차의 v6 계열
ARDUINO_ALT_BAUD = 57600     # E 줄이 안 오면 이 속도로 한 번 더 들어 본다 (Vr.1)
ARDUINO_BOOT_S = 2.0         # 부트로더가 지나갈 때까지 (그동안 온 줄도 모은다 — 부팅 줄 H)
ID_TIMEOUT_S = 4.0           # 공장 설정(NMEA 폭주)에서도 칩 ID 가 읽히게 (gps_mapper_dual.py 와 같다)
GPS_LISTEN_S = 2.0
IMU_LISTEN_S = (1.0, 1.5)
ARDUINO_LISTEN_S = 2.0

OK, WARN, BAD = "✅", "⚠️ ", "❌"


def say(mark, text, indent=3):
    print(" " * indent + f"{mark} {text}")


def usb_ports(match):
    """(장치, VID, PID, 설명) — /dev/tty* 중 match(vid, pid) 가 참인 것, 번호순."""
    out = []
    for p in serial.tools.list_ports.comports():
        if p.vid is None or not p.device.startswith("/dev/tty"):
            continue
        if match(p.vid, p.pid):
            out.append((p.device, p.vid, p.pid, p.product or p.description or ""))
    out.sort(key=lambda t: ports._sort_key(t[0]))
    return out


def open_port(dev, baud, timeout):
    """→ (Serial, None) 또는 (None, 까닭)."""
    try:
        return serial.Serial(dev, baud, timeout=timeout, exclusive=True), None
    except (serial.SerialException, OSError) as e:
        err = getattr(e, "errno", None)
        text = str(e)
        if err == errno.EACCES or "Permission denied" in text:
            return None, "권한 없음 — dialout 그룹인가? (sudo usermod -aG dialout $USER 후 다시 로그인)"
        if "lock" in text.lower() or err in (errno.EAGAIN, errno.EBUSY):
            return None, "사용 중 — 다른 프로그램(white 노드 · gps_mapper 등)이 열고 있다. 끄고 다시"
        return None, f"열기 실패: {text}"


# ── GPS ──────────────────────────────────────────────────────────────────────────
def poll_ubx(ser, cls, id_, timeout, stream_bytes):
    """UBX 조회를 보내고 응답 payload 를 돌려준다(설정은 바꾸지 않는다). 그동안 읽은 바이트는 stream_bytes 에 모은다."""
    want = b"\xb5\x62" + bytes([cls, id_])
    buf, t0, last_ask = b"", time.time(), 0.0
    while time.time() - t0 < timeout:
        if time.time() - last_ask > 0.5:
            ser.write(ubx(cls, id_))
            last_ask = time.time()
        data = ser.read(4096)
        stream_bytes.append(data)
        buf += data
        i = buf.find(want)
        while i >= 0:
            if len(buf) < i + 6:
                break
            n = int.from_bytes(buf[i + 4:i + 6], "little")
            if len(buf) < i + 8 + n:
                break
            frame = buf[i:i + 8 + n]
            if ubx(cls, id_, frame[6:6 + n]) == frame:
                return frame[6:6 + n]
            i = buf.find(want, i + 2)
        buf = buf[-2048:]
    return None


def firmware(ver):
    if not ver:
        return ""
    for k in range(40, len(ver), 30):
        s = ver[k:k + 30].split(b"\0")[0].decode("ascii", "replace")
        if s.startswith("FWVER="):
            return s[6:]
    return ""


def check_gps():
    print("\n■ GPS (u-blox VID 1546)")
    found = usb_ports(lambda v, p: v in ports.GPS_VIDS)
    if not found:
        say(BAD, "u-blox 장치가 USB 에 없다 — 케이블 · 허브 확인")
    seen = {}                    # 칩 ID → 장치
    for dev, vid, pid, desc in found:
        print(f"   ── {dev}  ({vid:04x}:{pid:04x} {desc})")
        ser, why = open_port(dev, GPS_BAUD, 0.05)
        if ser is None:
            say(BAD, why, 6)
            continue
        try:
            ser.reset_input_buffer()
            raw = []
            uid = poll_ubx(ser, 0x27, 0x03, ID_TIMEOUT_S, raw)
            fw = firmware(poll_ubx(ser, 0x0A, 0x04, 2.0, raw))
            t0 = time.time()
            while time.time() - t0 < GPS_LISTEN_S:
                raw.append(ser.read(4096))
        except (serial.SerialException, OSError) as e:
            say(BAD, f"읽기 중 단절: {e}", 6)
            continue
        finally:
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass
        chip = uid[4:9].hex() if uid and len(uid) >= 9 else ""
        if not chip:
            say(BAD, "칩 ID 응답 없음 — u-blox 가 맞는지, 펌웨어가 SEC-UNIQID 를 아는지", 6)
        else:
            name = ports.GPS_IDS.get(chip)
            fw_txt = f", 펌웨어 {fw}" if fw else ""
            if name:
                say(OK, f"{name} — 칩 ID {chip}{fw_txt}", 6)
            else:
                say(WARN, f"등록 안 된 수신기 — 칩 ID {chip}{fw_txt} (ports.GPS_IDS 에 없다)", 6)
            seen[chip] = dev
        ggas, pvts = F9PStream().feed(b"".join(raw))
        if pvts:
            p = pvts[-1]
            q = pvt_quality(p)
            pos = f"  {p['lat']:.7f}, {p['lon']:.7f}" if q else ""
            say(OK if q else WARN, f"NAV-PVT {len(pvts)}개 받음 — "
                                   f"{Q_LABEL[q]}, 위성 {p['sv']}, hAcc {p['hacc']:.2f} m{pos}", 6)
        elif ggas:
            g = ggas[-1]["q"]
            say(WARN, f"NMEA(GGA {len(ggas)}줄, 품질 {g})만 온다 — 공장 설정 상태. "
                      f"gps 노드가 열면 NAV-PVT 10 Hz 로 바꾼다(RAM)", 6)
        else:
            say(BAD if not chip else WARN, "위치 데이터(NAV-PVT · GGA)가 안 온다", 6)
    for chip, name in ports.GPS_IDS.items():
        if chip not in seen:
            say(WARN, f"{name} 없음 (칩 ID {chip})")
    known = [d for c, d in seen.items() if c in ports.GPS_IDS]
    if len(found) > 1:
        first = found[0][0]
        who = next((ports.GPS_IDS.get(c, f"칩 {c}") for c, d in seen.items() if d == first), "?")
        say(WARN, f"u-blox 가 {len(found)}대 — white gps 노드는 번호가 작은 {first} ({who}) 하나만 연다")
    return bool(known)


# ── IMU ──────────────────────────────────────────────────────────────────────────
def imu_lines(ser, secs):
    """secs 동안 9필드 숫자 줄 → [[9개 float]]"""
    out, t0 = [], time.time()
    while time.time() - t0 < secs:
        line = ser.readline()
        if not line:
            continue
        s = line.split(b"\r\n")[0].decode("utf-8", "ignore").strip()
        if not s or "=" in s:
            continue
        f = s.split(",")
        if len(f) != 9:
            continue
        try:
            out.append([float(x) for x in f])
        except ValueError:
            continue
    return out


def check_imu():
    print("\n■ IMU (iAHRS · CP210x 10c4:ea60)")
    found = usb_ports(lambda v, p: (v, p) in ports.IMU_VIDPIDS)
    if not found:
        say(BAD, "CP210x 장치가 USB 에 없다 — 케이블 확인")
        return False
    ok = False
    for dev, vid, pid, desc in found:
        print(f"   ── {dev}  ({vid:04x}:{pid:04x} {desc})")
        ser, why = open_port(dev, IMU_BAUD, 0.05)
        if ser is None:
            say(BAD, why, 6)
            continue
        try:
            ser.reset_input_buffer()
            rows = imu_lines(ser, IMU_LISTEN_S[0])
            secs = IMU_LISTEN_S[0]
            sent = False
            if not rows:
                for cmd in IMU_SETUP:
                    ser.write((cmd + "\n").encode())
                    time.sleep(0.15)
                sent = True
                ser.reset_input_buffer()
                rows = imu_lines(ser, IMU_LISTEN_S[1])
                secs = IMU_LISTEN_S[1]
        except (serial.SerialException, OSError) as e:
            say(BAD, f"읽기 중 단절: {e}", 6)
            continue
        finally:
            try:
                ser.close()
            except (serial.SerialException, OSError):
                pass
        if rows:
            a = rows[-1]
            g = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
            how = " (출력 명령을 보낸 뒤)" if sent else ""
            say(OK, f"데이터 {len(rows) / secs:.0f} 줄/s{how} — |가속도| {g:.2f} g, "
                    f"각속도 {a[3]:+.1f} {a[4]:+.1f} {a[5]:+.1f} °/s, 자세 {a[6]:+.0f} {a[7]:+.0f} {a[8]:+.0f}°", 6)
            if not 0.8 < g < 1.2:
                say(WARN, "가만히 둔 상태라면 |가속도| 가 1 g 근처여야 한다", 6)
            ok = True
        else:
            say(BAD, "데이터 줄이 안 온다(115200 bps) — iAHRS 가 맞는지, 전원 확인", 6)
    return ok


# ── 아두이노 ──────────────────────────────────────────────────────────────────────
def listen_arduino(dev, baud):
    """열고(리셋) 부팅 · 듣기 → (줄 목록, 까닭). 못 열면 (None, 까닭)."""
    ser, why = open_port(dev, baud, 0.1)
    if ser is None:
        return None, why
    lines, buf = [], b""
    try:
        t0 = time.time()
        while time.time() - t0 < ARDUINO_BOOT_S + ARDUINO_LISTEN_S:
            buf += ser.read(256)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                lines.append(line.strip().decode("ascii", errors="replace"))
    except (serial.SerialException, OSError) as e:
        return None, f"읽기 중 단절: {e}"
    finally:
        try:
            ser.close()
        except (serial.SerialException, OSError):
            pass
    return lines, ""


def e_format(line):
    n = len(line.split(",")) - 1
    return {1: "v6 계열", 2: "v6 계열(dt)", 5: "Vr.1", 7: "white_fw"}.get(n, f"{n}필드 — 모르는 형식")


def check_arduino(open_it, baud):
    print("\n■ 아두이노 (VID 2341 · 2a03 · 1a86)")
    found = usb_ports(lambda v, p: v in ports.ARDUINO_VIDS)
    if not found:
        say(BAD, "아두이노가 USB 에 없다 — 케이블 · 전원 확인")
        return False
    for dev, vid, pid, desc in found:
        say(OK, f"{dev}  ({vid:04x}:{pid:04x} {desc}) — USB 에 있다")
    if len(found) > 1:
        say(WARN, f"아두이노 후보가 {len(found)}개 — arduino 노드는 번호가 작은 {found[0][0]} 를 연다")
    if not open_it:
        say("  ", "포트는 열지 않았다(열면 리셋). 펌웨어 · E 줄까지 보려면 --arduino")
        return True
    dev = found[0][0]
    say("  ", f"{dev} 를 연다 — 리셋되므로 {ARDUINO_BOOT_S + ARDUINO_LISTEN_S:.0f}s 듣는다 ({baud} bps)")
    lines, why = listen_arduino(dev, baud)
    if lines is None:
        say(BAD, why, 6)
        return False
    boot = [s for s in lines if s.startswith("H,")]
    text = [s for s in lines if s and not s.startswith(("E,", "H,", "P,", "A,", "D,"))
            and sum(c.isalpha() for c in s) >= 3 and "\ufffd" not in s]
    e_lines = [s for s in lines if s.startswith("E,")]
    if boot:
        say(OK, f"부팅 줄: {boot[0]}", 6)
    elif text:
        say(WARN, f"부팅 줄(H) 없음 — 다른 스케치다. 처음 글자 줄: {text[0][:70]!r}", 6)
    if e_lines:
        say(OK, f"E 줄 {len(e_lines) / (ARDUINO_BOOT_S + ARDUINO_LISTEN_S):.0f} 줄/s(부팅 포함 평균) — {e_format(e_lines[-1])}, "
                f"마지막: {e_lines[-1]}", 6)
        if not e_lines[-1].count(",") == 7:
            say(WARN, "white_fw(hl_ws/tool/arduino_code.ino)가 아니다 — 올리면 부팅 줄 · dt · 리모컨 표시 · 명령 시한이 생긴다", 6)
        return True
    junk = sum(s.count("\ufffd") for s in lines)
    say(BAD, f"E 줄이 없다 ({len(lines)}줄{', 깨진 글자 많음' if junk > 20 else ''}) — {ARDUINO_ALT_BAUD} bps 로 다시 들어 본다", 6)
    alt, why = listen_arduino(dev, ARDUINO_ALT_BAUD)
    alt_e = [s for s in (alt or []) if s.startswith("E,")]
    if alt_e:
        say(WARN, f"{ARDUINO_ALT_BAUD} bps 에서는 E 줄이 온다 — {e_format(alt_e[-1])}: {alt_e[-1]}. "
                  f"스케치가 {ARDUINO_ALT_BAUD} bps 다 → hl_ws/tool/arduino_code.ino(115200)를 올리거나 arduino_baud:={ARDUINO_ALT_BAUD}", 6)
    else:
        say(BAD, f"{ARDUINO_ALT_BAUD} bps 에서도 E 줄이 없다 — 스케치 확인", 6)
    return False


def main(args=None):
    ap = argparse.ArgumentParser(prog="ros2 run white check", description="GPS 2대 · IMU · 아두이노 연결 점검")
    ap.add_argument("--arduino", action="store_true", help="아두이노 포트도 열어 E 줄 확인 (열면 리셋된다)")
    ap.add_argument("--baud", type=int, default=ARDUINO_BAUD, help=f"아두이노 baud (기본 {ARDUINO_BAUD})")
    opt, _ = ap.parse_known_args(args if args is not None else sys.argv[1:])   # --ros-args 등은 무시

    print("white 장치 점검 — one_launch 가 떠 있으면 포트가 '사용 중' 으로 나온다")
    gps_ok = check_gps()
    imu_ok = check_imu()
    ard_ok = check_arduino(opt.arduino, opt.baud)
    print("\n■ 요약")
    for name, ok in (("GPS", gps_ok), ("IMU", imu_ok), ("아두이노", ard_ok)):
        say(OK if ok else BAD, name)
    sys.exit(0 if (gps_ok and imu_ok and ard_ok) else 1)


if __name__ == '__main__':
    main()
