#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check.py ― 장치 연결 점검 (전방 · 후방 GPS, 아두이노)
════════════════════════════════════════════════════════════════════════════════
    ros2 run broon check            전방 GPS + 아두이노 (white1 구성)
    ros2 run broon check --dual     후방 GPS 까지 있어야 통과 (white2 구성)
    ros2 run broon check --arduino  아두이노 포트도 열어 2초 동안 오는 바이트를 본다

★one_launch 를 끄고★ 돌린다 — 노드가 포트를 쥐고 있으면 열 수 없어 '사용 중' 으로 나온다. ROS 노드가 아니다.

  GPS     u-blox(VID 1546) 포트를 하나씩 열어 칩 고유 ID · 펌웨어를 ★조회만★ 하고(설정은 바꾸지 않는다)
          2초 동안 들어오는 NAV-PVT · GGA 로 측위 상태를 본다. 배치 설정(config/gps_setting.json —
          ros2 run broon gps_setting)의 ID 와 맞춰 전방 · 후방을 표시한다.
  아두이노 USB 목록에 있는지 본다(UNO R4 Minima = 2341:0069). 새 규약은 아두이노 → PC 응답이 없어
          통신 자체는 확인할 수 없다. --arduino 면 포트를 열어 오는 바이트만 본다(아무것도 보내지 않는다).

끝 상태 : 전방 GPS(--dual 이면 후방도) + 아두이노가 다 확인되면 0, 아니면 1.
"""

import argparse
import sys
import time

import serial
import serial.tools.list_ports

from broon import gpsdev
from broon.arduino import ARDUINO_VIDS, BAUD, R4_MINIMA
from broon.gpsdev import F9PStream, Q_LABEL, ROLE_KO, pvt_quality

GPS_LISTEN_S = 2.0
ARDUINO_LISTEN_S = 2.0
OK, WARN, BAD = "✅", "⚠️ ", "❌"


def say(mark, text, indent=3):
    print(" " * indent + f"{mark} {text}")


def check_gps(need_rear):
    setting = gpsdev.load_setting()
    by_id = {setting[r]['id']: r for r in gpsdev.ROLES if setting[r]['id']}
    print(f"\n■ GPS (u-blox VID 1546) — 배치 설정 {gpsdev.config_path()}")
    for r in gpsdev.ROLES:
        print(f"   {ROLE_KO[r]} = {setting[r]['id'] or '미지정'}")
    ports = gpsdev.list_ublox_ports()
    if not ports:
        say(BAD, "u-blox 장치가 USB 에 없다 — 케이블 · 허브 확인")
    seen = {}                    # 역할 → 포트
    for p in ports:
        print(f"   ── {p.device}  ({p.vid:04x}:{p.pid:04x} {p.name})")
        ser, why = gpsdev.open_port(p.device)
        if ser is None:
            say(BAD, why, 6)
            continue
        raw = []
        try:
            ser.reset_input_buffer()
            chip = gpsdev.read_chip_id(ser, sink=raw)
            fw = gpsdev.read_firmware(ser, sink=raw)
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
        fw_txt = f", 펌웨어 {fw}" if fw else ""
        if not chip:
            say(BAD, "칩 ID 응답 없음 — u-blox 가 맞는지 확인", 6)
        elif chip in by_id:
            role = by_id[chip]
            say(OK, f"{ROLE_KO[role]} GPS — 칩 ID {chip}{fw_txt}", 6)
            seen[role] = p.device
        else:
            say(WARN, f"배치 설정에 없는 수신기 — 칩 ID {chip}{fw_txt} (ros2 run broon gps_setting 으로 지정)", 6)
        ggas, pvts = F9PStream().feed(b"".join(raw))
        if pvts:
            v = pvts[-1]
            q = pvt_quality(v)
            pos = f"  {v['lat']:.7f}, {v['lon']:.7f}" if q else ""
            say(OK if q else WARN, f"NAV-PVT {len(pvts)}개 — {Q_LABEL[q]}, 위성 {v['sv']}, hAcc {v['hacc']:.2f} m{pos}", 6)
        elif ggas:
            say(WARN, f"NMEA(GGA {len(ggas)}줄, 품질 {ggas[-1]['q']})만 온다 — 공장 설정 상태. "
                      f"gps 노드가 열면 NAV-PVT 10 Hz 로 바꾼다(RAM)", 6)
        else:
            say(WARN if chip else BAD, "위치 데이터(NAV-PVT · GGA)가 안 온다", 6)
    ok = True
    for r in gpsdev.ROLES:
        needed = r == 'front' or need_rear
        if r in seen:
            continue
        if needed:
            ok = False
        say(BAD if needed else WARN, f"{ROLE_KO[r]} GPS 없음 (칩 ID {setting[r]['id'] or '미지정'})"
            + ("" if needed else " — 단일 구성(white1)이면 없어도 된다"))
    return ok


def check_arduino(open_it):
    print("\n■ 아두이노 (VID 2341 · 2a03 · 1a86 — UNO R4 Minima 는 2341:0069)")
    found = sorted((0 if (p.vid, p.pid) == R4_MINIMA else 1, p.device, p.vid, p.pid, p.product or p.description or "")
                   for p in serial.tools.list_ports.comports() if p.vid in ARDUINO_VIDS)
    if not found:
        say(BAD, "아두이노가 USB 에 없다 — 케이블 · 전원 확인")
        return False
    for rank, dev, vid, pid, desc in found:
        kind = "UNO R4 Minima" if rank == 0 else "R4 Minima 가 아니다"
        say(OK if rank == 0 else WARN, f"{dev}  ({vid:04x}:{pid:04x} {desc}) — {kind}")
    if len(found) > 1:
        say(WARN, f"아두이노 후보가 {len(found)}개 — arduino 노드는 {found[0][1]} 를 연다")
    if not open_it:
        say("  ", "새 규약은 아두이노 → PC 응답이 없어 통신은 확인하지 못한다. 오는 바이트를 보려면 --arduino")
        return True
    dev = found[0][1]
    try:
        ser = serial.Serial(dev, BAUD, timeout=0.1, exclusive=True)
    except (serial.SerialException, OSError) as e:
        say(BAD, f"{dev} 열기 실패: {e}", 6)
        return False
    buf, t0 = b"", time.time()
    try:
        while time.time() - t0 < ARDUINO_LISTEN_S:
            buf += ser.read(256)
    except (serial.SerialException, OSError) as e:
        say(BAD, f"읽기 중 단절: {e}", 6)
        return False
    finally:
        ser.close()
    if buf:
        say(OK, f"{ARDUINO_LISTEN_S:.0f}초 동안 {len(buf)}바이트 옴: {buf[:32].hex(' ')} {buf[:32]!r}", 6)
    else:
        say("  ", f"{ARDUINO_LISTEN_S:.0f}초 동안 온 바이트 없음 (규약상 정상 — 아두이노는 받기만 한다)", 6)
    return True


def main(args=None):
    ap = argparse.ArgumentParser(prog="ros2 run broon check", description="전방 · 후방 GPS, 아두이노 연결 점검")
    ap.add_argument("--dual", action="store_true", help="후방 GPS 도 있어야 통과 (white2)")
    ap.add_argument("--arduino", action="store_true", help="아두이노 포트를 열어 오는 바이트를 본다")
    opt, _ = ap.parse_known_args(args if args is not None else sys.argv[1:])
    print("broon 장치 점검 — one_launch 가 떠 있으면 포트가 '사용 중' 으로 나온다")
    gps_ok = check_gps(opt.dual)
    ard_ok = check_arduino(opt.arduino)
    print("\n■ 요약")
    say(OK if gps_ok else BAD, "GPS" + (" (전방 + 후방)" if opt.dual else " (전방)"))
    say(OK if ard_ok else BAD, "아두이노")
    sys.exit(0 if (gps_ok and ard_ok) else 1)


if __name__ == '__main__':
    main()
