#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check.py ― 장치 연결 점검 (전방 · 후방 GPS, 구동 아두이노, 조이스틱)
════════════════════════════════════════════════════════════════════════════════
    ros2 run broon check            전방 GPS + 구동 아두이노 + 조이스틱 (white1 구성)
    ros2 run broon check --dual     후방 GPS 까지 있어야 통과 (듀얼 구성)
    ros2 run broon check --no-joy   조이스틱이 없어도 통과

★런치(broon.launch · one.launch)를 끄고★ 돌린다 — 노드가 포트를 쥐고 있으면 열 수 없어 '사용 중' 으로 나온다.
ROS 노드가 아니다.

  GPS      u-blox(VID 1546) 포트를 하나씩 열어 칩 고유 ID · 펌웨어를 ★조회만★ 하고(설정은 바꾸지 않는다)
           2초 동안 들어오는 NAV-PVT · GGA 로 측위 상태를 본다. 배치 설정(config/gps_setting.json —
           ros2 run broon gps_setting)의 ID 와 맞춰 전방 · 후방을 표시한다.
  아두이노 arduino 노드와 같은 방법으로 가른다(boards.py) — 아두이노 포트를 모두 열어 최대 3.5 초 동안 줄을 읽고,
           "J," 줄을 보내는 쪽 = ★조이스틱★(메가), 나머지 = ★구동계★(우노). 아무것도 보내지 않는다.
           구동은 broon_1119 의 출력 "받은 펄스 받은 조향 실측 펄스 실측 조향" 이 오는지까지 본다.
           메가는 포트를 열면 리셋된다(부트로더 1 ~ 2 초) — 그래서 기다린다.

끝 상태 : 전방 GPS(--dual 이면 후방도) + 구동 아두이노 + 조이스틱(--no-joy 면 빼고)이 다 확인되면 0, 아니면 1.
"""

import argparse
import sys
import time

import serial

from broon import boards, gpsdev
from broon.gpsdev import F9PStream, Q_LABEL, ROLE_KO, pvt_quality

GPS_LISTEN_S = 2.0
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


def check_arduino(need_joy):
    """(구동 ok, 조이스틱 ok)"""
    print("\n■ 아두이노 — \"J,\" 줄 = 조이스틱, 나머지 = 구동 (UNO R4 Minima 는 2341:0069)")
    found = boards.candidate_ports()
    if not found:
        say(BAD, "아두이노가 USB 에 없다 — 케이블 · 전원 확인")
        say(BAD if need_joy else WARN, "조이스틱 없음")
        return False, not need_joy
    for dev, vid, pid, desc in found:
        kind = " — UNO R4 Minima" if (vid, pid) == boards.R4_MINIMA else ""
        say("  ", f"{dev}  ({(vid or 0):04x}:{(pid or 0):04x} {desc}){kind}")
    print(f"   … 포트를 모두 열어 최대 {boards.DETECT_WINDOW_S:.1f}초 동안 줄을 읽는다 (메가는 리셋된다)")
    res = boards.probe([d for d, *_ in found], keep_drive=False)
    for dev, why in res['busy'].items():
        say(BAD, f"{dev} 열기 실패 — 런치가 쥐고 있나? ({why})", 6)

    drive_ok = False
    if res['drive']:
        lines = res['lines'].get(res['drive'], [])
        fbs = [fb for fb in (boards.parse_drive_line(t) for t in lines) if fb is not None]
        if fbs:
            r, a, m, ma = fbs[-1]
            say(OK, f"구동 = {res['drive']} — 출력 정상 (받은 {r:+d} {a:+d}° · 실측 {m:+d} {ma:+.2f}°)", 6)
            drive_ok = True
        elif lines:
            say(WARN, f"구동 = {res['drive']} — 줄은 오는데 broon_1119 출력 형식이 아니다: {lines[-1]!r} "
                      f"(스케치 확인)", 6)
        else:
            say(WARN, f"구동 = {res['drive']} — 아무 줄도 안 온다 (broon_1119 를 올렸나?)", 6)
    else:
        say(BAD, "구동 아두이노 없음 (\"J,\" 를 보내지 않는 아두이노가 없다)", 6)
    for dev in res['extra']:
        say(WARN, f"{dev} — 구동 후보가 하나 더 있다. arduino 노드는 하나만 쓴다", 6)

    joy_ok = False
    if res['joy']:
        vals = [v for v in (boards.parse_joy_line(t) for t in res['lines'].get(res['joy'], [])) if isinstance(v, tuple)]
        sw = f" — SWA {'ON' if vals[-1][8] else 'OFF'}, L({vals[-1][0]},{vals[-1][1]}) R({vals[-1][3]},{vals[-1][4]})" \
            if vals else ""
        say(OK, f"조이스틱 = {res['joy']}{sw}", 6)
        joy_ok = True
    else:
        say(BAD if need_joy else WARN, "조이스틱 없음 (\"J,\" 줄을 보내는 아두이노가 없다)", 6)
    return drive_ok, joy_ok or not need_joy


def main(args=None):
    ap = argparse.ArgumentParser(prog="ros2 run broon check", description="GPS · 구동 아두이노 · 조이스틱 연결 점검")
    ap.add_argument("--dual", action="store_true", help="후방 GPS 도 있어야 통과 (듀얼 구성)")
    ap.add_argument("--no-joy", action="store_true", help="조이스틱이 없어도 통과")
    opt, _ = ap.parse_known_args(args if args is not None else sys.argv[1:])
    print("broon 장치 점검 — 런치가 떠 있으면 포트가 '사용 중' 으로 나온다")
    gps_ok = check_gps(opt.dual)
    drive_ok, joy_ok = check_arduino(not opt.no_joy)
    print("\n■ 요약")
    say(OK if gps_ok else BAD, "GPS" + (" (전방 + 후방)" if opt.dual else " (전방)"))
    say(OK if drive_ok else BAD, "구동 아두이노")
    say(OK if joy_ok else BAD, "조이스틱" + (" (없어도 통과 — --no-joy)" if opt.no_joy else ""))
    sys.exit(0 if (gps_ok and drive_ok and joy_ok) else 1)


if __name__ == '__main__':
    main()
