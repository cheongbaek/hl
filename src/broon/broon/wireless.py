#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wireless.py ― 휴대폰 무선 조종 (broon 앱 ↔ joy 노드) : Wi-Fi 핫스팟 · UDP 규약 [broon]  (2026-11-23)
════════════════════════════════════════════════════════════════════════════════
ROS 를 import 하지 않는다. joy 노드가 쓴다.
2026-11-23 실기(사용자 PC · S25 Ultra) : 도우미로 손으로 켠 핫스팟(5 GHz 36ch)에 앱 v1.3 이 붙어 DHCP 10.42.0.x 를 받고
  "BRN1,…" 을 25.0 Hz · 빠짐 0 · 최대 간격 97 ms 로 보냈다. ⚠️ joy 노드와 함께(답 · 조종 · 켜고 끄기)는 아직 — 사용자가 직접 시험한다.

흐름 (2026-11-23 사용자 지정)
  · ★joy 노드가 켜지면 곧바로 이 PC 에 Wi-Fi 핫스팟(AP)을 만들고, joy 노드가 끝나면 내린다★ — joy 인자 hotspot(기본 true).
    도중에 내려가 있으면(CHECK_S 마다 본다) 다시 만들고, 실패하면 RETRY_S 뒤 다시. hotspot:=false 면 만들지 않는다.
  · 휴대폰 broon 앱 : 터치 모드의 Wi-Fi 단추 → AP_SSID 를 찾아 붙는다(Android WifiNetworkSpecifier) → AP_IP:UDP_PORT 로 명령.
  · ★우선순위 : 실물 조이스틱 > 휴대폰★. 실물 조이스틱 줄이 오는 동안 휴대폰 명령은 버린다.
  · 휴대폰 ON = 실물 조이스틱 SWA ON 과 같다 — /joy318 을 낸다(자율주행 정지 · 출발 거절).
  · SSID · 비밀번호는 코드에 둔다(사용자 지정 — ★공개 리포에 보인다★).

핫스팟 — ★켜진 동안 이 PC 의 Wi-Fi 접속(무선 인터넷)은 끊긴다★ (2026-11-23 사용자 지정)
  무선칩(Intel AX211, wlo1)을 핫스팟 전용으로 쓴다. 인터넷은 유선 · 테더링으로 받는다 — ★원격 E-STOP(estop)도 그쪽으로 붙어야 한다★.
  joy 를 끝내 핫스팟을 내리면 NetworkManager 가 원래 Wi-Fi 에 다시 붙는다.
  처음 설계(같은 칩에 가상 AP 인터페이스를 더 만들어 Wi-Fi 인터넷과 동시에)는 2026-11-23 실기에서 안 됐다 —
  NetworkManager · wpa_supplicant 가 새 인터페이스를 managed 로 바꿔 올리는데 AX211 은 managed 를 하나만 허락한다(EBUSY).
  만들기 · 지우기는 root 가 필요하다 → 도우미 /usr/local/sbin/broon-ap (src/broon/hotspot/broon-ap)를 `sudo -n` 으로 부른다.
  ★처음 한 번 사용자가★  sudo bash ~/hl_ws/src/broon/hotspot/setup_hotspot.sh
      (iw 설치 · 도우미 설치 · sudoers 한 줄 · ufw 가 켜져 있으면 Wi-Fi 장치의 DHCP · DNS · UDP_PORT 허용). 도우미를 고치면 다시 돌린다.
  규격 : WPA2-PSK (RSN · CCMP). 대역은 도우미가 고른다 — 5 GHz 36 → 149 → 2.4 GHz 6 (DFS 채널은 안 쓴다).
  주소 : PC = AP_IP(10.42.0.1/24, NetworkManager 공유 모드) · 휴대폰은 DHCP 로 10.42.0.x. 인터넷은 나눠 주지 않는다(앱이 안 쓴다).

UDP 규약 — ASCII, 데이터그램 하나 = 줄 하나 (끝 줄바꿈은 있어도 없어도 된다). PC 는 AP_IP 에만 묶는다(바깥 인터넷 쪽에서는 안 받는다).
  휴대폰 → PC  "BRN1,<seq>,<on>,<pulse>,<steer>"
      seq   0 부터 늘어나는 정수. 같은 휴대폰에서 앞 것보다 작거나 같으면 버린다(뒤바뀐 데이터그램).
      on    1 · 0 (앱의 ON/OFF).   pulse −92 ~ 92 · steer −21 ~ 21 (− 좌) 정수 — 우노에 가는 값 그대로.
      ★OFF 거나 스틱이 가운데여도 PHONE_SEND_HZ(25 Hz)로 계속 보낸다★ — PHONE_TIMEOUT_S(0.3 s) 동안 없으면 끊긴 것으로 보고 "0 0" 마무리.
  PC → 휴대폰  받은 줄마다 "BRN1S,<seq>,<state>,<estop>,<ref>,<ang>,<meas>,<meas_ang>"
      seq = 받은 그대로.  estop 0 · 1.  뒤 넷 = 우노 되먹임(/enc_pot) — 0.5 s 넘게 없으면 네 칸 모두 빈칸.
      state  READY  휴대폰이 조종할 수 있다 · 지금 OFF
             ACTIVE 휴대폰 명령을 /cmd_vel 로 낸다 (ON)
             JOY    실물 조이스틱이 붙어 있다 — 휴대폰 명령은 버린다 (우선순위)
             ARM    처음 붙었거나 실물 조이스틱에서 넘어왔다 — on=0 을 한 번 받아야 ON 을 받는다
             BUSY   다른 휴대폰이 쥐고 있다 (마지막 줄 뒤 PHONE_HOLD_S 동안)
"""

import ipaddress
import socket
import subprocess
import threading
import time

# ── 네트워크 (도우미 broon-ap · setup_hotspot.sh 와 같아야 한다) ──
AP_SSID = 'BRN-hl-l1QLctI38mzu'          # 2026-11-23 무작위 — 앱(E:\vibe\broon)에도 같은 값
AP_PSK = 'NIn3bZcZRTmymMDigW3i8UVc'      # WPA2 비밀번호 (사용자 지정: 코드에 둔다)
AP_IP = '10.42.0.1'                      # broon-ap 의 AP_ADDR
AP_NET = ipaddress.ip_network('10.42.0.0/24')
UDP_PORT = 47318                         # setup_hotspot.sh 의 ufw 허용 포트와 같다

# ── 핫스팟 도우미 ──
HELPER = '/usr/local/sbin/broon-ap'
SETUP_HINT = 'sudo bash ~/hl_ws/src/broon/hotspot/setup_hotspot.sh'
UP_TIMEOUT_S = 100.0                     # 도우미 'up' 최대 (후보 대역 셋, 하나마다 nmcli 가 최대 30 s)
DOWN_TIMEOUT_S = 15.0
RETRY_S = 30.0                           # 실패하면 이만큼 뒤에 다시
CHECK_S = 15.0                           # 켜진 뒤 이 간격으로 살아 있나 본다

# ── 휴대폰 링크 ──
PROTO = 'BRN1'
REPLY = 'BRN1S'
PHONE_SEND_HZ = 25                       # 앱이 보내는 빈도 (앱 쪽 약속)
PHONE_TIMEOUT_S = 0.3                    # 이만큼 줄이 없으면 휴대폰이 끊긴 것
PHONE_HOLD_S = 2.0                       # 다른 휴대폰이 끼어들려면 지금 휴대폰이 이만큼 조용해야 한다
STATES = ('READY', 'ACTIVE', 'JOY', 'ARM', 'BUSY')


def parse_phone(text, pulse_max, steer_max):
    """휴대폰 줄 → (seq, on, pulse, steer) 또는 None. 정수가 아니거나 범위 밖이면 None."""
    parts = text.strip().split(',')
    if len(parts) != 5 or parts[0] != PROTO:
        return None
    try:
        seq, on, pulse, steer = (int(p) for p in parts[1:])
    except ValueError:
        return None
    if seq < 0 or on not in (0, 1) or not (-pulse_max <= pulse <= pulse_max) or not (-steer_max <= steer <= steer_max):
        return None
    return seq, on, pulse, steer


def format_reply(seq, state, estop, fb):
    """PC → 휴대폰 한 줄. fb = (ref, ang, meas, meas_ang) 또는 None(빈칸)."""
    tail = (f"{int(fb[0])},{int(fb[1])},{int(fb[2])},{fb[3]:.1f}" if fb is not None else ",,,")
    return f"{REPLY},{seq},{state},{1 if estop else 0},{tail}"


class Hotspot:
    """도우미(broon-ap)를 `sudo -n` 으로 불러 핫스팟을 올리고 내린다. 부르는 일은 스레드에서 — 호출자를 막지 않는다.

    state : off(안 만듦) · starting · on · failed(다음 시도는 RETRY_S 뒤) · stopping
    """

    def __init__(self, log_info, log_warn):
        self.log_info, self.log_warn = log_info, log_warn
        self.lock = threading.Lock()
        self.state = 'off'
        self.detail = ''                 # 화면용 한 줄 (대역 · 채널 · 실패 이유)
        self.next_try = 0.0
        self.next_check = 0.0
        self.touched = False             # 한 번이라도 up 을 불렀다 → 끝낼 때 down
        self.closing = False

    @staticmethod
    def _run(args, timeout):
        try:
            r = subprocess.run(['sudo', '-n', HELPER, *args], capture_output=True, text=True, timeout=timeout)
        except FileNotFoundError as e:
            return 127, str(e)
        except subprocess.TimeoutExpired:
            return 124, f"{timeout:.0f}s 안에 안 끝났다"
        return r.returncode, (r.stdout + r.stderr).strip()

    @staticmethod
    def _why(code, out):
        low = out.lower()
        if 'password is required' in low or 'a terminal is required' in low or 'not allowed' in low or code == 127:
            return f"도우미를 쓸 수 없다 — 처음 한 번 {SETUP_HINT}"
        last = out.splitlines()[-1] if out else f"종료 코드 {code}"
        return last[:160]

    def request_up(self, now):
        """올리기 시작했으면 True. 이미 올라가는 중 · 켜짐 · 재시도 대기면 False."""
        with self.lock:
            if self.closing or self.state in ('starting', 'on', 'stopping') or now < self.next_try:
                return False
            self.state, self.touched = 'starting', True
            self.detail = f"만드는 중 — {AP_SSID}"
        threading.Thread(target=self._up, daemon=True).start()
        return True

    def _up(self):
        code, out = self._run(['up', AP_SSID, AP_PSK], UP_TIMEOUT_S)
        ok_line = next((ln for ln in out.splitlines() if ln.startswith('OK ')), None)
        with self.lock:
            if self.closing:
                return
            if code == 0 and ok_line:
                _, band, ch, dev = (ok_line.split() + ['?', '?', '?'])[:4]
                self.state = 'on'
                # 화면 한 줄에 들어가게 짧게 (joy 창은 크기가 고정이다)
                self.detail = f"켜짐 · {AP_SSID} · {'5 GHz' if band == 'a' else '2.4 GHz'} {ch}ch · {AP_IP}:{UDP_PORT}"
                self.next_check = time.monotonic() + CHECK_S
                self.log_info(f"📶 핫스팟 {self.detail} ({dev}) — 켜진 동안 이 PC 의 Wi-Fi 접속은 끊긴다")
            else:
                self.state = 'failed'
                self.detail = self._why(code, out)
                self.next_try = time.monotonic() + RETRY_S
                self.log_warn(f"📶 핫스팟을 못 만들었다 — {self.detail} ({RETRY_S:.0f}s 뒤 다시)")

    def maybe_check(self, now):
        """켜진 동안 CHECK_S 마다 살아 있나 본다 — 내려가 있으면 off 로 (joy 가 다음 tick 에 다시 올린다)."""
        with self.lock:
            if self.closing or self.state != 'on' or now < self.next_check:
                return
            self.next_check = now + CHECK_S
        threading.Thread(target=self._check, daemon=True).start()

    def _check(self):
        code, out = self._run(['status'], DOWN_TIMEOUT_S)
        with self.lock:
            if self.closing or self.state != 'on':
                return
            if code != 0:
                self.state, self.detail = 'off', f"내려가 있다 ({self._why(code, out)})"
                self.log_warn(f"📶 핫스팟이 내려갔다 — {out[-120:] if out else code}")

    def down(self):
        """끝낼 때 (막는다, 최대 DOWN_TIMEOUT_S). 한 번도 올린 적이 없으면 아무것도 안 한다.
        올리는 중('up' 이 아직 도는 중)이어도 된다 — 도우미의 down 이 그 up 을 먼저 멈추고 연결을 지운다."""
        with self.lock:
            self.closing = True
            if not self.touched:
                return
            self.state = 'stopping'
        code, out = self._run(['down'], DOWN_TIMEOUT_S)
        with self.lock:
            self.state = 'off'
            self.detail = '내림' if code == 0 else f"내리기 실패 — {self._why(code, out)} (손으로: sudo {HELPER} down)"
        (self.log_info if code == 0 else self.log_warn)(f"📶 핫스팟 {self.detail}")

    def describe(self):
        with self.lock:
            return self.state, self.detail


def clear_stale(log_info, log_warn):
    """hotspot:=false 로 켤 때 — 지난번 joy 가 강제로 끝나(SIGKILL 등) 남은 핫스팟이 있으면 내린다 (스레드에서, 막지 않는다).
    도우미가 없거나(setup 전) 꺼져 있으면 아무것도 안 한다."""
    def run():
        code, _ = Hotspot._run(['status'], DOWN_TIMEOUT_S)
        if code != 0:
            return
        code, out = Hotspot._run(['down'], DOWN_TIMEOUT_S)
        if code == 0:
            log_info("📶 지난번에 남은 핫스팟을 내렸다 (hotspot:=false)")
        else:
            log_warn(f"📶 남은 핫스팟을 못 내렸다 — {Hotspot._why(code, out)} (손으로: sudo {HELPER} down)")
    threading.Thread(target=run, daemon=True).start()


class UdpLink:
    """AP_IP:UDP_PORT 에서 휴대폰 줄을 받는다 (스레드). AP 가 아직 없으면 1 s 마다 다시 묶어 본다.
    on_line(text, addr) 이 답할 줄(str)을 돌려주면 보낸 쪽으로 돌려보낸다. AP_NET 밖에서 온 것은 버린다."""

    def __init__(self, on_line, stop_evt, log_info, log_warn):
        self.on_line = on_line
        self.stop_evt = stop_evt
        self.log_info, self.log_warn = log_info, log_warn
        self.bound = False
        self.rx_n = 0
        self.bad_n = 0                   # 형식 · 출처가 틀려 버린 것
        self.sock = None

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _bind(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((AP_IP, UDP_PORT))    # ★AP 주소에만★ — 바깥(인터넷 쪽) 주소로 온 줄은 받지 않는다
        except OSError:
            s.close()
            return None
        s.settimeout(0.5)
        return s

    def _run(self):
        while not self.stop_evt.is_set():
            if self.sock is None:
                self.sock = self._bind()
                if self.sock is None:
                    self.stop_evt.wait(1.0)  # AP 가 아직 없다 (주소가 안 생겼다)
                    continue
                self.bound = True
                self.log_info(f"📱 휴대폰 UDP 받기 시작 {AP_IP}:{UDP_PORT}")
            try:
                data, addr = self.sock.recvfrom(512)
            except socket.timeout:
                continue
            except OSError as e:
                self.log_warn(f"휴대폰 UDP 오류 — 다시 묶는다: {e}")
                self._close()
                self.stop_evt.wait(1.0)
                continue
            try:
                if ipaddress.ip_address(addr[0]) not in AP_NET:
                    self.bad_n += 1
                    continue
            except ValueError:
                self.bad_n += 1
                continue
            self.rx_n += 1
            reply = self.on_line(data.decode('ascii', errors='ignore'), addr)
            if reply is None:
                continue
            try:
                self.sock.sendto(reply.encode('ascii'), addr)
            except OSError:
                pass                         # 휴대폰이 막 떠났다 — 다음 줄에서 다시

    def _close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock, self.bound = None, False

    def close(self):
        self._close()
