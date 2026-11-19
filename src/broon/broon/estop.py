#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
estop.py ― domichat 채팅방으로 받는 원격 E-STOP → /estop [broon]  (2026-11-19 사용자 지정)
════════════════════════════════════════════════════════════════════════════════
    ros2 run broon estop            (broon.launch.py · white1 one.launch.py 가 띄운다 — use_estop:=false 로 끈다)

domichat 서버(domiman.duckdns.org)에 계정 broon_hl_ws 로 로그인해 채팅방 ★domi_fishing_broon★ 을 노드가 도는 동안
구독한다. 방이 없으면 ★비밀번호 제한방(pw)★ 으로 만든다(domi_fishing_* = domiman 낚시방 규격 — 일반 목록에 안 보인다,
비밀번호도 그 규격 FISHING_ROOM_PW). 웹 domiman.duckdns.org/estop 의 버튼이 'web' 계정으로 같은 방에 말을 넣는다.

방에서 받는 말 (앞뒤 공백 · 대소문자 무시. ★자기(broon_hl_ws)가 보낸 말은 무시★)
    STOP     E-STOP 발동 — /estop = 1, 방에 'STOP' 을 보낸다 (이미 STOP 이어도 다시 'STOP' 으로 답한다)
    GO       해제       — /estop = 0, 방에 'GO' 를 보낸다
    STATUS   지금 상태대로 'GO' 또는 'STOP' 을 보낸다
  방에 들어갈 때(재접속 포함)도 지금 상태를 한 번 보낸다 — 웹이 차가 들어온 것을 바로 안다.
  노드를 켜면 GO(0) 로 시작한다. 그 밖의 말은 무시한다(사람이 방에서 대화해도 된다).

발행
  /estop        (std_msgs/Int32, ★래치★ TRANSIENT_LOCAL)  1 = 발동 · 0 = 해제. 바뀌면 곧바로 + 1 초마다 다시.
                broon/arduino : 1 이면 우노에 's'(구동 · 조향 PWM 즉시 0) 뒤 '0 0'(조향 0°)을 보내고, 해제될 때까지
                                /cmd_vel 을 막고 '0 0' 을 0.5 s 마다 보낸다.
                white1/driving : 1 이면 자율주행을 끝낸다(DRIVE_DONE). 1 인 동안 출발을 거절한다.
                broon/record   : 세 방식(수동 · 런치 · 자율) 모두 estop 열에 적는다.
  /estop_status (std_msgs/String) 1 Hz 한 줄 — 채팅 연결 · 방 · 상태

계정 · 접속 — 아래 [파라미터] 절 상수가 정본이다(사용자 지정 2026-11-19 — 공개 리포에 비밀번호가 보여도 된다, 파일을 따로 두지 않는다).
  한 번만 다르게 띄우려면 같은 이름의 환경변수로 덮는다(BROON_ESTOP_HOST · _PORT · _ID · _PW · _ROOM · _ROOM_PW).

전송 계층은 gold nxde/tts.py(= cheongbaek/domiman 의 domichat.py)에서 옮겼다 — 프레임 [길이 4B][종류 1B 'T'][UTF-8 JSON],
TLS 1.2 고정 · 재협상 금지, 자체 서명 인증서는 ★지문 고정★ — 첫 접속은 SERVER_FP_PREFIX(실서버 지문 앞부분)와 맞아야 하고,
그 지문을 ~/.broon_estop_server_fp.json 에 기억해 다음부터 같아야 붙는다(서버를 다시 세웠으면 둘 다 고친다).
서버(domiserver 261114a, xeon 확인): 보낸 말이 자기에게도 되돌아온다 · 방을 만들면 자동 입장 · 옛 대화는 다시 보내지 않는다 ·
연결당 10 초에 20건 · 15 초 ping (45 초 침묵이면 서버가 끊는다).
접속 뒤 읽기 타임아웃 60 s(서버가 15 s 마다 ping), 끊기면 1 · 2 · 5 · 10 · 30 s 간격으로 다시 붙는다.
★채팅이 끊겨도 /estop 은 마지막 상태를 그대로 유지한다★ (끊김을 정지로 보지 않는다 — 사용자가 정하지 않았다).
⚠️ 실제 서버로는 아직 붙어 보지 않았다 — 가짜 domichat 서버로만 확인했다.
"""

import hashlib
import json
import os
import queue
import signal
import socket
import ssl
import struct
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Int32, String

from broon.boards import ESTOP_TOPIC

# ══════════════════════════════════════════════════════════════════════════════
#  [파라미터] — 고칠 곳은 여기뿐이다
# ══════════════════════════════════════════════════════════════════════════════
SERVER_HOST = 'domiman.duckdns.org'    # domichat 서버 (domiserver)
SERVER_PORT = 47821
USER_ID = 'broon_hl_ws'                # 이 노드 전용 계정 (사용자 지정)
USER_PW = 'broon'
ROOM = 'domi_fishing_broon'            # domi_fishing_* = domiman 낚시방 규격 — 일반 목록에 안 보인다
ROOM_PW = 'domi_fishing_9714'          # 낚시방 비밀번호 규칙 (domiserver FISHING_ROOM_PW)
USE_TLS = True
SERVER_FP_PREFIX = '31c75366356acc30'  # 실서버 인증서 SHA-256 지문 앞부분 (xeon 세션 확인 2026-11-19) — 첫 접속도 이것과 맞아야 한다.
                                       # 서버 인증서를 다시 만들었으면 여기를 고치고 ~/.broon_estop_server_fp.json 을 지운다


def _env(name, default):
    v = os.environ.get('BROON_ESTOP_' + name, '').strip()
    return v if v else default


PIN_PATH = os.path.expanduser('~/.broon_estop_server_fp.json')

FRAME_HEAD = struct.Struct('>IB')
MAX_FRAME = 1024 * 1024
CONNECT_TIMEOUT = 6.0
READ_TIMEOUT = 60.0                       # 서버 ping 15 s — 이만큼 조용하면 죽은 연결
RECONNECT_BACKOFF = (1, 2, 5, 10, 30)
REPUBLISH_S = 1.0
WORDS = ('STOP', 'GO', 'STATUS')


def load_config():
    """접속 설정 dict — 위 상수, 환경변수가 있으면 그쪽."""
    return {'host': _env('HOST', SERVER_HOST), 'port': int(_env('PORT', str(SERVER_PORT))),
            'id': _env('ID', USER_ID), 'pw': _env('PW', USER_PW),
            'room': _env('ROOM', ROOM), 'room_pw': _env('ROOM_PW', ROOM_PW), 'tls': USE_TLS,
            'fp_prefix': _env('FP_PREFIX', SERVER_FP_PREFIX)}


# ══════════════════════════════════════════════════════════════════════════════
#  domichat 클라이언트 (gold nxde/tts.py 의 ChatClient 를 옮김 — 보내기만 더했다)
# ══════════════════════════════════════════════════════════════════════════════
class CertChanged(Exception):
    pass


def _tls_context():
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE               # 자체 서명 — 신뢰는 지문 고정이 맡는다
    ctx.minimum_version = ssl.TLSVersion.TLSv1_2
    ctx.maximum_version = ssl.TLSVersion.TLSv1_2  # 수신 · 송신 두 스레드가 한 소켓을 만진다 — 1.3 의 KeyUpdate 를 피한다
    no_reneg = getattr(ssl, 'OP_NO_RENEGOTIATION', 0)
    if no_reneg:
        ctx.options |= no_reneg
    return ctx


def _load_pin(key):
    try:
        with open(PIN_PATH) as f:
            return (json.load(f) or {}).get(key)
    except (OSError, ValueError):
        return None


def _save_pin(key, fp):
    data = {}
    try:
        with open(PIN_PATH) as f:
            data = json.load(f) or {}
    except (OSError, ValueError):
        pass
    data[key] = fp
    try:
        with open(PIN_PATH, 'w') as f:
            json.dump(data, f)
    except OSError:
        pass


class ChatClient:
    """domichat 세션 하나. 끊기면 스스로 다시 붙는다. 받은 프레임 · 사건('_ev')은 self.q 로."""

    def __init__(self, host, port, uid, pw, use_tls=True, log=print, fp_prefix=''):
        self.q = queue.Queue()
        self.host, self.port, self.uid, self.pw, self.use_tls = host, port, uid, pw, use_tls
        self.key = f"{host}:{port}"
        self.pinned = _load_pin(self.key)
        self.fp_prefix = (fp_prefix or '').lower()
        self.sock = None
        self.want = False
        self.log = log
        self._send_lock = threading.Lock()

    def _raw_send(self, sock, obj):
        data = json.dumps(obj, ensure_ascii=False).encode('utf-8')
        with self._send_lock:
            sock.sendall(FRAME_HEAD.pack(len(data), ord('T')) + data)

    @staticmethod
    def _recv_exact(sock, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return bytes(buf)

    def _recv_obj(self, sock):
        head = self._recv_exact(sock, FRAME_HEAD.size)
        if head is None:
            return None
        ln, typ = FRAME_HEAD.unpack(head)
        if ln > MAX_FRAME:
            raise OSError('프레임 과대')
        body = self._recv_exact(sock, ln) if ln else b''
        if body is None:
            return None
        if chr(typ) != 'T':
            return {}                              # 이미지 청크 — 쓰지 않는다
        return json.loads(body.decode('utf-8'))

    def _connect(self):
        raw = socket.create_connection((self.host, self.port), CONNECT_TIMEOUT)
        if not self.use_tls:
            return raw
        try:
            sock = _tls_context().wrap_socket(raw)
        except (ssl.SSLError, OSError) as e:
            raw.close()
            self.log(f"TLS 실패({e}) — 평문으로 다시 붙는다")
            return socket.create_connection((self.host, self.port), CONNECT_TIMEOUT)
        fp = hashlib.sha256(sock.getpeercert(binary_form=True)).hexdigest()
        if self.pinned and self.pinned != fp:
            sock.close()
            raise CertChanged(f"{self.key}: {self.pinned[:16]}… → {fp[:16]}…")
        if not self.pinned and self.fp_prefix and not fp.startswith(self.fp_prefix):
            sock.close()
            raise CertChanged(f"{self.key}: 알려진 지문 {self.fp_prefix}… 와 다르다 → {fp[:16]}…")
        if not self.pinned:
            self.pinned = fp
            _save_pin(self.key, fp)
            self.log(f"서버 인증서 지문을 기억했다: {fp[:16]}… ({PIN_PATH})")
        return sock

    def start(self):
        self.want = True
        threading.Thread(target=self._session_loop, daemon=True).start()

    def send(self, obj):
        sock = self.sock
        if sock is None:
            return False
        try:
            self._raw_send(sock, obj)
            return True
        except (OSError, ValueError):
            return False

    def stop(self):
        self.want = False
        sock, self.sock = self.sock, None
        if sock is not None:
            try:
                self._raw_send(sock, {'t': 'logout'})
            except (OSError, ValueError):
                pass
            try:
                sock.close()
            except OSError:
                pass

    def _session_loop(self):
        idx = 0
        while self.want:
            try:
                sock = self._connect()
            except CertChanged as e:
                self.want = False
                self.q.put({'_ev': 'cert_changed', 'msg': str(e)})
                return
            except OSError as e:
                self.q.put({'_ev': 'connect_fail', 'msg': str(e)})
                time.sleep(RECONNECT_BACKOFF[min(idx, len(RECONNECT_BACKOFF) - 1)])
                idx += 1
                continue
            sock.settimeout(READ_TIMEOUT)          # ★접속 타임아웃을 읽기 타임아웃으로 바꾼다★ (tts.py 헤더)
            try:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            except OSError:
                pass
            self.sock = sock
            self.q.put({'_ev': 'connected'})
            logged = False
            try:
                self._raw_send(sock, {'t': 'login', 'id': self.uid, 'pw': self.pw})
                while self.want:
                    d = self._recv_obj(sock)
                    if d is None:
                        break
                    if d.get('t') == 'ping':
                        self._raw_send(sock, {'t': 'pong'})
                        continue
                    if d.get('t') == 'welcome':
                        logged = True
                    self.q.put(d)
            except (OSError, ValueError):
                pass
            finally:
                try:
                    sock.close()
                except OSError:
                    pass
                self.sock = None
            if not self.want:
                break
            if logged:
                idx = 0
            self.q.put({'_ev': 'disconnected', 'msg': '연결이 끊겼다'})
            time.sleep(RECONNECT_BACKOFF[min(idx, len(RECONNECT_BACKOFF) - 1)])
            idx += 1


# ══════════════════════════════════════════════════════════════════════════════
#  노드
# ══════════════════════════════════════════════════════════════════════════════
class EstopNode(Node):
    def __init__(self, cfg=None):
        super().__init__('estop')
        cfg = cfg or load_config()
        problem = "" if cfg.get('id') and cfg.get('pw') else "계정(id · pw)이 비어 있다"
        self.cfg = cfg
        self.room = str(cfg['room'])
        self.uid = str(cfg.get('id') or '')
        self.active = False                 # True = E-STOP 발동
        self.link = 'off'                   # off · connecting · login · room · ready · error
        self.why = problem
        self.last_word = ''
        self.stop_evt = threading.Event()
        self.lock = threading.Lock()

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Int32, ESTOP_TOPIC, latched)
        self.pub_status = self.create_publisher(String, '/estop_status', 10)
        self.create_timer(REPUBLISH_S, self._tick)
        self._publish()
        self.client = None
        if problem:
            self.link = 'error'
            self.get_logger().error(f"🛑 E-STOP 채팅 연결 안 함 — {problem}. /estop 0 만 낸다")
            return
        self.client = ChatClient(cfg['host'], cfg['port'], self.uid, str(cfg['pw']), bool(cfg.get('tls', True)),
                                 log=lambda m: self.get_logger().info(m), fp_prefix=cfg.get('fp_prefix', ''))
        self.client.start()
        self.link = 'connecting'
        threading.Thread(target=self._worker, daemon=True).start()
        self.get_logger().info(f"🛑 E-STOP — {cfg['host']}:{cfg['port']} 계정 {self.uid} · 방 {self.room} (제한방) 구독, "
                               f"'STOP' → {ESTOP_TOPIC} 1 · 'GO' → 0 · 'STATUS' → 상태 회신. 지금 GO")

    # ── 상태 ─────────────────────────────────────────────────────────────────
    def _publish(self):
        self.pub.publish(Int32(data=1 if self.active else 0))

    def _tick(self):
        self._publish()
        self.pub_status.publish(String(data=self.status_text()))

    def status_text(self):
        state = 'STOP(발동)' if self.active else 'GO(해제)'
        link = {'off': '꺼짐', 'connecting': '접속 중', 'login': '로그인 중', 'room': '방 준비 중',
                'ready': f'방 {self.room} 구독 중', 'error': '오류'}.get(self.link, self.link)
        return f"E-STOP {state} | 채팅 {link}" + (f" | {self.why}" if self.why and self.link != 'ready' else "") \
            + (f" | 마지막 '{self.last_word}'" if self.last_word else "")

    def set_active(self, on, who):
        with self.lock:
            changed = on != self.active
            self.active = on
        self._publish()
        if changed:
            self.get_logger().warning(f"{'🛑 E-STOP 발동' if on else '✅ E-STOP 해제'} — {who} 의 '{'STOP' if on else 'GO'}'")

    def say(self, word):
        if self.client is not None and self.client.send({'t': 'msg', 'room': self.room, 'body': word}):
            return True
        self.get_logger().warning(f"방에 '{word}' 를 못 보냈다(연결 없음)", throttle_duration_sec=5.0)
        return False

    def state_word(self):
        return 'STOP' if self.active else 'GO'

    # ── 프레임 처리 (스레드) ──────────────────────────────────────────────────
    def _worker(self):
        while not self.stop_evt.is_set():
            try:
                d = self.client.q.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._on_frame(d)
            except Exception as e:     # noqa: BLE001 — 프레임 하나 때문에 노드를 죽이지 않는다
                self.get_logger().error(f"프레임 처리 실패: {e} — {d.get('t') or d.get('_ev')}")

    def _on_frame(self, d):
        ev = d.get('_ev')
        if ev == 'connected':
            self.link = 'login'
        elif ev in ('connect_fail', 'disconnected'):
            self.link, self.why = 'connecting', d.get('msg', '')
            self.get_logger().warning(f"채팅 서버 {ev} ({d.get('msg')}) — 다시 붙는다. /estop 은 지금 상태 유지",
                                      throttle_duration_sec=10.0)
        elif ev == 'cert_changed':
            self.link, self.why = 'error', (f"서버 인증서 지문이 맞지 않는다 {d.get('msg')} — 서버를 다시 세웠다면 "
                                            f"estop.py 의 SERVER_FP_PREFIX 를 고치고 {PIN_PATH} 를 지운다")
            self.get_logger().error(self.why)
        if ev:
            return
        t = d.get('t')
        if t == 'welcome':
            names = {r.get('name') for r in (d.get('rooms') or [])}
            self.link = 'room'
            if self.room in names:
                self.client.send({'t': 'join', 'room': self.room, 'pw': self.cfg['room_pw']})
            else:
                self.client.send({'t': 'room_create', 'name': self.room, 'kind': 'pw', 'pw': self.cfg['room_pw']})
                self.get_logger().info(f"채팅방 {self.room} 이 없다 — 제한방으로 만든다")
        elif t == 'joined' and d.get('room') == self.room:
            self.client.send({'t': 'sub', 'room': self.room, 'on': True})
            self.link, self.why = 'ready', ''
            self.get_logger().info(f"채팅방 {self.room} 입장 · 구독 — 지금 상태 '{self.state_word()}' 를 알린다")
            self.say(self.state_word())
        elif t == 'denied' and d.get('room') == self.room:
            self.link, self.why = 'error', f"방 입장 거절 ({d.get('reason')}) {d.get('msg') or ''}"
            self.get_logger().error(self.why + " — 방 비밀번호(room_pw) 확인")
        elif t == 'room_deleted' and d.get('room') == self.room:
            self.link = 'room'
            self.get_logger().warning(f"채팅방 {self.room} 이 지워졌다 — 다시 만든다")
            self.client.send({'t': 'room_create', 'name': self.room, 'kind': 'pw', 'pw': self.cfg['room_pw']})
        elif t == 'kicked' and d.get('room') == self.room:
            self.link, self.why = 'error', "방에서 강제 퇴장됐다"
            self.get_logger().error(self.why)
        elif t == 'msg' and d.get('room') == self.room:
            self._on_msg(str(d.get('from') or ''), str(d.get('body') or ''))
        elif t == 'error':
            code = d.get('code')
            if code == 'room_name_taken':
                self.client.send({'t': 'join', 'room': self.room, 'pw': self.cfg['room_pw']})
            else:
                self.why = f"서버 오류 ({code}) {d.get('msg') or ''}"
                self.get_logger().error(self.why + (" — 계정(id · pw) · 승인 상태 확인"
                                                    if code in ('bad_login', 'already_online', 'disabled') else ""))

    def _on_msg(self, who, body):
        if who == self.uid:
            return                              # 자기가 보낸 말 — 되먹임을 막는다
        word = body.strip().upper()
        if word not in WORDS:
            return
        self.last_word = f"{word} ← {who}"
        if word == 'STOP':
            self.set_active(True, who)
        elif word == 'GO':
            self.set_active(False, who)
        self.say(self.state_word())

    def shutdown(self):
        self.stop_evt.set()
        if self.client is not None:
            self.client.stop()


def main(args=None):
    rclpy.init(args=args)
    node = EstopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
