#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ntrip.py ― 국토지리정보원(NGII) VRS NTRIP 클라이언트 (tool/gps_mapper.py → white/gps.py 의 것 그대로)
════════════════════════════════════════════════════════════════════════════════
PC 인터넷으로 RTCM3 보정을 받아 sink(bytes) 로 넘긴다 — gps 노드가 그것을 수신기(들)에 쓴다.
VRS 는 올린 GGA 자리에 가상 기준국을 만들므로 캐스터에 수신기 GGA 를 10 초마다 올린다.
접속 정보는 tool/info.md 와 같다. 듀얼이어도 접속은 하나다(받은 보정을 두 수신기에 똑같이 넣는다).
"""

import base64
import socket
import threading
import time

NTRIP = {
    "host": "rts1.ngii.go.kr",
    "port": 2101,
    "mountpoint": "VRS-RTCM34",
    "user": "knut_gps",
    "password": "ngii",
}

GGA_UPLOAD_SEC = 10          # VRS 캐스터로 현재 위치(GGA)를 올리는 주기
NTRIP_RETRY_SEC = 5          # 연결이 끊기면 이만큼 쉬고 다시 접속
NTRIP_FATAL_RETRY_SEC = 30   # 인증 실패 등은 길게 쉬고 다시 시도
NTRIP_NO_DATA_SEC = 20       # 접속 중 이 시간 동안 보정이 안 오면 끊고 다시 접속


# ══════════════════════════════════════════════════════════════════════════════
#  NTRIP (gps_mapper.py — 기지국 해독 RtcmMonitor 는 뺐다)
# ══════════════════════════════════════════════════════════════════════════════
class NtripFatal(Exception):
    pass


class NtripClient(threading.Thread):
    """NGII 캐스터 접속을 유지한다. 받은 RTCM3 는 sink(bytes) 로 넘긴다(시리얼 쓰기).

    status : off / wait_pos(위치 대기) → connecting → streaming, 실패하면 retry / fatal 후 다시 시도.
    """

    def __init__(self, cfg, sink, log):
        super().__init__(daemon=True)
        self.cfg, self.sink, self.log = cfg, sink, log
        self.sock = None
        self.gga_raw = None
        self.enabled = True
        self.status = "wait_pos"
        self.detail = ""
        self.bytes_in = 0
        self.last_data_t = 0.0
        self.stop_evt = threading.Event()
        self.wake = threading.Event()

    def stop(self):
        self.stop_evt.set()
        self.wake.set()
        self._close()

    def _sleep(self, sec):
        self.wake.wait(sec)
        self.wake.clear()

    def _set(self, status, detail=""):
        if status != self.status or detail != self.detail:
            if status in ("streaming", "retry", "fatal"):
                self.log(f"NTRIP {status} {detail}".rstrip())
        self.status, self.detail = status, detail

    def run(self):
        while not self.stop_evt.is_set():
            if not self.enabled:
                self._set("off")
                self._sleep(1.0)
                continue
            if not self.gga_raw:
                self._set("wait_pos")
                self._sleep(0.5)
                continue
            self._set("connecting")
            wait = NTRIP_RETRY_SEC
            try:
                self._stream(self._connect())
            except NtripFatal as e:
                self._set("fatal", str(e))
                wait = NTRIP_FATAL_RETRY_SEC
            except OSError as e:
                self._set("retry", str(e) or type(e).__name__)
            self._close()
            if self.stop_evt.is_set():
                return
            self._sleep(wait)

    def _connect(self):
        c = self.cfg
        auth = base64.b64encode(f"{c['user']}:{c['password']}".encode()).decode()
        req = (f"GET /{c['mountpoint']} HTTP/1.0\r\n"
               f"Host: {c['host']}\r\n"
               f"User-Agent: NTRIP white/1.0\r\n"
               f"Authorization: Basic {auth}\r\n"
               f"Accept: */*\r\n"
               f"Connection: close\r\n\r\n")
        sock = socket.create_connection((c["host"], c["port"]), timeout=10)
        self.sock = sock
        sock.sendall(req.encode())
        head = b""
        while b"\r\n" not in head:
            chunk = sock.recv(1024)
            if not chunk:
                raise OSError("응답 없이 연결 종료")
            head += chunk
        first, _, rest = head.partition(b"\r\n")
        first = first.decode("latin1").strip()
        if first.startswith("ICY 200"):
            pass
        elif first.startswith("HTTP/") and " 200" in first:
            rest = b"\r\n" + rest
            while b"\r\n\r\n" not in rest:
                chunk = sock.recv(1024)
                if not chunk:
                    raise OSError("헤더 수신 중 연결 종료")
                rest += chunk
            rest = rest.split(b"\r\n\r\n", 1)[1]
        elif "SOURCETABLE" in first:
            raise NtripFatal(f"마운트포인트 '{c['mountpoint']}' 없음")
        elif "401" in first:
            raise NtripFatal("인증 실패(401) — ID 확인, 다른 곳에서 같은 ID로 접속 중인지 확인")
        else:
            raise NtripFatal(f"예상 밖 응답: {first}")
        self._set("streaming", c["mountpoint"])
        self.last_data_t = time.time()
        self._send_gga(sock)
        if rest:
            self._feed(rest)
        return sock

    def _stream(self, sock):
        sock.settimeout(1.0)
        last_gga = time.time()
        while not self.stop_evt.is_set() and self.enabled:
            now = time.time()
            if now - last_gga >= GGA_UPLOAD_SEC:
                self._send_gga(sock)
                last_gga = now
            if now - self.last_data_t > NTRIP_NO_DATA_SEC:
                raise OSError(f"{NTRIP_NO_DATA_SEC}초 동안 보정 데이터 없음")
            try:
                data = sock.recv(4096)
            except socket.timeout:
                continue
            if not data:
                raise OSError("캐스터가 연결을 닫음")
            self._feed(data)

    def _send_gga(self, sock):
        if self.gga_raw:
            sock.sendall((self.gga_raw + "\r\n").encode("ascii"))

    def _feed(self, data):
        self.bytes_in += len(data)
        self.last_data_t = time.time()
        self.sink(data)

    def _close(self):
        s, self.sock = self.sock, None
        if s:
            try:
                s.close()
            except OSError:
                pass
