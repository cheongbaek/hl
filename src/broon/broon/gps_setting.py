#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gps_setting.py ― 전방 · 후방 GPS 배치 설정 (tkinter GUI)
════════════════════════════════════════════════════════════════════════════════
    ros2 run broon gps_setting

    전방 GPS : [ 블록 ]
    후방 GPS : [ 블록 ]
    = 연결된 GPS =
      54401a3a98                         ← 칩 고유 ID (큰 글씨)
      U-Blox AG u-blox GNSS receiver  ttyACM0   ← lsusb 에 나오는 이름 + 포트 (작은 글씨)
                                 [저장]

F9P 두 대는 USB 상으로 똑같아 ★칩 고유 ID★ 로만 가를 수 있다. 어느 ID 를 전방 · 후방에 달았는지를
배치 설정 파일(gpsdev.config_path() — 보통 <ws>/src/broon/config/gps_setting.json)에 남기고,
gps 노드(ros2 run broon gps)와 check 가 그 파일을 읽어 수신기를 고른다.

화면
  · 열면 설정 파일을 읽어 지금 배치를 위 두 칸에 보인다.
  · 꽂혀 있는 u-blox 포트를 하나씩 열어 ID 를 읽는다(설정은 바꾸지 않는다). 전방 · 후방에 이미 지정된
    수신기는 아래 '연결된 GPS' 가 아니라 ★위 칸에★ 블록으로 보인다.
  · 지정돼 있는데 지금 꽂혀 있지 않은 수신기는 ★기울임 + 취소선★ 으로 보인다.
  · 블록을 마우스(터치)로 끌어 위 칸에 놓으면 그 수신기로 바뀐다 — 칸에 있던 것은 대체된다.
    위 칸의 블록을 다른 칸에 놓으면 둘이 맞바뀌고, '연결된 GPS' 쪽에 놓으면 그 칸이 비워진다.
  · '연결된 GPS' 는 한 번에 4줄까지 보이고 더 많으면 스크롤한다.
  · ★[저장] 을 눌러야★ 설정 파일이 바뀐다. 누르지 않고 닫으면 파일은 그대로다.

포트를 다른 프로그램(gps 노드 · gps_mapper)이 쥐고 있으면 ID 를 읽지 못한다 — '사용 중' 으로 보인다.
one_launch 를 끄고 쓸 것. ROS 노드가 아니다(rclpy 를 쓰지 않는다).
"""

import argparse
import queue
import sys
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from tkinter import messagebox, ttk

import serial

from broon import gpsdev
from broon.gpsdev import ROLE_KO, ROLES

SCAN_SEC = 1.5               # 포트 목록을 다시 보는 주기
RETRY_SEC = 5.0              # ID 를 못 읽은 포트(사용 중 등)를 다시 열어 보는 간격
LIST_ROWS = 4                # '연결된 GPS' 에 한 번에 보이는 줄 수
DRAG_PX = 6                  # 이만큼 움직여야 끌기로 본다 (그냥 누른 것과 구분)
FONT_CANDIDATES = ("Malgun Gothic", "Noto Sans CJK KR", "NanumGothic", "UnDotum")

BG, SLOT_BG, SLOT_HOT = "#f4f5f7", "#ffffff", "#d6e9ff"
BLOCK_BG, BLOCK_EDGE, GHOST_BG = "#ffffff", "#9aa5b1", "#1c7ed6"
FG, FG_SUB, FG_OFF, FG_HINT = "#212529", "#495057", "#868e96", "#adb5bd"


class Scanner(threading.Thread):
    """u-blox 포트를 찾아 칩 ID 를 읽는다. 결과(포트 목록 스냅샷)를 큐로 넘긴다."""

    def __init__(self, out):
        super().__init__(daemon=True)
        self.out = out
        self.stop_evt = threading.Event()
        self.cache = {}              # 포트 → dict(id, err, t)

    def snapshot(self, ports):
        rows = []
        for p in ports:
            c = self.cache.get(p.device)
            rows.append(dict(device=p.device, short=p.short, name=p.name,
                             id=c["id"] if c else "", err=c["err"] if c else "ID 읽는 중…"))
        return rows

    def run(self):
        while not self.stop_evt.is_set():
            ports = gpsdev.list_ublox_ports()
            alive = {p.device for p in ports}
            self.cache = {d: c for d, c in self.cache.items() if d in alive}
            self.out.put(self.snapshot(ports))
            for p in ports:
                if self.stop_evt.is_set():
                    return
                c = self.cache.get(p.device)
                if c and (c["id"] or time.time() - c["t"] < RETRY_SEC):
                    continue
                ser, why = gpsdev.open_port(p.device)
                chip = ""
                if ser is not None:
                    try:
                        chip = gpsdev.read_chip_id(ser)
                        why = "" if chip else "칩 ID 응답 없음"
                    except (serial.SerialException, OSError) as e:
                        why = f"읽기 실패: {e}"
                    finally:
                        try:
                            ser.close()
                        except (serial.SerialException, OSError):
                            pass
                self.cache[p.device] = dict(id=chip, err=why, t=time.time())
                self.out.put(self.snapshot(ports))
            self.stop_evt.wait(SCAN_SEC)


class App:
    def __init__(self, root, path=None, scan=True):
        self.root = root
        self.path = path or gpsdev.config_path()
        saved = gpsdev.load_setting(self.path)
        self.saved = {r: saved[r]["id"] for r in ROLES}        # 파일에 있는 배치
        self.assign = dict(self.saved)                         # 화면의 배치 (저장 전)
        self.names = {saved[r]["id"]: saved[r]["name"] for r in ROLES if saved[r]["id"]}
        self.conn = {}               # 칩 ID → dict(name, short) — 지금 꽂혀 있고 ID 를 읽은 것
        self.unread = []             # ID 를 못 읽은 포트 [dict(short, name, err)]
        self.scanned = False
        self.drag = None
        self._render_later = False
        self.note = ""

        fam = next((f for f in FONT_CANDIDATES if f in set(tkfont.families(root))),
                   tkfont.nametofont("TkDefaultFont").actual("family"))
        self.f_label = tkfont.Font(family=fam, size=12, weight="bold")
        self.f_id = tkfont.Font(family=fam, size=15, weight="bold")
        self.f_sub = tkfont.Font(family=fam, size=9)
        self.f_id_off = tkfont.Font(family=fam, size=15, weight="bold", slant="italic", overstrike=1)
        self.f_sub_off = tkfont.Font(family=fam, size=9, slant="italic", overstrike=1)
        self.f_hint = tkfont.Font(family=fam, size=9)
        self.row_h = self.f_id.metrics("linespace") + self.f_sub.metrics("linespace") + 16

        root.title("GPS 배치 설정 — broon")
        root.configure(bg=BG)
        root.minsize(460, 0)
        self._build()
        self.render()
        root.protocol("WM_DELETE_WINDOW", self.close)

        self.q = queue.Queue()
        self.scanner = Scanner(self.q) if scan else None
        if self.scanner:
            self.scanner.start()
            root.after(150, self._poll)

    # ── 구성 ─────────────────────────────────────────────────────────────────
    def _build(self):
        top = tk.Frame(self.root, bg=BG, padx=14, pady=12)
        top.pack(fill="x")
        top.columnconfigure(1, weight=1)
        self.slot = {}
        for i, role in enumerate(ROLES):
            tk.Label(top, text=f"{ROLE_KO[role]} GPS :", font=self.f_label, bg=BG, fg=FG).grid(
                row=i, column=0, sticky="w", pady=4, padx=(0, 10))
            box = tk.Frame(top, bg=SLOT_BG, height=self.row_h + 6, highlightthickness=2,
                           highlightbackground=BLOCK_EDGE, highlightcolor=BLOCK_EDGE)
            box.grid(row=i, column=1, sticky="ew", pady=4)
            box.pack_propagate(False)
            self.slot[role] = box

        tk.Label(self.root, text="= 연결된 GPS =", font=self.f_label, bg=BG, fg=FG).pack(anchor="w", padx=14)
        wrap = tk.Frame(self.root, bg=BG, padx=14, pady=4)
        wrap.pack(fill="x")
        self.canvas = tk.Canvas(wrap, bg=BG, height=LIST_ROWS * self.row_h, highlightthickness=0)
        self.scroll = ttk.Scrollbar(wrap, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.scroll.set)
        self.canvas.pack(side="left", fill="x", expand=True)
        self.inner = tk.Frame(self.canvas, bg=BG)
        self.inner_id = self.canvas.create_window(0, 0, window=self.inner, anchor="nw")
        self.canvas.bind("<Configure>", lambda e: self.canvas.itemconfigure(self.inner_id, width=e.width))
        for seq, step in (("<Button-4>", -1), ("<Button-5>", 1)):
            self.root.bind_all(seq, lambda e, s=step: self._wheel(s))
        self.root.bind_all("<MouseWheel>", lambda e: self._wheel(-1 if e.delta > 0 else 1))

        bottom = tk.Frame(self.root, bg=BG, padx=14, pady=10)
        bottom.pack(fill="x")
        self.status = tk.Label(bottom, text="", font=self.f_hint, bg=BG, fg=FG_SUB, anchor="w", justify="left")
        self.status.pack(side="left", fill="x", expand=True)
        self.save_btn = ttk.Button(bottom, text="저장", command=self.save)
        self.save_btn.pack(side="right")

    def _wheel(self, step):
        if self.scroll.winfo_ismapped():
            self.canvas.yview_scroll(step, "units")

    # ── 블록 ─────────────────────────────────────────────────────────────────
    def _block(self, parent, gid, src):
        """ID + 정보 한 블록. src = 'front' | 'rear' | 'list' (끌기 시작한 자리)."""
        on = gid in self.conn
        if on:
            sub = f"{self.conn[gid]['name']}   {self.conn[gid]['short']}"
        else:
            sub = f"{self.names.get(gid) or '이름 모름'}   (연결 안 됨)"
        fr = tk.Frame(parent, bg=BLOCK_BG, highlightthickness=1, highlightbackground=BLOCK_EDGE, cursor="fleur")
        a = tk.Label(fr, text=gid, font=self.f_id if on else self.f_id_off, bg=BLOCK_BG,
                     fg=FG if on else FG_OFF, anchor="w")
        b = tk.Label(fr, text=sub, font=self.f_sub if on else self.f_sub_off, bg=BLOCK_BG,
                     fg=FG_SUB if on else FG_OFF, anchor="w")
        a.pack(fill="x", padx=8, pady=(4, 0))
        b.pack(fill="x", padx=8, pady=(0, 4))
        for w in (fr, a, b):
            w.bind("<ButtonPress-1>", lambda e, g=gid, s=src: self._press(e, g, s))
            w.bind("<B1-Motion>", self._motion)
            w.bind("<ButtonRelease-1>", self._release)
        return fr

    def render(self):
        if self.drag is not None:            # 끄는 중에 블록을 갈아 치우면 끌기가 끊긴다 — 놓은 뒤에
            self._render_later = True
            return
        self._render_later = False
        for role in ROLES:
            box = self.slot[role]
            for w in box.winfo_children():
                w.destroy()
            gid = self.assign[role]
            if gid:
                self._block(box, gid, role).pack(fill="both", expand=True, padx=2, pady=2)
            else:
                tk.Label(box, text="비어 있음 — 아래 블록을 끌어다 놓으세요", font=self.f_hint, bg=SLOT_BG,
                         fg=FG_HINT).pack(fill="both", expand=True)
        for w in self.inner.winfo_children():
            w.destroy()
        free = [g for g in sorted(self.conn, key=lambda g: self.conn[g]["short"]) if g not in self.assign.values()]
        for gid in free:
            self._block(self.inner, gid, "list").pack(fill="x", pady=2)
        for u in self.unread:
            tk.Label(self.inner, text=f"{u['short']}   {u['name']} — {u['err']}", font=self.f_hint, bg=BG,
                     fg=FG_OFF, anchor="w").pack(fill="x", pady=4)
        if not free and not self.unread:
            text = "지정되지 않은 GPS 가 없습니다" if self.scanned else "GPS 를 찾는 중…"
            tk.Label(self.inner, text=text, font=self.f_hint, bg=BG, fg=FG_HINT, anchor="w").pack(fill="x", pady=4)
        self.inner.update_idletasks()
        self.canvas.configure(scrollregion=(0, 0, 0, self.inner.winfo_reqheight()))
        if self.inner.winfo_reqheight() > LIST_ROWS * self.row_h:
            self.scroll.pack(side="right", fill="y")
        else:
            self.scroll.pack_forget()
            self.canvas.yview_moveto(0)
        dirty = self.assign != self.saved
        self.save_btn.state(["!disabled"] if dirty else ["disabled"])
        self.status.configure(text=("● 저장하지 않은 변경이 있습니다 — [저장] 을 눌러야 반영됩니다" if dirty
                                    else (self.note or f"설정 파일  {self.path}")),
                              fg="#c92a2a" if dirty else FG_SUB)

    # ── 포트 탐색 결과 ───────────────────────────────────────────────────────
    def _poll(self):
        rows = None
        try:
            while True:
                rows = self.q.get_nowait()
        except queue.Empty:
            pass
        if rows is not None:
            self.apply_scan(rows)
        self.root.after(150, self._poll)

    def apply_scan(self, rows):
        """포트 목록 스냅샷 [dict(device, short, name, id, err)] 을 화면에 반영한다."""
        conn = {r["id"]: dict(name=r["name"], short=r["short"]) for r in rows if r["id"]}
        unread = [dict(short=r["short"], name=r["name"], err=r["err"]) for r in rows if not r["id"]]
        for gid, c in conn.items():
            self.names[gid] = c["name"]
        self.scanned = True
        if conn != self.conn or unread != self.unread:
            self.conn, self.unread = conn, unread
            self.render()

    # ── 끌어다 놓기 ──────────────────────────────────────────────────────────
    def _press(self, e, gid, src):
        self.drag = dict(gid=gid, src=src, x0=e.x_root, y0=e.y_root, ghost=None)

    def _motion(self, e):
        d = self.drag
        if d is None:
            return
        if d["ghost"] is None:
            if abs(e.x_root - d["x0"]) + abs(e.y_root - d["y0"]) < DRAG_PX:
                return
            g = tk.Toplevel(self.root)
            g.overrideredirect(True)
            for key, val in (("-topmost", True), ("-alpha", 0.85)):
                try:
                    g.attributes(key, val)
                except tk.TclError:
                    pass
            tk.Label(g, text=d["gid"], font=self.f_id, bg=GHOST_BG, fg="white", padx=12, pady=6).pack()
            d["ghost"] = g
        d["ghost"].geometry(f"+{e.x_root + 14}+{e.y_root + 10}")
        self._highlight(self.target_at(e.x_root, e.y_root))

    def _release(self, e):
        d, self.drag = self.drag, None
        if d is None:
            return
        self._highlight(None)
        if d["ghost"] is not None:
            d["ghost"].destroy()
            self.drop(d["gid"], d["src"], self.target_at(e.x_root, e.y_root))
        elif self._render_later:
            self.render()

    @staticmethod
    def _inside(w, x, y):
        return (w.winfo_rootx() <= x < w.winfo_rootx() + w.winfo_width()
                and w.winfo_rooty() <= y < w.winfo_rooty() + w.winfo_height())

    def target_at(self, x, y):
        """화면 좌표 → 'front' | 'rear' | 'list' | None"""
        for role in ROLES:
            if self._inside(self.slot[role], x, y):
                return role
        return "list" if self._inside(self.canvas, x, y) else None

    def _highlight(self, target):
        for role in ROLES:
            self.slot[role].configure(bg=SLOT_HOT if role == target else SLOT_BG,
                                      highlightbackground=GHOST_BG if role == target else BLOCK_EDGE)

    def drop(self, gid, src, target):
        """블록 gid 를 src 에서 target 에 놓았다."""
        if target in ROLES and target != src:
            if src in ROLES:
                self.assign[src] = self.assign[target]       # 칸 → 칸 : 맞바꾼다
            self.assign[target] = gid                        # 목록 → 칸 : 있던 것은 대체된다
        elif target == "list" and src in ROLES:
            self.assign[src] = ""                            # 칸 → 목록 : 그 칸을 비운다
        self.note = ""
        self.render()

    # ── 저장 · 닫기 ──────────────────────────────────────────────────────────
    def save(self):
        setting = {r: {"id": self.assign[r], "name": self.names.get(self.assign[r], "")} for r in ROLES}
        try:
            path = gpsdev.save_setting(setting, self.path)
        except OSError as e:
            messagebox.showerror("저장 실패", f"설정 파일을 쓸 수 없습니다.\n{e}")
            return
        self.saved = dict(self.assign)
        self.note = f"저장했습니다  {path}"
        self.render()

    def close(self):
        if self.scanner:
            self.scanner.stop_evt.set()
        self.root.destroy()


def main(args=None):
    ap = argparse.ArgumentParser(prog="ros2 run broon gps_setting", description="전방 · 후방 GPS 배치 설정")
    ap.add_argument("--file", default="", help="배치 설정 파일 (기본: gpsdev.config_path())")
    opt, _ = ap.parse_known_args(args if args is not None else sys.argv[1:])
    root = tk.Tk()
    App(root, opt.file or None)
    root.mainloop()


if __name__ == "__main__":
    main()
