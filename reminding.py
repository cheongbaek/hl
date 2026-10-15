#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
reminding.py ― 고정점 검증 데이터 분석 (gps_check_data · matplotlib · tkinter)
════════════════════════════════════════════════════════════════════════════════
gps_mapper.py 검증 모드가 남긴 gps_check_data/point_*.csv 를 전부 읽어, 수신기를 한 자리에 고정해
둔 동안 위치가 원점에서 얼마나 벗어났는지를 matplotlib 창으로 본다. 지도 대신 동/북 [mm] 평면에
일정 mm 격자를 깔고, 원점에 가까운 점은 빨강 · 멀어질수록 파랑으로 칠한다.

왼쪽 목록에서 CSV 를
  하나 고르면 ─ 단독 분석 : 세션 원점 기준 점 분포 + 시간에 따른 원점 거리 · 동/북 오프셋 · 튐 · RTK 상태.
                 아래 칸에 통계 · RTK 구간 · 고른 점 정보.
  여럿 고르면 ─ 세션 비교 : 고른 세션 전체의 Fixed 최빈점을 원점으로 모든 점과 세션별 원점을 함께
                 그리고, 세션 원점이 서로 얼마나 어긋나는지 · Fixed 를 얼마 만에 잡았는지 비교한다.

시간 슬라이더 : 그림 아래 슬라이더(또는 ▶ 재생, ← → 키)로 시각을 옮기면 그 시각의 점 · 최근 궤적(꼬리)이
  평면 위에서 움직인다. 단독에서는 그 시각의 추세 위치(◇)와, 점과 추세를 잇는 점선도 함께 움직인다 —
    점만 혼자 튀었다 돌아오고 ◇ 는 제자리  → 순간 튐
    ◇ 자체가 원점에서 천천히 멀어짐          → 서서히 생기는 편향
  비교에서는 모든 세션의 '기록 시작 후 같은 시각' 점이 함께 움직인다.

튐 · 편향 : 같은 RTK 구간 안에서 앞뒤 '추세 창' 초의 이동 중앙값을 추세(천천히 움직이는 성분)로 본다.
  중앙값이라 한두 점이 튀어도 추세가 끌려가지 않는다. 구간이 바뀌는 순간(Float→Fixed)은 상태 변화라
  구간을 넘어 섞지 않는다.
  · 튐 : 점이 추세에서 '튐 기준' mm 넘게 벗어난 것. 이어진 튐 점은 1회로 센다. 추세 창의 절반보다 오래
         벗어나 있으면 추세가 따라가므로 튐이 아니라 편향으로 잡힌다.
  · 편향 : 추세가 원점에서 떨어진 양. '안정 기준' mm 를 넘으면 편향.
  · 판정 : 튐 없음 + 편향 → 서서히 편향 · 튐 + 편향 없음 → 순간 튐 · 둘 다 → 편향 + 튐 · 둘 다 없음 → 안정.

좌표 해상도 : CSV 는 소수 8자리로 쓰지만 NAV-PVT 위경도가 1e-7° 단위라, 점은 실제로 북 1.1 cm ·
  동 0.9 cm 간격 격자 위에만 놓인다. 1 cm 아래 차이 · 격자는 의미가 없고, Fixed 의 '1회 이동' 은
  대부분 0 아니면 이 한 칸이다.

원점(최빈점) : "반경 r 안 이웃이 가장 많은 점" 을 찾고 그 이웃들의 평균을 원점으로 삼는다(r 은 창에서
  조절). 기본 5 mm 는 격자 한 칸보다 작아 ★정확히 같은 좌표가 가장 많이 나온 점★이 되고, r 을 키우면
  이웃 격자까지 묶은 밀집 중심이 된다.
  · 단독 : 그 세션의 Fixed 점으로 구한 최빈점. 세션에 Fixed 가 하나도 없으면 폴더 기준(아래)을 쓴다.
  · 비교 : 고른 세션들의 Fixed 점을 모두 모아 구한 최빈점. 세션마다 자기 최빈점도 하나씩 찍는다
           (Fixed 가 없는 세션은 Float 점으로 구한 최빈점 — 표에 'Float' 로 표시, 막대에 빗금).
  · 폴더 기준 : 폴더의 모든 CSV 의 Fixed 점으로 구한 최빈점. 모든 세션을 같은 자리에서 쟀으므로
           단독 화면에서 참고점(흰 +)으로 찍는다.

용어
  Fixed 진입 : 기록 시작부터 처음 RTK Fixed 가 된 행까지의 시간. ★USB 를 꽂은 시각이 아니라 기록을
               시작한 시각 기준★이고, 측위가 안 된 동안은 기록되지 않으므로 실제보다 짧게 나온다.
  안정화     : 그 시점 이후로 끝까지 모든 점이 세션 원점에서 '안정 기준' mm 안에 머문 첫 시각.
               끝까지 못 들어오면 '없음'.
  이탈       : Fixed 였다가 Fixed 가 아닌 상태로 떨어진 횟수.
  드리프트   : Fixed 점들의 동/북 위치를 시간에 직선 맞춤한 기울기 [mm/분] — 지터가 아니라 천천히 흐르는 양.
  추세 속도  : 추세가 추세 창 길이 동안 옮겨 간 거리 [mm/분]. 그중 가장 빠른 값을 표에 낸다.
  진입 점프  : 구간이 바뀌는 순간 직전 점에서 몇 mm 를 건너뛰었나(Float→Fixed 전환 때 크다).
  퍼짐 / 오차 : 세션 원점 기준 거리(정밀도) / 전체 원점 기준 거리(접속마다 달라지는 Fixed 해까지 포함).
  실제/hAcc  : 원점 거리 ÷ 수신기가 스스로 낸 수평 정확도. 1 보다 훨씬 크면 수신기가 정확도를 과신한 것.

시각 : CSV 에 시각 열이 없어 파일 이름의 시작 시각 + 행 번호 × 간격으로 역산한다. 측위가 끊긴 동안은
  기록을 건너뛰므로 그 뒤부터는 실제 시각과 어긋날 수 있다.

조작 : 점을 클릭하면 슬라이더가 그 점으로 가고 정보가 나온다(시간 그래프를 클릭하면 그 시각의 점).
  좌표 평면은 지도처럼 — 끌면 이동, 휠로 커서 자리를 중심으로 확대/축소(제자리에서 누르고 떼야 점 고르기).
  시간 그래프도 휠로 가로 확대/축소. 아래 도구 막대로 처음 보기 · 뒤로 · 그림 저장. 그림 위에서 ← → 는 한 행씩,
  스페이스는 재생/정지. 비교 표의 행(또는 막대)을 누르면 그 세션만 강조하고, 표의 행을 두 번 누르면
  그 세션을 단독 분석으로 연다.

Windows · Ubuntu : 같은 파일이 두 OS 에서 돈다. 필요한 패키지 —
  Ubuntu  : sudo apt install python3-tk python3-numpy python3-matplotlib
  Windows : pip install numpy matplotlib
"""

import csv
import math
import re
import sys
import time
import unicodedata
from datetime import datetime, timedelta
from pathlib import Path
from types import SimpleNamespace
import tkinter as tk
import tkinter.font as tkfont
from tkinter import ttk

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view
import matplotlib
matplotlib.use("TkAgg")
from matplotlib import font_manager
from matplotlib import ticker as mticker
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.colors import LinearSegmentedColormap, LogNorm, Normalize, to_hex, to_rgb
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Patch


APP_DIR = Path(__file__).resolve().parent
CHECK_DIR = APP_DIR / "gps_check_data"
# point_dual_* (gps_mapper_dual.py) 도 받는다 — 앞 17열이 같아 1번 GPS 값으로 읽힌다
FNAME_RE = re.compile(r"point_(dual_)?(\d{8}_\d{6})_([\d.]+)s\.csv$")

# ── OS 분기 ───────────────────────────────────────────────────────────────────────
IS_WINDOWS = sys.platform == "win32"
FONT_CANDIDATES = ("Malgun Gothic", "Noto Sans CJK KR", "NanumGothic", "UnDotum")
MONO_CANDIDATES = ("D2Coding", "NanumGothicCoding", "Noto Sans Mono CJK KR", "GulimChe", "굴림체",
                   "Consolas", "DejaVu Sans Mono")
# matplotlib 은 .ttc 의 첫 글꼴만 읽어 Ubuntu 의 Noto CJK 가 'JP' 이름으로만 잡힌다 — 한글 글리프는 들어 있다
MPL_FONT_CANDIDATES = ("Malgun Gothic", "Noto Sans CJK KR", "NanumGothic", "Noto Sans CJK JP", "UnDotum")

# ── 표시 ──────────────────────────────────────────────────────────────────────────
FIXED, FLOAT = 4, 5                      # NMEA GGA 품질 번호
Q_NAME = {0: "없음", 1: "단독", 2: "DGNSS", 4: "Fixed", 5: "Float", 6: "추측"}
Q_COLOR = {FIXED: "#2ca02c", FLOAT: "#ff7f0e"}
Q_MARK = {FIXED: "o", FLOAT: "^"}
OTHER_COLOR, OTHER_MARK = "#7f7f7f", "x"

CMAP = LinearSegmentedColormap.from_list("near_far", ["#e41a1c", "#984ea3", "#1f5fe0"])  # 가까움 → 멂
CMAP.set_bad("#e41a1c")                  # 로그 스케일에서 거리 0 은 '원점 그 자리' — 빨강
ORIGIN_COLOR = "#e41a1c"
SPIKE_COLOR = "#e41a1c"
SIDE_COLOR = {1: "#2f6fe0", 2: "#14a37f"}  # 듀얼 1번 · 2번 (find_wc 의 파랑 · 민트 계열)
SIDE_MARK = {1: "o", 2: "s"}               # 듀얼 평면 — 1번 ●, 2번 ■
EAST_COLOR, NORTH_COLOR = "#8c564b", "#17a2b8"
GRID_CHOICES = ("1", "2", "5", "10", "20", "50", "100", "200", "500", "1000")
SPEED_CHOICES = ("1", "2", "5", "10", "20", "50", "100")
MAX_GRID_LINES = 80                      # 격자선이 이보다 많아지면 설정 간격의 2·5·10… 배로 성기게
PICK_PX = 15                             # 클릭이 점에서 이만큼(픽셀) 안이면 그 점을 고른다
DRAG_PX = 5                              # 평면에서 누른 채 이만큼(픽셀) 넘게 움직이면 클릭이 아니라 끌어 옮기기
PLAY_MS = 40                             # 재생 화면 갱신 간격

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


# ── 좌표 ──────────────────────────────────────────────────────────────────────────
def _radii(lat0):
    """위도 lat0 에서 자오선 · 묘유선 곡률 반경 [m]"""
    s = 1 - WGS84_E2 * math.sin(math.radians(lat0)) ** 2
    return WGS84_A * (1 - WGS84_E2) / s ** 1.5, WGS84_A / math.sqrt(s)


def enu_mm(lat, lon, lat0, lon0):
    """위경도 → (lat0, lon0) 기준 동 · 북 [mm]. 수 m 범위라 평면 근사 오차는 µm 수준이다."""
    m, n = _radii(lat0)
    east = np.radians(np.asarray(lon, float) - lon0) * n * math.cos(math.radians(lat0)) * 1000
    north = np.radians(np.asarray(lat, float) - lat0) * m * 1000
    return east, north


def latlon_of(e_mm, n_mm, lat0, lon0):
    m, n = _radii(lat0)
    return (lat0 + math.degrees(n_mm / 1000 / m),
            lon0 + math.degrees(e_mm / 1000 / (n * math.cos(math.radians(lat0)))))


def mode_point(lat, lon, radius_mm):
    """최빈점 — 반경 안 이웃이 가장 많은 점을 찾고 그 이웃들의 평균 위치를 낸다.
    반환 (위도, 경도, 이웃 수). 점이 없으면 None."""
    if len(lat) == 0:
        return None
    lat0, lon0 = float(lat[0]), float(lon[0])
    pts = np.column_stack(enu_mm(lat, lon, lat0, lon0))
    # 좌표가 1e-7° 격자라 같은 자리가 수천 번 되풀이된다 — ★서로 다른 자리만 남기고 나온 횟수를 무게로★ 센다.
    #   결과는 점마다 세는 것과 같고, 10 Hz 기록(세션당 수천~수만 점)도 순식간이다.
    uniq, first, weight = np.unique(np.round(pts, 1), axis=0, return_index=True, return_counts=True)
    r2 = radius_mm ** 2
    cnt = np.concatenate([(((uniq[s:s + 512, None, :] - uniq[None, :, :]) ** 2).sum(axis=2) <= r2) @ weight
                          for s in range(0, len(uniq), 512)])   # 512 자리씩 — 거리 행렬이 메모리를 덜 먹게
    best_cnt = int(cnt.max())
    tied = np.nonzero(cnt == best_cnt)[0]
    best_i = int(tied[np.argmin(first[tied])])             # 동점이면 시간상 먼저 나온 자리 (점마다 세던 때와 같게)
    near = ((uniq - uniq[best_i]) ** 2).sum(axis=1) <= r2
    ce, cn = np.average(uniq[near], axis=0, weights=weight[near])
    la, lo = latlon_of(ce, cn, lat0, lon0)
    return la, lo, best_cnt


# ── 데이터 ────────────────────────────────────────────────────────────────────────
class Session:
    """CSV 한 개 = 고정점 기록 한 번.
    듀얼 CSV(point_dual_*)는 ★GPS 마다 Session 하나★(side 1 · 2)로 읽고 pair 로 서로를 가리킨다 — 목록에는 한 줄,
    단독 분석은 두 GPS 를 한 화면에, 세션 비교는 두 항목으로 펼친다."""

    def __init__(self, path, side=None):
        self.path, self.name = path, path.name
        self.side = side                                  # None | 1 | 2
        self.key = f"{path.name}#{side}" if side else path.name   # 캐시 · 표 식별자 — 듀얼 두 쪽이 갈리게
        self.pair = None                                  # 듀얼의 다른 쪽 Session
        m = FNAME_RE.search(path.name)
        self.dual = bool(m and m.group(1))
        self.start = datetime.strptime(m.group(2), "%Y%m%d_%H%M%S") if m else None
        self.dt = float(m.group(3)) if m else 1.0
        sfx = "2" if side == 2 else ""                    # 2번은 같은 이름 끝에 '2' 붙은 열
        rows = []
        with open(path, newline="", encoding="utf-8") as f:
            for r in csv.DictReader(f):
                try:
                    t = r.get("t_rel")                    # 듀얼 기록에는 iTOW 로 잰 시각 열이 있다
                    rows.append((float(r["latitude" + sfx]), float(r["longitude" + sfx]),
                                 int(float(r.get("gps_quality" + sfx) or 0)),
                                 float(r.get("gps_sigma_m" + sfx) or "nan"),
                                 float(t) if t not in (None, "") else math.nan))
                except (KeyError, TypeError, ValueError):
                    continue
        a = np.array(rows, dtype=float).reshape(-1, 5)
        self.lat, self.lon = a[:, 0], a[:, 1]
        self.q = a[:, 2].astype(int)
        self.hacc = a[:, 3] * 1000                        # [mm]
        self.n = len(a)
        # 기록 시작 후 [s] — t_rel 이 있으면 그것(측위가 빠진 줄이 있어도 정확), 없으면 행 번호 × 간격으로 역산
        self.t = a[:, 4] if self.n and np.isfinite(a[:, 4]).all() else np.arange(self.n) * self.dt
        self.fixed = self.q == FIXED
        tag = f" ({side}번)" if side else ""
        self.label = (self.start.strftime("%H:%M:%S") if self.start else path.stem) + tag
        self.long_label = (self.start.strftime("%Y-%m-%d %H:%M:%S") if self.start else path.stem) + tag

    @property
    def fix_pct(self):
        return 100 * float(self.fixed.mean()) if self.n else 0.0

    @property
    def duration(self):
        return float(self.t[-1]) + self.dt if self.n else 0.0

    def index_at(self, t):
        """시각 t 에 가장 가까운 행 — t 가 t_rel(빠진 줄이 있을 수 있다)이어도 맞게 찾는다."""
        if not self.n:
            return 0
        i = int(np.searchsorted(self.t, t))
        if i >= self.n:
            return self.n - 1
        if i > 0 and t - self.t[i - 1] < self.t[i] - t:
            i -= 1
        return i

    def clock(self, i):
        if not self.start:
            return "-"
        return (self.start + timedelta(seconds=float(self.t[i]))).strftime("%H:%M:%S.%f")[:-5]


def load_session(path):
    """CSV → 목록에 올릴 Session (쓸 행이 없으면 None). 듀얼이면 1번 Session 이고 pair 가 2번이다.
    한쪽이 한 번도 측위되지 않은 듀얼 파일은 다른 쪽 하나만 돌려준다."""
    m = FNAME_RE.search(path.name)
    if not (m and m.group(1)):
        s = Session(path)
        return s if s.n else None
    sides = [s for s in (Session(path, 1), Session(path, 2)) if s.n]
    if len(sides) == 2:
        sides[0].pair, sides[1].pair = sides[1], sides[0]
    return sides[0] if sides else None


def expand(sessions):
    """세션 비교용 — 듀얼은 1번 · 2번 두 항목으로 펼친다."""
    out = []
    for s in sessions:
        out.append(s)
        if s.pair:
            out.append(s.pair)
    return out


def pair_series(s1, s2):
    """듀얼 — 같은 시각(t_rel)에 두 GPS 가 모두 측위된 행끼리의 짝.
    (t, 1번 행, 2번 행, 두 점 수평 거리 mm, 1번→2번 방위 °, 둘 다 Fixed) 를 배열로 낸다."""
    where2 = {round(float(t), 2): j for j, t in enumerate(s2.t)}
    hits = [(i, where2[k]) for i, t in enumerate(s1.t) if (k := round(float(t), 2)) in where2]
    i1 = np.array([a for a, _ in hits], dtype=int)
    i2 = np.array([b for _, b in hits], dtype=int)
    if not len(i1):
        z = np.empty(0)
        return SimpleNamespace(t=z, i1=i1, i2=i2, d=z, deg=z, fixed=np.empty(0, bool))
    lat0, lon0 = float(s1.lat[i1[0]]), float(s1.lon[i1[0]])
    e1, n1 = enu_mm(s1.lat[i1], s1.lon[i1], lat0, lon0)
    e2, n2 = enu_mm(s2.lat[i2], s2.lon[i2], lat0, lon0)
    return SimpleNamespace(t=s1.t[i1], i1=i1, i2=i2, d=np.hypot(e2 - e1, n2 - n1),
                           deg=np.degrees(np.arctan2(e2 - e1, n2 - n1)) % 360,
                           fixed=s1.fixed[i1] & s2.fixed[i2])


def quality_runs(q):
    """같은 품질이 이어지는 구간 [(품질, 시작 행, 끝 행(미포함))]"""
    if len(q) == 0:
        return []
    cut = np.nonzero(np.diff(q))[0] + 1
    return [(int(q[a]), int(a), int(b)) for a, b in zip(np.r_[0, cut], np.r_[cut, len(q)])]


def rolling_median(v, runs, half):
    """같은 RTK 구간 안에서만 앞뒤 half 행 이동 중앙값. 구간 끝에서는 창이 구간 안으로 줄어든다."""
    out = np.empty(len(v))
    w = 2 * half + 1
    for _, i0, i1 in runs:
        seg = v[i0:i1]
        m = len(seg)
        if m >= w:
            out[i0 + half:i1 - half] = np.median(sliding_window_view(seg, w), axis=1)
            edges = list(range(half)) + list(range(m - half, m))
        else:
            edges = range(m)
        for j in edges:
            out[i0 + j] = np.median(seg[max(0, j - half):j + half + 1])
    return out


def percentile(v, p):
    return float(np.percentile(v, p)) if len(v) else math.nan


def vmax(arrays):
    """여러 배열을 통틀은 최댓값 (없으면 nan)"""
    v = np.concatenate([np.asarray(a, float).ravel() for a in arrays]) if arrays else np.empty(0)
    v = v[np.isfinite(v)]
    return float(v.max()) if len(v) else math.nan


class Analysis:
    """한 세션을 원점 (lat0, lon0) 기준으로 잰 결과. o 는 설정(안정 기준 · 추세 창 · 튐 기준)."""

    def __init__(self, s, lat0, lon0, o):
        self.s = s
        self.e, self.n = enu_mm(s.lat, s.lon, lat0, lon0)
        self.d = np.hypot(self.e, self.n)
        self.step = np.full(s.n, math.nan)                # 직전 행에서 옮겨 간 거리
        self.step[1:] = np.hypot(np.diff(self.e), np.diff(self.n))
        self.runs = quality_runs(s.q)
        fix = np.nonzero(s.fixed)[0]
        self.first_fix = float(s.t[fix[0]]) if len(fix) else None
        self.dropouts = sum(1 for (q0, _, _), _ in zip(self.runs, self.runs[1:]) if q0 == FIXED)
        bad = np.nonzero(self.d > o.stable)[0]
        if len(bad) == 0:
            self.stable_t = 0.0
        elif bad[-1] == s.n - 1:
            self.stable_t = None
        else:
            self.stable_t = float(s.t[bad[-1] + 1])
        self.drift = None                                 # (동, 북) [mm/분]
        if len(fix) >= 10 and s.t[fix[-1]] - s.t[fix[0]] >= 30:
            tm = s.t[fix] / 60
            self.drift = (float(np.polyfit(tm, self.e[fix], 1)[0]),
                          float(np.polyfit(tm, self.n[fix], 1)[0]))
            self.drift_span = float(s.t[fix[-1]] - s.t[fix[0]])

        # 추세 · 튐
        half = max(1, int(round(o.trend / s.dt / 2)))
        self.te = rolling_median(self.e, self.runs, half)
        self.tn = rolling_median(self.n, self.runs, half)
        self.td = np.hypot(self.te, self.tn)              # 추세가 원점에서 떨어진 양 — 편향
        self.resid = np.hypot(self.e - self.te, self.n - self.tn)   # 추세에서 벗어난 양 — 튐
        self.spike = self.resid > o.spike
        self.trend_rate = np.full(s.n, math.nan)          # 추세 속도 [mm/분] — 창 가운데 행에 둔다
        w = 2 * half
        for _, i0, i1 in self.runs:
            if i1 - i0 > w:
                dv = np.hypot(self.te[i0 + w:i1] - self.te[i0:i1 - w], self.tn[i0 + w:i1] - self.tn[i0:i1 - w])
                self.trend_rate[i0 + half:i1 - half] = dv / (w * s.dt) * 60

    def run_of(self, i):
        for k, (q, i0, i1) in enumerate(self.runs):
            if i0 <= i < i1:
                return k, q, i0, i1
        return None

    def spike_events(self, mask):
        """mask 안 튐 묶음 [(시작 행, 끝 행(미포함))] — 이어진 튐 점은 1회"""
        idx = np.nonzero(self.spike & mask)[0]
        if len(idx) == 0:
            return []
        brk = np.nonzero(np.diff(idx) > 1)[0]
        return list(zip(np.r_[idx[0], idx[brk + 1]].tolist(), (np.r_[idx[brk], idx[-1]] + 1).tolist()))


def judge(n_spikes, trend_max, stable_mm):
    drift = not is_nan(trend_max) and trend_max > stable_mm
    if n_spikes and drift:
        return "편향 + 튐"
    if drift:
        return "서서히 편향"
    if n_spikes:
        return "순간 튐"
    return "안정"


def point_state(a, i, stable_mm):
    if a.spike[i]:
        return f"튐 — 추세에서 {a.resid[i]:.0f} mm 벗어남"
    if a.td[i] > stable_mm:
        return f"편향 — 추세 자체가 원점에서 {a.td[i]:.0f} mm"
    return "안정"


def dist_stats(parts, total):
    """parts = [(Analysis, mask)] 를 한데 모은 원점 거리 통계 [mm]"""
    d = np.concatenate([a.d[m] for a, m in parts])
    e = np.concatenate([a.e[m] for a, m in parts])
    n = np.concatenate([a.n[m] for a, m in parts])
    # 앞 행도 같은 묶음에 든 점만 — 한 간격 동안 옮겨 간 거리
    step = np.concatenate([a.step[m & np.r_[False, m[:-1]]] for a, m in parts])
    hacc = np.concatenate([a.s.hacc[m] for a, m in parts])
    ok = hacc > 0

    def agg(v, f):
        return float(f(v)) if len(v) else math.nan
    return dict(n=len(d), pct=100 * len(d) / total if total else math.nan,
                mean=agg(d, np.mean), med=percentile(d, 50), p95=percentile(d, 95),
                p99=percentile(d, 99), max=agg(d, np.max), sd_e=agg(e, np.std), sd_n=agg(n, np.std),
                step95=percentile(step, 95), stepmax=agg(step, np.max),
                hacc=percentile(hacc[ok], 50), ratio=percentile(d[ok] / hacc[ok], 50))


# ── 문자열 ────────────────────────────────────────────────────────────────────────
def is_nan(v):
    return v is None or (isinstance(v, float) and math.isnan(v))


def fmt_mm(v):
    if is_nan(v):
        return "-"
    return f"{v:.1f}" if abs(v) < 100 else f"{v:.0f}"


def fmt_s(v, none="없음"):
    return none if v is None else f"{v:g}"


def wlen(s):
    """고정폭 글꼴에서의 칸 수 — 한글은 두 칸"""
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in s)


def text_table(head, rows, indent="  "):
    """첫 열은 왼쪽, 나머지는 오른쪽 정렬한 고정폭 표"""
    rows = [[str(c) for c in r] for r in rows]
    head = [str(c) for c in head]
    w = [max(wlen(r[k]) for r in [head] + rows) for k in range(len(head))]

    def line(cells):
        out = []
        for k, c in enumerate(cells):
            pad = " " * (w[k] - wlen(c))
            out.append(c + pad if k == 0 else pad + c)
        return indent + "  ".join(out)
    return [line(head)] + [line(r) for r in rows]


def qualities(analyses):
    """나온 품질 — Fixed 먼저"""
    return sorted(set().union(*[set(a.s.q.tolist()) for a in analyses]), key=lambda v: (v != FIXED, v))


STAT_HEAD = ("품질", "점", "%", "평균", "중앙", "p95", "p99", "최대", "동σ", "북σ",
             "1회이동p95", "1회이동최대", "hAcc중앙", "실제/hAcc")


def stat_cells(name, st):
    return (name, st["n"], f"{st['pct']:.1f}", fmt_mm(st["mean"]), fmt_mm(st["med"]), fmt_mm(st["p95"]),
            fmt_mm(st["p99"]), fmt_mm(st["max"]), fmt_mm(st["sd_e"]), fmt_mm(st["sd_n"]),
            fmt_mm(st["step95"]), fmt_mm(st["stepmax"]), fmt_mm(st["hacc"]),
            "-" if is_nan(st["ratio"]) else f"{st['ratio']:.2f}")


def stats_table(analyses, total):
    """품질별 + 전체 원점 거리 표"""
    rows = [stat_cells(Q_NAME.get(q, str(q)), dist_stats([(a, a.s.q == q) for a in analyses], total))
            for q in qualities(analyses)]
    rows.append(stat_cells("전체", dist_stats([(a, np.ones(a.s.n, bool)) for a in analyses], total)))
    return text_table(STAT_HEAD, rows)


SD_HEAD = ("품질", "튐 횟수", "튐 점", "튐 최대", "최장 튐 s", "추세 최대", "추세 속도 최대(mm/분)", "판정")


def spike_drift(analyses, masks):
    """[(Analysis, mask)] 를 통틀은 (튐 횟수, 튐 점 수, 튐 최대, 최장 튐 s, 추세 최대, 추세 속도 최대)"""
    ev, n_pts, rmax, tmax, rate = [], 0, [], [], []
    for a, m in zip(analyses, masks):
        ev += [(i1 - i0) * a.s.dt for i0, i1 in a.spike_events(m)]
        sp = a.spike & m
        n_pts += int(sp.sum())
        rmax.append(a.resid[sp])
        tmax.append(a.td[m])
        rate.append(a.trend_rate[m])
    return len(ev), n_pts, vmax(rmax), (max(ev) if ev else None), vmax(tmax), vmax(rate)


def spike_drift_table(analyses, stable_mm):
    rows = []
    for q in qualities(analyses) + [None]:
        masks = [(a.s.q == q) if q is not None else np.ones(a.s.n, bool) for a in analyses]
        n_ev, n_pts, rmax, longest, tmax, rate = spike_drift(analyses, masks)
        rows.append(("전체" if q is None else Q_NAME.get(q, str(q)), n_ev, n_pts, fmt_mm(rmax),
                     fmt_s(longest, "-"), fmt_mm(tmax), fmt_mm(rate),
                     "-" if q is None else judge(n_ev, tmax, stable_mm)))
    return text_table(SD_HEAD, rows)


def rtk_lines(a, stable_mm):
    """RTK 요약 줄 (Fixed 진입 · 이탈 · 안정화 · 드리프트 · 구간)."""
    out = [f"  Fixed 진입   {fmt_s(a.first_fix)}{' s' if a.first_fix is not None else ''}"
           "   (기록 시작 기준 — USB 연결 시각이 아니다)",
           f"  Fixed 이탈   {a.dropouts} 회",
           (f"  안정화       {a.stable_t:g} s   (이후 끝까지 원점 ±{stable_mm:g} mm 안)" if a.stable_t is not None
            else f"  안정화       없음   (끝까지 원점 ±{stable_mm:g} mm 안에 머물지 못함)")]
    if a.drift:
        ve, vn = a.drift
        out.append(f"  드리프트     동 {ve:+.2f}  북 {vn:+.2f} mm/분   (합 {math.hypot(ve, vn):.2f} mm/분 · "
                   f"Fixed {a.drift_span:g} s 직선 맞춤)")
    out.append(f"  구간         {runs_summary(a.runs, a.s.dt)}")
    return out


def runs_summary(runs, dt, limit=12):
    parts = [f"{Q_NAME.get(q, q)} {(i1 - i0) * dt:g}s" for q, i0, i1 in runs[:limit]]
    if len(runs) > limit:
        parts.append(f"… 외 {len(runs) - limit}개")
    return " → ".join(parts)


def point_info(s, a, i, origin_name, stable_mm):
    """(상태 줄 한 줄, 자세한 여러 줄)"""
    q = int(s.q[i])
    h = s.hacc[i]
    k, _, i0, i1 = a.run_of(i)
    qn = Q_NAME.get(q, str(q))
    lines = [
        f"세션        {s.long_label}   ({s.name})",
        f"행          {i + 1} / {s.n}    기록 시작 후 {s.t[i]:g} s    추정 시각 {s.clock(i)}",
        f"RTK         {qn} ({q})    구간 #{k + 1} {qn} {s.t[i0]:g}~{s.t[i1 - 1]:g} s 의 {i - i0 + 1}번째 점",
        f"위경도      {s.lat[i]:.8f}, {s.lon[i]:.8f}",
        f"{origin_name}에서   동 {a.e[i]:+.1f} mm    북 {a.n[i]:+.1f} mm    거리 {a.d[i]:.1f} mm",
        f"직전 점에서 {a.step[i]:.1f} mm 이동" if i else "직전 점 없음 (첫 행)",
        f"추세        {origin_name}에서 {a.td[i]:.1f} mm (동 {a.te[i]:+.1f}  북 {a.tn[i]:+.1f})    "
        f"추세와 차이 {a.resid[i]:.1f} mm",
        f"판정        {point_state(a, i, stable_mm)}",
        (f"수신기 hAcc {h:.0f} mm    실제 거리 / hAcc = {a.d[i] / h:.2f}" if h > 0 else "수신기 hAcc 없음"),
        "",
        "※ 시각 열이 없어 시작 시각 + 행×간격으로 역산했다 — 측위가 끊겨 건너뛴 행이 있으면 그 뒤는 어긋난다.",
    ]
    short = (f"{s.label} #{i + 1}  t={s.t[i]:g}s  {qn}  동 {a.e[i]:+.1f}  북 {a.n[i]:+.1f}  "
             f"거리 {a.d[i]:.1f}  추세 {a.td[i]:.1f}  튐 {a.resid[i]:.1f} mm")
    return short, "\n".join(lines)


# ── 그림 보조 ─────────────────────────────────────────────────────────────────────
def nice_step(base, span, max_lines):
    """base 의 1·2·5·10·20… 배 중 span 안에 선이 max_lines 이하가 되는 가장 촘촘한 간격"""
    k, step = 0, base
    while span / step > max_lines:
        k += 1
        step = base * (1, 2, 5)[k % 3] * 10 ** (k // 3)
    return step


class GridLocator(mticker.Locator):
    """평면의 mm 격자. 가로 · 세로가 같은 간격을 쓰도록 축 하나가 아니라 Axes 전체 범위를 본다.
    parent 를 주면 그 격자 간격의 배수만 쓴다(굵은 선 · 눈금 글자가 가는 격자선 위에 오게)."""

    def __init__(self, ax, base, max_lines, parent=None, label=None):
        self.ax, self.base, self.max_lines = ax, base, max_lines
        self.parent, self.label = parent, label

    def current_step(self):
        x0, x1 = self.ax.get_xlim()
        y0, y1 = self.ax.get_ylim()
        base = self.parent.current_step() if self.parent else self.base
        return nice_step(base, max(abs(x1 - x0), abs(y1 - y0)), self.max_lines)

    def __call__(self):
        return self.tick_values(*self.axis.get_view_interval())

    def tick_values(self, vmin, vmax):
        vmin, vmax = sorted((vmin, vmax))
        step = self.current_step()
        if self.label is not None:
            note = "" if step == self.base else f"  (설정 {self.base:g} mm · 확대하면 촘촘해짐)"
            self.label.set_text(f"격자 {step:g} mm{note}")
        return np.arange(math.ceil(vmin / step) * step, vmax + step * 1e-6, step)


def setup_grid(ax, base):
    lbl = ax.text(0.99, 0.01, "", transform=ax.transAxes, ha="right", va="bottom", fontsize=8,
                  color="#444", zorder=10, bbox=dict(fc="white", ec="none", alpha=0.8, pad=2))
    for axis, label in ((ax.xaxis, lbl), (ax.yaxis, None)):
        minor = GridLocator(ax, base, MAX_GRID_LINES, label=label)
        axis.set_minor_locator(minor)
        axis.set_major_locator(GridLocator(ax, base, 8, parent=minor))
    ax.tick_params(which="minor", length=0)
    ax.grid(True, which="minor", color="#ececec", lw=0.6)
    ax.grid(True, which="major", color="#c4c4c4", lw=0.9)
    ax.set_axisbelow(True)


def fit_view(ax, x, y, min_half=5.0):
    """점들이 다 보이게 정사각 범위를 잡는다."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    if len(x) == 0:
        x = y = np.zeros(1)
    cx, cy = (x.min() + x.max()) / 2, (y.min() + y.max()) / 2
    half = max(x.max() - x.min(), y.max() - y.min()) / 2 * 1.12
    half = max(half, min_half)
    ax.set_xlim(cx - half, cx + half)
    ax.set_ylim(cy - half, cy + half)
    ax.set_aspect("equal", adjustable="datalim")


def dist_yscale(ax, o):
    """로그 보기의 세로축 — 10 mm 아래는 선형(symlog). 좌표가 ~1 cm 격자라 원점(0)과 한 칸 사이를
    오가는 점이 많아, 순수 로그 축이면 0 이 바닥으로 꺾여 뾰족한 선이 된다."""
    if o.log:
        ax.set_yscale("symlog", linthresh=10, linscale=0.6)
        ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{v:g}"))


def make_norm(d, o):
    top = o.cmax if o.cmax > 0 else (float(np.nanmax(d)) if len(d) else 1.0)
    if o.log:
        return LogNorm(vmin=1.0, vmax=max(top, 10.0), clip=True)
    return Normalize(vmin=0.0, vmax=max(top, 1.0), clip=True)


def add_colorbar(fig, sc, ax, o):
    cb = fig.colorbar(sc, ax=ax, fraction=0.045, pad=0.02)
    cb.set_label("원점 거리 (mm)")
    if o.log:
        cb.ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{v:g}"))
        cb.ax.yaxis.set_minor_formatter(mticker.NullFormatter())
    return cb


def shown_mask(q, show):
    other = (q != FIXED) & (q != FLOAT)
    return ((q == FIXED) & show[FIXED]) | ((q == FLOAT) & show[FLOAT]) | (other & show[0])


def quality_groups(q, show):
    """표시할 점을 품질별로 [(품질, mask)] — Fixed 먼저(아래에) 그린다"""
    return [(v, show & (q == v)) for v in sorted(set(q[show].tolist()), key=lambda v: (v != FIXED, v))]


def scatter_kw(q):
    filled = q in Q_MARK
    return dict(marker=Q_MARK.get(q, OTHER_MARK), linewidths=0 if filled else 0.9)


def quality_handles(qs):
    """나온 품질만 범례로 — 색은 거리에 따라 바뀌므로 모양만 회색으로"""
    out = []
    if FIXED in qs:
        out.append(Line2D([], [], marker="o", ls="none", color="#666", label="Fixed"))
    if FLOAT in qs:
        out.append(Line2D([], [], marker="^", ls="none", color="#666", label="Float"))
    if any(q not in Q_MARK for q in qs):
        out.append(Line2D([], [], marker="x", ls="none", color="#666", label="그 밖"))
    return out


def shade_runs(ax, s, runs):
    """Fixed 가 아닌 구간에 옅은 배경"""
    for q, i0, i1 in runs:
        if q != FIXED:
            ax.axvspan(s.t[i0] - s.dt / 2, s.t[i1 - 1] + s.dt / 2, color=Q_COLOR.get(q, OTHER_COLOR),
                       alpha=0.13, lw=0, zorder=0)


def run_breaks(runs, *arrays):
    """RTK 구간 사이에 nan 을 끼운다 — 추세선이 구간을 넘어 이어지지 않게(추세는 구간마다 따로 구했다)"""
    out = []
    for arr in arrays:
        parts = []
        for _, i0, i1 in runs:
            parts += [arr[i0:i1], [np.nan]]
        out.append(np.concatenate(parts[:-1]))
    return out


def lighten(color, k=0.78):
    r, g, b = to_rgb(color)
    return to_hex((r + (1 - r) * k, g + (1 - g) * k, b + (1 - b) * k))


def nearest(ax, x, y, ev, mask=None):
    """화면(픽셀)에서 클릭에 가장 가까운 점 (번호, 픽셀 거리)"""
    if len(x) == 0:
        return None, math.inf
    px = ax.transData.transform(np.column_stack([x, y]))
    dd = np.hypot(px[:, 0] - ev.x, px[:, 1] - ev.y)
    dd = np.where(np.isfinite(dd), dd, np.inf)
    if mask is not None:
        dd = np.where(mask, dd, np.inf)
    i = int(np.argmin(dd))
    return i, float(dd[i])


class Blitter:
    """슬라이더 커서처럼 자주 바뀌는 것만 다시 그린다 — 나머지는 떠 둔 배경을 그대로 쓴다.
    전체 그림을 다시 그리면(확대 · 창 크기 · 설정) draw_event 에서 배경을 새로 뜬다."""

    def __init__(self, canvas, artists):
        self.canvas, self.artists = canvas, list(artists)
        self.bg, self.key = None, None
        for a in self.artists:
            a.set_animated(True)
        self.cid = canvas.mpl_connect("draw_event", self.on_draw)

    def _key(self):
        f = self.canvas.figure
        return f.bbox.width, f.bbox.height, f.dpi

    def on_draw(self, _):
        self.bg = self.canvas.copy_from_bbox(self.canvas.figure.bbox)
        self.key = self._key()
        self._draw_artists()

    def _draw_artists(self):
        for a in self.artists:
            self.canvas.figure.draw_artist(a)

    def update(self):
        if self.bg is None or self.key != self._key():   # 아직 안 그렸거나 그림 저장 뒤 — 통째로
            self.canvas.draw_idle()
            return
        self.canvas.restore_region(self.bg)
        self._draw_artists()
        self.canvas.blit(self.canvas.figure.bbox)

    def disconnect(self):
        self.canvas.mpl_disconnect(self.cid)


# ── 창 ────────────────────────────────────────────────────────────────────────────
def pick_font(root, candidates):
    have = set(tkfont.families(root))
    for name in candidates:
        if name in have:
            return name
    return tkfont.nametofont("TkDefaultFont").actual("family")


def setup_mpl():
    have = {f.name for f in font_manager.fontManager.ttflist}
    for name in MPL_FONT_CANDIDATES:
        if name in have:
            matplotlib.rcParams["font.family"] = name
            break
    matplotlib.rcParams["axes.unicode_minus"] = False
    # ← → 는 슬라이더를 움직이는 데 쓴다(matplotlib 기본은 확대 이력 뒤로 · 앞으로)
    for key, drop in (("keymap.back", "left"), ("keymap.forward", "right")):
        matplotlib.rcParams[key] = [k for k in matplotlib.rcParams[key] if k != drop]


SEG_COLS = (("no", "#", 40), ("q", "상태", 70), ("t0", "시작 s", 70), ("t1", "끝 s", 70),
            ("dur", "길이 s", 70), ("jump", "진입 점프 mm", 105), ("dmean", "평균 거리 mm", 105),
            ("dmax", "최대 거리 mm", 105), ("step", "1회 이동 최대 mm", 125), ("spk", "튐", 45),
            ("tdmax", "추세 최대 mm", 100), ("hacc", "hAcc 중앙 mm", 105))
CMP_COLS = (("name", "세션", 160), ("rows", "행", 55), ("dur", "길이 s", 65), ("fix", "Fixed %", 70),
            ("first", "Fixed 진입 s", 95), ("drop", "이탈", 50), ("stable", "안정화 s", 75),
            ("oe", "원점 동 mm", 90), ("on", "원점 북 mm", 90), ("od", "원점 거리 mm", 100),
            ("spread", "Fixed 퍼짐 p95", 115), ("err", "Fixed 오차 p95", 115),
            ("spk", "튐 (Fixed)", 80), ("spkmax", "튐 최대 mm", 90), ("tmax", "편향 최대 mm", 100),
            ("verdict", "Fixed 판정", 95), ("flmax", "Float 최대 mm", 110), ("kind", "세션 원점", 80))
FOCUS_KEYS_OK = ("Canvas", "Tk", "Frame", "TFrame")      # 이 위젯에 초점이 있을 때만 ← → 스페이스를 가로챈다


class App:
    def __init__(self, root):
        self.root = root
        self.sessions, self.skipped = [], []
        self.cache = {}                  # 최빈점 — (세션 이름 | 'folder', 반경) → (위도, 경도, 이웃 수, 종류)
        self.pick = None                 # 지금 그림에서 커서 · 클릭에 필요한 것들
        self._redraw_job = None
        self._first_load = True
        self.cur_t = 0.0                 # 슬라이더 시각 [s]
        self.playing = False
        self._play_job = None
        self._play_t = 0.0
        self._play_last = 0.0

        self.F = pick_font(root, FONT_CANDIDATES)
        self.M = pick_font(root, MONO_CANDIDATES)
        root.title("reminding — GPS 고정점 검증 분석")
        root.geometry("1600x1000")
        root.minsize(1100, 700)
        style = ttk.Style(root)
        if not IS_WINDOWS:
            style.theme_use("clam")
        style.configure(".", font=(self.F, 10))
        style.configure("Treeview.Heading", font=(self.F, 9, "bold"))
        style.configure("Treeview", rowheight=22, font=(self.M, 9))
        setup_mpl()

        paned = ttk.PanedWindow(root, orient="horizontal")
        paned.pack(fill="both", expand=True)
        left = ttk.Frame(paned, padding=(10, 10, 6, 10))
        paned.add(left, weight=0)
        right = ttk.PanedWindow(paned, orient="vertical")
        paned.add(right, weight=1)
        self._build_left(left)
        plotf = ttk.Frame(right)
        right.add(plotf, weight=3)
        self._build_plot(plotf)
        self.nb = ttk.Notebook(right)
        right.add(self.nb, weight=1)
        self._build_tabs()
        root.bind("<Left>", lambda e: self.on_key(-1))
        root.bind("<Right>", lambda e: self.on_key(+1))
        root.bind("<space>", lambda e: self.on_key(0))
        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.reload()

    # ── 구성 ──
    def _build_left(self, f):
        head = ttk.Frame(f)
        head.pack(fill="x")
        self.dir_lbl = ttk.Label(head, text="", font=(self.F, 10, "bold"))
        self.dir_lbl.pack(side="left")
        ttk.Button(head, text="새로고침", width=8, command=self.reload).pack(side="right")

        lf = ttk.Frame(f)
        lf.pack(fill="both", expand=True, pady=(6, 4))
        self.lb = tk.Listbox(lf, selectmode="extended", font=(self.M, 10), width=40,
                             activestyle="none", exportselection=False)
        sb = ttk.Scrollbar(lf, command=self.lb.yview)
        self.lb.configure(yscrollcommand=sb.set)
        self.lb.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        self.lb.bind("<<ListboxSelect>>", lambda e: self.schedule_redraw())

        bf = ttk.Frame(f)
        bf.pack(fill="x")
        ttk.Button(bf, text="전체 선택", command=self.select_all).pack(side="left", expand=True, fill="x")
        ttk.Button(bf, text="선택 해제", command=self.select_none).pack(side="left", expand=True, fill="x",
                                                                      padx=(4, 0))
        ttk.Label(f, text="하나 고르면 단독 분석 · 여럿이면 세션 비교\nCtrl · Shift + 클릭으로 여러 개",
                  foreground="#666", justify="left").pack(anchor="w", pady=(4, 8))

        opt = ttk.LabelFrame(f, text="설정", padding=8)
        opt.pack(fill="x")
        self.v_grid = tk.StringVar(value="10")
        self.v_radius = tk.StringVar(value="5")
        self.v_stable = tk.StringVar(value="50")
        self.v_trend = tk.StringVar(value="15")
        self.v_spike = tk.StringVar(value="30")
        self.v_cmax = tk.StringVar(value="0")
        self.v_scale = tk.StringVar(value="log")
        self.v_range = tk.StringVar(value="fixed")
        self.v_colorby = tk.StringVar(value="dist")
        self.v_path = tk.BooleanVar(value=True)
        self.v_trendline = tk.BooleanVar(value=True)
        self.v_dualplane = tk.StringVar(value="real")      # 기본은 절대 좌표 그대로
        self.v_show = {FIXED: tk.BooleanVar(value=True), FLOAT: tk.BooleanVar(value=True),
                       0: tk.BooleanVar(value=True)}
        rows = iter(range(100))

        def number(text, var, lo, hi, inc, unit, combo=None):
            r = next(rows)
            ttk.Label(opt, text=text).grid(row=r, column=0, sticky="w", pady=2)
            if combo:
                w = ttk.Combobox(opt, textvariable=var, values=combo, width=7)
            else:
                w = ttk.Spinbox(opt, textvariable=var, from_=lo, to=hi, increment=inc, width=7)
            w.grid(row=r, column=1, sticky="w")
            ttk.Label(opt, text=unit, foreground="#666").grid(row=r, column=2, sticky="w", padx=(4, 0))

        def line(text):
            r = next(rows)
            ttk.Label(opt, text=text).grid(row=r, column=0, sticky="w", pady=2)
            fr = ttk.Frame(opt)
            fr.grid(row=r, column=1, columnspan=2, sticky="w")
            return fr

        def radios(text, var, items):
            fr = line(text)
            for value, label in items:
                ttk.Radiobutton(fr, text=label, value=value, variable=var).pack(side="left", padx=(0, 6))

        number("격자 간격", self.v_grid, 0, 0, 0, "mm", combo=GRID_CHOICES)
        number("최빈점 반경", self.v_radius, 1, 500, 1, "mm")
        number("안정 기준", self.v_stable, 1, 100000, 5, "mm")
        number("추세 창", self.v_trend, 2, 600, 1, "s (앞뒤 합쳐)")
        number("튐 기준", self.v_spike, 1, 100000, 5, "mm (추세에서)")
        number("색 상한", self.v_cmax, 0, 100000, 10, "mm (0=자동)")
        radios("스케일", self.v_scale, (("log", "로그"), ("linear", "선형")))
        radios("보기 범위", self.v_range, (("fixed", "Fixed 중심"), ("all", "전체")))
        fr = line("표시")
        for q, text in ((FIXED, "Fixed"), (FLOAT, "Float"), (0, "그 밖")):
            ttk.Checkbutton(fr, text=text, variable=self.v_show[q]).pack(side="left", padx=(0, 6))
        fr = line("단독")
        ttk.Checkbutton(fr, text="시간순 선", variable=self.v_path).pack(side="left", padx=(0, 6))
        ttk.Checkbutton(fr, text="추세선", variable=self.v_trendline).pack(side="left")
        radios("비교 색", self.v_colorby, (("dist", "거리"), ("session", "세션")))
        radios("듀얼 평면", self.v_dualplane, (("real", "실제 위치"), ("overlay", "원점 겹침")))

        for v in (self.v_grid, self.v_radius, self.v_stable, self.v_trend, self.v_spike, self.v_cmax,
                  self.v_scale, self.v_range, self.v_colorby, self.v_path, self.v_trendline,
                  self.v_dualplane, *self.v_show.values()):
            v.trace_add("write", lambda *_: self.schedule_redraw())

    def _build_plot(self, f):
        self.fig = Figure(figsize=(12, 6.6), dpi=100, constrained_layout=True)
        self.canvas = FigureCanvasTkAgg(self.fig, master=f)
        self.toolbar = NavigationToolbar2Tk(self.canvas, f, pack_toolbar=False)
        self.toolbar.update()
        self.toolbar.pack(side="bottom", fill="x")
        self.status = ttk.Label(f, text="", font=(self.M, 10), anchor="w", padding=(8, 2))
        self.status.pack(side="bottom", fill="x")
        self._build_timebar(f)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self.canvas.mpl_connect("button_press_event", self.on_click)
        self.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.canvas.mpl_connect("button_release_event", self.on_release)
        self.canvas.mpl_connect("scroll_event", self.on_scroll)
        self.drag = None                 # 좌표 평면을 끄는 중이면 그 상태

    def _build_timebar(self, f):
        bar = ttk.Frame(f, padding=(8, 4, 8, 0))
        bar.pack(side="bottom", fill="x")
        self.play_btn = ttk.Button(bar, text="▶ 재생", width=8, command=self.toggle_play)
        self.play_btn.pack(side="left")
        ttk.Button(bar, text="◀", width=3, command=lambda: self.step_time(-1)).pack(side="left", padx=(6, 0))
        ttk.Button(bar, text="▶", width=3, command=lambda: self.step_time(+1)).pack(side="left", padx=(2, 6))
        self.v_tail = tk.StringVar(value="30")
        self.v_speed = tk.StringVar(value="10")
        right = ttk.Frame(bar)
        right.pack(side="right")
        ttk.Label(right, text="꼬리").pack(side="left")
        ttk.Spinbox(right, textvariable=self.v_tail, from_=0, to=100000, increment=10, width=6).pack(side="left",
                                                                                                   padx=2)
        ttk.Label(right, text="s   속도").pack(side="left")
        ttk.Combobox(right, textvariable=self.v_speed, values=SPEED_CHOICES, width=4).pack(side="left", padx=2)
        ttk.Label(right, text="배").pack(side="left")
        self.time_lbl = ttk.Label(bar, text="", font=(self.M, 10), width=36, anchor="w")
        self.time_lbl.pack(side="right", padx=(6, 10))
        self.scale = tk.Scale(bar, orient="horizontal", showvalue=False, from_=0, to=1, resolution=0.1,
                              command=self.on_scale, highlightthickness=0, sliderlength=16, width=14,
                              troughcolor="#dcdcdc", takefocus=0, state="disabled")
        self.scale.pack(side="left", fill="x", expand=True)
        self.v_tail.trace_add("write", lambda *_: self.pick and self.set_time(self.cur_t))

    def _text_tab(self, title):
        fr = ttk.Frame(self.nb)
        txt = tk.Text(fr, font=(self.M, 10), wrap="none", height=12, relief="flat", padx=8, pady=6,
                      state="disabled")
        ys = ttk.Scrollbar(fr, orient="vertical", command=txt.yview)
        xs = ttk.Scrollbar(fr, orient="horizontal", command=txt.xview)
        txt.configure(yscrollcommand=ys.set, xscrollcommand=xs.set)
        ys.pack(side="right", fill="y")
        xs.pack(side="bottom", fill="x")
        txt.pack(fill="both", expand=True)
        self.nb.add(fr, text=title)
        return fr, txt

    def _tree_tab(self, title, cols):
        fr = ttk.Frame(self.nb)
        tree = ttk.Treeview(fr, columns=[c[0] for c in cols], show="headings", selectmode="browse")
        for key, text, width in cols:
            tree.heading(key, text=text)
            tree.column(key, width=width, minwidth=40, anchor="w" if key == "name" else "e", stretch=False)
        ys = ttk.Scrollbar(fr, orient="vertical", command=tree.yview)
        xs = ttk.Scrollbar(fr, orient="horizontal", command=tree.xview)
        tree.configure(yscrollcommand=ys.set, xscrollcommand=xs.set)
        ys.pack(side="right", fill="y")
        xs.pack(side="bottom", fill="x")
        tree.pack(fill="both", expand=True)
        tree.tag_configure(f"q{FIXED}", background=lighten(Q_COLOR[FIXED], 0.85))
        tree.tag_configure(f"q{FLOAT}", background=lighten(Q_COLOR[FLOAT], 0.82))
        self.nb.add(fr, text=title)
        return fr, tree

    def _build_tabs(self):
        self.stats_tab, self.stats_text = self._text_tab("통계")
        self.seg_tab, self.seg_tree = self._tree_tab("RTK 구간", SEG_COLS)
        self.cmp_tab, self.cmp_tree = self._tree_tab("세션 비교", CMP_COLS)
        self.point_tab, self.point_text = self._text_tab("선택 점")
        self.seg_tree.bind("<<TreeviewSelect>>", self.on_seg_select)
        self.cmp_tree.bind("<<TreeviewSelect>>", self.on_cmp_select)
        self.cmp_tree.bind("<Double-1>", self.on_cmp_double)

    @staticmethod
    def set_text(w, s):
        w.configure(state="normal")
        w.delete("1.0", "end")
        w.insert("1.0", s)
        w.configure(state="disabled")

    def show_tab(self, single):
        cur = self.nb.select()
        self.nb.tab(self.seg_tab, state="normal" if single else "hidden")
        self.nb.tab(self.cmp_tab, state="hidden" if single else "normal")
        hidden = self.cmp_tab if single else self.seg_tab
        if cur == str(hidden):
            self.nb.select(self.seg_tab if single else self.cmp_tab)

    # ── 데이터 ──
    def reload(self):
        prev = {self.sessions[i].name for i in self.lb.curselection()}
        self.sessions, self.skipped = [], []
        self.cache.clear()
        if CHECK_DIR.is_dir():
            for p in sorted(CHECK_DIR.glob("point_*.csv")):
                try:
                    s = load_session(p)
                except (OSError, UnicodeError, csv.Error):
                    self.skipped.append(p.name)
                    continue
                if s:
                    self.sessions.append(s)
                else:
                    self.skipped.append(p.name)
        self.lb.delete(0, "end")
        for s in self.sessions:
            when = s.start.strftime("%m-%d %H:%M:%S") if s.start else s.path.stem
            if s.pair:
                self.lb.insert("end", f"{when} 듀얼 {s.n}/{s.pair.n}행 Fix{s.fix_pct:.0f}/{s.pair.fix_pct:.0f}%")
            elif s.dual:
                self.lb.insert("end", f"{when} 듀얼 {s.side}번만 {s.n}행 Fix{s.fix_pct:.0f}%")
            else:
                self.lb.insert("end", f"{when} {s.n:5d}행 Fix{s.fix_pct:4.0f}%")
        for i, s in enumerate(self.sessions):
            if s.name in prev or self._first_load:     # 처음 열 때는 전부 골라 세션 비교부터
                self.lb.selection_set(i)
        self._first_load = False
        note = f" · 건너뜀 {len(self.skipped)}" if self.skipped else ""
        self.dir_lbl.configure(text=f"gps_check_data · {len(self.sessions)}개{note}")
        self.redraw()

    def select_all(self):
        self.lb.selection_set(0, "end")
        self.schedule_redraw()

    def select_none(self):
        self.lb.selection_clear(0, "end")
        self.schedule_redraw()

    def folder_ref(self, radius):
        """폴더 기준 — 모든 CSV 의 Fixed 점으로 구한 최빈점"""
        key = ("folder", radius)
        if key not in self.cache:
            # 듀얼은 안테나 자리가 단일 GPS 시험과 다를 수 있어 넣지 않는다
            fx = [s for s in self.sessions if s.fixed.any() and not s.dual]
            self.cache[key] = mode_point(np.concatenate([s.lat[s.fixed] for s in fx]),
                                         np.concatenate([s.lon[s.fixed] for s in fx]), radius) if fx else None
        return self.cache[key]

    def session_origin(self, s, radius):
        """세션 최빈점 (위도, 경도, 이웃 수, 'Fixed' | 'Float'). Fixed 가 없으면 모든 점으로 구한다."""
        key = (s.key, radius)
        if key not in self.cache:
            has_fix = bool(s.fixed.any())
            m = s.fixed if has_fix else np.ones(s.n, bool)
            self.cache[key] = (*mode_point(s.lat[m], s.lon[m], radius), "Fixed" if has_fix else "Float")
        return self.cache[key]

    def read_opts(self):
        try:
            o = SimpleNamespace(grid=float(self.v_grid.get()), radius=float(self.v_radius.get()),
                                stable=float(self.v_stable.get()), trend=float(self.v_trend.get()),
                                spike=float(self.v_spike.get()), cmax=float(self.v_cmax.get()))
        except (ValueError, tk.TclError):
            return None
        if min(o.grid, o.radius, o.stable, o.trend, o.spike) <= 0 or o.cmax < 0:
            return None
        o.log = self.v_scale.get() == "log"
        o.fit_fixed = self.v_range.get() == "fixed"
        o.by_session = self.v_colorby.get() == "session"
        o.path = self.v_path.get()
        o.trendline = self.v_trendline.get()
        o.dual_real = self.v_dualplane.get() == "real"
        o.show = {q: v.get() for q, v in self.v_show.items()}
        return o

    def read_number(self, var, default):
        try:
            v = float(var.get())
        except (ValueError, tk.TclError):
            return default
        return v if v >= 0 else default

    # ── 그리기 ──
    def schedule_redraw(self):
        if self._redraw_job:
            self.root.after_cancel(self._redraw_job)
        self._redraw_job = self.root.after(250, self.redraw)

    def redraw(self):
        self._redraw_job = None
        o = self.read_opts()
        if o is None:
            self.status.configure(text="설정 값을 확인하세요 — 색 상한만 0 이상, 나머지는 양수")
            return
        sel = [self.sessions[i] for i in self.lb.curselection()]
        if self.drag is not None:
            self.drag = None
            self._end_drag()
        if self.pick is not None:
            self.pick.blit.disconnect()
        self.fig.clear()
        self.pick = None
        self.status.configure(text="")
        self.set_text(self.point_text, "")
        for t in (self.seg_tree, self.cmp_tree):
            t.delete(*t.get_children())
        if not sel:
            msg = ("gps_check_data 폴더에 point_*.csv 가 없습니다" if not self.sessions else
                   "왼쪽에서 CSV 를 고르세요\n하나: 단독 분석 · 여럿: 세션 비교")
            self.fig.text(0.5, 0.5, msg, ha="center", va="center", fontsize=13, color="#666")
            self.set_text(self.stats_text, f"폴더: {CHECK_DIR}\n" +
                          (f"읽지 못한 · 빈 파일: {', '.join(self.skipped)}" if self.skipped else ""))
        elif len(sel) == 1:
            self.show_tab(True)
            if sel[0].pair:
                self.draw_dual(sel[0], sel[0].pair, o)
            else:
                self.draw_single(sel[0], o)
        else:
            self.show_tab(False)
            self.draw_compare(expand(sel), o)
        if self.pick is None:
            self.stop_play()
            self.scale.configure(state="disabled")
            self.time_lbl.configure(text="")
        else:
            self.scale.configure(state="normal", to=self.pick.t_end)
            self.cur_t = min(self.cur_t, self.pick.t_end)     # 설정만 바꿨으면 보던 시각 그대로
            self.scale.set(self.cur_t)
            self.set_time(self.cur_t)
        self.canvas.draw_idle()
        self.toolbar.update()            # 확대 이력(처음 보기 버튼)을 새 그림 기준으로

    def draw_single(self, s, o):
        ref = None if s.dual else self.folder_ref(o.radius)
        so = self.session_origin(s, o.radius)
        if so[3] == "Fixed":
            lat0, lon0, origin_name = so[0], so[1], "세션 원점"
            origin_note = f"세션 Fixed 최빈점 (반경 {o.radius:g} mm 안 {so[2]}점)"
        elif ref is not None:
            lat0, lon0, origin_name = ref[0], ref[1], "폴더 기준 원점"
            origin_note = "이 세션은 Fixed 가 없어 폴더 기준(모든 CSV 의 Fixed 최빈점)을 원점으로 썼다"
        else:
            lat0, lon0, origin_name = so[0], so[1], "Float 최빈점"
            origin_note = "폴더 어디에도 Fixed 가 없어 Float 최빈점을 원점으로 썼다 — 참값과 수 m 다를 수 있다"
        a = Analysis(s, lat0, lon0, o)
        show = shown_mask(s.q, o.show)
        norm = make_norm(a.d[show] if show.any() else a.d, o)
        fig = self.fig
        gs = fig.add_gridspec(4, 2, width_ratios=(1.05, 1), height_ratios=(1.3, 1.05, 0.8, 0.28))
        ax = fig.add_subplot(gs[:, 0])
        ax_d = fig.add_subplot(gs[0, 1])
        ax_en = fig.add_subplot(gs[1, 1], sharex=ax_d)
        ax_r = fig.add_subplot(gs[2, 1], sharex=ax_d)
        ax_q = fig.add_subplot(gs[3, 1], sharex=ax_d)

        # ── 평면: 원점 기준 동/북
        if o.path and show.sum() > 1:
            ax.plot(a.e[show], a.n[show], color="#888", lw=0.6, alpha=0.35, zorder=1)
        sc = None
        for q, m in quality_groups(s.q, show):
            sc = ax.scatter(a.e[m], a.n[m], c=a.d[m], cmap=CMAP, norm=norm, s=18, zorder=2, **scatter_kw(q))
        handles = quality_handles(set(s.q[show].tolist()))
        tb, teb, tnb, tdb = run_breaks(a.runs, s.t, a.te, a.tn, a.td)
        if o.trendline:
            handles.append(ax.plot(teb, tnb, color="black", lw=1.3, alpha=0.5, zorder=3,
                                   label=f"추세 ({o.trend:g} s 이동 중앙값)")[0])
        sp = a.spike & show
        if sp.any():
            handles.append(ax.scatter(a.e[sp], a.n[sp], s=70, facecolors="none", edgecolors=SPIKE_COLOR,
                                      linewidths=1.2, zorder=4, label=f"튐 ({o.spike:g} mm 넘게)"))
        handles.append(ax.plot(0, 0, marker="*", ms=17, color=ORIGIN_COLOR, mec="black", mew=0.8, ls="none",
                               zorder=6, label=origin_name)[0])
        view_x, view_y = [0.0], [0.0]
        ref_off = None
        if ref is not None and so[3] == "Fixed":
            re_, rn_ = (float(v) for v in enu_mm(ref[0], ref[1], lat0, lon0))
            ref_off = (-re_, -rn_)                         # 폴더 기준에서 본 세션 원점
            handles.append(ax.plot(re_, rn_, marker="P", ms=11, color="black", mfc="white", mew=1.3,
                                   ls="none", zorder=6, label=f"폴더 기준 ({math.hypot(re_, rn_):.1f} mm)")[0])
            view_x.append(re_)
            view_y.append(rn_)
        handles += [Line2D([], [], marker="o", ls="none", ms=10, mfc="#bbb", mec="black", mew=2,
                           label="슬라이더 위치 · 꼬리"),
                    Line2D([], [], marker="D", ls="--", color="black", lw=1, ms=8, mfc="white", mew=1.5,
                           label="그 시각 추세 · 차이")]
        if sc is not None:
            add_colorbar(fig, sc, ax, o)
        fm = show & s.fixed
        if o.fit_fixed and fm.any():
            fit_view(ax, np.r_[a.e[fm], view_x], np.r_[a.n[fm], view_y])
        else:
            fit_view(ax, np.r_[a.e[show], view_x], np.r_[a.n[show], view_y])
        setup_grid(ax, o.grid)
        ax.set_xlabel("동 (mm)")
        ax.set_ylabel("북 (mm)")
        ax.set_title(f"{s.long_label} · {s.n}행 · {origin_name} 기준 위치", fontsize=11)
        ax.legend(handles=handles, loc="upper left", fontsize=8, framealpha=0.85)

        # ── 시간: 원점 거리 (원 위치 · 추세)
        shade_runs(ax_d, s, a.runs)
        ax_d.plot(s.t, a.d, color="#999", lw=0.6, zorder=1)
        ax_d.scatter(s.t[show], a.d[show], c=a.d[show], cmap=CMAP, norm=norm, s=7, linewidths=0, zorder=2)
        ax_d.plot(tb, tdb, color="black", lw=1.4, alpha=0.75, zorder=3, label=f"추세 ({o.trend:g} s)")
        if np.isfinite(s.hacc).any():
            ax_d.plot(s.t, s.hacc, color="#444", ls="--", lw=0.9, label="수신기 hAcc")
        ax_d.axhline(o.stable, color="#1b7837", ls=":", lw=1.2, label=f"안정 기준 {o.stable:g} mm")
        if a.first_fix:
            ax_d.axvline(a.first_fix, color=Q_COLOR[FIXED], ls="--", lw=1, label=f"Fixed 진입 {a.first_fix:g} s")
        if a.stable_t:
            ax_d.axvline(a.stable_t, color="#1b7837", lw=1.2, label=f"안정화 {a.stable_t:g} s")
        dist_yscale(ax_d, o)
        ax_d.set_ylabel("원점 거리 (mm)")
        ax_d.set_title("시간에 따른 원점 거리 · 동/북 오프셋 · 튐 · RTK 상태", fontsize=11)
        ax_d.legend(fontsize=7.5, loc="upper right", framealpha=0.85, ncol=2)
        ax_d.grid(True, color="#e2e2e2")
        ax_d.tick_params(labelbottom=False)

        # ── 시간: 동/북 오프셋 (가는 선 원 위치 · 굵은 선 추세)
        shade_runs(ax_en, s, a.runs)
        ax_en.plot(s.t, a.e, color=EAST_COLOR, lw=0.7, alpha=0.45)
        ax_en.plot(s.t, a.n, color=NORTH_COLOR, lw=0.7, alpha=0.45)
        ax_en.plot(tb, teb, color=EAST_COLOR, lw=1.6, label="동 (굵은 선 추세)")
        ax_en.plot(tb, tnb, color=NORTH_COLOR, lw=1.6, label="북 (굵은 선 추세)")
        ax_en.axhline(0, color="black", lw=0.6)
        dist_yscale(ax_en, o)
        ax_en.set_ylabel("오프셋 (mm)")
        ax_en.legend(fontsize=7.5, loc="upper right", framealpha=0.85)
        ax_en.grid(True, color="#e2e2e2")
        ax_en.tick_params(labelbottom=False)

        # ── 시간: 튐 (추세에서 벗어난 양)
        shade_runs(ax_r, s, a.runs)
        ax_r.plot(s.t, a.resid, color="#555", lw=0.8)
        n_ev = len(a.spike_events(np.ones(s.n, bool)))
        if a.spike.any():
            ax_r.scatter(s.t[a.spike], a.resid[a.spike], color=SPIKE_COLOR, s=14, zorder=3,
                         label=f"튐 {n_ev}회 · {int(a.spike.sum())}점")
        ax_r.axhline(o.spike, color=SPIKE_COLOR, ls=":", lw=1.2, label=f"튐 기준 {o.spike:g} mm")
        dist_yscale(ax_r, o)
        ax_r.set_ylabel("추세와 차이 (mm)")
        ax_r.legend(fontsize=7.5, loc="upper right", framealpha=0.85)
        ax_r.grid(True, color="#e2e2e2")
        ax_r.tick_params(labelbottom=False)

        # ── 시간: RTK 상태 띠
        span = max(s.duration, s.dt)
        for q, i0, i1 in a.runs:
            ax_q.broken_barh([(s.t[i0] - s.dt / 2, (i1 - i0) * s.dt)], (0, 1),
                             facecolors=Q_COLOR.get(q, OTHER_COLOR))
            if (i1 - i0) * s.dt > span * 0.07:
                ax_q.text((s.t[i0] + s.t[i1 - 1]) / 2, 0.5, Q_NAME.get(q, str(q)), ha="center", va="center",
                          fontsize=8, color="white", fontweight="bold")
        ax_q.set_ylim(0, 1)
        ax_q.set_yticks([])
        ax_q.set_ylabel("RTK", rotation=0, ha="right", va="center")
        ax_q.set_xlabel("기록 시작 후 시간 (s)")
        ax_q.set_xlim(-s.dt, s.t[-1] + s.dt)

        # ── 슬라이더 커서 (blit 으로 이것만 다시 그린다)
        cur = SimpleNamespace(
            trail=ax.plot([], [], color="black", lw=1.6, alpha=0.85, marker=".", ms=5, zorder=8)[0],
            resid=ax.plot([], [], color="black", ls="--", lw=1.1, zorder=8)[0],
            trend=ax.plot([], [], "D", ms=9, mfc="white", mec="black", mew=1.5, zorder=9)[0],
            pt=ax.plot([], [], "o", ms=13, mec="black", mew=2, zorder=10)[0],
            text=ax.text(0.99, 0.985, "", transform=ax.transAxes, ha="right", va="top", fontsize=9, zorder=11,
                         bbox=dict(fc="white", ec="#999", alpha=0.92, pad=4)),
            dmark=ax_d.plot([], [], "o", ms=9, mfc="none", mec="black", mew=1.6, zorder=7)[0],
            tmark=ax_d.plot([], [], "D", ms=6, mfc="white", mec="black", mew=1.2, zorder=7)[0],
            rmark=ax_r.plot([], [], "o", ms=8, mfc="none", mec="black", mew=1.5, zorder=7)[0],
            vls=[axx.axvline(0, color="black", lw=1, alpha=0.6) for axx in (ax_d, ax_en, ax_r, ax_q)])
        cur.text.set_in_layout(False)
        artists = [cur.trail, cur.resid, cur.trend, cur.pt, cur.text, cur.dmark, cur.tmark, cur.rmark, *cur.vls]
        self.pick = SimpleNamespace(mode="single", s=s, a=a, o=o, show=show, norm=norm, ax_xy=ax,
                                    ax_time=(ax_d, ax_en, ax_r, ax_q), origin_name=origin_name,
                                    t_end=float(s.t[-1]), step=s.dt, cur=cur, blit=Blitter(self.canvas, artists))

        # ── 통계 글
        L = [f"파일      {s.name}",
             f"시작      {s.long_label}    간격 {s.dt:g} s    {s.n}행 (약 {s.duration:g} s)",
             f"원점      {lat0:.9f}, {lon0:.9f}  — {origin_note}"]
        if ref_off is not None:
            L.append(f"폴더 기준  세션 원점이 폴더 기준(모든 CSV 의 Fixed 최빈점)에서 동 {ref_off[0]:+.1f}  "
                     f"북 {ref_off[1]:+.1f}  = {math.hypot(*ref_off):.1f} mm 떨어져 있다")
        L += ["",
              "RTK",
              f"  Fixed 진입   {fmt_s(a.first_fix)}{' s' if a.first_fix is not None else ''}"
              "   (기록 시작 기준 — USB 연결 시각이 아니다)",
              f"  Fixed 이탈   {a.dropouts} 회",
              (f"  안정화       {a.stable_t:g} s   (이후 끝까지 원점 ±{o.stable:g} mm 안)" if a.stable_t is not None
               else f"  안정화       없음   (끝까지 원점 ±{o.stable:g} mm 안에 머물지 못함)")]
        if a.drift:
            ve, vn = a.drift
            L.append(f"  드리프트     동 {ve:+.2f}  북 {vn:+.2f} mm/분   (합 {math.hypot(ve, vn):.2f} mm/분 · "
                     f"Fixed {a.drift_span:g} s 직선 맞춤)")
        L.append(f"  구간         {runs_summary(a.runs, s.dt)}")
        L += ["", f"튐 · 편향   (추세 = 같은 RTK 구간 안 {o.trend:g} s 이동 중앙값 · 튐 = 추세에서 {o.spike:g} mm 넘게"
                  f" 벗어난 점 · 편향 = 추세가 원점에서 {o.stable:g} mm 넘게, 단위 mm)"]
        L += spike_drift_table([a], o.stable)
        L += ["", f"품질별 {origin_name} 거리 (mm · '1회 이동' 은 한 간격 {s.dt:g} s 동안 옮겨 간 거리)"]
        L += stats_table([a], s.n)
        self.set_text(self.stats_text, "\n".join(L))

        # ── RTK 구간 표
        for k, (q, i0, i1) in enumerate(a.runs):
            sl = slice(i0, i1)
            inner = a.step[i0 + 1:i1]
            hacc = s.hacc[sl]
            hacc = hacc[np.isfinite(hacc)]
            seg = np.zeros(s.n, bool)
            seg[sl] = True
            self.seg_tree.insert("", "end", iid=str(i0), tags=(f"q{q}",), values=(
                k + 1, Q_NAME.get(q, str(q)), f"{s.t[i0]:g}", f"{s.t[i1 - 1]:g}", f"{(i1 - i0) * s.dt:g}",
                fmt_mm(a.step[i0]) if i0 else "-", fmt_mm(float(a.d[sl].mean())), fmt_mm(float(a.d[sl].max())),
                fmt_mm(float(inner.max())) if len(inner) else "-", len(a.spike_events(seg)),
                fmt_mm(float(a.td[sl].max())), fmt_mm(float(np.median(hacc))) if len(hacc) else "-"))

    def draw_dual(self, s1, s2, o):
        """듀얼 CSV 단독 — 두 GPS 를 한 화면에. 각 GPS 는 자기 Fixed 최빈점을 원점으로 잰다(Fixed 가 없으면 Float 최빈점).
        평면은 기본이 '실제 위치' — 두 GPS 를 ★절대 좌표 그대로★(1번 원점 기준 같은 평면) 그려 안테나 간격이 보인다.
        '원점 겹침' 은 각자 원점을 포개는 ★상대 표시★ 라 두 점이 겹쳐 보인다(퍼짐 크기만 비교할 때)."""
        sides = {1: s1, 2: s2}
        so = {k: self.session_origin(s, o.radius) for k, s in sides.items()}
        A = {k: Analysis(s, so[k][0], so[k][1], o) for k, s in sides.items()}
        oe, on = (float(v) for v in enu_mm(so[2][0], so[2][1], so[1][0], so[1][1]))   # 1번 원점 → 2번 원점 [mm]
        shift = {1: (0.0, 0.0), 2: (oe, on) if o.dual_real else (0.0, 0.0)}
        show = {k: shown_mask(s.q, o.show) for k, s in sides.items()}
        pr = pair_series(s1, s2)
        base = pr.d[pr.fixed] if pr.fixed.any() else pr.d          # 중앙값은 둘 다 Fixed 인 행으로
        bl_med = float(np.median(base)) if len(base) else math.nan
        dev = pr.d - bl_med
        shown_d = np.concatenate([A[k].d[show[k]] for k in sides])
        norm = make_norm(shown_d if len(shown_d) else np.zeros(1), o)
        fig = self.fig
        gs = fig.add_gridspec(4, 2, width_ratios=(1.05, 1), height_ratios=(1.25, 1.05, 0.8, 0.36))
        ax = fig.add_subplot(gs[:, 0])
        ax_d = fig.add_subplot(gs[0, 1])
        ax_b = fig.add_subplot(gs[1, 1], sharex=ax_d)
        ax_r = fig.add_subplot(gs[2, 1], sharex=ax_d)
        ax_q = fig.add_subplot(gs[3, 1], sharex=ax_d)

        # ── 평면 — 색은 각자 원점 거리(빨강 → 파랑), 모양은 GPS(1번 ●, 2번 ■), 속 빈 표식은 Fixed 아님
        handles, sc, vx, vy, spiked = [], None, [np.zeros(1)], [np.zeros(1)], False
        for k, s in sides.items():
            a, m, (dx, dy), col = A[k], show[k], shift[k], SIDE_COLOR[k]
            if o.path and m.sum() > 1:
                ax.plot(a.e[m] + dx, a.n[m] + dy, color=col, lw=0.6, alpha=0.3, zorder=1)
            fm, om = m & s.fixed, m & ~s.fixed
            if fm.any():
                sc = ax.scatter(a.e[fm] + dx, a.n[fm] + dy, c=a.d[fm], cmap=CMAP, norm=norm, s=18,
                                marker=SIDE_MARK[k], linewidths=0, zorder=2)
            if om.any():
                ax.scatter(a.e[om] + dx, a.n[om] + dy, s=22, marker=SIDE_MARK[k], facecolors="none",
                           edgecolors=CMAP(norm(a.d[om])), linewidths=1.0, zorder=2)
            if o.trendline:
                _, teb, tnb = run_breaks(a.runs, s.t, a.te, a.tn)
                ax.plot(teb + dx, tnb + dy, color=col, lw=1.3, alpha=0.75, ls="--", zorder=3)
            sp = a.spike & m
            if sp.any():
                spiked = True
                ax.scatter(a.e[sp] + dx, a.n[sp] + dy, s=70, facecolors="none", edgecolors=SPIKE_COLOR,
                           linewidths=1.2, zorder=4)
            handles.append(Line2D([], [], marker=SIDE_MARK[k], ls="--" if o.trendline else "none", color=col,
                                  mfc=col, mec="black", label=f"{k}번 GPS" + (" · 점선 추세" if o.trendline else "")))
            view = (m & s.fixed) if o.fit_fixed and (m & s.fixed).any() else m
            vx.append(a.e[view] + dx)
            vy.append(a.n[view] + dy)
        if o.dual_real:
            handles.append(ax.plot(0, 0, "*", ms=17, color=ORIGIN_COLOR, mec="black", mew=0.8, ls="none", zorder=6,
                                   label="1번 원점")[0])
            handles.append(ax.plot(oe, on, "X", ms=12, color=ORIGIN_COLOR, mec="black", mew=0.8, ls="none", zorder=6,
                                   label="2번 원점")[0])
            ax.plot([0, oe], [0, on], color="black", lw=1, ls=":", zorder=5)
            ax.annotate(f"두 원점 {math.hypot(oe, on) / 1000:.4f} m", (oe / 2, on / 2), xytext=(6, 6),
                        textcoords="offset points", fontsize=8.5, zorder=6,
                        bbox=dict(fc="white", ec="none", alpha=0.8, pad=2))
            vx.append(np.array([oe]))
            vy.append(np.array([on]))
        else:
            handles.append(ax.plot(0, 0, "*", ms=17, color=ORIGIN_COLOR, mec="black", mew=0.8, ls="none", zorder=6,
                                   label="각자 원점 (Fixed 최빈점)")[0])
        handles.append(Line2D([], [], marker="o", ls="none", mfc="none", mec="#666", label="속 빈 표식 = Fixed 아님"))
        if spiked:
            handles.append(Line2D([], [], marker="o", ls="none", ms=9, mfc="none", mec=SPIKE_COLOR,
                                  label=f"튐 ({o.spike:g} mm 넘게)"))
        handles.append(Line2D([], [], marker="o", ls="none", ms=10, mfc="#bbb", mec="black", mew=2,
                              label="슬라이더 위치 · 꼬리"))
        if sc is not None:
            add_colorbar(fig, sc, ax, o)
        fit_view(ax, np.concatenate(vx), np.concatenate(vy))
        setup_grid(ax, o.grid)
        ax.set_xlabel("동 (mm)")
        ax.set_ylabel("북 (mm)")
        when = s1.start.strftime("%Y-%m-%d %H:%M:%S") if s1.start else s1.path.stem
        ax.set_title(f"{when} 듀얼 · 1번 {s1.n}행 · 2번 {s2.n}행 · "
                     + ("실제 위치 (1번 원점 기준 절대 좌표)" if o.dual_real
                        else "원점 겹침 (각자 원점 기준 · 상대 표시 — 실제 간격 아님)"), fontsize=11)
        ax.legend(handles=handles, loc="upper left", fontsize=8, framealpha=0.85)

        # ── 시간: 각자 원점 거리 (실선) · 수신기 hAcc (점선)
        for k, s in sides.items():
            ax_d.plot(s.t, A[k].d, color=SIDE_COLOR[k], lw=0.9, label=f"{k}번")
            if np.isfinite(s.hacc).any():
                ax_d.plot(s.t, s.hacc, color=SIDE_COLOR[k], lw=0.7, ls="--", alpha=0.55)
        ax_d.plot([], [], color="#666", lw=0.7, ls="--", label="수신기 hAcc")
        ax_d.axhline(o.stable, color="#1b7837", ls=":", lw=1.2, label=f"안정 기준 {o.stable:g} mm")
        dist_yscale(ax_d, o)
        ax_d.set_ylabel("원점 거리 (mm)")
        ax_d.set_title("시간에 따른 각자 원점 거리 · 두 점 간격 · 튐 · RTK 상태", fontsize=11)
        ax_d.legend(fontsize=7.5, loc="upper right", framealpha=0.85, ncol=2)
        ax_d.grid(True, color="#e2e2e2")
        ax_d.tick_params(labelbottom=False)

        # ── 시간: 같은 시각 두 점 간격 — 중앙값에서 얼마나 벗어났나 [mm]
        if len(pr.t):
            ax_b.plot(pr.t, dev, color="#444", lw=0.8, zorder=1)
            nf = ~pr.fixed
            if nf.any():
                ax_b.scatter(pr.t[nf], dev[nf], s=8, color=Q_COLOR[FLOAT], zorder=2, label="둘 중 하나라도 Fixed 아님")
                ax_b.legend(fontsize=7.5, loc="upper right", framealpha=0.85)
            ax_b.axhline(0, color="black", lw=0.6)
            ax_b.text(0.01, 0.95, f"중앙값 {bl_med / 1000:.4f} m" + (" (둘 다 Fixed 인 행)" if pr.fixed.any() else ""),
                      transform=ax_b.transAxes, va="top", fontsize=8, bbox=dict(fc="white", ec="none", alpha=0.8, pad=2))
        else:
            ax_b.text(0.5, 0.5, "두 GPS 가 같은 시각에 함께 측위된 행이 없다", transform=ax_b.transAxes,
                      ha="center", va="center", fontsize=9, color="#666")
        dist_yscale(ax_b, o)
        ax_b.set_ylabel("두 점 간격\n− 중앙값 (mm)")
        ax_b.grid(True, color="#e2e2e2")
        ax_b.tick_params(labelbottom=False)

        # ── 시간: 튐 (추세에서 벗어난 양)
        for k, s in sides.items():
            a = A[k]
            ax_r.plot(s.t, a.resid, color=SIDE_COLOR[k], lw=0.8, label=f"{k}번 튐 {len(a.spike_events(np.ones(s.n, bool)))}회")
            if a.spike.any():
                ax_r.scatter(s.t[a.spike], a.resid[a.spike], color=SPIKE_COLOR, s=12, zorder=3)
        ax_r.axhline(o.spike, color=SPIKE_COLOR, ls=":", lw=1.2, label=f"튐 기준 {o.spike:g} mm")
        dist_yscale(ax_r, o)
        ax_r.set_ylabel("추세와 차이 (mm)")
        ax_r.legend(fontsize=7.5, loc="upper right", framealpha=0.85, ncol=3)
        ax_r.grid(True, color="#e2e2e2")
        ax_r.tick_params(labelbottom=False)

        # ── 시간: RTK 상태 띠 — 위 1번, 아래 2번
        t_end = max(float(s1.t[-1]), float(s2.t[-1]))
        for k, s in sides.items():
            y0 = 0.52 if k == 1 else 0.0
            for q, i0, i1 in A[k].runs:
                ax_q.broken_barh([(s.t[i0] - s.dt / 2, float(s.t[i1 - 1] - s.t[i0]) + s.dt)], (y0, 0.46),
                                 facecolors=Q_COLOR.get(q, OTHER_COLOR))
        ax_q.set_ylim(0, 1)
        ax_q.set_yticks([0.75, 0.23])
        ax_q.set_yticklabels(["1번", "2번"], fontsize=8)
        ax_q.tick_params(axis="y", length=0)
        ax_q.set_xlabel("기록 시작 후 시간 (s)")
        dt = min(s1.dt, s2.dt)
        ax_q.set_xlim(-dt, t_end + dt)

        # ── 슬라이더 커서
        cur = {k: SimpleNamespace(
            trail=ax.plot([], [], color=SIDE_COLOR[k], lw=1.6, alpha=0.9, zorder=8)[0],
            pt=ax.plot([], [], SIDE_MARK[k], ms=13, mec="black", mew=2, zorder=10)[0],
            dmark=ax_d.plot([], [], "o", ms=8, mfc="none", mec=SIDE_COLOR[k], mew=1.8, zorder=7)[0],
            rmark=ax_r.plot([], [], "o", ms=7, mfc="none", mec=SIDE_COLOR[k], mew=1.6, zorder=7)[0]) for k in sides}
        link = ax.plot([], [], color="black", lw=1.2, ls="--", zorder=9)[0]      # 실제 위치 — 같은 시각 두 점
        bmark = ax_b.plot([], [], "o", ms=8, mfc="none", mec="black", mew=1.6, zorder=7)[0]
        vls = [axx.axvline(0, color="black", lw=1, alpha=0.6) for axx in (ax_d, ax_b, ax_r, ax_q)]
        text = ax.text(0.99, 0.985, "", transform=ax.transAxes, ha="right", va="top", fontsize=9, zorder=11,
                       bbox=dict(fc="white", ec="#999", alpha=0.92, pad=4))
        text.set_in_layout(False)
        artists = [link, bmark, text, *vls]
        for c in cur.values():
            artists += [c.trail, c.pt, c.dmark, c.rmark]
        self.pick = SimpleNamespace(mode="dual", sides=sides, A=A, shift=shift, show=show, pr=pr, dev=dev,
                                    bl_med=bl_med, o=o, norm=norm, ax_xy=ax, ax_time=(ax_d, ax_b, ax_r, ax_q),
                                    cur=cur, link=link, bmark=bmark, vls=vls, text=text, t_end=t_end, step=dt,
                                    blit=Blitter(self.canvas, artists))

        # ── 통계 글
        def bl_line(mask, name):
            d = pr.d[mask]
            if not len(d):
                return f"  {name}  없음"
            med = float(np.median(d))
            dv = np.abs(d - med)
            return (f"  {name}  {len(d)}행 · 중앙 {med / 1000:.4f} m · 평균 {d.mean() / 1000:.4f} m · "
                    f"표준편차 {d.std():.1f} mm · 중앙 대비 p95 {np.percentile(dv, 95):.1f} mm · 최대 {dv.max():.1f} mm · "
                    f"범위 {d.min() / 1000:.4f}~{d.max() / 1000:.4f} m")
        L = [f"파일      {s1.name} — 듀얼 (1번 {s1.n}행 · 2번 {s2.n}행 · 같은 시각 짝 {len(pr.t)}행)",
             f"시작      {when}    간격 {s1.dt:g} s    약 {t_end + dt:g} s"]
        for k in sides:
            L.append(f"{k}번 원점  {so[k][0]:.9f}, {so[k][1]:.9f}  — {so[k][3]} 최빈점 (반경 {o.radius:g} mm 안 {so[k][2]}점)")
        L.append(f"두 원점    1번 → 2번  동 {oe:+.1f}  북 {on:+.1f}  = {math.hypot(oe, on) / 1000:.4f} m · "
                 f"방위 {math.degrees(math.atan2(oe, on)) % 360:.1f}°")
        L += ["", "두 점 간격 (같은 시각 1번 ↔ 2번 수평 거리)",
              bl_line(pr.fixed, "둘 다 Fixed "), bl_line(np.ones(len(pr.t), bool), "전체        ")]
        if len(pr.t):
            deg = pr.deg[pr.fixed] if pr.fixed.any() else pr.deg      # 간격과 같이 둘 다 Fixed 인 행으로
            dd = (deg - np.median(deg) + 180) % 360 - 180
            L.append(f"  방위 1→2   중앙 {np.median(deg):.1f}° · 표준편차 {dd.std():.2f}°"
                     + ("  (둘 다 Fixed 인 행)" if pr.fixed.any() else ""))
        for k, s in sides.items():
            L += ["", f"RTK — {k}번 GPS"] + rtk_lines(A[k], o.stable)
        L += ["", f"튐 · 편향   (추세 = 같은 RTK 구간 안 {o.trend:g} s 이동 중앙값 · 튐 = 추세에서 {o.spike:g} mm 넘게"
                  f" · 편향 = 추세가 각자 원점에서 {o.stable:g} mm 넘게, 단위 mm)"]
        for k in sides:
            L += [f"  [{k}번]"] + spike_drift_table([A[k]], o.stable)
        L += ["", "품질별 각자 원점 거리 (mm)"]
        for k, s in sides.items():
            L += [f"  [{k}번]"] + stats_table([A[k]], s.n)
        self.set_text(self.stats_text, "\n".join(L))

        # ── RTK 구간 표 — 1번 구간들, 이어서 2번 구간들
        for k, s in sides.items():
            a = A[k]
            for n, (q, i0, i1) in enumerate(a.runs):
                sl = slice(i0, i1)
                inner = a.step[i0 + 1:i1]
                hacc = s.hacc[sl]
                hacc = hacc[np.isfinite(hacc)]
                seg = np.zeros(s.n, bool)
                seg[sl] = True
                self.seg_tree.insert("", "end", iid=f"{k}:{i0}", tags=(f"q{q}",), values=(
                    f"{k}-{n + 1}", f"{k}번 {Q_NAME.get(q, str(q))}", f"{s.t[i0]:g}", f"{s.t[i1 - 1]:g}",
                    f"{float(s.t[i1 - 1] - s.t[i0]) + s.dt:g}", fmt_mm(a.step[i0]) if i0 else "-",
                    fmt_mm(float(a.d[sl].mean())), fmt_mm(float(a.d[sl].max())),
                    fmt_mm(float(inner.max())) if len(inner) else "-", len(a.spike_events(seg)),
                    fmt_mm(float(a.td[sl].max())), fmt_mm(float(np.median(hacc))) if len(hacc) else "-"))

    def draw_compare(self, sel, o):
        fx = [s for s in sel if s.fixed.any()]
        if fx:
            g = mode_point(np.concatenate([s.lat[s.fixed] for s in fx]),
                           np.concatenate([s.lon[s.fixed] for s in fx]), o.radius)
            gnote = f"고른 세션 전체의 Fixed 최빈점 (반경 {o.radius:g} mm 안 {g[2]}점)"
        else:
            g = mode_point(np.concatenate([s.lat for s in sel]), np.concatenate([s.lon for s in sel]), o.radius)
            gnote = "고른 세션에 Fixed 가 하나도 없어 모든 점의 최빈점 — 참값과 수 m 다를 수 있다"
        lat0, lon0 = g[0], g[1]
        palette = matplotlib.cm.get_cmap("tab10" if len(sel) <= 10 else "tab20")
        items = []
        for k, s in enumerate(sel):
            so = self.session_origin(s, o.radius)
            oe, on = (float(v) for v in enu_mm(so[0], so[1], lat0, lon0))
            items.append(SimpleNamespace(
                s=s, so=so, oe=oe, on=on, od=math.hypot(oe, on), color=to_hex(palette(k % palette.N)),
                a=Analysis(s, lat0, lon0, o),                   # 전체 원점 기준 — 그림 · 오차
                own=Analysis(s, so[0], so[1], o),               # 세션 원점 기준 — 퍼짐 · 안정화 · 튐 · 편향
                show=shown_mask(s.q, o.show), arts=[]))

        fig = self.fig
        gs = fig.add_gridspec(3, 2, width_ratios=(1.05, 1), height_ratios=(1.4, 1, 1))
        ax = fig.add_subplot(gs[:, 0])
        ax_t = fig.add_subplot(gs[0, 1])
        ax_off = fig.add_subplot(gs[1, 1])
        ax_fix = fig.add_subplot(gs[2, 1], sharex=ax_off)
        shown_d = np.concatenate([it.a.d[it.show] for it in items])
        norm = make_norm(shown_d if len(shown_d) else np.zeros(1), o)

        # ── 평면
        sc = None
        for it in items:
            s = it.s
            for q, m in quality_groups(s.q, it.show):
                kw = dict(color=it.color) if o.by_session else dict(c=it.a.d[m], cmap=CMAP, norm=norm)
                art = ax.scatter(it.a.e[m], it.a.n[m], s=12, zorder=2, **scatter_kw(q), **kw)
                it.arts.append(art)
                if not o.by_session:
                    sc = art
            mk, = ax.plot(it.oe, it.on, marker="D", ms=8, color=it.color, mec="black", mew=0.8, ls="none",
                          zorder=5)
            tx = ax.annotate(s.label, (it.oe, it.on), xytext=(5, 4), textcoords="offset points", fontsize=7.5,
                             color="#222", zorder=5)
            it.arts += [mk, tx]
        handles = quality_handles(set().union(*[set(it.s.q[it.show].tolist()) for it in items])) + [
            Line2D([], [], marker="D", ls="none", color="#999", mec="black", label="세션 원점"),
            ax.plot(0, 0, marker="*", ms=18, color=ORIGIN_COLOR, mec="black", mew=0.8, ls="none", zorder=6,
                    label="전체 원점")[0],
            Line2D([], [], marker="o", ls="-", color="#999", mec="black", mew=1.5, ms=9,
                   label="슬라이더 시각의 점 · 꼬리")]
        if sc is not None:
            add_colorbar(fig, sc, ax, o)
        if o.fit_fixed and any((it.show & it.s.fixed).any() for it in items):
            xs = [it.a.e[it.show & it.s.fixed] for it in items] + [[it.oe] for it in items if it.so[3] == "Fixed"]
            ys = [it.a.n[it.show & it.s.fixed] for it in items] + [[it.on] for it in items if it.so[3] == "Fixed"]
        else:
            xs = [it.a.e[it.show] for it in items] + [[it.oe] for it in items]
            ys = [it.a.n[it.show] for it in items] + [[it.on] for it in items]
        fit_view(ax, np.concatenate(xs + [[0.0]]), np.concatenate(ys + [[0.0]]))
        setup_grid(ax, o.grid)
        ax.set_xlabel("동 (mm)")
        ax.set_ylabel("북 (mm)")
        ax.set_title(f"세션 비교 {len(sel)}개 · 전체 원점 기준 위치", fontsize=11)
        ax.legend(handles=handles, loc="upper left", fontsize=8, framealpha=0.85)
        hl_xy, = ax.plot([], [], "o", ms=15, mfc="none", mec="black", mew=1.8, zorder=7)

        # ── 시간에 따른 전체 원점 거리
        for it in items:
            s = it.s
            l1, = ax_t.plot(s.t, np.where(s.fixed, it.a.d, np.nan), color=it.color, lw=1.1)
            l2, = ax_t.plot(s.t, np.where(s.fixed, np.nan, it.a.d), color=it.color, lw=1, ls=(0, (2, 1.5)))
            it.arts += [l1, l2]
        dist_yscale(ax_t, o)
        ax_t.set_ylabel("전체 원점 거리 (mm)")
        ax_t.set_xlabel("기록 시작 후 시간 (s)")
        ax_t.set_title("시간에 따른 거리 (실선 Fixed · 점선 그 밖)", fontsize=11)
        ax_t.grid(True, color="#e2e2e2")
        hl_t, = ax_t.plot([], [], "o", ms=10, mfc="none", mec="black", mew=1.6, zorder=7)

        # ── 세션 원점 오프셋 · Fixed 진입
        x = np.arange(len(items))
        bars = ax_off.bar(x, [it.od for it in items], color=[it.color for it in items], edgecolor="black", lw=0.6)
        for b, it in zip(bars, items):
            if it.so[3] != "Fixed":
                b.set_hatch("//")
            it.arts.append(b)
        ax_off.bar_label(bars, labels=[f"{it.od:.0f}" for it in items], fontsize=7, padding=1)
        spread = [percentile(it.own.d[it.s.fixed], 95) for it in items]
        has = [k for k, v in enumerate(spread) if not math.isnan(v)]
        if has:
            ax_off.scatter(x[has], [spread[k] for k in has], marker="_", s=260, color="black",
                           linewidths=2, zorder=3, label="세션 안 Fixed 퍼짐 p95")
            ax_off.legend(fontsize=8, loc="upper right", framealpha=0.85)
        dist_yscale(ax_off, o)
        ax_off.set_ylabel("mm")
        ax_off.set_title("세션 원점 ↔ 전체 원점 거리 (빗금: Fixed 없는 세션)", fontsize=11)
        ax_off.grid(True, axis="y", color="#e2e2e2")
        ax_off.tick_params(labelbottom=False)

        ff = [it.own.first_fix for it in items]
        bars = ax_fix.bar(x, [f if f is not None else it.s.duration for f, it in zip(ff, items)],
                          color=[it.color if f is not None else "#dddddd" for f, it in zip(ff, items)],
                          edgecolor="black", lw=0.6)
        for b, f, it in zip(bars, ff, items):
            if f is None:
                b.set_hatch("xx")
            it.arts.append(b)
        ax_fix.bar_label(bars, labels=[f"{f:g}" if f is not None else "없음" for f in ff], fontsize=7, padding=1)
        st = [(k, it.own.stable_t) for k, it in enumerate(items) if it.own.stable_t is not None]
        if st:
            ax_fix.scatter([k for k, _ in st], [v for _, v in st], marker="v", s=36, color="black", zorder=3,
                           label=f"안정화 (세션 원점 ±{o.stable:g} mm)")
        ax_fix.legend(handles=[Patch(fc="#dddddd", ec="black", hatch="xx", label="Fixed 없음 (기록 길이)")]
                      + ([ax_fix.collections[-1]] if st else []), fontsize=8, loc="upper right", framealpha=0.85)
        ax_fix.set_ylabel("s")
        ax_fix.set_title("Fixed 진입 시간 (기록 시작 기준)", fontsize=11)
        ax_fix.set_xticks(x)
        ax_fix.set_xticklabels([it.s.label for it in items], rotation=45, ha="right", fontsize=8)
        ax_fix.grid(True, axis="y", color="#e2e2e2")

        # ── 슬라이더 커서 — 세션마다 '기록 시작 후 같은 시각' 의 점 · 꼬리
        artists = []
        for it in items:
            it.cur_trail, = ax.plot([], [], color=it.color, lw=1.6, alpha=0.9, zorder=8)
            it.cur_pt, = ax.plot([], [], "o", ms=10, color=it.color, mec="black", mew=1.5, zorder=9)
            it.cur_tmark, = ax_t.plot([], [], "o", ms=6, color=it.color, mec="black", mew=1, zorder=8)
            new = [it.cur_trail, it.cur_pt, it.cur_tmark]
            it.arts += new
            artists += new
        vl = ax_t.axvline(0, color="black", lw=1, alpha=0.6)
        text = ax.text(0.99, 0.985, "", transform=ax.transAxes, ha="right", va="top", fontsize=9, zorder=11,
                       bbox=dict(fc="white", ec="#999", alpha=0.92, pad=4))
        text.set_in_layout(False)
        artists += [vl, text]
        self.pick = SimpleNamespace(mode="compare", items=items, o=o, ax_xy=ax, ax_t=ax_t, ax_bars=(ax_off, ax_fix),
                                    hl_xy=hl_xy, hl_t=hl_t, cur=None, vl=vl, text=text,
                                    t_end=max(float(it.s.t[-1]) for it in items),
                                    step=min(it.s.dt for it in items), blit=Blitter(self.canvas, artists))

        # ── 세션 비교 표
        for k, it in enumerate(items):
            s = it.s
            tag = f"s{k}"
            self.cmp_tree.tag_configure(tag, background=lighten(it.color))
            fl = it.a.d[~s.fixed]
            n_ev, _, rmax, _, tmax, _ = spike_drift([it.own], [s.fixed])
            self.cmp_tree.insert("", "end", iid=str(k), tags=(tag,), values=(
                s.long_label, s.n, f"{s.duration:g}", f"{s.fix_pct:.0f}", fmt_s(it.own.first_fix),
                it.own.dropouts, fmt_s(it.own.stable_t), f"{it.oe:+.1f}", f"{it.on:+.1f}", f"{it.od:.1f}",
                fmt_mm(spread[k]), fmt_mm(percentile(it.a.d[s.fixed], 95)),
                n_ev if s.fixed.any() else "-", fmt_mm(rmax), fmt_mm(tmax),
                judge(n_ev, tmax, o.stable) if s.fixed.any() else "-",
                fmt_mm(float(fl.max())) if len(fl) else "-", it.so[3]))

        # ── 통계 글
        total = sum(it.s.n for it in items)
        L = [f"비교      {len(items)}개 세션 · {total}행 · {items[0].s.long_label} ~ {items[-1].s.long_label}",
             f"원점      {lat0:.9f}, {lon0:.9f}  — {gnote}"]
        ref = self.folder_ref(o.radius)
        if ref is not None:
            re_, rn_ = (float(v) for v in enu_mm(ref[0], ref[1], lat0, lon0))
            L.append(f"폴더 기준  {math.hypot(re_, rn_):.1f} mm 떨어져 있다 (모든 CSV 의 Fixed 최빈점)")
        fxi = [it for it in items if it.so[3] == "Fixed"]
        if fxi:
            od = np.array([it.od for it in fxi])
            oe = np.array([it.oe for it in fxi])
            on = np.array([it.on for it in fxi])
            far = max(fxi, key=lambda it: it.od)
            sp = [v for v in spread if not math.isnan(v)]
            L += ["",
                  f"세션 원점 어긋남 (전체 원점 기준, Fixed 세션 {len(fxi)}개)",
                  f"  거리   평균 {od.mean():.1f}   중앙 {np.median(od):.1f}   최대 {od.max():.1f} mm ({far.s.label})",
                  f"  동     평균 {oe.mean():+.1f}   표준편차 {oe.std():.1f} mm      "
                  f"북  평균 {on.mean():+.1f}   표준편차 {on.std():.1f} mm",
                  f"  세션 안 Fixed 퍼짐 p95 의 중앙값 {np.median(sp):.1f} mm — 이보다 세션 사이 어긋남이 크면 "
                  "접속할 때마다 Fixed 해가 달라진다는 뜻"]
        got = [(it.own.first_fix, it.s.label) for it in items if it.own.first_fix is not None]
        miss = [it.s.label for it in items if it.own.first_fix is None]
        stab = [(it.own.stable_t, it.s.label) for it in items if it.own.stable_t is not None]
        nost = [it.s.label for it in items if it.own.stable_t is None]
        drops = [(it.own.dropouts, it.s.label) for it in items if it.own.dropouts]
        L += ["", "RTK"]
        if got:
            v = [f for f, _ in got]
            L.append(f"  Fixed 진입   중앙 {np.median(v):g} s · 최대 {max(got)[0]:g} s ({max(got)[1]})   "
                     "(기록 시작 기준 — USB 연결 시각이 아니다)")
        L.append(f"  Fixed 못 잡음 {len(miss)}개" + (f": {', '.join(miss)}" if miss else ""))
        if stab:
            v = [t for t, _ in stab]
            L.append(f"  안정화       중앙 {np.median(v):g} s · 최대 {max(stab)[0]:g} s ({max(stab)[1]})   "
                     f"(세션 원점 ±{o.stable:g} mm)")
        L.append(f"  끝까지 불안정 {len(nost)}개" + (f": {', '.join(nost)}" if nost else ""))
        L.append(f"  Fixed 이탈   총 {sum(n for n, _ in drops)}회"
                 + (f": {', '.join(f'{lb} {n}회' for n, lb in drops)}" if drops else ""))
        verdicts = {}
        for it in items:
            if it.s.fixed.any():
                n_ev, _, _, _, tmax, _ = spike_drift([it.own], [it.s.fixed])
                verdicts.setdefault(judge(n_ev, tmax, o.stable), []).append(it.s.label)
        L += ["", f"튐 · 편향   (세션 원점 기준 · 추세 {o.trend:g} s 이동 중앙값 · 튐 {o.spike:g} mm · "
                  f"편향 {o.stable:g} mm, 단위 mm)"]
        L += spike_drift_table([it.own for it in items], o.stable)
        L += ["  Fixed 판정   " + " · ".join(f"{k} {len(v)}개 ({', '.join(v)})" for k, v in verdicts.items())]
        L += ["", "품질별 전체 원점 거리 (고른 세션의 모든 점, mm)"]
        L += stats_table([it.a for it in items], total)
        L += ["", "품질별 세션 원점 거리 (세션마다 자기 원점 기준 — 정밀도, mm)"]
        L += stats_table([it.own for it in items], total)
        self.set_text(self.stats_text, "\n".join(L))

    # ── 시간 슬라이더 ──
    def set_time(self, t):
        """슬라이더 시각 t 의 커서를 그린다."""
        p = self.pick
        if p is None:
            return
        t = float(np.clip(t, 0.0, p.t_end))
        self.cur_t = t
        tail = self.read_number(self.v_tail, 30.0)
        if p.mode == "single":
            self._cursor_single(p, t, tail)
        elif p.mode == "dual":
            self._cursor_dual(p, t, tail)
        else:
            self._cursor_compare(p, t, tail)
        p.blit.update()

    def _cursor_single(self, p, t, tail):
        s, a, c = p.s, p.a, p.cur
        i = s.index_at(t)
        j0 = max(0, i - int(round(tail / s.dt)))
        c.trail.set_data(a.e[j0:i + 1], a.n[j0:i + 1])
        c.pt.set_data([a.e[i]], [a.n[i]])
        c.pt.set_markerfacecolor(CMAP(p.norm(a.d[i])))
        c.trend.set_data([a.te[i]], [a.tn[i]])
        c.resid.set_data([a.te[i], a.e[i]], [a.tn[i], a.n[i]])
        c.dmark.set_data([s.t[i]], [a.d[i]])
        c.tmark.set_data([s.t[i]], [a.td[i]])
        c.rmark.set_data([s.t[i]], [a.resid[i]])
        for v in c.vls:
            v.set_xdata([s.t[i], s.t[i]])
        q = int(s.q[i])
        c.text.set_text(f"t = {s.t[i]:g} s · {Q_NAME.get(q, q)}\n"
                        f"거리 {a.d[i]:.0f} · 추세 {a.td[i]:.0f} · 튐 {a.resid[i]:.0f} mm\n"
                        f"{point_state(a, i, p.o.stable)}")
        short, detail = point_info(s, a, i, p.origin_name, p.o.stable)
        self.status.configure(text=short)
        self.set_text(self.point_text, detail)
        self.time_lbl.configure(text=f"t {s.t[i]:g} / {s.t[-1]:g} s  #{i + 1}  {s.clock(i)}")

    def _cursor_dual(self, p, t, tail):
        info = {}
        for k, s in p.sides.items():
            a, c, (dx, dy) = p.A[k], p.cur[k], p.shift[k]
            i = s.index_at(t)
            if abs(float(s.t[i]) - t) > s.dt * 0.6:          # 이 시각 측위 없음(빈칸) 또는 기록 끝
                for art in (c.trail, c.pt, c.dmark, c.rmark):
                    art.set_data([], [])
                info[k] = None
                continue
            j0 = s.index_at(t - tail)                        # 빠진 행이 있어도 시간으로 자른다
            c.trail.set_data(a.e[j0:i + 1] + dx, a.n[j0:i + 1] + dy)
            c.pt.set_data([a.e[i] + dx], [a.n[i] + dy])
            c.pt.set_markerfacecolor(CMAP(p.norm(a.d[i])))
            c.dmark.set_data([s.t[i]], [a.d[i]])
            c.rmark.set_data([s.t[i]], [a.resid[i]])
            info[k] = i
        pr, kb = p.pr, None
        if len(pr.t):
            kb = int(np.clip(np.searchsorted(pr.t, t), 0, len(pr.t) - 1))
            if kb > 0 and abs(pr.t[kb - 1] - t) < abs(pr.t[kb] - t):
                kb -= 1
            if abs(pr.t[kb] - t) > p.step * 0.6:
                kb = None
        p.bmark.set_data(([pr.t[kb]], [p.dev[kb]]) if kb is not None else ([], []))
        if p.o.dual_real and info[1] is not None and info[2] is not None:
            a1, a2, (dx, dy) = p.A[1], p.A[2], p.shift[2]
            p.link.set_data([a1.e[info[1]], a2.e[info[2]] + dx], [a1.n[info[1]], a2.n[info[2]] + dy])
        else:
            p.link.set_data([], [])
        for v in p.vls:
            v.set_xdata([t, t])
        bl = (f"두 점 간격 {pr.d[kb] / 1000:.4f} m (중앙값 대비 {p.dev[kb]:+.1f} mm) · 방위 {pr.deg[kb]:.1f}°"
              if kb is not None else "두 점 간격 — 한쪽이 이 시각 측위 없음")
        lines, parts, details = [f"t = {t:.1f} s"], [], []
        for k in (1, 2):
            i, s, a = info[k], p.sides[k], p.A[k]
            if i is None:
                lines.append(f"{k}번 측위 없음")
                parts.append(f"{k}번 없음")
                details.append(f"── {k}번 GPS — 이 시각 측위 없음 (빈칸)")
                continue
            q = int(s.q[i])
            lines.append(f"{k}번 {Q_NAME.get(q, q)} · 원점 {a.d[i]:.0f} mm · 튐 {a.resid[i]:.0f} mm")
            parts.append(f"{k}번 {Q_NAME.get(q, q)} {a.d[i]:.1f}mm")
            _, det = point_info(s, a, i, f"{k}번 원점", p.o.stable)
            details.append(f"── {k}번 GPS\n" + det.split("\n\n※")[0])   # 듀얼은 t_rel 이 있어 역산 주석이 필요 없다
        lines.append(bl)
        p.text.set_text("\n".join(lines))
        self.status.configure(text=f"t={t:.1f}s  " + "  ".join(parts) + "  " + bl)
        self.set_text(self.point_text, bl + "\n\n" + "\n\n".join(details))
        self.time_lbl.configure(text=f"t {t:.1f} / {p.t_end:g} s  (기록 시작 후)")

    def _cursor_compare(self, p, t, tail):
        live = 0
        for it in p.items:
            s, a = it.s, it.a
            if t > s.t[-1] + s.dt / 2:                       # 이 세션은 기록이 끝났다
                for art in (it.cur_trail, it.cur_pt, it.cur_tmark):
                    art.set_data([], [])
                continue
            live += 1
            i = s.index_at(t)
            j0 = max(0, i - int(round(tail / s.dt)))
            it.cur_trail.set_data(a.e[j0:i + 1], a.n[j0:i + 1])
            it.cur_pt.set_data([a.e[i]], [a.n[i]])
            it.cur_tmark.set_data([s.t[i]], [a.d[i]])
        p.vl.set_xdata([t, t])
        lines = [f"t = {t:g} s · 기록 중 {live}/{len(p.items)} 세션"]
        if p.cur is not None:                                # 강조한 세션의 그 시각 판정
            it = p.items[p.cur]
            if t <= it.s.t[-1] + it.s.dt / 2:
                i = it.s.index_at(t)
                lines.append(f"{it.s.label}: 전체 원점 {it.a.d[i]:.0f} mm · 세션 원점 {it.own.d[i]:.0f} mm")
                lines.append(f"{point_state(it.own, i, p.o.stable)}")
            else:
                lines.append(f"{it.s.label}: 기록 끝 ({it.s.t[-1]:g} s)")
        p.text.set_text("\n".join(lines))
        self.time_lbl.configure(text=f"t {t:g} / {p.t_end:g} s  (기록 시작 후)")

    def on_scale(self, val):
        t = float(val)
        if abs(t - self.cur_t) < 0.06:                       # 코드가 옮긴 값 — 이미 그렸다
            return
        if self.playing:
            self._play_t = t                                 # 재생 중에 끌면 거기서부터 이어 간다
        self.set_time(t)

    def goto_time(self, t):
        self.scale.set(t)
        self.set_time(t)

    def step_time(self, sign):
        if self.pick is None:
            return
        self.stop_play()
        self.goto_time(self.cur_t + sign * self.pick.step)

    def toggle_play(self):
        if self.playing:
            self.stop_play()
            return
        if self.pick is None:
            return
        if self.cur_t >= self.pick.t_end - 1e-9:             # 끝에서 누르면 처음부터
            self.goto_time(0.0)
        self.playing = True
        self._play_t = self.cur_t
        self._play_last = time.monotonic()
        self.play_btn.configure(text="■ 정지")
        self._tick()

    def stop_play(self):
        self.playing = False
        if self._play_job:
            self.root.after_cancel(self._play_job)
            self._play_job = None
        self.play_btn.configure(text="▶ 재생")

    def _tick(self):
        self._play_job = None
        if not self.playing or self.pick is None:
            self.stop_play()
            return
        now = time.monotonic()
        self._play_t += (now - self._play_last) * self.read_number(self.v_speed, 10.0)
        self._play_last = now
        if self._play_t >= self.pick.t_end:
            self._play_t = self.pick.t_end
            self.stop_play()
        self.goto_time(self._play_t)
        if self.playing:
            self._play_job = self.root.after(PLAY_MS, self._tick)

    def on_key(self, sign):
        w = self.root.focus_get()
        if w is not None and w.winfo_class() not in FOCUS_KEYS_OK:
            return                                           # 입력 칸 · 목록 · 표에서는 원래 동작대로
        if sign:
            self.step_time(sign)
        else:
            self.toggle_play()

    # ── 고르기 ──
    def select_single(self, i):
        self.stop_play()
        self.goto_time(float(self.pick.s.t[i]))

    def select_time(self, t):
        self.stop_play()
        self.goto_time(float(t))

    def select_compare(self, k, i=None):
        p = self.pick
        p.cur = k
        for j, it in enumerate(p.items):
            alpha = 1.0 if k is None or j == k else 0.12
            for art in it.arts:
                art.set_alpha(alpha)
        if k is None or i is None:
            p.hl_xy.set_data([], [])
            p.hl_t.set_data([], [])
        else:
            it = p.items[k]
            p.hl_xy.set_data([it.a.e[i]], [it.a.n[i]])
            p.hl_t.set_data([it.s.t[i]], [it.a.d[i]])
        if k is None:
            self.cmp_tree.selection_remove(*self.cmp_tree.selection())
            self.status.configure(text="")
        else:
            if self.cmp_tree.selection() != (str(k),):
                self.cmp_tree.selection_set(str(k))
                self.cmp_tree.see(str(k))
            it = p.items[k]
            s = it.s
            head = (f"{s.long_label} · {s.n}행 · Fixed {s.fix_pct:.0f}% · Fixed 진입 {fmt_s(it.own.first_fix)} s · "
                    f"이탈 {it.own.dropouts}회 · 안정화 {fmt_s(it.own.stable_t)} s\n"
                    f"세션 원점({it.so[3]} 최빈점) — 전체 원점에서 동 {it.oe:+.1f}  북 {it.on:+.1f}  = {it.od:.1f} mm")
            if i is None:
                self.status.configure(text=f"{s.label}  세션 원점 {it.od:.1f} mm  Fixed {s.fix_pct:.0f}%")
                self.set_text(self.point_text, head)
            else:
                # 판정 · 추세는 세션 원점 기준(own)으로 — 전체 원점 기준이면 세션 원점 어긋남이 편향으로 섞인다
                short, detail = point_info(s, it.own, i, "세션 원점", p.o.stable)
                self.status.configure(text=short + f"  (전체 원점 {it.a.d[i]:.1f} mm)")
                self.set_text(self.point_text, detail.replace(
                    "\n직전", f"\n전체 원점에서   거리 {it.a.d[i]:.1f} mm\n직전", 1) + "\n\n" + head)
                self.stop_play()
                self.scale.set(float(s.t[i]))
                self.cur_t = float(s.t[i])
        if self.pick is p:
            self.set_time(self.cur_t)                         # 커서 글(강조 세션 판정)도 바꾼다
        self.canvas.draw_idle()

    def on_click(self, ev):
        self.canvas.get_tk_widget().focus_set()               # ← → 스페이스가 그림에 오게
        p = self.pick
        if ev.button != 1 or p is None or ev.inaxes is None or getattr(self.toolbar, "mode", ""):
            return
        if ev.inaxes is p.ax_xy:
            # 좌표 평면은 지도처럼 — 끌면 옮기고(on_motion), 제자리에서 떼면 그 점을 고른다(on_release)
            self.drag = SimpleNamespace(ax=ev.inaxes, ev=ev, x=ev.x, y=ev.y, moved=False,
                                        xlim=ev.inaxes.get_xlim(), ylim=ev.inaxes.get_ylim())
            return
        self._pick_at(ev)

    def on_motion(self, ev):
        d = self.drag
        if d is None or ev.x is None or ev.y is None:
            return
        dx, dy = ev.x - d.x, ev.y - d.y
        if not d.moved:
            if math.hypot(dx, dy) < DRAG_PX:
                return
            d.moved = True
            self.stop_play()
            self.toolbar.push_current()                       # 도구 막대 '뒤로' 로 끌기 전 화면에 돌아갈 수 있게
            self.fig.set_constrained_layout(False)            # 끄는 동안엔 배치를 다시 계산하지 않는다
            self.canvas.get_tk_widget().configure(cursor="fleur")
            # ★평면만 다시 그린다★ 평면을 뺀 그림을 배경으로 한 번 떠 두고, 움직일 때마다 그 위에 평면만 얹는다(blit).
            #   오른쪽 시간 그래프까지 매번 그리면 한 번에 0.1~0.4 초라 끌기가 끊긴다. 슬라이더 커서는 놓을 때 다시 보인다.
            d.hidden = [a for a in self.pick.blit.artists if a.axes is d.ax and a.get_visible()]
            for a in d.hidden:
                a.set_visible(False)
            d.ax.set_visible(False)
            self.canvas.draw()
            d.bg = self.canvas.copy_from_bbox(self.fig.bbox)
            d.ax.set_visible(True)
        box = d.ax.bbox                                       # 누른 순간의 범위에서 움직인 픽셀만큼 반대로 민다
        sx = (d.xlim[1] - d.xlim[0]) / box.width
        sy = (d.ylim[1] - d.ylim[0]) / box.height
        d.ax.set_xlim(d.xlim[0] - dx * sx, d.xlim[1] - dx * sx)
        d.ax.set_ylim(d.ylim[0] - dy * sy, d.ylim[1] - dy * sy)
        self.canvas.restore_region(d.bg)
        self.fig.draw_artist(d.ax)
        self.canvas.blit(self.fig.bbox)

    def on_release(self, ev):
        d, self.drag = self.drag, None
        if d is None:
            return
        if d.moved:
            for a in d.hidden:
                a.set_visible(True)
            self._end_drag()
            self.canvas.draw_idle()
        else:
            self._pick_at(d.ev)                               # 끌지 않았다 — 누른 자리의 점을 고른다

    def _end_drag(self):
        self.canvas.get_tk_widget().configure(cursor="")
        self.fig.set_constrained_layout(True)

    def _pick_at(self, ev):
        """클릭 한 번 — 평면에서는 가까운 점, 시간 그래프에서는 그 시각, 막대에서는 그 세션."""
        p = self.pick
        if p is None:
            return
        if p.mode == "dual":
            if ev.inaxes is p.ax_xy:
                best = (None, math.inf)
                for side, s in p.sides.items():
                    a, (dx, dy) = p.A[side], p.shift[side]
                    i, dist = nearest(ev.inaxes, a.e + dx, a.n + dy, ev, p.show[side])
                    if dist < best[1]:
                        best = (float(s.t[i]), dist)
                if best[1] <= PICK_PX:
                    self.select_time(best[0])
            elif ev.inaxes in p.ax_time and ev.xdata is not None:
                self.select_time(ev.xdata)
            return
        if p.mode == "single":
            s, a = p.s, p.a
            if ev.inaxes is p.ax_xy:
                i, dist = nearest(ev.inaxes, a.e, a.n, ev, p.show)
                if i is not None and dist <= PICK_PX:
                    self.select_single(i)
            elif ev.inaxes in p.ax_time and ev.xdata is not None:
                self.select_single(s.index_at(ev.xdata))
            return
        if ev.inaxes is p.ax_xy or ev.inaxes is p.ax_t:
            best = (None, None, math.inf)
            # 좌표가 ~1 cm 격자라 여러 세션의 점이 한자리에 겹친다 — 강조 중인 세션을 먼저 봐 동점이면 그 세션을 고른다
            order = sorted(range(len(p.items)), key=lambda k: k != p.cur)
            for k in order:
                it = p.items[k]
                if ev.inaxes is p.ax_xy:
                    i, dist = nearest(ev.inaxes, it.a.e, it.a.n, ev, it.show)
                else:
                    i, dist = nearest(ev.inaxes, it.s.t, it.a.d, ev)
                if dist < best[2]:
                    best = (k, i, dist)
            if best[2] <= PICK_PX:
                self.select_compare(best[0], best[1])
            else:
                self.select_compare(None)
        elif ev.inaxes in p.ax_bars and ev.xdata is not None:
            k = int(round(ev.xdata))
            self.select_compare(k if 0 <= k < len(p.items) else None)

    def on_scroll(self, ev):
        p, ax = self.pick, ev.inaxes
        if p is None or ax is None or ev.xdata is None:
            return
        f = 1 / 1.25 if ev.button == "up" else 1.25
        time_axes = p.ax_time if p.mode in ("single", "dual") else (p.ax_t,)
        if ax is not p.ax_xy and ax not in time_axes:
            return
        self.toolbar.push_current()
        x0, x1 = ax.get_xlim()
        ax.set_xlim(ev.xdata - (ev.xdata - x0) * f, ev.xdata + (x1 - ev.xdata) * f)
        if ax is p.ax_xy:                                  # 평면은 가로세로 함께
            y0, y1 = ax.get_ylim()
            ax.set_ylim(ev.ydata - (ev.ydata - y0) * f, ev.ydata + (y1 - ev.ydata) * f)
        self.canvas.draw_idle()

    def on_seg_select(self, _):
        sel = self.seg_tree.selection()
        if sel and self.pick and self.pick.mode == "single":
            self.select_single(int(sel[0]))                # 구간 시작 행
        elif sel and self.pick and self.pick.mode == "dual":
            side, i0 = sel[0].split(":")                   # 듀얼은 '번호:구간 시작 행'
            self.select_time(self.pick.sides[int(side)].t[int(i0)])

    def on_cmp_select(self, _):
        sel = self.cmp_tree.selection()
        if sel and self.pick and self.pick.mode == "compare" and int(sel[0]) != self.pick.cur:
            self.select_compare(int(sel[0]))

    def on_cmp_double(self, ev):
        row = self.cmp_tree.identify_row(ev.y)
        if not row or not self.pick or self.pick.mode != "compare":
            return
        s = self.pick.items[int(row)].s
        idx = self.sessions.index(s if s in self.sessions else s.pair)   # 듀얼 2번 → 목록의 그 파일
        self.lb.selection_clear(0, "end")
        self.lb.selection_set(idx)
        self.lb.see(idx)
        self.redraw()

    def on_close(self):
        self.stop_play()
        self.root.quit()
        self.root.destroy()


def main():
    if IS_WINDOWS:
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)   # 고해상도 화면에서 글자가 번지지 않게
        except (AttributeError, OSError):
            pass
    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
