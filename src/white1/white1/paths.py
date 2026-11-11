#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
paths.py ― 경로(맵) CSV · 주행 기록 폴더의 단일 소유자
════════════════════════════════════════════════════════════════════════════════
mapping 이 쓰고 driving · prompt 가 읽는 경로 폴더(gps_data)와 record 가 쓰는 주행 기록
폴더(ros2bag)를 여기 한 곳에서 정한다 (black 은 세 파일이 각자 "~/black_ws/black_ws/gps_data" 를
들고 있었다).

우선순위 (경로 폴더 / 기록 폴더)
  1) 노드 파라미터 data_dir / output_dir
  2) 환경변수 WHITE1_DATA_DIR / WHITE1_RECORD_DIR
  3) ★소스 트리★ <ws>/src/white1/gps_data · ros2bag   — hl_ws 면 ~/hl_ws/src/white1/…
  4) ~/white1/gps_data · ros2bag
install/ 안에는 쌓지 않는다(다시 빌드하면 날아간다). 3) 은 --symlink-install 이 아니어도
찾는다 — 설치본 위치에서 조상 폴더를 거슬러 올라가 <조상>/src/white1/package.xml 을 찾는다
(gold paths.py 2026-09-04 의 복사설치 함정과 같은 처리).
"""

import os

PKG = "white1"


def source_root():
    """<ws>/src/white1 경로. 못 찾으면 ''."""
    # --symlink-install : 이 파일이 소스를 가리키는 링크다
    here = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    if os.path.isfile(os.path.join(here, "package.xml")):
        return here
    # 복사설치 : <ws>/install/white1/lib/python3.x/site-packages/white1/paths.py
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        cand = os.path.join(d, "src", PKG)
        if os.path.isfile(os.path.join(cand, "package.xml")):
            return cand
        parent = os.path.dirname(d)
        if parent == d:
            return ""
        d = parent


def _resolve(explicit, env_name, sub):
    if explicit:
        return os.path.expanduser(explicit)
    env = os.environ.get(env_name, "")
    if env:
        return os.path.expanduser(env)
    root = source_root()
    if root:
        return os.path.join(root, sub)
    return os.path.expanduser(os.path.join("~", PKG, sub))


def data_dir(explicit=""):
    """경로(맵) CSV 폴더 — mapping 이 쓰고 driving · prompt 가 읽는다."""
    return _resolve(explicit, "WHITE1_DATA_DIR", "gps_data")


def record_dir(explicit=""):
    """주행 기록 CSV 폴더 — record 가 쓴다 (gold white1 의 ros2bag 과 같은 자리)."""
    return _resolve(explicit, "WHITE1_RECORD_DIR", "ros2bag")


def list_routes(d):
    """폴더의 경로 CSV 이름 (이름순 = 날짜순)."""
    try:
        return sorted(f for f in os.listdir(d) if f.lower().endswith(".csv"))
    except OSError:
        return []
