#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
one_launch.py ― white1 통합 런치 (단일 GPS · 전방 안테나 — GPS · 아두이노 · 매핑 · 주행 · 주행 기록)
════════════════════════════════════════════════════════════════════════════════
    ros2 launch white1 one_launch.py
    ros2 run white1 prompt               ← ★별 터미널★ 에서 (메뉴는 표준입력이 필요하다)
    ros2 run broon check                 ← 띄우기 전에 장치 연결 점검 (전방 GPS · 아두이노)
    ros2 run broon gps_setting           ← 전방 · 후방에 어느 GPS(칩 ID)를 달았는지 설정 (GUI)

띄우는 것 — ★하드웨어 노드는 broon 패키지 것을 쓴다★ (gold 의 white1 · white2 가 nxde 의 arduino 를 띄우는 것과 같은 구조)
    broon/gps       전방 GPS(배치 설정의 칩 ID) + NTRIP RTK 보정 → /gps_pvt · /fix          (dual:=false)
    broon/arduino   아두이노(UNO R4 Minima) 브리지 — /cmd_vel_raw(km/h · 도) → 4바이트 프레임 (가작성 규약)
    white1/driving  상태기계 + GPS 만으로 헤딩(직진 초기화 → 전방 안테나 추적) + 경로추종 (black v6.7.3 제어)
    white1/mapping  전방 GPS → 뒷차축 궤적으로 경로 수집
    white1/record   자율주행 구간의 토픽 → <ws>/src/white1/ros2bag/<경로>-<시각>.csv   (use_record:=false 로 끈다)
IMU · 카메라 · 라이다는 쓰지 않는다. 듀얼 GPS 는 white2.

처음 한 번 `sudo usermod -aG dialout $USER` 후 다시 로그인해야 /dev/ttyACM* 를 연다.
NTRIP 은 PC 인터넷으로 받는다 — 망이 없으면 ntrip:=false (그러면 RTK Fixed 가 안 되므로 출발이 거절된다).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

NODE_ENV = {'PYTHONUNBUFFERED': '1', 'RCUTILS_LOGGING_BUFFERED_STREAM': '0'}
HW_PKG = 'broon'             # 하드웨어(아두이노 · GPS) 노드가 있는 패키지
PKG = 'white1'

ARGS = [
    # (이름, 기본값, 형, 설명)
    ('data_dir', '', str, '경로 CSV 폴더. 비우면 <ws>/src/white1/gps_data'),
    ('cruise_kmh', '6.0', float, '순항 상한 [km/h] (아두이노 규약 상한 16). 명령은 정수 km/h 로 나간다'),
    ('min_kmh', '4.0', float, '주행 중 최저(코너) 속도 [km/h]'),
    ('heading_kmh', '4.0', float, '출발 직진(헤딩 초기화) 속도 [km/h]'),
    ('run_min_quality', '3', int, '주행 중 필요한 GPS 품질 — 3 = Float 여도 계속(기본), 4 = Fixed 에서만. 출발은 언제나 Fixed'),
    ('antenna_base_m', '1.0', float, '뒷차축 → 전방 안테나 거리 [m] ★실측해서 넣을 것★'),
    ('wp_reach_m', '0.9', float, '도착 허용반경 [m]'),
    ('cte_deviation_m', '2.0', float, '경로이탈 안전정지 거리 [m]'),
    ('ntrip', 'true', bool, 'NGII VRS NTRIP 보정 받기'),
    ('arduino_port', '', str, '아두이노 포트. 비우면 USB VID 로 찾는다 (UNO R4 Minima 2341:0069 우선)'),
    ('steer_sign', '1.0', float, '조향 부호. 1 = + 좌(기본), -1 = 뒤집기 (새 아두이노가 + 우 일 때)'),
    ('idle_heartbeat', 'true', bool, '자율주행이 아닐 때도 0·0 을 1초마다 보낸다. 손으로 몰 때 방해되면 false'),
    ('use_record', 'true', bool, 'record 노드(주행 CSV 기록). 자율주행 구간에서만 파일이 생긴다'),
    ('record_dir', '', str, '주행 기록 폴더. 비우면 <ws>/src/white1/ros2bag'),
]


def generate_launch_description():
    def p(name):
        typ = next(a[2] for a in ARGS if a[0] == name)
        return ParameterValue(LaunchConfiguration(name), value_type=typ)

    decl = [DeclareLaunchArgument(n, default_value=d, description=desc) for n, d, _, desc in ARGS]

    gps = Node(
        package=HW_PKG, executable='gps', name='gps_node', output='screen',
        additional_env=NODE_ENV,
        parameters=[{'dual': False, 'ntrip': p('ntrip')}],
    )
    arduino = Node(
        package=HW_PKG, executable='arduino', name='arduino_node', output='screen',
        additional_env=NODE_ENV,
        parameters=[{'port': p('arduino_port'), 'steer_sign': p('steer_sign'),
                     'idle_heartbeat': p('idle_heartbeat')}],
    )
    driving = Node(
        package=PKG, executable='driving', name='driving_node', output='screen',
        additional_env=NODE_ENV,
        parameters=[{'data_dir': p('data_dir'), 'cruise_kmh': p('cruise_kmh'),
                     'min_kmh': p('min_kmh'), 'heading_kmh': p('heading_kmh'),
                     'run_min_quality': p('run_min_quality'), 'antenna_base_m': p('antenna_base_m'),
                     'wp_reach_m': p('wp_reach_m'), 'cte_deviation_m': p('cte_deviation_m')}],
    )
    mapping = Node(
        package=PKG, executable='mapping', name='mapping_node', output='screen',
        additional_env=NODE_ENV,
        parameters=[{'data_dir': p('data_dir'), 'antenna_base_m': p('antenna_base_m')}],
    )
    # 구독만 하는 노드라 제어에 끼어들지 않는다(발행 토픽 없음)
    record = Node(
        package=PKG, executable='record', name='record_node', output='screen',
        additional_env=NODE_ENV,
        parameters=[{'output_dir': p('record_dir')}],
        condition=IfCondition(LaunchConfiguration('use_record')),
    )
    hint = LogInfo(msg=f"\n  ▶ 메뉴는 별 터미널에서:  ros2 run {PKG} prompt\n")
    return LaunchDescription(decl + [gps, arduino, driving, mapping, record, hint])
