#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
one_launch.py ― white 통합 런치 (GPS · IMU · 아두이노 · 매핑 · 주행 · 주행 기록)
════════════════════════════════════════════════════════════════════════════════
    ros2 launch white one_launch.py
    ros2 run white prompt                ← ★별 터미널★ 에서 (메뉴는 표준입력이 필요하다)
    ros2 run white check                 ← 띄우기 전에 장치 연결 점검 (GPS 1번·2번 칩 ID · IMU · 아두이노)

띄우는 것 (카메라 · 라이다 · HUD 없음)
    white/gps       F9P + NTRIP RTK 보정 → /gps_pvt · /fix,  iAHRS → /imu
    white/arduino   아두이노 Mega 브리지 ↔ /cmd_vel_raw · /encoder · /wheel_speed (115200 bps, 리모컨은 아두이노가 쥔다)
    white/driving   상태기계 + 헤딩 + 경로추종 (black v6.7.3 제어)
    white/mapping   /gps_pvt 원값으로 경로 수집
    white/record    자율주행 구간(DRIVE_HEADING~DONE)의 토픽 → <ws>/src/white/ros2bag/<경로>-<시각>.csv
                    (gold white1 record.py 계승, 구독만 한다. use_record:=false 로 끈다)

포트는 비워 두면 각 노드가 USB VID/PID 로 찾는다(ports.py). 고정하려면 예:
    ros2 launch white one_launch.py gps_port:=/dev/ttyACM1 arduino_port:=/dev/ttyACM0
처음 한 번 `sudo usermod -aG dialout $USER` 후 다시 로그인해야 /dev/ttyACM* · ttyUSB* 를 연다.
NTRIP 은 PC 인터넷으로 받는다 — 망이 없으면 ntrip:=false (그러면 RTK Fixed 가 안 되므로
rtk_only:=true 인 채로는 출발하지 않고 서서 기다린다).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

NODE_ENV = {'PYTHONUNBUFFERED': '1', 'RCUTILS_LOGGING_BUFFERED_STREAM': '0'}

ARGS = [
    # (이름, 기본값, 형, 설명)
    ('data_dir', '', str, '경로 CSV 폴더. 비우면 <ws>/src/white/gps_data'),
    ('cruise_speed', '1.8', float, '순항 상한 [m/s] — 걷기~뛰기 사이'),
    ('min_speed', '1.0', float, '주행 중 최저(코너) 속도 [m/s] (black)'),
    ('heading_speed', '1.0', float, '출발 직진(헤딩 초기화) 속도 [m/s]'),
    ('rtk_only', 'true', bool, 'RTK Fixed 일 때만 주행 — 아니면 서서 기다린다 (gold)'),
    ('wp_reach_m', '0.9', float, '도착 허용반경 [m] (gold)'),
    ('cte_deviation_m', '2.0', float, '경로이탈 안전정지 거리 [m] (gold)'),
    ('gps_port', '', str, 'F9P 포트. 비우면 VID 1546 으로 찾는다'),
    ('imu_port', '', str, 'iAHRS 포트. 비우면 10c4:ea60 으로 찾는다'),
    ('ntrip', 'true', bool, 'NGII VRS NTRIP 보정 받기'),
    ('imu_period_ms', '50', int, 'iAHRS 출력 주기 [ms] (50 = 20 Hz)'),
    ('arduino_port', '', str, '아두이노 포트. 비우면 VID 2341/2a03/1a86 으로 찾는다'),
    ('arduino_baud', '115200', int, '115200 = hl_ws/tool/arduino_code.ino(white_fw) · 지금 차의 v6 계열 스케치, 57600 = Vr.1'),
    ('steer_scale', '0.0', float, '조향 눈금 환산. 0 = 펌웨어를 알아보고 자동 (v6 1.0 · white_fw/Vr.1 19/21)'),
    ('enc_period_s', '0.0', float, '아두이노 PID 주기 [s]. 0 = E 줄 간격으로 측정'),
    ('ticks_per_rev', '300.0', float, '엔코더 틱/바퀴 1회전 (black)'),
    ('wheel_circ_m', '0.8482', float, '바퀴 둘레 [m] (black)'),
    ('use_record', 'true', bool, 'record 노드(주행 CSV 기록). 자율주행 구간에서만 파일이 생긴다'),
    ('record_dir', '', str, '주행 기록 폴더. 비우면 <ws>/src/white/ros2bag'),
]


def generate_launch_description():
    def p(name):
        typ = next(a[2] for a in ARGS if a[0] == name)
        return ParameterValue(LaunchConfiguration(name), value_type=typ)

    decl = [DeclareLaunchArgument(n, default_value=d, description=desc) for n, d, _, desc in ARGS]

    gps = Node(
        package='white', executable='gps', name='gps_node', output='screen',
        additional_env=NODE_ENV,
        parameters=[{'gps_port': p('gps_port'), 'imu_port': p('imu_port'),
                     'ntrip': p('ntrip'), 'imu_period_ms': p('imu_period_ms')}],
    )
    arduino = Node(
        package='white', executable='arduino', name='arduino_node', output='screen',
        additional_env=NODE_ENV,
        parameters=[{'port': p('arduino_port'), 'baud': p('arduino_baud'),
                     'steer_scale': p('steer_scale'), 'enc_period_s': p('enc_period_s'),
                     'ticks_per_rev': p('ticks_per_rev'), 'wheel_circ_m': p('wheel_circ_m')}],
    )
    driving = Node(
        package='white', executable='driving', name='driving_node', output='screen',
        additional_env=NODE_ENV,
        parameters=[{'data_dir': p('data_dir'), 'cruise_speed': p('cruise_speed'),
                     'min_speed': p('min_speed'), 'heading_speed': p('heading_speed'),
                     'rtk_only': p('rtk_only'), 'wp_reach_m': p('wp_reach_m'),
                     'cte_deviation_m': p('cte_deviation_m')}],
    )
    mapping = Node(
        package='white', executable='mapping', name='mapping_node', output='screen',
        additional_env=NODE_ENV,
        parameters=[{'data_dir': p('data_dir')}],
    )
    # 구독만 하는 노드라 제어에 끼어들지 않는다(발행 토픽 없음)
    record = Node(
        package='white', executable='record', name='record_node', output='screen',
        additional_env=NODE_ENV,
        parameters=[{'output_dir': p('record_dir')}],
        condition=IfCondition(LaunchConfiguration('use_record')),
    )
    hint = LogInfo(msg="\n  ▶ 메뉴는 별 터미널에서:  ros2 run white prompt\n")
    return LaunchDescription(decl + [gps, arduino, driving, mapping, record, hint])
