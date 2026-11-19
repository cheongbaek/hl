#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
one.launch.py ― white1 통합 런치 (단일 GPS · 전방 안테나 — GPS · 아두이노 · 조이스틱 · 매핑 · 주행 · 주행 기록)
════════════════════════════════════════════════════════════════════════════════
    ros2 launch white1 one.launch.py
    ros2 run white1 prompt               ← ★별 터미널★ 에서 (메뉴는 표준입력이 필요하다)
    ros2 run broon check                 ← 띄우기 전에 장치 연결 점검 (전방 GPS · 구동 아두이노 · 조이스틱)
    ros2 run broon gps_setting           ← 전방 · 후방에 어느 GPS(칩 ID)를 달았는지 설정 (GUI)
    ros2 run broon kill                  ← 런치가 깔끔히 안 내려갈 때 남은 ROS2 를 한 번에 정리
(2026-11-19 one_launch.py 에서 이름을 바꿨다 — 사용자 지정)

띄우는 것 — ★하드웨어 노드는 broon 패키지 것을 쓴다★ (차를 움직이는 것은 broon 만으로 된다)
    broon/gps       전방 GPS(배치 설정의 칩 ID) + NTRIP RTK 보정 → /gps_pvt · /fix          (dual:=false)
    broon/arduino   아두이노 판별("J," = 조이스틱, 나머지 = 구동) · /cmd_vel → 구동 우노 · 우노 출력 → /enc_pot
    broon/joy       조이스틱 → /cmd_vel (SWA ON 일 때만) · /joy318 + 화면 (use_joy_gui:=false 면 창 없이)
    white1/driving  상태기계 + GPS 만으로 헤딩 + 경로추종 (black v6.7.3 제어) → /cmd_vel [펄스, 조향]
    white1/mapping  전방 GPS 궤적 → map_<번호>.csv
    broon/record    ★자율주행 기록★(mode:=drive) — 자율주행 구간마다 route_<매핑>_<연도월일>_<시분초>.csv
                    (<ws>/src/broon/ros2bag). 따로 ros2 run broon record 로 수동 기록을 돌려도 둘 다 남는다
    broon/estop     원격 E-STOP — 웹 domiman.duckdns.org/estop · domichat 방 domi_fishing_broon 의 STOP → /estop 1 →
                    driving 은 자율주행을 끝내고 arduino 는 차를 세운다. GO 로 해제 (use_estop:=false 로 끈다)
IMU · 카메라 · 라이다는 쓰지 않는다. 듀얼 GPS 는 white2(아직 옛 구조 — 2026-11-19 보류).

조이스틱과 자율주행 (2026-11-19 사용자 지정)
  SWA ON = 조이스틱이 /cmd_vel 을 낸다(스틱이 0 이면 '0 0' 을 잠깐 내고 조용해진다) · /joy318 을 낸다.
  SWA OFF = 스틱을 무시한다. 자율주행은 SWA OFF 에서만 출발하고, 주행 중 SWA 를 ON 으로 바꾸면 driving 이 멈춘다.

처음 한 번 `sudo usermod -aG dialout $USER` 후 다시 로그인해야 /dev/ttyACM* 를 연다.
NTRIP 은 PC 인터넷으로 받는다 — 망이 없으면 ntrip:=false (그러면 RTK Fixed 가 안 되므로 출발이 거절된다).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

NODE_ENV = {'PYTHONUNBUFFERED': '1', 'RCUTILS_LOGGING_BUFFERED_STREAM': '0'}
HW_PKG = 'broon'             # 하드웨어(아두이노 · 조이스틱 · GPS · 기록) 노드가 있는 패키지
PKG = 'white1'

ARGS = [
    # (이름, 기본값, 형, 설명)
    ('data_dir', '', str, '경로 CSV 폴더. 비우면 <ws>/src/white1/gps_data'),
    ('cruise_pulse', '75', int, '순항 상한 [펄스] (우노 상한 92)'),
    ('min_pulse', '50', int, '주행 중 최저(코너) 속도 [펄스]'),
    ('heading_pulse', '50', int, '출발 직진(헤딩 초기화) 속도 [펄스]'),
    ('pulse_per_mps', '45.0', float, '1 m/s = 몇 펄스 ★추정 — record 의 GPS 속도 · 실측 펄스로 맞출 것★'),
    ('steer_trim_deg', '0.0', float, '직진 트림 [도, + 좌] (black 은 −1.5 — 새 조향은 중립에서 시작)'),
    ('steer_gain_l', '1.0', float, '좌회전 실효게인 (black 0.85)'),
    ('steer_gain_r', '1.0', float, '우회전 실효게인 (black 0.82)'),
    ('run_min_quality', '3', int, '주행 중 필요한 GPS 품질 — 3 = Float 여도 계속(기본), 4 = Fixed 에서만. 출발은 언제나 Fixed'),
    ('antenna_base_m', '1.0', float, '뒷차축 → 전방 안테나 거리 [m] ★실측해서 넣을 것★'),
    ('wp_reach_m', '0.9', float, '도착 허용반경 [m]'),
    ('cte_deviation_m', '2.0', float, '경로이탈 안전정지 거리 [m]'),
    ('ntrip', 'true', bool, 'NGII VRS NTRIP 보정 받기'),
    ('drive_port', '', str, '구동 우노 포트. 비우면 판별 ("J," 를 보내지 않는 아두이노)'),
    ('joy_port', '', str, '조이스틱 메가 포트. 비우면 판별 ("J," 를 보내는 아두이노)'),
    ('use_joy_gui', 'true', bool, '조이스틱 화면'),
    ('use_record', 'true', bool, '자율주행 기록(route_*.csv). 자율주행 구간에서만 파일이 생긴다'),
    ('record_dir', '', str, '주행 기록 폴더. 비우면 <ws>/src/broon/ros2bag'),
    ('use_estop', 'true', bool, '원격 E-STOP (domichat · 인터넷 필요)'),
]


def generate_launch_description():
    def p(name):
        typ = next(a[2] for a in ARGS if a[0] == name)
        return ParameterValue(LaunchConfiguration(name), value_type=typ)

    decl = [DeclareLaunchArgument(n, default_value=d, description=desc) for n, d, _, desc in ARGS]

    gps = Node(package=HW_PKG, executable='gps', name='gps_node', output='screen', additional_env=NODE_ENV,
               parameters=[{'dual': False, 'ntrip': p('ntrip')}])
    arduino = Node(package=HW_PKG, executable='arduino', name='arduino', output='screen', additional_env=NODE_ENV,
                   parameters=[{'drive_port': p('drive_port'), 'joy_port': p('joy_port')}])
    joy_gui = Node(package=HW_PKG, executable='joy', name='joy', output='screen', additional_env=NODE_ENV,
                   condition=IfCondition(LaunchConfiguration('use_joy_gui')))
    joy_nogui = Node(package=HW_PKG, executable='joy', name='joy', output='screen', additional_env=NODE_ENV,
                     arguments=['--nogui'], condition=UnlessCondition(LaunchConfiguration('use_joy_gui')))
    driving = Node(
        package=PKG, executable='driving', name='driving_node', output='screen', additional_env=NODE_ENV,
        parameters=[{n: p(n) for n in ('data_dir', 'cruise_pulse', 'min_pulse', 'heading_pulse', 'pulse_per_mps',
                                       'steer_trim_deg', 'steer_gain_l', 'steer_gain_r', 'run_min_quality',
                                       'antenna_base_m', 'wp_reach_m', 'cte_deviation_m')}],
    )
    mapping = Node(package=PKG, executable='mapping', name='mapping_node', output='screen', additional_env=NODE_ENV,
                   parameters=[{'data_dir': p('data_dir')}])
    # 구독만 하는 노드라 제어에 끼어들지 않는다(발행 토픽 없음)
    record = Node(package=HW_PKG, executable='record', name='record_drive', output='screen', additional_env=NODE_ENV,
                  parameters=[{'mode': 'drive', 'output_dir': p('record_dir')}],
                  condition=IfCondition(LaunchConfiguration('use_record')))
    estop = Node(package=HW_PKG, executable='estop', name='estop', output='screen', additional_env=NODE_ENV,
                 condition=IfCondition(LaunchConfiguration('use_estop')))
    hint = LogInfo(msg=f"\n  ▶ 메뉴는 별 터미널에서:  ros2 run {PKG} prompt\n")
    return LaunchDescription(decl + [gps, arduino, joy_gui, joy_nogui, driving, mapping, record, estop, hint])
