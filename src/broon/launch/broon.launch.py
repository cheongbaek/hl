#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
broon.launch.py ― 손으로 몰기 + 기록 (joy · arduino · record · estop) [broon]  (2026-11-19)
════════════════════════════════════════════════════════════════════════════════
    ros2 launch broon broon.launch.py
    ros2 launch broon broon.launch.py use_gui:=false         조이스틱 창 없이
    ros2 launch broon broon.launch.py use_record:=false      기록 없이
    ros2 launch broon broon.launch.py use_estop:=false       원격 E-STOP 없이 (인터넷이 없을 때)

띄우는 것 — ★차를 움직이는 데 필요한 것은 broon 만으로 된다★ (사용자 지정)
    broon/arduino   아두이노 판별("J," = 조이스틱, 나머지 = 구동) · /cmd_vel → 구동 우노 · 우노 출력 → /enc_pot
    broon/joy       조이스틱 → /cmd_vel (SWA ON 일 때만) · /joy318 + 화면
    broon/record    ★수동 기록★ — 띄우는 즉시 rec_<연도월일>_<시분초>.csv, Ctrl+C 로 끝내면 저장
                    (ros2 run broon record 와 같다. 런치에서는 '아무 키' 로 끝내기는 안 된다)
    broon/estop     원격 E-STOP — domichat 방 domi_fishing_broon 의 STOP · GO → /estop (웹 domiman.duckdns.org/estop)
                    arduino 가 /estop 1 이면 차를 세우고 /cmd_vel 을 막는다
GPS 는 띄우지 않는다 — 궤적까지 남기려면 별 터미널에서 ros2 run broon gps (안 띄우면 기록의 GPS 칸이 빈다).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NODE_ENV = {'PYTHONUNBUFFERED': '1', 'RCUTILS_LOGGING_BUFFERED_STREAM': '0'}


def generate_launch_description():
    decl = [
        DeclareLaunchArgument('drive_port', default_value='', description='구동 우노 포트. 비우면 판별'),
        DeclareLaunchArgument('joy_port', default_value='', description='조이스틱 메가 포트. 비우면 판별'),
        DeclareLaunchArgument('use_gui', default_value='true', description='조이스틱 화면'),
        DeclareLaunchArgument('use_record', default_value='true', description='수동 기록 (rec_*.csv)'),
        DeclareLaunchArgument('record_dir', default_value='', description='기록 폴더. 비우면 <ws>/src/broon/ros2bag'),
        DeclareLaunchArgument('use_estop', default_value='true', description='원격 E-STOP (domichat · 인터넷 필요)'),
    ]
    arduino = Node(package='broon', executable='arduino', name='arduino', output='screen', additional_env=NODE_ENV,
                   parameters=[{'drive_port': LaunchConfiguration('drive_port'),
                                'joy_port': LaunchConfiguration('joy_port')}])
    joy_gui = Node(package='broon', executable='joy', name='joy', output='screen', additional_env=NODE_ENV,
                   condition=IfCondition(LaunchConfiguration('use_gui')))
    joy_nogui = Node(package='broon', executable='joy', name='joy', output='screen', additional_env=NODE_ENV,
                     arguments=['--nogui'],
                     condition=UnlessCondition(LaunchConfiguration('use_gui')))
    record = Node(package='broon', executable='record', name='record', output='screen', additional_env=NODE_ENV,
                  parameters=[{'mode': 'manual', 'output_dir': LaunchConfiguration('record_dir')}],
                  condition=IfCondition(LaunchConfiguration('use_record')))
    estop = Node(package='broon', executable='estop', name='estop', output='screen', additional_env=NODE_ENV,
                 condition=IfCondition(LaunchConfiguration('use_estop')))
    return LaunchDescription(decl + [arduino, joy_gui, joy_nogui, record, estop])
