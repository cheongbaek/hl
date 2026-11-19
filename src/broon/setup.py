import os
from glob import glob

from setuptools import setup

package_name = 'broon'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cheongbaek',
    maintainer_email='cheongbaek@users.noreply.github.com',
    description='헤네스 브룬 T870 하드웨어 — 아두이노 · 조이스틱 · 기록 · GPS 노드 · GPS 배치 설정 · 장치 점검 · 정리',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            # ★차를 움직이는 것은 broon 만으로 된다★ — ros2 launch broon broon.launch.py = joy · arduino · record
            # 아두이노 판별("J," = 조이스틱, 나머지 = 구동) · /cmd_vel → 구동 우노(broon_1119) · 우노 출력 → /enc_pot
            'arduino     = broon.arduino:main',
            # 조이스틱(메가) → /cmd_vel · /joy318 + 화면. 포트는 arduino 가 /joy_port 로 알려 준다
            'joy         = broon.joy:main',
            # 주행 기록 CSV — 수동(rec_*) · 자율주행(mode:=drive, route_*)
            # 원격 E-STOP — domichat 방 domi_fishing_broon 의 STOP · GO · STATUS → /estop (웹 domiman.duckdns.org/estop)
            'estop       = broon.estop:main',
            'record      = broon.record:main',
            # GPS 노드 — 배치 설정의 칩 ID 로 전방(· 후방) 수신기를 골라 연다. dual:=true 면 두 대
            'gps         = broon.gps:main',
            # 전방 · 후방 GPS 배치 설정 GUI (ROS 노드 아님)
            'gps_setting = broon.gps_setting:main',
            # 런치 전 장치 점검 — GPS · 구동 아두이노 · 조이스틱 (ROS 노드 아님)
            'check       = broon.check:main',
            # 돌고 있는 ROS2 를 한 번에 끝낸다 (gold nxde kill 과 같은 기능, ROS 노드 아님)
            'kill        = broon.kill:main',
        ],
    },
)
