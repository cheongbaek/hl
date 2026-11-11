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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cheongbaek',
    maintainer_email='cheongbaek@users.noreply.github.com',
    description='헤네스 브룬 T870 하드웨어 — 아두이노 브리지 · GPS 노드 · GPS 배치 설정 · 장치 점검',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            # 아두이노(UNO R4 Minima) 브리지 — white1 · white2 런치가 띄운다. 새 4바이트 규약(가작성)
            'arduino     = broon.arduino:main',
            # GPS 노드 — 배치 설정의 칩 ID 로 전방(· 후방) 수신기를 골라 연다. dual:=true 면 두 대
            'gps         = broon.gps:main',
            # 전방 · 후방 GPS 배치 설정 GUI (ROS 노드 아님)
            'gps_setting = broon.gps_setting:main',
            # 런치 전 장치 점검 (ROS 노드 아님)
            'check       = broon.check:main',
        ],
    },
)
