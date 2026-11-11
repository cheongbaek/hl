import os
from glob import glob

from setuptools import setup

package_name = 'white1'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cheongbaek',
    maintainer_email='cheongbaek@users.noreply.github.com',
    description='HL FMA 1/5카 GPS 경로추종 — 단일 GPS(전방 안테나), IMU 없음',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            # GPS · 아두이노 노드는 broon 패키지에 있다 — one_launch.py 가 package='broon' 으로 띄운다
            'driving = white1.driving:main',
            'mapping = white1.mapping:main',
            'prompt  = white1.prompt:main',
            'record  = white1.record:main',
        ],
    },
)
