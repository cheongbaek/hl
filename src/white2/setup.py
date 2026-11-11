import os
from glob import glob

from setuptools import setup

package_name = 'white2'

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
    description='HL FMA 1/5카 GPS 경로추종 — 듀얼 GPS(전방 + 후방 안테나), IMU 없음',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            # GPS · 아두이노 노드는 broon 패키지에 있다 — one_launch.py 가 package='broon' 으로 띄운다
            'driving = white2.driving:main',
            'mapping = white2.mapping:main',
            'prompt  = white2.prompt:main',
            'record  = white2.record:main',
        ],
    },
)
