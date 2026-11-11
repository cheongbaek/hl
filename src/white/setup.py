import os
from glob import glob

from setuptools import setup

package_name = 'white'

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
    description='HL FMA 1/5카 GPS 경로추종',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'gps     = white.gps:main',
            'arduino = white.arduino:main',
            'driving = white.driving:main',
            'mapping = white.mapping:main',
            'prompt  = white.prompt:main',
            'check   = white.check:main',
            'record  = white.record:main',
        ],
    },
)
