import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'agv_odom'


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            os.path.join('share', package_name),
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AXEL',
    maintainer_email='axel@example.com',
    description='Mecanum odometry for AXEL AGV',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mecanum_odometry = agv_odom.mecanum_odometry:main',
        ],
    },
)