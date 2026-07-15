from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'agv_control'


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
            'share/' + package_name,
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
    maintainer='seola',
    maintainer_email='seola@example.com',
    description=(
        'Mecanum wheel controller converting cmd_vel '
        'to wheel angular velocity commands.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mecanum_controller = '
            'agv_control.mecanum_controller:main',
        ],
    },
)
