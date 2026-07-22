from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'agv_control'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
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
    maintainer='SEOLA CHO',
    maintainer_email='256262348+SEOLA-CHO@users.noreply.github.com',
    description='Mecanum cmd_vel to fixed-order wheel command controller.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mecanum_controller = '
            'agv_control.mecanum_controller:main',
            'relative_motion_controller = '
            'agv_control.relative_motion_controller:main',
        ],
    },
)
