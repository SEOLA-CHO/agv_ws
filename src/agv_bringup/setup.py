from glob import glob

from setuptools import setup


package_name = 'agv_bringup'


setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SEOLA CHO',
    maintainer_email='256262348+SEOLA-CHO@users.noreply.github.com',
    description='Integrated simulation and hardware mapping launch.',
    license='Apache-2.0',
)
