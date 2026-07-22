from glob import glob
import os

from setuptools import find_packages, setup


package_name = "agv_sim"


setup(
    name=package_name,
    version="0.0.0",

    packages=find_packages(
        exclude=["test"],
    ),

    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml"],
        ),
        (
            os.path.join(
                "share",
                package_name,
                "launch",
            ),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join(
                "share",
                package_name,
                "config",
            ),
            glob("config/*.yaml"),
        ),
    ],

    install_requires=[
        "setuptools",
    ],

    zip_safe=True,

    maintainer="go",
    maintainer_email="go@example.com",

    description=(
        "Mock wheel simulator for ROS 2 "
        "odometry development."
    ),

    license="Apache-2.0",

    tests_require=[
        "pytest",
    ],

    entry_points={
        "console_scripts": [
            (
                "wheel_simulator = "
                "agv_sim.wheel_simulator_node:main"
            ),
        ],
    },
)
