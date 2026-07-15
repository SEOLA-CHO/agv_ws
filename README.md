# Mecanum AGV ROS 2 Workspace

## Project Overview

This repository contains the ROS 2 and STM32 software for a four-wheel mecanum AGV.

## Hardware

- Native Ubuntu PC
- ROS 2
- STM32F446RE
- Four VESC motor controllers
- Four BLDC geared motors
- RPLIDAR C1

## VESC Mapping

- Front Left: CAN ID 2
- Front Right: CAN ID 1
- Rear Left: CAN ID 4
- Rear Right: CAN ID 3

## Repository Structure

- firmware/: STM32 micro-ROS firmware
- src/agv_sim/: software wheel simulator
- src/agv_odometry/: mecanum odometry
- src/agv_description/: URDF and TF
- src/agv_bringup/: LiDAR, SLAM and system launch
- docs/: shared specifications and team workflow

## ROS Interfaces

See:

- docs/interface_spec.md
- docs/team_workflow.md

## Build

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

## Project Status

- RPLIDAR C1 standalone test: completed
- /scan verification: completed
- wheel simulator: in progress
- micro-ROS: in progress
- odometry and TF: in progress
- SLAM integration: pending
