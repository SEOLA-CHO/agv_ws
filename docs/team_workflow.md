# Team Git Workflow

## 1. Main Branch Rule

Do not develop directly on main.

All work must be done on a feature branch and merged through a Pull Request.

## 2. Branch Names

- feature/micro-ros
- feature/odometry-tf
- feature/wheel-simulator
- feature/slam-bringup
- chore/project-setup

## 3. Assigned Paths

### micro-ROS Team

- firmware/stm32_agv_microros/
- micro-ROS related documentation

### Odometry and TF Team

- src/agv_odometry/
- src/agv_description/

### Wheel Simulator Team

- src/agv_sim/

### SLAM and Integration Team

- src/agv_bringup/

### Shared Files

The following files require team agreement before modification:

- docs/interface_spec.md
- README.md
- .gitignore

## 4. Before Starting Work

Run:

git switch main
git pull origin main
git switch <your-branch>
git merge main

## 5. Before Committing

Run:

git status
git diff
colcon build --symlink-install
colcon test
colcon test-result --verbose

## 6. Commit Message Examples

- feat: add wheel simulator node
- feat: add mecanum odometry node
- feat: add micro-ROS wheel state publisher
- fix: correct rear wheel velocity sign
- docs: add ROS interface specification
- test: add mecanum kinematics tests

## 7. Pull Request Requirements

A Pull Request must include:

- implementation summary
- changed files
- build result
- test result
- hardware test status
- unresolved parameters
- screenshots or logs when applicable

## 8. Prohibited Actions

Do not use:

git push --force

Do not commit:

- build/
- install/
- log/
- Debug/
- Release/
- __pycache__/
