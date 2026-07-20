# 노트북 Codex용 전체 작업 프롬프트

아래 구분선 안의 내용을 노트북에서 새 Codex 대화에 그대로 붙여 넣는다.

---

너는 `SEOLA-CHO/agv_ws`의 ROS 2 Humble 기반 메카넘 AGV 통합을 끝까지
검증하는 개발 에이전트다. 설명만 하지 말고, 안전하게 실행할 수 있는 작업은 직접
수행하고 실제 명령 출력으로 확인하라. GUI, 관리자 권한, USB 연결, 펌웨어 플래시,
물리 시험처럼 내가 직접 해야 하는 단계에서는 정확히 한 가지 즉시 실행할 작업과
그 직후 확인할 결과를 알려주고 기다려라.

## 노트북과 STM32의 역할

이 노트북이 AGV의 상위 제어기다. 최종 실물 운용에서는 노트북이 다음 프로세스를
담당한다.

- micro-ROS Agent
- `agv_control`: `/cmd_vel -> /wheel_commands`
- `agv_odom`: `/wheel_states -> /odom` 및 `odom -> base_footprint`
- LiDAR/센서 driver
- robot_state_publisher
- SLAM Toolbox, RViz, 이후 Nav2/상위 명령

STM32F446RE는 하위 실시간 제어기다. STM32는 `/wheel_commands`를 받아 VESC CAN을
제어하고 `/wheel_states`를 노트북으로 돌려준다. STM32에 `/cmd_vel` 메카넘 계산,
오도메트리, TF, SLAM을 넣지 마라.

두 운용 모드를 명확히 분리한다.

- 시뮬레이션 모드: 노트북에서 `agv_sim`이 STM32를 대신함
- 실물 모드: `agv_sim`을 완전히 종료하고 micro-ROS Agent를 통해 STM32를 사용함

`agv_sim`과 STM32가 동시에 `/wheel_states`를 발행하지 않게 하라.

## 저장소와 기준 브랜치

- 저장소: `https://github.com/SEOLA-CHO/agv_ws.git`
- 시작 브랜치: `codex/adapt-sim-odom`
- 기준이 된 펌웨어 브랜치: `feature/stm32-f446re-microros`
- 먼저 `git fetch --all --prune`을 실행하고 원격 브랜치와 최신 커밋을 확인하라.
- 저장소가 없으면 영문/공백 없는 경로에 clone하고, 있으면 재복제하지 말고 기존
  checkout의 변경 상태를 먼저 확인하라.
- 기존 사용자 변경은 절대 덮어쓰지 말고, 작업이 필요하면 별도 통합 브랜치를 만들어라.

Windows용 펌웨어 checkout과 WSL용 ROS checkout을 분리한다. CubeIDE에서
`\\wsl$` 경로를 직접 import하거나 WSL native workspace에서 Windows CubeIDE를
실행하지 마라.

- Windows checkout 예시: `C:\AGV\agv_ws`
  - CubeIDE 프로젝트 import, micro-ROS static library 생성 결과, 펌웨어 빌드에 사용
- WSL checkout: `~/agv_ws`
  - colcon, ROS 노드, Agent, SLAM 실행에 사용

두 checkout은 모두 같은 `codex/adapt-sim-odom` 커밋이어야 한다. 서로 다른 checkout의
수정 내용을 수동 복사하지 말고 Git 커밋으로 동기화하라.

`Windows PowerShell`에서 Windows checkout을 준비한다. `C:\AGV\agv_ws`가 이미
있다면 다시 clone하지 말고 먼저 `git status`와 remote를 확인하라.

```powershell
New-Item -ItemType Directory -Force C:\AGV | Out-Null
Set-Location C:\AGV
git clone https://github.com/SEOLA-CHO/agv_ws.git
Set-Location C:\AGV\agv_ws
git fetch --all --prune
git switch --track origin/codex/adapt-sim-odom
git status --short --branch
git log -1 --oneline
```

`WSL Ubuntu 터미널`에서는 Linux filesystem 안에 별도로 준비한다. 실제 저장 위치는
먼저 찾아서 확인하고 임의로 가정하지 마라.

```bash
git clone https://github.com/SEOLA-CHO/agv_ws.git
cd ~/agv_ws
git fetch --all --prune
git switch --track origin/codex/adapt-sim-odom
git status --short --branch
git log -1 --oneline
```

각 checkout에 이미 로컬 브랜치가 있다면 그 checkout 안에서 다음을 사용하라.

```bash
git switch codex/adapt-sim-odom
git pull --ff-only
```

## 절대 변경하면 안 되는 인터페이스 계약

- `/cmd_vel`: `geometry_msgs/msg/Twist`
- `/wheel_commands`: `agv_msgs/msg/WheelCommands`
- `WheelCommands.velocity_rad_s`: `[FL, FR, RL, RR]` 고정 순서
- `/wheel_states`: `agv_msgs/msg/WheelStates`
- `WheelStates.velocity_rad_s`, `erpm`, `tachometer_counts`, `online`도 모두
  `[FL, FR, RL, RR]` 고정 순서
- CAN 매핑: `[FL, FR, RL, RR] -> [2, 1, 4, 3]`
- `/wheel_commands`, `/wheel_states` QoS: Best Effort, Volatile, Keep Last 1
- 바퀴 반지름 `0.0762 m`, 축간거리 `0.445 m`, 윤거 `0.400 m`
- 최대 바퀴 속도 `16.36 rad/s`
- F446의 `WheelStates.velocity_rad_s`는 이미 전진 양수로 정규화되어 있으므로
  오도메트리에서 오른쪽 바퀴 부호를 다시 뒤집지 마라.
- 제어기의 `rotation_direction=-1.0`과 오도메트리의
  `angular_z_scale=-1.0` 조합을 유지하되 실제 CCW 시험으로 최종 확인하라.

이번 작업에서는 새로 맞춘 `agv_odom`을 사용한다. 기존 F446 브랜치의
`agv_odometry`를 동시에 실행하지 마라. 두 노드를 함께 실행하면 `/odom`과
`odom -> base_footprint` TF가 중복된다. 기존 패키지를 임의로 삭제하지는 마라.

TF 소유권은 다음과 같다.

- `agv_odom`: `odom -> base_footprint`
- robot description 또는 robot_state_publisher: 로봇 내부 정적/관절 TF
- SLAM: `map -> odom`
- `agv_odom`이 `map -> odom`을 발행하게 만들지 마라.

## 1단계: 노트북 환경 조사와 준비

명령을 실행할 위치를 항상 다음 셋 중 하나로 표시하라.

1. `Windows 관리자 PowerShell`
2. `Docker Desktop 설정 화면`
3. `WSL Ubuntu 터미널`

먼저 설치 상태만 조사하고, 이미 있는 프로그램을 무조건 재설치하지 마라.

`Windows 관리자 PowerShell`에서 확인:

```powershell
git --version
wsl --status
wsl --list --verbose
winget list --id Docker.DockerDesktop
usbipd list
```

WSL/Ubuntu가 없으면 `Windows 관리자 PowerShell`에서 먼저 가능한 배포판을 확인하라.

```powershell
wsl --list --online
```

그 결과에 Ubuntu 22.04가 있을 때만 정확한 이름으로 설치하라. 일반적인 명령은
다음과 같지만 실제 목록을 우선한다.

```powershell
wsl --install -d Ubuntu-22.04
```

재부팅이 요구되면 다른 설치를 이어가지 말고 재부팅 후 `wsl --list --verbose`에서
Ubuntu가 WSL 2인지 확인한다.

Docker Desktop이 없다면 `Windows PowerShell`에서 다음 설치를 사용자 승인 후
실행하라.

```powershell
winget install --exact --id Docker.DockerDesktop
```

설치 후 Docker Desktop을 한 번 직접 실행하고 초기 약관/WSL backend 설정을 마쳐야
한다. Docker Desktop을 사용할 경우 WSL 안에 별도의 Docker Engine을 중복 설치하지
마라.

`Docker Desktop 설정 화면`에서 확인:

- WSL 2 engine 활성화
- 사용하는 Ubuntu 배포판의 WSL Integration 활성화
- Docker Engine이 실제로 실행 중인지 확인

`WSL Ubuntu 터미널`에서 확인:

```bash
lsb_release -a
docker info
test -f /opt/ros/humble/setup.bash && echo ROS_HUMBLE_FOUND
which colcon || true
which rosdep || true
```

ROS 2가 없다면 Ubuntu 버전이 22.04인지 확인한 후 공식 ROS 2 Humble 설치 절차를
사용하라. 기존 apt source와 shell 설정을 보존하고 중복 항목을 만들지 마라.

STM32CubeIDE가 없으면 ST 공식 설치 프로그램으로 설치해야 한다. 이 다운로드와
설치 약관/관리자 승인은 사용자에게 한 단계씩 요청하라. 설치 후 다음도 확인한다.

- NUCLEO-F446RE device support
- STM32CubeF4 firmware package `1.27.1`
- ST-LINK driver/programmer 접근 가능
- 영문/공백 없는 새 workspace 경로 사용, 예: `C:\STM32_WS\agv_f446`

## 2단계: Windows checkout과 CubeIDE 프로젝트 복구

펌웨어 프로젝트의 실제 경로는 다음이다.

```text
C:\AGV\agv_ws\firmware\f446re_microros
```

이 디렉터리에는 이미 다음 파일이 있으므로 새 STM32 프로젝트를 만들거나 `.ioc`만으로
프로젝트를 재생성하지 마라.

```text
.project
.cproject
f446re_microros.ioc
```

`STM32CubeIDE GUI`에서 사용자가 수행할 순서:

1. `File > Switch Workspace > Other...`
2. 새 workspace로 `C:\STM32_WS\agv_f446` 선택
3. `File > Import > General > Existing Projects into Workspace`
4. root directory로 `C:\AGV\agv_ws\firmware\f446re_microros` 선택
5. 발견된 프로젝트 이름이 정확히 `f446re_microros`인지 확인
6. `Copy projects into workspace`는 선택하지 않음
7. import 후 `Properties > Resource > Location`이 위 Git checkout을 가리키는지 확인

프로젝트가 검색되지 않으면 새 프로젝트를 만들지 말고 `.project` 존재 여부, checkout
브랜치, 경로 길이와 문자를 다시 확인하라. 같은 이름의 오래된 프로젝트가 workspace에
있으면 어느 복사본이 활성인지 Resource Location으로 확인한 후 사용자에게 선택을
요청하라.

아직 static library를 재생성하지 않았다면 import 직후 Build/Generate Code를 실행하지
마라. 특히 `.ioc`를 열었을 때 external changes나 code generation을 요구하더라도 현재
사용자 코드를 덮어쓸 수 있으므로 먼저 Git 상태와 `USER CODE BEGIN/END` 보존 여부를
확인한다.

## 3단계: Docker로 CubeIDE checkout의 micro-ROS library 재생성

Docker Desktop과 Ubuntu WSL Integration이 실제로 동작한 뒤, `WSL Ubuntu 터미널`에서
Windows CubeIDE checkout을 대상으로 실행한다. WSL용 `~/agv_ws`에서 생성하면
CubeIDE가 사용하는 Windows checkout에는 반영되지 않으므로 경로를 혼동하지 마라.

```bash
cd /mnt/c/AGV/agv_ws
git status --short --branch
git rev-parse --short HEAD
docker info
cd firmware/f446re_microros
bash ./build_microros_library.sh --force
```

스크립트가 실패하면 기존 library를 보존/복구하도록 구현되어 있으므로 수동으로 vendor
디렉터리를 삭제하지 마라. 성공 후 반드시 다음을 확인한다.

```bash
test -f micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/libmicroros.a
test -f micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include/agv_msgs/msg/wheel_commands.h
test -f micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include/agv_msgs/msg/wheel_states.h
grep -E 'agv_msgs/(WheelCommands|WheelStates).msg' \
  micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/available_ros2_types
```

네 검사가 모두 성공한 후에만 CubeIDE에서 `Release` configuration을 선택하고
`Project > Clean`, `Project > Build Project`를 실행한다. Console의 실제 종료 결과와
`Release/f446re_microros.elf` 존재를 확인한다. GUI에서 열려 있는 같은 workspace를
동시에 headless CLI build하지 마라.

## 4단계: 상위 제어기 ROS 패키지 실제 빌드

`WSL Ubuntu 터미널`에서 실행하라.

```bash
cd <실제_agv_ws_경로>
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
unset ROS_NAMESPACE
colcon list
colcon build --symlink-install \
  --packages-select agv_msgs agv_control agv_sim agv_odom
source install/setup.bash
ros2 pkg prefix agv_msgs
ros2 pkg prefix agv_control
ros2 pkg prefix agv_sim
ros2 pkg prefix agv_odom
colcon test --packages-select agv_control agv_sim agv_odom
colcon test-result --verbose
```

빌드 전 기존 `build`, `install`, `log`를 무조건 삭제하지 마라. 오류가 캐시 때문이라는
근거가 생겼을 때만 정확한 대상 경로를 확인하고 별도 백업 또는 패키지 단위 정리를
사용하라.

다음이 모두 확인돼야 빌드 완료로 보고한다.

- 위 `colcon build`가 실제로 종료 코드 0
- 네 패키지의 `ros2 pkg prefix`가 현재 workspace의 `install`을 가리킴
- `colcon test-result --verbose`에 실패 없음
- 정적 Python 테스트만 실행하고 ROS 빌드 성공이라고 보고하지 않음

## 5단계: description 없이 상위 제어기 파이프라인 검증

각 터미널에서 공통으로 다음을 실행하라.

```bash
cd <실제_agv_ws_경로>
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
unset ROS_NAMESPACE
```

그 후 서로 다른 `WSL Ubuntu 터미널`에서 실행한다.

터미널 A:

```bash
ros2 run agv_control mecanum_controller
```

터미널 B:

```bash
ros2 launch agv_sim wheel_simulator.launch.py
```

터미널 C:

```bash
ros2 launch agv_odom mecanum_odometry.launch.py
```

터미널 D에서 계약을 확인한다.

```bash
ros2 topic type /wheel_commands
ros2 topic type /wheel_states
ros2 topic info --verbose /wheel_commands
ros2 topic info --verbose /wheel_states
ros2 topic hz /wheel_states
ros2 topic echo /odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

다음 명령은 한 번에 하나씩 실행하고 2~3초 관찰한 후 `Ctrl+C`로 중지한다. 각 시험
사이에 0속도를 보내거나 0.5초 이상 기다려 timeout 정지를 확인하라.

전진 시험:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.20, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

좌측 횡이동 시험:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.20, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

반시계 회전 시험:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.30}}"
```

기대 결과:

- 전진: 바퀴 명령이 모두 양수이고 `/odom`의 x가 증가
- 좌측 횡이동: 바퀴 패턴이 대략 `[-, +, +, -]`이고 `/odom`의 y가 증가
- 반시계 회전: 현재 보정 기준 바퀴 패턴이 대략 `[+, -, +, -]`이고
  `/odom`의 yaw/angular.z가 양수
- 시뮬레이터 `/wheel_states.online` 네 값이 모두 `true`
- 명령 중단 후 `/wheel_commands`가 0이 되고 시뮬레이터 속도가 감속하여 0으로 수렴
- `/wheel_commands`와 `/wheel_states`의 실제 QoS가 Best Effort

값을 추측하지 말고 topic echo 결과를 저장하여 보고하라. 방향이 틀리면 임의로 여러
부호를 동시에 바꾸지 말고, 입력 명령·바퀴 명령·바퀴 상태·오도메트리 순서로 어느
경계에서 처음 반전되는지 증거를 잡아 한 변수만 수정하라.

## 6단계: agv_description과 전체 시뮬레이션 통합

현재 `agv_odom/launch/full_sim.launch.py`는 `agv_description`을 요구한다. 원격
브랜치 중 description/URDF가 있는 브랜치를 먼저 조사하되 현재 브랜치에 통째로
blind merge하지 마라. 다음을 확인하라.

- 실제 패키지 이름이 `agv_description`인지
- `urdf/agv.urdf.xacro`가 존재하는지
- `base_footprint`, `base_link`, 바퀴 joint와 센서 frame 이름
- 바퀴 축과 joint 회전 방향
- 현재 작업과 충돌하는 `agv_control`, `agv_sim`, `agv_odom` 파일이 있는지

필요한 description 변경만 별도 통합 브랜치에 가져오고 다시 빌드한 뒤 실행하라.

```bash
ros2 launch agv_odom full_sim.launch.py
ros2 node list
ros2 topic list -t
ros2 run tf2_tools view_frames
```

반드시 controller, simulator, odometry가 각각 하나만 실행되는지 확인한다. TF tree에서
중복 publisher, 끊긴 frame, 좌우/전후가 뒤집힌 wheel joint가 없어야 한다.

## 7단계: SLAM 준비와 실행

바퀴 시뮬레이터만으로는 `/scan`이 생기지 않으므로 SLAM 성공이라고 판단하지 마라.
팀 브랜치의 LiDAR driver, Gazebo sensor plugin 또는 검증용 rosbag 중 실제 `/scan`
source가 무엇인지 조사하라.

SLAM 실행 전 다음 게이트를 모두 통과해야 한다.

- `/scan` 타입이 `sensor_msgs/msg/LaserScan`
- LaserScan의 `header.frame_id`가 TF tree에 존재
- `odom -> base_footprint -> ... -> laser_frame` 연결이 끊기지 않음
- `/odom` timestamp와 scan timestamp가 합리적이며 오래된 데이터가 아님
- `odom -> base_footprint` publisher가 오직 `agv_odom` 하나
- `map -> odom` publisher가 SLAM 노드 하나
- 정지 상태에서 odom과 scan이 폭주하지 않음

그 다음 ROS 2 Humble의 `slam_toolbox` 설정을 현재 frame/topic에 맞춰 실행한다.
최소한 다음 항목을 실제 설정 파일과 launch에서 확인하라.

- `scan_topic`
- `map_frame`
- `odom_frame`
- `base_frame`
- `use_sim_time`
- transform publish period 및 scan queue/tolerance

직선, 횡이동, 회전 후 원점 근처 복귀를 저속으로 시험하고 RViz에서 map, scan, TF,
odom을 함께 확인한다. map 저장까지 실제로 성공해야 SLAM 완료로 보고한다.

## 8단계: 상위 제어기 노트북을 실제 STM32에 연결하는 안전 절차

실물 시험은 내가 명시적으로 하드웨어 시험을 시작하겠다고 할 때만 진행하라.
시뮬레이터와 STM32가 동시에 `/wheel_states`를 발행하지 않게 `agv_sim`을 종료한다.

안전 순서:

1. 바퀴를 지면에서 띄우고 즉시 전원 차단 수단을 준비
2. VESC 구동 전원 OFF 상태에서 보드/Agent 통신부터 검증
3. micro-ROS 생성 헤더에 `WheelCommands`, `WheelStates`가 모두 있는지 확인
4. 필요하면 Docker로 micro-ROS static library 재생성
5. CubeIDE Clean/Release Build의 실제 성공 로그 확인
6. ST-LINK program/verify 확인
7. Agent 연결과 `/base_controller`, `/wheel_states` 확인
8. 네 바퀴 `online=true` 확인 후 저속 한 바퀴씩 시험
9. 방향·ID 매핑 확인 후에만 네 바퀴 통합 시험

USB 장치 경로를 `/dev/ttyACM0`으로 가정하지 마라.

`usbipd`가 없다면 `Windows 관리자 PowerShell`에서 사용자 승인 후 설치하고 재확인한다.

```powershell
winget install --interactive --exact --id dorssel.usbipd-win
usbipd list
```

ST-LINK USB 장치는 CubeIDE와 WSL이 동시에 소유할 수 없다고 가정한다. CubeIDE에서
flash할 때는 먼저 Windows가 소유하도록 한다.

`Windows 관리자 PowerShell`:

```powershell
usbipd list
usbipd detach --busid <실제_BUSID>
```

그 상태에서 CubeIDE의 `Run > Run As > STM32 C/C++ Application` 또는 ST-LINK
programming을 수행하고 program/verify 결과를 확인한다. flash가 끝난 뒤 debug 세션과
ST-LINK를 사용하는 Windows 프로그램을 종료한 다음에만 WSL로 넘긴다.

`Windows 관리자 PowerShell`:

```powershell
usbipd list
usbipd bind --busid <실제_BUSID>
usbipd attach --wsl --busid <실제_BUSID>
```

`WSL Ubuntu 터미널`:

```bash
lsusb
ls -l /dev/ttyACM* 2>/dev/null || true
ls -l /dev/serial/by-id/ 2>/dev/null || true
```

권한 오류가 있을 때만 실제 장치 group을 확인하고 `dialout` 추가가 필요한지 판단한다.
group 변경 후에는 WSL shell을 완전히 다시 열어 적용 여부를 확인한다.

상위 제어기에는 micro-ROS Agent가 실제 설치되어 있어야 한다.

```bash
source /opt/ros/humble/setup.bash
ros2 pkg prefix micro_ros_agent
```

패키지가 없다면 먼저 ROS apt repository 상태를 확인하고 제공되는 경우 다음을 사용한다.

```bash
sudo apt update
apt-cache policy ros-humble-micro-ros-agent
sudo apt install ros-humble-micro-ros-agent
ros2 pkg prefix micro_ros_agent
```

apt package가 제공되지 않을 때만 공식 micro-ROS Humble source-build 절차를 사용한다.
오래된 블로그 명령을 그대로 쓰지 말고 공식 `micro_ros_setup`의 `humble` branch를
확인하며, `rosdep install`, `colcon build`, `ros2 pkg prefix micro_ros_agent`의 실제
성공 결과를 남긴다.

장치가 하나일 때도 먼저 실제 by-id 값을 출력한다. 여러 장치가 있으면 자동으로 첫
장치를 고르지 말고 NUCLEO ST-LINK serial을 식별한다.

```bash
ls -l /dev/serial/by-id/
```

선택한 실제 경로로 Agent를 실행한다.

```bash
source /opt/ros/humble/setup.bash
source ~/agv_ws/install/setup.bash
export ROS_DOMAIN_ID=0
unset ROS_NAMESPACE
agent_dev='/dev/serial/by-id/<실제_STLINK_장치>'
test -e "$agent_dev"
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev "$agent_dev" -b 115200 -v6
```

Agent 로그에서 session/entity 생성이 확인된 다음, 별도 WSL 터미널에서 상위 제어기
노드만 실행한다. `agv_sim` 프로세스가 없는지 먼저 확인한다.

```bash
ros2 node list
pgrep -af 'wheel_simulator|agv_sim' || true
ros2 run agv_control mecanum_controller
```

다른 WSL 터미널:

```bash
ros2 launch agv_odom mecanum_odometry.launch.py
```

VESC 전원 OFF 상태에서도 `/base_controller`와 topic/type/QoS가 보이는지 먼저
확인한다. 이후 relay OFF 명령과 zero command를 확인한 다음에만 사용자 승인을 받고
VESC 전원을 인가한다.

```bash
ros2 node info /base_controller
ros2 topic info --verbose /wheel_commands
ros2 topic info --verbose /wheel_states
ros2 topic pub --once /relay_cmd std_msgs/msg/Bool "{data: false}"
ros2 topic echo --qos-reliability best_effort /wheel_states
```

가능하면 `/dev/serial/by-id/...`의 안정적인 경로로 Agent를 실행한다. 모든 ROS
터미널에서 `ROS_DOMAIN_ID=0`, `unset ROS_NAMESPACE`를 적용한다. `ros2 topic pub`이
성공했다는 사실만으로 STM32 callback이나 CAN `SET_RPM`이 실행됐다고 결론 내리지
말고 Agent 로그, `/wheel_states`, online 상태, ERPM, tachometer, 가능하면 CAN frame을
함께 확인하라.

## 완료 보고 형식

각 단계가 끝날 때 다음을 간결하게 보고하라.

- 실행한 환경: Windows 관리자 PowerShell / Docker Desktop / WSL Ubuntu / CubeIDE
- 실행한 핵심 명령
- 종료 코드와 실제 관찰값
- 변경한 파일
- 통과한 완료 조건
- 아직 실행하지 못한 항목과 정확한 이유
- 다음 한 가지 즉시 실행할 작업 및 그 결과 확인 방법

실제로 실행하지 않은 colcon build, CubeIDE build, 플래시, Agent 연결, 모터 회전,
SLAM map 생성을 성공했다고 표현하지 마라. 안전하고 범위가 명확한 작업은 질문만
반복하지 말고 직접 진행하되, 관리자 승인·GUI 클릭·USB 연결·전원 인가·물리 움직임은
내 확인을 받고 진행하라.

---
