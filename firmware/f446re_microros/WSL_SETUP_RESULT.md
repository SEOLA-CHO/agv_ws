# WSL micro-ROS Setup Result

작성일: 2026-07-15 (Asia/Seoul)

## 최종 상태

- **micro-ROS 정적 라이브러리 생성: 성공**
- **Docker Engine 설치 및 daemon 검증: 성공**
- **ROS 2 Humble 및 개발 도구 검증: 성공**
- **micro-ROS Agent 소스 빌드 및 도움말 실행: 성공**
- **실제 보드/시리얼 연결: 미검증** (`/dev/ttyACM0`이 아직 WSL에 없음)

## 설치 환경

- Ubuntu: `Ubuntu 22.04.5 LTS`
- Kernel: `6.6.114.1-microsoft-standard-WSL2`
- WSL: WSL2, `wslinfo --wsl-version` 결과 `2.7.3.0`
- systemd: `/etc/wsl.conf`에 `systemd=true`, Docker service `active`
- Docker Engine/Server: `29.6.1` (공식 Docker Ubuntu 저장소)
- Docker storage driver: `overlayfs`
- ROS 2: Humble
  - `ros-humble-desktop 0.10.0-1jammy.20260423.142311`
  - `ros-humble-ros-base 0.10.0-1jammy.20260423.142225`
- `ros-dev-tools 1.0.1`
- micro-ROS Agent: `3.0.6`, source workspace `/home/gyun/microros_agent_ws`
- `usbutils 1:014-1build1`
- `binutils-arm-none-eabi 2.38-3ubuntu1+15build1`

`gyun` 사용자는 `docker`와 `dialout` 그룹에 추가됐다. 그룹 정보는 시스템 데이터베이스에 반영됐으며 기존 터미널에서는 새 로그인 또는 WSL 셸 재시작이 필요할 수 있다.

## 주요 실행 명령

Docker 공식 저장소와 도구 설치:

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg ros-dev-tools usbutils binutils-arm-none-eabi git python3-rosdep
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl enable --now docker
sudo usermod -aG docker,dialout gyun
docker run --rm hello-world
```

정적 라이브러리 빌드:

```bash
cd /mnt/c/Users/user/Documents/Codex/2026-07-10/sk/work/IDE_WS/f446re_microros
./build_microros_library.sh
```

현재 셸에는 새 docker 그룹을 즉시 적용하기 위해 실제 실행 시 `sg docker -c './build_microros_library.sh'`를 사용했다.

Agent 설치:

```bash
mkdir -p /home/gyun/microros_agent_ws/src
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git /home/gyun/microros_agent_ws/src/micro_ros_setup
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

`~/.bashrc`에는 `/opt/ros/humble` 및 기존 overlay 이후 `/home/gyun/microros_agent_ws/install/local_setup.bash`를 source하는 조건부 설정을 추가했다.

## libmicroros.a 검증

생성 경로:

```text
/mnt/c/Users/user/Documents/Codex/2026-07-10/sk/work/IDE_WS/f446re_microros/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/libmicroros.a
```

- 크기: `10,008,052 bytes`
- `file`: `current ar archive`
- archive 내부 object: `ELF32`, `ARM`, `Version5 EABI`
- ARM attributes: Cortex-M4/ARMv7E-M, `VFPv4-D16`, `Tag_ABI_VFP_args: VFP registers`
- 빌드 플래그: `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Os`
- 확인된 심볼:
  - `rclc_support_init`
  - `rclc_node_init_default`
  - `rmw_uros_set_custom_transport`
  - `rmw_uros_ping_agent`
  - `std_msgs__msg__Int32__init`
  - `rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__Int32`
- 확인된 헤더:
  - `include/rcl/rcl.h`
  - `include/rclc/rclc.h`
  - `include/rmw_microros/rmw_microros.h`
  - `include/std_msgs/msg/int32.h`

검증에는 `file`, `arm-none-eabi-ar`, `arm-none-eabi-readelf`, `arm-none-eabi-nm`을 사용했다.

## Agent 검증 및 실행 명령

검증:

```bash
source /opt/ros/humble/setup.bash
source /home/gyun/microros_agent_ws/install/local_setup.bash
ros2 pkg prefix micro_ros_agent
ros2 run micro_ros_agent micro_ros_agent --help
```

패키지 prefix는 `/home/gyun/microros_agent_ws/install/micro_ros_agent`로 확인됐고 도움말에 serial transport와 기본 baudrate `115200`이 표시됐다. 이 Agent 버전은 도움말을 출력한 뒤 종료 코드 `1`을 반환하지만, 실행 파일 발견과 로딩은 정상이다.

보드 연결 후 실행:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v6
```

## 문제와 해결 내용

1. Docker가 설치되어 있지 않았다.
   - Docker 공식 Ubuntu 저장소를 등록해 Engine 29.6.1을 설치했다.
   - daemon 활성화, `docker info`, `hello-world`, 비-root 그룹 실행을 확인했다.
2. 프로젝트에 STM32CubeIDE 생성 `.mk` 파일이 없어 upstream builder가 CPU/ABI 플래그를 추출할 수 없었다.
   - `.ioc`나 CubeMX 코드를 재생성하지 않고, `build_microros_library.sh`가 `.mk` 부재 시 STM32F446RE Release 플래그를 담은 임시 `.microros_cflags.mk`를 생성하도록 최소 수정했다.
   - 임시 파일은 `trap`으로 종료 시 자동 삭제되며 실제 삭제도 확인했다.
3. `build_microros_library.sh`는 처음부터 LF였다.
   - CRLF 변환은 불필요했다. 실행 비트가 설정되어 있다. Windows DrvFs mount가 Unix mode metadata를 보존하지 않아 `chmod 755` 후에도 표시값은 `777`이지만 실행에는 문제가 없다.
4. 공식 Humble apt 인덱스에 `ros-humble-micro-ros-agent`가 없었다.
   - 공식 `micro_ros_setup` Humble 브랜치를 이용해 Agent 3.0.6을 소스 빌드했다.
5. 기존 rosdep 설정에 제거된 Ubuntu `python3-rosdep2` 패키지의 `10-debian.list`가 남아 존재하지 않는 `/usr/share/python3-rosdep2/debian.yaml`을 참조했다.
   - 설정을 `10-debian.list.disabled`로 보존·비활성화한 뒤 `rosdep update`와 의존성 설치를 성공시켰다.
6. micro-ROS library 빌드에서 일부 upstream 컴파일 경고가 발생했다.
   - 오류는 없었으며 69개 패키지가 모두 빌드되고 archive 및 필수 심볼을 별도로 검증했다.

## 남은 Windows/USB 작업

현재 `lsusb`에 전달된 장치가 없고 `/dev/ttyACM0`도 존재하지 않는다. 보드를 Windows에 연결한 다음 Windows 관리자 PowerShell에서 설치된 `usbipd-win` 버전에 맞춰 장치를 WSL에 전달해야 한다. 일반적인 흐름은 다음과 같다.

```powershell
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl --busid <BUSID>
```

그 후 WSL에서 다음을 확인한다.

```bash
lsusb
ls -l /dev/ttyACM0
```

보드 플래시, Windows STM32CubeIDE 전체 링크/빌드, 실제 ROS topic 수신은 USB 전달과 펌웨어 플래시 후 수행해야 한다. `f446re_microros.ioc`와 CubeMX 생성 코드는 이번 작업에서 변경하지 않았다.
