# STM32F446RE build and flash handoff

This document is the minimum handoff procedure for building and flashing the
STM32 wheel-controller firmware from commit `f4fa109` or later on branch
`feature/stm32-f446re-microros`.

## Important build prerequisite

The prebuilt `libmicroros.a` currently checked into the branch predates
`agv_msgs`. Do not attempt the final CubeIDE build until the static library has
been regenerated. A correct generated tree contains both files below:

```text
micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include/agv_msgs/msg/wheel_commands.h
micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include/agv_msgs/msg/wheel_states.h
```

## Required software

- Git
- Docker Desktop with Linux containers, or Docker Engine on Linux
- STM32CubeIDE with STM32CubeF4 firmware package 1.27.1
- ST-LINK USB connection to the NUCLEO-F446RE

Ubuntu 22.04 or WSL2 is recommended for the shell steps. ROS 2 is not required
to build the static library or flash the MCU.

## 1. Clone the exact branch

```bash
git clone --branch feature/stm32-f446re-microros \
  https://github.com/SEOLA-CHO/agv_ws.git
cd agv_ws
git rev-parse --short HEAD
```

Confirm that the revision is `f4fa109` or a later reviewed commit.

## 2. Regenerate the micro-ROS static library

Start Docker, then run:

```bash
cd firmware/f446re_microros
./build_microros_library.sh --force
```

The script stages `src/agv_msgs`, builds the Humble micro-ROS library, and
restores the previous library automatically if generation fails.

Confirm these outputs before opening CubeIDE:

```bash
test -f micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/libmicroros.a
test -f micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include/agv_msgs/msg/wheel_commands.h
test -f micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include/agv_msgs/msg/wheel_states.h
grep -E 'agv_msgs/(WheelCommands|WheelStates).msg' \
  micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/available_ros2_types
```

Do not continue if any command fails.

## 3. Build in STM32CubeIDE

1. Open STM32CubeIDE.
2. Select **File > Import > General > Existing Projects into Workspace**.
3. Select `agv_ws/firmware/f446re_microros` as the project directory.
4. Select the **Release** build configuration.
5. Run **Project > Clean** and then **Project > Build Project**.
6. Confirm that there are no unresolved `agv_msgs` or typesupport symbols.
7. Confirm that `Release/f446re_microros.elf` was generated.

## 4. Flash safely

1. Turn OFF VESC motor power.
2. Raise all four wheels off the ground before later motor testing.
3. Connect the NUCLEO ST-LINK USB cable.
4. In CubeIDE, select **Run > Run As > STM32 C/C++ Application**. The ELF can
   also be programmed with STM32CubeProgrammer through ST-LINK.
5. Confirm that programming and verification both complete successfully.

Hardware configuration used by this firmware:

| Function | Configuration |
|---|---|
| Board | NUCLEO-F446RE |
| micro-ROS serial | USART2 PA2/PA3, 115200 baud |
| CAN1 | PB8 RX, PB9 TX, 500 kbit/s |
| Relay | PA8, active-low, initially ON |
| Wheel order | `[FL, FR, RL, RR]` |
| VESC IDs | `[2, 1, 4, 3]` |

Although the relay initializes ON to preserve the existing hardware behavior,
the motor task continuously commands zero until the Agent is connected and a
fresh valid wheel command arrives.

## 5. Post-flash communication check

On the ROS 2 Humble computer:

```bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 -b 115200 -v6
```

Confirm the interfaces:

```bash
ros2 node list
ros2 topic list | grep -E '/wheel_commands|/wheel_states|/relay_cmd'
ros2 topic echo --qos-reliability best_effort /wheel_states
```

Before any powered wheel test, request relay OFF and verify the non-blocking
zero-command shutdown sequence:

```bash
ros2 topic pub --once /relay_cmd std_msgs/msg/Bool "{data: false}"
```

## Report back

Send the following information to the firmware owner:

- checked-out commit SHA;
- static-library build result;
- CubeIDE build console result;
- generated ELF path and size;
- ST-LINK programming/verification result;
- whether the micro-ROS node and three topics appeared;
- any warnings, screenshots, or full error logs.
