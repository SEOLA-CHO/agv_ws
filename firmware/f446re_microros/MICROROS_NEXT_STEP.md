# micro-ROS integration status

The CubeMX configuration now contains:

- FreeRTOS CMSIS-RTOS v2
- `microRosTask`, priority Normal, 3000 words (12000 bytes) stack
- TIM1 as the HAL timebase
- USART2 at the board-default 115200 baud
- USART2 RX: DMA1 Stream 5, circular, very-high priority
- USART2 TX: DMA1 Stream 6, normal, very-high priority
- USART2 and DMA IRQ preemption priority 5

## Completed

- CubeMX code generation
- FreeRTOS task repair and `microros_app_run()` connection
- 48 KiB FreeRTOS heap
- Official Humble STM32 integration sources
- CubeIDE include path and linker configuration for Debug and Release

## Required one-time WSL action

From Ubuntu 22.04 on WSL:

```bash
cd /mnt/c/Users/user/Documents/Codex/2026-07-10/sk/work/IDE_WS/f446re_microros
chmod +x build_microros_library.sh
./build_microros_library.sh
```

Docker Desktop must be running with WSL integration enabled. After the script succeeds, return to Codex for the headless CubeIDE build and compiler/linker error correction.
