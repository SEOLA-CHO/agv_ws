# Windows STM32CubeIDE build result

Date: 2026-07-15 (Asia/Seoul)

## Result

- STM32CubeIDE: 1.11.0
- Configuration: Debug
- Build: successful
- Errors: 0
- Warnings: 0
- Output: `Debug/f446re_microros.elf`

## Image size

```text
text     data     bss      total
75892    292      124944   201128 bytes
```

The 48 KiB FreeRTOS heap is part of BSS. The 12 KiB micro-ROS task stack and the micro-ROS runtime allocations are taken from that heap at runtime.

## Correction applied during build validation

The generated Humble transport API expects a `const uint8_t *` write buffer. `dma_transport.c` and the declaration in `microros_app.c` were aligned with that signature. The HAL call uses a local cast because STM32 HAL's DMA transmit API takes a non-const pointer.

## Remaining validation

1. Connect the NUCLEO-F446RE through the ST-LINK USB connector.
2. Flash `Debug/f446re_microros.elf`.
3. Pass ST-LINK/VCP to WSL using usbipd-win.
4. Start the Humble micro-ROS Agent on `/dev/ttyACM0` at 115200 baud.
5. Verify `/stm32_counter` with `ros2 topic echo`.

