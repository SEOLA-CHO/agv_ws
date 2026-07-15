#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
library_dir="${project_dir}/micro_ros_stm32cubemx_utils/microros_static_library_ide"
library_file="${library_dir}/libmicroros/libmicroros.a"
fallback_flags_file=""

cleanup() {
  if [[ -n "${fallback_flags_file}" ]]; then
    rm -f "${fallback_flags_file}"
  fi
}
trap cleanup EXIT

if ! command -v docker >/dev/null 2>&1; then
  echo "docker was not found. Start Docker Desktop and enable WSL integration." >&2
  exit 1
fi

docker info >/dev/null

# The upstream builder extracts the target ABI from STM32CubeIDE-generated
# *.mk files.  This project has not been built by CubeIDE yet, so provide the
# equivalent STM32F446RE Release flags temporarily without regenerating code.
if ! find "${project_dir}" -type f -name '*.mk' -print -quit | grep -q .; then
  fallback_flags_file="${project_dir}/.microros_cflags.mk"
  printf '%s\n' \
    'CFLAGS = -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Os -DUSE_HAL_DRIVER -DSTM32F446xx' \
    > "${fallback_flags_file}"
fi

docker pull microros/micro_ros_static_library_builder:humble
docker run --rm \
  -v "${project_dir}:/project" \
  --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide \
  microros/micro_ros_static_library_builder:humble

if [[ ! -f "${library_file}" ]]; then
  echo "Builder finished but the library was not created: ${library_file}" >&2
  exit 1
fi

echo "micro-ROS library ready: ${library_file}"
