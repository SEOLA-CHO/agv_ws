#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${project_dir}/../.." && pwd)"
library_dir="${project_dir}/micro_ros_stm32cubemx_utils/microros_static_library_ide"
library_file="${library_dir}/libmicroros/libmicroros.a"
required_header="${library_dir}/libmicroros/include/agv_msgs/msg/wheel_commands.h"
required_state_header="${library_dir}/libmicroros/include/agv_msgs/msg/wheel_states.h"
custom_package_source="${repo_root}/src/agv_msgs"
custom_package_root="${project_dir}/microros_component/extra_packages"
staged_custom_package="${custom_package_root}/agv_msgs"
backup_library_dir="${library_dir}/.libmicroros.backup"
fallback_flags_file=""
force_rebuild=false
build_succeeded=false
backup_created=false
library_replacement_started=false

if [[ "${1:-}" == "--force" ]]; then
  force_rebuild=true
elif [[ $# -ne 0 ]]; then
  echo "Usage: $0 [--force]" >&2
  exit 2
fi

cleanup() {
  status=$?
  trap - EXIT
  if [[ -n "${fallback_flags_file}" ]]; then
    rm -f "${fallback_flags_file}"
  fi
  rm -rf "${staged_custom_package}"
  rmdir "${custom_package_root}" 2>/dev/null || true
  rmdir "$(dirname "${custom_package_root}")" 2>/dev/null || true

  if [[ "${build_succeeded}" == true ]]; then
    if [[ "${backup_created}" == true && -d "${backup_library_dir}" ]]; then
      rm -rf "${backup_library_dir}"
    fi
  elif [[ "${library_replacement_started}" == true ]]; then
    rm -rf "${library_dir}/libmicroros"
    if [[ "${backup_created}" == true && -d "${backup_library_dir}" ]]; then
      mv "${backup_library_dir}" "${library_dir}/libmicroros"
      echo "micro-ROS build failed; restored the previous library." >&2
    fi
  fi
  exit "${status}"
}
trap cleanup EXIT

if [[ ! -f "${custom_package_source}/package.xml" ]]; then
  echo "agv_msgs was not found: ${custom_package_source}" >&2
  exit 1
fi

if [[ "${force_rebuild}" == false &&
      -f "${library_file}" && -f "${required_header}" &&
      -f "${required_state_header}" ]]; then
  build_succeeded=true
  echo "micro-ROS library already contains agv_msgs: ${library_file}"
  exit 0
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "docker was not found. Start Docker Desktop and enable WSL integration." >&2
  exit 1
fi

docker info >/dev/null

mkdir -p "${custom_package_root}"
rm -rf "${staged_custom_package}"
cp -R "${custom_package_source}" "${staged_custom_package}"

if [[ -e "${backup_library_dir}" ]]; then
  echo "Remove stale backup before rebuilding: ${backup_library_dir}" >&2
  exit 1
fi
library_replacement_started=true
if [[ -d "${library_dir}/libmicroros" ]]; then
  mv "${library_dir}/libmicroros" "${backup_library_dir}"
  backup_created=true
fi

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

if [[ ! -f "${library_file}" || ! -f "${required_header}" ||
      ! -f "${required_state_header}" ]]; then
  echo "Builder finished but the library was not created: ${library_file}" >&2
  exit 1
fi

build_succeeded=true
echo "micro-ROS library ready: ${library_file}"
