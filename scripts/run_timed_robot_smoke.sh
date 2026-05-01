#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"

wpilib_root="${WPILIB_ROOT:-/home/will/wpilib/2026}"
java_home="${JAVA_HOME:-${wpilib_root}/jdk}"
java_bin="${java_home}/bin/java"
timeout_seconds="${ROBOSIM_TIMED_ROBOT_TIMEOUT:-15s}"

shim_lib_dir="${repo_root}/build/src/backend/shim"
smoke_host="${shim_lib_dir}/robosim_timed_robot_smoke_host"
template_dir="${repo_root}/tools/timed-robot-template"
template_jni_dir="${template_dir}/build/jni/release"
template_jar="${template_dir}/build/libs/timed-robot-template.jar"

if [[ ! -x "${java_bin}" ]]; then
  echo "Java executable not found: ${java_bin}" >&2
  echo "Set WPILIB_ROOT or JAVA_HOME to your WPILib JDK." >&2
  exit 1
fi

if [[ ! -f "${shim_lib_dir}/libwpiHal.so" || ! -f "${shim_lib_dir}/libwpiHaljni.so" ]]; then
  echo "Robosim HAL shim libraries are missing in ${shim_lib_dir}." >&2
  echo "Build them first: cmake --build build --target robosim_wpi_hal robosim_wpi_hal_jni" >&2
  exit 1
fi

if [[ ! -x "${smoke_host}" ]]; then
  echo "Robosim timed robot smoke host is missing: ${smoke_host}" >&2
  echo "Build it first: cmake --build build --target robosim_timed_robot_smoke_host" >&2
  exit 1
fi

if [[ ! -f "${template_jar}" ]]; then
  echo "TimedRobot template jar is missing: ${template_jar}" >&2
  echo "Build it first from tools/timed-robot-template." >&2
  exit 1
fi

if [[ ! -d "${template_jni_dir}" ]]; then
  echo "TimedRobot template JNI directory is missing: ${template_jni_dir}" >&2
  echo "Build the template native runtime first from tools/timed-robot-template." >&2
  exit 1
fi

native_path="${shim_lib_dir}:${template_jni_dir}"

export JAVA_HOME="${java_home}"
export PATH="${java_home}/bin:${PATH}"
export LD_LIBRARY_PATH="${native_path}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

exec timeout "${timeout_seconds}" \
  "${smoke_host}" \
  -- \
  "${java_bin}" \
  "-Djava.library.path=${native_path}" \
  -cp "${template_jar}" \
  frc.robot.Main
