#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"

wpilib_root="${WPILIB_ROOT:-/home/will/wpilib/2026}"
java_home="${JAVA_HOME:-${wpilib_root}/jdk}"
export JAVA_HOME="${java_home}"
export PATH="${java_home}/bin:${PATH}"

cmake --build "${repo_root}/build" \
  --target robosim_wpi_hal robosim_wpi_hal_jni robosim_timed_robot_smoke_host

(
  cd "${repo_root}/tools/timed-robot-template"
  ./gradlew build
)

exec "${repo_root}/scripts/run_timed_robot_smoke.sh"
