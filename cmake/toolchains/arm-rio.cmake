# cmake/toolchains/arm-rio.cmake
#
# CMake toolchain file for cross-compiling to RoboRIO 2 (Xilinx Zynq-7020,
# ARMv7-A Cortex-A9, hard-float ABI). Used for tier-2 (qemu-user) and
# tier-4 (HIL) backend builds.
#
# By default this uses the system `arm-linux-gnueabihf-gcc` toolchain
# (from Ubuntu's gcc-arm-linux-gnueabihf package), which is good enough
# for the scaffold-build verification CI runs. Real RIO deployment
# eventually needs the **WPILib FRC roboRIO toolchain** — same toolchain
# RIOEmulator uses, distributed by WPILib per FRC season — for ABI
# compatibility with NI Linux RT and Phoenix 6 native libraries.
#
# To use the WPILib toolchain, set ROBOSIM_FRC_TOOLCHAIN_DIR (env var or
# -D) to the unpacked toolchain root (the directory containing `bin/`).
# Example:
#   cmake -B build-arm \
#     -DCMAKE_TOOLCHAIN_FILE=cmake/toolchains/arm-rio.cmake \
#     -DROBOSIM_FRC_TOOLCHAIN_DIR=$HOME/wpilib/2026/roborio
#
# TODO(robosim): wire in WPILib toolchain download when tier-2 work
# starts; until then the system cross-compiler is enough to verify the
# tree cross-compiles cleanly.

set(CMAKE_SYSTEM_NAME      Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

if(NOT DEFINED ROBOSIM_FRC_TOOLCHAIN_DIR
    AND DEFINED ENV{ROBOSIM_FRC_TOOLCHAIN_DIR})
  set(ROBOSIM_FRC_TOOLCHAIN_DIR "$ENV{ROBOSIM_FRC_TOOLCHAIN_DIR}")
endif()

if(DEFINED ROBOSIM_FRC_TOOLCHAIN_DIR)
  if(NOT IS_DIRECTORY "${ROBOSIM_FRC_TOOLCHAIN_DIR}/bin")
    message(FATAL_ERROR
      "ROBOSIM_FRC_TOOLCHAIN_DIR='${ROBOSIM_FRC_TOOLCHAIN_DIR}' has no bin/ "
      "subdirectory. Point it at the unpacked WPILib roboRIO toolchain root.")
  endif()
  set(_tc_prefix "${ROBOSIM_FRC_TOOLCHAIN_DIR}/bin/arm-frc-linux-gnueabi-")
  set(CMAKE_C_COMPILER   "${_tc_prefix}gcc")
  set(CMAKE_CXX_COMPILER "${_tc_prefix}g++")
  set(CMAKE_AR           "${_tc_prefix}ar"      CACHE FILEPATH "")
  set(CMAKE_RANLIB       "${_tc_prefix}ranlib"  CACHE FILEPATH "")
  set(CMAKE_STRIP        "${_tc_prefix}strip"   CACHE FILEPATH "")
  set(CMAKE_FIND_ROOT_PATH "${ROBOSIM_FRC_TOOLCHAIN_DIR}/arm-frc-linux-gnueabi")
else()
  # System cross-compiler (gcc-arm-linux-gnueabihf). ABI-compatible enough
  # to verify "this tree cross-compiles cleanly" but NOT for real RIO
  # deployment (different libc layout, no NI RT-specific headers).
  set(CMAKE_C_COMPILER   arm-linux-gnueabihf-gcc)
  set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)
endif()

# Cortex-A9 is ARMv7-A with NEON (FPv3) and hard-float.
set(_robosim_arm_flags
  "-march=armv7-a -mfpu=neon -mfloat-abi=hard")
set(CMAKE_C_FLAGS_INIT   "${_robosim_arm_flags}")
set(CMAKE_CXX_FLAGS_INIT "${_robosim_arm_flags}")

# Standard cross-find behavior: programs from host, libraries/includes
# from sysroot only.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
