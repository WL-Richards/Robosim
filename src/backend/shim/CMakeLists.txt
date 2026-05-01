# src/backend/shim
#
# Layer 2 — HAL shim core. The in-process orchestrator the user's
# robot binary will eventually load (as libhal.so) to talk to the
# Sim Core. Cycle 1 covers the boot handshake plus the inbound
# clock_state cache slot and the shutdown terminal receive path.
# See `.claude/skills/hal-shim.md` and tests/backend/shim/TEST_PLAN.md.

set(ROBOSIM_BACKEND_SHIM_SOURCES
  shim_core.cpp
  shim_host_driver.cpp
  timed_robot_smoke_host.cpp
  hal_c.cpp
  hal_cpp_support.cpp
)

add_library(robosim_backend_shim STATIC
  ${ROBOSIM_BACKEND_SHIM_SOURCES}
)

target_include_directories(robosim_backend_shim PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(robosim_backend_shim
  PUBLIC robosim_backend_common robosim_backend_tier1
  PRIVATE robosim_baseline
)

add_library(robosim_backend_shim_jni STATIC
  hal_jni.cpp
)

target_include_directories(robosim_backend_shim_jni PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(robosim_backend_shim_jni
  PUBLIC robosim_backend_shim
  PRIVATE robosim_baseline
)

add_library(robosim_wpi_hal SHARED
  ${ROBOSIM_BACKEND_SHIM_SOURCES}
)

set_target_properties(robosim_wpi_hal PROPERTIES
  OUTPUT_NAME wpiHal
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(robosim_wpi_hal PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(robosim_wpi_hal
  PUBLIC robosim_backend_common robosim_backend_tier1
  PRIVATE robosim_baseline
)

add_library(robosim_wpi_hal_jni SHARED
  hal_jni.cpp
)

set_target_properties(robosim_wpi_hal_jni PROPERTIES
  OUTPUT_NAME wpiHaljni
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(robosim_wpi_hal_jni PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(robosim_wpi_hal_jni
  PUBLIC robosim_wpi_hal
  PRIVATE robosim_baseline
)

add_executable(robosim_timed_robot_smoke_host
  timed_robot_smoke_host_main.cpp
)

target_link_libraries(robosim_timed_robot_smoke_host
  PRIVATE
    robosim_backend_shim
    robosim_baseline
)
