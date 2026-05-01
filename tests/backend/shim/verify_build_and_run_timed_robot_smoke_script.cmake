file(READ "${ROBOSIM_BUILD_AND_RUN_TIMED_ROBOT_SMOKE_SCRIPT}" script_text)

foreach(required_target IN ITEMS
    robosim_wpi_hal
    robosim_wpi_hal_jni
    robosim_timed_robot_smoke_host)
  if(NOT script_text MATCHES "${required_target}")
    message(FATAL_ERROR
      "build-and-run smoke script does not build ${required_target}")
  endif()
endforeach()

if(NOT script_text MATCHES "tools/timed-robot-template")
  message(FATAL_ERROR
    "build-and-run smoke script does not build the timed robot template")
endif()

if(NOT script_text MATCHES "scripts/run_timed_robot_smoke\\.sh")
  message(FATAL_ERROR
    "build-and-run smoke script does not delegate to run_timed_robot_smoke.sh")
endif()
