# tests/backend/shim
#
# Tests for the cycle-1 HAL shim core. See TEST_PLAN.md for the
# approved suite and `.claude/skills/hal-shim.md` for working
# knowledge.

add_executable(robosim_backend_shim_test
  shim_core_test.cpp
)

target_include_directories(robosim_backend_shim_test PRIVATE
  ${CMAKE_SOURCE_DIR}/tests/backend/tier1
)

target_link_libraries(robosim_backend_shim_test
  PRIVATE
    robosim_backend_shim_jni
    robosim_backend_shim
    robosim_backend_tier1
    robosim_backend_common
    robosim_baseline
    GTest::gtest
    GTest::gtest_main
)

gtest_discover_tests(robosim_backend_shim_test)

add_test(
  NAME robosim_wpi_hal_artifact_exists
  COMMAND ${CMAKE_COMMAND}
    -DROBOSIM_WPI_HAL_PATH=$<TARGET_FILE:robosim_wpi_hal>
    -P ${CMAKE_CURRENT_SOURCE_DIR}/verify_wpi_hal_artifact.cmake
)

set(ROBOSIM_WPI_HAL_REQUIRED_SYMBOLS
  HAL_Initialize
  HAL_Shutdown
  HAL_GetFPGATime
  HAL_RefreshDSData
  HAL_WaitForNotifierAlarm
  HAL_CAN_SendMessage
  HAL_GetMatchInfo
  HAL_SetJoystickOutputs
  HAL_ObserveUserProgramTeleop
  HAL_GetRuntimeType
  HAL_Report
  HAL_GetComments
  _ZN3hal10HandleBaseC1Ev
  _ZN3hal10HandleBaseC2Ev
  _ZN3hal10HandleBaseD1Ev
  _ZN3hal10HandleBaseD2Ev
  _ZN3hal10HandleBase12ResetHandlesEv
  _ZN3hal10HandleBase18ResetGlobalHandlesEv
  _ZTIN3hal10HandleBaseE
  _ZTSN3hal10HandleBaseE
  _ZTVN3hal10HandleBaseE
)

add_test(
  NAME robosim_wpi_hal_exports_core_symbols
  COMMAND ${CMAKE_COMMAND}
    -DROBOSIM_WPI_HAL_PATH=$<TARGET_FILE:robosim_wpi_hal>
    -DROBOSIM_WPI_HAL_NM=${CMAKE_NM}
    "-DROBOSIM_WPI_HAL_REQUIRED_SYMBOLS=${ROBOSIM_WPI_HAL_REQUIRED_SYMBOLS}"
    -P ${CMAKE_CURRENT_SOURCE_DIR}/verify_wpi_hal_artifact.cmake
)

add_test(
  NAME robosim_hal_jni_classifier_rejects_halsim_candidate
  COMMAND ${CMAKE_COMMAND}
    -DROBOSIM_HAL_JNI_UNDEFINED_SYMBOLS_FILE=${CMAKE_CURRENT_SOURCE_DIR}/fixtures/hal_jni_undefined_with_halsim.nm
    -DROBOSIM_HAL_JNI_CLASSIFIER=${CMAKE_CURRENT_SOURCE_DIR}/classify_hal_jni_candidate.cmake
    -DROBOSIM_EXPECTED_HAL_JNI_BLOCKER=HALSIM_CancelRoboRioVInCurrentCallback
    -P ${CMAKE_CURRENT_SOURCE_DIR}/expect_hal_jni_classifier_failure.cmake
)

add_test(
  NAME robosim_hal_jni_classifier_accepts_non_halsim_candidate
  COMMAND ${CMAKE_COMMAND}
    -DROBOSIM_HAL_JNI_UNDEFINED_SYMBOLS_FILE=${CMAKE_CURRENT_SOURCE_DIR}/fixtures/hal_jni_undefined_without_halsim.nm
    -P ${CMAKE_CURRENT_SOURCE_DIR}/classify_hal_jni_candidate.cmake
)

add_test(
  NAME robosim_wpi_hal_jni_artifact_exists
  COMMAND ${CMAKE_COMMAND}
    -DROBOSIM_WPI_HAL_JNI_PATH=$<TARGET_FILE:robosim_wpi_hal_jni>
    -P ${CMAKE_CURRENT_SOURCE_DIR}/verify_wpi_hal_jni_artifact.cmake
)

set(ROBOSIM_WPI_HAL_JNI_REQUIRED_SYMBOLS
  Java_edu_wpi_first_hal_HAL_initialize
  Java_edu_wpi_first_hal_HAL_shutdown
  Java_edu_wpi_first_hal_HAL_hasMain
  Java_edu_wpi_first_hal_HAL_runMain
  Java_edu_wpi_first_hal_HAL_exitMain
  Java_edu_wpi_first_hal_HAL_terminate
  Java_edu_wpi_first_hal_DriverStationJNI_refreshDSData
  Java_edu_wpi_first_hal_DriverStationJNI_observeUserProgramStarting
  Java_edu_wpi_first_hal_DriverStationJNI_observeUserProgramDisabled
  Java_edu_wpi_first_hal_DriverStationJNI_observeUserProgramAutonomous
  Java_edu_wpi_first_hal_DriverStationJNI_observeUserProgramTeleop
  Java_edu_wpi_first_hal_DriverStationJNI_observeUserProgramTest
  Java_edu_wpi_first_hal_DriverStationJNI_sendError
  Java_edu_wpi_first_hal_DriverStationJNI_nativeGetControlWord
  Java_edu_wpi_first_hal_DriverStationJNI_nativeGetAllianceStation
  Java_edu_wpi_first_hal_DriverStationJNI_getJoystickAxes
  Java_edu_wpi_first_hal_DriverStationJNI_getJoystickAxesRaw
  Java_edu_wpi_first_hal_DriverStationJNI_getJoystickPOVs
  Java_edu_wpi_first_hal_DriverStationJNI_getJoystickButtons
  Java_edu_wpi_first_hal_DriverStationJNI_getMatchInfo
  Java_edu_wpi_first_hal_DriverStationJNI_report
  Java_edu_wpi_first_hal_NotifierJNI_initializeNotifier
  Java_edu_wpi_first_hal_NotifierJNI_setHALThreadPriority
  Java_edu_wpi_first_hal_NotifierJNI_setNotifierName
  Java_edu_wpi_first_hal_NotifierJNI_stopNotifier
  Java_edu_wpi_first_hal_NotifierJNI_cleanNotifier
  Java_edu_wpi_first_hal_NotifierJNI_updateNotifierAlarm
  Java_edu_wpi_first_hal_NotifierJNI_cancelNotifierAlarm
  Java_edu_wpi_first_hal_NotifierJNI_waitForNotifierAlarm
  Java_edu_wpi_first_hal_HALUtil_getHALRuntimeType
  Java_edu_wpi_first_hal_HALUtil_getFPGATime
)

add_test(
  NAME robosim_wpi_hal_jni_exports_startup_symbols
  COMMAND ${CMAKE_COMMAND}
    -DROBOSIM_WPI_HAL_JNI_PATH=$<TARGET_FILE:robosim_wpi_hal_jni>
    -DROBOSIM_WPI_HAL_JNI_NM=${CMAKE_NM}
    "-DROBOSIM_WPI_HAL_JNI_REQUIRED_SYMBOLS=${ROBOSIM_WPI_HAL_JNI_REQUIRED_SYMBOLS}"
    -P ${CMAKE_CURRENT_SOURCE_DIR}/verify_wpi_hal_jni_artifact.cmake
)

add_test(
  NAME robosim_wpi_hal_jni_is_non_halsim_candidate
  COMMAND ${CMAKE_COMMAND}
    -DROBOSIM_HAL_JNI_PATH=$<TARGET_FILE:robosim_wpi_hal_jni>
    -DROBOSIM_HAL_JNI_NM=${CMAKE_NM}
    -P ${CMAKE_CURRENT_SOURCE_DIR}/classify_hal_jni_candidate.cmake
)

add_test(
  NAME timed_robot_smoke_script_uses_host_runtime_launcher
  COMMAND ${CMAKE_COMMAND}
    -DROBOSIM_TIMED_ROBOT_SMOKE_SCRIPT=${CMAKE_SOURCE_DIR}/scripts/run_timed_robot_smoke.sh
    -P ${CMAKE_CURRENT_SOURCE_DIR}/verify_timed_robot_smoke_script.cmake
)

add_test(
  NAME build_and_run_timed_robot_smoke_script_builds_required_artifacts
  COMMAND ${CMAKE_COMMAND}
    -DROBOSIM_BUILD_AND_RUN_TIMED_ROBOT_SMOKE_SCRIPT=${CMAKE_SOURCE_DIR}/scripts/build_and_run_timed_robot_smoke.sh
    -P ${CMAKE_CURRENT_SOURCE_DIR}/verify_build_and_run_timed_robot_smoke_script.cmake
)
