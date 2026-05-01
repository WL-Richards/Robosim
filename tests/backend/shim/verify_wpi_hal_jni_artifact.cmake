if(NOT DEFINED ROBOSIM_WPI_HAL_JNI_PATH)
  message(FATAL_ERROR "ROBOSIM_WPI_HAL_JNI_PATH is required")
endif()

if(NOT EXISTS "${ROBOSIM_WPI_HAL_JNI_PATH}")
  message(FATAL_ERROR "WPILib HAL JNI artifact does not exist: ${ROBOSIM_WPI_HAL_JNI_PATH}")
endif()

get_filename_component(robosim_wpi_hal_jni_name "${ROBOSIM_WPI_HAL_JNI_PATH}" NAME)
if(NOT robosim_wpi_hal_jni_name STREQUAL "libwpiHaljni.so")
  message(FATAL_ERROR
    "WPILib HAL JNI artifact must be named libwpiHaljni.so, got ${robosim_wpi_hal_jni_name}")
endif()

if(DEFINED ROBOSIM_WPI_HAL_JNI_REQUIRED_SYMBOLS)
  if(NOT DEFINED ROBOSIM_WPI_HAL_JNI_NM)
    message(FATAL_ERROR "ROBOSIM_WPI_HAL_JNI_NM is required for symbol verification")
  endif()

  execute_process(
    COMMAND "${ROBOSIM_WPI_HAL_JNI_NM}" -D --defined-only "${ROBOSIM_WPI_HAL_JNI_PATH}"
    RESULT_VARIABLE robosim_wpi_hal_jni_nm_result
    OUTPUT_VARIABLE robosim_wpi_hal_jni_nm_output
    ERROR_VARIABLE robosim_wpi_hal_jni_nm_error
  )
  if(NOT robosim_wpi_hal_jni_nm_result EQUAL 0)
    message(FATAL_ERROR
      "failed to inspect WPILib HAL JNI artifact symbols: ${robosim_wpi_hal_jni_nm_error}")
  endif()

  foreach(robosim_wpi_hal_jni_symbol IN LISTS ROBOSIM_WPI_HAL_JNI_REQUIRED_SYMBOLS)
    string(REGEX MATCH
      "(^|[\r\n])[0-9A-Fa-f]+[ \t]+[A-Za-z][ \t]+${robosim_wpi_hal_jni_symbol}($|[\r\n])"
      robosim_wpi_hal_jni_symbol_match "${robosim_wpi_hal_jni_nm_output}")
    if(NOT robosim_wpi_hal_jni_symbol_match)
      message(FATAL_ERROR
        "WPILib HAL JNI artifact is missing defined exported symbol: ${robosim_wpi_hal_jni_symbol}")
    endif()
  endforeach()
endif()
