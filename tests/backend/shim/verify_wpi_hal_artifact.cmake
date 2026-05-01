if(NOT DEFINED ROBOSIM_WPI_HAL_PATH)
  message(FATAL_ERROR "ROBOSIM_WPI_HAL_PATH is required")
endif()

if(NOT EXISTS "${ROBOSIM_WPI_HAL_PATH}")
  message(FATAL_ERROR "WPILib HAL shim artifact does not exist: ${ROBOSIM_WPI_HAL_PATH}")
endif()

get_filename_component(robosim_wpi_hal_name "${ROBOSIM_WPI_HAL_PATH}" NAME)
if(NOT robosim_wpi_hal_name STREQUAL "libwpiHal.so")
  message(FATAL_ERROR
    "WPILib HAL shim artifact must be named libwpiHal.so, got ${robosim_wpi_hal_name}")
endif()

if(DEFINED ROBOSIM_WPI_HAL_REQUIRED_SYMBOLS)
  if(NOT DEFINED ROBOSIM_WPI_HAL_NM)
    message(FATAL_ERROR "ROBOSIM_WPI_HAL_NM is required when checking exported symbols")
  endif()

  execute_process(
    COMMAND "${ROBOSIM_WPI_HAL_NM}" -D --defined-only "${ROBOSIM_WPI_HAL_PATH}"
    RESULT_VARIABLE robosim_nm_result
    OUTPUT_VARIABLE robosim_nm_output
    ERROR_VARIABLE robosim_nm_error
  )

  if(NOT robosim_nm_result EQUAL 0)
    message(FATAL_ERROR
      "Failed to inspect ${ROBOSIM_WPI_HAL_PATH} with ${ROBOSIM_WPI_HAL_NM}: ${robosim_nm_error}")
  endif()

  foreach(robosim_symbol IN LISTS ROBOSIM_WPI_HAL_REQUIRED_SYMBOLS)
    string(REGEX MATCH
      "(^|[\r\n])[0-9A-Fa-f]+[ \t]+[A-Za-z][ \t]+${robosim_symbol}($|[\r\n])"
      robosim_symbol_match
      "${robosim_nm_output}"
    )
    if(NOT robosim_symbol_match)
      message(FATAL_ERROR
        "Expected ${ROBOSIM_WPI_HAL_PATH} to define exported HAL symbol ${robosim_symbol}")
    endif()
  endforeach()
endif()
