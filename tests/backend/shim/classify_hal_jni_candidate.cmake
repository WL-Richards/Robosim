if(DEFINED ROBOSIM_HAL_JNI_UNDEFINED_SYMBOLS_FILE)
  file(READ "${ROBOSIM_HAL_JNI_UNDEFINED_SYMBOLS_FILE}" robosim_hal_jni_nm_output)
elseif(DEFINED ROBOSIM_HAL_JNI_PATH)
  if(NOT DEFINED ROBOSIM_HAL_JNI_NM)
    message(FATAL_ERROR "ROBOSIM_HAL_JNI_NM is required when ROBOSIM_HAL_JNI_PATH is used")
  endif()
  execute_process(
    COMMAND "${ROBOSIM_HAL_JNI_NM}" -D --undefined-only "${ROBOSIM_HAL_JNI_PATH}"
    RESULT_VARIABLE robosim_hal_jni_nm_result
    OUTPUT_VARIABLE robosim_hal_jni_nm_output
    ERROR_VARIABLE robosim_hal_jni_nm_error
  )
  if(NOT robosim_hal_jni_nm_result EQUAL 0)
    message(FATAL_ERROR
      "failed to inspect HAL JNI candidate: ${robosim_hal_jni_nm_error}")
  endif()
else()
  message(FATAL_ERROR
    "set ROBOSIM_HAL_JNI_UNDEFINED_SYMBOLS_FILE or ROBOSIM_HAL_JNI_PATH")
endif()

string(REPLACE "\r\n" "\n" robosim_hal_jni_nm_output "${robosim_hal_jni_nm_output}")
string(REPLACE "\r" "\n" robosim_hal_jni_nm_output "${robosim_hal_jni_nm_output}")
string(REPLACE "\n" ";" robosim_hal_jni_nm_lines "${robosim_hal_jni_nm_output}")

set(robosim_first_halsim_symbol "")
foreach(robosim_hal_jni_nm_line IN LISTS robosim_hal_jni_nm_lines)
  string(STRIP "${robosim_hal_jni_nm_line}" robosim_hal_jni_nm_line)
  if(robosim_hal_jni_nm_line MATCHES "^U[ \t]+([^ \t]+)")
    set(robosim_symbol "${CMAKE_MATCH_1}")
  elseif(robosim_hal_jni_nm_line MATCHES "^[0-9A-Fa-f]+[ \t]+U[ \t]+([^ \t]+)")
    set(robosim_symbol "${CMAKE_MATCH_1}")
  else()
    continue()
  endif()

  if(robosim_symbol MATCHES "^HALSIM_")
    set(robosim_first_halsim_symbol "${robosim_symbol}")
    break()
  endif()
endforeach()

if(NOT robosim_first_halsim_symbol STREQUAL "")
  message(FATAL_ERROR
    "HAL JNI candidate is not valid for the non-HALSIM shim path; first HALSIM dependency: ${robosim_first_halsim_symbol}")
endif()

message(STATUS "HAL JNI candidate is non-HALSIM-compatible")
