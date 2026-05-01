if(NOT DEFINED ROBOSIM_HAL_JNI_CLASSIFIER)
  message(FATAL_ERROR "ROBOSIM_HAL_JNI_CLASSIFIER is required")
endif()
if(NOT DEFINED ROBOSIM_HAL_JNI_UNDEFINED_SYMBOLS_FILE)
  message(FATAL_ERROR "ROBOSIM_HAL_JNI_UNDEFINED_SYMBOLS_FILE is required")
endif()
if(NOT DEFINED ROBOSIM_EXPECTED_HAL_JNI_BLOCKER)
  message(FATAL_ERROR "ROBOSIM_EXPECTED_HAL_JNI_BLOCKER is required")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}"
    -DROBOSIM_HAL_JNI_UNDEFINED_SYMBOLS_FILE=${ROBOSIM_HAL_JNI_UNDEFINED_SYMBOLS_FILE}
    -P "${ROBOSIM_HAL_JNI_CLASSIFIER}"
  RESULT_VARIABLE robosim_classifier_result
  OUTPUT_VARIABLE robosim_classifier_output
  ERROR_VARIABLE robosim_classifier_error
)

if(robosim_classifier_result EQUAL 0)
  message(FATAL_ERROR "classifier unexpectedly accepted HALSIM candidate")
endif()

set(robosim_classifier_combined_output
  "${robosim_classifier_output}\n${robosim_classifier_error}")
if(NOT robosim_classifier_combined_output MATCHES "${ROBOSIM_EXPECTED_HAL_JNI_BLOCKER}")
  message(FATAL_ERROR
    "classifier failure did not mention expected blocker ${ROBOSIM_EXPECTED_HAL_JNI_BLOCKER}: ${robosim_classifier_combined_output}")
endif()

message(STATUS
  "classifier rejected HALSIM candidate at ${ROBOSIM_EXPECTED_HAL_JNI_BLOCKER}")
