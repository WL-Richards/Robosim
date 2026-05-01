file(READ "${ROBOSIM_TIMED_ROBOT_SMOKE_SCRIPT}" script_text)

if(NOT script_text MATCHES "robosim_timed_robot_smoke_host")
  message(FATAL_ERROR
    "timed robot smoke script does not delegate to robosim_timed_robot_smoke_host")
endif()

if(NOT script_text MATCHES "--[ \t\r\n\\\\]+\"\\$\\{java_bin\\}\"")
  message(FATAL_ERROR
    "timed robot smoke script does not pass the Java command after --")
endif()

if(script_text MATCHES "exec[ \t\r\n]+timeout[ \t\r\n]+\\$\\{timeout_seconds\\}[ \t\r\n]+\\\\?[ \t\r\n]*\\$\\{java_bin\\}")
  message(FATAL_ERROR
    "timed robot smoke script still directly execs timeout/java")
endif()
