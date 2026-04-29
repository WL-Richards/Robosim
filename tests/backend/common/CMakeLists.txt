# tests/backend/common
#
# Tests for Layer 2 protocol schema. See TEST_PLAN.md for the approved
# test plan (test-reviewer ratified 2026-04-29, three rounds).
#
# Three translation units:
#   compile_time_invariants_test.cpp — build-only TU per A1; #includes
#     every payload header so the static_asserts get checked.
#   protocol_test.cpp — sections A2–A6, B, C, D, E, F, G, I, J, K, L,
#     M, N, O, Q.
#   wpilib_parity_test.cpp — section H. Uses wpilib_mirror.h (local
#     transcription of WPILib HAL types pinned at kWpilibParitySha).

add_executable(robosim_backend_common_test
  compile_time_invariants_test.cpp
  protocol_session_test.cpp
  protocol_test.cpp
  wpilib_parity_test.cpp
)

target_link_libraries(robosim_backend_common_test
  PRIVATE
    robosim_backend_common
    robosim_baseline
    GTest::gtest
    GTest::gmock
    GTest::gtest_main
)

gtest_discover_tests(robosim_backend_common_test)
