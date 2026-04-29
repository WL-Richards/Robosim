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
    robosim_backend_shim
    robosim_backend_tier1
    robosim_backend_common
    robosim_baseline
    GTest::gtest
    GTest::gtest_main
)

gtest_discover_tests(robosim_backend_shim_test)
