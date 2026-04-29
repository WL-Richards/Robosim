# tests/description — robot description loader tests.
#
# See tests/description/TEST_PLAN.md for the canonical test plan
# (approved by test-reviewer agent, 2026-04-29).

add_executable(robosim_description_test
  loader_test.cpp
)

target_link_libraries(robosim_description_test
  PRIVATE
    robosim_description
    robosim_baseline
    GTest::gtest
    GTest::gmock
    GTest::gtest_main
    nlohmann_json::nlohmann_json
)

# Pass the absolute path to the production v0-arm.json fixture so tests
# don't rely on a working directory.
target_compile_definitions(robosim_description_test PRIVATE
  ROBOSIM_V0_ARM_FIXTURE_PATH="${CMAKE_SOURCE_DIR}/descriptions/v0-arm.json"
)

gtest_discover_tests(robosim_description_test)
