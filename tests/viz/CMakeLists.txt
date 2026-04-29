# tests/viz — visualizer unit tests.
#
# Tests under this subtree are exempt from sim-core determinism rules
# (banned clocks, banned RNG); state that explicitly in test plans so
# the test-reviewer agent sees the exception. See
# .claude/skills/visualizer.md "Determinism exception."
#
if(NOT ROBOSIM_BUILD_VIZ)
  return()
endif()

add_executable(robosim_viz_test
  camera_test.cpp
  edit_mode_builder_test.cpp
  fixtures.cpp
  picking_test.cpp
  scene_snapshot_test.cpp
)

target_link_libraries(robosim_viz_test
  PRIVATE
    robosim_viz_core
    robosim_viz_edit_builder
    robosim_description
    robosim_baseline
    GTest::gtest
    GTest::gmock
    GTest::gtest_main
)

target_compile_definitions(robosim_viz_test PRIVATE
  ROBOSIM_V0_ARM_FIXTURE_PATH="${CMAKE_SOURCE_DIR}/descriptions/v0-arm.json"
)

gtest_discover_tests(robosim_viz_test)
