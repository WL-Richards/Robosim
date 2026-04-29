# tests/description — robot description loader tests.
#
# See tests/description/TEST_PLAN.md (v1, approved 2026-04-29) and
# tests/description/TEST_PLAN_VC.md (v2 + serializer, ready-to-
# implement after rev-4 reviewer pass) for the canonical test plans.

add_executable(robosim_description_test
  loader_test.cpp
  loader_origin_test.cpp
  origin_pose_decompose_test.cpp
  serializer_test.cpp
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

# Pass absolute fixture paths via preprocessor defines so tests
# don't rely on a working directory.
#  - v0-arm.json: production fixture (currently schema_version 1;
#    bumped to 2 by the implementing session per VC1).
#  - v0_arm_v1.json: frozen schema_version 1 copy used by V0
#    (forward-compat regression guard) and kept stable across the
#    v1→v2 production migration.
target_compile_definitions(robosim_description_test PRIVATE
  ROBOSIM_V0_ARM_FIXTURE_PATH="${CMAKE_SOURCE_DIR}/descriptions/v0-arm.json"
  ROBOSIM_V0_ARM_V1_FIXTURE_PATH="${CMAKE_SOURCE_DIR}/tests/description/fixtures/v0_arm_v1.json"
)

gtest_discover_tests(robosim_description_test)
