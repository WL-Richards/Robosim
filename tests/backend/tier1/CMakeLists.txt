# tests/backend/tier1
#
# Tests for tier-1 shared-memory transport.

add_executable(robosim_backend_tier1_test
  shared_memory_mapping_test.cpp
  shared_memory_transport_test.cpp
)

target_link_libraries(robosim_backend_tier1_test
  PRIVATE
    robosim_backend_tier1
    robosim_backend_common
    robosim_baseline
    GTest::gtest
    GTest::gtest_main
)

gtest_discover_tests(robosim_backend_tier1_test)
