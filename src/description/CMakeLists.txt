# src/description — robot description loader.
#
# Public surface in loader.h (load_from_file), serializer.h
# (save_to_file), origin_pose.h (compose_origin, validate_finite_*).
# Types in schema.h, error.h. See TEST_PLAN.md (v1) and
# TEST_PLAN_VC.md (v2 + serializer) for the test-reviewer-approved
# contracts.

add_library(robosim_description STATIC
  loader.cpp
  origin_pose.cpp
  serializer.cpp
)

target_include_directories(robosim_description PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(robosim_description
  PUBLIC nlohmann_json::nlohmann_json
  PRIVATE robosim_baseline
)

# D-VC-3 loader-finite-seam check — configure-time tripwire that
# forbids bare std::isfinite/isnan/isinf/isnormal/fpclassify calls
# anywhere in src/description/ except origin_pose.cpp. Runs
# unconditionally on every configure (loader-only and viz-on
# matrices alike).
include("${CMAKE_SOURCE_DIR}/cmake/lint_loader_finite_seam.cmake")
robosim_check_loader_finite_seam()
