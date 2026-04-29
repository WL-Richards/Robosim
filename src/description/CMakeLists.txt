# src/description — robot description loader.
#
# Public surface in loader.h; types in schema.h, error.h. Implementation
# in loader.cpp. Validation logic will split into validation.{h,cpp} when
# the real implementation lands per V0_PLAN B7; the B5/B6 stub is a
# single .cpp that throws.

add_library(robosim_description STATIC
  loader.cpp
)

target_include_directories(robosim_description PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(robosim_description
  PUBLIC nlohmann_json::nlohmann_json
  PRIVATE robosim_baseline
)
