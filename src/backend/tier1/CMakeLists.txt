# src/backend/tier1
#
# Layer 2 — native tier: shared-memory transport between HAL shim and sim core.

add_library(robosim_backend_tier1 STATIC
  shared_memory_transport.cpp
)

target_include_directories(robosim_backend_tier1 PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(robosim_backend_tier1
  PUBLIC robosim_backend_common
  PRIVATE robosim_baseline
)
