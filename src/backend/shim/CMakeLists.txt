# src/backend/shim
#
# Layer 2 — HAL shim core. The in-process orchestrator the user's
# robot binary will eventually load (as libhal.so) to talk to the
# Sim Core. Cycle 1 covers the boot handshake plus the inbound
# clock_state cache slot and the shutdown terminal receive path.
# See `.claude/skills/hal-shim.md` and tests/backend/shim/TEST_PLAN.md.

add_library(robosim_backend_shim STATIC
  shim_core.cpp
  hal_c.cpp
)

target_include_directories(robosim_backend_shim PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(robosim_backend_shim
  PUBLIC robosim_backend_common robosim_backend_tier1
  PRIVATE robosim_baseline
)
