# src/backend/common
#
# Layer 2 — protocol POD structs, truncation helpers, and the stateless
# envelope validator. See tests/backend/common/TEST_PLAN.md for the
# approved schema (test-reviewer ratified 2026-04-29, three rounds).
#
# Headers are PODs with static_assert pinning size, alignment, and
# layout invariants. The static_assert is the contract — tests do not
# re-assert those at runtime; a single build-only translation unit in
# tests/ #includes every header to ensure the asserts get checked.

add_library(robosim_backend_common STATIC
  protocol_session.cpp
  truncate.cpp
  validator.cpp
)

target_include_directories(robosim_backend_common PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(robosim_backend_common
  PRIVATE robosim_baseline
)
