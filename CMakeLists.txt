cmake_minimum_required(VERSION 3.24)

project(robosim
  VERSION 0.0.0
  DESCRIPTION "High-fidelity, control-system-agnostic FRC robot simulator"
  LANGUAGES CXX
)

# ---------------------------------------------------------------------------
# Language standard
# ---------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "" FORCE)

# ---------------------------------------------------------------------------
# Build types
#
# We layer the sanitizer build types ASAN / TSAN / UBSAN on top of DEBUG.
# Multiple sanitizers can be selected per build by passing
#   -DROBOSIM_ENABLE_ASAN=ON -DROBOSIM_ENABLE_UBSAN=ON
# (TSAN is incompatible with ASAN, so it must be selected alone.)
# ---------------------------------------------------------------------------

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the build type." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    Debug Release RelWithDebInfo MinSizeRel)
endif()

option(ROBOSIM_ENABLE_ASAN  "Build with AddressSanitizer"            OFF)
option(ROBOSIM_ENABLE_UBSAN "Build with UndefinedBehaviorSanitizer"  OFF)
option(ROBOSIM_ENABLE_TSAN  "Build with ThreadSanitizer"             OFF)

if(ROBOSIM_ENABLE_TSAN AND ROBOSIM_ENABLE_ASAN)
  message(FATAL_ERROR
    "ROBOSIM_ENABLE_TSAN and ROBOSIM_ENABLE_ASAN are mutually exclusive.")
endif()

# ---------------------------------------------------------------------------
# Compiler detection and flags
# ---------------------------------------------------------------------------

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang"
    OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
  set(ROBOSIM_COMPILER_CLANG ON)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  set(ROBOSIM_COMPILER_GCC ON)
else()
  message(WARNING
    "Unsupported compiler '${CMAKE_CXX_COMPILER_ID}'. "
    "robosim is developed against Clang (primary) and GCC (secondary). "
    "Build may not succeed.")
endif()

add_library(robosim_warnings INTERFACE)
target_compile_options(robosim_warnings INTERFACE
  -Wall -Wextra -Wpedantic -Werror
  -Wconversion -Wshadow -Wnon-virtual-dtor -Wold-style-cast
  -Wcast-align -Wunused -Woverloaded-virtual -Wdouble-promotion
  -Wformat=2 -Wimplicit-fallthrough)

add_library(robosim_sanitizers INTERFACE)
if(ROBOSIM_ENABLE_ASAN)
  target_compile_options(robosim_sanitizers INTERFACE
    -fsanitize=address -fno-omit-frame-pointer)
  target_link_options(robosim_sanitizers INTERFACE -fsanitize=address)
endif()
if(ROBOSIM_ENABLE_UBSAN)
  target_compile_options(robosim_sanitizers INTERFACE
    -fsanitize=undefined -fno-omit-frame-pointer)
  target_link_options(robosim_sanitizers INTERFACE -fsanitize=undefined)
endif()
if(ROBOSIM_ENABLE_TSAN)
  target_compile_options(robosim_sanitizers INTERFACE
    -fsanitize=thread -fno-omit-frame-pointer)
  target_link_options(robosim_sanitizers INTERFACE -fsanitize=thread)
endif()

# Convenience aggregate: link to robosim_baseline and inherit warnings +
# sanitizers in one shot. Per-target opt-in keeps third-party FetchContent
# code unaffected.
add_library(robosim_baseline INTERFACE)
target_link_libraries(robosim_baseline INTERFACE
  robosim_warnings robosim_sanitizers)

# ---------------------------------------------------------------------------
# Dependencies
#
# Pinned to specific commit SHAs (resolved from release tags) for
# reproducibility. Update by:
#   1. picking a new release tag,
#   2. running scripts/resolve-dep-sha.sh <repo> <tag>,
#   3. updating both the comment (for humans) and GIT_TAG (for CMake).
# ---------------------------------------------------------------------------

include(FetchContent)

# Reduce noise: only configure each dep once.
set(FETCHCONTENT_QUIET OFF)

# nlohmann/json v3.12.0
FetchContent_Declare(nlohmann_json
  GIT_REPOSITORY https://github.com/nlohmann/json.git
  GIT_TAG        55f93686c01528224f448c19128836e7df245f72
  GIT_SHALLOW    FALSE
)
set(JSON_BuildTests OFF CACHE INTERNAL "")
set(JSON_Install    OFF CACHE INTERNAL "")

# GoogleTest v1.17.0 (includes GoogleMock)
FetchContent_Declare(googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        52eb8108c5bdec04579160ae17225d66034bd723
  GIT_SHALLOW    FALSE
)
set(BUILD_GMOCK   ON  CACHE INTERNAL "")
set(INSTALL_GTEST OFF CACHE INTERNAL "")
set(gtest_force_shared_crt ON CACHE INTERNAL "")

# spdlog v1.17.0
FetchContent_Declare(spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG        79524ddd08a4ec981b7fea76afd08ee05f83755d
  GIT_SHALLOW    FALSE
)
set(SPDLOG_BUILD_EXAMPLE OFF CACHE INTERNAL "")
set(SPDLOG_BUILD_TESTS   OFF CACHE INTERNAL "")
set(SPDLOG_INSTALL       OFF CACHE INTERNAL "")

# MuJoCo 3.8.0
# Heavy dep; only fetched when ROBOSIM_BUILD_PHYSICS=ON. Layer 5 is the
# only consumer; until Layer 5 work lands the rest of the tree builds
# without it. Default OFF until Layer 5 begins.
option(ROBOSIM_BUILD_PHYSICS
  "Fetch and build MuJoCo (required for Layer 5)" OFF)
FetchContent_Declare(mujoco
  GIT_REPOSITORY https://github.com/google-deepmind/mujoco.git
  GIT_TAG        34d69ad4cb1a21846b8297e2bc5e68a4938276c1
  GIT_SHALLOW    FALSE
)
set(MUJOCO_BUILD_EXAMPLES OFF CACHE INTERNAL "")
set(MUJOCO_BUILD_SIMULATE OFF CACHE INTERNAL "")
set(MUJOCO_BUILD_TESTS    OFF CACHE INTERNAL "")
set(MUJOCO_TEST_LOAD_ABSL OFF CACHE INTERNAL "")

FetchContent_MakeAvailable(nlohmann_json googletest spdlog)
if(ROBOSIM_BUILD_PHYSICS)
  FetchContent_MakeAvailable(mujoco)
endif()

# ---------------------------------------------------------------------------
# Source and tests
# ---------------------------------------------------------------------------

add_subdirectory(src)

enable_testing()
include(GoogleTest)
add_subdirectory(tests)
