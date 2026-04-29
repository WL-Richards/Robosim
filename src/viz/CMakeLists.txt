# src/viz — 3D visualizer (Edit / Live / Replay; Edit shipped in v0).
#
# Standalone GUI binary, gated behind ROBOSIM_BUILD_VIZ=ON (default OFF).
# Sim core does not link against this. See .claude/skills/visualizer.md
# for scope and the determinism exception that applies inside this
# subtree.
#
# Library layout pinned by tests/viz/TEST_PLAN.md T1 (renderer / loader
# isolation):
#
#   robosim_viz_core         — scene_snapshot, camera, picking, AABB,
#                              frame-fit. Pure math; depends on nothing
#                              from src/description/.
#   robosim_viz_edit_builder — robot_description -> scene_snapshot. The
#                              ONLY library that depends on
#                              robosim_description in the viz subtree
#                              (alongside the panels and main CLI).
#   robosim_viz_renderer     — OpenGL primitive rendering. Reads only
#                              from scene_snapshot. Must NOT link
#                              robosim_description (T1 enforces this).
#   robosim_viz_panels       — ImGui scene tree, inspector, status bar.
#                              Reads description for inspector fields.
#
# T1 also runs cmake/lint_renderer_isolation.cmake at configure time.

if(NOT ROBOSIM_BUILD_VIZ)
  return()
endif()

# ---------------------------------------------------------------------------
# Generated OpenGL 3.3 core loader (glad).
# ---------------------------------------------------------------------------

glad_add_library(robosim_viz_glad STATIC
  REPRODUCIBLE
  API gl:core=3.3
)

# ---------------------------------------------------------------------------
# Dear ImGui (docking branch).
# ---------------------------------------------------------------------------

add_library(robosim_viz_imgui STATIC
  "${ROBOSIM_IMGUI_SOURCE_DIR}/imgui.cpp"
  "${ROBOSIM_IMGUI_SOURCE_DIR}/imgui_demo.cpp"
  "${ROBOSIM_IMGUI_SOURCE_DIR}/imgui_draw.cpp"
  "${ROBOSIM_IMGUI_SOURCE_DIR}/imgui_tables.cpp"
  "${ROBOSIM_IMGUI_SOURCE_DIR}/imgui_widgets.cpp"
  "${ROBOSIM_IMGUI_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
  "${ROBOSIM_IMGUI_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp"
)

target_include_directories(robosim_viz_imgui SYSTEM PUBLIC
  "${ROBOSIM_IMGUI_SOURCE_DIR}"
  "${ROBOSIM_IMGUI_SOURCE_DIR}/backends"
)

target_compile_definitions(robosim_viz_imgui PUBLIC
  IMGUI_DEFINE_MATH_OPERATORS
)

target_link_libraries(robosim_viz_imgui
  PUBLIC glfw
)

# ---------------------------------------------------------------------------
# ImGuizmo.
# ---------------------------------------------------------------------------

add_library(robosim_viz_imguizmo STATIC
  "${ROBOSIM_IMGUIZMO_SOURCE_DIR}/ImGuizmo.cpp"
)

target_include_directories(robosim_viz_imguizmo SYSTEM PUBLIC
  "${ROBOSIM_IMGUIZMO_SOURCE_DIR}"
)

target_link_libraries(robosim_viz_imguizmo
  PUBLIC robosim_viz_imgui
)

# ---------------------------------------------------------------------------
# robosim_viz_core — pure math: scene_snapshot, camera, picking, AABB.
# Must not depend on src/description/.
# ---------------------------------------------------------------------------

add_library(robosim_viz_core STATIC
  scene_snapshot.cpp
  camera.cpp
  picking.cpp
)

target_include_directories(robosim_viz_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(robosim_viz_core
  PRIVATE robosim_baseline
)

# ---------------------------------------------------------------------------
# robosim_viz_edit_builder — robot_description -> scene_snapshot.
# Lives in its own library so robosim_viz_core remains description-free.
# ---------------------------------------------------------------------------

add_library(robosim_viz_edit_builder STATIC
  edit_mode_builder.cpp
)

target_link_libraries(robosim_viz_edit_builder
  PUBLIC robosim_viz_core robosim_description
  PRIVATE robosim_baseline
)

# ---------------------------------------------------------------------------
# robosim_viz_edit_apply — apply_gizmo_target. Pure logic for writing a
# gizmo target back to the description's origin fields. Depends on the
# loader (uses compose_origin / decompose_origin) and on scene_snapshot
# (uses node_kind / entity_name lookup).
# ---------------------------------------------------------------------------

add_library(robosim_viz_edit_apply STATIC
  edit_mode_apply.cpp
)

target_link_libraries(robosim_viz_edit_apply
  PUBLIC robosim_viz_core robosim_description
  PRIVATE robosim_baseline
)

# ---------------------------------------------------------------------------
# robosim_viz_edit_session — load/save/reload wrapper around
# robot_description with a dirty bit.
# ---------------------------------------------------------------------------

add_library(robosim_viz_edit_session STATIC
  edit_session.cpp
)

target_include_directories(robosim_viz_edit_session PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(robosim_viz_edit_session
  PUBLIC robosim_description
  PRIVATE robosim_baseline
)

# ---------------------------------------------------------------------------
# robosim_viz_renderer — OpenGL primitive renderer. Reads scene_snapshot
# only. The T1 isolation check asserts it does not transitively link
# robosim_description.
# ---------------------------------------------------------------------------

add_library(robosim_viz_renderer STATIC
  renderer.cpp
)

target_link_libraries(robosim_viz_renderer
  PUBLIC
    robosim_viz_core
    robosim_viz_glad
    glm::glm
  PRIVATE
    robosim_baseline
)

# ---------------------------------------------------------------------------
# robosim_viz_panels — ImGui panels (scene tree, inspector, status bar).
# Reads from the loaded description for inspector field display.
# ---------------------------------------------------------------------------

add_library(robosim_viz_panels STATIC
  panels.cpp
)

target_link_libraries(robosim_viz_panels
  PUBLIC
    robosim_viz_core
    robosim_description
    robosim_viz_imgui
  PRIVATE
    robosim_baseline
)

# ---------------------------------------------------------------------------
# Renderer / loader isolation check (TEST_PLAN.md T1).
# ---------------------------------------------------------------------------

include("${CMAKE_SOURCE_DIR}/cmake/lint_renderer_isolation.cmake")
robosim_viz_check_renderer_isolation()

# ---------------------------------------------------------------------------
# robosim-viz binary.
# ---------------------------------------------------------------------------

add_executable(robosim-viz
  main.cpp
)

target_link_libraries(robosim-viz
  PRIVATE
    robosim_viz_core
    robosim_viz_edit_builder
    robosim_viz_edit_apply
    robosim_viz_edit_session
    robosim_viz_renderer
    robosim_viz_panels
    robosim_viz_imgui
    robosim_viz_imguizmo
    robosim_viz_glad
    robosim_description
    glfw
    glm::glm
    robosim_baseline
)
