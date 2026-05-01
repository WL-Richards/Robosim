#pragma once

// Layout persistence for the visualizer:
//
//   * resolve_imgui_ini_path: per-user persistent path for ImGui's ini
//     (positions, dock layout). Argument-driven; the caller in main.cpp
//     reads $XDG_CONFIG_HOME / $HOME via getenv once and forwards them.
//
//   * ensure_ini_parent_dir: create the parent directory chain for
//     ini_path so ImGui's first save doesn't fail.
//
//   * should_apply_default_layout: true iff ini_path doesn't yet
//     exist. Caller uses this to decide whether to call
//     apply_default_dock_layout on first run.
//
//   * apply_default_dock_layout: build the first-run dock layout using
//     the ImGui::DockBuilder* API. Does NOT require an active frame —
//     see the doc comment below for the load-bearing constraint.
//
// See tests/viz/TEST_PLAN_VL.md (rev 4, ready-to-implement) for the
// pinned contracts.

#include <filesystem>
#include <optional>

namespace robosim::viz {

[[nodiscard]] std::optional<std::filesystem::path> resolve_imgui_ini_path(
    const char* xdg_config_home,
    const char* home);

[[nodiscard]] bool ensure_ini_parent_dir(
    const std::filesystem::path& ini_path);

[[nodiscard]] bool should_apply_default_layout(
    const std::filesystem::path& ini_path);

inline constexpr const char* default_layout_scene_window     = "Scene";
inline constexpr const char* default_layout_inspector_window = "Inspector";
inline constexpr const char* default_layout_gizmo_window     = "Gizmo";

struct dock_layout_node_ids {
  unsigned int scene_leaf_id     = 0;
  unsigned int inspector_leaf_id = 0;
  unsigned int central_id        = 0;
};

// Constraint (load-bearing for testability): the implementation MUST
// call ImGui::DockBuilderAddNode(dockspace_id, 0) — not with the
// ImGuiDockNodeFlags_DockSpace flag. With that flag, DockBuilderAddNode
// dispatches to ImGui::DockSpace, which calls GetCurrentWindowRead()
// and crashes when no frame is active (test harness has no NewFrame).
// Production callers create the dockspace via DockSpaceOverViewport
// each frame, so this helper only configures an already-alive
// dockspace.
void apply_default_dock_layout(unsigned int dockspace_id,
                               dock_layout_node_ids* out_node_ids = nullptr);

}  // namespace robosim::viz
