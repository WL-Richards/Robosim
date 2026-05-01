#include "layout.h"

#include <imgui.h>
#include <imgui_internal.h>

#include <filesystem>
#include <optional>
#include <string>
#include <system_error>

namespace robosim::viz {

namespace {

namespace fs = std::filesystem;

constexpr const char* kAppDirName = "robosim-viz";
constexpr const char* kIniFileName = "imgui.ini";

[[nodiscard]] bool is_nonempty_cstr(const char* s) {
  return s != nullptr && s[0] != '\0';
}

// Default-layout sizing ratios. UX tuning, not behavior — D1 asserts
// shape, not these values.
constexpr float kRightColumnWidthFraction   = 0.25F;
constexpr float kSceneOverInspectorFraction = 0.50F;

}  // namespace

std::optional<fs::path> resolve_imgui_ini_path(const char* xdg_config_home,
                                               const char* home) {
  if (is_nonempty_cstr(xdg_config_home)) {
    fs::path xdg(xdg_config_home);
    if (xdg.is_absolute()) {
      return xdg / kAppDirName / kIniFileName;
    }
    // Relative XDG_CONFIG_HOME is malformed per the XDG spec; fall
    // through to the HOME branch rather than silently absolutizing.
  }
  if (is_nonempty_cstr(home)) {
    return fs::path(home) / ".config" / kAppDirName / kIniFileName;
  }
  return std::nullopt;
}

bool ensure_ini_parent_dir(const fs::path& ini_path) {
  const fs::path parent = ini_path.parent_path();
  if (parent.empty()) {
    return true;
  }
  std::error_code ec;
  fs::create_directories(parent, ec);
  if (ec) {
    return false;
  }
  return fs::is_directory(parent, ec);
}

bool should_apply_default_layout(const fs::path& ini_path) {
  std::error_code ec;
  return !fs::exists(ini_path, ec);
}

void apply_default_dock_layout(unsigned int dockspace_id,
                               dock_layout_node_ids* out_node_ids) {
  // Constraint: the DockBuilderAddNode call below MUST use 0 flags
  // (not ImGuiDockNodeFlags_DockSpace). With that flag, DockBuilderAddNode
  // dispatches to ImGui::DockSpace -> GetCurrentWindowRead, which
  // crashes when no frame is active.
  ImGui::DockBuilderRemoveNode(dockspace_id);
  ImGui::DockBuilderAddNode(dockspace_id, 0);
  ImGui::DockBuilderSetNodeSize(dockspace_id, ImVec2(1280.0F, 800.0F));

  ImGuiID central_id = 0;
  ImGuiID right_id   = 0;
  ImGui::DockBuilderSplitNode(dockspace_id,
                              ImGuiDir_Right,
                              kRightColumnWidthFraction,
                              &right_id,
                              &central_id);

  ImGuiID scene_id     = 0;
  ImGuiID inspector_id = 0;
  ImGui::DockBuilderSplitNode(right_id,
                              ImGuiDir_Up,
                              kSceneOverInspectorFraction,
                              &scene_id,
                              &inspector_id);

  ImGuiDockNode* central_node = ImGui::DockBuilderGetNode(central_id);
  if (central_node != nullptr) {
    // SetLocalFlags refreshes MergedFlags; assigning LocalFlags
    // directly would leave IsCentralNode() reading stale state.
    central_node->SetLocalFlags(central_node->LocalFlags |
                                ImGuiDockNodeFlags_CentralNode |
                                ImGuiDockNodeFlags_NoTabBar);
  }

  // Scene + Inspector are docked into the right column. The Gizmo
  // window is intentionally NOT docked: it floats as a small,
  // transparent, auto-sized HUD over the central viewport. The
  // user can drag it into a dock node if they want — the saved
  // layout persists either way.
  ImGui::DockBuilderDockWindow(default_layout_scene_window, scene_id);
  ImGui::DockBuilderDockWindow(default_layout_inspector_window, inspector_id);

  ImGui::DockBuilderFinish(dockspace_id);

  if (out_node_ids != nullptr) {
    out_node_ids->scene_leaf_id     = scene_id;
    out_node_ids->inspector_leaf_id = inspector_id;
    out_node_ids->central_id        = central_id;
  }
}

}  // namespace robosim::viz
