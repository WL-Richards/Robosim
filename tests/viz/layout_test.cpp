// Visualizer layout-persistence tests. See TEST_PLAN_VL.md (rev 4,
// ready-to-implement). Determinism exception applies (visualizer
// subtree); these tests touch the filesystem and create an ImGui
// context, which is fine here.

#include "viz/layout.h"

#include <gtest/gtest.h>

#include <imgui.h>
#include <imgui_internal.h>

#include <array>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

namespace robosim::viz {
namespace {

namespace fs = std::filesystem;

// L1
TEST(LayoutPath, resolve_imgui_ini_path_uses_xdg_config_home_when_set) {
  const auto p = resolve_imgui_ini_path("/tmp/xdg-fake", "/home/anybody");
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(*p, fs::path("/tmp/xdg-fake/robosim-viz/imgui.ini"));
}

// L2
class LayoutPathUnsetXdg : public ::testing::TestWithParam<const char*> {};

TEST_P(LayoutPathUnsetXdg,
       resolve_imgui_ini_path_falls_back_to_home_dot_config_when_xdg_unset) {
  const auto p = resolve_imgui_ini_path(GetParam(), "/home/will");
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(*p, fs::path("/home/will/.config/robosim-viz/imgui.ini"));
}

INSTANTIATE_TEST_SUITE_P(NullAndEmpty,
                         LayoutPathUnsetXdg,
                         ::testing::Values(static_cast<const char*>(nullptr),
                                           ""));

// L3
struct nullopt_case {
  const char* xdg;
  const char* home;
};

class LayoutPathBothUnset : public ::testing::TestWithParam<nullopt_case> {};

TEST_P(LayoutPathBothUnset,
       resolve_imgui_ini_path_returns_nullopt_when_neither_env_var_set) {
  const auto& c = GetParam();
  EXPECT_EQ(resolve_imgui_ini_path(c.xdg, c.home), std::nullopt);
}

INSTANTIATE_TEST_SUITE_P(
    AllNullEmptyCombos,
    LayoutPathBothUnset,
    ::testing::Values(nullopt_case{nullptr, nullptr},
                      nullopt_case{nullptr, ""},
                      nullopt_case{"", nullptr},
                      nullopt_case{"", ""}));

// L4 — case A (relative XDG + valid HOME → HOME-derived) and
// case B (relative XDG + null HOME → nullopt). The conjunction
// pins that relative XDG is *rejected* rather than silently
// passed through std::filesystem::absolute().
TEST(LayoutPath,
     resolve_imgui_ini_path_rejects_relative_xdg_falls_through_to_home) {
  const auto p = resolve_imgui_ini_path("relative/path", "/home/will");
  ASSERT_TRUE(p.has_value());
  EXPECT_EQ(*p, fs::path("/home/will/.config/robosim-viz/imgui.ini"));
}

TEST(LayoutPath,
     resolve_imgui_ini_path_rejects_relative_xdg_with_no_home_returns_nullopt) {
  EXPECT_EQ(resolve_imgui_ini_path("relative/path", nullptr), std::nullopt);
}

// L5–L10 — filesystem-touching tests share LayoutFsTest.
class LayoutFsTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const auto* info =
        ::testing::UnitTest::GetInstance()->current_test_info();
    scratch_ = fs::temp_directory_path() /
               (std::string("robosim_viz_layout_") +
                info->test_suite_name() + "_" + info->name());
    std::error_code ec;
    fs::remove_all(scratch_, ec);
    fs::create_directories(scratch_);
  }

  void TearDown() override {
    std::error_code ec;
    fs::remove_all(scratch_, ec);
  }

  fs::path scratch_;
};

// L5
TEST_F(LayoutFsTest,
       ensure_ini_parent_dir_creates_missing_intermediate_directories) {
  const auto path = scratch_ / "a" / "b" / "c" / "imgui.ini";
  EXPECT_TRUE(ensure_ini_parent_dir(path));
  EXPECT_TRUE(fs::is_directory(scratch_ / "a" / "b" / "c"));
}

// L6
TEST_F(LayoutFsTest,
       ensure_ini_parent_dir_succeeds_when_directory_already_exists) {
  fs::create_directories(scratch_ / "x");
  EXPECT_TRUE(ensure_ini_parent_dir(scratch_ / "x" / "imgui.ini"));
  EXPECT_TRUE(fs::is_directory(scratch_ / "x"));
}

// L7
TEST_F(LayoutFsTest,
       ensure_ini_parent_dir_returns_false_when_parent_is_an_existing_file) {
  std::ofstream(scratch_ / "clash") << "x";
  EXPECT_FALSE(ensure_ini_parent_dir(scratch_ / "clash" / "imgui.ini"));
}

// L8
TEST_F(LayoutFsTest, should_apply_default_layout_returns_true_when_ini_missing) {
  EXPECT_TRUE(should_apply_default_layout(scratch_ / "never-written.ini"));
}

// L9
TEST_F(LayoutFsTest, should_apply_default_layout_returns_false_when_ini_present) {
  std::ofstream(scratch_ / "saved.ini") << "[Window]";
  EXPECT_FALSE(should_apply_default_layout(scratch_ / "saved.ini"));
}

// L10
TEST_F(LayoutFsTest, should_apply_default_layout_returns_false_for_empty_ini) {
  std::ofstream(scratch_ / "empty.ini");  // open and close, leaving empty
  EXPECT_FALSE(should_apply_default_layout(scratch_ / "empty.ini"));
}

// D1 — dock-layout test in a bare ImGui context (no NewFrame).
class DockLayoutTest : public ::testing::Test {
 protected:
  void SetUp() override {
    IMGUI_CHECKVERSION();
    ctx_ = ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(1280.0F, 800.0F);
    io.IniFilename = nullptr;  // suppress disk I/O
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  }

  void TearDown() override {
    ImGui::DestroyContext(ctx_);
  }

  ImGuiContext* ctx_ = nullptr;
};

TEST_F(DockLayoutTest,
       apply_default_dock_layout_assigns_panel_settings_to_distinct_leaf_nodes) {
  const ImGuiID dockspace_id = ImHashStr("test_dockspace");
  dock_layout_node_ids node_ids;

  apply_default_dock_layout(dockspace_id, &node_ids);

  // Structural shape.
  ImGuiDockNode* root = ImGui::DockBuilderGetNode(dockspace_id);
  ASSERT_NE(root, nullptr);
  EXPECT_TRUE(root->IsSplitNode());

  // DockBuilderGetCentralNode reads root->CentralNode, which is
  // populated by the per-frame DockNodeUpdateForRootNode walk —
  // this test runs without NewFrame, so we read IsCentralNode()
  // directly from the node identified by out_node_ids.central_id.
  ImGuiDockNode* central = ImGui::DockBuilderGetNode(node_ids.central_id);
  ASSERT_NE(central, nullptr);
  EXPECT_TRUE(central->IsCentralNode());
  EXPECT_TRUE(central->IsLeafNode());

  // Per-window settings assignment (the pre-Begin observable).
  const auto dock_id_for = [](const char* name) -> ImGuiID {
    ImGuiWindowSettings* s = ImGui::FindWindowSettingsByID(ImHashStr(name));
    return s != nullptr ? s->DockId : 0;
  };
  EXPECT_EQ(dock_id_for(default_layout_scene_window),
            node_ids.scene_leaf_id);
  EXPECT_EQ(dock_id_for(default_layout_inspector_window),
            node_ids.inspector_leaf_id);

  // Gizmo is intentionally NOT docked by default — it floats as a
  // transparent HUD over the central viewport. Either no settings
  // exist for the Gizmo window yet, or they exist with DockId == 0.
  EXPECT_EQ(dock_id_for(default_layout_gizmo_window), 0u);

  // Pairwise distinct + non-zero leaf IDs across the three docked
  // nodes (Scene leaf, Inspector leaf, central).
  const std::array<ImGuiID, 3> ids = {node_ids.scene_leaf_id,
                                      node_ids.inspector_leaf_id,
                                      node_ids.central_id};
  for (std::size_t i = 0; i < ids.size(); ++i) {
    EXPECT_NE(ids[i], 0u) << "leaf " << i << " was not produced";
    for (std::size_t j = i + 1; j < ids.size(); ++j) {
      EXPECT_NE(ids[i], ids[j])
          << "leaves " << i << " and " << j << " collided";
    }
  }
}

}  // namespace
}  // namespace robosim::viz
