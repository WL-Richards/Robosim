#include "viz/scene_snapshot.h"

#include "description/loader.h"
#include "fixtures.h"
#include "viz/edit_mode_builder.h"

#include <gtest/gtest.h>

#include <array>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

namespace robosim::viz {
namespace {

#ifndef ROBOSIM_V0_ARM_FIXTURE_PATH
#error "ROBOSIM_V0_ARM_FIXTURE_PATH not defined; CMake target setup is broken."
#endif

constexpr const char* v0_arm_fixture_path = ROBOSIM_V0_ARM_FIXTURE_PATH;

static_assert(std::is_same_v<decltype(scene_node{}.entity_name), std::string>);
static_assert(std::is_same_v<decltype(scene_node{}.contact_points_world),
                             std::vector<std::array<double, 3>>>);
static_assert(std::is_same_v<decltype(scene_node{}.shape), std::optional<primitive>>);
static_assert(std::is_same_v<decltype(scene_node{}.world_from_local), transform>);

TEST(Transform, IdentityUsesColumnMajorHomogeneousLayout) {
  const transform t = transform::identity();

  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      EXPECT_DOUBLE_EQ(t.m[col][row], col == row ? 1.0 : 0.0);
    }
  }
}

TEST(SceneSnapshot, SceneNodeSurvivesSourceDescriptionDestruction) {
  scene_snapshot from_destroyed_source;
  {
    auto desc = testing::make_long_named_description();
    from_destroyed_source = build_edit_mode_snapshot(desc);
  }

  const auto expected = build_edit_mode_snapshot(testing::make_long_named_description());
  EXPECT_EQ(from_destroyed_source, expected);
}

TEST(SceneSnapshot, PerFrameSlotsAndSelectionDefaultEmpty) {
  const auto snapshot = build_edit_mode_snapshot(testing::make_v0_arm_description());

  EXPECT_EQ(snapshot.selected_index, std::nullopt);
  for (const auto& node : snapshot.nodes) {
    EXPECT_EQ(node.joint_angle_rad, std::nullopt);
    EXPECT_TRUE(node.contact_points_world.empty());
    EXPECT_TRUE(node.force_vectors_world.empty());
  }
}

TEST(EndToEnd, LoadsV0ArmFromDiskAndBuildsConsistentSnapshot) {
  const auto loaded = description::load_from_file(v0_arm_fixture_path);
  ASSERT_TRUE(loaded.has_value()) << loaded.error().message;

  const auto from_disk = build_edit_mode_snapshot(*loaded);
  const auto from_memory = build_edit_mode_snapshot(testing::make_v0_arm_description());

  EXPECT_EQ(from_disk, from_memory);
}

}  // namespace
}  // namespace robosim::viz
