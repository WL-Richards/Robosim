#include "viz/edit_mode_builder.h"

#include "fixtures.h"
#include "viz/scene_snapshot.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <optional>
#include <ranges>
#include <vector>

namespace robosim::viz {
namespace {

using testing::find_node;
using testing::find_node_index;

TEST(EditModeBuilder, SnapshotNodeCountMatchesLinksPlusJoints) {
  const auto snapshot = build_edit_mode_snapshot(testing::make_v0_arm_description());

  EXPECT_EQ(snapshot.nodes.size(), 2U);
}

TEST(EditModeBuilder, SnapshotDistinguishesLinkAndJointNodesByKind) {
  const auto snapshot = build_edit_mode_snapshot(testing::make_v0_arm_description());

  const auto link_count =
      std::ranges::count_if(snapshot.nodes, [](auto& n) { return n.kind == node_kind::link; });
  const auto joint_count =
      std::ranges::count_if(snapshot.nodes, [](auto& n) { return n.kind == node_kind::joint; });
  EXPECT_EQ(link_count, 1);
  EXPECT_EQ(joint_count, 1);
  EXPECT_EQ(find_node(snapshot, node_kind::link, "arm").entity_name, "arm");
  EXPECT_EQ(find_node(snapshot, node_kind::joint, "shoulder").entity_name, "shoulder");
}

TEST(EditModeBuilder, LinkNodeHasCylinderPrimitiveWithCorrectDimensions) {
  const auto snapshot = build_edit_mode_snapshot(testing::make_v0_arm_description());

  const auto& arm = find_node(snapshot, node_kind::link, "arm");
  ASSERT_TRUE(arm.shape.has_value());
  EXPECT_EQ(arm.shape->kind, primitive_kind::cylinder);
  EXPECT_DOUBLE_EQ(arm.shape->length_m, 0.5);
  EXPECT_DOUBLE_EQ(arm.shape->radius_m, 0.05);
}

TEST(EditModeBuilder, JointNodeHasArrowPrimitiveWithCorrectDimensions) {
  const auto snapshot = build_edit_mode_snapshot(testing::make_v0_arm_description());

  const auto& shoulder = find_node(snapshot, node_kind::joint, "shoulder");
  ASSERT_TRUE(shoulder.shape.has_value());
  EXPECT_EQ(shoulder.shape->kind, primitive_kind::arrow);
  EXPECT_DOUBLE_EQ(shoulder.shape->length_m, 0.20);
  EXPECT_DOUBLE_EQ(shoulder.shape->radius_m, 0.01);
}

TEST(EditModeBuilder, JointNodeRecordsAxisInLocalFrame) {
  const auto snapshot = build_edit_mode_snapshot(testing::make_v0_arm_description());

  const auto& shoulder = find_node(snapshot, node_kind::joint, "shoulder");
  EXPECT_EQ(shoulder.joint_axis_local, (std::array{0.0, 1.0, 0.0}));
}

TEST(EditModeBuilder, ParentIndexesRepresentKinematicTree) {
  const auto snapshot = build_edit_mode_snapshot(testing::make_v0_arm_description());

  const auto shoulder_index = find_node_index(snapshot, node_kind::joint, "shoulder");
  const auto& shoulder = snapshot.nodes[shoulder_index];
  const auto& arm = find_node(snapshot, node_kind::link, "arm");

  EXPECT_EQ(shoulder.parent_index, std::nullopt);
  ASSERT_TRUE(arm.parent_index.has_value());
  EXPECT_EQ(*arm.parent_index, shoulder_index);
}

class JointOrder : public ::testing::TestWithParam<bool> {};

TEST_P(JointOrder, MultiLinkChainPreservesTopologicalOrder) {
  const auto snapshot =
      build_edit_mode_snapshot(testing::make_two_joint_chain_description(GetParam()));

  const auto shoulder = find_node_index(snapshot, node_kind::joint, "shoulder");
  const auto arm = find_node_index(snapshot, node_kind::link, "arm");
  const auto elbow = find_node_index(snapshot, node_kind::joint, "elbow");
  const auto forearm = find_node_index(snapshot, node_kind::link, "forearm");

  EXPECT_LT(shoulder, arm);
  EXPECT_LT(arm, elbow);
  EXPECT_LT(elbow, forearm);
  EXPECT_EQ(snapshot.nodes[elbow].parent_index, std::optional{arm});
  EXPECT_EQ(snapshot.nodes[forearm].parent_index, std::optional{elbow});
}

INSTANTIATE_TEST_SUITE_P(TopologicalAndReverseSourceOrder,
                         JointOrder,
                         ::testing::Values(false, true));

TEST(EditModeBuilder, SnapshotIsValueEqualAcrossFreshBuilds) {
  const auto expected = build_edit_mode_snapshot(testing::make_two_joint_chain_description());

  for (int i = 0; i < 7; ++i) {
    EXPECT_EQ(build_edit_mode_snapshot(testing::make_two_joint_chain_description()), expected);
  }
}

}  // namespace
}  // namespace robosim::viz
