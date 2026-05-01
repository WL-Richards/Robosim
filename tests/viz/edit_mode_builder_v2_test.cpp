// Phase VD section B — snapshot builder reads v2 origins.
// See tests/viz/TEST_PLAN_VD.md "B. Snapshot builder reads v2 origins".

#include "viz/edit_mode_builder.h"

#include "fixtures.h"
#include "viz/scene_snapshot.h"

#include "description/loader.h"
#include "description/origin_pose.h"
#include "description/schema.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <filesystem>

namespace robosim::viz {
namespace {

namespace desc = robosim::description;

constexpr double tol_1e_12 = 1e-12;

void expect_matrix_near(const transform& a,
                        const desc::transform_4x4& b,
                        double tol) {
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      EXPECT_NEAR(a.m[col][row], b[col][row], tol)
          << "mismatch at m[" << col << "][" << row << "]";
    }
  }
}

void expect_identity_exact(const transform& t) {
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      const double expected = (col == row) ? 1.0 : 0.0;
      EXPECT_EQ(t.m[col][row], expected)
          << "non-identity at m[" << col << "][" << row << "]";
    }
  }
}

// ---------------------------------------------------------------------
// B1 — joint origin translation propagates to world_from_local.
// ---------------------------------------------------------------------

TEST(EditModeBuilder, JointOriginTranslationPropagatesToWorldFromLocal) {
  const auto d = testing::make_v0_arm_with_joint_origin(
      desc::origin_pose{.xyz_m = {0.5, 0.0, 0.0}, .rpy_rad = {0.0, 0.0, 0.0}});

  const auto snapshot = build_edit_mode_snapshot(d);
  const auto& shoulder = testing::find_node(snapshot, node_kind::joint, "shoulder");

  EXPECT_EQ(shoulder.world_from_local.m[3][0], 0.5);
  EXPECT_EQ(shoulder.world_from_local.m[3][1], 0.0);
  EXPECT_EQ(shoulder.world_from_local.m[3][2], 0.0);
  EXPECT_EQ(shoulder.world_from_local.m[3][3], 1.0);
  EXPECT_EQ(shoulder.world_from_local.m[0][0], 1.0);
  EXPECT_EQ(shoulder.world_from_local.m[1][1], 1.0);
  EXPECT_EQ(shoulder.world_from_local.m[2][2], 1.0);
  EXPECT_EQ(shoulder.world_from_local.m[0][1], 0.0);
  EXPECT_EQ(shoulder.world_from_local.m[0][2], 0.0);
  EXPECT_EQ(shoulder.world_from_local.m[1][0], 0.0);
  EXPECT_EQ(shoulder.world_from_local.m[1][2], 0.0);
  EXPECT_EQ(shoulder.world_from_local.m[2][0], 0.0);
  EXPECT_EQ(shoulder.world_from_local.m[2][1], 0.0);
}

// ---------------------------------------------------------------------
// B2 — joint origin rotation matches compose_origin (joint AND child link).
// ---------------------------------------------------------------------

TEST(EditModeBuilder, JointOriginRotationMatchesComposeOriginOutput) {
  const desc::origin_pose joint_origin{
      .xyz_m = {0.0, 0.0, 0.0},
      .rpy_rad = {+M_PI / 4.0, +M_PI / 6.0, +M_PI / 3.0},
  };
  const auto d = testing::make_v0_arm_with_joint_origin(joint_origin);

  const auto snapshot = build_edit_mode_snapshot(d);
  const auto& shoulder = testing::find_node(snapshot, node_kind::joint, "shoulder");
  const auto& arm = testing::find_node(snapshot, node_kind::link, "arm");

  const auto m_expected = desc::compose_origin(joint_origin);

  expect_matrix_near(shoulder.world_from_local, m_expected, tol_1e_12);
  // Child link (arm) inherits the joint's full kinematic frame
  // because arm.visual_origin is identity in this fixture.
  expect_matrix_near(arm.world_from_local, m_expected, tol_1e_12);
}

// ---------------------------------------------------------------------
// B3 — child link inherits joint kinematic frame (translation case).
// ---------------------------------------------------------------------

TEST(EditModeBuilder, ChildLinkInheritsJointKinematicFrame) {
  const auto d = testing::make_v0_arm_with_joint_origin(
      desc::origin_pose{.xyz_m = {0.5, 0.0, 0.0}, .rpy_rad = {0.0, 0.0, 0.0}});

  const auto snapshot = build_edit_mode_snapshot(d);
  const auto& arm = testing::find_node(snapshot, node_kind::link, "arm");

  EXPECT_EQ(arm.world_from_local.m[3][0], 0.5);
  EXPECT_EQ(arm.world_from_local.m[3][1], 0.0);
  EXPECT_EQ(arm.world_from_local.m[3][2], 0.0);
  EXPECT_EQ(arm.world_from_local.m[0][0], 1.0);
  EXPECT_EQ(arm.world_from_local.m[1][1], 1.0);
  EXPECT_EQ(arm.world_from_local.m[2][2], 1.0);
}

// ---------------------------------------------------------------------
// B4 — link visual_origin offsets the link's world_from_local.
// ---------------------------------------------------------------------

TEST(EditModeBuilder, LinkVisualOriginOffsetsLinkWorldFromLocal) {
  auto d = testing::make_v0_arm_description();
  d.links[0].visual_origin =
      desc::origin_pose{.xyz_m = {0.1, 0.0, 0.0}, .rpy_rad = {0.0, 0.0, 0.0}};

  const auto snapshot = build_edit_mode_snapshot(d);
  const auto& arm = testing::find_node(snapshot, node_kind::link, "arm");

  EXPECT_EQ(arm.world_from_local.m[3][0], 0.1);
  EXPECT_EQ(arm.world_from_local.m[3][1], 0.0);
  EXPECT_EQ(arm.world_from_local.m[3][2], 0.0);
  EXPECT_EQ(arm.world_from_local.m[0][0], 1.0);
  EXPECT_EQ(arm.world_from_local.m[1][1], 1.0);
  EXPECT_EQ(arm.world_from_local.m[2][2], 1.0);
}

// ---------------------------------------------------------------------
// B5 — link visual_origin does NOT propagate to descendant joint kinematic.
// ---------------------------------------------------------------------

TEST(EditModeBuilder,
     LinkVisualOriginDoesNotPropagateToDescendantJointKinematicFrame) {
  const desc::origin_pose identity{};
  const auto d = testing::make_two_joint_chain_with_origins(
      /* shoulder_origin = */ identity,
      /* elbow_origin    = */ desc::origin_pose{.xyz_m = {0.0, 1.0, 0.0},
                                                 .rpy_rad = {0.0, 0.0, 0.0}},
      /* arm_visual      = */ desc::origin_pose{.xyz_m = {10.0, 0.0, 0.0},
                                                 .rpy_rad = {0.0, 0.0, 0.0}},
      /* forearm_visual  = */ identity);

  const auto snapshot = build_edit_mode_snapshot(d);
  const auto& arm = testing::find_node(snapshot, node_kind::link, "arm");
  const auto& elbow = testing::find_node(snapshot, node_kind::joint, "elbow");
  const auto& forearm = testing::find_node(snapshot, node_kind::link, "forearm");

  // arm.visual is shifted by visual_origin.
  EXPECT_EQ(arm.world_from_local.m[3][0], 10.0);
  EXPECT_EQ(arm.world_from_local.m[3][1], 0.0);
  EXPECT_EQ(arm.world_from_local.m[3][2], 0.0);

  // elbow's parent kinematic = arm's kinematic = identity.
  // elbow's own origin xyz = (0, 1, 0) composes against identity.
  EXPECT_EQ(elbow.world_from_local.m[3][0], 0.0);
  EXPECT_EQ(elbow.world_from_local.m[3][1], 1.0);
  EXPECT_EQ(elbow.world_from_local.m[3][2], 0.0);

  // forearm inherits elbow's kinematic; forearm's visual_origin is identity.
  EXPECT_EQ(forearm.world_from_local.m[3][0], 0.0);
  EXPECT_EQ(forearm.world_from_local.m[3][1], 1.0);
  EXPECT_EQ(forearm.world_from_local.m[3][2], 0.0);

  // Rotation blocks all identity.
  for (const auto* node : {&arm, &elbow, &forearm}) {
    EXPECT_EQ(node->world_from_local.m[0][0], 1.0);
    EXPECT_EQ(node->world_from_local.m[1][1], 1.0);
    EXPECT_EQ(node->world_from_local.m[2][2], 1.0);
  }
}

// ---------------------------------------------------------------------
// B6 — joint chain composes origins parent-to-child (left-to-right).
// ---------------------------------------------------------------------

TEST(EditModeBuilder, JointChainComposesOriginsLeftToRight) {
  const desc::origin_pose identity{};
  const auto d = testing::make_two_joint_chain_with_origins(
      /* shoulder_origin = */ desc::origin_pose{.xyz_m = {1.0, 0.0, 0.0},
                                                 .rpy_rad = {0.0, 0.0, 0.0}},
      /* elbow_origin    = */ desc::origin_pose{.xyz_m = {0.0, 1.0, 0.0},
                                                 .rpy_rad = {0.0, 0.0, 0.0}},
      /* arm_visual      = */ identity,
      /* forearm_visual  = */ identity);

  const auto snapshot = build_edit_mode_snapshot(d);
  const auto& shoulder = testing::find_node(snapshot, node_kind::joint, "shoulder");
  const auto& arm = testing::find_node(snapshot, node_kind::link, "arm");
  const auto& elbow = testing::find_node(snapshot, node_kind::joint, "elbow");
  const auto& forearm = testing::find_node(snapshot, node_kind::link, "forearm");

  EXPECT_EQ(shoulder.world_from_local.m[3][0], 1.0);
  EXPECT_EQ(shoulder.world_from_local.m[3][1], 0.0);
  EXPECT_EQ(arm.world_from_local.m[3][0], 1.0);
  EXPECT_EQ(arm.world_from_local.m[3][1], 0.0);
  EXPECT_EQ(elbow.world_from_local.m[3][0], 1.0);
  EXPECT_EQ(elbow.world_from_local.m[3][1], 1.0);
  EXPECT_EQ(forearm.world_from_local.m[3][0], 1.0);
  EXPECT_EQ(forearm.world_from_local.m[3][1], 1.0);

  for (const auto* node : {&shoulder, &arm, &elbow, &forearm}) {
    EXPECT_EQ(node->world_from_local.m[0][0], 1.0);
    EXPECT_EQ(node->world_from_local.m[1][1], 1.0);
    EXPECT_EQ(node->world_from_local.m[2][2], 1.0);
  }
}

// ---------------------------------------------------------------------
// B7 — identity origins still yield identity world_from_local.
// ---------------------------------------------------------------------

TEST(EditModeBuilder, IdentityOriginsYieldIdentityWorldFromLocal) {
  auto description = testing::make_v0_arm_description();
  for (auto& joint : description.joints) {
    joint.origin = desc::origin_pose{};
  }
  for (auto& link : description.links) {
    link.visual_origin = desc::origin_pose{};
  }
  for (auto& motor : description.motors) {
    motor.visual_origin = desc::origin_pose{};
  }

  const auto snapshot = build_edit_mode_snapshot(description);

  for (const auto& node : snapshot.nodes) {
    if (node.kind == node_kind::motor_direction_arrow) {
      continue;
    }
    expect_identity_exact(node.world_from_local);
  }
}

}  // namespace
}  // namespace robosim::viz
