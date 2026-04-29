// Phase VD section A — apply_gizmo_target (the gizmo write path).
// See tests/viz/TEST_PLAN_VD.md "A. apply_gizmo_target".

#include "viz/edit_mode_apply.h"

#include "fixtures.h"
#include "viz/edit_mode_builder.h"
#include "viz/scene_snapshot.h"

#include "description/origin_pose.h"
#include "description/schema.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>

namespace robosim::viz {
namespace {

namespace desc = robosim::description;

constexpr double tol_1e_9 = 1e-9;

void expect_matrix_near(const transform& a, const transform& b, double tol) {
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      EXPECT_NEAR(a.m[col][row], b.m[col][row], tol)
          << "mismatch at m[" << col << "][" << row << "]";
    }
  }
}

transform from_4x4(const desc::transform_4x4& m) {
  transform t;
  t.m = m;
  return t;
}

transform make_translation_transform(double x, double y, double z) {
  transform t;
  t.m[0][0] = 1.0;
  t.m[1][1] = 1.0;
  t.m[2][2] = 1.0;
  t.m[3][3] = 1.0;
  t.m[3][0] = x;
  t.m[3][1] = y;
  t.m[3][2] = z;
  return t;
}

// ---------------------------------------------------------------------
// A1 — pure translation at root joint.
// ---------------------------------------------------------------------

TEST(EditModeApply, AppliesPureTranslationToRootJointOrigin) {
  const auto source = testing::make_v0_arm_description();
  const auto snapshot = build_edit_mode_snapshot(source);
  const auto shoulder_idx =
      testing::find_node_index(snapshot, node_kind::joint, "shoulder");

  const auto target = make_translation_transform(0.7, 0.0, 0.0);
  const auto result = apply_gizmo_target(source, snapshot, shoulder_idx, target);

  EXPECT_EQ(result.joints[0].origin.xyz_m,
            (std::array<double, 3>{0.7, 0.0, 0.0}));
  EXPECT_EQ(result.joints[0].origin.rpy_rad,
            (std::array<double, 3>{0.0, 0.0, 0.0}));
}

// ---------------------------------------------------------------------
// A2 — rotation via composition round-trip (parameterized).
// ---------------------------------------------------------------------

class ApplyRotationRoundTrip
    : public ::testing::TestWithParam<std::array<double, 3>> {};

TEST_P(ApplyRotationRoundTrip, AppliesRotationViaCompositionRoundTrip) {
  const auto rpy = GetParam();
  const auto source = testing::make_v0_arm_description();
  const auto snapshot = build_edit_mode_snapshot(source);
  const auto shoulder_idx =
      testing::find_node_index(snapshot, node_kind::joint, "shoulder");

  const auto target = from_4x4(
      desc::compose_origin({.xyz_m = {0.0, 0.0, 0.0}, .rpy_rad = rpy}));
  const auto result = apply_gizmo_target(source, snapshot, shoulder_idx, target);

  const auto m_round = from_4x4(desc::compose_origin(result.joints[0].origin));
  expect_matrix_near(target, m_round, tol_1e_9);
}

INSTANTIATE_TEST_SUITE_P(
    Cases,
    ApplyRotationRoundTrip,
    ::testing::Values(
        std::array<double, 3>{+M_PI / 4.0, +M_PI / 6.0, +M_PI / 3.0},
        std::array<double, 3>{-M_PI / 4.0, +M_PI / 6.0, -M_PI / 3.0},
        std::array<double, 3>{+M_PI / 4.0, -M_PI / 6.0, +M_PI / 3.0},
        std::array<double, 3>{0.0, +M_PI / 2.0 - 0.05, 0.0}));

// ---------------------------------------------------------------------
// A3 — apply at child joint inverts parent kinematic.
// ---------------------------------------------------------------------

TEST(EditModeApply, AppliesAtChildJointInvertsParentKinematic) {
  const desc::origin_pose identity{};
  const auto source = testing::make_two_joint_chain_with_origins(
      desc::origin_pose{.xyz_m = {1.0, 0.0, 0.0}, .rpy_rad = {0.0, 0.0, 0.0}},
      identity, identity, identity);
  const auto snapshot = build_edit_mode_snapshot(source);
  const auto elbow_idx =
      testing::find_node_index(snapshot, node_kind::joint, "elbow");

  const auto target = make_translation_transform(1.5, 0.0, 0.0);
  const auto result = apply_gizmo_target(source, snapshot, elbow_idx, target);

  // Field assertion.
  EXPECT_EQ(result.joints[1].origin.xyz_m,
            (std::array<double, 3>{0.5, 0.0, 0.0}));
  EXPECT_EQ(result.joints[1].origin.rpy_rad,
            (std::array<double, 3>{0.0, 0.0, 0.0}));

  // Rebuild assertion — what the user sees in the viewport.
  const auto rebuilt = build_edit_mode_snapshot(result);
  const auto rebuilt_elbow_idx =
      testing::find_node_index(rebuilt, node_kind::joint, "elbow");
  EXPECT_EQ(rebuilt.nodes[rebuilt_elbow_idx].world_from_local.m[3][0], 1.5);
  EXPECT_EQ(rebuilt.nodes[rebuilt_elbow_idx].world_from_local.m[3][1], 0.0);
  EXPECT_EQ(rebuilt.nodes[rebuilt_elbow_idx].world_from_local.m[3][2], 0.0);
}

// ---------------------------------------------------------------------
// A4 — apply at link inverts link kinematic to write visual_origin.
// ---------------------------------------------------------------------

TEST(EditModeApply, AppliesAtLinkInvertsLinkKinematicToWriteVisualOrigin) {
  // Powers-of-2 throughout so the inverse / matmul / decompose chain
  // produces bit-exact results (0.5, 0.75, 0.25 are all representable;
  // 0.7 / 0.2 would round through the subtraction and force a tolerance
  // band even on translation-only paths).
  const auto source = testing::make_v0_arm_with_joint_origin(
      desc::origin_pose{.xyz_m = {0.5, 0.0, 0.0}, .rpy_rad = {0.0, 0.0, 0.0}});
  const auto snapshot = build_edit_mode_snapshot(source);
  const auto arm_idx =
      testing::find_node_index(snapshot, node_kind::link, "arm");

  const auto target = make_translation_transform(0.75, 0.0, 0.0);
  const auto result = apply_gizmo_target(source, snapshot, arm_idx, target);

  // Field assertions.
  EXPECT_EQ(result.links[0].visual_origin.xyz_m,
            (std::array<double, 3>{0.25, 0.0, 0.0}));
  EXPECT_EQ(result.joints[0].origin.xyz_m,
            (std::array<double, 3>{0.5, 0.0, 0.0}));  // joint untouched

  // Rebuild assertion.
  const auto rebuilt = build_edit_mode_snapshot(result);
  const auto rebuilt_arm_idx =
      testing::find_node_index(rebuilt, node_kind::link, "arm");
  EXPECT_EQ(rebuilt.nodes[rebuilt_arm_idx].world_from_local.m[3][0], 0.75);
  EXPECT_EQ(rebuilt.nodes[rebuilt_arm_idx].world_from_local.m[3][1], 0.0);
  EXPECT_EQ(rebuilt.nodes[rebuilt_arm_idx].world_from_local.m[3][2], 0.0);
}

// ---------------------------------------------------------------------
// A4b — rotation on a link round-trips under rebuild.
// ---------------------------------------------------------------------

TEST(EditModeApply, AppliesRotationAtLinkRoundTripsUnderRebuild) {
  const auto source = testing::make_v0_arm_with_joint_origin(
      desc::origin_pose{.xyz_m = {0.0, 0.0, 0.0},
                        .rpy_rad = {0.0, 0.0, +M_PI / 4.0}});
  const auto snapshot = build_edit_mode_snapshot(source);
  const auto arm_idx =
      testing::find_node_index(snapshot, node_kind::link, "arm");

  const auto target = from_4x4(desc::compose_origin(
      {.xyz_m = {0.1, 0.2, 0.3},
       .rpy_rad = {+M_PI / 6.0, -M_PI / 8.0, +M_PI / 12.0}}));
  const auto result = apply_gizmo_target(source, snapshot, arm_idx, target);

  // Joint untouched.
  EXPECT_EQ(result.joints[0].origin, source.joints[0].origin);

  // Rebuild assertion.
  const auto rebuilt = build_edit_mode_snapshot(result);
  const auto rebuilt_arm_idx =
      testing::find_node_index(rebuilt, node_kind::link, "arm");
  expect_matrix_near(rebuilt.nodes[rebuilt_arm_idx].world_from_local, target,
                     tol_1e_9);
}

// ---------------------------------------------------------------------
// A5 — unrelated fields copied through.
// ---------------------------------------------------------------------

TEST(EditModeApply, UnrelatedFieldsAreCopiedThrough) {
  const auto source = testing::make_v0_arm_description();
  const auto snapshot = build_edit_mode_snapshot(source);
  const auto shoulder_idx =
      testing::find_node_index(snapshot, node_kind::joint, "shoulder");

  const auto target = make_translation_transform(0.5, 0.0, 0.0);
  const auto result = apply_gizmo_target(source, snapshot, shoulder_idx, target);

  EXPECT_EQ(result.name, source.name);
  EXPECT_EQ(result.schema_version, source.schema_version);
  EXPECT_EQ(result.motors, source.motors);
  EXPECT_EQ(result.sensors, source.sensors);
  EXPECT_EQ(result.links[0].mass_kg, source.links[0].mass_kg);
  EXPECT_EQ(result.links[0].length_m, source.links[0].length_m);
  EXPECT_EQ(result.links[0].inertia_kgm2, source.links[0].inertia_kgm2);
  EXPECT_EQ(result.links[0].visual_origin, source.links[0].visual_origin);

  EXPECT_EQ(result.joints[0].name, source.joints[0].name);
  EXPECT_EQ(result.joints[0].type, source.joints[0].type);
  EXPECT_EQ(result.joints[0].parent, source.joints[0].parent);
  EXPECT_EQ(result.joints[0].child, source.joints[0].child);
  EXPECT_EQ(result.joints[0].axis, source.joints[0].axis);
  EXPECT_EQ(result.joints[0].limit_lower_rad, source.joints[0].limit_lower_rad);
  EXPECT_EQ(result.joints[0].limit_upper_rad, source.joints[0].limit_upper_rad);
  EXPECT_EQ(result.joints[0].viscous_friction_nm_per_rad_per_s,
            source.joints[0].viscous_friction_nm_per_rad_per_s);
  // Only origin differs.
  EXPECT_NE(result.joints[0].origin, source.joints[0].origin);
}

// ---------------------------------------------------------------------
// A6 — identity-rotation no-op apply at root joint returns equal description.
// ---------------------------------------------------------------------

TEST(EditModeApply, IdentityRotationNoOpApplyAtRootJointReturnsEqualDescription) {
  const auto source = testing::make_v0_arm_with_joint_origin(
      desc::origin_pose{.xyz_m = {0.5, 0.0, 0.0}, .rpy_rad = {0.0, 0.0, 0.0}});
  const auto snapshot = build_edit_mode_snapshot(source);
  const auto shoulder_idx =
      testing::find_node_index(snapshot, node_kind::joint, "shoulder");

  const auto target = snapshot.nodes[shoulder_idx].world_from_local;
  const auto result = apply_gizmo_target(source, snapshot, shoulder_idx, target);

  EXPECT_EQ(result, source);
}

// ---------------------------------------------------------------------
// A7 — applied target reproduces under snapshot rebuild.
// ---------------------------------------------------------------------

TEST(EditModeApply, AppliedTargetReproducesUnderSnapshotRebuild) {
  const auto source = testing::make_v0_arm_description();
  const auto snapshot = build_edit_mode_snapshot(source);
  const auto shoulder_idx =
      testing::find_node_index(snapshot, node_kind::joint, "shoulder");

  const auto target = from_4x4(desc::compose_origin(
      {.xyz_m = {0.3, 0.4, 0.5},
       .rpy_rad = {+M_PI / 4.0, +M_PI / 6.0, +M_PI / 3.0}}));
  const auto result = apply_gizmo_target(source, snapshot, shoulder_idx, target);

  const auto rebuilt = build_edit_mode_snapshot(result);
  const auto rebuilt_idx =
      testing::find_node_index(rebuilt, node_kind::joint, "shoulder");
  expect_matrix_near(rebuilt.nodes[rebuilt_idx].world_from_local, target,
                     tol_1e_9);
}

// ---------------------------------------------------------------------
// A8 — apply at gimbal pole preserves composed transform.
// ---------------------------------------------------------------------

TEST(EditModeApply, AtGimbalPolePreservesComposedTransformOnRebuild) {
  const auto source = testing::make_v0_arm_description();
  const auto snapshot = build_edit_mode_snapshot(source);
  const auto shoulder_idx =
      testing::find_node_index(snapshot, node_kind::joint, "shoulder");

  const auto target = from_4x4(desc::compose_origin(
      {.xyz_m = {0.0, 0.0, 0.0},
       .rpy_rad = {+0.4, +M_PI / 2.0, -0.3}}));
  const auto result = apply_gizmo_target(source, snapshot, shoulder_idx, target);

  // Finite output (no NaN poisoning).
  EXPECT_FALSE(desc::validate_finite_xyz(
      result.joints[0].origin.xyz_m, "/test/xyz_m"));
  EXPECT_FALSE(desc::validate_finite_rpy(
      result.joints[0].origin.rpy_rad, "/test/rpy_rad"));

  const auto rebuilt = build_edit_mode_snapshot(result);
  const auto rebuilt_idx =
      testing::find_node_index(rebuilt, node_kind::joint, "shoulder");
  expect_matrix_near(rebuilt.nodes[rebuilt_idx].world_from_local, target,
                     tol_1e_9);
}

}  // namespace
}  // namespace robosim::viz
