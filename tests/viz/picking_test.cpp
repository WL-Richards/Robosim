#include "viz/picking.h"

#include "viz/scene_snapshot.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <numbers>
#include <optional>

namespace robosim::viz {
namespace {

constexpr double kTolerance = 1e-9;

transform translated(double x, double y, double z) {
  transform t = transform::identity();
  t.m[3][0] = x;
  t.m[3][1] = y;
  t.m[3][2] = z;
  return t;
}

transform rotate_x(double radians) {
  transform t = transform::identity();
  const double c = std::cos(radians);
  const double s = std::sin(radians);
  t.m[1][1] = c;
  t.m[1][2] = s;
  t.m[2][1] = -s;
  t.m[2][2] = c;
  return t;
}

scene_node shaped_node(primitive p, transform world_from_local) {
  scene_node node;
  node.entity_name = "node";
  node.kind = node_kind::link;
  node.world_from_local = world_from_local;
  node.shape = p;
  return node;
}

TEST(Picking, PicksCylinderWhenRayPassesThroughAxis) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(shaped_node(primitive{primitive_kind::cylinder, 1.0, 0.1, 0.0, 0.0, 0.0},
                                       transform::identity()));

  // Cylinder spans z ∈ [0, 1] per TEST_PLAN convention #8.
  const auto hit = pick(snapshot, ray{{0.0, 0.0, -10.0}, {0.0, 0.0, 1.0}});

  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->node_index, 0U);
  EXPECT_NEAR(hit->t_along_ray, 10.0, kTolerance);
}

TEST(Picking, MissesCylinderWhenRayPassesOutsideRadius) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(shaped_node(primitive{primitive_kind::cylinder, 1.0, 0.1, 0.0, 0.0, 0.0},
                                       transform::identity()));

  EXPECT_EQ(pick(snapshot, ray{{0.5, 0.0, -10.0}, {0.0, 0.0, 1.0}}), std::nullopt);
}

TEST(Picking, MissesCylinderPastFarCapWhenDirectionIsAway) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(shaped_node(primitive{primitive_kind::cylinder, 1.0, 0.1, 0.0, 0.0, 0.0},
                                       transform::identity()));

  EXPECT_EQ(pick(snapshot, ray{{0.0, 0.0, 10.0}, {0.0, 0.0, 1.0}}), std::nullopt);
}

TEST(Picking, HitsCylinderPastFarCapWhenDirectionReturns) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(shaped_node(primitive{primitive_kind::cylinder, 1.0, 0.1, 0.0, 0.0, 0.0},
                                       transform::identity()));

  // Cylinder spans z ∈ [0, 1]; ray from z=10 returning entering far
  // cap at z=1, t = 10 - 1 = 9.
  const auto hit = pick(snapshot, ray{{0.0, 0.0, 10.0}, {0.0, 0.0, -1.0}});

  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->t_along_ray, 9.0, kTolerance);
}

TEST(Picking, PicksCloserOfTwoCylindersAlongRay) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(shaped_node(primitive{primitive_kind::cylinder, 1.0, 0.1, 0.0, 0.0, 0.0},
                                       transform::identity()));
  snapshot.nodes.push_back(shaped_node(primitive{primitive_kind::cylinder, 1.0, 0.1, 0.0, 0.0, 0.0},
                                       translated(0.0, 0.0, 5.0)));

  // Cylinder 0 spans z ∈ [0, 1]; cylinder 1 spans z ∈ [5, 6].
  const auto hit = pick(snapshot, ray{{0.0, 0.0, -10.0}, {0.0, 0.0, 1.0}});

  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->node_index, 0U);
  EXPECT_NEAR(hit->t_along_ray, 10.0, kTolerance);
}

TEST(Picking, GrazingRayInsideBoundingSphereButOutsideCylinderMisses) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(shaped_node(primitive{primitive_kind::cylinder, 1.0, 0.1, 0.0, 0.0, 0.0},
                                       transform::identity()));

  EXPECT_EQ(pick(snapshot, ray{{0.3, 0.0, -10.0}, {0.0, 0.0, 1.0}}), std::nullopt);
}

TEST(Picking, RayStartingInsidePrimitiveReturnsZero) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(shaped_node(primitive{primitive_kind::sphere, 0.0, 1.0, 0.0, 0.0, 0.0},
                                       transform::identity()));

  const auto hit = pick(snapshot, ray{{0.25, 0.0, 0.0}, {1.0, 0.0, 0.0}});

  ASSERT_TRUE(hit.has_value());
  EXPECT_DOUBLE_EQ(hit->t_along_ray, 0.0);
}

TEST(Picking, ParallelRayExactlyAtCylinderRadiusIsClosedShapeHit) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(shaped_node(primitive{primitive_kind::cylinder, 1.0, 0.1, 0.0, 0.0, 0.0},
                                       transform::identity()));

  // Cylinder spans z ∈ [0, 1]; ray hits proximal cap rim at z = 0.
  const auto hit = pick(snapshot, ray{{0.1, 0.0, -10.0}, {0.0, 0.0, 1.0}});

  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->t_along_ray, 10.0, kTolerance);
}

TEST(Picking, BoxUsesExplicitHalfExtents) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(
      shaped_node(primitive{primitive_kind::box, 0.0, 0.0, 0.5, 1.0, 1.5}, transform::identity()));

  const auto hit = pick(snapshot, ray{{2.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}});

  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->t_along_ray, 1.5, kTolerance);
}

TEST(Picking, PicksBoxPrimitiveViaSlabTest) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(
      shaped_node(primitive{primitive_kind::box, 0.0, 0.0, 0.5, 0.5, 0.5}, transform::identity()));

  const auto hit = pick(snapshot, ray{{-10.0, 0.0, 0.0}, {1.0, 0.0, 0.0}});

  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->t_along_ray, 9.5, kTolerance);
}

TEST(Picking, MissesBoxWhenRayPassesOutsideSlab) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(
      shaped_node(primitive{primitive_kind::box, 0.0, 0.0, 0.5, 0.5, 0.5}, transform::identity()));

  EXPECT_EQ(pick(snapshot, ray{{-10.0, 1.0, 0.0}, {1.0, 0.0, 0.0}}), std::nullopt);
}

TEST(Picking, PicksSphereWhenRayPassesThroughCenter) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(shaped_node(primitive{primitive_kind::sphere, 0.0, 0.5, 0.0, 0.0, 0.0},
                                       transform::identity()));

  const auto hit = pick(snapshot, ray{{0.0, 0.0, -10.0}, {0.0, 0.0, 1.0}});

  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->t_along_ray, 9.5, kTolerance);
}

TEST(Picking, MissesSphereWhenRayPassesOutsideRadius) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(shaped_node(primitive{primitive_kind::sphere, 0.0, 0.5, 0.0, 0.0, 0.0},
                                       transform::identity()));

  EXPECT_EQ(pick(snapshot, ray{{1.0, 0.0, -10.0}, {0.0, 0.0, 1.0}}), std::nullopt);
}

TEST(Picking, ChoosesNearestNodeAlongRay) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(shaped_node(primitive{primitive_kind::sphere, 0.0, 0.5, 0.0, 0.0, 0.0},
                                       translated(0.0, 0.0, 5.0)));
  snapshot.nodes.push_back(shaped_node(primitive{primitive_kind::sphere, 0.0, 0.5, 0.0, 0.0, 0.0},
                                       translated(0.0, 0.0, 2.0)));

  const auto hit = pick(snapshot, ray{{0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}});

  ASSERT_TRUE(hit.has_value());
  EXPECT_EQ(hit->node_index, 1U);
  EXPECT_NEAR(hit->t_along_ray, 1.5, kTolerance);
}

TEST(Picking, RayPointingAwayFromPrimitiveDoesNotPickIt) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(shaped_node(primitive{primitive_kind::cylinder, 1.0, 0.1, 0.0, 0.0, 0.0},
                                       transform::identity()));

  EXPECT_EQ(pick(snapshot, ray{{0.0, 0.0, -10.0}, {0.0, 0.0, -1.0}}), std::nullopt);
}

TEST(Picking, IgnoresNodesWithoutShapes) {
  scene_snapshot snapshot;
  scene_node empty;
  empty.entity_name = "empty";
  empty.kind = node_kind::link;
  empty.world_from_local = transform::identity();
  snapshot.nodes.push_back(empty);

  EXPECT_EQ(pick(snapshot, ray{{0.0, 0.0, -10.0}, {0.0, 0.0, 1.0}}), std::nullopt);
}

TEST(Picking, AppliesNodeWorldTransformBeforeTesting) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(shaped_node(primitive{primitive_kind::cylinder, 1.0, 0.1, 0.0, 0.0, 0.0},
                                       translated(5.0, 0.0, 0.0)));

  // Cylinder local span z ∈ [0, 1]; world span unchanged in z (no Z
  // translation). Ray hits proximal cap at world z = 0.
  const auto hit = pick(snapshot, ray{{5.0, 0.0, -10.0}, {0.0, 0.0, 1.0}});

  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->t_along_ray, 10.0, kTolerance);
}

TEST(Picking, RespectsWorldFromLocalRotation) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(shaped_node(primitive{primitive_kind::cylinder, 1.0, 0.1, 0.0, 0.0, 0.0},
                                       rotate_x(std::numbers::pi / 2.0)));

  // Local +Z lands on world -Y under rotate_x(+π/2); local cylinder
  // span z ∈ [0, 1] becomes world y ∈ [-1, 0]. Ray entering far cap
  // at world y = -1, t = -1 - (-10) = 9.
  const auto hit = pick(snapshot, ray{{0.0, -10.0, 0.0}, {0.0, 1.0, 0.0}});

  ASSERT_TRUE(hit.has_value());
  EXPECT_NEAR(hit->t_along_ray, 9.0, kTolerance);
}

TEST(Picking, ReturnsNulloptForEmptySnapshot) {
  EXPECT_EQ(pick(scene_snapshot{}, ray{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}), std::nullopt);
}

}  // namespace
}  // namespace robosim::viz
