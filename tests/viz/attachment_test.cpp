#include "viz/attachment.h"

#include "viz/scene_snapshot.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

namespace robosim::viz {
namespace {

constexpr double tol = 1e-9;

scene_node node_with_transform(const transform& world_from_local) {
  scene_node n;
  n.entity_name = "moving";
  n.kind = node_kind::link;
  n.world_from_local = world_from_local;
  return n;
}

std::array<double, 3> transform_point(const transform& t,
                                      const std::array<double, 3>& p) {
  return {
      t.m[0][0] * p[0] + t.m[1][0] * p[1] + t.m[2][0] * p[2] + t.m[3][0],
      t.m[0][1] * p[0] + t.m[1][1] * p[1] + t.m[2][1] * p[2] + t.m[3][1],
      t.m[0][2] * p[0] + t.m[1][2] * p[1] + t.m[2][2] * p[2] + t.m[3][2],
  };
}

std::array<double, 3> rotate_vector(const transform& t,
                                    const std::array<double, 3>& v) {
  return {
      t.m[0][0] * v[0] + t.m[1][0] * v[1] + t.m[2][0] * v[2],
      t.m[0][1] * v[0] + t.m[1][1] * v[1] + t.m[2][1] * v[2],
      t.m[0][2] * v[0] + t.m[1][2] * v[1] + t.m[2][2] * v[2],
  };
}

void expect_vec_near(const std::array<double, 3>& a,
                     const std::array<double, 3>& b) {
  EXPECT_NEAR(a[0], b[0], tol);
  EXPECT_NEAR(a[1], b[1], tol);
  EXPECT_NEAR(a[2], b[2], tol);
}

TEST(Attachment, MatesOpposingPlaneNormalsAndCoincidentPoints) {
  const scene_node moving = node_with_transform(transform::identity());
  const attachment_plane moving_plane{
      .node_index = 0,
      .point_world = {0.0, 0.0, 0.25},
      .normal_world = {0.0, 0.0, 1.0},
      .tangent_world = {1.0, 0.0, 0.0},
  };
  const attachment_plane fixed_plane{
      .node_index = 1,
      .point_world = {1.0, 2.0, 3.0},
      .normal_world = {0.0, 0.0, 1.0},
      .tangent_world = {1.0, 0.0, 0.0},
  };

  const transform target = compute_attachment_target(
      moving, moving_plane, fixed_plane, attachment_options{});

  expect_vec_near(transform_point(target, {0.0, 0.0, 0.25}),
                  {1.0, 2.0, 3.0});
  expect_vec_near(rotate_vector(target, {0.0, 0.0, 1.0}),
                  {0.0, 0.0, -1.0});
}

TEST(Attachment, ExtractsCylinderCapPlaneInWorldSpace) {
  transform world_from_local = transform::identity();
  world_from_local.m[3][0] = 2.0;
  world_from_local.m[3][1] = 3.0;
  world_from_local.m[3][2] = 4.0;
  scene_node node = node_with_transform(world_from_local);
  node.shape = primitive{primitive_kind::cylinder, 0.75, 0.05, 0.0, 0.0, 0.0};

  const auto plane = attachment_plane_for_node(
      node, 7, attachment_face::cylinder_distal_cap);

  ASSERT_TRUE(plane.has_value());
  EXPECT_EQ(plane->node_index, 7U);
  expect_vec_near(plane->point_world, {2.0, 3.0, 4.75});
  expect_vec_near(plane->normal_world, {0.0, 0.0, 1.0});
  expect_vec_near(plane->tangent_world, {1.0, 0.0, 0.0});
}

TEST(Attachment, ExtractsBoxFacePlaneInWorldSpace) {
  transform world_from_local = transform::identity();
  world_from_local.m[3][0] = -1.0;
  scene_node node = node_with_transform(world_from_local);
  node.shape = primitive{primitive_kind::box, 0.0, 0.0, 0.25, 0.5, 0.75};

  const auto plane = attachment_plane_for_node(
      node, 3, attachment_face::box_negative_x);

  ASSERT_TRUE(plane.has_value());
  EXPECT_EQ(plane->node_index, 3U);
  expect_vec_near(plane->point_world, {-1.25, 0.0, 0.0});
  expect_vec_near(plane->normal_world, {-1.0, 0.0, 0.0});
  expect_vec_near(plane->tangent_world, {0.0, 1.0, 0.0});
}

TEST(Attachment, RejectsFaceThatDoesNotMatchPrimitiveKind) {
  scene_node node = node_with_transform(transform::identity());
  node.shape = primitive{primitive_kind::cylinder, 1.0, 0.05, 0.0, 0.0, 0.0};

  EXPECT_EQ(attachment_plane_for_node(node, 0, attachment_face::box_positive_x),
            std::nullopt);
}

TEST(Attachment, PicksCylinderCapSurfacePlaneFromRayHit) {
  scene_snapshot snapshot;
  scene_node node = node_with_transform(transform::identity());
  node.shape = primitive{primitive_kind::cylinder, 1.0, 0.1, 0.0, 0.0, 0.0};
  snapshot.nodes.push_back(node);

  const auto plane = pick_attachment_plane(
      snapshot, ray{{0.0, 0.0, 3.0}, {0.0, 0.0, -1.0}});

  ASSERT_TRUE(plane.has_value());
  EXPECT_EQ(plane->node_index, 0U);
  expect_vec_near(plane->point_world, {0.0, 0.0, 1.0});
  expect_vec_near(plane->normal_world, {0.0, 0.0, 1.0});
}

TEST(Attachment, PicksCylinderSideSurfacePlaneFromRayHit) {
  scene_snapshot snapshot;
  scene_node node = node_with_transform(transform::identity());
  node.shape = primitive{primitive_kind::cylinder, 1.0, 0.1, 0.0, 0.0, 0.0};
  snapshot.nodes.push_back(node);

  const auto plane = pick_attachment_plane(
      snapshot, ray{{1.0, 0.0, 0.5}, {-1.0, 0.0, 0.0}});

  ASSERT_TRUE(plane.has_value());
  EXPECT_EQ(plane->node_index, 0U);
  expect_vec_near(plane->point_world, {0.1, 0.0, 0.5});
  expect_vec_near(plane->normal_world, {1.0, 0.0, 0.0});
  expect_vec_near(plane->tangent_world, {0.0, 0.0, 1.0});
}

TEST(Attachment, PicksBoxSurfacePlaneFromRayHit) {
  scene_snapshot snapshot;
  scene_node node = node_with_transform(transform::identity());
  node.shape = primitive{primitive_kind::box, 0.0, 0.0, 0.25, 0.5, 0.75};
  snapshot.nodes.push_back(node);

  const auto plane = pick_attachment_plane(
      snapshot, ray{{0.0, 2.0, 0.0}, {0.0, -1.0, 0.0}});

  ASSERT_TRUE(plane.has_value());
  EXPECT_EQ(plane->node_index, 0U);
  expect_vec_near(plane->point_world, {0.0, 0.5, 0.0});
  expect_vec_near(plane->normal_world, {0.0, 1.0, 0.0});
}

TEST(Attachment, KrakenMeshSurfacePlaneMatchesRenderedYFlip) {
  scene_snapshot snapshot;
  scene_node node = node_with_transform(transform::identity());
  primitive p{};
  p.kind = primitive_kind::mesh;
  p.mesh_id = "kraken_x60";
  p.mesh_scale_m_per_unit = 1.0;
  p.mesh_min_local = {-1.0, -1.0, -2.0};
  p.mesh_max_local = {1.0, 1.0, 4.0};
  node.shape = p;
  snapshot.nodes.push_back(node);

  const auto plane = pick_attachment_plane(
      snapshot, ray{{0.0, 0.0, 10.0}, {0.0, 0.0, -1.0}});

  ASSERT_TRUE(plane.has_value());
  expect_vec_near(plane->point_world, {0.0, 0.0, 2.0});
  expect_vec_near(plane->normal_world, {0.0, 0.0, 1.0});
}

TEST(Attachment, FlipMakesNormalsPointSameDirection) {
  const scene_node moving = node_with_transform(transform::identity());
  const attachment_plane moving_plane{
      .node_index = 0,
      .point_world = {0.0, 0.0, 0.0},
      .normal_world = {0.0, 0.0, 1.0},
      .tangent_world = {1.0, 0.0, 0.0},
  };
  const attachment_plane fixed_plane{
      .node_index = 1,
      .point_world = {0.0, 0.0, 0.0},
      .normal_world = {0.0, 1.0, 0.0},
      .tangent_world = {1.0, 0.0, 0.0},
  };

  const transform target = compute_attachment_target(
      moving, moving_plane, fixed_plane, attachment_options{.flip = true});

  expect_vec_near(rotate_vector(target, {0.0, 0.0, 1.0}),
                  {0.0, 1.0, 0.0});
}

TEST(Attachment, OffsetSeparatesAlongFixedNormal) {
  const scene_node moving = node_with_transform(transform::identity());
  const attachment_plane moving_plane{
      .node_index = 0,
      .point_world = {0.0, 0.0, 0.0},
      .normal_world = {0.0, 0.0, 1.0},
      .tangent_world = {1.0, 0.0, 0.0},
  };
  const attachment_plane fixed_plane{
      .node_index = 1,
      .point_world = {2.0, 0.0, 0.0},
      .normal_world = {1.0, 0.0, 0.0},
      .tangent_world = {0.0, 1.0, 0.0},
  };

  const transform target = compute_attachment_target(
      moving, moving_plane, fixed_plane, attachment_options{.offset_m = 0.125});

  expect_vec_near(transform_point(target, {0.0, 0.0, 0.0}),
                  {2.125, 0.0, 0.0});
}

TEST(Attachment, QuarterTurnsRotateTangentAroundFixedNormal) {
  const scene_node moving = node_with_transform(transform::identity());
  const attachment_plane moving_plane{
      .node_index = 0,
      .point_world = {0.0, 0.0, 0.0},
      .normal_world = {0.0, 0.0, 1.0},
      .tangent_world = {1.0, 0.0, 0.0},
  };
  const attachment_plane fixed_plane{
      .node_index = 1,
      .point_world = {0.0, 0.0, 0.0},
      .normal_world = {0.0, 0.0, 1.0},
      .tangent_world = {1.0, 0.0, 0.0},
  };

  const transform target = compute_attachment_target(
      moving, moving_plane, fixed_plane,
      attachment_options{.quarter_turns = 1});

  expect_vec_near(rotate_vector(target, {1.0, 0.0, 0.0}),
                  {0.0, 1.0, 0.0});
  expect_vec_near(rotate_vector(target, {0.0, 0.0, 1.0}),
                  {0.0, 0.0, -1.0});
}

TEST(Attachment, PreservesMovingNodeLocalOffsetWhenComputingTarget) {
  transform initial = transform::identity();
  initial.m[3][0] = 10.0;
  initial.m[3][1] = 0.0;
  initial.m[3][2] = 0.0;
  const scene_node moving = node_with_transform(initial);
  const attachment_plane moving_plane{
      .node_index = 0,
      .point_world = {10.0, 0.0, 0.5},
      .normal_world = {0.0, 0.0, 1.0},
      .tangent_world = {1.0, 0.0, 0.0},
  };
  const attachment_plane fixed_plane{
      .node_index = 1,
      .point_world = {0.0, 0.0, 0.0},
      .normal_world = {0.0, 0.0, 1.0},
      .tangent_world = {1.0, 0.0, 0.0},
  };

  const transform target = compute_attachment_target(
      moving, moving_plane, fixed_plane, attachment_options{});

  expect_vec_near(transform_point(target, {0.0, 0.0, 0.5}),
                  {0.0, 0.0, 0.0});
}

}  // namespace
}  // namespace robosim::viz
