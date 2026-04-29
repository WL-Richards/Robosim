#include "viz/camera.h"

#include "viz/scene_snapshot.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <numbers>

namespace robosim::viz {
namespace {

constexpr double kMatrixTolerance = 1e-9;
constexpr double kTrigTolerance = 1e-6;

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

scene_node shaped_node(const char* name, primitive p, transform world_from_local) {
  scene_node node;
  node.entity_name = name;
  node.kind = node_kind::link;
  node.world_from_local = world_from_local;
  node.shape = p;
  return node;
}

std::array<double, 4> mat_vec(const std::array<std::array<double, 4>, 4>& m,
                              const std::array<double, 4>& v) {
  std::array<double, 4> result{};
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      result[row] += m[col][row] * v[col];
    }
  }
  return result;
}

std::array<std::array<double, 4>, 4> matmul(const std::array<std::array<double, 4>, 4>& a,
                                            const std::array<std::array<double, 4>, 4>& b) {
  std::array<std::array<double, 4>, 4> result{};
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      for (int k = 0; k < 4; ++k) {
        result[col][row] += a[k][row] * b[col][k];
      }
    }
  }
  return result;
}

TEST(OrbitCamera, EyeWorldUsesPinnedYawAndPitchConventions) {
  orbit_camera cam;
  cam.distance_m = 5.0;

  EXPECT_NEAR(cam.eye_world()[0], 0.0, kMatrixTolerance);
  EXPECT_NEAR(cam.eye_world()[1], 0.0, kMatrixTolerance);
  EXPECT_NEAR(cam.eye_world()[2], 5.0, kMatrixTolerance);

  cam.yaw_rad = std::numbers::pi / 2.0;
  EXPECT_NEAR(cam.eye_world()[0], 5.0, kTrigTolerance);
  EXPECT_NEAR(cam.eye_world()[1], 0.0, kTrigTolerance);
  EXPECT_NEAR(cam.eye_world()[2], 0.0, kTrigTolerance);

  cam.yaw_rad = 0.0;
  cam.pitch_rad = std::numbers::pi / 4.0;
  EXPECT_NEAR(cam.eye_world()[0], 0.0, kTrigTolerance);
  EXPECT_NEAR(cam.eye_world()[1], 5.0 * std::sin(std::numbers::pi / 4.0), kTrigTolerance);
  EXPECT_NEAR(cam.eye_world()[2], 5.0 * std::cos(std::numbers::pi / 4.0), kTrigTolerance);
}

TEST(OrbitCamera, PitchClampsAtBothPolesInDerivedOutputs) {
  orbit_camera cam;
  cam.distance_m = 5.0;

  cam.pitch_rad = 1e9;
  const auto positive = cam.eye_world();
  const double positive_pitch = std::numbers::pi / 2.0 - 1e-3;
  EXPECT_NEAR(positive[0], 0.0, kTrigTolerance);
  EXPECT_NEAR(positive[1], 5.0 * std::sin(positive_pitch), kTrigTolerance);
  EXPECT_NEAR(positive[2], 5.0 * std::cos(positive_pitch), kTrigTolerance);

  cam.pitch_rad = -1e9;
  const auto negative = cam.eye_world();
  const double negative_pitch = -std::numbers::pi / 2.0 + 1e-3;
  EXPECT_NEAR(negative[0], 0.0, kTrigTolerance);
  EXPECT_NEAR(negative[1], 5.0 * std::sin(negative_pitch), kTrigTolerance);
  EXPECT_NEAR(negative[2], 5.0 * std::cos(negative_pitch), kTrigTolerance);
}

TEST(OrbitCamera, ViewMatrixMatchesLookAtRhAtDefaultPose) {
  const orbit_camera cam;
  const auto view = cam.view_matrix();

  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      double expected = col == row ? 1.0 : 0.0;
      if (col == 3 && row == 2)
        expected = -5.0;
      EXPECT_NEAR(view[col][row], expected, kMatrixTolerance);
    }
  }
}

TEST(OrbitCamera, ProjectionMatrixMatchesStandardOpenGlPerspective) {
  orbit_camera cam;

  const auto p = cam.projection_matrix();
  const double tan_half = std::tan(cam.fov_y_rad / 2.0);

  EXPECT_NEAR(p[0][0], 1.0 / (cam.aspect * tan_half), kMatrixTolerance);
  EXPECT_NEAR(p[1][1], 1.0 / tan_half, kMatrixTolerance);
  EXPECT_NEAR(p[2][2],
              -(cam.far_plane_m + cam.near_plane_m) / (cam.far_plane_m - cam.near_plane_m),
              kMatrixTolerance);
  EXPECT_NEAR(p[3][2],
              -(2.0 * cam.far_plane_m * cam.near_plane_m) / (cam.far_plane_m - cam.near_plane_m),
              kMatrixTolerance);
  EXPECT_NEAR(p[2][3], -1.0, kMatrixTolerance);
  EXPECT_NEAR(p[0][1], 0.0, kMatrixTolerance);
  EXPECT_NEAR(p[1][0], 0.0, kMatrixTolerance);
}

TEST(OrbitCamera, PanMovesPivotInCameraLocalAxes) {
  {
    orbit_camera cam;
    pan(cam, 1.0, 0.0);
    EXPECT_EQ(cam.pivot_world, (std::array{1.0, 0.0, 0.0}));
    EXPECT_DOUBLE_EQ(cam.distance_m, 5.0);
  }

  orbit_camera cam;
  cam.yaw_rad = std::numbers::pi / 2.0;

  pan(cam, 1.0, 0.0);

  EXPECT_NEAR(cam.pivot_world[0], 0.0, kTrigTolerance);
  EXPECT_NEAR(cam.pivot_world[1], 0.0, kTrigTolerance);
  EXPECT_NEAR(cam.pivot_world[2], -1.0, kTrigTolerance);

  orbit_camera vertical;
  pan(vertical, 0.0, 1.0);
  EXPECT_EQ(vertical.pivot_world, (std::array{0.0, 1.0, 0.0}));
}

TEST(OrbitCamera, ZoomScalesDistanceAndClampsPositive) {
  orbit_camera cam;

  zoom(cam, 1.0);
  EXPECT_NEAR(cam.distance_m, 4.5, kMatrixTolerance);

  cam.distance_m = 5.0;
  zoom(cam, -1.0);
  EXPECT_NEAR(cam.distance_m, 5.0 / 0.9, kMatrixTolerance);

  zoom(cam, 1000.0);
  EXPECT_DOUBLE_EQ(cam.distance_m, 1e-3);
}

TEST(Aabb, EmptySnapshotProducesDegenerateOriginBox) {
  const scene_snapshot snapshot;

  EXPECT_EQ(compute_world_aabb(snapshot), (aabb{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}));
}

TEST(Aabb, ComputesBoundsForSingleCylinderAtOrigin) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(shaped_node("cylinder",
                                       primitive{primitive_kind::cylinder, 1.0, 0.1, 0.0, 0.0, 0.0},
                                       transform::identity()));

  // Cylinder anchored at proximal cap z = 0 (TEST_PLAN convention #8).
  const aabb box = compute_world_aabb(snapshot);

  EXPECT_EQ(box.min_world, (std::array{-0.1, -0.1, 0.0}));
  EXPECT_EQ(box.max_world, (std::array{0.1, 0.1, 1.0}));
}

TEST(Aabb, CombinesMultipleTranslatedCylinders) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(shaped_node(
      "c0", primitive{primitive_kind::cylinder, 1.0, 0.1, 0.0, 0.0, 0.0}, transform::identity()));
  snapshot.nodes.push_back(shaped_node("c1",
                                       primitive{primitive_kind::cylinder, 1.0, 0.1, 0.0, 0.0, 0.0},
                                       translated(2.0, 0.0, 0.0)));

  const aabb box = compute_world_aabb(snapshot);

  EXPECT_EQ(box.min_world, (std::array{-0.1, -0.1, 0.0}));
  EXPECT_EQ(box.max_world, (std::array{2.1, 0.1, 1.0}));
}

TEST(Aabb, RespectsWorldFromLocalTranslation) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(shaped_node("cylinder",
                                       primitive{primitive_kind::cylinder, 1.0, 0.1, 0.0, 0.0, 0.0},
                                       translated(5.0, 0.0, 0.0)));

  const aabb box = compute_world_aabb(snapshot);

  EXPECT_EQ(box.min_world, (std::array{4.9, -0.1, 0.0}));
  EXPECT_EQ(box.max_world, (std::array{5.1, 0.1, 1.0}));
}

TEST(Aabb, HandlesRotatedCylinderViaObbEnvelope) {
  scene_snapshot snapshot;
  snapshot.nodes.push_back(shaped_node("cylinder",
                                       primitive{primitive_kind::cylinder, 1.0, 0.1, 0.0, 0.0, 0.0},
                                       rotate_x(std::numbers::pi / 2.0)));

  // Local +Z → world -Y under rotate_x(+π/2); cylinder local span
  // z ∈ [0, 1] (proximal-anchored) becomes world y ∈ [-1, 0].
  const aabb box = compute_world_aabb(snapshot);

  EXPECT_NEAR(box.min_world[0], -0.1, kMatrixTolerance);
  EXPECT_NEAR(box.min_world[1], -1.0, kMatrixTolerance);
  EXPECT_NEAR(box.min_world[2], -0.1, kMatrixTolerance);
  EXPECT_NEAR(box.max_world[0], 0.1, kMatrixTolerance);
  EXPECT_NEAR(box.max_world[1], 0.0, kMatrixTolerance);
  EXPECT_NEAR(box.max_world[2], 0.1, kMatrixTolerance);
}

TEST(Aabb, ComputesWorldBoundsAcrossPrimitiveKinds) {
  scene_snapshot snapshot;
  snapshot.nodes = {
      // Cylinder length 2.0 anchored at z = 0 → local z ∈ [0, 2];
      // translated +X by 1: world x ∈ [0.5, 1.5], z ∈ [0, 2].
      shaped_node("cylinder",
                  primitive{primitive_kind::cylinder, 2.0, 0.5, 0.0, 0.0, 0.0},
                  translated(1.0, 0.0, 0.0)),
      // Box centered at translated (-2, 1, 0) with half-extents
      // (0.25, 0.5, 0.75) → world x ∈ [-2.25, -1.75], y ∈ [0.5, 1.5],
      // z ∈ [-0.75, 0.75].
      shaped_node("box",
                  primitive{primitive_kind::box, 0.0, 0.0, 0.25, 0.5, 0.75},
                  translated(-2.0, 1.0, 0.0)),
      // Sphere at (0, -3, 4) radius 1 → world y ∈ [-4, -2],
      // z ∈ [3, 5].
      shaped_node("sphere",
                  primitive{primitive_kind::sphere, 0.0, 1.0, 0.0, 0.0, 0.0},
                  translated(0.0, -3.0, 4.0)),
  };

  const aabb box = compute_world_aabb(snapshot);

  EXPECT_NEAR(box.min_world[0], -2.25, kMatrixTolerance);
  EXPECT_NEAR(box.min_world[1], -4.0, kMatrixTolerance);
  EXPECT_NEAR(box.min_world[2], -0.75, kMatrixTolerance);
  EXPECT_NEAR(box.max_world[0], 1.5, kMatrixTolerance);
  EXPECT_NEAR(box.max_world[1], 1.5, kMatrixTolerance);
  EXPECT_NEAR(box.max_world[2], 5.0, kMatrixTolerance);
}

TEST(OrbitCamera, FrameFitCentersPivotAndSatisfiesFovDistance) {
  orbit_camera cam;
  const aabb box{{-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}};

  frame_fit(cam, box);

  EXPECT_NEAR(cam.pivot_world[0], 0.0, kMatrixTolerance);
  EXPECT_NEAR(cam.pivot_world[1], 0.0, kMatrixTolerance);
  EXPECT_NEAR(cam.pivot_world[2], 0.0, kMatrixTolerance);
  EXPECT_NEAR(cam.distance_m, std::sqrt(3.0) / std::sin(0.4 * cam.fov_y_rad), kMatrixTolerance);
}

TEST(OrbitCamera, FrameFitCentersOffsetAabbAndIsIdempotent) {
  orbit_camera cam;
  const aabb box{{0.5, 1.5, 2.5}, {1.5, 2.5, 3.5}};

  frame_fit(cam, box);
  const orbit_camera first = cam;
  frame_fit(cam, box);

  EXPECT_EQ(cam.pivot_world, (std::array{1.0, 2.0, 3.0}));
  EXPECT_NEAR(cam.distance_m, first.distance_m, kMatrixTolerance);
  EXPECT_NEAR(cam.yaw_rad, first.yaw_rad, kMatrixTolerance);
  EXPECT_NEAR(cam.pitch_rad, first.pitch_rad, kMatrixTolerance);
}

TEST(OrbitCamera, FrameFitHandlesDegenerateBox) {
  orbit_camera cam;
  const aabb box{{2.0, 3.0, 4.0}, {2.0, 3.0, 4.0}};

  frame_fit(cam, box);

  EXPECT_EQ(cam.pivot_world, (std::array{2.0, 3.0, 4.0}));
  EXPECT_DOUBLE_EQ(cam.distance_m, 1.0);
}

TEST(OrbitCamera, FrameFitOnEmptySnapshotAabbUsesDefaultDistance) {
  const scene_snapshot snapshot;
  const aabb box = compute_world_aabb(snapshot);
  orbit_camera cam;

  frame_fit(cam, box);

  EXPECT_EQ(cam.pivot_world, (std::array{0.0, 0.0, 0.0}));
  EXPECT_DOUBLE_EQ(cam.distance_m, 1.0);
}

TEST(OrbitCamera, FrameFitAabbCornersStayInClipSpace) {
  orbit_camera cam;
  const aabb box{{-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}};
  frame_fit(cam, box);
  const auto vp = matmul(cam.projection_matrix(), cam.view_matrix());

  for (double x : {-1.0, 1.0}) {
    for (double y : {-1.0, 1.0}) {
      for (double z : {-1.0, 1.0}) {
        const auto clip = mat_vec(vp, {x, y, z, 1.0});
        const std::array ndc = {clip[0] / clip[3], clip[1] / clip[3], clip[2] / clip[3]};
        EXPECT_GE(ndc[0], -1.0 - 1e-6);
        EXPECT_LE(ndc[0], 1.0 + 1e-6);
        EXPECT_GE(ndc[1], -1.0 - 1e-6);
        EXPECT_LE(ndc[1], 1.0 + 1e-6);
        EXPECT_GE(ndc[2], -1.0 - 1e-6);
        EXPECT_LE(ndc[2], 1.0 + 1e-6);
      }
    }
  }
}

}  // namespace
}  // namespace robosim::viz
