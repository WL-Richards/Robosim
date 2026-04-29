#pragma once

// Orbit camera + AABB + frame-fit. Pure math; no GL state.
//
// Conventions pinned in tests/viz/TEST_PLAN.md "Conventions": right-
// handed, column-major, m[col][row] indexing, GLM lookAtRH for view,
// standard right-handed perspective for projection.

#include "scene_snapshot.h"

#include <array>

namespace robosim::viz {

struct orbit_camera {
  std::array<double, 3> pivot_world{};
  double distance_m = 5.0;
  double yaw_rad = 0.0;
  double pitch_rad = 0.0;
  double fov_y_rad = 0.785398;  // 45 deg
  double aspect = 16.0 / 9.0;
  double near_plane_m = 0.05;
  double far_plane_m = 100.0;

  [[nodiscard]] std::array<double, 3> eye_world() const;
  [[nodiscard]] std::array<std::array<double, 4>, 4> view_matrix() const;
  [[nodiscard]] std::array<std::array<double, 4>, 4> projection_matrix() const;
};

void pan(orbit_camera& cam, double right_offset_m, double up_offset_m);
void zoom(orbit_camera& cam, double scroll_units);

struct aabb {
  std::array<double, 3> min_world{};
  std::array<double, 3> max_world{};

  bool operator==(const aabb&) const = default;
};

[[nodiscard]] aabb compute_world_aabb(const scene_snapshot& s);

void frame_fit(orbit_camera& cam, const aabb& world_box);

}  // namespace robosim::viz
