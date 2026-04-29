#include "camera.h"

#include "scene_snapshot.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace robosim::viz {

namespace {

constexpr double pitch_clamp_epsilon_rad = 1e-3;
constexpr double frame_fit_fov_fraction = 0.40;  // half of 80% angular extent
constexpr double frame_fit_degenerate_extent_m = 1e-9;
constexpr double zoom_factor_per_unit = 0.9;
constexpr double zoom_min_distance_m = 1e-3;

[[nodiscard]] double clamp_pitch(double p) {
  const double limit = (std::numbers::pi / 2.0) - pitch_clamp_epsilon_rad;
  if (p > limit) return limit;
  if (p < -limit) return -limit;
  return p;
}

// Camera-local +Z (forward, from pivot toward eye) for the given orbit
// parameters. yaw is right-handed about world +Y; pitch is right-handed
// about the camera's local +X after yaw.
[[nodiscard]] std::array<double, 3> forward_from_pivot(
    double yaw, double pitch_clamped) {
  const double cp = std::cos(pitch_clamped);
  const double sp = std::sin(pitch_clamped);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  return {cp * sy, sp, cp * cy};
}

// Apply a 4x4 column-major rigid transform to a point.
[[nodiscard]] std::array<double, 3> apply_to_point(
    const transform& t, const std::array<double, 3>& p) {
  std::array<double, 3> r{};
  for (int row = 0; row < 3; ++row) {
    double s = t.m[3][row];
    for (int col = 0; col < 3; ++col) {
      s += t.m[col][row] * p[col];
    }
    r[row] = s;
  }
  return r;
}

// 8 local-frame AABB corners for a primitive (used by compute_world_aabb).
struct corner_box {
  std::array<double, 3> lo;
  std::array<double, 3> hi;
};

[[nodiscard]] corner_box local_box_for(const primitive& p) {
  switch (p.kind) {
    case primitive_kind::cylinder:
    case primitive_kind::arrow:
      // Cylinder/arrow: local origin at the proximal cap (z = 0),
      // extending to z = length_m along +Z. See TEST_PLAN
      // conventions #8 / #9.
      return {{-p.radius_m, -p.radius_m, 0.0},
              {p.radius_m, p.radius_m, p.length_m}};
    case primitive_kind::sphere:
      return {{-p.radius_m, -p.radius_m, -p.radius_m},
              {p.radius_m, p.radius_m, p.radius_m}};
    case primitive_kind::box:
      return {{-p.half_extent_x_m, -p.half_extent_y_m, -p.half_extent_z_m},
              {p.half_extent_x_m, p.half_extent_y_m, p.half_extent_z_m}};
  }
  return {};
}

}  // namespace

std::array<double, 3> orbit_camera::eye_world() const {
  const auto fwd = forward_from_pivot(yaw_rad, clamp_pitch(pitch_rad));
  return {pivot_world[0] + distance_m * fwd[0],
          pivot_world[1] + distance_m * fwd[1],
          pivot_world[2] + distance_m * fwd[2]};
}

std::array<std::array<double, 4>, 4> orbit_camera::view_matrix() const {
  // GLM lookAtRH(eye, target, up): builds an orthonormal view basis
  // looking from eye toward target. Output is column-major m[col][row].
  const auto eye = eye_world();
  const std::array<double, 3> target = pivot_world;
  const std::array<double, 3> up = {0.0, 1.0, 0.0};

  std::array<double, 3> f = {target[0] - eye[0], target[1] - eye[1],
                             target[2] - eye[2]};
  const double f_len = std::sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
  for (auto& c : f) c /= f_len;

  // s = normalize(cross(f, up))
  std::array<double, 3> s = {f[1] * up[2] - f[2] * up[1],
                             f[2] * up[0] - f[0] * up[2],
                             f[0] * up[1] - f[1] * up[0]};
  const double s_len = std::sqrt(s[0] * s[0] + s[1] * s[1] + s[2] * s[2]);
  for (auto& c : s) c /= s_len;

  // u = cross(s, f)
  const std::array<double, 3> u = {s[1] * f[2] - s[2] * f[1],
                                   s[2] * f[0] - s[0] * f[2],
                                   s[0] * f[1] - s[1] * f[0]};

  std::array<std::array<double, 4>, 4> m{};
  // Column 0: s, with translation entry -dot(s, eye).
  m[0][0] = s[0];
  m[1][0] = s[1];
  m[2][0] = s[2];
  m[3][0] = -(s[0] * eye[0] + s[1] * eye[1] + s[2] * eye[2]);
  // Column 1: u, with translation entry -dot(u, eye).
  m[0][1] = u[0];
  m[1][1] = u[1];
  m[2][1] = u[2];
  m[3][1] = -(u[0] * eye[0] + u[1] * eye[1] + u[2] * eye[2]);
  // Column 2: -f, with translation entry +dot(f, eye).
  m[0][2] = -f[0];
  m[1][2] = -f[1];
  m[2][2] = -f[2];
  m[3][2] = f[0] * eye[0] + f[1] * eye[1] + f[2] * eye[2];
  // Row 3.
  m[0][3] = 0.0;
  m[1][3] = 0.0;
  m[2][3] = 0.0;
  m[3][3] = 1.0;
  return m;
}

std::array<std::array<double, 4>, 4> orbit_camera::projection_matrix() const {
  std::array<std::array<double, 4>, 4> m{};
  const double tan_half = std::tan(0.5 * fov_y_rad);
  m[0][0] = 1.0 / (aspect * tan_half);
  m[1][1] = 1.0 / tan_half;
  m[2][2] = -(far_plane_m + near_plane_m) / (far_plane_m - near_plane_m);
  m[3][2] = -(2.0 * far_plane_m * near_plane_m) /
            (far_plane_m - near_plane_m);
  m[2][3] = -1.0;
  return m;
}

void pan(orbit_camera& cam,
         double right_offset_m, double up_offset_m) {
  // Camera-local screen-right and up in world space are the inverse-view
  // basis vectors, i.e. the view matrix's first/second rows.
  const auto V = cam.view_matrix();
  const std::array<double, 3> right_world = {V[0][0], V[1][0], V[2][0]};
  const std::array<double, 3> up_world = {V[0][1], V[1][1], V[2][1]};
  for (int i = 0; i < 3; ++i) {
    cam.pivot_world[i] += right_offset_m * right_world[i] +
                          up_offset_m * up_world[i];
  }
}

void zoom(orbit_camera& cam, double scroll_units) {
  cam.distance_m *= std::pow(zoom_factor_per_unit, scroll_units);
  if (cam.distance_m < zoom_min_distance_m) {
    cam.distance_m = zoom_min_distance_m;
  }
}

aabb compute_world_aabb(const scene_snapshot& s) {
  bool any = false;
  std::array<double, 3> mn{};
  std::array<double, 3> mx{};
  for (const auto& node : s.nodes) {
    if (!node.shape) continue;
    const auto box = local_box_for(*node.shape);
    // 8 corners of the local AABB.
    for (int dx = 0; dx < 2; ++dx) {
      for (int dy = 0; dy < 2; ++dy) {
        for (int dz = 0; dz < 2; ++dz) {
          const std::array<double, 3> local_corner = {
              dx == 0 ? box.lo[0] : box.hi[0],
              dy == 0 ? box.lo[1] : box.hi[1],
              dz == 0 ? box.lo[2] : box.hi[2],
          };
          const auto world_corner =
              apply_to_point(node.world_from_local, local_corner);
          if (!any) {
            mn = world_corner;
            mx = world_corner;
            any = true;
          } else {
            for (int i = 0; i < 3; ++i) {
              if (world_corner[i] < mn[i]) mn[i] = world_corner[i];
              if (world_corner[i] > mx[i]) mx[i] = world_corner[i];
            }
          }
        }
      }
    }
  }
  if (!any) return aabb{};  // {0,0,0}-{0,0,0} sentinel; see TEST_PLAN A5.
  return aabb{mn, mx};
}

void frame_fit(orbit_camera& cam, const aabb& world_box) {
  // Degenerate AABB: extent below 1e-9 along every axis. Falls back to
  // pivot = min_world, distance = 1.0. See TEST_PLAN.md X1.
  bool degenerate = true;
  for (int i = 0; i < 3; ++i) {
    if (world_box.max_world[i] - world_box.min_world[i] >=
        frame_fit_degenerate_extent_m) {
      degenerate = false;
      break;
    }
  }
  if (degenerate) {
    cam.pivot_world = world_box.min_world;
    cam.distance_m = 1.0;
    return;
  }

  // Pivot on AABB centroid.
  for (int i = 0; i < 3; ++i) {
    cam.pivot_world[i] =
        0.5 * (world_box.min_world[i] + world_box.max_world[i]);
  }

  // 80%-angular framing: bounding-sphere radius / sin(0.40 * fov_y).
  // Bounding sphere radius is half the AABB diagonal.
  double extent_sq = 0.0;
  for (int i = 0; i < 3; ++i) {
    const double e = world_box.max_world[i] - world_box.min_world[i];
    extent_sq += e * e;
  }
  const double half_diag = 0.5 * std::sqrt(extent_sq);
  cam.distance_m =
      half_diag / std::sin(frame_fit_fov_fraction * cam.fov_y_rad);
}

}  // namespace robosim::viz
