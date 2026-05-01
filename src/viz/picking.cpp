#include "picking.h"

#include "scene_snapshot.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>

namespace robosim::viz {

namespace {

// Apply the inverse of a rigid 4x4 transform (R | T; 0 | 1) to a point.
// Local point = R^T * (world_point - T).
[[nodiscard]] std::array<double, 3> inverse_apply_point(
    const transform& t, const std::array<double, 3>& p) {
  const std::array<double, 3> q = {p[0] - t.m[3][0], p[1] - t.m[3][1],
                                   p[2] - t.m[3][2]};
  std::array<double, 3> r{};
  for (int row = 0; row < 3; ++row) {
    double s = 0.0;
    for (int col = 0; col < 3; ++col) {
      // R^T[row, col] = R[col, row] = m[row][col].
      s += t.m[row][col] * q[col];
    }
    r[row] = s;
  }
  return r;
}

[[nodiscard]] std::array<double, 3> inverse_apply_direction(
    const transform& t, const std::array<double, 3>& d) {
  std::array<double, 3> r{};
  for (int row = 0; row < 3; ++row) {
    double s = 0.0;
    for (int col = 0; col < 3; ++col) {
      s += t.m[row][col] * d[col];
    }
    r[row] = s;
  }
  return r;
}

// Closed-shape ray-vs-cylinder along local +Z. Cylinder local origin
// is at the proximal cap (z = 0) and extends to z = length per
// TEST_PLAN convention #8 / #11. Returns smallest t >= 0 on a hit.
[[nodiscard]] std::optional<double> ray_vs_cylinder_local(
    const std::array<double, 3>& o, const std::array<double, 3>& d,
    double length, double radius) {
  // Inside test (closed shape): origin strictly inside or on the
  // boundary returns t == 0. See TEST_PLAN P7.
  const double radial_sq = o[0] * o[0] + o[1] * o[1];
  const double r_sq = radius * radius;
  if (radial_sq <= r_sq && o[2] >= 0.0 && o[2] <= length) {
    return 0.0;
  }

  double best = std::numeric_limits<double>::infinity();
  bool hit = false;

  // Side surface: A t^2 + B t + C = 0 with A = dx^2 + dy^2, etc.
  const double a = d[0] * d[0] + d[1] * d[1];
  if (a > 0.0) {
    const double b = 2.0 * (o[0] * d[0] + o[1] * d[1]);
    const double c = radial_sq - r_sq;
    const double disc = b * b - 4.0 * a * c;
    if (disc >= 0.0) {
      const double s = std::sqrt(disc);
      const std::array<double, 2> roots = {(-b - s) / (2.0 * a),
                                           (-b + s) / (2.0 * a)};
      for (double t : roots) {
        if (t >= 0.0 && t < best) {
          const double z_at = o[2] + t * d[2];
          if (z_at >= 0.0 && z_at <= length) {
            best = t;
            hit = true;
          }
        }
      }
    }
  }

  // Caps at z = 0 (proximal) and z = length (distal).
  if (d[2] != 0.0) {
    for (double z_cap : {0.0, length}) {
      const double t = (z_cap - o[2]) / d[2];
      if (t >= 0.0 && t < best) {
        const double x = o[0] + t * d[0];
        const double y = o[1] + t * d[1];
        if (x * x + y * y <= r_sq) {
          best = t;
          hit = true;
        }
      }
    }
  }

  if (hit) return best;
  return std::nullopt;
}

// Slab-method ray-vs-axis-aligned-box centered at origin.
[[nodiscard]] std::optional<double> ray_vs_box_local(
    const std::array<double, 3>& o, const std::array<double, 3>& d,
    double hx, double hy, double hz) {
  if (std::abs(o[0]) <= hx && std::abs(o[1]) <= hy &&
      std::abs(o[2]) <= hz) {
    return 0.0;
  }

  double t_near = 0.0;
  double t_far = std::numeric_limits<double>::infinity();
  const std::array<double, 3> half = {hx, hy, hz};
  for (int i = 0; i < 3; ++i) {
    if (std::abs(d[i]) < 1e-30) {
      if (std::abs(o[i]) > half[i]) return std::nullopt;
      continue;
    }
    double t1 = (-half[i] - o[i]) / d[i];
    double t2 = (half[i] - o[i]) / d[i];
    if (t1 > t2) std::swap(t1, t2);
    if (t1 > t_near) t_near = t1;
    if (t2 < t_far) t_far = t2;
    if (t_near > t_far) return std::nullopt;
  }
  return t_near;
}

[[nodiscard]] std::optional<double> ray_vs_sphere_local(
    const std::array<double, 3>& o, const std::array<double, 3>& d,
    double radius) {
  const double r_sq = radius * radius;
  const double dist_sq = o[0] * o[0] + o[1] * o[1] + o[2] * o[2];
  if (dist_sq <= r_sq) return 0.0;

  const double a = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
  const double b = 2.0 * (o[0] * d[0] + o[1] * d[1] + o[2] * d[2]);
  const double c = dist_sq - r_sq;
  const double disc = b * b - 4.0 * a * c;
  if (disc < 0.0) return std::nullopt;
  const double s = std::sqrt(disc);
  const double t_minus = (-b - s) / (2.0 * a);
  if (t_minus >= 0.0) return t_minus;
  const double t_plus = (-b + s) / (2.0 * a);
  if (t_plus >= 0.0) return t_plus;
  return std::nullopt;
}

[[nodiscard]] std::optional<double> ray_vs_primitive_local(
    const primitive& p, const std::array<double, 3>& o,
    const std::array<double, 3>& d) {
  switch (p.kind) {
    case primitive_kind::cylinder:
    case primitive_kind::arrow:
      return ray_vs_cylinder_local(o, d, p.length_m, p.radius_m);
    case primitive_kind::box:
      return ray_vs_box_local(o, d, p.half_extent_x_m, p.half_extent_y_m,
                              p.half_extent_z_m);
    case primitive_kind::sphere:
      return ray_vs_sphere_local(o, d, p.radius_m);
    case primitive_kind::rotation_arrow:
      return ray_vs_box_local(o, d, p.radius_m, p.radius_m, 0.01);
    case primitive_kind::mesh: {
      const double scale = p.mesh_scale_m_per_unit;
      const double hx = 0.5 * (p.mesh_max_local[0] - p.mesh_min_local[0]) * scale;
      const double hy = 0.5 * (p.mesh_max_local[1] - p.mesh_min_local[1]) * scale;
      const double hz = 0.5 * (p.mesh_max_local[2] - p.mesh_min_local[2]) * scale;
      const double x_sign = p.mesh_id == "kraken_x60" ? -1.0 : 1.0;
      const double z_sign = p.mesh_id == "kraken_x60" ? -1.0 : 1.0;
      const std::array<double, 3> center = {
          x_sign * 0.5 * (p.mesh_min_local[0] + p.mesh_max_local[0]) * scale,
          0.5 * (p.mesh_min_local[1] + p.mesh_max_local[1]) * scale,
          z_sign * 0.5 * (p.mesh_min_local[2] + p.mesh_max_local[2]) * scale,
      };
      return ray_vs_box_local({o[0] - center[0], o[1] - center[1], o[2] - center[2]},
                              d, hx, hy, hz);
    }
  }
  return std::nullopt;
}

}  // namespace

std::optional<pick_hit> pick(const scene_snapshot& s, const ray& r) {
  std::optional<pick_hit> best;
  for (std::size_t i = 0; i < s.nodes.size(); ++i) {
    const auto& node = s.nodes[i];
    if (!node.shape) continue;
    const auto local_origin =
        inverse_apply_point(node.world_from_local, r.origin_world);
    const auto local_direction =
        inverse_apply_direction(node.world_from_local, r.direction_world);
    const auto t = ray_vs_primitive_local(*node.shape, local_origin,
                                          local_direction);
    if (!t.has_value()) continue;
    if (!best.has_value() || *t < best->t_along_ray) {
      best = pick_hit{i, *t};
    }
  }
  return best;
}

}  // namespace robosim::viz
