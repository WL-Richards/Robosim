#include "attachment.h"

#include "picking.h"
#include "scene_snapshot.h"

#include <array>
#include <cmath>
#include <optional>

namespace robosim::viz {

namespace {

using vec3 = std::array<double, 3>;

vec3 add(const vec3& a, const vec3& b) {
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

vec3 sub(const vec3& a, const vec3& b) {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

vec3 scale(const vec3& v, double s) {
  return {v[0] * s, v[1] * s, v[2] * s};
}

double dot(const vec3& a, const vec3& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

vec3 cross(const vec3& a, const vec3& b) {
  return {a[1] * b[2] - a[2] * b[1],
          a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}

double norm(const vec3& v) {
  return std::sqrt(dot(v, v));
}

vec3 normalize(const vec3& v) {
  const double n = norm(v);
  return scale(v, 1.0 / n);
}

vec3 project_off_axis(const vec3& v, const vec3& axis) {
  return sub(v, scale(axis, dot(v, axis)));
}

vec3 fallback_tangent_for(const vec3& normal) {
  const vec3 x_axis{1.0, 0.0, 0.0};
  const vec3 y_axis{0.0, 1.0, 0.0};
  const vec3 projected_x = project_off_axis(x_axis, normal);
  if (norm(projected_x) > 1e-12) {
    return normalize(projected_x);
  }
  return normalize(project_off_axis(y_axis, normal));
}

vec3 normalized_tangent(const vec3& tangent, const vec3& normal) {
  const vec3 projected = project_off_axis(tangent, normal);
  if (norm(projected) > 1e-12) {
    return normalize(projected);
  }
  return fallback_tangent_for(normal);
}

vec3 rotate_quarter_turns_around_axis(vec3 v, const vec3& axis,
                                      int quarter_turns) {
  int turns = quarter_turns % 4;
  if (turns < 0) {
    turns += 4;
  }

  for (int i = 0; i < turns; ++i) {
    v = cross(axis, v);
  }
  return v;
}

struct basis {
  vec3 x;
  vec3 y;
  vec3 z;
};

basis make_basis(const vec3& normal, const vec3& tangent) {
  basis b;
  b.z = normalize(normal);
  b.x = normalized_tangent(tangent, b.z);
  b.y = cross(b.z, b.x);
  return b;
}

double rotation_entry(const basis& to, const basis& from, int col, int row) {
  const vec3 to_axes[3] = {to.x, to.y, to.z};
  const vec3 from_axes[3] = {from.x, from.y, from.z};
  double entry = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    entry += to_axes[axis][row] * from_axes[axis][col];
  }
  return entry;
}

vec3 transform_point(const transform& t, const vec3& p) {
  return {
      t.m[0][0] * p[0] + t.m[1][0] * p[1] + t.m[2][0] * p[2] + t.m[3][0],
      t.m[0][1] * p[0] + t.m[1][1] * p[1] + t.m[2][1] * p[2] + t.m[3][1],
      t.m[0][2] * p[0] + t.m[1][2] * p[1] + t.m[2][2] * p[2] + t.m[3][2],
  };
}

vec3 rotate_vector(const transform& t, const vec3& v) {
  return {
      t.m[0][0] * v[0] + t.m[1][0] * v[1] + t.m[2][0] * v[2],
      t.m[0][1] * v[0] + t.m[1][1] * v[1] + t.m[2][1] * v[2],
      t.m[0][2] * v[0] + t.m[1][2] * v[1] + t.m[2][2] * v[2],
  };
}

vec3 primitive_box_half_extents(const primitive& p) {
  if (p.kind == primitive_kind::mesh) {
    const double scale = p.mesh_scale_m_per_unit;
    return {
        0.5 * (p.mesh_max_local[0] - p.mesh_min_local[0]) * scale,
        0.5 * (p.mesh_max_local[1] - p.mesh_min_local[1]) * scale,
        0.5 * (p.mesh_max_local[2] - p.mesh_min_local[2]) * scale,
    };
  }
  return {p.half_extent_x_m, p.half_extent_y_m, p.half_extent_z_m};
}

vec3 primitive_box_center(const primitive& p) {
  if (p.kind == primitive_kind::mesh) {
    const double scale = p.mesh_scale_m_per_unit;
    const double x_sign = p.mesh_id == "kraken_x60" ? -1.0 : 1.0;
    const double z_sign = p.mesh_id == "kraken_x60" ? -1.0 : 1.0;
    return {
        x_sign * 0.5 * (p.mesh_max_local[0] + p.mesh_min_local[0]) * scale,
        0.5 * (p.mesh_max_local[1] + p.mesh_min_local[1]) * scale,
        z_sign * 0.5 * (p.mesh_max_local[2] + p.mesh_min_local[2]) * scale,
    };
  }
  return {0.0, 0.0, 0.0};
}

vec3 inverse_apply_point(const transform& t, const vec3& p) {
  const vec3 q = {p[0] - t.m[3][0], p[1] - t.m[3][1], p[2] - t.m[3][2]};
  vec3 r{};
  for (int row = 0; row < 3; ++row) {
    double s = 0.0;
    for (int col = 0; col < 3; ++col) {
      s += t.m[row][col] * q[col];
    }
    r[row] = s;
  }
  return r;
}

vec3 point_on_ray(const ray& r, double t) {
  return add(r.origin_world, scale(r.direction_world, t));
}

attachment_plane world_plane_from_local(const scene_node& node,
                                        std::size_t node_index,
                                        const vec3& point_local,
                                        const vec3& normal_local,
                                        const vec3& tangent_local) {
  return attachment_plane{
      .node_index = node_index,
      .point_world = transform_point(node.world_from_local, point_local),
      .normal_world = normalize(rotate_vector(node.world_from_local, normal_local)),
      .tangent_world = normalize(rotate_vector(node.world_from_local, tangent_local)),
  };
}

}  // namespace

const char* attachment_face_label(attachment_face face) {
  switch (face) {
    case attachment_face::cylinder_proximal_cap:
      return "Cylinder start cap";
    case attachment_face::cylinder_distal_cap:
      return "Cylinder end cap";
    case attachment_face::box_positive_x:
      return "Box +X face";
    case attachment_face::box_negative_x:
      return "Box -X face";
    case attachment_face::box_positive_y:
      return "Box +Y face";
    case attachment_face::box_negative_y:
      return "Box -Y face";
    case attachment_face::box_positive_z:
      return "Box +Z face";
    case attachment_face::box_negative_z:
      return "Box -Z face";
  }
  return "Unknown face";
}

std::optional<attachment_plane> attachment_plane_for_node(
    const scene_node& node, std::size_t node_index, attachment_face face) {
  if (!node.shape.has_value()) {
    return std::nullopt;
  }

  const primitive& p = *node.shape;
  vec3 point_local{};
  vec3 normal_local{};
  vec3 tangent_local{1.0, 0.0, 0.0};

  if (face == attachment_face::cylinder_proximal_cap ||
      face == attachment_face::cylinder_distal_cap) {
    if (p.kind != primitive_kind::cylinder && p.kind != primitive_kind::arrow) {
      return std::nullopt;
    }
    point_local = {0.0, 0.0,
                   face == attachment_face::cylinder_distal_cap ? p.length_m
                                                                 : 0.0};
    normal_local = {0.0, 0.0,
                    face == attachment_face::cylinder_distal_cap ? 1.0
                                                                  : -1.0};
  } else {
    if (p.kind != primitive_kind::box && p.kind != primitive_kind::mesh) {
      return std::nullopt;
    }
    const vec3 center = primitive_box_center(p);
    const vec3 half = primitive_box_half_extents(p);
    point_local = center;

    switch (face) {
      case attachment_face::box_positive_x:
        point_local[0] += half[0];
        normal_local = {1.0, 0.0, 0.0};
        tangent_local = {0.0, 1.0, 0.0};
        break;
      case attachment_face::box_negative_x:
        point_local[0] -= half[0];
        normal_local = {-1.0, 0.0, 0.0};
        tangent_local = {0.0, 1.0, 0.0};
        break;
      case attachment_face::box_positive_y:
        point_local[1] += half[1];
        normal_local = {0.0, 1.0, 0.0};
        tangent_local = {1.0, 0.0, 0.0};
        break;
      case attachment_face::box_negative_y:
        point_local[1] -= half[1];
        normal_local = {0.0, -1.0, 0.0};
        tangent_local = {1.0, 0.0, 0.0};
        break;
      case attachment_face::box_positive_z:
        point_local[2] += half[2];
        normal_local = {0.0, 0.0, 1.0};
        tangent_local = {1.0, 0.0, 0.0};
        break;
      case attachment_face::box_negative_z:
        point_local[2] -= half[2];
        normal_local = {0.0, 0.0, -1.0};
        tangent_local = {1.0, 0.0, 0.0};
        break;
      case attachment_face::cylinder_proximal_cap:
      case attachment_face::cylinder_distal_cap:
        return std::nullopt;
    }
  }

  return world_plane_from_local(node, node_index, point_local, normal_local,
                                tangent_local);
}

std::optional<attachment_plane> pick_attachment_plane(
    const scene_snapshot& snapshot, const ray& pick_ray) {
  const auto hit = pick(snapshot, pick_ray);
  if (!hit.has_value() || hit->node_index >= snapshot.nodes.size()) {
    return std::nullopt;
  }

  const scene_node& node = snapshot.nodes[hit->node_index];
  if (!node.shape.has_value()) {
    return std::nullopt;
  }

  const vec3 hit_world = point_on_ray(pick_ray, hit->t_along_ray);
  const vec3 hit_local = inverse_apply_point(node.world_from_local, hit_world);
  const primitive& p = *node.shape;

  if (p.kind == primitive_kind::cylinder || p.kind == primitive_kind::arrow) {
    const double cap_eps = std::max(1e-6, p.length_m * 1e-5);
    if (std::abs(hit_local[2]) <= cap_eps) {
      return world_plane_from_local(node, hit->node_index, {0.0, 0.0, 0.0},
                                    {0.0, 0.0, -1.0}, {1.0, 0.0, 0.0});
    }
    if (std::abs(hit_local[2] - p.length_m) <= cap_eps) {
      return world_plane_from_local(node, hit->node_index,
                                    {0.0, 0.0, p.length_m},
                                    {0.0, 0.0, 1.0}, {1.0, 0.0, 0.0});
    }

    const vec3 normal = normalize({hit_local[0], hit_local[1], 0.0});
    return world_plane_from_local(node, hit->node_index, hit_local, normal,
                                  {0.0, 0.0, 1.0});
  }

  if (p.kind == primitive_kind::box || p.kind == primitive_kind::mesh ||
      p.kind == primitive_kind::rotation_arrow) {
    const vec3 center = p.kind == primitive_kind::rotation_arrow
                            ? vec3{0.0, 0.0, 0.0}
                            : primitive_box_center(p);
    const vec3 half = p.kind == primitive_kind::rotation_arrow
                          ? vec3{p.radius_m, p.radius_m, 0.01}
                          : primitive_box_half_extents(p);
    const vec3 local_from_center = sub(hit_local, center);

    int axis = 0;
    double best = half[0] > 0.0 ? std::abs(local_from_center[0]) / half[0] : -1.0;
    for (int i = 1; i < 3; ++i) {
      const double score =
          half[i] > 0.0 ? std::abs(local_from_center[i]) / half[i] : -1.0;
      if (score > best) {
        best = score;
        axis = i;
      }
    }

    vec3 point = center;
    vec3 normal{};
    point[axis] += local_from_center[axis] >= 0.0 ? half[axis] : -half[axis];
    normal[axis] = local_from_center[axis] >= 0.0 ? 1.0 : -1.0;
    const vec3 tangent = axis == 0 ? vec3{0.0, 1.0, 0.0}
                                   : vec3{1.0, 0.0, 0.0};
    return world_plane_from_local(node, hit->node_index, point, normal, tangent);
  }

  return std::nullopt;
}

transform compute_attachment_target(const scene_node& moving_node,
                                    const attachment_plane& moving_plane,
                                    const attachment_plane& fixed_plane,
                                    const attachment_options& options) {
  const vec3 moving_normal_world = normalize(moving_plane.normal_world);
  const vec3 fixed_normal_world = normalize(fixed_plane.normal_world);
  const vec3 desired_normal_world =
      options.flip ? fixed_normal_world : scale(fixed_normal_world, -1.0);

  const basis from = make_basis(moving_normal_world, moving_plane.tangent_world);
  basis to = make_basis(desired_normal_world, fixed_plane.tangent_world);
  to.x = rotate_quarter_turns_around_axis(to.x, fixed_normal_world,
                                          options.quarter_turns);
  to.y = cross(to.z, to.x);

  transform delta = transform::identity();
  for (int col = 0; col < 3; ++col) {
    for (int row = 0; row < 3; ++row) {
      delta.m[col][row] = rotation_entry(to, from, col, row);
    }
  }

  const vec3 rotated_plane_point =
      rotate_vector(delta, moving_plane.point_world);
  const vec3 desired_plane_point =
      add(fixed_plane.point_world, scale(fixed_normal_world, options.offset_m));
  const vec3 translation = sub(desired_plane_point, rotated_plane_point);
  delta.m[3][0] = translation[0];
  delta.m[3][1] = translation[1];
  delta.m[3][2] = translation[2];

  transform target = transform::identity();
  for (int col = 0; col < 3; ++col) {
    const vec3 rotated_col =
        rotate_vector(delta, {moving_node.world_from_local.m[col][0],
                              moving_node.world_from_local.m[col][1],
                              moving_node.world_from_local.m[col][2]});
    target.m[col][0] = rotated_col[0];
    target.m[col][1] = rotated_col[1];
    target.m[col][2] = rotated_col[2];
  }

  const vec3 moved_origin = transform_point(
      delta,
      {moving_node.world_from_local.m[3][0],
       moving_node.world_from_local.m[3][1],
       moving_node.world_from_local.m[3][2]});
  target.m[3][0] = moved_origin[0];
  target.m[3][1] = moved_origin[1];
  target.m[3][2] = moved_origin[2];

  return target;
}

}  // namespace robosim::viz
