#pragma once

// Attachment / mate math for the Edit-mode "click one plane, then
// another plane" workflow. This file intentionally stays
// description-free: UI code can feed the returned transform into the
// existing apply_gizmo_target write path.

#include "scene_snapshot.h"
#include "picking.h"

#include <array>
#include <cstddef>
#include <optional>

namespace robosim::viz {

struct attachment_plane {
  std::size_t node_index = 0;
  std::array<double, 3> point_world{};
  std::array<double, 3> normal_world{0.0, 0.0, 1.0};
  std::array<double, 3> tangent_world{1.0, 0.0, 0.0};
};

struct attachment_options {
  double offset_m = 0.0;
  int quarter_turns = 0;
  bool flip = false;
};

enum class attachment_face {
  cylinder_proximal_cap,
  cylinder_distal_cap,
  box_positive_x,
  box_negative_x,
  box_positive_y,
  box_negative_y,
  box_positive_z,
  box_negative_z,
};

[[nodiscard]] const char* attachment_face_label(attachment_face face);

[[nodiscard]] std::optional<attachment_plane> attachment_plane_for_node(
    const scene_node& node, std::size_t node_index, attachment_face face);

[[nodiscard]] std::optional<attachment_plane> pick_attachment_plane(
    const scene_snapshot& snapshot, const ray& pick_ray);

// Computes a target world_from_local for moving `moving_node` so the
// picked `moving_plane` mates to `fixed_plane`.
//
// Default behavior places the moving plane's point on the fixed plane
// point, with its normal opposing the fixed plane normal. `flip`
// makes the normals point the same way instead. `quarter_turns` rotates
// the result around the fixed plane normal in 90 degree increments.
// `offset_m` separates the planes along the fixed normal.
[[nodiscard]] transform compute_attachment_target(
    const scene_node& moving_node, const attachment_plane& moving_plane,
    const attachment_plane& fixed_plane, const attachment_options& options);

}  // namespace robosim::viz
