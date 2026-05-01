#pragma once

// Scene snapshot — the data-source seam between any producer (Edit-mode
// builder, Live-mode state stream, Replay-mode WPILOG sample) and the
// renderer. The renderer reads only from this struct.
//
// See .claude/skills/visualizer.md "Scene-snapshot seam" for the
// architectural commitment, and tests/viz/TEST_PLAN.md sections T and S
// for the contract pinned by tests.

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace robosim::viz {

enum class primitive_kind {
  cylinder,
  box,
  sphere,
  arrow,
  rotation_arrow,
  mesh,
};

// Cylinder / arrow are anchored at the proximal cap (local origin
// at z = 0) and extend along +Z to z = length_m. Rotation arrows
// are flat glyphs centered at the local origin in the XY plane, with
// radius_m as the ring radius. Box / sphere are centered at the local origin.
// See tests/viz/TEST_PLAN.md
// conventions #8 / #9 / #10 / #11.
struct primitive {
  primitive() = default;
  primitive(primitive_kind kind_in, double length_m_in, double radius_m_in,
            double half_extent_x_m_in, double half_extent_y_m_in,
            double half_extent_z_m_in)
      : kind(kind_in),
        length_m(length_m_in),
        radius_m(radius_m_in),
        half_extent_x_m(half_extent_x_m_in),
        half_extent_y_m(half_extent_y_m_in),
        half_extent_z_m(half_extent_z_m_in) {}

  primitive_kind kind = primitive_kind::box;
  double length_m = 0.0;          // cylinder, arrow
  double radius_m = 0.0;          // cylinder, arrow, sphere, rotation_arrow
  double half_extent_x_m = 0.0;   // box
  double half_extent_y_m = 0.0;   // box
  double half_extent_z_m = 0.0;   // box
  std::string mesh_id;      // mesh
  double mesh_scale_m_per_unit = 1.0;
  std::array<double, 3> mesh_min_local{};
  std::array<double, 3> mesh_max_local{};

  bool operator==(const primitive&) const = default;
};

// 4x4 column-major homogeneous transform. m[col][row] indexing — see
// tests/viz/TEST_PLAN.md "Conventions" #2/#3.
struct transform {
  std::array<std::array<double, 4>, 4> m{};

  static transform identity();
  bool operator==(const transform&) const = default;
};

enum class node_kind {
  link,
  joint,
  motor,
  motor_direction_arrow,
};

struct scene_node {
  std::string entity_name;
  node_kind kind;
  std::optional<std::size_t> parent_index;
  transform world_from_local;
  std::optional<primitive> shape;
  std::array<double, 3> joint_axis_local{};

  // Per-frame slots reserved for Live / Replay; v0 leaves these empty.
  std::optional<double> joint_angle_rad;
  std::vector<std::array<double, 3>> contact_points_world;
  std::vector<std::array<double, 3>> force_vectors_world;

  bool operator==(const scene_node&) const = default;
};

struct scene_snapshot {
  std::vector<scene_node> nodes;
  std::optional<std::size_t> selected_index;

  bool operator==(const scene_snapshot&) const = default;
};

}  // namespace robosim::viz
