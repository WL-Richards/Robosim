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
};

// Cylinder / arrow are anchored at the proximal cap (local origin
// at z = 0) and extend along +Z to z = length_m. Box / sphere are
// centered at the local origin. See tests/viz/TEST_PLAN.md
// conventions #8 / #9 / #10 / #11.
struct primitive {
  primitive_kind kind;
  double length_m;          // cylinder, arrow
  double radius_m;          // cylinder, arrow, sphere
  double half_extent_x_m;   // box
  double half_extent_y_m;   // box
  double half_extent_z_m;   // box

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
