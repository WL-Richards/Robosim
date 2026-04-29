#pragma once

// Apply a gizmo-produced target world transform to the selected node's
// origin in a robot_description. Pure logic — no ImGuizmo dependency.
//
// See tests/viz/TEST_PLAN_VD.md "A. apply_gizmo_target" for the
// pinned contract.

#include "scene_snapshot.h"

#include "description/schema.h"

#include <cstddef>

namespace robosim::viz {

// Returns the description with the appropriate origin field
// (joint.origin for joint nodes, link.visual_origin for link nodes)
// updated so that build_edit_mode_snapshot(returned) places the
// selected node at target_world_from_local within 1e-9 (within 1e-12
// for the special root + identity-rotation no-op case).
//
// Preconditions (caller's responsibility):
//   - selected_index < snapshot.nodes.size();
//   - snapshot was built from `source` via build_edit_mode_snapshot.
//
// Behavior is undefined when preconditions are violated. ASan/UBSan
// catch out-of-range access; v0 does not pin a runtime test for the
// out-of-range case — main.cpp only invokes apply when ImGuizmo's
// IsUsing() returns true, which implies a current selection, which
// implies a valid index.
//
// Does not mutate `source` or `snapshot`. Other description fields
// (motors, sensors, mass_kg, every other link/joint) are copied
// through unchanged.
[[nodiscard]] description::robot_description apply_gizmo_target(
    const description::robot_description& source,
    const scene_snapshot& snapshot,
    std::size_t selected_index,
    const transform& target_world_from_local);

}  // namespace robosim::viz
