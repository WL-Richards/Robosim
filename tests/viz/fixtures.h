#pragma once

// In-memory test fixtures used across the visualizer test suite.
// See tests/viz/TEST_PLAN.md "Test fixtures".

#include "description/schema.h"

#include <string>

namespace robosim::viz::testing {

// In-memory mirror of descriptions/v0-arm.json.
[[nodiscard]] description::robot_description make_v0_arm_description();

// Two-joint chain: extends v0-arm with a second link `forearm` and a
// second revolute joint `elbow` whose parent="arm", child="forearm".
// Use `joints_in_reverse_order = true` to construct the JSON-equivalent
// with `elbow` listed before `shoulder` in the joints vector — used by
// the topological-sort tests.
[[nodiscard]] description::robot_description
make_two_joint_chain_description(bool joints_in_reverse_order = false);

// Same shape as v0-arm, but every name field is at least 32 chars to
// defeat std::string's small-string optimization. Used by the
// scene_node string-lifetime tests.
[[nodiscard]] description::robot_description make_long_named_description();

}  // namespace robosim::viz::testing
