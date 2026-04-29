#pragma once

// In-memory test fixtures used across the visualizer test suite.
// See tests/viz/TEST_PLAN.md "Test fixtures" and
// tests/viz/TEST_PLAN_VD.md "Test fixture mechanics".

#include "viz/scene_snapshot.h"

#include "description/schema.h"

#include <cstddef>
#include <string>
#include <string_view>

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

// v0-arm with shoulder.origin set to `joint_origin`. arm.visual_origin
// is left at identity. Used by VD section B / A tests.
[[nodiscard]] description::robot_description make_v0_arm_with_joint_origin(
    const description::origin_pose& joint_origin);

// Two-joint chain with all four origins explicitly set. Used by VD
// section B / A tests that need to discriminate kinematic-vs-visual
// frame propagation through more than one level.
[[nodiscard]] description::robot_description make_two_joint_chain_with_origins(
    const description::origin_pose& shoulder_origin,
    const description::origin_pose& elbow_origin,
    const description::origin_pose& arm_visual,
    const description::origin_pose& forearm_visual);

// Helpers shared across viz tests for locating snapshot nodes by
// (kind, name). Both fail the test (ADD_FAILURE) when no match exists
// and return a sentinel (the first node / index 0) so the test can
// continue and report further assertions.
[[nodiscard]] const scene_node& find_node(
    const scene_snapshot& s, node_kind kind, std::string_view name);
[[nodiscard]] std::size_t find_node_index(
    const scene_snapshot& s, node_kind kind, std::string_view name);

}  // namespace robosim::viz::testing
