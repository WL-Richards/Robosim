#include "fixtures.h"

#include "viz/scene_snapshot.h"

#include "description/schema.h"

#include <algorithm>
#include <cstddef>
#include <string>
#include <string_view>

#include <gtest/gtest.h>

namespace robosim::viz::testing {

namespace desc = robosim::description;

desc::robot_description make_v0_arm_description() {
  desc::robot_description d;
  d.schema_version = 2;
  d.name = "v0-arm";
  d.links = {
      desc::link{"arm", 2.0, 0.5, 0.166},
  };
  d.joints = {
      desc::joint{
          "shoulder", desc::joint_type::revolute, "world", "arm",
          {0.0, 1.0, 0.0}, -1.5708, 1.5708, 0.1,
      },
  };
  d.motors = {
      desc::motor{
          "shoulder_motor", "kraken_x60", "talon_fx", 1,
          "TBD-pinned-at-v0-ship", "shoulder", 100.0,
      },
  };
  d.sensors = {
      desc::sensor{
          "shoulder_encoder", "cancoder", 2, "TBD-pinned-at-v0-ship",
          "shoulder",
      },
  };
  return d;
}

desc::robot_description make_two_joint_chain_description(
    bool joints_in_reverse_order) {
  desc::robot_description d = make_v0_arm_description();
  d.name = "two-joint-chain";
  d.links.push_back(desc::link{"forearm", 1.5, 0.4, 0.08});
  desc::joint elbow{
      "elbow", desc::joint_type::revolute, "arm", "forearm",
      {0.0, 1.0, 0.0}, -1.5708, 1.5708, 0.05,
  };
  if (joints_in_reverse_order) {
    d.joints.insert(d.joints.begin(), std::move(elbow));
  } else {
    d.joints.push_back(std::move(elbow));
  }
  return d;
}

desc::robot_description make_long_named_description() {
  desc::robot_description d;
  d.schema_version = 2;
  d.name = "v0-arm-with-thirty-two-character-name";  // 36 chars
  const std::string long_link_name =
      "arm_link_with_thirty_two_character_name";  // 39
  const std::string long_joint_name =
      "shoulder_joint_thirty_two_character_name";  // 40
  const std::string long_motor_name =
      "shoulder_motor_thirty_two_character_name";  // 40
  const std::string long_sensor_name =
      "shoulder_encoder_thirty_two_char_name__";  // 39
  d.links = {desc::link{long_link_name, 2.0, 0.5, 0.166}};
  d.joints = {
      desc::joint{
          long_joint_name, desc::joint_type::revolute, "world",
          long_link_name, {0.0, 1.0, 0.0}, -1.5708, 1.5708, 0.1,
      },
  };
  d.motors = {
      desc::motor{
          long_motor_name, "kraken_x60", "talon_fx", 1,
          "TBD-pinned-at-v0-ship-thirty-two-char-version",
          long_joint_name, 100.0,
      },
  };
  d.sensors = {
      desc::sensor{
          long_sensor_name, "cancoder", 2,
          "TBD-pinned-at-v0-ship-thirty-two-char-version",
          long_joint_name,
      },
  };
  return d;
}

desc::robot_description make_v0_arm_with_joint_origin(
    const desc::origin_pose& joint_origin) {
  desc::robot_description d = make_v0_arm_description();
  d.joints[0].origin = joint_origin;
  d.links[0].visual_origin = desc::origin_pose{};
  d.motors[0].visual_origin = desc::origin_pose{};
  return d;
}

desc::robot_description make_two_joint_chain_with_origins(
    const desc::origin_pose& shoulder_origin,
    const desc::origin_pose& elbow_origin,
    const desc::origin_pose& arm_visual,
    const desc::origin_pose& forearm_visual) {
  desc::robot_description d = make_two_joint_chain_description();
  // make_two_joint_chain_description appends elbow at the end, so
  // joints[0] = shoulder, joints[1] = elbow; links[0] = arm,
  // links[1] = forearm.
  d.joints[0].origin = shoulder_origin;
  d.joints[1].origin = elbow_origin;
  d.links[0].visual_origin = arm_visual;
  d.links[1].visual_origin = forearm_visual;
  return d;
}

const scene_node& find_node(
    const scene_snapshot& s, node_kind kind, std::string_view name) {
  for (const auto& node : s.nodes) {
    if (node.kind == kind && node.entity_name == name) {
      return node;
    }
  }
  ADD_FAILURE() << "find_node: no matching node (kind="
                << static_cast<int>(kind) << ", name=" << name << ")";
  return s.nodes.front();
}

std::size_t find_node_index(
    const scene_snapshot& s, node_kind kind, std::string_view name) {
  for (std::size_t i = 0; i < s.nodes.size(); ++i) {
    if (s.nodes[i].kind == kind && s.nodes[i].entity_name == name) {
      return i;
    }
  }
  ADD_FAILURE() << "find_node_index: no matching node (kind="
                << static_cast<int>(kind) << ", name=" << name << ")";
  return 0;
}

}  // namespace robosim::viz::testing
