#include "fixtures.h"

#include "description/schema.h"

#include <algorithm>
#include <string>

namespace robosim::viz::testing {

namespace desc = robosim::description;

desc::robot_description make_v0_arm_description() {
  desc::robot_description d;
  d.schema_version = 1;
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
  d.schema_version = 1;
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

}  // namespace robosim::viz::testing
