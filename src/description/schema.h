#pragma once

#include <array>
#include <string>
#include <vector>

namespace robosim::description {

enum class joint_type {
  revolute,
};

// Schema v2 additions: see tests/description/TEST_PLAN_VC.md D-VC-5a.
// Stored raw (no composed transform on the struct, per D-VC-5b);
// composition into a 4x4 lives in compose_origin() in origin_pose.h.
//
// rpy_rad element order is URDF's [roll, pitch, yaw]:
//   rpy_rad[0] = roll, rpy_rad[1] = pitch, rpy_rad[2] = yaw.
//
// Default-initialized origin_pose{} is the identity pose
// (xyz_m == {0,0,0}, rpy_rad == {0,0,0}); v1 inputs and v2 inputs
// without an origin key both land here per D-VC-2.
struct origin_pose {
  std::array<double, 3> xyz_m{};
  std::array<double, 3> rpy_rad{};

  bool operator==(const origin_pose&) const = default;
};

struct link {
  std::string name;
  double mass_kg;
  double length_m;
  double inertia_kgm2;
  origin_pose visual_origin{};  // schema v2; identity for v1 inputs.

  bool operator==(const link&) const = default;
};

struct joint {
  std::string name;
  joint_type type;
  std::string parent;  // link name or the special value "world"
  std::string child;   // link name; "world" rejected
  std::array<double, 3> axis;
  double limit_lower_rad;
  double limit_upper_rad;
  double viscous_friction_nm_per_rad_per_s;
  origin_pose origin{};  // schema v2; identity for v1 inputs.

  bool operator==(const joint&) const = default;
};

struct motor {
  std::string name;
  std::string motor_model;
  std::string controller;
  int controller_can_id;
  std::string controller_firmware_version;
  std::string joint_name;
  double gear_ratio;
  std::string connection_type = "CAN";
  origin_pose visual_origin{};
  bool show_direction_arrow = true;

  bool operator==(const motor&) const = default;
};

struct sensor {
  std::string name;
  std::string sensor_model;
  int controller_can_id;
  std::string controller_firmware_version;
  std::string joint_name;

  bool operator==(const sensor&) const = default;
};

struct robot_description {
  int schema_version;
  std::string name;
  std::vector<link> links;
  std::vector<joint> joints;
  std::vector<motor> motors;
  std::vector<sensor> sensors;

  bool operator==(const robot_description&) const = default;
};

}  // namespace robosim::description
