#pragma once

#include <array>
#include <string>
#include <vector>

namespace robosim::description {

enum class joint_type {
  revolute,
};

struct link {
  std::string name;
  double mass_kg;
  double length_m;
  double inertia_kgm2;

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
