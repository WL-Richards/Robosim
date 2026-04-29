#include "serializer.h"

#include "schema.h"

#include <nlohmann/json.hpp>

#include <bit>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <filesystem>
#include <fstream>
#include <string>

namespace robosim::description {

using nlohmann::ordered_json;
namespace fs = std::filesystem;

namespace {

// Bit-exact identity check. IEEE-754 == conflates -0.0 and +0.0, which would
// cause the serializer to omit an origin containing -0.0 (since -0.0 == +0.0).
// S5 requires -0.0 to survive save→load round-trip, so the comparison is
// bit-exact: identity means every bit is zero (all elements are +0.0).
bool origin_is_identity(const origin_pose& p) {
  for (std::size_t i = 0; i < 3; ++i) {
    if (std::bit_cast<std::uint64_t>(p.xyz_m[i]) != 0) return false;
    if (std::bit_cast<std::uint64_t>(p.rpy_rad[i]) != 0) return false;
  }
  return true;
}

ordered_json serialize_origin(const origin_pose& p) {
  ordered_json o = ordered_json::object();
  o["xyz_m"] = p.xyz_m;
  o["rpy_rad"] = p.rpy_rad;
  return o;
}

ordered_json serialize_link(const link& l) {
  ordered_json j = ordered_json::object();
  j["name"] = l.name;
  j["mass_kg"] = l.mass_kg;
  j["length_m"] = l.length_m;
  j["inertia_kgm2"] = l.inertia_kgm2;
  if (!origin_is_identity(l.visual_origin)) {
    j["visual_origin"] = serialize_origin(l.visual_origin);
  }
  return j;
}

std::string joint_type_to_string(joint_type t) {
  switch (t) {
    case joint_type::revolute:
      return "revolute";
  }
  return "";
}

ordered_json serialize_joint(const joint& jt) {
  ordered_json j = ordered_json::object();
  j["name"] = jt.name;
  j["type"] = joint_type_to_string(jt.type);
  j["parent"] = jt.parent;
  j["child"] = jt.child;
  j["axis"] = jt.axis;
  j["limit_lower_rad"] = jt.limit_lower_rad;
  j["limit_upper_rad"] = jt.limit_upper_rad;
  j["viscous_friction_nm_per_rad_per_s"] = jt.viscous_friction_nm_per_rad_per_s;
  if (!origin_is_identity(jt.origin)) {
    j["origin"] = serialize_origin(jt.origin);
  }
  return j;
}

ordered_json serialize_motor(const motor& m) {
  ordered_json j = ordered_json::object();
  j["name"] = m.name;
  j["motor_model"] = m.motor_model;
  j["controller"] = m.controller;
  j["controller_can_id"] = m.controller_can_id;
  j["controller_firmware_version"] = m.controller_firmware_version;
  j["joint"] = m.joint_name;
  j["gear_ratio"] = m.gear_ratio;
  return j;
}

ordered_json serialize_sensor(const sensor& s) {
  ordered_json j = ordered_json::object();
  j["name"] = s.name;
  j["sensor_model"] = s.sensor_model;
  j["controller_can_id"] = s.controller_can_id;
  j["controller_firmware_version"] = s.controller_firmware_version;
  j["joint"] = s.joint_name;
  return j;
}

}  // namespace

std::expected<void, save_error> save_to_file(
    const robot_description& d, const fs::path& path) {
  ordered_json root = ordered_json::object();
  root["schema_version"] = d.schema_version;
  root["name"] = d.name;

  ordered_json links_arr = ordered_json::array();
  for (const auto& l : d.links) {
    links_arr.push_back(serialize_link(l));
  }
  root["links"] = std::move(links_arr);

  ordered_json joints_arr = ordered_json::array();
  for (const auto& jt : d.joints) {
    joints_arr.push_back(serialize_joint(jt));
  }
  root["joints"] = std::move(joints_arr);

  ordered_json motors_arr = ordered_json::array();
  for (const auto& m : d.motors) {
    motors_arr.push_back(serialize_motor(m));
  }
  root["motors"] = std::move(motors_arr);

  ordered_json sensors_arr = ordered_json::array();
  for (const auto& s : d.sensors) {
    sensors_arr.push_back(serialize_sensor(s));
  }
  root["sensors"] = std::move(sensors_arr);

  const std::string text = root.dump(2);

  std::ofstream out(path);
  if (!out) {
    return std::unexpected(save_error{
        .kind = save_error_kind::io_error,
        .file_path = path,
        .message = "failed to open file for writing: " + path.string(),
    });
  }
  out << text;
  out.close();
  if (!out) {
    return std::unexpected(save_error{
        .kind = save_error_kind::io_error,
        .file_path = path,
        .message = "failed to write file: " + path.string(),
    });
  }
  return {};
}

}  // namespace robosim::description
