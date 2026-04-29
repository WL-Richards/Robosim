#include "loader.h"

#include "error.h"
#include "schema.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstddef>
#include <expected>
#include <filesystem>
#include <fstream>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace robosim::description {
namespace {

using nlohmann::json;
namespace fs = std::filesystem;

// ---------------------------------------------------------------------------
// Error construction
// ---------------------------------------------------------------------------

load_error mkerr(load_error_kind kind, fs::path path, std::string ptr, std::string msg) {
  return load_error{kind, std::move(path), std::move(ptr), std::move(msg)};
}

std::string ptr_join(std::string_view parent, std::string_view leaf) {
  std::string out;
  out.reserve(parent.size() + 1 + leaf.size());
  out.append(parent);
  out.push_back('/');
  out.append(leaf);
  return out;
}

std::string ptr_index(std::string_view parent, std::size_t index) {
  return ptr_join(parent, std::to_string(index));
}

// ---------------------------------------------------------------------------
// Type-strict field accessors. Each returns the leaf value or a schema_error
// pointing at /<parent>/<key>. The "missing" message contains the leaf field
// name so C1 tests pass.
// ---------------------------------------------------------------------------

std::expected<const json*, load_error> require_present(const json& obj,
                                                       std::string_view key,
                                                       const fs::path& path,
                                                       std::string_view parent_ptr) {
  auto it = obj.find(key);
  if (it == obj.end()) {
    return std::unexpected(mkerr(load_error_kind::schema_error,
                                 path,
                                 ptr_join(parent_ptr, key),
                                 "missing required field: " + std::string(key)));
  }
  return &*it;
}

std::expected<int, load_error> require_integer(const json& obj,
                                               std::string_view key,
                                               const fs::path& path,
                                               std::string_view parent_ptr) {
  auto present = require_present(obj, key, path, parent_ptr);
  if (!present) {
    return std::unexpected(present.error());
  }
  if (!(*present)->is_number_integer()) {
    return std::unexpected(mkerr(load_error_kind::schema_error,
                                 path,
                                 ptr_join(parent_ptr, key),
                                 "field " + std::string(key) + " must be an integer"));
  }
  return (*present)->get<int>();
}

std::expected<double, load_error> require_number(const json& obj,
                                                 std::string_view key,
                                                 const fs::path& path,
                                                 std::string_view parent_ptr) {
  auto present = require_present(obj, key, path, parent_ptr);
  if (!present) {
    return std::unexpected(present.error());
  }
  // Accept JSON integer literals in float fields (decision #6) but reject
  // bool / string / etc.
  if (!(*present)->is_number_integer() && !(*present)->is_number_float()) {
    return std::unexpected(mkerr(load_error_kind::schema_error,
                                 path,
                                 ptr_join(parent_ptr, key),
                                 "field " + std::string(key) + " must be a number"));
  }
  return (*present)->get<double>();
}

std::expected<std::string, load_error> require_string(const json& obj,
                                                      std::string_view key,
                                                      const fs::path& path,
                                                      std::string_view parent_ptr) {
  auto present = require_present(obj, key, path, parent_ptr);
  if (!present) {
    return std::unexpected(present.error());
  }
  if (!(*present)->is_string()) {
    return std::unexpected(mkerr(load_error_kind::schema_error,
                                 path,
                                 ptr_join(parent_ptr, key),
                                 "field " + std::string(key) + " must be a string"));
  }
  return (*present)->get<std::string>();
}

std::expected<std::array<double, 3>, load_error> require_axis(const json& obj,
                                                              std::string_view key,
                                                              const fs::path& path,
                                                              std::string_view parent_ptr) {
  auto present = require_present(obj, key, path, parent_ptr);
  if (!present) {
    return std::unexpected(present.error());
  }
  const json& v = **present;
  if (!v.is_array() || v.size() != 3) {
    return std::unexpected(
        mkerr(load_error_kind::schema_error,
              path,
              ptr_join(parent_ptr, key),
              "field " + std::string(key) + " must be an array of three numbers"));
  }
  std::array<double, 3> out{};
  for (std::size_t i = 0; i < 3; ++i) {
    if (!v[i].is_number_integer() && !v[i].is_number_float()) {
      return std::unexpected(
          mkerr(load_error_kind::schema_error,
                path,
                ptr_join(parent_ptr, key),
                "field " + std::string(key) + " must be an array of three numbers"));
    }
    out[i] = v[i].get<double>();
  }
  return out;
}

std::expected<const json*, load_error> require_array(const json& obj,
                                                     std::string_view key,
                                                     const fs::path& path,
                                                     std::string_view parent_ptr) {
  auto present = require_present(obj, key, path, parent_ptr);
  if (!present) {
    return std::unexpected(present.error());
  }
  if (!(*present)->is_array()) {
    return std::unexpected(mkerr(load_error_kind::schema_error,
                                 path,
                                 ptr_join(parent_ptr, key),
                                 "field " + std::string(key) + " must be an array"));
  }
  return *present;
}

// ---------------------------------------------------------------------------
// Unknown-field rejection (decision #8).
// ---------------------------------------------------------------------------

std::expected<void, load_error> check_no_unknown_keys(const json& obj,
                                                      std::span<const std::string_view> known,
                                                      const fs::path& path,
                                                      std::string_view parent_ptr) {
  for (auto it = obj.begin(); it != obj.end(); ++it) {
    const std::string& key = it.key();
    const bool is_known =
        std::any_of(known.begin(), known.end(), [&key](std::string_view k) { return key == k; });
    if (!is_known) {
      return std::unexpected(mkerr(
          load_error_kind::schema_error, path, ptr_join(parent_ptr, key), "unknown field: " + key));
    }
  }
  return {};
}

// ---------------------------------------------------------------------------
// Domain checks (decision #10 plus other numerical invariants).
// ---------------------------------------------------------------------------

std::expected<void, load_error> check_positive(double value,
                                               std::string_view leaf_key,
                                               const fs::path& path,
                                               const std::string& field_ptr) {
  if (!(value > 0.0)) {
    return std::unexpected(mkerr(load_error_kind::domain_error,
                                 path,
                                 field_ptr,
                                 "field " + std::string(leaf_key) + " must be > 0"));
  }
  return {};
}

std::expected<void, load_error> check_non_negative(double value,
                                                   std::string_view leaf_key,
                                                   const fs::path& path,
                                                   const std::string& field_ptr) {
  if (!(value >= 0.0)) {
    return std::unexpected(mkerr(load_error_kind::domain_error,
                                 path,
                                 field_ptr,
                                 "field " + std::string(leaf_key) + " must be >= 0"));
  }
  return {};
}

std::expected<void, load_error> check_can_id(int value,
                                             std::string_view leaf_key,
                                             const fs::path& path,
                                             const std::string& field_ptr) {
  if (value < 0 || value > 63) {
    return std::unexpected(
        mkerr(load_error_kind::domain_error,
              path,
              field_ptr,
              "field " + std::string(leaf_key) + " must be in the inclusive range [0, 63]"));
  }
  return {};
}

// ---------------------------------------------------------------------------
// Per-entity loaders.
// ---------------------------------------------------------------------------

constexpr std::array<std::string_view, 4> k_link_keys = {
    "name", "mass_kg", "length_m", "inertia_kgm2"};

constexpr std::array<std::string_view, 8> k_joint_keys = {"name",
                                                          "type",
                                                          "parent",
                                                          "child",
                                                          "axis",
                                                          "limit_lower_rad",
                                                          "limit_upper_rad",
                                                          "viscous_friction_nm_per_rad_per_s"};

constexpr std::array<std::string_view, 7> k_motor_keys = {"name",
                                                          "motor_model",
                                                          "controller",
                                                          "controller_can_id",
                                                          "controller_firmware_version",
                                                          "joint",
                                                          "gear_ratio"};

constexpr std::array<std::string_view, 5> k_sensor_keys = {
    "name", "sensor_model", "controller_can_id", "controller_firmware_version", "joint"};

constexpr std::array<std::string_view, 6> k_root_keys = {
    "schema_version", "name", "links", "joints", "motors", "sensors"};

std::expected<link, load_error> load_link(const json& obj,
                                          std::size_t index,
                                          const fs::path& path) {
  const std::string ptr = ptr_index("/links", index);
  link out;
  if (auto r = require_string(obj, "name", path, ptr); r) {
    out.name = std::move(*r);
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = require_number(obj, "mass_kg", path, ptr); r) {
    out.mass_kg = *r;
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = require_number(obj, "length_m", path, ptr); r) {
    out.length_m = *r;
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = require_number(obj, "inertia_kgm2", path, ptr); r) {
    out.inertia_kgm2 = *r;
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = check_no_unknown_keys(obj, k_link_keys, path, ptr); !r) {
    return std::unexpected(r.error());
  }
  if (auto r = check_positive(out.mass_kg, "mass_kg", path, ptr_join(ptr, "mass_kg")); !r) {
    return std::unexpected(r.error());
  }
  if (auto r =
          check_positive(out.inertia_kgm2, "inertia_kgm2", path, ptr_join(ptr, "inertia_kgm2"));
      !r) {
    return std::unexpected(r.error());
  }
  if (auto r = check_non_negative(out.length_m, "length_m", path, ptr_join(ptr, "length_m")); !r) {
    return std::unexpected(r.error());
  }
  return out;
}

std::expected<joint_type, load_error> parse_joint_type(const std::string& s,
                                                       const fs::path& path,
                                                       const std::string& field_ptr) {
  if (s == "revolute") {
    return joint_type::revolute;
  }
  return std::unexpected(
      mkerr(load_error_kind::schema_error, path, field_ptr, "unknown joint type: " + s));
}

std::expected<joint, load_error> load_joint(const json& obj,
                                            std::size_t index,
                                            const fs::path& path) {
  const std::string ptr = ptr_index("/joints", index);
  joint out;
  if (auto r = require_string(obj, "name", path, ptr); r) {
    out.name = std::move(*r);
  } else {
    return std::unexpected(r.error());
  }
  std::string type_str;
  if (auto r = require_string(obj, "type", path, ptr); r) {
    type_str = std::move(*r);
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = require_string(obj, "parent", path, ptr); r) {
    out.parent = std::move(*r);
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = require_string(obj, "child", path, ptr); r) {
    out.child = std::move(*r);
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = require_axis(obj, "axis", path, ptr); r) {
    out.axis = *r;
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = require_number(obj, "limit_lower_rad", path, ptr); r) {
    out.limit_lower_rad = *r;
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = require_number(obj, "limit_upper_rad", path, ptr); r) {
    out.limit_upper_rad = *r;
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = require_number(obj, "viscous_friction_nm_per_rad_per_s", path, ptr); r) {
    out.viscous_friction_nm_per_rad_per_s = *r;
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = check_no_unknown_keys(obj, k_joint_keys, path, ptr); !r) {
    return std::unexpected(r.error());
  }
  // Enum strictness (L1/L2). Done after presence + type checks.
  if (auto r = parse_joint_type(type_str, path, ptr_join(ptr, "type")); r) {
    out.type = *r;
  } else {
    return std::unexpected(r.error());
  }
  // Domain: lower <= upper. Per second-offender rule, point at limit_upper_rad
  // (parsed after limit_lower_rad).
  if (out.limit_lower_rad > out.limit_upper_rad) {
    return std::unexpected(
        mkerr(load_error_kind::domain_error,
              path,
              ptr_join(ptr, "limit_upper_rad"),
              "joint " + out.name + ": limit_upper_rad must be >= limit_lower_rad"));
  }
  return out;
}

std::expected<motor, load_error> load_motor(const json& obj,
                                            std::size_t index,
                                            const fs::path& path) {
  const std::string ptr = ptr_index("/motors", index);
  motor out;
  if (auto r = require_string(obj, "name", path, ptr); r) {
    out.name = std::move(*r);
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = require_string(obj, "motor_model", path, ptr); r) {
    out.motor_model = std::move(*r);
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = require_string(obj, "controller", path, ptr); r) {
    out.controller = std::move(*r);
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = require_integer(obj, "controller_can_id", path, ptr); r) {
    out.controller_can_id = *r;
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = require_string(obj, "controller_firmware_version", path, ptr); r) {
    out.controller_firmware_version = std::move(*r);
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = require_string(obj, "joint", path, ptr); r) {
    out.joint_name = std::move(*r);
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = require_number(obj, "gear_ratio", path, ptr); r) {
    out.gear_ratio = *r;
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = check_no_unknown_keys(obj, k_motor_keys, path, ptr); !r) {
    return std::unexpected(r.error());
  }
  if (auto r = check_can_id(
          out.controller_can_id, "controller_can_id", path, ptr_join(ptr, "controller_can_id"));
      !r) {
    return std::unexpected(r.error());
  }
  if (auto r = check_positive(out.gear_ratio, "gear_ratio", path, ptr_join(ptr, "gear_ratio"));
      !r) {
    return std::unexpected(r.error());
  }
  return out;
}

std::expected<sensor, load_error> load_sensor(const json& obj,
                                              std::size_t index,
                                              const fs::path& path) {
  const std::string ptr = ptr_index("/sensors", index);
  sensor out;
  if (auto r = require_string(obj, "name", path, ptr); r) {
    out.name = std::move(*r);
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = require_string(obj, "sensor_model", path, ptr); r) {
    out.sensor_model = std::move(*r);
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = require_integer(obj, "controller_can_id", path, ptr); r) {
    out.controller_can_id = *r;
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = require_string(obj, "controller_firmware_version", path, ptr); r) {
    out.controller_firmware_version = std::move(*r);
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = require_string(obj, "joint", path, ptr); r) {
    out.joint_name = std::move(*r);
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = check_no_unknown_keys(obj, k_sensor_keys, path, ptr); !r) {
    return std::unexpected(r.error());
  }
  if (auto r = check_can_id(
          out.controller_can_id, "controller_can_id", path, ptr_join(ptr, "controller_can_id"));
      !r) {
    return std::unexpected(r.error());
  }
  return out;
}

// ---------------------------------------------------------------------------
// Cross-entity validation: name uniqueness, references, CAN ID conflicts.
// All apply the second-offender rule (decision #2).
// ---------------------------------------------------------------------------

template <typename Entity>
std::expected<void, load_error> check_unique_names(const std::vector<Entity>& entities,
                                                   std::string_view kind_pointer_prefix,
                                                   const fs::path& path) {
  std::unordered_map<std::string, std::size_t> seen;
  for (std::size_t i = 0; i < entities.size(); ++i) {
    const auto& name = entities[i].name;
    auto [it, inserted] = seen.try_emplace(name, i);
    if (!inserted) {
      return std::unexpected(mkerr(load_error_kind::conflict_error,
                                   path,
                                   ptr_join(ptr_index(kind_pointer_prefix, i), "name"),
                                   "duplicate name '" + name + "'"));
    }
  }
  return {};
}

std::expected<void, load_error> check_joint_references(const std::vector<joint>& joints,
                                                       const std::vector<link>& links,
                                                       const fs::path& path) {
  std::unordered_set<std::string> link_names;
  for (const auto& l : links) {
    link_names.insert(l.name);
  }
  for (std::size_t i = 0; i < joints.size(); ++i) {
    const auto& j = joints[i];
    const std::string jptr = ptr_index("/joints", i);
    // Parent: link or "world".
    if (j.parent != "world" && !link_names.contains(j.parent)) {
      return std::unexpected(
          mkerr(load_error_kind::reference_error,
                path,
                ptr_join(jptr, "parent"),
                "joint " + j.name + " parent references unknown link '" + j.parent + "'"));
    }
    // Child: link only — "world" rejected (decision #9).
    if (j.child == "world" || !link_names.contains(j.child)) {
      return std::unexpected(
          mkerr(load_error_kind::reference_error,
                path,
                ptr_join(jptr, "child"),
                "joint " + j.name + " child references unknown link '" + j.child + "'"));
    }
  }
  return {};
}

std::expected<void, load_error> check_motor_joint_references(const std::vector<motor>& motors,
                                                             const std::vector<joint>& joints,
                                                             const fs::path& path) {
  std::unordered_set<std::string> joint_names;
  for (const auto& j : joints) {
    joint_names.insert(j.name);
  }
  for (std::size_t i = 0; i < motors.size(); ++i) {
    const auto& m = motors[i];
    if (!joint_names.contains(m.joint_name)) {
      return std::unexpected(
          mkerr(load_error_kind::reference_error,
                path,
                ptr_join(ptr_index("/motors", i), "joint"),
                "motor " + m.name + " references unknown joint '" + m.joint_name + "'"));
    }
  }
  return {};
}

std::expected<void, load_error> check_sensor_joint_references(const std::vector<sensor>& sensors,
                                                              const std::vector<joint>& joints,
                                                              const fs::path& path) {
  std::unordered_set<std::string> joint_names;
  for (const auto& j : joints) {
    joint_names.insert(j.name);
  }
  for (std::size_t i = 0; i < sensors.size(); ++i) {
    const auto& s = sensors[i];
    if (!joint_names.contains(s.joint_name)) {
      return std::unexpected(
          mkerr(load_error_kind::reference_error,
                path,
                ptr_join(ptr_index("/sensors", i), "joint"),
                "sensor " + s.name + " references unknown joint '" + s.joint_name + "'"));
    }
  }
  return {};
}

std::expected<void, load_error> check_can_id_conflicts(const std::vector<motor>& motors,
                                                       const std::vector<sensor>& sensors,
                                                       const fs::path& path) {
  // (CAN ID -> "<motor|sensor>:<name>") map of the first device that claimed
  // the ID. Second offender wins on report.
  std::unordered_map<int, std::string> seen;
  for (std::size_t i = 0; i < motors.size(); ++i) {
    const auto& m = motors[i];
    auto [it, inserted] = seen.try_emplace(m.controller_can_id, m.name);
    if (!inserted) {
      return std::unexpected(mkerr(load_error_kind::conflict_error,
                                   path,
                                   ptr_join(ptr_index("/motors", i), "controller_can_id"),
                                   "duplicate CAN ID " + std::to_string(m.controller_can_id) +
                                       " between '" + it->second + "' and '" + m.name + "'"));
    }
  }
  for (std::size_t i = 0; i < sensors.size(); ++i) {
    const auto& s = sensors[i];
    auto [it, inserted] = seen.try_emplace(s.controller_can_id, s.name);
    if (!inserted) {
      return std::unexpected(mkerr(load_error_kind::conflict_error,
                                   path,
                                   ptr_join(ptr_index("/sensors", i), "controller_can_id"),
                                   "duplicate CAN ID " + std::to_string(s.controller_can_id) +
                                       " between '" + it->second + "' and '" + s.name + "'"));
    }
  }
  return {};
}

// ---------------------------------------------------------------------------
// Top-level orchestrator.
// ---------------------------------------------------------------------------

// Bundle of pointers/values produced by validate_root_shape, threaded into
// validate_and_build.
struct root_basics {
  int schema_version;
  std::string name;
  const json* links_arr;
  const json* joints_arr;
  const json* motors_arr;
  const json* sensors_arr;
};

// Validates the root object's shape: schema_version gate, required-fields
// presence + types, unknown-key rejection, and non-empty links/joints. Per-
// entity content is validated separately by the load_<kind> functions called
// from validate_and_build.
std::expected<root_basics, load_error> validate_root_shape(const json& root, const fs::path& path) {
  if (!root.is_object()) {
    return std::unexpected(
        mkerr(load_error_kind::schema_error, path, "", "root must be a JSON object"));
  }
  // Schema-version gate runs first (B1b: must fire before any other field
  // validation). Unsupported version → same kind/pointer as missing version,
  // with the offending number in the message.
  root_basics out{};
  if (auto r = require_integer(root, "schema_version", path, ""); r) {
    out.schema_version = *r;
  } else {
    return std::unexpected(r.error());
  }
  if (out.schema_version != 1) {
    return std::unexpected(
        mkerr(load_error_kind::schema_error,
              path,
              "/schema_version",
              "unsupported schema_version: " + std::to_string(out.schema_version)));
  }
  if (auto r = require_string(root, "name", path, ""); r) {
    out.name = std::move(*r);
  } else {
    return std::unexpected(r.error());
  }
  // The four required arrays. require_array reports schema_error pointing at
  // /<key> for missing or wrong-type — which is what C1's row for a missing
  // root key expects.
  for (auto [arr_dst, key] : std::initializer_list<std::pair<const json**, std::string_view>>{
           {&out.links_arr, "links"},
           {&out.joints_arr, "joints"},
           {&out.motors_arr, "motors"},
           {&out.sensors_arr, "sensors"}}) {
    if (auto r = require_array(root, key, path, ""); r) {
      *arr_dst = *r;
    } else {
      return std::unexpected(r.error());
    }
  }
  // Reject unknown root keys (M1 root case). After required-key checks so a
  // doc missing both a required and an unknown key reports the missing first.
  if (auto r = check_no_unknown_keys(root, k_root_keys, path, ""); !r) {
    return std::unexpected(r.error());
  }
  // I2 / I3: links and joints must be non-empty. (motors / sensors may be
  // empty per I1.)
  if (out.links_arr->empty()) {
    return std::unexpected(mkerr(load_error_kind::schema_error,
                                 path,
                                 "/links",
                                 "field links must contain at least one link"));
  }
  if (out.joints_arr->empty()) {
    return std::unexpected(mkerr(load_error_kind::schema_error,
                                 path,
                                 "/joints",
                                 "field joints must contain at least one joint"));
  }
  return out;
}

// Loads every element of `arr` via `loader`, then enforces per-kind name
// uniqueness. Returns the populated vector or the first error encountered
// (per second-offender rule, the loader and the uniqueness check both report
// the entity at index i that triggered the conflict).
template <typename Entity, typename Loader>
std::expected<std::vector<Entity>, load_error> load_array(const json& arr,
                                                          std::string_view kind_ptr_prefix,
                                                          const fs::path& path,
                                                          Loader loader) {
  std::vector<Entity> out;
  out.reserve(arr.size());
  for (std::size_t i = 0; i < arr.size(); ++i) {
    auto r = loader(arr[i], i, path);
    if (!r) {
      return std::unexpected(r.error());
    }
    out.push_back(std::move(*r));
  }
  if (auto u = check_unique_names(out, kind_ptr_prefix, path); !u) {
    return std::unexpected(u.error());
  }
  return out;
}

std::expected<robot_description, load_error> validate_and_build(const json& root,
                                                                const fs::path& path) {
  auto basics = validate_root_shape(root, path);
  if (!basics) {
    return std::unexpected(basics.error());
  }

  robot_description out;
  out.schema_version = basics->schema_version;
  out.name = std::move(basics->name);

  if (auto r = load_array<link>(*basics->links_arr, "/links", path, load_link); r) {
    out.links = std::move(*r);
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = load_array<joint>(*basics->joints_arr, "/joints", path, load_joint); r) {
    out.joints = std::move(*r);
  } else {
    return std::unexpected(r.error());
  }
  // Joint references resolved before motor/sensor loading so dangling joint
  // endpoints are reported with /joints/i/{parent,child} rather than surfacing
  // later as downstream confusion.
  if (auto r = check_joint_references(out.joints, out.links, path); !r) {
    return std::unexpected(r.error());
  }
  if (auto r = load_array<motor>(*basics->motors_arr, "/motors", path, load_motor); r) {
    out.motors = std::move(*r);
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = check_motor_joint_references(out.motors, out.joints, path); !r) {
    return std::unexpected(r.error());
  }
  if (auto r = load_array<sensor>(*basics->sensors_arr, "/sensors", path, load_sensor); r) {
    out.sensors = std::move(*r);
  } else {
    return std::unexpected(r.error());
  }
  if (auto r = check_sensor_joint_references(out.sensors, out.joints, path); !r) {
    return std::unexpected(r.error());
  }
  // CAN ID uniqueness across motors ∪ sensors (F1, F2, F3).
  if (auto r = check_can_id_conflicts(out.motors, out.sensors, path); !r) {
    return std::unexpected(r.error());
  }
  return out;
}

}  // namespace

std::expected<robot_description, load_error> load_from_file(const fs::path& path) {
  if (!fs::is_regular_file(path)) {
    return std::unexpected(
        mkerr(load_error_kind::file_not_found, path, "", "file not found: " + path.string()));
  }
  std::ifstream in(path);
  if (!in) {
    return std::unexpected(
        mkerr(load_error_kind::file_not_found, path, "", "could not open file: " + path.string()));
  }
  json parsed;
  try {
    in >> parsed;
  } catch (const json::parse_error& e) {
    return std::unexpected(mkerr(
        load_error_kind::parse_error, path, "", std::string("JSON parse error: ") + e.what()));
  }
  return validate_and_build(parsed, path);
}

}  // namespace robosim::description
