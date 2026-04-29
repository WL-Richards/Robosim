#include "error.h"
#include "loader.h"
#include "schema.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <array>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

namespace robosim::description {
namespace {

namespace fs = std::filesystem;
using nlohmann::json;
using ::testing::HasSubstr;

// ---------------------------------------------------------------------------
// Fixture path: production v0-arm.json. CMake passes the absolute path via a
// preprocessor define so tests don't rely on the working directory.
// ---------------------------------------------------------------------------

#ifndef ROBOSIM_V0_ARM_FIXTURE_PATH
#error "ROBOSIM_V0_ARM_FIXTURE_PATH not defined; CMake target setup is broken."
#endif

constexpr const char* kV0ArmFixturePath = ROBOSIM_V0_ARM_FIXTURE_PATH;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// The canonical v0 arm description, in memory. Every error test starts from
// this and applies a single delta. Kept in lockstep with descriptions/v0-arm.json.
json make_v0_arm_json() {
  return json::parse(R"({
    "schema_version": 1,
    "name": "v0-arm",
    "links": [
      { "name": "arm", "mass_kg": 2.0, "length_m": 0.5, "inertia_kgm2": 0.166 }
    ],
    "joints": [
      {
        "name": "shoulder",
        "type": "revolute",
        "parent": "world",
        "child": "arm",
        "axis": [0.0, 1.0, 0.0],
        "limit_lower_rad": -1.5708,
        "limit_upper_rad": 1.5708,
        "viscous_friction_nm_per_rad_per_s": 0.1
      }
    ],
    "motors": [
      {
        "name": "shoulder_motor",
        "motor_model": "kraken_x60",
        "controller": "talon_fx",
        "controller_can_id": 1,
        "controller_firmware_version": "TBD-pinned-at-v0-ship",
        "joint": "shoulder",
        "gear_ratio": 100.0
      }
    ],
    "sensors": [
      {
        "name": "shoulder_encoder",
        "sensor_model": "cancoder",
        "controller_can_id": 2,
        "controller_firmware_version": "TBD-pinned-at-v0-ship",
        "joint": "shoulder"
      }
    ]
  })");
}

// Erase the leaf key at `pointer` (whole keys only — never array elements,
// since removal would cascade-shift sibling indices and silently break the
// expected pointer of subsequent test cases).
void erase_at(json& j, std::string_view pointer) {
  const auto last_slash = pointer.find_last_of('/');
  ASSERT_NE(last_slash, std::string_view::npos)
      << "erase_at pointer must contain at least one '/': " << pointer;
  const std::string parent(pointer.substr(0, last_slash));
  const std::string leaf(pointer.substr(last_slash + 1));
  j.at(json::json_pointer(parent)).erase(leaf);
}

// Replace the value at `pointer` with `value`. Creates the path if missing
// (used by M1 to add unknown fields).
void set_at(json& j, std::string_view pointer, const json& value) {
  j[json::json_pointer(std::string(pointer))] = value;
}

// Per-test temp dir, RAII-cleaned. All "modified fixture" tests write here.
class LoaderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
    tmp_dir_ = fs::temp_directory_path() /
               (std::string("robosim_loader_test_") + info->test_suite_name() +
                "_" + info->name());
    fs::remove_all(tmp_dir_);
    fs::create_directories(tmp_dir_);
  }

  void TearDown() override {
    std::error_code ec;
    fs::remove_all(tmp_dir_, ec);
  }

  // Serialize j to a temp file and load it. Returns the result so the test
  // can assert on the success/error shape.
  std::expected<robot_description, load_error> load_json(const json& j,
                                                          std::string_view name = "robot.json") {
    const auto path = tmp_dir_ / std::string(name);
    std::ofstream out(path);
    out << j.dump(2);
    out.close();
    return load_from_file(path);
  }

  fs::path tmp_dir_;
};

// Cross-cutting error-shape assertion. Asserts:
//   1. result has no value
//   2. kind matches
//   3. json_pointer matches
//   (file_path is asserted separately by the caller because temp paths vary)
// Plus optional message-substring checks.
void expect_error_shape(const std::expected<robot_description, load_error>& result,
                        load_error_kind expected_kind,
                        std::string_view expected_pointer) {
  ASSERT_FALSE(result.has_value())
      << "expected error " << static_cast<int>(expected_kind)
      << " at " << expected_pointer << " but got success";
  EXPECT_EQ(result.error().kind, expected_kind);
  EXPECT_EQ(result.error().json_pointer, expected_pointer);
  EXPECT_FALSE(result.error().message.empty()) << "error message must be non-empty";
}

void expect_message_contains(const load_error& e, std::initializer_list<std::string_view> needles) {
  for (const auto needle : needles) {
    EXPECT_THAT(e.message, HasSubstr(std::string(needle)))
        << "error message should mention '" << needle << "': got '" << e.message << "'";
  }
}

// ===========================================================================
// A. Happy path
// ===========================================================================

TEST(RobotDescriptionLoader, A1_loads_minimal_valid_v0_arm) {
  const auto result = load_from_file(kV0ArmFixturePath);
  ASSERT_TRUE(result.has_value())
      << "load failed: " << (result ? "" : result.error().message);

  const auto& d = *result;
  EXPECT_EQ(d.schema_version, 1);
  EXPECT_EQ(d.name, "v0-arm");

  ASSERT_EQ(d.links.size(), 1U);
  EXPECT_EQ(d.links[0].name, "arm");
  // Exact FP equality: the v0 sketch numbers (2.0, 0.5, 0.166, 100.0, 0.1,
  // 1.5708) are all exactly representable as double, and nlohmann::json
  // round-trips representable JSON literals losslessly. Any deviation = bug.
  // Do NOT relax these to EXPECT_NEAR.
  EXPECT_EQ(d.links[0].mass_kg, 2.0);
  EXPECT_EQ(d.links[0].length_m, 0.5);
  EXPECT_EQ(d.links[0].inertia_kgm2, 0.166);

  ASSERT_EQ(d.joints.size(), 1U);
  EXPECT_EQ(d.joints[0].name, "shoulder");
  EXPECT_EQ(d.joints[0].type, joint_type::revolute);
  EXPECT_EQ(d.joints[0].parent, "world");
  EXPECT_EQ(d.joints[0].child, "arm");
  EXPECT_EQ(d.joints[0].axis, (std::array<double, 3>{0.0, 1.0, 0.0}));
  EXPECT_EQ(d.joints[0].limit_lower_rad, -1.5708);
  EXPECT_EQ(d.joints[0].limit_upper_rad, 1.5708);
  EXPECT_EQ(d.joints[0].viscous_friction_nm_per_rad_per_s, 0.1);

  ASSERT_EQ(d.motors.size(), 1U);
  EXPECT_EQ(d.motors[0].name, "shoulder_motor");
  EXPECT_EQ(d.motors[0].motor_model, "kraken_x60");
  EXPECT_EQ(d.motors[0].controller, "talon_fx");
  EXPECT_EQ(d.motors[0].controller_can_id, 1);
  EXPECT_EQ(d.motors[0].controller_firmware_version, "TBD-pinned-at-v0-ship");
  EXPECT_EQ(d.motors[0].joint_name, "shoulder");
  EXPECT_EQ(d.motors[0].gear_ratio, 100.0);

  ASSERT_EQ(d.sensors.size(), 1U);
  EXPECT_EQ(d.sensors[0].name, "shoulder_encoder");
  EXPECT_EQ(d.sensors[0].sensor_model, "cancoder");
  EXPECT_EQ(d.sensors[0].controller_can_id, 2);
  EXPECT_EQ(d.sensors[0].controller_firmware_version, "TBD-pinned-at-v0-ship");
  EXPECT_EQ(d.sensors[0].joint_name, "shoulder");
}

TEST_F(LoaderTest, A1b_accepts_json_integer_literal_in_float_field) {
  // Decision #6: JSON integer literals are accepted in float fields. mass_kg: 2
  // parses to 2.0.
  auto j = make_v0_arm_json();
  j["links"][0]["mass_kg"] = 2;  // JSON integer, not 2.0
  const auto result = load_json(j);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->links[0].mass_kg, 2.0);
}

// ===========================================================================
// B. Schema version
// ===========================================================================

TEST_F(LoaderTest, B1_rejects_when_schema_version_unsupported) {
  auto j = make_v0_arm_json();
  j["schema_version"] = 999;
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::schema_error, "/schema_version");
  expect_message_contains(result.error(), {"999"});
}

TEST_F(LoaderTest, B1b_schema_version_gate_runs_before_field_validation) {
  // The schema-version gate must fire before deeper field validation. A
  // document with both an unsupported version and a missing required field
  // should report the version error, not the missing-field error.
  auto j = make_v0_arm_json();
  j["schema_version"] = 999;
  erase_at(j, "/links/0/mass_kg");
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::schema_error, "/schema_version");
  expect_message_contains(result.error(), {"999"});
}

// ===========================================================================
// C. Required-field validation (parameterized)
// ===========================================================================

struct RequiredFieldCase {
  std::string field_path_to_remove;
  std::string expected_pointer;
};

class RequiredFieldTest : public LoaderTest, public ::testing::WithParamInterface<RequiredFieldCase> {};

TEST_P(RequiredFieldTest, C1_rejects_when_required_field_missing) {
  const auto& [field, ptr] = GetParam();
  auto j = make_v0_arm_json();
  erase_at(j, "/" + field);
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::schema_error, ptr);
  // The leaf field name appears in the message so the user can fix their file.
  const auto last_slash = field.find_last_of('/');
  const std::string leaf = (last_slash == std::string::npos) ? field : field.substr(last_slash + 1);
  expect_message_contains(result.error(), {leaf});
}

INSTANTIATE_TEST_SUITE_P(
    RequiredFields, RequiredFieldTest,
    ::testing::Values(
        RequiredFieldCase{"schema_version", "/schema_version"},
        RequiredFieldCase{"name", "/name"},
        RequiredFieldCase{"links", "/links"},
        RequiredFieldCase{"joints", "/joints"},
        RequiredFieldCase{"motors", "/motors"},
        RequiredFieldCase{"sensors", "/sensors"},
        RequiredFieldCase{"links/0/name", "/links/0/name"},
        RequiredFieldCase{"links/0/mass_kg", "/links/0/mass_kg"},
        RequiredFieldCase{"links/0/length_m", "/links/0/length_m"},
        RequiredFieldCase{"links/0/inertia_kgm2", "/links/0/inertia_kgm2"},
        RequiredFieldCase{"joints/0/name", "/joints/0/name"},
        RequiredFieldCase{"joints/0/type", "/joints/0/type"},
        RequiredFieldCase{"joints/0/parent", "/joints/0/parent"},
        RequiredFieldCase{"joints/0/child", "/joints/0/child"},
        RequiredFieldCase{"joints/0/axis", "/joints/0/axis"},
        RequiredFieldCase{"joints/0/limit_lower_rad", "/joints/0/limit_lower_rad"},
        RequiredFieldCase{"joints/0/limit_upper_rad", "/joints/0/limit_upper_rad"},
        RequiredFieldCase{"joints/0/viscous_friction_nm_per_rad_per_s",
                          "/joints/0/viscous_friction_nm_per_rad_per_s"},
        RequiredFieldCase{"motors/0/name", "/motors/0/name"},
        RequiredFieldCase{"motors/0/motor_model", "/motors/0/motor_model"},
        RequiredFieldCase{"motors/0/controller", "/motors/0/controller"},
        RequiredFieldCase{"motors/0/controller_can_id", "/motors/0/controller_can_id"},
        RequiredFieldCase{"motors/0/controller_firmware_version",
                          "/motors/0/controller_firmware_version"},
        RequiredFieldCase{"motors/0/joint", "/motors/0/joint"},
        RequiredFieldCase{"motors/0/gear_ratio", "/motors/0/gear_ratio"},
        RequiredFieldCase{"sensors/0/name", "/sensors/0/name"},
        RequiredFieldCase{"sensors/0/sensor_model", "/sensors/0/sensor_model"},
        RequiredFieldCase{"sensors/0/controller_can_id", "/sensors/0/controller_can_id"},
        RequiredFieldCase{"sensors/0/controller_firmware_version",
                          "/sensors/0/controller_firmware_version"},
        RequiredFieldCase{"sensors/0/joint", "/sensors/0/joint"}));

// ===========================================================================
// D. Type-mismatch validation (parameterized)
// ===========================================================================

struct TypeMismatchCase {
  std::string field_path;
  json replacement;
  std::string expected_pointer;
};

class TypeMismatchTest : public LoaderTest, public ::testing::WithParamInterface<TypeMismatchCase> {};

TEST_P(TypeMismatchTest, D1_rejects_when_field_type_mismatched) {
  const auto& [field, replacement, ptr] = GetParam();
  auto j = make_v0_arm_json();
  set_at(j, "/" + field, replacement);
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::schema_error, ptr);
  const auto last_slash = field.find_last_of('/');
  const std::string leaf = (last_slash == std::string::npos) ? field : field.substr(last_slash + 1);
  expect_message_contains(result.error(), {leaf});
}

INSTANTIATE_TEST_SUITE_P(
    TypeMismatches, TypeMismatchTest,
    ::testing::Values(
        TypeMismatchCase{"schema_version", 1.0, "/schema_version"},
        TypeMismatchCase{"links/0/mass_kg", "two", "/links/0/mass_kg"},
        TypeMismatchCase{"joints/0/axis", 1.0, "/joints/0/axis"},
        TypeMismatchCase{"joints/0/axis", "y", "/joints/0/axis"},
        TypeMismatchCase{"joints/0/axis", json::array({1.0, 0.0}), "/joints/0/axis"},
        TypeMismatchCase{"motors/0/controller_can_id", "1", "/motors/0/controller_can_id"},
        TypeMismatchCase{"motors/0/controller_can_id", 1.5, "/motors/0/controller_can_id"},
        TypeMismatchCase{"motors/0/controller_can_id", 1.0, "/motors/0/controller_can_id"},
        TypeMismatchCase{"motors/0/gear_ratio", true, "/motors/0/gear_ratio"}));

// ===========================================================================
// E. Reference integrity
// ===========================================================================

TEST_F(LoaderTest, E1_rejects_motor_referencing_unknown_joint) {
  auto j = make_v0_arm_json();
  j["motors"][0]["joint"] = "no_such_joint";
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::reference_error, "/motors/0/joint");
  expect_message_contains(result.error(), {"shoulder_motor", "no_such_joint"});
}

TEST_F(LoaderTest, E2_rejects_sensor_referencing_unknown_joint) {
  auto j = make_v0_arm_json();
  j["sensors"][0]["joint"] = "no_such_joint";
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::reference_error, "/sensors/0/joint");
  expect_message_contains(result.error(), {"shoulder_encoder", "no_such_joint"});
}

TEST_F(LoaderTest, E3_rejects_joint_referencing_unknown_link_via_child) {
  auto j = make_v0_arm_json();
  j["joints"][0]["child"] = "no_such_link";
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::reference_error, "/joints/0/child");
  expect_message_contains(result.error(), {"shoulder", "no_such_link"});
}

TEST_F(LoaderTest, E3b_rejects_joint_referencing_unknown_link_via_parent) {
  // Build a fixture with a second link "forearm" and a second joint "elbow"
  // whose parent is dangling. The distinct joint name avoids tripping the
  // duplicate-name check (decision #11) before the reference check fires.
  auto j = make_v0_arm_json();
  j["links"].push_back({
      {"name", "forearm"}, {"mass_kg", 1.0}, {"length_m", 0.3}, {"inertia_kgm2", 0.05}});
  j["joints"].push_back({
      {"name", "elbow"},
      {"type", "revolute"},
      {"parent", "no_such_link"},
      {"child", "forearm"},
      {"axis", json::array({0.0, 1.0, 0.0})},
      {"limit_lower_rad", -1.0},
      {"limit_upper_rad", 1.0},
      {"viscous_friction_nm_per_rad_per_s", 0.05}});
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::reference_error, "/joints/1/parent");
  expect_message_contains(result.error(), {"elbow", "no_such_link"});
}

TEST(RobotDescriptionLoader, E4_accepts_world_as_special_parent_in_joint) {
  // Coverage-only: pins the magic-string contract against a future "tighten
  // everything" refactor that drops the special case. A1 already exercises
  // this implicitly.
  const auto result = load_from_file(kV0ArmFixturePath);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->joints[0].parent, "world");
}

TEST_F(LoaderTest, E4b_rejects_world_as_joint_child) {
  auto j = make_v0_arm_json();
  j["joints"][0]["child"] = "world";
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::reference_error, "/joints/0/child");
  expect_message_contains(result.error(), {"world", "shoulder"});
}

// ===========================================================================
// F. Conflicts (per decision #2: validator reports the second offender)
// ===========================================================================

TEST_F(LoaderTest, F1_rejects_two_motors_with_same_can_id) {
  auto j = make_v0_arm_json();
  j["motors"].push_back({
      {"name", "shoulder_motor_2"},
      {"motor_model", "kraken_x60"},
      {"controller", "talon_fx"},
      {"controller_can_id", 1},  // duplicate
      {"controller_firmware_version", "TBD-pinned-at-v0-ship"},
      {"joint", "shoulder"},
      {"gear_ratio", 100.0}});
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::conflict_error, "/motors/1/controller_can_id");
  expect_message_contains(result.error(), {"shoulder_motor", "shoulder_motor_2", "1"});
}

TEST_F(LoaderTest, F2_rejects_motor_and_sensor_with_same_can_id) {
  auto j = make_v0_arm_json();
  j["sensors"][0]["controller_can_id"] = 1;  // matches the motor's ID
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::conflict_error, "/sensors/0/controller_can_id");
  expect_message_contains(result.error(), {"shoulder_motor", "shoulder_encoder", "1"});
}

TEST_F(LoaderTest, F3_rejects_two_sensors_with_same_can_id) {
  auto j = make_v0_arm_json();
  j["sensors"].push_back({
      {"name", "shoulder_encoder_2"},
      {"sensor_model", "cancoder"},
      {"controller_can_id", 2},  // duplicate of sensors[0]
      {"controller_firmware_version", "TBD-pinned-at-v0-ship"},
      {"joint", "shoulder"}});
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::conflict_error, "/sensors/1/controller_can_id");
  expect_message_contains(result.error(), {"shoulder_encoder", "shoulder_encoder_2", "2"});
}

// ===========================================================================
// G. File and JSON-level errors
// ===========================================================================

TEST_F(LoaderTest, G1_rejects_when_file_not_found) {
  const auto missing = tmp_dir_ / "definitely_not_here.json";
  ASSERT_FALSE(fs::exists(missing));
  const auto result = load_from_file(missing);
  expect_error_shape(result, load_error_kind::file_not_found, "");
  EXPECT_EQ(result.error().file_path, missing);
}

struct MalformedJsonCase {
  std::string contents;
};

class MalformedJsonTest : public LoaderTest, public ::testing::WithParamInterface<MalformedJsonCase> {};

TEST_P(MalformedJsonTest, G2_rejects_when_json_syntactically_invalid) {
  const auto path = tmp_dir_ / "bad.json";
  std::ofstream out(path);
  out << GetParam().contents;
  out.close();
  const auto result = load_from_file(path);
  expect_error_shape(result, load_error_kind::parse_error, "");
  EXPECT_EQ(result.error().file_path, path);
}

INSTANTIATE_TEST_SUITE_P(
    MalformedJson, MalformedJsonTest,
    ::testing::Values(MalformedJsonCase{"{"},                  // single-byte truncation
                      MalformedJsonCase{R"({"name": })"}));    // missing value, multi-char

// ===========================================================================
// H. Determinism
// ===========================================================================

TEST(RobotDescriptionLoader, H1_loads_deterministically) {
  // Project non-negotiable #5: same input -> same output. 8 loads (rather than
  // 2) shakes out hash-iteration variance some implementations introduce after
  // a warm-up period.
  std::vector<robot_description> loads;
  loads.reserve(8);
  for (int i = 0; i < 8; ++i) {
    auto result = load_from_file(kV0ArmFixturePath);
    ASSERT_TRUE(result.has_value()) << "load " << i << " failed";
    loads.push_back(*std::move(result));
  }
  for (std::size_t i = 1; i < loads.size(); ++i) {
    EXPECT_EQ(loads[0], loads[i]) << "load " << i << " differs from load 0";
  }
}

TEST_F(LoaderTest, H2_error_reporting_is_deterministic) {
  // Pins decision #2 from the validator side: same invalid document, same
  // reported error every time. Uses the F2 fixture (motor + sensor sharing
  // CAN ID 1).
  auto j = make_v0_arm_json();
  j["sensors"][0]["controller_can_id"] = 1;
  const auto path = tmp_dir_ / "robot.json";
  std::ofstream out(path);
  out << j.dump(2);
  out.close();

  std::vector<load_error> errors;
  errors.reserve(8);
  for (int i = 0; i < 8; ++i) {
    auto result = load_from_file(path);
    ASSERT_FALSE(result.has_value()) << "load " << i << " unexpectedly succeeded";
    errors.push_back(result.error());
  }
  for (std::size_t i = 1; i < errors.size(); ++i) {
    EXPECT_EQ(errors[0].kind, errors[i].kind);
    EXPECT_EQ(errors[0].json_pointer, errors[i].json_pointer);
  }
}

// ===========================================================================
// I. Empty / missing arrays
// ===========================================================================

TEST_F(LoaderTest, I1_accepts_description_with_empty_motors_and_sensors) {
  auto j = make_v0_arm_json();
  j["motors"] = json::array();
  j["sensors"] = json::array();
  const auto result = load_json(j);
  ASSERT_TRUE(result.has_value()) << result.error().message;
  EXPECT_TRUE(result->motors.empty());
  EXPECT_TRUE(result->sensors.empty());
}

TEST_F(LoaderTest, I2_rejects_when_links_empty) {
  auto j = make_v0_arm_json();
  j["links"] = json::array();
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::schema_error, "/links");
  expect_message_contains(result.error(), {"links"});
}

TEST_F(LoaderTest, I3_rejects_when_joints_empty) {
  auto j = make_v0_arm_json();
  j["joints"] = json::array();
  // Empty motors/sensors so reference checks don't fire first.
  j["motors"] = json::array();
  j["sensors"] = json::array();
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::schema_error, "/joints");
  expect_message_contains(result.error(), {"joints"});
}

// ===========================================================================
// J. Domain-range validation
// ===========================================================================

struct DomainCase {
  std::string field_path;
  json replacement;
  std::string expected_pointer;
};

class DomainTest : public LoaderTest, public ::testing::WithParamInterface<DomainCase> {};

TEST_P(DomainTest, J1_rejects_when_value_outside_allowed_range) {
  const auto& [field, replacement, ptr] = GetParam();
  auto j = make_v0_arm_json();
  set_at(j, "/" + field, replacement);
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::domain_error, ptr);
  const auto last_slash = field.find_last_of('/');
  const std::string leaf = (last_slash == std::string::npos) ? field : field.substr(last_slash + 1);
  expect_message_contains(result.error(), {leaf});
}

INSTANTIATE_TEST_SUITE_P(
    DomainRanges, DomainTest,
    ::testing::Values(
        DomainCase{"links/0/mass_kg", -2.0, "/links/0/mass_kg"},
        DomainCase{"links/0/mass_kg", 0.0, "/links/0/mass_kg"},
        DomainCase{"links/0/inertia_kgm2", 0.0, "/links/0/inertia_kgm2"},
        DomainCase{"links/0/length_m", -0.5, "/links/0/length_m"},
        DomainCase{"motors/0/controller_can_id", -1, "/motors/0/controller_can_id"},
        DomainCase{"motors/0/controller_can_id", 64, "/motors/0/controller_can_id"},
        DomainCase{"motors/0/gear_ratio", 0.0, "/motors/0/gear_ratio"},
        DomainCase{"motors/0/gear_ratio", -1.0, "/motors/0/gear_ratio"}));

struct CanIdBoundaryCase {
  int can_id;
};

class CanIdBoundaryTest : public LoaderTest,
                          public ::testing::WithParamInterface<CanIdBoundaryCase> {};

TEST_P(CanIdBoundaryTest, J1b_accepts_can_id_boundaries) {
  // Pins decision #10 from the positive side: 0 and 63 are both inside the
  // inclusive range and accepted.
  auto j = make_v0_arm_json();
  j["motors"][0]["controller_can_id"] = GetParam().can_id;
  // Make sure the sensor's CAN ID doesn't accidentally collide.
  j["sensors"][0]["controller_can_id"] = (GetParam().can_id == 2) ? 5 : 2;
  const auto result = load_json(j);
  ASSERT_TRUE(result.has_value()) << result.error().message;
  EXPECT_EQ(result->motors[0].controller_can_id, GetParam().can_id);
}

INSTANTIATE_TEST_SUITE_P(
    CanIdBoundaries, CanIdBoundaryTest,
    ::testing::Values(CanIdBoundaryCase{0}, CanIdBoundaryCase{63}));

TEST_F(LoaderTest, J2_accepts_equal_joint_limits) {
  // Decision #7 case A: lower == upper is a deliberate fixed-joint declaration.
  auto j = make_v0_arm_json();
  j["joints"][0]["limit_lower_rad"] = 0.5;
  j["joints"][0]["limit_upper_rad"] = 0.5;
  const auto result = load_json(j);
  ASSERT_TRUE(result.has_value()) << result.error().message;
  EXPECT_EQ(result->joints[0].limit_lower_rad, 0.5);
  EXPECT_EQ(result->joints[0].limit_upper_rad, 0.5);
}

TEST_F(LoaderTest, J2_rejects_inverted_joint_limits) {
  // Decision #7 case B: lower > upper is a typo. Per second-offender rule,
  // pointer is /limit_upper_rad (parsed after lower).
  auto j = make_v0_arm_json();
  j["joints"][0]["limit_lower_rad"] = 1.0;
  j["joints"][0]["limit_upper_rad"] = -1.0;
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::domain_error, "/joints/0/limit_upper_rad");
  expect_message_contains(result.error(), {"shoulder"});
}

// ===========================================================================
// K. Duplicate names (per-kind uniqueness; decision #11)
// ===========================================================================

TEST_F(LoaderTest, K1_rejects_duplicate_link_name) {
  // Add a second link with duplicate name "arm". Add a second joint that
  // references it consistently so no reference error fires first.
  auto j = make_v0_arm_json();
  j["links"].push_back({
      {"name", "arm"},  // duplicate
      {"mass_kg", 1.0},
      {"length_m", 0.3},
      {"inertia_kgm2", 0.05}});
  // The duplicate-link check fires before any further validation.
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::conflict_error, "/links/1/name");
  expect_message_contains(result.error(), {"arm"});
}

TEST_F(LoaderTest, K1_rejects_duplicate_joint_name) {
  // Two joints named "shoulder", with otherwise valid distinct geometry.
  auto j = make_v0_arm_json();
  j["links"].push_back({
      {"name", "forearm"}, {"mass_kg", 1.0}, {"length_m", 0.3}, {"inertia_kgm2", 0.05}});
  j["joints"].push_back({
      {"name", "shoulder"},  // duplicate
      {"type", "revolute"},
      {"parent", "arm"},
      {"child", "forearm"},
      {"axis", json::array({0.0, 1.0, 0.0})},
      {"limit_lower_rad", -1.0},
      {"limit_upper_rad", 1.0},
      {"viscous_friction_nm_per_rad_per_s", 0.05}});
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::conflict_error, "/joints/1/name");
  expect_message_contains(result.error(), {"shoulder"});
}

TEST_F(LoaderTest, K1_rejects_duplicate_motor_name) {
  // Distinct CAN ID so the duplicate-name conflict is the only conflict.
  auto j = make_v0_arm_json();
  j["motors"].push_back({
      {"name", "shoulder_motor"},  // duplicate
      {"motor_model", "kraken_x60"},
      {"controller", "talon_fx"},
      {"controller_can_id", 3},  // distinct from existing 1 and 2
      {"controller_firmware_version", "TBD-pinned-at-v0-ship"},
      {"joint", "shoulder"},
      {"gear_ratio", 100.0}});
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::conflict_error, "/motors/1/name");
  expect_message_contains(result.error(), {"shoulder_motor"});
}

TEST_F(LoaderTest, K1_rejects_duplicate_sensor_name) {
  auto j = make_v0_arm_json();
  j["sensors"].push_back({
      {"name", "shoulder_encoder"},  // duplicate
      {"sensor_model", "cancoder"},
      {"controller_can_id", 4},  // distinct
      {"controller_firmware_version", "TBD-pinned-at-v0-ship"},
      {"joint", "shoulder"}});
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::conflict_error, "/sensors/1/name");
  expect_message_contains(result.error(), {"shoulder_encoder"});
}

// ===========================================================================
// L. Enum-like field strictness (case-sensitive; decision #12)
// ===========================================================================

TEST_F(LoaderTest, L1_rejects_when_enum_field_has_wrong_case) {
  auto j = make_v0_arm_json();
  j["joints"][0]["type"] = "Revolute";
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::schema_error, "/joints/0/type");
  expect_message_contains(result.error(), {"Revolute"});
}

TEST_F(LoaderTest, L2_rejects_when_enum_field_has_unknown_value) {
  auto j = make_v0_arm_json();
  j["joints"][0]["type"] = "wibble";
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::schema_error, "/joints/0/type");
  expect_message_contains(result.error(), {"wibble"});
}

// ===========================================================================
// M. Unknown-field rejection (decision #8 — strongest "no shortcuts" guard)
// ===========================================================================

struct UnknownFieldCase {
  std::string parent_pointer;  // "" for root, "/links/0" for a link
  std::string unknown_key;
  std::string expected_pointer;
};

class UnknownFieldTest : public LoaderTest, public ::testing::WithParamInterface<UnknownFieldCase> {};

TEST_P(UnknownFieldTest, M1_rejects_when_unknown_field_present) {
  const auto& [parent, key, ptr] = GetParam();
  auto j = make_v0_arm_json();
  if (parent.empty()) {
    j[key] = "x";
  } else {
    j[json::json_pointer(parent)][key] = "x";
  }
  const auto result = load_json(j);
  expect_error_shape(result, load_error_kind::schema_error, ptr);
  expect_message_contains(result.error(), {key});
}

INSTANTIATE_TEST_SUITE_P(
    UnknownFields, UnknownFieldTest,
    ::testing::Values(UnknownFieldCase{"", "extra_root_key", "/extra_root_key"},
                      UnknownFieldCase{"/links/0", "mass_g", "/links/0/mass_g"},
                      UnknownFieldCase{"/joints/0", "axisz", "/joints/0/axisz"},
                      UnknownFieldCase{"/motors/0", "gear_ration", "/motors/0/gear_ration"},
                      UnknownFieldCase{"/sensors/0", "cancoder_thing",
                                       "/sensors/0/cancoder_thing"}));

}  // namespace
}  // namespace robosim::description
