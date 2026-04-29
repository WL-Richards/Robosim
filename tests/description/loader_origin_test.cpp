// Phase VC v2 schema + origin tests.
//
// Implements V0/V0b/V1/V1b/V2/V3/V4/V4b/V5/V6/V6b/V6c/V7/V7b/V8/V9/V10
// from tests/description/TEST_PLAN_VC.md.
//
// All tests are in the RobotDescriptionLoader suite, alongside the v1
// loader_test.cpp tests, per the cross-cutting assertion shape pinned
// in the v1 plan and extended in TEST_PLAN_VC.md.

#include "loader.h"
#include "origin_pose.h"

#include "error.h"
#include "schema.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <array>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numbers>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace robosim::description {
namespace {

namespace fs = std::filesystem;
using nlohmann::json;
using ::testing::HasSubstr;

#ifndef ROBOSIM_V0_ARM_V1_FIXTURE_PATH
#error "ROBOSIM_V0_ARM_V1_FIXTURE_PATH not defined; CMake target setup is broken."
#endif

constexpr const char* v0_arm_v1_fixture_path = ROBOSIM_V0_ARM_V1_FIXTURE_PATH;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Canonical v2 fixture as json. Identity origins (per the
// D-VC-9 omission rule, v0-arm.json's intent: no origin keys present).
json make_v0_arm_v2_json() {
  return json::parse(R"({
    "schema_version": 2,
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

// Same as the v1 helper in loader_test.cpp, kept locally so V1/V1b
// can derive from a v1-shaped fixture without depending on translation
// units in the other test file.
json make_v0_arm_v1_json() {
  json j = make_v0_arm_v2_json();
  j["schema_version"] = 1;
  return j;
}

json make_identity_origin() {
  json o;
  o["xyz_m"] = {0.0, 0.0, 0.0};
  o["rpy_rad"] = {0.0, 0.0, 0.0};
  return o;
}

// Hand-built reference for V0: the shape we expect after loading the
// frozen v1 fixture through the v2 loader. Origin members default to
// origin_pose{} (identity).
robot_description make_v0_arm_v1_reference() {
  robot_description d;
  d.schema_version = 1;
  d.name = "v0-arm";
  d.links = {
      link{"arm", 2.0, 0.5, 0.166, origin_pose{}},
  };
  d.joints = {
      joint{
          "shoulder", joint_type::revolute, "world", "arm",
          {0.0, 1.0, 0.0}, -1.5708, 1.5708, 0.1, origin_pose{},
      },
  };
  d.motors = {
      motor{
          "shoulder_motor", "kraken_x60", "talon_fx", 1,
          "TBD-pinned-at-v0-ship", "shoulder", 100.0,
      },
  };
  d.sensors = {
      sensor{
          "shoulder_encoder", "cancoder", 2,
          "TBD-pinned-at-v0-ship", "shoulder",
      },
  };
  return d;
}

void erase_at(json& j, std::string_view pointer) {
  const auto last_slash = pointer.find_last_of('/');
  ASSERT_NE(last_slash, std::string_view::npos)
      << "erase_at pointer must contain at least one '/': " << pointer;
  const std::string parent(pointer.substr(0, last_slash));
  const std::string leaf(pointer.substr(last_slash + 1));
  // Use operator[] with a typed json_pointer rather than at(): nlohmann
  // 3.11.2+ deprecates the templated at<KeyType>() path that clang
  // resolves for json_pointer arguments under -Wdeprecated-declarations.
  const json::json_pointer ptr(parent);
  j[ptr].erase(leaf);
}

void set_at(json& j, std::string_view pointer, const json& value) {
  // See erase_at note: nlohmann's templated key-based operator[]
  // resolves through a deprecated path on lvalue json_pointer
  // arguments under -Wdeprecated-declarations.
  const json::json_pointer ptr(std::string{pointer});
  j[ptr] = value;
}

class loader_v2_test : public ::testing::Test {
 protected:
  void SetUp() override {
    const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
    tmp_dir_ = fs::temp_directory_path() /
               (std::string("robosim_loader_v2_test_") + info->test_suite_name() + "_" +
                info->name());
    fs::remove_all(tmp_dir_);
    fs::create_directories(tmp_dir_);
  }

  void TearDown() override {
    std::error_code ec;
    fs::remove_all(tmp_dir_, ec);
  }

  std::expected<robot_description, load_error> load_json(
      const json& j, std::string_view name = "robot.json") {
    const auto path = tmp_dir_ / std::string(name);
    std::ofstream out(path);
    out << j.dump(2);
    out.close();
    return load_from_file(path);
  }

  fs::path tmp_dir_;
};

void expect_error_shape(const std::expected<robot_description, load_error>& result,
                        load_error_kind expected_kind,
                        std::string_view expected_pointer) {
  ASSERT_FALSE(result.has_value())
      << "expected error " << static_cast<int>(expected_kind) << " at " << expected_pointer
      << " but got success";
  EXPECT_EQ(result.error().kind, expected_kind);
  EXPECT_EQ(result.error().json_pointer, expected_pointer);
  EXPECT_FALSE(result.error().message.empty()) << "error message must be non-empty";
}

void expect_message_contains(const load_error& e, std::string_view needle) {
  EXPECT_THAT(e.message, HasSubstr(std::string(needle)))
      << "error message should mention '" << needle << "': got '" << e.message << "'";
}

// ===========================================================================
// V0 / V0b — v1 forward-compatibility under the v2 loader
// ===========================================================================

TEST(RobotDescriptionLoader, V0_loads_v1_arm_unchanged_under_v2_loader) {
  const auto result = load_from_file(v0_arm_v1_fixture_path);
  ASSERT_TRUE(result.has_value()) << "load failed: " << (result ? "" : result.error().message);

  EXPECT_EQ(*result, make_v0_arm_v1_reference());
}

class V0bMissingV1Leaf
    : public loader_v2_test,
      public ::testing::WithParamInterface<std::pair<std::string_view, std::string_view>> {};

TEST_P(V0bMissingV1Leaf, V0b_v2_missing_required_v1_leaf_is_schema_error) {
  const auto& [pointer, expected_substring] = GetParam();
  auto j = make_v0_arm_v2_json();
  erase_at(j, pointer);

  const auto result = load_json(j);

  expect_error_shape(result, load_error_kind::schema_error, pointer);
  expect_message_contains(result.error(), expected_substring);
}

INSTANTIATE_TEST_SUITE_P(
    RepresentativeV1Leaves, V0bMissingV1Leaf,
    ::testing::Values(
        std::pair<std::string_view, std::string_view>{"/links/0/mass_kg", "mass_kg"},
        std::pair<std::string_view, std::string_view>{
            "/joints/0/viscous_friction_nm_per_rad_per_s",
            "viscous_friction_nm_per_rad_per_s"},
        std::pair<std::string_view, std::string_view>{
            "/motors/0/controller_can_id", "controller_can_id"},
        std::pair<std::string_view, std::string_view>{"/motors", "motors"}));

// ===========================================================================
// V1 / V1b — v2-era keys rejected under schema_version: 1
// ===========================================================================

TEST_F(loader_v2_test, V1_schema_version_1_rejects_origin_key) {
  auto j = make_v0_arm_v1_json();
  set_at(j, "/joints/0/origin", make_identity_origin());

  const auto result = load_json(j);

  expect_error_shape(result, load_error_kind::schema_error, "/joints/0/origin");
}

TEST_F(loader_v2_test, V1b_schema_version_1_rejects_visual_origin_key) {
  auto j = make_v0_arm_v1_json();
  set_at(j, "/links/0/visual_origin", make_identity_origin());

  const auto result = load_json(j);

  expect_error_shape(result, load_error_kind::schema_error, "/links/0/visual_origin");
}

// ===========================================================================
// V2 / V3 — identity defaults, absent vs. explicit-identity equality
// ===========================================================================

TEST_F(loader_v2_test, V2_loads_identity_when_origin_keys_absent) {
  const auto result = load_json(make_v0_arm_v2_json());
  ASSERT_TRUE(result.has_value()) << result.error().message;

  for (const auto& l : result->links) {
    EXPECT_EQ(l.visual_origin.xyz_m, (std::array{0.0, 0.0, 0.0}));
    EXPECT_EQ(l.visual_origin.rpy_rad, (std::array{0.0, 0.0, 0.0}));
  }
  for (const auto& jn : result->joints) {
    EXPECT_EQ(jn.origin.xyz_m, (std::array{0.0, 0.0, 0.0}));
    EXPECT_EQ(jn.origin.rpy_rad, (std::array{0.0, 0.0, 0.0}));
  }
}

TEST_F(loader_v2_test, V3_absent_and_explicit_identity_origins_compare_equal) {
  const auto absent = load_json(make_v0_arm_v2_json(), "absent.json");
  ASSERT_TRUE(absent.has_value()) << absent.error().message;

  auto j_explicit = make_v0_arm_v2_json();
  set_at(j_explicit, "/joints/0/origin", make_identity_origin());
  set_at(j_explicit, "/links/0/visual_origin", make_identity_origin());
  const auto explicit_id = load_json(j_explicit, "explicit.json");
  ASSERT_TRUE(explicit_id.has_value()) << explicit_id.error().message;

  EXPECT_EQ(*absent, *explicit_id);
}

// ===========================================================================
// V4 / V4b — raw propagation without composition
// ===========================================================================

TEST_F(loader_v2_test, V4_joint_origin_xyz_translation_is_propagated_raw) {
  auto j = make_v0_arm_v2_json();
  json o;
  o["xyz_m"] = {0.1, 0.2, 0.3};
  o["rpy_rad"] = {0.0, 0.0, 0.0};
  set_at(j, "/joints/0/origin", o);

  const auto result = load_json(j);
  ASSERT_TRUE(result.has_value()) << result.error().message;

  EXPECT_EQ(result->joints[0].origin.xyz_m, (std::array{0.1, 0.2, 0.3}));
  EXPECT_EQ(result->joints[0].origin.rpy_rad, (std::array{0.0, 0.0, 0.0}));
}

TEST_F(loader_v2_test, V4b_link_visual_origin_propagated_raw) {
  auto j = make_v0_arm_v2_json();
  json o;
  o["xyz_m"] = {-0.4, 0.5, -0.6};
  o["rpy_rad"] = {0.0, 0.0, 0.0};
  set_at(j, "/links/0/visual_origin", o);

  const auto result = load_json(j);
  ASSERT_TRUE(result.has_value()) << result.error().message;

  EXPECT_EQ(result->links[0].visual_origin.xyz_m, (std::array{-0.4, 0.5, -0.6}));
  EXPECT_EQ(result->links[0].visual_origin.rpy_rad, (std::array{0.0, 0.0, 0.0}));
}

// ===========================================================================
// V5 — Euler-convention discriminator (intrinsic-Z-Y'-X'' / URDF order)
// ===========================================================================
//
// rpy_rad = [roll, pitch, yaw] per D-VC-5a.
// R = R_z(yaw) * R_y(pitch) * R_x(roll) applied to column vectors.
//
// This test does NOT load JSON; it asserts on compose_origin directly,
// the loader's exposed composition seam.

namespace {

constexpr double kComposeTolerance = 1e-9;

// Apply a 4x4 transform (col-major, m[col][row]) to a homogeneous
// column vector [x, y, z, 1] and return the (x, y, z) of the result.
std::array<double, 3> apply_to_point(const transform_4x4& m,
                                     const std::array<double, 3>& v) {
  std::array<double, 3> r{};
  for (std::size_t row = 0; row < 3; ++row) {
    double s = m[3][row];  // translation column
    for (std::size_t col = 0; col < 3; ++col) {
      s += m[col][row] * v[col];
    }
    r[row] = s;
  }
  return r;
}

void expect_vec_near(const std::array<double, 3>& got,
                     const std::array<double, 3>& want, double tol) {
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(got[i], want[i], tol)
        << "component " << i << ": got " << got[i] << " want " << want[i];
  }
}

}  // namespace

// Case A: rpy = [pi/2, pi/2, 0]. Apply to [1,0,0]^T:
//   R_x(pi/2) * [1,0,0]^T = [1,0,0]^T
//   R_y(pi/2) * [1,0,0]^T = [0,0,-1]^T
//   R_z(0) * [0,0,-1]^T = [0,0,-1]^T
// Naive R_x*R_y*R_z would give [0,1,0]^T (debugging aid).
TEST(RobotDescriptionLoader, V5a_compose_origin_pins_zyx_for_roll_then_pitch) {
  const double pi = std::numbers::pi;
  const origin_pose p{{0.0, 0.0, 0.0}, {pi / 2.0, pi / 2.0, 0.0}};
  const auto v_out = apply_to_point(compose_origin(p), {1.0, 0.0, 0.0});
  expect_vec_near(v_out, {0.0, 0.0, -1.0}, kComposeTolerance);
}

// Case B: rpy = [0, pi/2, pi/2]. Apply to [0,1,0]^T:
//   R_x(0) * [0,1,0]^T = [0,1,0]^T
//   R_y(pi/2) * [0,1,0]^T = [0,1,0]^T (Y-axis fixed under Y-rotation)
//   R_z(pi/2) * [0,1,0]^T = [-1,0,0]^T
// Naive R_x*R_y*R_z would give [0,0,1]^T.
TEST(RobotDescriptionLoader, V5b_compose_origin_pins_zyx_for_pitch_then_yaw) {
  const double pi = std::numbers::pi;
  const origin_pose p{{0.0, 0.0, 0.0}, {0.0, pi / 2.0, pi / 2.0}};
  const auto v_out = apply_to_point(compose_origin(p), {0.0, 1.0, 0.0});
  expect_vec_near(v_out, {-1.0, 0.0, 0.0}, kComposeTolerance);
}

// Case C: rpy = [-pi/2, -pi/2, 0]. Apply to [1,0,0]^T:
//   R_x(-pi/2) * [1,0,0]^T = [1,0,0]^T
//   R_y(-pi/2) * [1,0,0]^T = [0,0,1]^T
//   R_z(0) * [0,0,1]^T = [0,0,1]^T
// Pins sign symmetry (case A's negation lands on the +Z mirror).
TEST(RobotDescriptionLoader, V5c_compose_origin_pins_zyx_under_negative_angles) {
  const double pi = std::numbers::pi;
  const origin_pose p{{0.0, 0.0, 0.0}, {-pi / 2.0, -pi / 2.0, 0.0}};
  const auto v_out = apply_to_point(compose_origin(p), {1.0, 0.0, 0.0});
  expect_vec_near(v_out, {0.0, 0.0, 1.0}, kComposeTolerance);
}

// Case D: rpy = [pi/2, 0, 0] (roll only). Apply to [0,1,0]^T:
//   R_x(pi/2) * [0,1,0]^T = [0,0,1]^T (rotation about X sends +Y to +Z).
// Index-swap impl mapping rpy[0] to yaw would compute
//   R_z(pi/2) * [0,1,0]^T = [-1,0,0]^T, distinct from the correct answer.
TEST(RobotDescriptionLoader, V5d_compose_origin_pins_array_index_layout) {
  const double pi = std::numbers::pi;
  const origin_pose p{{0.0, 0.0, 0.0}, {pi / 2.0, 0.0, 0.0}};
  const auto v_out = apply_to_point(compose_origin(p), {0.0, 1.0, 0.0});
  expect_vec_near(v_out, {0.0, 0.0, 1.0}, kComposeTolerance);
}

// ===========================================================================
// V6 / V6b / V6c — partial origin → schema_error
// ===========================================================================

TEST_F(loader_v2_test, V6_partial_origin_missing_xyz_is_schema_error) {
  auto j = make_v0_arm_v2_json();
  json o;
  o["rpy_rad"] = {0.0, 0.0, 0.0};
  set_at(j, "/joints/0/origin", o);

  const auto result = load_json(j);

  expect_error_shape(result, load_error_kind::schema_error, "/joints/0/origin/xyz_m");
  expect_message_contains(result.error(), "xyz_m");
}

TEST_F(loader_v2_test, V6b_partial_origin_missing_rpy_is_schema_error) {
  auto j = make_v0_arm_v2_json();
  json o;
  o["xyz_m"] = {0.0, 0.0, 0.0};
  set_at(j, "/joints/0/origin", o);

  const auto result = load_json(j);

  expect_error_shape(result, load_error_kind::schema_error, "/joints/0/origin/rpy_rad");
  expect_message_contains(result.error(), "rpy_rad");
}

TEST_F(loader_v2_test, V6c_empty_origin_object_is_schema_error_at_declaration_first_field) {
  auto j = make_v0_arm_v2_json();
  set_at(j, "/joints/0/origin", json::object());

  const auto result = load_json(j);

  // xyz_m precedes rpy_rad in origin_pose's declaration order, so
  // it's the deterministically-reported missing field.
  expect_error_shape(result, load_error_kind::schema_error, "/joints/0/origin/xyz_m");
  expect_message_contains(result.error(), "xyz_m");
}

// ===========================================================================
// V7 / V7b — non-finite component domain_error at the validator seam
// ===========================================================================
//
// Per D-VC-3 + the rev-3 reviewer finding: nlohmann v3.12's strict
// parser rejects NaN/Infinity/-Infinity literals, so we cannot
// exercise this contract via load_from_file. Instead we call the
// per-scalar validator helper directly. The seam call-site pin is
// enforced separately by cmake/lint_loader_finite_seam.cmake.

class V7NonFiniteXyz
    : public ::testing::TestWithParam<std::tuple<int, double>> {};

TEST_P(V7NonFiniteXyz, V7_non_finite_xyz_component_is_domain_error) {
  const auto [slot, bad_value] = GetParam();
  std::array<double, 3> xyz{0.0, 0.0, 0.0};
  xyz[static_cast<std::size_t>(slot)] = bad_value;

  const auto result = validate_finite_xyz(xyz, "/joints/0/origin/xyz_m");

  ASSERT_TRUE(result.has_value()) << "expected domain_error, got std::nullopt";
  EXPECT_EQ(result->kind, load_error_kind::domain_error);
  EXPECT_EQ(result->json_pointer,
            std::string("/joints/0/origin/xyz_m/") + std::to_string(slot));
  // file_path is filled by the caller, not the helper (cross-cutting
  // assertion shape — pinned in the V7 plan).
  EXPECT_TRUE(result->file_path.empty());
  EXPECT_THAT(result->message, HasSubstr("xyz_m"));
}

INSTANTIATE_TEST_SUITE_P(
    NonFiniteValuesAtEachSlot, V7NonFiniteXyz,
    ::testing::Combine(
        ::testing::Values(0, 1, 2),
        ::testing::Values(std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::infinity(),
                          -std::numeric_limits<double>::infinity())));

class V7bNonFiniteRpy
    : public ::testing::TestWithParam<std::tuple<int, double>> {};

TEST_P(V7bNonFiniteRpy, V7b_non_finite_rpy_component_is_domain_error) {
  const auto [slot, bad_value] = GetParam();
  std::array<double, 3> rpy{0.0, 0.0, 0.0};
  rpy[static_cast<std::size_t>(slot)] = bad_value;

  const auto result = validate_finite_rpy(rpy, "/joints/0/origin/rpy_rad");

  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->kind, load_error_kind::domain_error);
  EXPECT_EQ(result->json_pointer,
            std::string("/joints/0/origin/rpy_rad/") + std::to_string(slot));
  EXPECT_TRUE(result->file_path.empty());
  EXPECT_THAT(result->message, HasSubstr("rpy_rad"));
}

INSTANTIATE_TEST_SUITE_P(
    NonFiniteValuesAtEachSlot, V7bNonFiniteRpy,
    ::testing::Combine(
        ::testing::Values(0, 1, 2),
        ::testing::Values(std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::infinity(),
                          -std::numeric_limits<double>::infinity())));

TEST(RobotDescriptionLoader, V7_finite_xyz_returns_nullopt) {
  EXPECT_FALSE(validate_finite_xyz({0.0, 0.0, 0.0}, "/anywhere").has_value());
  EXPECT_FALSE(validate_finite_xyz({1.5, -2.0, 3.5}, "/anywhere").has_value());
}

TEST(RobotDescriptionLoader, V7b_finite_rpy_returns_nullopt) {
  EXPECT_FALSE(validate_finite_rpy({0.0, 0.0, 0.0}, "/anywhere").has_value());
  EXPECT_FALSE(validate_finite_rpy({1.5, -2.0, 3.5}, "/anywhere").has_value());
}

// ===========================================================================
// V8 — wrong-shape origin field (parameterized)
// ===========================================================================

struct V8Case {
  std::string_view label;
  std::string_view pointer_to_set;
  json bad_value;
  std::string_view expected_pointer;
  std::string_view message_substring;
};

class V8WrongShape : public loader_v2_test, public ::testing::WithParamInterface<V8Case> {};

TEST_P(V8WrongShape, V8_wrong_shape_origin_field_is_schema_error) {
  const auto& c = GetParam();
  auto j = make_v0_arm_v2_json();
  set_at(j, c.pointer_to_set, c.bad_value);

  const auto result = load_json(j);

  expect_error_shape(result, load_error_kind::schema_error, c.expected_pointer);
  expect_message_contains(result.error(), c.message_substring);
}

INSTANTIATE_TEST_SUITE_P(
    InnerArrayAndOuterContainer, V8WrongShape,
    ::testing::Values(
        V8Case{"xyz_arity_2", "/joints/0/origin",
               json{{"xyz_m", {0.0, 0.0}}, {"rpy_rad", {0.0, 0.0, 0.0}}},
               "/joints/0/origin/xyz_m", "xyz_m"},
        V8Case{"xyz_arity_4", "/joints/0/origin",
               json{{"xyz_m", {0.0, 0.0, 0.0, 0.0}},
                            {"rpy_rad", {0.0, 0.0, 0.0}}},
               "/joints/0/origin/xyz_m", "xyz_m"},
        V8Case{"xyz_empty", "/joints/0/origin",
               json{{"xyz_m", json::array()},
                            {"rpy_rad", {0.0, 0.0, 0.0}}},
               "/joints/0/origin/xyz_m", "xyz_m"},
        V8Case{"xyz_element_string_at_slot_1", "/joints/0/origin",
               json{{"xyz_m", {0.0, "x", 0.0}},
                            {"rpy_rad", {0.0, 0.0, 0.0}}},
               "/joints/0/origin/xyz_m/1", "xyz_m"},
        V8Case{"rpy_arity_2", "/joints/0/origin",
               json{{"xyz_m", {0.0, 0.0, 0.0}}, {"rpy_rad", {0.0, 0.0}}},
               "/joints/0/origin/rpy_rad", "rpy_rad"},
        V8Case{"rpy_arity_4", "/joints/0/origin",
               json{{"xyz_m", {0.0, 0.0, 0.0}},
                            {"rpy_rad", {0.0, 0.0, 0.0, 0.0}}},
               "/joints/0/origin/rpy_rad", "rpy_rad"},
        V8Case{"rpy_empty", "/joints/0/origin",
               json{{"xyz_m", {0.0, 0.0, 0.0}},
                            {"rpy_rad", json::array()}},
               "/joints/0/origin/rpy_rad", "rpy_rad"},
        V8Case{"origin_number", "/joints/0/origin", json(5),
               "/joints/0/origin", "origin"},
        V8Case{"origin_string", "/joints/0/origin", json("identity"),
               "/joints/0/origin", "origin"},
        V8Case{"origin_array", "/joints/0/origin",
               json{1, 2, 3}, "/joints/0/origin", "origin"},
        V8Case{"origin_null", "/joints/0/origin", json(nullptr),
               "/joints/0/origin", "origin"},
        V8Case{"visual_origin_number", "/links/0/visual_origin", json(5),
               "/links/0/visual_origin", "visual_origin"}));

// ===========================================================================
// V9 — subnormals load successfully (positive corner)
// ===========================================================================

TEST_F(loader_v2_test, V9_accepts_subnormal_finite_xyz_value) {
  const double subnormal = std::numeric_limits<double>::denorm_min();
  ASSERT_TRUE(std::isfinite(subnormal));  // sanity for the test setup

  auto j = make_v0_arm_v2_json();
  json o;
  o["xyz_m"] = {subnormal, 0.0, 0.0};
  o["rpy_rad"] = {0.0, 0.0, 0.0};
  set_at(j, "/joints/0/origin", o);

  const auto result = load_json(j);
  ASSERT_TRUE(result.has_value()) << result.error().message;

  EXPECT_EQ(result->joints[0].origin.xyz_m[0], subnormal);
}

// ===========================================================================
// V10 — cross-reference under v2
// ===========================================================================

TEST_F(loader_v2_test, V10_v2_file_with_dangling_motor_joint_reference_returns_reference_error) {
  auto j = make_v0_arm_v2_json();
  set_at(j, "/motors/0/joint", json("nonexistent"));

  const auto result = load_json(j);

  expect_error_shape(result, load_error_kind::reference_error, "/motors/0/joint");
  expect_message_contains(result.error(), "nonexistent");
}

}  // namespace
}  // namespace robosim::description
