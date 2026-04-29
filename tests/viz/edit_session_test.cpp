// Phase VD sections D + S — edit_session dirty bit, save / reload,
// save-flow content checks. See tests/viz/TEST_PLAN_VD.md sections D
// and S.

#include "viz/edit_session.h"

#include "fixtures.h"
#include "viz/edit_mode_apply.h"
#include "viz/edit_mode_builder.h"
#include "viz/scene_snapshot.h"

#include "description/error.h"
#include "description/loader.h"
#include "description/origin_pose.h"
#include "description/schema.h"
#include "description/serializer.h"

#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <string>

namespace robosim::viz {
namespace {

namespace desc = robosim::description;
namespace fs = std::filesystem;

// Per-test temp file path derived from the running GoogleTest test
// info, so parallel runs don't collide. RAII cleanup ensures the file
// goes away even on test failure.
class TempFile {
 public:
  explicit TempFile(std::string_view suffix = "") {
    const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
    std::string name = std::string("vd_test_") + info->test_suite_name() + "_" +
                       info->name();
    if (!suffix.empty()) {
      name += "_";
      name += suffix;
    }
    name += ".json";
    // GoogleTest parameterized suite/test names contain '/' (e.g.
    // "Suite/Param.test/0"), which would create non-existent
    // intermediate directories under temp_directory_path. Flatten
    // them to underscores.
    for (auto& c : name) {
      if (c == '/') c = '_';
    }
    path_ = fs::temp_directory_path() / name;
    fs::remove(path_);  // start clean
  }
  TempFile(const TempFile&) = delete;
  TempFile& operator=(const TempFile&) = delete;
  ~TempFile() {
    std::error_code ec;
    fs::remove(path_, ec);
  }
  const fs::path& path() const { return path_; }

 private:
  fs::path path_;
};

// Write a v0-arm fixture to the given path so we can load it through
// load_session. Tests don't go through ROBOSIM_V0_ARM_FIXTURE_PATH
// directly because they need to mutate the on-disk file.
void write_v0_arm_to(const fs::path& path) {
  ASSERT_TRUE(desc::save_to_file(testing::make_v0_arm_description(), path)
                  .has_value());
}

transform make_translation_transform(double x, double y, double z) {
  transform t;
  t.m[0][0] = 1.0;
  t.m[1][1] = 1.0;
  t.m[2][2] = 1.0;
  t.m[3][3] = 1.0;
  t.m[3][0] = x;
  t.m[3][1] = y;
  t.m[3][2] = z;
  return t;
}

transform from_4x4(const desc::transform_4x4& m) {
  transform t;
  t.m = m;
  return t;
}

// ---------------------------------------------------------------------
// D0 — load_session initializes dirty=false, source_path set.
// ---------------------------------------------------------------------

TEST(EditSession, LoadInitializesDirtyFalseAndPathSet) {
  TempFile tmp;
  write_v0_arm_to(tmp.path());

  auto session = load_session(tmp.path());
  ASSERT_TRUE(session.has_value());

  EXPECT_FALSE(session->dirty);
  EXPECT_EQ(session->source_path, tmp.path());
  EXPECT_EQ(session->description.name, "v0-arm");
}

// ---------------------------------------------------------------------
// D0b — load failure propagates load_error.
// ---------------------------------------------------------------------

TEST(EditSession, LoadFailurePropagatesLoadError) {
  const fs::path bogus = "/nonexistent/vd_test/path.json";
  auto result = load_session(bogus);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().kind, desc::load_error_kind::file_not_found);
  EXPECT_EQ(result.error().file_path, bogus);
}

// ---------------------------------------------------------------------
// D1 — save clears dirty bit on success.
// ---------------------------------------------------------------------

TEST(EditSession, SaveClearsDirtyBitOnSuccess) {
  TempFile tmp;
  write_v0_arm_to(tmp.path());
  auto session = load_session(tmp.path()).value();

  session.description.joints[0].origin =
      desc::origin_pose{.xyz_m = {0.5, 0.0, 0.0}, .rpy_rad = {0.0, 0.0, 0.0}};
  session.dirty = true;

  ASSERT_TRUE(save_session(session).has_value());

  EXPECT_FALSE(session.dirty);
  EXPECT_EQ(session.source_path, tmp.path());

  const auto reloaded = desc::load_from_file(tmp.path());
  ASSERT_TRUE(reloaded.has_value());
  EXPECT_EQ(*reloaded, session.description);
}

// ---------------------------------------------------------------------
// D2 — reload replaces description and clears dirty bit.
// ---------------------------------------------------------------------

TEST(EditSession, ReloadReplacesDescriptionAndClearsDirtyBit) {
  TempFile tmp;
  write_v0_arm_to(tmp.path());
  auto session = load_session(tmp.path()).value();

  session.description.name = "DIRTY";
  session.dirty = true;

  ASSERT_TRUE(reload_session(session).has_value());

  EXPECT_FALSE(session.dirty);
  EXPECT_EQ(session.description.name, "v0-arm");

  const auto on_disk = desc::load_from_file(tmp.path());
  ASSERT_TRUE(on_disk.has_value());
  EXPECT_EQ(session.description, *on_disk);
}

// ---------------------------------------------------------------------
// D3 — repeated save/reload alternates dirty deterministically.
// ---------------------------------------------------------------------

TEST(EditSession, RepeatedSaveAndReloadAlternatesDirtyBitDeterministically) {
  TempFile tmp;
  write_v0_arm_to(tmp.path());
  auto session = load_session(tmp.path()).value();

  // Step 2.
  session.description.joints[0].origin.xyz_m[0] = 0.5;  // representable
  session.dirty = true;
  const auto desc_after_first_apply = session.description;
  ASSERT_TRUE(save_session(session).has_value());
  EXPECT_FALSE(session.dirty);
  EXPECT_EQ(session.description, desc_after_first_apply);

  // Step 3.
  session.description.joints[0].origin.xyz_m[0] = 0.25;
  session.dirty = true;
  ASSERT_TRUE(reload_session(session).has_value());
  EXPECT_FALSE(session.dirty);
  EXPECT_EQ(session.description, desc_after_first_apply);

  // Step 4.
  session.description.joints[0].origin.xyz_m[0] = 0.125;
  session.dirty = true;
  const auto desc_after_third_apply = session.description;
  ASSERT_TRUE(save_session(session).has_value());
  EXPECT_FALSE(session.dirty);
  ASSERT_TRUE(reload_session(session).has_value());
  EXPECT_FALSE(session.dirty);
  EXPECT_EQ(session.description, desc_after_third_apply);
}

// ---------------------------------------------------------------------
// D4 — Save As (mutate source_path then save) redirects output.
// ---------------------------------------------------------------------

TEST(EditSession, SaveAsByMutatingSourcePathRedirectsOutputAndClearsDirty) {
  TempFile path_a("a");
  write_v0_arm_to(path_a.path());

  auto session = load_session(path_a.path()).value();
  session.description.joints[0].origin.xyz_m[0] = 0.5;
  session.dirty = true;

  // Capture the original on-disk bytes before redirecting Save.
  std::ifstream a_in(path_a.path(), std::ios::binary);
  const std::string original_a_bytes((std::istreambuf_iterator<char>(a_in)),
                                     std::istreambuf_iterator<char>());

  TempFile path_b("b");
  session.source_path = path_b.path();
  ASSERT_TRUE(save_session(session).has_value());

  EXPECT_FALSE(session.dirty);
  EXPECT_TRUE(fs::exists(path_b.path()));

  std::ifstream a_in2(path_a.path(), std::ios::binary);
  const std::string a_after_bytes((std::istreambuf_iterator<char>(a_in2)),
                                  std::istreambuf_iterator<char>());
  EXPECT_EQ(original_a_bytes, a_after_bytes);
}

// ---------------------------------------------------------------------
// D5b — clean-state reload-failure preserves not-dirty.
// ---------------------------------------------------------------------

TEST(EditSession, ReloadFailureFromCleanStatePreservesNotDirty) {
  TempFile tmp;
  write_v0_arm_to(tmp.path());
  auto session = load_session(tmp.path()).value();
  ASSERT_FALSE(session.dirty);
  const auto desc_clean = session.description;

  // Corrupt the file.
  {
    std::ofstream out(tmp.path());
    out << "{ malformed";
  }

  auto result = reload_session(session);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().kind, desc::load_error_kind::parse_error);
  EXPECT_FALSE(session.dirty);
  EXPECT_EQ(session.description, desc_clean);
  EXPECT_EQ(session.source_path, tmp.path());
}

// ---------------------------------------------------------------------
// D5 — reload failure leaves session unchanged (dirty stays true).
// ---------------------------------------------------------------------

TEST(EditSession, ReloadFailureLeavesSessionUnchanged) {
  TempFile tmp;
  write_v0_arm_to(tmp.path());
  auto session = load_session(tmp.path()).value();

  session.description.name = "MUTATED";
  session.dirty = true;
  const auto desc_dirty = session.description;

  // Corrupt.
  {
    std::ofstream out(tmp.path());
    out << "{ malformed";
  }

  auto result = reload_session(session);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().kind, desc::load_error_kind::parse_error);
  EXPECT_EQ(session.description, desc_dirty);
  EXPECT_EQ(session.source_path, tmp.path());
  EXPECT_TRUE(session.dirty);
}

// ---------------------------------------------------------------------
// S1 — save after apply round-trips byte-identically.
// ---------------------------------------------------------------------

TEST(EditSession, SaveAfterApplyRoundTripsToByteIdenticalFile) {
  TempFile path_a("a");
  TempFile path_b("b");
  write_v0_arm_to(path_a.path());
  auto session = load_session(path_a.path()).value();

  const auto snapshot = build_edit_mode_snapshot(session.description);
  const auto shoulder_idx =
      testing::find_node_index(snapshot, node_kind::joint, "shoulder");
  const auto target = make_translation_transform(0.125, 0.25, 0.5);
  session.description = apply_gizmo_target(
      session.description, snapshot, shoulder_idx, target);
  session.dirty = true;

  ASSERT_TRUE(save_session(session).has_value());

  auto session2 = load_session(path_a.path()).value();
  session2.source_path = path_b.path();
  ASSERT_TRUE(save_session(session2).has_value());

  std::ifstream a_in(path_a.path(), std::ios::binary);
  const std::string a_bytes((std::istreambuf_iterator<char>(a_in)),
                            std::istreambuf_iterator<char>());
  std::ifstream b_in(path_b.path(), std::ios::binary);
  const std::string b_bytes((std::istreambuf_iterator<char>(b_in)),
                            std::istreambuf_iterator<char>());
  EXPECT_EQ(a_bytes, b_bytes);
}

// ---------------------------------------------------------------------
// S2 — identity no-op apply omits origin keys on save.
// ---------------------------------------------------------------------

TEST(EditSession, IdentityNoOpApplyOmitsOriginKeysOnSave) {
  TempFile tmp;
  write_v0_arm_to(tmp.path());
  auto session = load_session(tmp.path()).value();

  const auto snapshot = build_edit_mode_snapshot(session.description);
  const auto shoulder_idx =
      testing::find_node_index(snapshot, node_kind::joint, "shoulder");
  const auto target = snapshot.nodes[shoulder_idx].world_from_local;
  session.description = apply_gizmo_target(
      session.description, snapshot, shoulder_idx, target);
  session.dirty = true;

  ASSERT_TRUE(save_session(session).has_value());

  std::ifstream in(tmp.path());
  const auto parsed = nlohmann::ordered_json::parse(in);
  EXPECT_FALSE(parsed["joints"][0].contains("origin"));
  EXPECT_FALSE(parsed["links"][0].contains("visual_origin"));
}

// ---------------------------------------------------------------------
// S3 — non-identity apply on joint emits origin key on save.
// ---------------------------------------------------------------------

TEST(EditSession, NonIdentityApplyEmitsOriginKeysOnSave) {
  TempFile tmp;
  write_v0_arm_to(tmp.path());
  auto session = load_session(tmp.path()).value();

  const auto snapshot = build_edit_mode_snapshot(session.description);
  const auto shoulder_idx =
      testing::find_node_index(snapshot, node_kind::joint, "shoulder");
  const auto target = make_translation_transform(0.5, 0.0, 0.0);
  session.description = apply_gizmo_target(
      session.description, snapshot, shoulder_idx, target);
  session.dirty = true;

  ASSERT_TRUE(save_session(session).has_value());

  std::ifstream in(tmp.path());
  const auto parsed = nlohmann::ordered_json::parse(in);
  ASSERT_TRUE(parsed["joints"][0].contains("origin"));
  EXPECT_EQ(parsed["joints"][0]["origin"]["xyz_m"],
            (nlohmann::json::array({0.5, 0.0, 0.0})));
  EXPECT_EQ(parsed["joints"][0]["origin"]["rpy_rad"],
            (nlohmann::json::array({0.0, 0.0, 0.0})));
}

// ---------------------------------------------------------------------
// S3b — non-identity apply on link emits visual_origin (not joint origin).
// ---------------------------------------------------------------------

TEST(EditSession,
     NonIdentityApplyOnLinkEmitsVisualOriginAndOmitsJointOrigin) {
  TempFile tmp;
  write_v0_arm_to(tmp.path());
  auto session = load_session(tmp.path()).value();

  const auto snapshot = build_edit_mode_snapshot(session.description);
  const auto arm_idx =
      testing::find_node_index(snapshot, node_kind::link, "arm");
  const auto target = make_translation_transform(0.5, 0.0, 0.0);
  session.description = apply_gizmo_target(
      session.description, snapshot, arm_idx, target);
  session.dirty = true;

  ASSERT_TRUE(save_session(session).has_value());

  std::ifstream in(tmp.path());
  const auto parsed = nlohmann::ordered_json::parse(in);
  ASSERT_TRUE(parsed["links"][0].contains("visual_origin"));
  EXPECT_EQ(parsed["links"][0]["visual_origin"]["xyz_m"],
            (nlohmann::json::array({0.5, 0.0, 0.0})));
  EXPECT_EQ(parsed["links"][0]["visual_origin"]["rpy_rad"],
            (nlohmann::json::array({0.0, 0.0, 0.0})));
  EXPECT_FALSE(parsed["joints"][0].contains("origin"));
}

// ---------------------------------------------------------------------
// S4 — save failure retains dirty and propagates error.
// ---------------------------------------------------------------------

TEST(EditSession, SaveFailureRetainsDirtyAndPropagatesError) {
  TempFile seed;
  write_v0_arm_to(seed.path());
  auto session = load_session(seed.path()).value();

  // Make a chmod-restricted directory; redirect save into it.
  const auto unwritable_dir =
      fs::temp_directory_path() / "vd_test_save_failure_dir";
  fs::remove_all(unwritable_dir);
  fs::create_directory(unwritable_dir);
  fs::permissions(unwritable_dir,
                  fs::perms::owner_read | fs::perms::owner_exec);

  session.description.joints[0].origin.xyz_m[0] = 0.5;
  session.dirty = true;
  session.source_path = unwritable_dir / "out.json";

  auto result = save_session(session);
  EXPECT_FALSE(result.has_value());
  if (!result.has_value()) {
    EXPECT_EQ(result.error().kind, desc::save_error_kind::io_error);
  }
  EXPECT_TRUE(session.dirty);
  EXPECT_EQ(session.source_path, unwritable_dir / "out.json");

  // Restore and clean up.
  fs::permissions(unwritable_dir,
                  fs::perms::owner_read | fs::perms::owner_write |
                      fs::perms::owner_exec);
  fs::remove_all(unwritable_dir);
}

// ---------------------------------------------------------------------
// S5 — adversarial translation targets round-trip bit-equal (parameterized).
// ---------------------------------------------------------------------

class S5AdversarialTranslation
    : public ::testing::TestWithParam<double> {};

TEST_P(S5AdversarialTranslation,
       ApplyAtAdversarialTranslationTargetsRoundTripsBitEqualThroughSave) {
  TempFile tmp;
  write_v0_arm_to(tmp.path());
  auto session = load_session(tmp.path()).value();

  const auto snapshot = build_edit_mode_snapshot(session.description);
  const auto shoulder_idx =
      testing::find_node_index(snapshot, node_kind::joint, "shoulder");

  const double value = GetParam();
  const auto target = make_translation_transform(value, 0.0, 0.0);

  session.description = apply_gizmo_target(
      session.description, snapshot, shoulder_idx, target);
  session.dirty = true;
  ASSERT_TRUE(save_session(session).has_value());

  const auto reloaded = desc::load_from_file(tmp.path());
  ASSERT_TRUE(reloaded.has_value());

  const std::uint64_t original_bits = std::bit_cast<std::uint64_t>(value);
  const std::uint64_t reloaded_bits =
      std::bit_cast<std::uint64_t>(reloaded->joints[0].origin.xyz_m[0]);
  EXPECT_EQ(original_bits, reloaded_bits)
      << "value " << value << " did not round-trip bit-equal";
}

INSTANTIATE_TEST_SUITE_P(
    AdversarialFloats,
    S5AdversarialTranslation,
    ::testing::Values(
        std::numeric_limits<double>::denorm_min(),
        std::numeric_limits<double>::min(),
        std::nextafter(1.0, 2.0),
        std::nextafter(1.0, 0.0),
        0.1,
        1e-300));

// ---------------------------------------------------------------------
// S5b — adversarial rotation targets round-trip on recomposed transform.
// ---------------------------------------------------------------------

class S5bAdversarialRotation : public ::testing::TestWithParam<double> {};

TEST_P(S5bAdversarialRotation,
       ApplyAtAdversarialRotationTargetsRoundTripsRecomposedTransformThroughSave) {
  TempFile tmp;
  write_v0_arm_to(tmp.path());
  auto session = load_session(tmp.path()).value();

  const auto snapshot = build_edit_mode_snapshot(session.description);
  const auto shoulder_idx =
      testing::find_node_index(snapshot, node_kind::joint, "shoulder");

  const double roll = GetParam();
  const auto target_4x4 = desc::compose_origin(
      {.xyz_m = {0.0, 0.0, 0.0}, .rpy_rad = {roll, 0.0, 0.0}});
  const auto target = from_4x4(target_4x4);

  session.description = apply_gizmo_target(
      session.description, snapshot, shoulder_idx, target);
  session.dirty = true;
  ASSERT_TRUE(save_session(session).has_value());

  const auto reloaded = desc::load_from_file(tmp.path());
  ASSERT_TRUE(reloaded.has_value());

  const auto m_round =
      desc::compose_origin(reloaded->joints[0].origin);
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      EXPECT_NEAR(target_4x4[col][row], m_round[col][row], 1e-9)
          << "mismatch at m[" << col << "][" << row << "]";
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
    AdversarialRolls,
    S5bAdversarialRotation,
    ::testing::Values(
        +1e-12,
        +M_PI / 1000.0,
        +M_PI / 4.0,
        +M_PI / 2.0 - 1e-3));

}  // namespace
}  // namespace robosim::viz
