// Phase VD section F — end-to-end load → apply → rebuild → save → reload.
// See tests/viz/TEST_PLAN_VD.md "F. Fixture-driven end-to-end".

#include "fixtures.h"
#include "viz/edit_mode_apply.h"
#include "viz/edit_mode_builder.h"
#include "viz/edit_session.h"
#include "viz/scene_snapshot.h"

#include "description/origin_pose.h"
#include "description/schema.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <string>

namespace robosim::viz {
namespace {

namespace desc = robosim::description;
namespace fs = std::filesystem;

class TempFile {
 public:
  TempFile() {
    const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
    path_ = fs::temp_directory_path() /
            (std::string("vd_e2e_") + info->test_suite_name() + "_" +
             info->name() + ".json");
    fs::remove(path_);
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

transform from_4x4(const desc::transform_4x4& m) {
  transform t;
  t.m = m;
  return t;
}

TEST(VdEndToEnd,
     ApplyThenRebuildThenSaveThenReloadYieldsRoundTripEquality) {
  // Step 1 — load v0-arm into a session via a temp file (so the test
  // does not mutate the production fixture).
  TempFile tmp;
  ASSERT_TRUE(
      desc::save_to_file(testing::make_v0_arm_description(), tmp.path())
          .has_value());

  auto session = load_session(tmp.path()).value();

  // Step 2 — build snapshot from session.
  const auto snapshot = build_edit_mode_snapshot(session.description);
  const auto shoulder_idx =
      testing::find_node_index(snapshot, node_kind::joint, "shoulder");

  // Step 3 — non-trivial target (rotation + translation).
  const auto target = from_4x4(desc::compose_origin(
      {.xyz_m = {0.125, 0.25, 0.5},
       .rpy_rad = {+0.1, +0.2, +0.3}}));

  // Step 4 — apply.
  session.description = apply_gizmo_target(
      session.description, snapshot, shoulder_idx, target);
  session.dirty = true;

  // Step 5 — rebuild snapshot, assert apply round-trips through builder.
  const auto snapshot_after_apply =
      build_edit_mode_snapshot(session.description);
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      EXPECT_NEAR(snapshot_after_apply.nodes[shoulder_idx]
                      .world_from_local.m[col][row],
                  target.m[col][row],
                  1e-9)
          << "step-5 mismatch at m[" << col << "][" << row << "]";
    }
  }

  // Step 6 — save.
  ASSERT_TRUE(save_session(session).has_value());
  EXPECT_FALSE(session.dirty);

  // Step 7 — reload via a fresh session; description bit-equal.
  const auto session2 = load_session(tmp.path()).value();
  EXPECT_EQ(session2.description, session.description);

  // Step 8 — snapshot bit-equal.
  const auto snapshot2 = build_edit_mode_snapshot(session2.description);
  EXPECT_EQ(snapshot2, snapshot_after_apply);
}

}  // namespace
}  // namespace robosim::viz
