#pragma once

// Session wrapper around a loaded robot_description: tracks the
// source path and a dirty bit, and centralizes save / reload through
// description::save_to_file / load_from_file.
//
// See tests/viz/TEST_PLAN_VD.md "D. edit_session dirty bit + reload"
// and "S. Save flow" for the pinned contract.
//
// The "Discard changes?" modal is the *caller's* affordance — the
// session does not gate reload on dirty. The caller (main.cpp) checks
// session.dirty before invoking reload_session and shows the modal in
// ImGui.

#include "description/error.h"
#include "description/schema.h"
#include "description/serializer.h"

#include <expected>
#include <filesystem>

namespace robosim::viz {

struct edit_session {
  description::robot_description description;
  std::filesystem::path source_path;
  bool dirty = false;
};

[[nodiscard]] std::expected<edit_session, description::load_error>
load_session(const std::filesystem::path& path);

[[nodiscard]] std::expected<void, description::save_error>
save_session(edit_session& session);

[[nodiscard]] std::expected<void, description::load_error>
reload_session(edit_session& session);

}  // namespace robosim::viz
