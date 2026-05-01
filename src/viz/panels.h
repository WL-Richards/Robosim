#pragma once

// ImGui panels for Edit mode: scene tree, inspector, status bar.
//
// Reads from a loaded `robot_description` for inspector field display
// (Edit-mode-only concern; Live / Replay modes use a different source
// at the same UI surface). The renderer-isolation rule (TEST_PLAN T1)
// applies to renderer.cpp specifically — panels are allow-listed.

#include "scene_snapshot.h"

#include "description/error.h"
#include "description/schema.h"

#include <expected>
#include <optional>
#include <string>

namespace robosim::viz {

struct panels_state {
  // Loaded description (Edit mode source of truth). nullopt if loading
  // failed — error message goes in `load_error_message`.
  std::optional<robosim::description::robot_description> description;
  bool description_dirty = false;
  std::string load_error_message;
  std::string source_path_display;
};

// Draws the scene tree, inspector, and status bar inside the current
// ImGui frame. May mutate `s.selected_index` if the user clicks an
// entry in the scene tree.
void draw_panels(panels_state& state, scene_snapshot& s);

}  // namespace robosim::viz
