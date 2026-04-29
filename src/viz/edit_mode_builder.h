#pragma once

// Edit-mode snapshot builder — robot_description → scene_snapshot.
//
// This is the only file in src/viz/ permitted (alongside the panels and
// CLI entry points) to depend on `description::`. The renderer must not.
// See cmake/lint_renderer_isolation.cmake.

#include "scene_snapshot.h"

#include "description/schema.h"

namespace robosim::viz {

[[nodiscard]] scene_snapshot build_edit_mode_snapshot(
    const robosim::description::robot_description& desc);

}  // namespace robosim::viz
