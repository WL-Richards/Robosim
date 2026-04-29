#pragma once

// Ray-vs-primitive picking against a scene snapshot.
//
// Half-line semantics (t >= 0); closed-shape primitives. See
// tests/viz/TEST_PLAN.md section P for the pinned contract.

#include "scene_snapshot.h"

#include <array>
#include <cstddef>
#include <optional>

namespace robosim::viz {

struct ray {
  std::array<double, 3> origin_world;
  std::array<double, 3> direction_world;  // not required to be unit
};

struct pick_hit {
  std::size_t node_index;
  double t_along_ray;  // origin + t * direction = hit point; t >= 0
};

[[nodiscard]] std::optional<pick_hit> pick(
    const scene_snapshot& s, const ray& r);

}  // namespace robosim::viz
