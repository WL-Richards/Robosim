#pragma once

// Renderer — draws a scene_snapshot using OpenGL 3.3 core. The renderer
// is the single architectural commitment of the visualizer's data-source
// seam: it must not read from `robot_description`. See
// `cmake/lint_renderer_isolation.cmake` (TEST_PLAN.md T1).

#include "scene_snapshot.h"

#include <array>
#include <memory>

namespace robosim::viz {

class renderer {
 public:
  renderer();
  ~renderer();

  renderer(const renderer&) = delete;
  renderer& operator=(const renderer&) = delete;
  renderer(renderer&&) = delete;
  renderer& operator=(renderer&&) = delete;

  // Draw the snapshot using the supplied view + projection matrices.
  // framebuffer_size is the viewport pixel size (width, height).
  void draw(const scene_snapshot& s,
            const std::array<std::array<double, 4>, 4>& view,
            const std::array<std::array<double, 4>, 4>& projection,
            std::array<int, 2> framebuffer_size);

 private:
  struct impl;
  std::unique_ptr<impl> impl_;
};

}  // namespace robosim::viz
