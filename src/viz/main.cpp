// robosim-viz — 3D visualizer entry point.
//
// Edit-mode read-side: load a robot description, build a scene snapshot,
// render primitive geometry, and expose scene/inspector/status panels.
//
// Determinism exception: this file lives under src/viz/, where the
// bans on system_clock / steady_clock / random_device do not apply
// (see .claude/skills/visualizer.md "Determinism exception").

#include <glad/gl.h>
// glad must come before any other GL header. GLFW's gl.h auto-include is
// suppressed by the GLFW_INCLUDE_NONE macro below.

#define GLFW_INCLUDE_NONE
#include "camera.h"
#include "description/loader.h"
#include "edit_mode_builder.h"
#include "panels.h"
#include "picking.h"
#include "renderer.h"
#include "scene_snapshot.h"

#include <GLFW/glfw3.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#include <ImGuizmo.h>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <imgui.h>
#include <optional>
#include <string_view>

namespace {

constexpr int initial_window_width = 1280;
constexpr int initial_window_height = 800;
constexpr std::string_view glsl_version = "#version 330 core";

struct cli_options {
  bool smoke_test = false;
  std::filesystem::path description_path;
};

struct gizmo_state {
  ImGuizmo::OPERATION operation = ImGuizmo::TRANSLATE;
  ImGuizmo::MODE mode = ImGuizmo::WORLD;
  bool snap_enabled = false;
  float translate_snap_m = 0.05F;
  float rotate_snap_deg = 5.0F;
};

void glfw_error_callback(int error_code, const char* description) {
  std::fprintf(stderr, "GLFW error %d: %s\n", error_code, description);
}

void print_usage() {
  std::fprintf(stderr,
               "usage: robosim-viz <path-to-description.json> "
               "[--mode=edit] [--smoke-test]\n");
}

std::optional<cli_options> parse_cli(int argc, char** argv) {
  cli_options options;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "--smoke-test") {
      options.smoke_test = true;
    } else if (arg == "--mode=edit") {
      // v0 ships only Edit mode. The flag is accepted to pin the public CLI.
    } else if (arg.starts_with("--mode=")) {
      std::fprintf(stderr, "robosim-viz: unsupported mode '%s'\n", argv[i]);
      return std::nullopt;
    } else if (arg.starts_with("--")) {
      std::fprintf(stderr, "robosim-viz: unknown argument '%s'\n", argv[i]);
      return std::nullopt;
    } else if (options.description_path.empty()) {
      options.description_path = argv[i];
    } else {
      std::fprintf(stderr, "robosim-viz: multiple description paths given\n");
      return std::nullopt;
    }
  }
  if (options.description_path.empty() && !options.smoke_test) {
    print_usage();
    return std::nullopt;
  }
  return options;
}

std::string load_error_to_string(const robosim::description::load_error& e) {
  std::string message = e.file_path.string();
  if (!e.json_pointer.empty()) {
    message += " ";
    message += e.json_pointer;
  }
  message += ": ";
  message += e.message;
  return message;
}

robosim::viz::scene_snapshot selected_or_whole_snapshot(
    const robosim::viz::scene_snapshot& snapshot) {
  if (!snapshot.selected_index.has_value() || *snapshot.selected_index >= snapshot.nodes.size()) {
    return snapshot;
  }

  robosim::viz::scene_snapshot selected;
  selected.nodes.push_back(snapshot.nodes[*snapshot.selected_index]);
  selected.selected_index = 0;
  return selected;
}

std::array<double, 3> normalize(std::array<double, 3> v) {
  const double len = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  if (len == 0.0) {
    return {0.0, 0.0, -1.0};
  }
  return {v[0] / len, v[1] / len, v[2] / len};
}

std::array<float, 16> to_imguizmo_matrix(const std::array<std::array<double, 4>, 4>& matrix) {
  std::array<float, 16> out{};
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      out[static_cast<std::size_t>(col * 4 + row)] = static_cast<float>(matrix[col][row]);
    }
  }
  return out;
}

void copy_from_imguizmo_matrix(const std::array<float, 16>& source,
                               robosim::viz::transform& target) {
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      target.m[col][row] = static_cast<double>(source[static_cast<std::size_t>(col * 4 + row)]);
    }
  }
}

robosim::viz::ray mouse_ray_world(const robosim::viz::orbit_camera& camera,
                                  std::array<int, 2> framebuffer_size,
                                  ImVec2 mouse_pos) {
  const double width = static_cast<double>(framebuffer_size[0]);
  const double height = static_cast<double>(framebuffer_size[1]);
  const double ndc_x = (2.0 * static_cast<double>(mouse_pos.x) / width) - 1.0;
  const double ndc_y = 1.0 - (2.0 * static_cast<double>(mouse_pos.y) / height);
  const double tan_half_fov = std::tan(camera.fov_y_rad / 2.0);

  const auto view = camera.view_matrix();
  const std::array<double, 3> right_world = {view[0][0], view[1][0], view[2][0]};
  const std::array<double, 3> up_world = {view[0][1], view[1][1], view[2][1]};
  const std::array<double, 3> back_world = {view[0][2], view[1][2], view[2][2]};

  const std::array<double, 3> direction = {
      ndc_x * camera.aspect * tan_half_fov * right_world[0] + ndc_y * tan_half_fov * up_world[0] -
          back_world[0],
      ndc_x * camera.aspect * tan_half_fov * right_world[1] + ndc_y * tan_half_fov * up_world[1] -
          back_world[1],
      ndc_x * camera.aspect * tan_half_fov * right_world[2] + ndc_y * tan_half_fov * up_world[2] -
          back_world[2],
  };

  return robosim::viz::ray{
      .origin_world = camera.eye_world(),
      .direction_world = normalize(direction),
  };
}

void handle_camera_input(robosim::viz::orbit_camera& camera,
                         robosim::viz::scene_snapshot& snapshot,
                         std::array<int, 2> framebuffer_size,
                         gizmo_state& gizmo) {
  ImGuiIO& io = ImGui::GetIO();

  if (!io.WantCaptureKeyboard) {
    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
      snapshot.selected_index = std::nullopt;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_F)) {
      const auto target = selected_or_whole_snapshot(snapshot);
      robosim::viz::frame_fit(camera, robosim::viz::compute_world_aabb(target));
    }
    if (ImGui::IsKeyPressed(ImGuiKey_W)) {
      gizmo.operation = ImGuizmo::TRANSLATE;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_E)) {
      gizmo.operation = ImGuizmo::ROTATE;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_X)) {
      gizmo.mode = gizmo.mode == ImGuizmo::WORLD ? ImGuizmo::LOCAL : ImGuizmo::WORLD;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_S)) {
      gizmo.snap_enabled = !gizmo.snap_enabled;
    }
  }

  if (io.WantCaptureMouse || framebuffer_size[0] <= 0 || framebuffer_size[1] <= 0) {
    return;
  }

  if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
    if (ImGuizmo::IsOver() || ImGuizmo::IsUsing()) {
      return;
    }
    const auto hit =
        robosim::viz::pick(snapshot, mouse_ray_world(camera, framebuffer_size, io.MousePos));
    snapshot.selected_index = hit.has_value() ? std::optional{hit->node_index} : std::nullopt;
  }

  constexpr double orbit_radians_per_pixel = 0.005;
  if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle)) {
    camera.yaw_rad -= static_cast<double>(io.MouseDelta.x) * orbit_radians_per_pixel;
    camera.pitch_rad += static_cast<double>(io.MouseDelta.y) * orbit_radians_per_pixel;
  }

  if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
    const double visible_height_m = 2.0 * camera.distance_m * std::tan(camera.fov_y_rad / 2.0);
    const double meters_per_pixel = visible_height_m / static_cast<double>(framebuffer_size[1]);
    robosim::viz::pan(camera,
                      -static_cast<double>(io.MouseDelta.x) * meters_per_pixel,
                      static_cast<double>(io.MouseDelta.y) * meters_per_pixel);
  }

  if (io.MouseWheel != 0.0F) {
    robosim::viz::zoom(camera, static_cast<double>(io.MouseWheel));
  }
}

void draw_selected_gizmo(const robosim::viz::orbit_camera& camera,
                         robosim::viz::scene_snapshot& snapshot,
                         const gizmo_state& gizmo) {
  if (!snapshot.selected_index.has_value() || *snapshot.selected_index >= snapshot.nodes.size()) {
    return;
  }

  auto& selected = snapshot.nodes[*snapshot.selected_index];
  auto view = to_imguizmo_matrix(camera.view_matrix());
  auto projection = to_imguizmo_matrix(camera.projection_matrix());
  auto model = to_imguizmo_matrix(selected.world_from_local.m);

  const ImGuiViewport* viewport = ImGui::GetMainViewport();
  ImGuizmo::SetOrthographic(false);
  ImGuizmo::SetDrawlist(ImGui::GetForegroundDrawList());
  ImGuizmo::SetRect(viewport->Pos.x, viewport->Pos.y, viewport->Size.x, viewport->Size.y);

  const std::array<float, 3> translate_snap = {
      gizmo.translate_snap_m, gizmo.translate_snap_m, gizmo.translate_snap_m};
  const std::array<float, 3> rotate_snap = {
      gizmo.rotate_snap_deg, gizmo.rotate_snap_deg, gizmo.rotate_snap_deg};
  const float* snap = nullptr;
  if (gizmo.snap_enabled && gizmo.operation == ImGuizmo::TRANSLATE) {
    snap = translate_snap.data();
  } else if (gizmo.snap_enabled && gizmo.operation == ImGuizmo::ROTATE) {
    snap = rotate_snap.data();
  }

  if (ImGuizmo::Manipulate(view.data(),
                           projection.data(),
                           gizmo.operation,
                           gizmo.mode,
                           model.data(),
                           nullptr,
                           snap)) {
    copy_from_imguizmo_matrix(model, selected.world_from_local);
  }
}

void draw_gizmo_controls(gizmo_state& gizmo) {
  ImGuiWindowFlags flags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_AlwaysAutoResize |
                           ImGuiWindowFlags_NoSavedSettings;
  ImGui::SetNextWindowPos(ImVec2(12.0F, 42.0F), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Gizmo", nullptr, flags)) {
    ImGui::End();
    return;
  }

  if (ImGui::RadioButton("Translate", gizmo.operation == ImGuizmo::TRANSLATE)) {
    gizmo.operation = ImGuizmo::TRANSLATE;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Rotate", gizmo.operation == ImGuizmo::ROTATE)) {
    gizmo.operation = ImGuizmo::ROTATE;
  }

  if (ImGui::RadioButton("World", gizmo.mode == ImGuizmo::WORLD)) {
    gizmo.mode = ImGuizmo::WORLD;
  }
  ImGui::SameLine();
  if (ImGui::RadioButton("Local", gizmo.mode == ImGuizmo::LOCAL)) {
    gizmo.mode = ImGuizmo::LOCAL;
  }

  ImGui::Checkbox("Snap", &gizmo.snap_enabled);
  ImGui::SetNextItemWidth(110.0F);
  ImGui::DragFloat("Move m", &gizmo.translate_snap_m, 0.005F, 0.001F, 10.0F, "%.3f");
  ImGui::SetNextItemWidth(110.0F);
  ImGui::DragFloat("Rotate deg", &gizmo.rotate_snap_deg, 0.5F, 0.1F, 180.0F, "%.1f");

  ImGui::End();
}

}  // namespace

int main(int argc, char** argv) {
  const auto cli = parse_cli(argc, argv);
  if (!cli.has_value()) {
    return EXIT_FAILURE;
  }

  robosim::viz::panels_state panels;
  robosim::viz::scene_snapshot snapshot;
  robosim::viz::orbit_camera camera;
  gizmo_state gizmo;

  if (!cli->description_path.empty()) {
    panels.source_path_display = cli->description_path.string();
    auto loaded = robosim::description::load_from_file(cli->description_path);
    if (loaded.has_value()) {
      panels.description = *loaded;
      snapshot = robosim::viz::build_edit_mode_snapshot(*panels.description);
      robosim::viz::frame_fit(camera, robosim::viz::compute_world_aabb(snapshot));
    } else {
      panels.load_error_message = load_error_to_string(loaded.error());
    }
  }

  glfwSetErrorCallback(glfw_error_callback);
  if (glfwInit() == GLFW_FALSE) {
    std::fprintf(stderr, "robosim-viz: glfwInit failed\n");
    if (cli->smoke_test) {
      std::fprintf(stderr, "robosim-viz: smoke test skipped (no display)\n");
      return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);

  GLFWwindow* window = glfwCreateWindow(
      initial_window_width, initial_window_height, "robosim-viz", nullptr, nullptr);
  if (window == nullptr) {
    std::fprintf(stderr, "robosim-viz: glfwCreateWindow failed\n");
    glfwTerminate();
    if (cli->smoke_test) {
      std::fprintf(stderr, "robosim-viz: smoke test skipped (no window)\n");
      return EXIT_SUCCESS;
    }
    return EXIT_FAILURE;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  if (gladLoadGL(glfwGetProcAddress) == 0) {
    std::fprintf(stderr, "robosim-viz: gladLoadGL failed\n");
    glfwDestroyWindow(window);
    glfwTerminate();
    return EXIT_FAILURE;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version.data());

  // The renderer owns GL buffer / VAO handles; it must destruct
  // before glfwDestroyWindow / glfwTerminate kill the GL context.
  // Scoping it inside this block ensures cleanup order:
  //   ~renderer → ImGui_ImplOpenGL3_Shutdown → glfwDestroyWindow.
  {
    robosim::viz::renderer scene_renderer;
    if (panels.description.has_value()) {
      const std::string title = "robosim-viz - " + panels.description->name + " [edit]";
      glfwSetWindowTitle(window, title.c_str());
    }

    while (glfwWindowShouldClose(window) == GLFW_FALSE) {
      glfwPollEvents();

      int framebuffer_width = 0;
      int framebuffer_height = 0;
      glfwGetFramebufferSize(window, &framebuffer_width, &framebuffer_height);
      if (framebuffer_height > 0) {
        camera.aspect =
            static_cast<double>(framebuffer_width) / static_cast<double>(framebuffer_height);
      }

      ImGui_ImplOpenGL3_NewFrame();
      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();
      ImGuizmo::BeginFrame();

      handle_camera_input(camera, snapshot, {framebuffer_width, framebuffer_height}, gizmo);

      scene_renderer.draw(snapshot,
                          camera.view_matrix(),
                          camera.projection_matrix(),
                          {framebuffer_width, framebuffer_height});

      ImGui::DockSpaceOverViewport(
          0, ImGui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode);
      robosim::viz::draw_panels(panels, snapshot);
      draw_gizmo_controls(gizmo);
      draw_selected_gizmo(camera, snapshot, gizmo);

      ImGui::Render();
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

      glfwSwapBuffers(window);

      if (cli->smoke_test) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
      }
    }
  }  // ~renderer here, while the GL context is still current.

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();
  return EXIT_SUCCESS;
}
