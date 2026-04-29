#include "panels.h"

#include "scene_snapshot.h"

#include "description/error.h"
#include "description/schema.h"

#include <imgui.h>

#include <cstdio>
#include <string>

namespace robosim::viz {

namespace {

namespace desc = robosim::description;

void draw_link_inspector(const desc::link& l) {
  ImGui::Text("link: %s", l.name.c_str());
  ImGui::Separator();
  ImGui::Text("mass_kg = %.6g", l.mass_kg);
  ImGui::Text("length_m = %.6g", l.length_m);
  ImGui::Text("inertia_kgm2 = %.6g", l.inertia_kgm2);
}

void draw_joint_inspector(const desc::joint& j) {
  ImGui::Text("joint: %s", j.name.c_str());
  ImGui::Separator();
  ImGui::Text("type = revolute");
  ImGui::Text("parent = %s", j.parent.c_str());
  ImGui::Text("child = %s", j.child.c_str());
  ImGui::Text("axis = [%.6g, %.6g, %.6g]", j.axis[0], j.axis[1], j.axis[2]);
  ImGui::Text("limit_lower_rad = %.6g", j.limit_lower_rad);
  ImGui::Text("limit_upper_rad = %.6g", j.limit_upper_rad);
  ImGui::Text("viscous_friction_nm_per_rad_per_s = %.6g",
              j.viscous_friction_nm_per_rad_per_s);
}

void draw_motor_inspector(const desc::motor& m) {
  ImGui::Text("motor: %s", m.name.c_str());
  ImGui::Separator();
  ImGui::Text("motor_model = %s", m.motor_model.c_str());
  ImGui::Text("controller = %s", m.controller.c_str());
  ImGui::Text("controller_can_id = %d", m.controller_can_id);
  ImGui::Text("controller_firmware_version = %s",
              m.controller_firmware_version.c_str());
  ImGui::Text("joint = %s", m.joint_name.c_str());
  ImGui::Text("gear_ratio = %.6g", m.gear_ratio);
}

void draw_sensor_inspector(const desc::sensor& s) {
  ImGui::Text("sensor: %s", s.name.c_str());
  ImGui::Separator();
  ImGui::Text("sensor_model = %s", s.sensor_model.c_str());
  ImGui::Text("controller_can_id = %d", s.controller_can_id);
  ImGui::Text("controller_firmware_version = %s",
              s.controller_firmware_version.c_str());
  ImGui::Text("joint = %s", s.joint_name.c_str());
}

[[nodiscard]] const desc::link* find_link(
    const desc::robot_description& d, const std::string& name) {
  for (const auto& l : d.links) {
    if (l.name == name) return &l;
  }
  return nullptr;
}

[[nodiscard]] const desc::joint* find_joint(
    const desc::robot_description& d, const std::string& name) {
  for (const auto& j : d.joints) {
    if (j.name == name) return &j;
  }
  return nullptr;
}

void draw_scene_tree(const scene_snapshot& s_snapshot,
                     scene_snapshot& mut) {
  if (ImGui::Begin("Scene")) {
    for (std::size_t i = 0; i < s_snapshot.nodes.size(); ++i) {
      const auto& node = s_snapshot.nodes[i];
      const bool selected = mut.selected_index.has_value() &&
                            *mut.selected_index == i;
      char label[256];
      std::snprintf(label, sizeof(label), "%s %s##%zu",
                    node.kind == node_kind::link ? "[L]" : "[J]",
                    node.entity_name.c_str(), i);
      if (ImGui::Selectable(label, selected)) {
        mut.selected_index = i;
      }
    }
  }
  ImGui::End();
}

void draw_inspector(const panels_state& state, const scene_snapshot& s) {
  if (ImGui::Begin("Inspector")) {
    if (!state.description.has_value()) {
      ImGui::TextWrapped("No robot description loaded.");
      ImGui::End();
      return;
    }
    if (!s.selected_index.has_value()) {
      ImGui::TextWrapped("Select a node from the scene tree.");
      ImGui::End();
      return;
    }
    const auto& node = s.nodes[*s.selected_index];
    if (node.kind == node_kind::link) {
      if (const auto* l = find_link(*state.description, node.entity_name);
          l != nullptr) {
        draw_link_inspector(*l);
        // Bound motors / sensors for this link's parent joint, if any.
      }
    } else {
      if (const auto* j = find_joint(*state.description, node.entity_name);
          j != nullptr) {
        draw_joint_inspector(*j);
        ImGui::Separator();
        ImGui::Text("--- bound devices ---");
        for (const auto& m : state.description->motors) {
          if (m.joint_name == j->name) {
            ImGui::Spacing();
            draw_motor_inspector(m);
          }
        }
        for (const auto& sensor : state.description->sensors) {
          if (sensor.joint_name == j->name) {
            ImGui::Spacing();
            draw_sensor_inspector(sensor);
          }
        }
      }
    }
  }
  ImGui::End();
}

void draw_status_bar(const panels_state& state) {
  const ImGuiViewport* vp = ImGui::GetMainViewport();
  ImGui::SetNextWindowPos(
      ImVec2(vp->WorkPos.x, vp->WorkPos.y + vp->WorkSize.y - 28.0F));
  ImGui::SetNextWindowSize(ImVec2(vp->WorkSize.x, 28.0F));
  const ImGuiWindowFlags flags = ImGuiWindowFlags_NoTitleBar |
                                 ImGuiWindowFlags_NoResize |
                                 ImGuiWindowFlags_NoMove |
                                 ImGuiWindowFlags_NoSavedSettings |
                                 ImGuiWindowFlags_NoScrollbar |
                                 ImGuiWindowFlags_NoBringToFrontOnFocus;
  if (ImGui::Begin("##status", nullptr, flags)) {
    if (!state.load_error_message.empty()) {
      ImGui::TextColored(ImVec4(1.0F, 0.4F, 0.4F, 1.0F), "load error: %s",
                         state.load_error_message.c_str());
    } else if (state.description.has_value()) {
      ImGui::Text("loaded: %s (%s)",
                  state.description->name.c_str(),
                  state.source_path_display.c_str());
    } else {
      ImGui::Text("no description loaded");
    }
  }
  ImGui::End();
}

}  // namespace

void draw_panels(const panels_state& state, scene_snapshot& s) {
  draw_scene_tree(s, s);
  draw_inspector(state, s);
  draw_status_bar(state);
}

}  // namespace robosim::viz
