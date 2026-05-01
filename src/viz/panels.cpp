#include "panels.h"

#include "scene_snapshot.h"

#include "description/error.h"
#include "description/schema.h"

#include <imgui.h>

#include <cstdarg>
#include <cstdio>
#include <string>

namespace robosim::viz {

namespace {

namespace desc = robosim::description;

constexpr ImVec4 kEntityHeaderColor{0.55F, 0.82F, 1.0F, 1.0F};
constexpr ImVec4 kSubHeaderColor{0.85F, 0.7F, 0.45F, 1.0F};

bool begin_props_table(const char* id) {
  return ImGui::BeginTable(
      id, 2,
      ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_PadOuterX |
          ImGuiTableFlags_NoBordersInBody);
}

void prop_row(const char* label, const char* fmt, ...) {
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::TextDisabled("%s", label);
  ImGui::TableSetColumnIndex(1);
  std::va_list args;
  va_start(args, fmt);
  ImGui::TextV(fmt, args);
  va_end(args);
}

const char* node_kind_label(node_kind kind) {
  switch (kind) {
    case node_kind::link:
      return "[L]";
    case node_kind::joint:
      return "[J]";
    case node_kind::motor:
      return "[M]";
    case node_kind::motor_direction_arrow:
      return "[A]";
  }
  return "[?]";
}

void entity_header(const char* tag, const char* name) {
  ImGui::PushStyleColor(ImGuiCol_Text, kEntityHeaderColor);
  ImGui::Text("%s  %s", tag, name);
  ImGui::PopStyleColor();
  ImGui::Spacing();
}

void draw_link_inspector(const desc::link& l) {
  entity_header("Link", l.name.c_str());

  if (ImGui::CollapsingHeader("Physical", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (begin_props_table("##link_physical")) {
      prop_row("mass",    "%.4g kg",     l.mass_kg);
      prop_row("length",  "%.4g m",      l.length_m);
      prop_row("inertia", "%.4g kg·m²",  l.inertia_kgm2);
      ImGui::EndTable();
    }
  }
}

void draw_joint_inspector(const desc::joint& j) {
  entity_header("Joint", j.name.c_str());

  if (ImGui::CollapsingHeader("Topology", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (begin_props_table("##joint_topology")) {
      prop_row("type",   "revolute");
      prop_row("parent", "%s", j.parent.c_str());
      prop_row("child",  "%s", j.child.c_str());
      prop_row("axis",   "[%.4g, %.4g, %.4g]",
               j.axis[0], j.axis[1], j.axis[2]);
      ImGui::EndTable();
    }
  }

  if (ImGui::CollapsingHeader("Limits", ImGuiTreeNodeFlags_DefaultOpen)) {
    if (begin_props_table("##joint_limits")) {
      prop_row("lower",   "%.4g rad", j.limit_lower_rad);
      prop_row("upper",   "%.4g rad", j.limit_upper_rad);
      prop_row("viscous", "%.4g N·m/(rad/s)",
               j.viscous_friction_nm_per_rad_per_s);
      ImGui::EndTable();
    }
  }
}

void draw_motor_inspector(desc::motor& m, bool& dirty) {
  ImGui::PushStyleColor(ImGuiCol_Text, kSubHeaderColor);
  ImGui::Text("Motor  %s", m.name.c_str());
  ImGui::PopStyleColor();
  if (begin_props_table("##motor_props")) {
    prop_row("type",       "%s", m.motor_model.c_str());
    prop_row("controller", "%s", m.controller.c_str());
    prop_row("firmware",   "%s", m.controller_firmware_version.c_str());
    prop_row("gear ratio", "%.4g", m.gear_ratio);
    ImGui::EndTable();
  }

  if (ImGui::CollapsingHeader("Connection", ImGuiTreeNodeFlags_DefaultOpen)) {
    const char* connection_types[] = {"CAN"};
    int selected_connection = 0;
    ImGui::SetNextItemWidth(140.0F);
    if (ImGui::Combo("Connection type",
                     &selected_connection,
                     connection_types,
                     IM_ARRAYSIZE(connection_types))) {
      m.connection_type = "CAN";
      dirty = true;
    }

    int can_id = m.controller_can_id;
    ImGui::SetNextItemWidth(90.0F);
    if (ImGui::InputInt("CAN ID", &can_id)) {
      if (can_id < 0) can_id = 0;
      if (can_id > 63) can_id = 63;
      if (can_id != m.controller_can_id) {
        m.controller_can_id = can_id;
        dirty = true;
      }
    }
  }

  if (ImGui::CollapsingHeader("Visualization", ImGuiTreeNodeFlags_DefaultOpen)) {
    bool show_arrow = m.show_direction_arrow;
    if (ImGui::Checkbox("Show direction arrow", &show_arrow)) {
      m.show_direction_arrow = show_arrow;
      dirty = true;
    }
  }
}

void draw_sensor_inspector(const desc::sensor& s) {
  ImGui::PushStyleColor(ImGuiCol_Text, kSubHeaderColor);
  ImGui::Text("Sensor  %s", s.name.c_str());
  ImGui::PopStyleColor();
  if (begin_props_table("##sensor_props")) {
    prop_row("model",      "%s", s.sensor_model.c_str());
    prop_row("controller", "CAN %d", s.controller_can_id);
    prop_row("firmware",   "%s", s.controller_firmware_version.c_str());
    ImGui::EndTable();
  }
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

[[nodiscard]] desc::motor* find_motor(
    desc::robot_description& d, const std::string& name) {
  for (auto& m : d.motors) {
    if (m.name == name) return &m;
  }
  return nullptr;
}

void draw_scene_tree(const scene_snapshot& s_snapshot,
                     scene_snapshot& mut) {
  if (ImGui::Begin("Scene")) {
    for (std::size_t i = 0; i < s_snapshot.nodes.size(); ++i) {
      const auto& node = s_snapshot.nodes[i];
      if (node.kind == node_kind::motor_direction_arrow) {
        continue;
      }
      const bool selected = mut.selected_index.has_value() &&
                            *mut.selected_index == i;
      char label[256];
      std::snprintf(label, sizeof(label), "%s %s##%zu",
                    node_kind_label(node.kind),
                    node.entity_name.c_str(), i);
      if (ImGui::Selectable(label, selected)) {
        mut.selected_index = i;
      }
    }
  }
  ImGui::End();
}

void draw_inspector(panels_state& state, const scene_snapshot& s) {
  if (ImGui::Begin("Inspector")) {
    if (!state.description.has_value()) {
      ImGui::TextDisabled("No robot description loaded.");
      ImGui::End();
      return;
    }
    if (!s.selected_index.has_value()) {
      ImGui::TextDisabled("Select a node from the scene tree.");
      ImGui::End();
      return;
    }
    const auto& node = s.nodes[*s.selected_index];
    if (node.kind == node_kind::link) {
      if (const auto* l = find_link(*state.description, node.entity_name);
          l != nullptr) {
        draw_link_inspector(*l);
      }
    } else if (node.kind == node_kind::joint) {
      if (const auto* j = find_joint(*state.description, node.entity_name);
          j != nullptr) {
        draw_joint_inspector(*j);

        bool any_bound = false;
        for (const auto& m : state.description->motors) {
          if (m.joint_name == j->name) { any_bound = true; break; }
        }
        if (!any_bound) {
          for (const auto& sensor : state.description->sensors) {
            if (sensor.joint_name == j->name) { any_bound = true; break; }
          }
        }
        if (any_bound &&
            ImGui::CollapsingHeader("Bound devices",
                                    ImGuiTreeNodeFlags_DefaultOpen)) {
          for (const auto& m : state.description->motors) {
            if (m.joint_name == j->name) {
              auto* editable_motor = find_motor(*state.description, m.name);
              if (editable_motor != nullptr) {
                draw_motor_inspector(*editable_motor, state.description_dirty);
              }
              ImGui::Spacing();
            }
          }
          for (const auto& sensor : state.description->sensors) {
            if (sensor.joint_name == j->name) {
              draw_sensor_inspector(sensor);
              ImGui::Spacing();
            }
          }
        }
      }
    } else {
      if (auto* m = find_motor(*state.description, node.entity_name);
          m != nullptr) {
        draw_motor_inspector(*m, state.description_dirty);
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

void draw_panels(panels_state& state, scene_snapshot& s) {
  draw_scene_tree(s, s);
  draw_inspector(state, s);
  draw_status_bar(state);
}

}  // namespace robosim::viz
