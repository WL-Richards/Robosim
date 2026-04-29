#include "edit_mode_builder.h"

#include "scene_snapshot.h"
#include "description/schema.h"

#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

namespace robosim::viz {

namespace {

namespace desc = robosim::description;

// Defaults pinned in tests/viz/TEST_PLAN.md S2/S3.
constexpr double link_default_radius_m = 0.05;
constexpr double arrow_default_length_m = 0.20;
constexpr double arrow_default_radius_m = 0.01;

constexpr const char* world_parent_token = "world";

primitive make_link_primitive(const desc::link& l) {
  return primitive{
      .kind = primitive_kind::cylinder,
      .length_m = l.length_m,
      .radius_m = link_default_radius_m,
      .half_extent_x_m = 0.0,
      .half_extent_y_m = 0.0,
      .half_extent_z_m = 0.0,
  };
}

primitive make_joint_primitive() {
  return primitive{
      .kind = primitive_kind::arrow,
      .length_m = arrow_default_length_m,
      .radius_m = arrow_default_radius_m,
      .half_extent_x_m = 0.0,
      .half_extent_y_m = 0.0,
      .half_extent_z_m = 0.0,
  };
}

}  // namespace

scene_snapshot build_edit_mode_snapshot(const desc::robot_description& d) {
  scene_snapshot snapshot;
  snapshot.nodes.reserve(d.links.size() + d.joints.size());

  std::unordered_map<std::string, const desc::link*> link_by_name;
  link_by_name.reserve(d.links.size());
  for (const auto& l : d.links) {
    link_by_name.emplace(l.name, &l);
  }

  // Joints grouped by their parent link's name (or "world"). Insertion
  // order matches description.joints, which makes BFS deterministic.
  std::unordered_map<std::string, std::vector<const desc::joint*>>
      joints_by_parent;
  for (const auto& j : d.joints) {
    joints_by_parent[j.parent].push_back(&j);
  }

  std::unordered_map<std::string, std::size_t> link_index_by_name;
  std::unordered_map<std::string, std::size_t> joint_index_by_name;

  std::queue<const desc::joint*> joint_queue;
  if (auto roots = joints_by_parent.find(world_parent_token);
      roots != joints_by_parent.end()) {
    for (const auto* jp : roots->second) {
      joint_queue.push(jp);
    }
  }

  while (!joint_queue.empty()) {
    const desc::joint& j = *joint_queue.front();
    joint_queue.pop();

    scene_node joint_node;
    joint_node.entity_name = j.name;
    joint_node.kind = node_kind::joint;
    joint_node.world_from_local = transform::identity();
    joint_node.shape = make_joint_primitive();
    joint_node.joint_axis_local = j.axis;
    if (j.parent != world_parent_token) {
      joint_node.parent_index = link_index_by_name.at(j.parent);
    }
    const std::size_t joint_idx = snapshot.nodes.size();
    joint_index_by_name.emplace(j.name, joint_idx);
    snapshot.nodes.push_back(std::move(joint_node));

    const desc::link& child_link = *link_by_name.at(j.child);
    scene_node link_node;
    link_node.entity_name = child_link.name;
    link_node.kind = node_kind::link;
    link_node.world_from_local = transform::identity();
    link_node.shape = make_link_primitive(child_link);
    link_node.parent_index = joint_idx;
    const std::size_t link_idx = snapshot.nodes.size();
    link_index_by_name.emplace(child_link.name, link_idx);
    snapshot.nodes.push_back(std::move(link_node));

    if (auto child_joints = joints_by_parent.find(child_link.name);
        child_joints != joints_by_parent.end()) {
      for (const auto* jp : child_joints->second) {
        joint_queue.push(jp);
      }
    }
  }

  return snapshot;
}

}  // namespace robosim::viz
