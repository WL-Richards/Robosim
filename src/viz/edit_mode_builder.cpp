#include "edit_mode_builder.h"

#include "scene_snapshot.h"

#include "description/origin_pose.h"
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

// 4x4 column-major homogeneous matmul: out = a * b. Used for the
// kinematic-chain composition in the builder. Pure rigid transforms;
// no projective rows. The viz subtree could pull in glm here, but
// keeping the math local avoids dragging glm into description-only
// callers and is small enough to inline.
desc::transform_4x4 mul_4x4(const desc::transform_4x4& a,
                            const desc::transform_4x4& b) {
  desc::transform_4x4 out{};
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      double sum = 0.0;
      for (int k = 0; k < 4; ++k) {
        sum += a[k][row] * b[col][k];
      }
      out[col][row] = sum;
    }
  }
  return out;
}

transform from_4x4(const desc::transform_4x4& m) {
  transform t;
  t.m = m;
  return t;
}

desc::transform_4x4 identity_4x4() {
  desc::transform_4x4 m{};
  m[0][0] = 1.0;
  m[1][1] = 1.0;
  m[2][2] = 1.0;
  m[3][3] = 1.0;
  return m;
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

  // Per-link kinematic frame in world. For a child link of joint j,
  // link_kinematic[link_name] = parent_link_kinematic *
  // compose_origin(j.origin). This is *not* the link's visual frame
  // (that adds compose_origin(link.visual_origin)) — see TEST_PLAN_VD
  // convention #13.
  std::unordered_map<std::string, desc::transform_4x4> link_kinematic_by_name;

  // BFS queue carries (joint_pointer, parent_link_kinematic).
  using queue_entry = std::pair<const desc::joint*, desc::transform_4x4>;
  std::queue<queue_entry> joint_queue;
  if (auto roots = joints_by_parent.find(world_parent_token);
      roots != joints_by_parent.end()) {
    for (const auto* jp : roots->second) {
      joint_queue.push({jp, identity_4x4()});
    }
  }

  while (!joint_queue.empty()) {
    const auto [jp, parent_link_kinematic] = joint_queue.front();
    const desc::joint& j = *jp;
    joint_queue.pop();

    const desc::transform_4x4 joint_kinematic =
        mul_4x4(parent_link_kinematic, desc::compose_origin(j.origin));

    scene_node joint_node;
    joint_node.entity_name = j.name;
    joint_node.kind = node_kind::joint;
    joint_node.world_from_local = from_4x4(joint_kinematic);
    joint_node.shape = make_joint_primitive();
    joint_node.joint_axis_local = j.axis;
    if (j.parent != world_parent_token) {
      joint_node.parent_index = link_index_by_name.at(j.parent);
    }
    const std::size_t joint_idx = snapshot.nodes.size();
    joint_index_by_name.emplace(j.name, joint_idx);
    snapshot.nodes.push_back(std::move(joint_node));

    const desc::link& child_link = *link_by_name.at(j.child);
    // Child link kinematic = joint kinematic (no joint motion in Edit
    // mode; that's a Live-mode concern). Visual frame = kinematic *
    // compose_origin(visual_origin).
    const desc::transform_4x4 link_kinematic = joint_kinematic;
    const desc::transform_4x4 link_visual =
        mul_4x4(link_kinematic, desc::compose_origin(child_link.visual_origin));
    link_kinematic_by_name.emplace(child_link.name, link_kinematic);

    scene_node link_node;
    link_node.entity_name = child_link.name;
    link_node.kind = node_kind::link;
    link_node.world_from_local = from_4x4(link_visual);
    link_node.shape = make_link_primitive(child_link);
    link_node.parent_index = joint_idx;
    const std::size_t link_idx = snapshot.nodes.size();
    link_index_by_name.emplace(child_link.name, link_idx);
    snapshot.nodes.push_back(std::move(link_node));

    if (auto child_joints = joints_by_parent.find(child_link.name);
        child_joints != joints_by_parent.end()) {
      for (const auto* child_jp : child_joints->second) {
        joint_queue.push({child_jp, link_kinematic});
      }
    }
  }

  return snapshot;
}

}  // namespace robosim::viz
