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
constexpr double motor_body_half_extent_x_m = 0.08;
constexpr double motor_body_half_extent_y_m = 0.08;
constexpr double motor_body_half_extent_z_m = 0.15;
constexpr double kraken_x60_mesh_scale_m_per_unit = 0.001;
constexpr double kraken_x60_mesh_min_x = -30.045158;
constexpr double kraken_x60_mesh_min_y = -33.396683;
constexpr double kraken_x60_mesh_min_z = -43.377052;
constexpr double kraken_x60_mesh_max_x = 30.044394;
constexpr double kraken_x60_mesh_max_y = 32.478725;
constexpr double kraken_x60_mesh_max_z = 68.603653;
constexpr double motor_direction_arrow_radius_m = 0.022;
constexpr double motor_direction_arrow_back_offset_m = 0.003;

constexpr const char* world_parent_token = "world";

primitive make_link_primitive(const desc::link& l) {
  return primitive{primitive_kind::cylinder, l.length_m, link_default_radius_m,
                   0.0, 0.0, 0.0};
}

primitive make_joint_primitive() {
  return primitive{primitive_kind::arrow, arrow_default_length_m,
                   arrow_default_radius_m, 0.0, 0.0, 0.0};
}

primitive make_box_motor_body_primitive() {
  return primitive{primitive_kind::box, 0.0, 0.0, motor_body_half_extent_x_m,
                   motor_body_half_extent_y_m, motor_body_half_extent_z_m};
}

primitive make_kraken_x60_motor_body_primitive() {
  primitive p{};
  p.kind = primitive_kind::mesh;
  p.mesh_id = "kraken_x60";
  p.mesh_scale_m_per_unit = kraken_x60_mesh_scale_m_per_unit;
  p.mesh_min_local = {kraken_x60_mesh_min_x, kraken_x60_mesh_min_y,
                      kraken_x60_mesh_min_z};
  p.mesh_max_local = {kraken_x60_mesh_max_x, kraken_x60_mesh_max_y,
                      kraken_x60_mesh_max_z};
  return p;
}

primitive make_motor_body_primitive(const desc::motor& m) {
  if (m.motor_model == "kraken_x60") {
    return make_kraken_x60_motor_body_primitive();
  }
  return make_box_motor_body_primitive();
}

primitive make_motor_direction_arrow_primitive() {
  return primitive{primitive_kind::rotation_arrow, 0.0,
                   motor_direction_arrow_radius_m, 0.0, 0.0, 0.0};
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

desc::transform_4x4 translate_4x4(double x, double y, double z) {
  desc::transform_4x4 m = identity_4x4();
  m[3][0] = x;
  m[3][1] = y;
  m[3][2] = z;
  return m;
}

double motor_back_z_m(const primitive& p) {
  if (p.kind == primitive_kind::mesh) {
    return p.mesh_min_local[2] * p.mesh_scale_m_per_unit;
  }
  return -p.half_extent_z_m;
}

}  // namespace

scene_snapshot build_edit_mode_snapshot(const desc::robot_description& d) {
  scene_snapshot snapshot;
  snapshot.nodes.reserve(d.links.size() + d.joints.size() + (2 * d.motors.size()));

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
  std::unordered_map<std::string, desc::transform_4x4> joint_kinematic_by_name;

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
    joint_kinematic_by_name.emplace(j.name, joint_kinematic);
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

  for (const auto& m : d.motors) {
    auto joint_it = joint_kinematic_by_name.find(m.joint_name);
    auto parent_it = joint_index_by_name.find(m.joint_name);
    if (joint_it == joint_kinematic_by_name.end() || parent_it == joint_index_by_name.end()) {
      continue;
    }

    const desc::transform_4x4 motor_visual =
        mul_4x4(joint_it->second, desc::compose_origin(m.visual_origin));

    scene_node motor_node;
    motor_node.entity_name = m.name;
    motor_node.kind = node_kind::motor;
    motor_node.world_from_local = from_4x4(motor_visual);
    motor_node.shape = make_motor_body_primitive(m);
    motor_node.parent_index = parent_it->second;
    const primitive motor_body_shape = *motor_node.shape;
    const std::size_t motor_idx = snapshot.nodes.size();
    snapshot.nodes.push_back(std::move(motor_node));

    if (m.show_direction_arrow) {
      scene_node arrow_node;
      arrow_node.entity_name = m.name;
      arrow_node.kind = node_kind::motor_direction_arrow;
      arrow_node.world_from_local = from_4x4(mul_4x4(
          motor_visual,
          translate_4x4(0.0, 0.0,
                        motor_back_z_m(motor_body_shape) -
                            motor_direction_arrow_back_offset_m)));
      arrow_node.shape = make_motor_direction_arrow_primitive();
      arrow_node.parent_index = motor_idx;
      snapshot.nodes.push_back(std::move(arrow_node));
    }
  }

  return snapshot;
}

}  // namespace robosim::viz
