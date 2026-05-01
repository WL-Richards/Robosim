#include "edit_mode_apply.h"

#include "scene_snapshot.h"

#include "description/origin_pose.h"
#include "description/schema.h"

#include <cstddef>
#include <string>

namespace robosim::viz {

namespace {

namespace desc = robosim::description;

desc::transform_4x4 to_4x4(const transform& t) { return t.m; }

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

desc::transform_4x4 identity_4x4() {
  desc::transform_4x4 m{};
  m[0][0] = 1.0;
  m[1][1] = 1.0;
  m[2][2] = 1.0;
  m[3][3] = 1.0;
  return m;
}

// Inverse of a rigid transform (rotation block + translation column,
// last row [0 0 0 1]). Rotation block is orthonormal so its inverse is
// its transpose; translation inverse is -R^T * t.
desc::transform_4x4 inverse_rigid(const desc::transform_4x4& m) {
  desc::transform_4x4 out = identity_4x4();
  // Transposed rotation block.
  for (int col = 0; col < 3; ++col) {
    for (int row = 0; row < 3; ++row) {
      out[col][row] = m[row][col];
    }
  }
  // Inverse translation = -R^T * t.
  for (int row = 0; row < 3; ++row) {
    double sum = 0.0;
    for (int k = 0; k < 3; ++k) {
      sum += out[k][row] * m[3][k];
    }
    out[3][row] = -sum;
  }
  return out;
}

constexpr const char* world_parent_token = "world";

// Walk source.joints to find the joint whose `child` link name matches
// `link_name`. Every link except the root must be the child of exactly
// one joint (loader decision); for the well-formed v0 fixture this
// always succeeds.
const desc::joint* find_parent_joint_of_link(const desc::robot_description& d,
                                             const std::string& link_name) {
  for (const auto& j : d.joints) {
    if (j.child == link_name) {
      return &j;
    }
  }
  return nullptr;
}

// Compute the kinematic frame of a link in world (i.e., the parent
// joint's kinematic frame, since a link does not move relative to its
// parent joint in Edit mode). Walks the description directly so the
// snapshot is not load-bearing here — the snapshot is consulted only
// for entity_name / kind / parent_index lookups.
desc::transform_4x4 link_kinematic_world(const desc::robot_description& d,
                                          const std::string& link_name);

desc::transform_4x4 joint_kinematic_world(const desc::robot_description& d,
                                           const desc::joint& j) {
  const desc::transform_4x4 parent_kinematic =
      (j.parent == world_parent_token)
          ? identity_4x4()
          : link_kinematic_world(d, j.parent);
  return mul_4x4(parent_kinematic, desc::compose_origin(j.origin));
}

desc::transform_4x4 link_kinematic_world(const desc::robot_description& d,
                                          const std::string& link_name) {
  const desc::joint* parent_joint = find_parent_joint_of_link(d, link_name);
  if (parent_joint == nullptr) {
    return identity_4x4();  // floating link with no parent joint (not in v0).
  }
  return joint_kinematic_world(d, *parent_joint);
}

const desc::joint* find_joint_by_name(const desc::robot_description& d,
                                      const std::string& joint_name) {
  for (const auto& j : d.joints) {
    if (j.name == joint_name) {
      return &j;
    }
  }
  return nullptr;
}

}  // namespace

desc::robot_description apply_gizmo_target(
    const desc::robot_description& source,
    const scene_snapshot& snapshot,
    std::size_t selected_index,
    const transform& target_world_from_local) {
  desc::robot_description result = source;
  const auto& selected = snapshot.nodes[selected_index];
  const desc::transform_4x4 target = to_4x4(target_world_from_local);

  if (selected.kind == node_kind::joint) {
    // Find the joint in result.joints by name.
    desc::joint* j = nullptr;
    for (auto& candidate : result.joints) {
      if (candidate.name == selected.entity_name) {
        j = &candidate;
        break;
      }
    }
    if (j == nullptr) {
      return result;  // shouldn't happen given preconditions
    }

    // Parent kinematic = world if root, else parent link's kinematic.
    const desc::transform_4x4 parent_kinematic =
        (j->parent == world_parent_token)
            ? identity_4x4()
            : link_kinematic_world(result, j->parent);

    // new_origin_4x4 = inverse(parent_kinematic) * target.
    const desc::transform_4x4 new_origin =
        mul_4x4(inverse_rigid(parent_kinematic), target);
    j->origin = desc::decompose_origin(new_origin);
    return result;
  }

  if (selected.kind == node_kind::motor ||
      selected.kind == node_kind::motor_direction_arrow) {
    desc::motor* m = nullptr;
    for (auto& candidate : result.motors) {
      if (candidate.name == selected.entity_name) {
        m = &candidate;
        break;
      }
    }
    if (m == nullptr) {
      return result;
    }

    const desc::joint* parent_joint = find_joint_by_name(result, m->joint_name);
    if (parent_joint == nullptr) {
      return result;
    }
    const desc::transform_4x4 joint_kinematic =
        joint_kinematic_world(result, *parent_joint);
    const desc::transform_4x4 new_visual_origin =
        mul_4x4(inverse_rigid(joint_kinematic), target);
    m->visual_origin = desc::decompose_origin(new_visual_origin);
    return result;
  }

  // Link node: write to link.visual_origin. The link's kinematic frame
  // equals the parent joint's kinematic frame.
  desc::link* l = nullptr;
  for (auto& candidate : result.links) {
    if (candidate.name == selected.entity_name) {
      l = &candidate;
      break;
    }
  }
  if (l == nullptr) {
    return result;
  }

  const desc::transform_4x4 link_kinematic =
      link_kinematic_world(result, l->name);

  const desc::transform_4x4 new_visual_origin =
      mul_4x4(inverse_rigid(link_kinematic), target);
  l->visual_origin = desc::decompose_origin(new_visual_origin);
  return result;
}

}  // namespace robosim::viz
