#include "origin_pose.h"

#include "error.h"

#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>

// The only TU in src/description/ allowed to call std::isfinite /
// std::isnan / std::isinf / std::isnormal / std::fpclassify.
// Enforced at configure time by cmake/lint_loader_finite_seam.cmake.

namespace robosim::description {

// R = R_z(rpy_rad[2]) * R_y(rpy_rad[1]) * R_x(rpy_rad[0])
// applied to column vectors (intrinsic-Z-Y'-X'', URDF order).
// Column-major storage: m[col][row].
transform_4x4 compose_origin(const origin_pose& p) {
  const double cr = std::cos(p.rpy_rad[0]);
  const double sr = std::sin(p.rpy_rad[0]);
  const double cp = std::cos(p.rpy_rad[1]);
  const double sp = std::sin(p.rpy_rad[1]);
  const double cy = std::cos(p.rpy_rad[2]);
  const double sy = std::sin(p.rpy_rad[2]);

  transform_4x4 m{};
  // Column 0
  m[0][0] = cy * cp;
  m[0][1] = sy * cp;
  m[0][2] = -sp;
  m[0][3] = 0.0;
  // Column 1
  m[1][0] = cy * sp * sr - sy * cr;
  m[1][1] = sy * sp * sr + cy * cr;
  m[1][2] = cp * sr;
  m[1][3] = 0.0;
  // Column 2
  m[2][0] = cy * sp * cr + sy * sr;
  m[2][1] = sy * sp * cr - cy * sr;
  m[2][2] = cp * cr;
  m[2][3] = 0.0;
  // Column 3 — translation + homogeneous row
  m[3][0] = p.xyz_m[0];
  m[3][1] = p.xyz_m[1];
  m[3][2] = p.xyz_m[2];
  m[3][3] = 1.0;
  return m;
}

std::optional<load_error> validate_finite_xyz(
    const std::array<double, 3>& xyz,
    std::string_view base_pointer) {
  for (std::size_t i = 0; i < 3; ++i) {
    if (!std::isfinite(xyz[i])) {
      return load_error{
          load_error_kind::domain_error,
          {},
          std::string(base_pointer) + "/" + std::to_string(i),
          "xyz_m component " + std::to_string(i) + " is not finite",
      };
    }
  }
  return std::nullopt;
}

std::optional<load_error> validate_finite_rpy(
    const std::array<double, 3>& rpy,
    std::string_view base_pointer) {
  for (std::size_t i = 0; i < 3; ++i) {
    if (!std::isfinite(rpy[i])) {
      return load_error{
          load_error_kind::domain_error,
          {},
          std::string(base_pointer) + "/" + std::to_string(i),
          "rpy_rad component " + std::to_string(i) + " is not finite",
      };
    }
  }
  return std::nullopt;
}

// Inverse of compose_origin per D-VD-3.
//
// compose_origin builds R = R_z(yaw) * R_y(pitch) * R_x(roll), so:
//   m[0][0] =  cy * cp           m[0][1] =  sy * cp           m[0][2] = -sp
//   m[1][2] =  cp * sr           m[2][2] =  cp * cr
// Therefore (away from the gimbal pole, |sp| != 1):
//   pitch = -asin(m[0][2])
//   yaw   = atan2(m[0][1], m[0][0])
//   roll  = atan2(m[1][2], m[2][2])
//
// At the gimbal pole (sp == ±1, so cp == 0) yaw and roll are
// underdetermined; we pin yaw = 0 and absorb the residual rotation
// into roll. With cp = 0 and yaw = 0, compose's column 1 / column 2
// first rows reduce to:
//   m[1][0] = sp * sr,  m[2][0] = sp * cr  (since cy = 1, sy = 0)
// so sr = sp * m[1][0], cr = sp * m[2][0], and
// roll = atan2(sp * m[1][0], sp * m[2][0]).
// At sp = +1 this is atan2(m[1][0], m[2][0]); at sp = -1 this is
// atan2(-m[1][0], -m[2][0]) — both equivalent to "roll - yaw"
// since the input's roll and yaw collapse to that single residual
// at the pole.
origin_pose decompose_origin(const transform_4x4& m) {
  origin_pose out{};
  // Translation column.
  out.xyz_m = {m[3][0], m[3][1], m[3][2]};

  // Pole gate: |m[0][2]| within 1 - 1e-12 of unity.
  constexpr double pole_epsilon = 1e-12;
  const double sin_pitch = -m[0][2];
  if (std::abs(sin_pitch) > 1.0 - pole_epsilon) {
    // At the pole. yaw = 0 by convention.
    const double sp = sin_pitch >= 0.0 ? 1.0 : -1.0;
    out.rpy_rad[1] = sp * (M_PI / 2.0);
    out.rpy_rad[2] = 0.0;
    out.rpy_rad[0] = std::atan2(sp * m[1][0], sp * m[2][0]);
    return out;
  }

  // Away from the pole.
  out.rpy_rad[1] = std::asin(sin_pitch);  // pitch = asin(-m[0][2])
  out.rpy_rad[2] = std::atan2(m[0][1], m[0][0]);
  out.rpy_rad[0] = std::atan2(m[1][2], m[2][2]);
  // Normalize signed zeros: when the input matrix has m[0][2] = +0.0,
  // the negation -m[0][2] produces -0.0, and asin(-0.0) = -0.0. The
  // serializer's bit-exact identity check (origin_is_identity in
  // serializer.cpp) rejects -0.0 as non-identity, so an identity-input
  // round-trip would emit a spurious origin key. `x + 0.0` collapses
  // -0.0 to +0.0 in IEEE-754 without affecting any other value.
  for (std::size_t i = 0; i < 3; ++i) {
    out.rpy_rad[i] += 0.0;
  }
  return out;
}

}  // namespace robosim::description
