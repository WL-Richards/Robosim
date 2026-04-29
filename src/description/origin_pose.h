#pragma once

// Origin-pose helpers: composition + per-scalar finite validation.
// See tests/description/TEST_PLAN_VC.md decisions D-VC-3 and D-VC-5a.
//
// validate_finite_xyz / validate_finite_rpy are the loader's
// SOLE non-finite-rejection mechanism (D-VC-3 seam call-site pin,
// enforced by cmake/lint_loader_finite_seam.cmake at configure
// time). Bare std::isfinite/isnan/isinf/isnormal/fpclassify calls
// in any other src/description/*.{h,cpp} are forbidden.

#include "error.h"
#include "schema.h"

#include <array>
#include <optional>
#include <string_view>

namespace robosim::description {

// 4x4 column-major homogeneous transform: m[col][row].
using transform_4x4 = std::array<std::array<double, 4>, 4>;

// Compose an origin_pose into a 4x4 transform per D-VC-5a:
// R = R_z(rpy_rad[2]) * R_y(rpy_rad[1]) * R_x(rpy_rad[0])
// applied to column vectors, i.e. intrinsic-Z-Y'-X'' (URDF).
// Translation column populated from xyz_m.
[[nodiscard]] transform_4x4 compose_origin(const origin_pose& p);

// Validate that every element of an xyz_m array is finite. On the
// first non-finite element, returns a domain_error with json_pointer
// `<base_pointer>/<slot>` (D-VC-1b per-element pointer convention).
// `base_pointer` is the pointer of the array itself, e.g.
// "/joints/0/origin/xyz_m". The returned error has an empty
// file_path; the caller fills it in before propagating.
//
// Returns std::nullopt when every element is finite.
[[nodiscard]] std::optional<load_error> validate_finite_xyz(
    const std::array<double, 3>& xyz, std::string_view base_pointer);

// Symmetric to validate_finite_xyz for rpy_rad. The base_pointer is
// the rpy_rad array's pointer, e.g. "/joints/0/origin/rpy_rad".
[[nodiscard]] std::optional<load_error> validate_finite_rpy(
    const std::array<double, 3>& rpy, std::string_view base_pointer);

// Algebraic inverse of compose_origin (Phase VD; D-VD-3).
//
// Extracts xyz_m from the translation column (m[3][0..2]) and rpy_rad
// via intrinsic-Z-Y'-X'' Euler decomposition (URDF convention,
// matching compose_origin / D-VC-5a).
//
// At the gimbal pole (|m[0][2]| > 1 - 1e-12) the decomposition is
// non-unique; this function returns one valid solution (yaw == 0 by
// convention; roll absorbs the residual rotation). Callers that
// round-trip via compose_origin recover the same transform within
// 1e-9 at the pole, 1e-12 away from it. Callers that compare
// rpy_rad component-wise must be aware of the gimbal-pole branch.
//
// See tests/viz/TEST_PLAN_VD.md section O for the pinned contract.
[[nodiscard]] origin_pose decompose_origin(const transform_4x4& m);

}  // namespace robosim::description
