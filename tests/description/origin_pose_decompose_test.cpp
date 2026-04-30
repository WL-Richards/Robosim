// Phase VD section O — decompose_origin contract.
// See tests/viz/TEST_PLAN_VD.md "O. Decompose helper".

#include "origin_pose.h"

#include "schema.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>

namespace robosim::description {
namespace {

constexpr double tol_1e_12 = 1e-12;
constexpr double tol_1e_9 = 1e-9;

void expect_matrix_near(const transform_4x4& a, const transform_4x4& b, double tol) {
  for (std::size_t col = 0; col < 4; ++col) {
    for (std::size_t row = 0; row < 4; ++row) {
      EXPECT_NEAR(a[col][row], b[col][row], tol)
          << "mismatch at m[" << col << "][" << row << "]";
    }
  }
}

// ---------------------------------------------------------------------
// O1 — translation extraction (parameterized).
// ---------------------------------------------------------------------

class DecomposeTranslation
    : public ::testing::TestWithParam<std::array<double, 3>> {};

TEST_P(DecomposeTranslation, ReproducesXyzFromTranslationColumn) {
  const auto xyz = GetParam();
  const auto m = compose_origin({.xyz_m = xyz, .rpy_rad = {0.0, 0.0, 0.0}});

  const auto decomposed = decompose_origin(m);

  EXPECT_EQ(decomposed.xyz_m, xyz);
}

INSTANTIATE_TEST_SUITE_P(
    Cases,
    DecomposeTranslation,
    ::testing::Values(
        std::array<double, 3>{0.0, 0.0, 0.0},
        std::array<double, 3>{1.0, 0.0, 0.0},
        std::array<double, 3>{0.0, -2.5, 0.0},
        std::array<double, 3>{0.1, 0.2, 0.3},
        std::array<double, 3>{-1e3, 1e3, 1e3}));

// ---------------------------------------------------------------------
// O2 — single-axis rotations (parameterized).
// ---------------------------------------------------------------------

class DecomposeSingleAxis
    : public ::testing::TestWithParam<std::array<double, 3>> {};

TEST_P(DecomposeSingleAxis, ReproducesRpyForSingleAxisRotations) {
  const auto rpy = GetParam();
  const auto m = compose_origin({.xyz_m = {0.0, 0.0, 0.0}, .rpy_rad = rpy});

  const auto decomposed = decompose_origin(m);

  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_NEAR(decomposed.rpy_rad[i], rpy[i], tol_1e_12)
        << "rpy_rad[" << i << "] mismatch";
  }
}

INSTANTIATE_TEST_SUITE_P(
    SingleAxis,
    DecomposeSingleAxis,
    ::testing::Values(
        std::array<double, 3>{+M_PI / 6.0, 0.0, 0.0},
        std::array<double, 3>{-M_PI / 6.0, 0.0, 0.0},
        std::array<double, 3>{0.0, +M_PI / 6.0, 0.0},
        std::array<double, 3>{0.0, -M_PI / 6.0, 0.0},
        std::array<double, 3>{0.0, 0.0, +M_PI / 6.0},
        std::array<double, 3>{0.0, 0.0, -M_PI / 6.0}));

// ---------------------------------------------------------------------
// O3 — decompose-then-compose round-trips away from pole (parameterized).
// ---------------------------------------------------------------------

struct AwayFromPoleCase {
  std::array<double, 3> xyz_m;
  std::array<double, 3> rpy_rad;
};

class DecomposeRoundTrip : public ::testing::TestWithParam<AwayFromPoleCase> {};

TEST_P(DecomposeRoundTrip, DecomposeThenComposeRoundTripsAwayFromPole) {
  const auto& c = GetParam();
  const auto m = compose_origin({.xyz_m = c.xyz_m, .rpy_rad = c.rpy_rad});

  const auto m_round = compose_origin(decompose_origin(m));

  expect_matrix_near(m, m_round, tol_1e_12);
}

INSTANTIATE_TEST_SUITE_P(
    AwayFromPole,
    DecomposeRoundTrip,
    ::testing::Values(
        AwayFromPoleCase{{0.1, -0.2, 0.3}, {+M_PI / 4.0, +M_PI / 6.0, +M_PI / 3.0}},
        AwayFromPoleCase{{0.1, -0.2, 0.3}, {-M_PI / 4.0, +M_PI / 6.0, -M_PI / 3.0}},
        AwayFromPoleCase{{0.1, -0.2, 0.3}, {+M_PI / 4.0, -M_PI / 6.0, +M_PI / 3.0}},
        AwayFromPoleCase{{0.1, -0.2, 0.3}, {+0.1, +0.2, +0.3}}));

// ---------------------------------------------------------------------
// O4 — identity decomposes to zero pose.
// ---------------------------------------------------------------------

TEST(OriginPose, DecomposeIdentityYieldsZeroPose) {
  transform_4x4 identity{};
  identity[0][0] = 1.0;
  identity[1][1] = 1.0;
  identity[2][2] = 1.0;
  identity[3][3] = 1.0;

  const auto decomposed = decompose_origin(identity);

  EXPECT_EQ(decomposed.xyz_m, (std::array<double, 3>{0.0, 0.0, 0.0}));
  EXPECT_EQ(decomposed.rpy_rad, (std::array<double, 3>{0.0, 0.0, 0.0}));
}

// ---------------------------------------------------------------------
// O5 — gimbal pole preserves composed transform (rpy_rad non-unique).
// ---------------------------------------------------------------------

TEST(OriginPose, DecomposeAtGimbalPolePreservesComposedTransform) {
  const auto m = compose_origin(
      {.xyz_m = {0.0, 0.0, 0.0}, .rpy_rad = {+0.4, +M_PI / 2.0, -0.3}});

  const auto decomposed = decompose_origin(m);
  const auto m_round = compose_origin(decomposed);

  expect_matrix_near(m, m_round, tol_1e_9);

  // Output must be finite (no NaN poisoning). validate_finite_*
  // returns std::nullopt when every element is finite.
  EXPECT_FALSE(validate_finite_xyz(decomposed.xyz_m, "/test/xyz_m"));
  EXPECT_FALSE(validate_finite_rpy(decomposed.rpy_rad, "/test/rpy_rad"));
}

}  // namespace
}  // namespace robosim::description
