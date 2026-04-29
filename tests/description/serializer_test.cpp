// Phase VC serializer tests.
//
// Implements S1a/S1/S2/S3/S4/S5/S6/S8 from
// tests/description/TEST_PLAN_VC.md "VC4 — serializer test plan".
// S7 was dropped in rev 3; non-atomicity is documented in the loader
// skill, not pinned by a test.

#include "loader.h"
#include "serializer.h"

#include "schema.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <ios>
#include <limits>
#include <numbers>
#include <sstream>
#include <string>
#include <system_error>

namespace robosim::description {
namespace {

namespace fs = std::filesystem;
using nlohmann::ordered_json;
using ::testing::HasSubstr;

#ifndef ROBOSIM_V0_ARM_FIXTURE_PATH
#error "ROBOSIM_V0_ARM_FIXTURE_PATH not defined; CMake target setup is broken."
#endif

constexpr const char* v0_arm_fixture_path = ROBOSIM_V0_ARM_FIXTURE_PATH;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

class serializer_test : public ::testing::Test {
 protected:
  void SetUp() override {
    const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
    tmp_dir_ = fs::temp_directory_path() /
               (std::string("robosim_serializer_test_") + info->test_suite_name() + "_" +
                info->name());
    fs::remove_all(tmp_dir_);
    fs::create_directories(tmp_dir_);
  }

  void TearDown() override {
    std::error_code ec;
    fs::remove_all(tmp_dir_, ec);
  }

  std::string slurp(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
  }

  fs::path tmp_dir_;
};

robot_description load_v0_arm_v2_or_die() {
  auto loaded = load_from_file(v0_arm_fixture_path);
  EXPECT_TRUE(loaded.has_value())
      << "v0-arm fixture failed to load: "
      << (loaded ? "" : loaded.error().message);
  return loaded ? *loaded : robot_description{};
}

// SHA-256 — small, dependency-free implementation for the S8 guard.
// Public-domain-style; correctness pinned by S8 itself (a wrong
// implementation would just produce a different pinned digest).
struct sha256 {
  std::uint32_t h[8] = {0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
                        0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U};
  std::uint64_t bit_length = 0;
  std::uint8_t buffer[64] = {};
  std::size_t buffer_len = 0;

  static std::uint32_t rotr(std::uint32_t x, std::uint32_t n) {
    return (x >> n) | (x << (32 - n));
  }

  void process_block(const std::uint8_t* block) {
    static constexpr std::uint32_t k[64] = {
        0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U,
        0x923f82a4U, 0xab1c5ed5U, 0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
        0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U, 0xe49b69c1U, 0xefbe4786U,
        0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
        0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U,
        0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
        0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U, 0xa2bfe8a1U, 0xa81a664bU,
        0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
        0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU,
        0x5b9cca4fU, 0x682e6ff3U, 0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
        0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

    std::uint32_t w[64];
    for (int i = 0; i < 16; ++i) {
      w[i] = (static_cast<std::uint32_t>(block[i * 4]) << 24) |
             (static_cast<std::uint32_t>(block[i * 4 + 1]) << 16) |
             (static_cast<std::uint32_t>(block[i * 4 + 2]) << 8) |
             (static_cast<std::uint32_t>(block[i * 4 + 3]));
    }
    for (int i = 16; i < 64; ++i) {
      const std::uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
      const std::uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }

    std::uint32_t a = h[0], b = h[1], c = h[2], d = h[3];
    std::uint32_t e = h[4], f = h[5], g = h[6], hh = h[7];

    for (int i = 0; i < 64; ++i) {
      const std::uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      const std::uint32_t ch = (e & f) ^ (~e & g);
      const std::uint32_t t1 = hh + s1 + ch + k[i] + w[i];
      const std::uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      const std::uint32_t mj = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t t2 = s0 + mj;
      hh = g; g = f; f = e; e = d + t1;
      d = c; c = b; b = a; a = t1 + t2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d;
    h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
  }

  void update(const std::uint8_t* data, std::size_t len) {
    bit_length += static_cast<std::uint64_t>(len) * 8U;
    while (len > 0) {
      const std::size_t take = std::min(len, std::size_t{64} - buffer_len);
      std::memcpy(buffer + buffer_len, data, take);
      buffer_len += take;
      data += take;
      len -= take;
      if (buffer_len == 64) {
        process_block(buffer);
        buffer_len = 0;
      }
    }
  }

  std::string hex_digest() {
    buffer[buffer_len++] = 0x80;
    if (buffer_len > 56) {
      while (buffer_len < 64) buffer[buffer_len++] = 0;
      process_block(buffer);
      buffer_len = 0;
    }
    while (buffer_len < 56) buffer[buffer_len++] = 0;
    for (int i = 7; i >= 0; --i) {
      buffer[buffer_len++] = static_cast<std::uint8_t>((bit_length >> (i * 8)) & 0xFFU);
    }
    process_block(buffer);

    std::ostringstream ss;
    ss << std::hex << std::setfill('0');
    for (int i = 0; i < 8; ++i) {
      ss << std::setw(8) << h[i];
    }
    return ss.str();
  }
};

std::string sha256_hex(std::string_view bytes) {
  sha256 s;
  s.update(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  return s.hex_digest();
}

// ===========================================================================
// S1a — same input twice → byte-identical output
// ===========================================================================

TEST_F(serializer_test, S1a_same_input_yields_byte_identical_output_twice) {
  const auto d = load_v0_arm_v2_or_die();
  const auto path_a = tmp_dir_ / "a.json";
  const auto path_b = tmp_dir_ / "b.json";

  ASSERT_TRUE(save_to_file(d, path_a).has_value());
  ASSERT_TRUE(save_to_file(d, path_b).has_value());

  EXPECT_EQ(slurp(path_a), slurp(path_b));
}

// ===========================================================================
// S1 — full round-trip byte-identical
// ===========================================================================

TEST_F(serializer_test, S1_round_trip_v0_arm_v2_yields_byte_identical_file) {
  const auto d = load_v0_arm_v2_or_die();
  const auto path_a = tmp_dir_ / "a.json";
  const auto path_b = tmp_dir_ / "b.json";

  ASSERT_TRUE(save_to_file(d, path_a).has_value());
  const auto reloaded = load_from_file(path_a);
  ASSERT_TRUE(reloaded.has_value()) << reloaded.error().message;
  ASSERT_TRUE(save_to_file(*reloaded, path_b).has_value());

  EXPECT_EQ(slurp(path_a), slurp(path_b));
}

// ===========================================================================
// S2 — modified joint origin persists through save → load
// ===========================================================================

TEST_F(serializer_test, S2_round_trip_with_modified_joint_origin_persists_value) {
  auto d = load_v0_arm_v2_or_die();
  d.joints[0].origin.xyz_m = {0.1, 0.2, 0.3};

  const auto path = tmp_dir_ / "modified.json";
  ASSERT_TRUE(save_to_file(d, path).has_value());

  const auto reloaded = load_from_file(path);
  ASSERT_TRUE(reloaded.has_value()) << reloaded.error().message;

  // Exact equality, not tolerance: %.17g is a lossless round-trip
  // for IEEE-754 doubles regardless of decimal representability —
  // the bit pattern of the unique IEEE-754 approximation of 0.1
  // (etc.) is written and re-read losslessly.
  EXPECT_EQ(reloaded->joints[0].origin.xyz_m, (std::array{0.1, 0.2, 0.3}));
}

// ===========================================================================
// S3 — identity origin omitted from output
// ===========================================================================

TEST_F(serializer_test, S3_identity_origin_is_omitted_from_output) {
  const auto d = load_v0_arm_v2_or_die();
  const auto path = tmp_dir_ / "identity.json";
  ASSERT_TRUE(save_to_file(d, path).has_value());

  const auto bytes = slurp(path);
  const auto parsed = ordered_json::parse(bytes);

  ASSERT_TRUE(parsed.contains("joints"));
  for (const auto& j : parsed.at("joints")) {
    EXPECT_FALSE(j.contains("origin")) << "joint object should omit identity origin: " << j.dump();
  }
  ASSERT_TRUE(parsed.contains("links"));
  for (const auto& l : parsed.at("links")) {
    EXPECT_FALSE(l.contains("visual_origin"))
        << "link object should omit identity visual_origin: " << l.dump();
  }
}

// ===========================================================================
// S4 — non-identity origin emitted in canonical key order
// ===========================================================================

TEST_F(serializer_test, S4_non_identity_origin_is_emitted_in_canonical_key_order) {
  auto d = load_v0_arm_v2_or_die();
  d.joints[0].origin.xyz_m = {1.0, 2.0, 3.0};

  const auto path = tmp_dir_ / "ordered.json";
  ASSERT_TRUE(save_to_file(d, path).has_value());

  const auto parsed = ordered_json::parse(slurp(path));
  std::vector<std::string> joint_keys;
  for (auto it = parsed.at("joints").at(0).begin();
       it != parsed.at("joints").at(0).end(); ++it) {
    joint_keys.push_back(it.key());
  }
  EXPECT_EQ(joint_keys, (std::vector<std::string>{
                            "name", "type", "parent", "child", "axis",
                            "limit_lower_rad", "limit_upper_rad",
                            "viscous_friction_nm_per_rad_per_s", "origin"}));
}

// ===========================================================================
// S5 — adversarial float corners round-trip bit-equal
// ===========================================================================

class S5AdversarialFloats : public serializer_test,
                            public ::testing::WithParamInterface<double> {};

TEST_P(S5AdversarialFloats, S5_round_trips_adversarial_float_corners) {
  const double v = GetParam();
  auto d = load_v0_arm_v2_or_die();
  d.joints[0].origin.xyz_m = {v, 0.0, 0.0};

  const auto path = tmp_dir_ / "adversarial.json";
  ASSERT_TRUE(save_to_file(d, path).has_value());

  const auto reloaded = load_from_file(path);
  ASSERT_TRUE(reloaded.has_value()) << reloaded.error().message;

  // Bit-equal, not ==. == on -0.0 and +0.0 returns true and would
  // hide a sign-bit drop.
  EXPECT_EQ(std::bit_cast<std::uint64_t>(reloaded->joints[0].origin.xyz_m[0]),
            std::bit_cast<std::uint64_t>(v))
      << "round-trip drift: input " << v << ", reloaded "
      << reloaded->joints[0].origin.xyz_m[0];
}

INSTANTIATE_TEST_SUITE_P(
    AdversarialFloats, S5AdversarialFloats,
    ::testing::Values(
        std::numeric_limits<double>::denorm_min(),
        std::numeric_limits<double>::min(),
        std::numeric_limits<double>::max(),
        std::nextafter(1.0, 2.0),
        0.1,
        1e-300,
        1e+300,
        -0.0,
        std::numbers::pi,
        std::nextafter(1.0, 0.0)));

// ===========================================================================
// S6 — unwritable path returns io_error
// ===========================================================================

TEST_F(serializer_test, S6_unwritable_path_returns_io_error) {
  // Make the temp dir read+execute only so writes fail.
  fs::permissions(tmp_dir_,
                  fs::perms::owner_read | fs::perms::owner_exec,
                  fs::perm_options::replace);

  const auto path = tmp_dir_ / "out.json";
  const auto result = save_to_file(load_v0_arm_v2_or_die(), path);

  // Restore perms so TearDown can remove the dir.
  fs::permissions(tmp_dir_,
                  fs::perms::owner_all,
                  fs::perm_options::replace);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().kind, save_error_kind::io_error);
  EXPECT_EQ(result.error().file_path, path);
  EXPECT_FALSE(result.error().message.empty());
}

// ===========================================================================
// S8 — nlohmann dump byte-format SHA guard
// ===========================================================================

TEST(SerializerSHAGuard, S8_nlohmann_dump_byte_format_matches_pinned_sha) {
  // Hand-authored reference value covering the surface that matters
  // for byte-stability: ints, signed/unsigned, floats including
  // classic adversarial values (0.1, 1e-300, M_PI), nested array,
  // nested object. ordered_json preserves insertion order.
  ordered_json j = ordered_json::object();
  j["a_int"] = 42;
  j["b_neg_int"] = -7;
  j["c_zero"] = 0;
  j["d_float_simple"] = 1.5;
  j["e_float_classic"] = 0.1;
  j["f_float_small"] = 1e-300;
  j["g_float_pi"] = std::numbers::pi;
  j["h_array"] = {1, 2, 3};
  j["i_nested"] = ordered_json::object();
  j["i_nested"]["x"] = "hello";
  j["i_nested"]["y"] = false;

  const std::string dump = j.dump(2);
  const std::string digest = sha256_hex(dump);

  // PINNED. If this test fails, nlohmann's dump format changed —
  // verify the new dump is what we want, then update both this
  // constant and the FetchContent SHA in CMakeLists.txt.
  // Initial value is intentionally a placeholder; the implementing
  // session computes the actual digest against the pinned nlohmann
  // v3.12 build and pastes it here. See D-VC-10 / S8.
  static constexpr const char* kPinnedDigest =
      "73fd52187cebcec93ff245f959c58b16f2ef1ff89604aaf64aabac8a0a5edeae";

  EXPECT_EQ(digest, kPinnedDigest)
      << "nlohmann::ordered_json::dump(2) format changed.\n"
      << "Re-run this test to print the new digest below, then update\n"
      << "kPinnedDigest after a human verifies the dump output.\n"
      << "Actual dump:\n" << dump << "\n"
      << "Actual digest: " << digest;
}

}  // namespace
}  // namespace robosim::description
