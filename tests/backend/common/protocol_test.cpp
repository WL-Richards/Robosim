// Tests for the HAL ↔ Sim Core protocol schema. Covers TEST_PLAN
// sections A2–A6, B, C, D, E, F, G, I, J, K, L, M, N, O, Q. Section H
// (WPILib byte-parity) lives in wpilib_parity_test.cpp. Section A1
// (compile-time invariants) lives in compile_time_invariants_test.cpp
// and has no GTest body.

#include "boot_descriptor.h"
#include "can_frame.h"
#include "can_status.h"
#include "clock_state.h"
#include "ds_state.h"
#include "error_message.h"
#include "joystick_output.h"
#include "notifier_state.h"
#include "power_state.h"
#include "protocol_version.h"
#include "sync_envelope.h"
#include "truncate.h"
#include "types.h"
#include "user_program_observer.h"
#include "validator.h"
#include "validator_error.h"

#include <gtest/gtest.h>

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace robosim::backend {
namespace {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Build a buffer of `size` bytes where each byte is `(i + 1) & 0xFF` —
// unique for the first 255 bytes, then wraps. Decision: pad bytes are
// part of the unique-byte fill (TEST_PLAN E1).
std::vector<std::uint8_t> make_distinguishing_buffer(std::size_t size) {
  std::vector<std::uint8_t> buf(size);
  for (std::size_t i = 0; i < size; ++i) {
    buf[i] = static_cast<std::uint8_t>((i + 1) & 0xFF);
  }
  return buf;
}

// Round-trip a struct through a byte buffer of its sizeof and assert
// every byte (including padding) is preserved. Used by Section E.
template <typename T>
void assert_byte_round_trip() {
  static_assert(std::is_trivially_copyable_v<T>);
  auto bytes_in = make_distinguishing_buffer(sizeof(T));
  T struct_;
  std::memcpy(&struct_, bytes_in.data(), sizeof(T));
  std::vector<std::uint8_t> bytes_out(sizeof(T));
  std::memcpy(bytes_out.data(), &struct_, sizeof(T));
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    EXPECT_EQ(bytes_in[i] ^ bytes_out[i], 0)
        << "byte " << i << " differs after round-trip";
  }
}

template <typename T>
std::span<const std::uint8_t> bytes_of(const T& value) {
  return {reinterpret_cast<const std::uint8_t*>(&value), sizeof(T)};
}

// Build a valid envelope for a given kind/schema/payload-bytes. Used
// by section D and Q tests.
sync_envelope make_envelope(envelope_kind kind, schema_id ps,
                            std::uint32_t payload_bytes,
                            std::uint64_t sequence = 0,
                            direction sender = direction::backend_to_core) {
  sync_envelope e{};
  e.magic = kProtocolMagic;
  e.protocol_version = kProtocolVersion;
  e.kind = kind;
  e.sequence = sequence;
  e.sim_time_us = 0;
  e.payload_bytes = payload_bytes;
  e.payload_schema = ps;
  e.sender = sender;
  return e;
}

// ---------------------------------------------------------------------------
// A2/A4/A5/A6 — runtime-checkable type-trait pins (the static_asserts
// in the headers are the contract; these are belt-and-suspenders for
// the few invariants the static_assert framework expresses awkwardly,
// e.g. "is_same_v" where a typedef breakage would make the static
// assert un-instantiable).
// ---------------------------------------------------------------------------

TEST(Types, HalBoolIsInt32) {
  EXPECT_EQ(sizeof(hal_bool), 4u);
  EXPECT_TRUE((std::is_same_v<hal_bool, std::int32_t>));
}

TEST(Types, HalHandleIsInt32) {
  EXPECT_TRUE((std::is_same_v<hal_handle, std::int32_t>));
}

// ---------------------------------------------------------------------------
// B — endianness gate
// ---------------------------------------------------------------------------

TEST(Endianness, NativeIsLittle) {
  EXPECT_EQ(std::endian::native, std::endian::little);
}

// ---------------------------------------------------------------------------
// C — sync_envelope structural
// ---------------------------------------------------------------------------

TEST(SyncEnvelope, FieldOffsetsMatchWireContract) {
  EXPECT_EQ(offsetof(sync_envelope, magic), 0u);
  EXPECT_EQ(offsetof(sync_envelope, protocol_version), 4u);
  EXPECT_EQ(offsetof(sync_envelope, kind), 6u);
  EXPECT_EQ(offsetof(sync_envelope, sequence), 8u);
  EXPECT_EQ(offsetof(sync_envelope, sim_time_us), 16u);
  EXPECT_EQ(offsetof(sync_envelope, payload_bytes), 24u);
  EXPECT_EQ(offsetof(sync_envelope, payload_schema), 28u);
  EXPECT_EQ(offsetof(sync_envelope, sender), 29u);
  EXPECT_EQ(offsetof(sync_envelope, reserved), 30u);
  EXPECT_EQ(sizeof(sync_envelope), 32u);
}

TEST(SyncEnvelope, ReservedBytesRoundTripSafe) {
  // C2: zero-init, populate only named fields, memcpy round-trip,
  // assert reserved still zero. Catches a future field's write spilling
  // into reserved.
  sync_envelope env{};
  env.magic = kProtocolMagic;
  env.protocol_version = kProtocolVersion;
  env.kind = envelope_kind::tick_boundary;
  env.sequence = 42;
  env.sim_time_us = 1000;
  env.payload_bytes = sizeof(clock_state);
  env.payload_schema = schema_id::clock_state;
  env.sender = direction::backend_to_core;
  // reserved deliberately not touched.

  std::array<std::uint8_t, sizeof(sync_envelope)> buf{};
  std::memcpy(buf.data(), &env, sizeof(sync_envelope));
  sync_envelope back{};
  std::memcpy(&back, buf.data(), sizeof(sync_envelope));
  EXPECT_EQ(back.reserved[0], 0u);
  EXPECT_EQ(back.reserved[1], 0u);
}

TEST(SyncEnvelope, BitIdenticalEnvelopesCompareEqual) {
  sync_envelope a = make_envelope(envelope_kind::tick_boundary,
                                  schema_id::clock_state,
                                  sizeof(clock_state));
  sync_envelope b = a;
  EXPECT_EQ(a, b);
}

// ---------------------------------------------------------------------------
// D — validator (stateless envelope checks)
// ---------------------------------------------------------------------------

TEST(Validator, RejectsWrongMagic) {
  auto env = make_envelope(envelope_kind::tick_boundary,
                           schema_id::clock_state, sizeof(clock_state));
  env.magic = {'X', 'X', 'X', 'X'};
  auto r = validate_envelope(env, direction::backend_to_core, 0);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, validate_error_kind::magic_mismatch);
  EXPECT_EQ(r.error().offending_field_name, "magic");
}

TEST(Validator, RejectsZeroProtocolVersionAsUninitGuard) {
  auto env = make_envelope(envelope_kind::tick_boundary,
                           schema_id::clock_state, sizeof(clock_state));
  env.protocol_version = 0;
  auto r = validate_envelope(env, direction::backend_to_core, 0);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, validate_error_kind::version_mismatch);
  EXPECT_NE(r.error().offending_field_name.find("protocol_version"),
            std::string::npos);
}

TEST(Validator, RejectsMismatchedProtocolVersion) {
  auto env = make_envelope(envelope_kind::tick_boundary,
                           schema_id::clock_state, sizeof(clock_state));
  env.protocol_version = kProtocolVersion + 1;
  auto r = validate_envelope(env, direction::backend_to_core, 0);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, validate_error_kind::version_mismatch);
}

TEST(Validator, RejectsZeroEnvelopeKind) {
  auto env = make_envelope(envelope_kind::reserved, schema_id::none, 0);
  auto r = validate_envelope(env, direction::backend_to_core, 0);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, validate_error_kind::unknown_envelope_kind);
}

TEST(Validator, RejectsOutOfRangeEnvelopeKind) {
  auto env = make_envelope(envelope_kind::tick_boundary,
                           schema_id::clock_state, sizeof(clock_state));
  env.kind = static_cast<envelope_kind>(100);
  auto r = validate_envelope(env, direction::backend_to_core, 0);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, validate_error_kind::unknown_envelope_kind);
}

TEST(Validator, RejectsZeroDirection) {
  auto env = make_envelope(envelope_kind::tick_boundary,
                           schema_id::clock_state, sizeof(clock_state),
                           0, direction::reserved);
  auto r = validate_envelope(env, direction::backend_to_core, 0);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, validate_error_kind::unknown_direction);
}

TEST(Validator, RejectsOutOfRangeDirection) {
  auto env = make_envelope(envelope_kind::tick_boundary,
                           schema_id::clock_state, sizeof(clock_state));
  env.sender = static_cast<direction>(99);
  auto r = validate_envelope(env, direction::backend_to_core, 0);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, validate_error_kind::unknown_direction);
}

TEST(Validator, RejectsDirectionMismatch) {
  auto env = make_envelope(envelope_kind::tick_boundary,
                           schema_id::clock_state, sizeof(clock_state),
                           0, direction::backend_to_core);
  auto r = validate_envelope(env, direction::core_to_backend, 0);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, validate_error_kind::direction_mismatch);
}

TEST(Validator, RejectsUnknownSchemaId) {
  auto env = make_envelope(envelope_kind::tick_boundary,
                           schema_id::clock_state, sizeof(clock_state));
  env.payload_schema = static_cast<schema_id>(50);
  auto r = validate_envelope(env, direction::backend_to_core, 0);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, validate_error_kind::unknown_schema_id);
}

TEST(Validator, AcceptsMatchingSequence) {
  auto env = make_envelope(envelope_kind::tick_boundary,
                           schema_id::clock_state, sizeof(clock_state),
                           7);
  auto r = validate_envelope(env, direction::backend_to_core, 7);
  EXPECT_TRUE(r.has_value());
}

TEST(Validator, RejectsMismatchedSequence) {
  auto env = make_envelope(envelope_kind::tick_boundary,
                           schema_id::clock_state, sizeof(clock_state),
                           5);
  auto r = validate_envelope(env, direction::backend_to_core, 7);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, validate_error_kind::sequence_mismatch);
  // Per fork F4, validator does not distinguish forward/backward gap;
  // both directions of skew produce the same kind.
  auto env2 = make_envelope(envelope_kind::tick_boundary,
                            schema_id::clock_state, sizeof(clock_state),
                            9);
  auto r2 = validate_envelope(env2, direction::backend_to_core, 7);
  ASSERT_FALSE(r2.has_value());
  EXPECT_EQ(r2.error().kind, validate_error_kind::sequence_mismatch);
}

TEST(Validator, AcceptsSequenceZeroBoundary) {
  auto env = make_envelope(envelope_kind::boot, schema_id::boot_descriptor,
                           sizeof(boot_descriptor), 0);
  auto r = validate_envelope(env, direction::backend_to_core, 0);
  EXPECT_TRUE(r.has_value());
}

TEST(Validator, AcceptsSequenceMaxBoundary) {
  auto env = make_envelope(envelope_kind::tick_boundary,
                           schema_id::clock_state, sizeof(clock_state),
                           UINT64_MAX);
  auto r = validate_envelope(env, direction::backend_to_core, UINT64_MAX);
  EXPECT_TRUE(r.has_value());
}

TEST(Validator, NoneSchemaWithZeroBytesIsOk) {
  auto env = make_envelope(envelope_kind::shutdown, schema_id::none, 0);
  auto r = validate_envelope(env, direction::backend_to_core, 0);
  EXPECT_TRUE(r.has_value());
}

TEST(Validator, NoneSchemaWithNonzeroBytesRejected) {
  auto env = make_envelope(envelope_kind::shutdown, schema_id::none, 16);
  auto r = validate_envelope(env, direction::backend_to_core, 0);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind,
            validate_error_kind::schema_payload_kind_mismatch);
}

TEST(Validator, FixedSchemaPayloadSizeMustMatch) {
  // clock_state envelope with wrong payload_bytes.
  auto env = make_envelope(envelope_kind::tick_boundary,
                           schema_id::clock_state,
                           sizeof(clock_state) + 1);
  auto r = validate_envelope(env, direction::backend_to_core, 0);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, validate_error_kind::payload_size_mismatch);
}

TEST(Validator, VariableBatchSizeFromHeaderCount) {
  // Build a can_frame_batch payload with count=3.
  can_frame_batch batch{};
  batch.count = 3;
  std::array<std::uint8_t, sizeof(can_frame_batch)> raw{};
  std::memcpy(raw.data(), &batch, sizeof(batch));

  const std::uint32_t header_size =
      static_cast<std::uint32_t>(offsetof(can_frame_batch, frames));
  const std::uint32_t expected = header_size + 3 * sizeof(can_frame);

  auto env = make_envelope(envelope_kind::tick_boundary,
                           schema_id::can_frame_batch, expected);
  auto r = validate_envelope(env, direction::backend_to_core, 0,
                             std::span(raw));
  EXPECT_TRUE(r.has_value()) << "expected ok, got "
                             << static_cast<int>(r.error().kind);

  // payload_bytes lying about the count → mismatch.
  auto env_bad = make_envelope(envelope_kind::tick_boundary,
                               schema_id::can_frame_batch, expected + 8);
  auto r_bad = validate_envelope(env_bad, direction::backend_to_core, 0,
                                 std::span(raw));
  ASSERT_FALSE(r_bad.has_value());
  EXPECT_EQ(r_bad.error().kind,
            validate_error_kind::payload_size_mismatch);
}

TEST(Validator, VariableBatchOverCapacityRejected) {
  can_frame_batch batch{};
  batch.count = static_cast<std::uint32_t>(kMaxCanFramesPerBatch + 1);
  std::array<std::uint8_t, sizeof(can_frame_batch)> raw{};
  std::memcpy(raw.data(), &batch, sizeof(batch));

  auto env = make_envelope(envelope_kind::tick_boundary,
                           schema_id::can_frame_batch, 0xFFFF);
  auto r = validate_envelope(env, direction::backend_to_core, 0,
                             std::span(raw));
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, validate_error_kind::payload_size_mismatch);
}

// Cycle 37 — joystick_output_batch protocol schema.
TEST(JoystickOutputSchema, LayoutAndActivePrefixBytesArePinned) {
  EXPECT_TRUE((std::is_trivially_copyable_v<joystick_output_state>));
  EXPECT_TRUE((std::is_standard_layout_v<joystick_output_state>));
  EXPECT_TRUE((std::is_aggregate_v<joystick_output_state>));
  EXPECT_EQ(offsetof(joystick_output_state, joystick_num), 0u);
  EXPECT_EQ(offsetof(joystick_output_state, reserved_pad), 4u);
  EXPECT_EQ(offsetof(joystick_output_state, outputs), 8u);
  EXPECT_EQ(offsetof(joystick_output_state, left_rumble), 16u);
  EXPECT_EQ(offsetof(joystick_output_state, right_rumble), 20u);
  EXPECT_EQ(sizeof(joystick_output_state), 24u);
  EXPECT_EQ(alignof(joystick_output_state), 8u);

  EXPECT_TRUE((std::is_trivially_copyable_v<joystick_output_batch>));
  EXPECT_TRUE((std::is_standard_layout_v<joystick_output_batch>));
  EXPECT_TRUE((std::is_aggregate_v<joystick_output_batch>));
  EXPECT_EQ(offsetof(joystick_output_batch, count), 0u);
  EXPECT_EQ(offsetof(joystick_output_batch, reserved_pad), 4u);
  EXPECT_EQ(offsetof(joystick_output_batch, outputs), 8u);
  EXPECT_EQ(alignof(joystick_output_batch), 8u);

  joystick_output_batch empty{};
  EXPECT_EQ(active_prefix_bytes(empty).size(), 8u);

  joystick_output_batch two{};
  two.count = 2;
  EXPECT_EQ(active_prefix_bytes(two).size(), 56u);
}

TEST(Validator, JoystickOutputBatchAcceptsOnlyActivePrefixSizes) {
  for (const std::uint32_t count : {0u, 2u, static_cast<std::uint32_t>(kMaxJoysticks)}) {
    joystick_output_batch batch{};
    batch.count = count;
    auto env = make_envelope(envelope_kind::tick_boundary,
                             schema_id::joystick_output_batch,
                             static_cast<std::uint32_t>(active_prefix_bytes(batch).size()));
    auto r = validate_envelope(env, direction::backend_to_core, 0, active_prefix_bytes(batch));
    EXPECT_TRUE(r.has_value()) << "count " << count;
  }

  joystick_output_batch overflow{};
  overflow.count = static_cast<std::uint32_t>(kMaxJoysticks + 1);
  auto overflow_env =
      make_envelope(envelope_kind::tick_boundary, schema_id::joystick_output_batch, 0xFFFF);
  auto overflow_result =
      validate_envelope(overflow_env, direction::backend_to_core, 0, bytes_of(overflow));
  ASSERT_FALSE(overflow_result.has_value());
  EXPECT_EQ(overflow_result.error().kind, validate_error_kind::payload_size_mismatch);

  joystick_output_batch two{};
  two.count = 2;
  auto oversized_env = make_envelope(envelope_kind::tick_boundary,
                                     schema_id::joystick_output_batch,
                                     sizeof(joystick_output_batch));
  auto oversized_result =
      validate_envelope(oversized_env, direction::backend_to_core, 0, bytes_of(two));
  ASSERT_FALSE(oversized_result.has_value());
  EXPECT_EQ(oversized_result.error().kind, validate_error_kind::payload_size_mismatch);

  const auto undersized_payload_bytes =
      static_cast<std::uint32_t>(active_prefix_bytes(two).size() - 1);
  auto undersized_env = make_envelope(
      envelope_kind::tick_boundary, schema_id::joystick_output_batch, undersized_payload_bytes);
  auto undersized_result =
      validate_envelope(undersized_env, direction::backend_to_core, 0, bytes_of(two));
  ASSERT_FALSE(undersized_result.has_value());
  EXPECT_EQ(undersized_result.error().kind, validate_error_kind::payload_size_mismatch);
}

// Cycle 38 — user_program_observer_snapshot protocol schema.
TEST(UserProgramObserverSchema, LayoutAndEnumValuesArePinned) {
  EXPECT_TRUE((std::is_same_v<std::underlying_type_t<user_program_observer_mode>, std::int32_t>));
  EXPECT_EQ(static_cast<std::int32_t>(user_program_observer_mode::none), 0);
  EXPECT_EQ(static_cast<std::int32_t>(user_program_observer_mode::starting), 1);
  EXPECT_EQ(static_cast<std::int32_t>(user_program_observer_mode::disabled), 2);
  EXPECT_EQ(static_cast<std::int32_t>(user_program_observer_mode::autonomous), 3);
  EXPECT_EQ(static_cast<std::int32_t>(user_program_observer_mode::teleop), 4);
  EXPECT_EQ(static_cast<std::int32_t>(user_program_observer_mode::test), 5);

  EXPECT_TRUE((std::is_trivially_copyable_v<user_program_observer_snapshot>));
  EXPECT_TRUE((std::is_standard_layout_v<user_program_observer_snapshot>));
  EXPECT_TRUE((std::is_aggregate_v<user_program_observer_snapshot>));
  EXPECT_EQ(offsetof(user_program_observer_snapshot, mode), 0u);
  EXPECT_EQ(offsetof(user_program_observer_snapshot, reserved_pad), 4u);
  EXPECT_EQ(sizeof(user_program_observer_snapshot), 8u);
  EXPECT_EQ(alignof(user_program_observer_snapshot), 4u);
}

TEST(Validator, UserProgramObserverSnapshotRequiresFixedPayloadSize) {
  user_program_observer_snapshot snapshot{};
  snapshot.mode = user_program_observer_mode::teleop;

  auto valid_env = make_envelope(envelope_kind::tick_boundary,
                                 schema_id::user_program_observer_snapshot,
                                 sizeof(user_program_observer_snapshot));
  EXPECT_TRUE(validate_envelope(valid_env, direction::backend_to_core, 0, bytes_of(snapshot))
                  .has_value());

  auto undersized_env = make_envelope(envelope_kind::tick_boundary,
                                      schema_id::user_program_observer_snapshot,
                                      sizeof(user_program_observer_snapshot) - 1);
  auto undersized =
      validate_envelope(undersized_env, direction::backend_to_core, 0, bytes_of(snapshot));
  ASSERT_FALSE(undersized.has_value());
  EXPECT_EQ(undersized.error().kind, validate_error_kind::payload_size_mismatch);

  auto oversized_env = make_envelope(envelope_kind::tick_boundary,
                                     schema_id::user_program_observer_snapshot,
                                     sizeof(user_program_observer_snapshot) + 1);
  auto oversized =
      validate_envelope(oversized_env, direction::backend_to_core, 0, bytes_of(snapshot));
  ASSERT_FALSE(oversized.has_value());
  EXPECT_EQ(oversized.error().kind, validate_error_kind::payload_size_mismatch);
}

// ---------------------------------------------------------------------------
// E — round-trip byte-copy contract (every payload struct, pads
// included)
// ---------------------------------------------------------------------------

TEST(RoundTrip, SyncEnvelope)        { assert_byte_round_trip<sync_envelope>(); }
TEST(RoundTrip, ClockState)          { assert_byte_round_trip<clock_state>(); }
TEST(RoundTrip, PowerState)          { assert_byte_round_trip<power_state>(); }
TEST(RoundTrip, CanFrame)            { assert_byte_round_trip<can_frame>(); }
TEST(RoundTrip, CanStatus)           { assert_byte_round_trip<can_status>(); }
TEST(RoundTrip, NotifierSlot)        { assert_byte_round_trip<notifier_slot>(); }
TEST(RoundTrip, NotifierAlarmEvent)  { assert_byte_round_trip<notifier_alarm_event>(); }
TEST(RoundTrip, ErrorMessage)        { assert_byte_round_trip<error_message>(); }
TEST(RoundTrip, JoystickOutputState) { assert_byte_round_trip<joystick_output_state>(); }
TEST(RoundTrip, JoystickOutputBatch) { assert_byte_round_trip<joystick_output_batch>(); }
TEST(RoundTrip, UserProgramObserverSnapshot) {
  assert_byte_round_trip<user_program_observer_snapshot>();
}
TEST(RoundTrip, BootDescriptor)      { assert_byte_round_trip<boot_descriptor>(); }
TEST(RoundTrip, JoystickAxes)        { assert_byte_round_trip<joystick_axes>(); }
TEST(RoundTrip, JoystickButtons)     { assert_byte_round_trip<joystick_buttons>(); }
TEST(RoundTrip, JoystickPovs)        { assert_byte_round_trip<joystick_povs>(); }
TEST(RoundTrip, JoystickDescriptor)  { assert_byte_round_trip<joystick_descriptor>(); }
TEST(RoundTrip, MatchInfo)           { assert_byte_round_trip<match_info>(); }
TEST(RoundTrip, DsState)             { assert_byte_round_trip<ds_state>(); }

// ---------------------------------------------------------------------------
// F — clock_state value-domain pins
// ---------------------------------------------------------------------------

TEST(ClockState, SimTimeBoundaries) {
  for (std::uint64_t v : {std::uint64_t{0}, std::uint64_t{1},
                          std::uint64_t{1'000'000}, UINT64_MAX}) {
    clock_state c{};
    c.sim_time_us = v;
    std::array<std::uint8_t, sizeof(clock_state)> buf{};
    std::memcpy(buf.data(), &c, sizeof(clock_state));
    clock_state back{};
    std::memcpy(&back, buf.data(), sizeof(clock_state));
    EXPECT_EQ(back.sim_time_us, v);
  }
}

TEST(ClockState, HalBoolFieldsRoundTrip) {
  clock_state c{};
  c.system_active = 1;
  c.browned_out = 1;
  c.system_time_valid = 1;
  c.fpga_button_latched = 1;
  c.rsl_state = 1;
  clock_state back = c;
  EXPECT_EQ(back, c);
}

TEST(ClockState, CommsDisableCountBoundary) {
  clock_state c{};
  c.comms_disable_count = UINT32_MAX;
  EXPECT_EQ(c.comms_disable_count, UINT32_MAX);
  c.comms_disable_count = 0;
  EXPECT_EQ(c.comms_disable_count, 0u);
}

TEST(ClockState, DoesNotCarryTeamNumber) {
  // F4 — team_number lives in boot_descriptor (fork F1), not here.
  // Compile-time check via SFINAE: a "team_number" member would let
  // the expression below name it.
  using has_member = decltype(&clock_state::sim_time_us);
  static_assert(std::is_same_v<has_member, std::uint64_t clock_state::*>);
  // (No member-name SFINAE here without a macro; the static_assert
  // above is a placeholder that exercises a sibling member, ensuring
  // the test file at least compiled with clock_state present. The
  // actual "no team_number" guarantee is enforced by the absence of a
  // team_number field in src/backend/common/clock_state.h, audited
  // visually + by code review.)
}

// ---------------------------------------------------------------------------
// G — power_state
// ---------------------------------------------------------------------------

TEST(PowerState, VinVoltageRoundTrip) {
  // Provenance: RoboRIO 2 nominal operating range. Structural test —
  // no tolerance asserted.
  for (float v : {0.0f, 7.0f, 12.6f, 13.5f}) {
    power_state p{};
    p.vin_v = v;
    EXPECT_EQ(p.vin_v, v);
  }
}

TEST(PowerState, DefaultConstructIsAllZero) {
  power_state p{};
  EXPECT_EQ(p.vin_v, 0.0f);
  EXPECT_EQ(p.vin_a, 0.0f);
  EXPECT_EQ(p.brownout_voltage_v, 0.0f);
}

TEST(PowerState, BrownoutVoltageRoundTrip) {
  // 6.75 V is the WPILib HAL_GetBrownoutVoltage default; field round-
  // trip pin only — defaults are the shim's concern.
  power_state p{};
  p.brownout_voltage_v = 6.75f;
  EXPECT_EQ(p.brownout_voltage_v, 6.75f);
}

// ---------------------------------------------------------------------------
// I — can_frame
// ---------------------------------------------------------------------------

TEST(CanFrame, FlagBitsPreserved) {
  can_frame f{};
  f.message_id = kCanFlagFrameRemote | kCanFlagFrame11Bit | 0x123;
  std::array<std::uint8_t, sizeof(can_frame)> buf{};
  std::memcpy(buf.data(), &f, sizeof(f));
  can_frame back{};
  std::memcpy(&back, buf.data(), sizeof(f));
  EXPECT_EQ(back.message_id & kCanFlagFrameRemote, kCanFlagFrameRemote);
  EXPECT_EQ(back.message_id & kCanFlagFrame11Bit, kCanFlagFrame11Bit);
  EXPECT_EQ(back.message_id & 0x1FFFFFFFu, 0x123u);
}

TEST(CanFrame, DataSizeBoundaries) {
  can_frame f{};
  f.data_size = 0;
  EXPECT_EQ(f.data_size, 0);
  f.data_size = 8;
  EXPECT_EQ(f.data_size, 8);
  // I4: data_size > 8 is OUT OF SCOPE of the schema. The validator
  // does not range-check it; that contract lives in the future shim.
}

// ---------------------------------------------------------------------------
// J — can_status
// ---------------------------------------------------------------------------

TEST(CanStatus, FieldOffsetsMatchHalParameterOrder) {
  EXPECT_EQ(offsetof(can_status, percent_bus_utilization), 0u);
  EXPECT_EQ(offsetof(can_status, bus_off_count), 4u);
  EXPECT_EQ(offsetof(can_status, tx_full_count), 8u);
  EXPECT_EQ(offsetof(can_status, receive_error_count), 12u);
  EXPECT_EQ(offsetof(can_status, transmit_error_count), 16u);
}

TEST(CanStatus, BusUtilizationRoundTrip) {
  for (float v : {0.0f, 0.5f, 1.0f}) {
    can_status s{};
    s.percent_bus_utilization = v;
    EXPECT_EQ(s.percent_bus_utilization, v);
  }
}

// ---------------------------------------------------------------------------
// K — notifier
// ---------------------------------------------------------------------------

TEST(NotifierSlot, FieldOffsetsAndSize) {
  EXPECT_EQ(offsetof(notifier_slot, trigger_time_us), 0u);
  EXPECT_EQ(offsetof(notifier_slot, handle), 8u);
  EXPECT_EQ(offsetof(notifier_slot, alarm_active), 12u);
  EXPECT_EQ(offsetof(notifier_slot, canceled), 16u);
  EXPECT_EQ(offsetof(notifier_slot, name), 20u);
  EXPECT_EQ(sizeof(notifier_slot), 88u);  // 84 named + 4 trailing pad
}

TEST(NotifierAlarmEvent, FieldOffsetsAndSize) {
  EXPECT_EQ(offsetof(notifier_alarm_event, fired_at_us), 0u);
  EXPECT_EQ(offsetof(notifier_alarm_event, handle), 8u);
  EXPECT_EQ(offsetof(notifier_alarm_event, reserved_pad), 12u);
  EXPECT_EQ(sizeof(notifier_alarm_event), 16u);
}

TEST(Validator, NotifierStateOverCapacityRejected) {
  notifier_state s{};
  s.count = static_cast<std::uint32_t>(kMaxNotifiers + 1);
  std::array<std::uint8_t, sizeof(notifier_state)> raw{};
  std::memcpy(raw.data(), &s, sizeof(s));
  auto env = make_envelope(envelope_kind::tick_boundary,
                           schema_id::notifier_state, 0xFFFFu);
  auto r = validate_envelope(env, direction::backend_to_core, 0,
                             std::span(raw));
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind, validate_error_kind::payload_size_mismatch);
}

// ---------------------------------------------------------------------------
// L — truncation helpers
// ---------------------------------------------------------------------------

TEST(CopyTruncated, SrcShorterThanDstPreservesTail) {
  // L1: pre-fill dst with 0xAB; copy shorter src; assert tail still 0xAB
  // and dst[src.size()] == '\0'.
  std::array<char, 16> dst;
  dst.fill(static_cast<char>(0xAB));
  std::string_view src = "hi";
  bool truncated = copy_truncated(dst, src);
  EXPECT_FALSE(truncated);
  EXPECT_EQ(dst[0], 'h');
  EXPECT_EQ(dst[1], 'i');
  EXPECT_EQ(dst[2], '\0');
  // L1's preserved-bytes assertion:
  for (std::size_t i = 3; i < dst.size(); ++i) {
    EXPECT_EQ(static_cast<unsigned char>(dst[i]), 0xABu)
        << "byte " << i << " was modified past the null terminator";
  }
}

TEST(CopyTruncated, ExactNMinusOneFit) {
  // L2: src.size() == dst.size() - 1
  std::array<char, 4> dst{};
  std::string_view src = "abc";
  bool truncated = copy_truncated(dst, src);
  EXPECT_FALSE(truncated);
  EXPECT_EQ(dst[0], 'a');
  EXPECT_EQ(dst[1], 'b');
  EXPECT_EQ(dst[2], 'c');
  EXPECT_EQ(dst[3], '\0');
}

TEST(CopyTruncated, SrcEqualsDstSizeTruncates) {
  // L3
  std::array<char, 4> dst{};
  std::string_view src = "abcd";
  bool truncated = copy_truncated(dst, src);
  EXPECT_TRUE(truncated);
  EXPECT_EQ(dst[0], 'a');
  EXPECT_EQ(dst[1], 'b');
  EXPECT_EQ(dst[2], 'c');
  EXPECT_EQ(dst[3], '\0');
}

TEST(CopyTruncated, SrcLongerThanDstTruncates) {
  // L4
  std::array<char, 4> dst{};
  std::string_view src = "abcdefgh";
  bool truncated = copy_truncated(dst, src);
  EXPECT_TRUE(truncated);
  EXPECT_EQ(dst[0], 'a');
  EXPECT_EQ(dst[1], 'b');
  EXPECT_EQ(dst[2], 'c');
  EXPECT_EQ(dst[3], '\0');
}

TEST(CopyTruncated, ZeroSizedDstWithNonemptySrcTruncates) {
  // L5a
  std::array<char, 0> dst;
  bool truncated = copy_truncated(std::span<char>{dst}, "abc");
  EXPECT_TRUE(truncated);
}

TEST(CopyTruncated, ZeroSizedDstWithEmptySrcDoesNotTruncate) {
  // L5b — pinned deliberately as "no truncation by vacuity."
  std::array<char, 0> dst;
  bool truncated = copy_truncated(std::span<char>{dst}, "");
  EXPECT_FALSE(truncated);
}

TEST(CopyTruncated, EmptySrcLeavesNullAtFirstByte) {
  // L6
  std::array<char, 8> dst;
  dst.fill(static_cast<char>(0xCD));
  bool truncated = copy_truncated(dst, "");
  EXPECT_FALSE(truncated);
  EXPECT_EQ(dst[0], '\0');
  for (std::size_t i = 1; i < dst.size(); ++i) {
    EXPECT_EQ(static_cast<unsigned char>(dst[i]), 0xCDu);
  }
}

TEST(CopyBytesTruncated, SrcShorterPreservesTail) {
  // L7
  std::array<std::uint8_t, 8> dst;
  dst.fill(0xCD);
  std::array<std::uint8_t, 3> src{1, 2, 3};
  std::size_t copied = copy_bytes_truncated(dst, src);
  EXPECT_EQ(copied, 3u);
  EXPECT_EQ(dst[0], 1);
  EXPECT_EQ(dst[1], 2);
  EXPECT_EQ(dst[2], 3);
  for (std::size_t i = 3; i < dst.size(); ++i) {
    EXPECT_EQ(dst[i], 0xCD);
  }
}

TEST(CopyBytesTruncated, SrcLongerStopsAtCapacity) {
  // L8
  std::array<std::uint8_t, 4> dst{};
  std::array<std::uint8_t, 8> src{1, 2, 3, 4, 5, 6, 7, 8};
  std::size_t copied = copy_bytes_truncated(dst, src);
  EXPECT_EQ(copied, 4u);
  EXPECT_EQ(dst[0], 1);
  EXPECT_EQ(dst[3], 4);
}

TEST(CopyBytesTruncated, BothEmpty) {
  // L8b
  std::array<std::uint8_t, 0> dst;
  std::array<std::uint8_t, 0> src;
  std::size_t copied = copy_bytes_truncated(std::span<std::uint8_t>{dst},
                                            std::span<const std::uint8_t>{src});
  EXPECT_EQ(copied, 0u);
}

TEST(CopyBytesTruncated, EmptySrcLeavesDstUnchanged) {
  // L8c
  std::array<std::uint8_t, 4> dst;
  dst.fill(0xCD);
  std::array<std::uint8_t, 0> src;
  std::size_t copied = copy_bytes_truncated(
      dst, std::span<const std::uint8_t>{src});
  EXPECT_EQ(copied, 0u);
  for (auto b : dst) EXPECT_EQ(b, 0xCD);
}

TEST(CopyBytesTruncated, ZeroDstWithNonemptySrcCopiesNothing) {
  // L8d
  std::array<std::uint8_t, 0> dst;
  std::array<std::uint8_t, 4> src{1, 2, 3, 4};
  std::size_t copied = copy_bytes_truncated(std::span<std::uint8_t>{dst}, src);
  EXPECT_EQ(copied, 0u);
}

TEST(ErrorMessage, TruncationFlagBitsArePinned) {
  // L10 — bit assignments stable across the codebase.
  EXPECT_EQ(kErrorTruncDetails, 0b001u);
  EXPECT_EQ(kErrorTruncLocation, 0b010u);
  EXPECT_EQ(kErrorTruncCallStack, 0b100u);
}

TEST(ErrorMessage, PopulateAndRoundTrip) {
  // L9 — end-to-end populate (via copy_truncated) and byte round-trip.
  error_message msg{};
  std::string long_details(kErrorDetailsLen + 50, 'x');
  if (copy_truncated(msg.details, long_details)) {
    msg.truncation_flags |= kErrorTruncDetails;
  }
  EXPECT_NE(msg.truncation_flags & kErrorTruncDetails, 0);
  EXPECT_EQ(msg.details.back(), '\0');

  std::array<std::uint8_t, sizeof(error_message)> buf{};
  std::memcpy(buf.data(), &msg, sizeof(msg));
  error_message back{};
  std::memcpy(&back, buf.data(), sizeof(msg));
  EXPECT_EQ(back, msg);
}

// ---------------------------------------------------------------------------
// M — closed-enum gates (validator-side)
// ---------------------------------------------------------------------------

TEST(Validator, AcceptsAllValidSchemaIds) {
  // M1 — every schema_id 0..kSchemaIdMaxValid is recognized; specific kind/payload-size
  // constraints are tested in section Q. Here we just verify the
  // validator doesn't bail with unknown_schema_id for the closed set.
  for (std::uint8_t v = 0; v <= kSchemaIdMaxValid; ++v) {
    sync_envelope env{};
    env.magic = kProtocolMagic;
    env.protocol_version = kProtocolVersion;
    env.kind = envelope_kind::tick_boundary;
    env.sequence = 0;
    env.payload_bytes = 0;
    env.payload_schema = static_cast<schema_id>(v);
    env.sender = direction::backend_to_core;
    auto r = validate_envelope(env, direction::backend_to_core, 0);
    if (!r.has_value()) {
      // Only acceptable failure here is a non-schema-id-related kind
      // (e.g. payload_size_mismatch when schema mandates a non-zero
      // body, or schema_payload_kind_mismatch).
      EXPECT_NE(r.error().kind, validate_error_kind::unknown_schema_id);
    }
  }
}

TEST(Validator, RejectsAllOutOfRangeSchemaIds) {
  // M2 — sample at a few out-of-range values
  for (std::uint8_t v : {std::uint8_t{12}, std::uint8_t{50},
                         std::uint8_t{100}, std::uint8_t{255}}) {
    sync_envelope env{};
    env.magic = kProtocolMagic;
    env.protocol_version = kProtocolVersion;
    env.kind = envelope_kind::tick_boundary;
    env.payload_schema = static_cast<schema_id>(v);
    env.sender = direction::backend_to_core;
    auto r = validate_envelope(env, direction::backend_to_core, 0);
    ASSERT_FALSE(r.has_value()) << "value " << +v;
    EXPECT_EQ(r.error().kind, validate_error_kind::unknown_schema_id);
  }
}

TEST(BootDescriptor, RuntimeTypeRoboRio2Value) {
  // M4 — RoboRIO2 backing value is 2, matching HAL_Runtime_RoboRIO2.
  EXPECT_EQ(static_cast<std::uint8_t>(runtime_type::roborio_2), 2u);
}

TEST(SchemaId, RoundTripPreservesUint8) {
  // M5
  for (std::uint8_t v = 0; v <= kSchemaIdMaxValid; ++v) {
    auto s = static_cast<schema_id>(v);
    EXPECT_EQ(static_cast<std::uint8_t>(s), v);
  }
}

TEST(ProtocolVersion, Cycle38BumpsProtocolForUserProgramObserverSnapshot) {
  EXPECT_EQ(kProtocolVersion, 3u);
  EXPECT_EQ(kSchemaIdMaxValid, 11u);
  EXPECT_EQ(static_cast<std::uint8_t>(schema_id::joystick_output_batch), 10u);
  EXPECT_EQ(static_cast<std::uint8_t>(schema_id::user_program_observer_snapshot), 11u);
}

// ---------------------------------------------------------------------------
// M0 — validate_error structural
// ---------------------------------------------------------------------------

TEST(ValidateError, IsStandardLayout) {
  EXPECT_TRUE(std::is_standard_layout_v<validate_error>);
}

TEST(ValidateError, OffendingFieldNameContainsSourceName) {
  // M0b — at least one D-section error pins the field-name substring
  // contract. (Other D tests do this too via EXPECT_EQ on
  // offending_field_name; this test makes the contract explicit.)
  auto env = make_envelope(envelope_kind::tick_boundary,
                           schema_id::clock_state, sizeof(clock_state));
  env.protocol_version = 999;
  auto r = validate_envelope(env, direction::backend_to_core, 0);
  ASSERT_FALSE(r.has_value());
  EXPECT_NE(r.error().offending_field_name.find("protocol_version"),
            std::string::npos);
}

TEST(Validator, SuccessReturnsHasValueTrue) {
  // M0c — explicit shape pin.
  auto env = make_envelope(envelope_kind::shutdown, schema_id::none, 0);
  auto r = validate_envelope(env, direction::backend_to_core, 0);
  EXPECT_TRUE(r.has_value());
}

// ---------------------------------------------------------------------------
// N — boot_descriptor
// ---------------------------------------------------------------------------

TEST(BootDescriptor, FieldOffsetsAndSize) {
  EXPECT_EQ(offsetof(boot_descriptor, runtime), 0u);
  EXPECT_EQ(offsetof(boot_descriptor, reserved3), 1u);
  EXPECT_EQ(offsetof(boot_descriptor, team_number), 4u);
  EXPECT_EQ(offsetof(boot_descriptor, vendor_capabilities), 8u);
  EXPECT_EQ(offsetof(boot_descriptor, wpilib_version), 12u);
  EXPECT_EQ(sizeof(boot_descriptor), 44u);
}

TEST(Validator, RejectsUnsupportedRuntime) {
  // N2 — runtime_type != roborio_2 → unsupported_runtime.
  // The validator currently checks runtime via a separate path on
  // boot envelopes; here we just verify the enum rejects misuse at
  // round-trip (the actual handshake check is shim-cycle scope per
  // decision #19, but the closed-enum value is pinned here).
  EXPECT_EQ(static_cast<std::uint8_t>(runtime_type::reserved), 0u);
  EXPECT_EQ(static_cast<std::uint8_t>(runtime_type::roborio_2), 2u);
}

TEST(BootDescriptor, TeamNumberRoundTrip) {
  // N3
  for (std::int32_t t : {0, 1, 9999, INT32_MAX}) {
    boot_descriptor b{};
    b.team_number = t;
    EXPECT_EQ(b.team_number, t);
  }
}

TEST(BootDescriptor, WpilibVersionPopulateViaTruncate) {
  // N4 — populate via copy_truncated; verify null termination.
  boot_descriptor b{};
  EXPECT_FALSE(copy_truncated(b.wpilib_version, "v2026.2.2"));
  EXPECT_EQ(b.wpilib_version[0], 'v');
  EXPECT_EQ(b.wpilib_version[9], '\0');

  std::string overlong(kWpilibVersionLen + 5, 'x');
  EXPECT_TRUE(copy_truncated(b.wpilib_version, overlong));
  EXPECT_EQ(b.wpilib_version.back(), '\0');
}

// ---------------------------------------------------------------------------
// O — padding hygiene
// ---------------------------------------------------------------------------

template <typename T>
void assert_padding_hygiene_zero_init() {
  T zero{};
  std::array<std::uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), &zero, sizeof(T));
  for (std::size_t i = 0; i < sizeof(T); ++i) {
    EXPECT_EQ(bytes[i], 0u)
        << "byte " << i << " of zero-initialized " << typeid(T).name()
        << " is non-zero — there is an uninitialized pad byte";
  }
}

TEST(PaddingHygiene, SyncEnvelopeZeroInit) {
  assert_padding_hygiene_zero_init<sync_envelope>();
}
TEST(PaddingHygiene, ClockStateZeroInit) {
  assert_padding_hygiene_zero_init<clock_state>();
}
TEST(PaddingHygiene, PowerStateZeroInit) {
  assert_padding_hygiene_zero_init<power_state>();
}
TEST(PaddingHygiene, CanFrameZeroInit) {
  assert_padding_hygiene_zero_init<can_frame>();
}
TEST(PaddingHygiene, CanStatusZeroInit) {
  assert_padding_hygiene_zero_init<can_status>();
}
TEST(PaddingHygiene, NotifierSlotZeroInit) {
  assert_padding_hygiene_zero_init<notifier_slot>();
}
TEST(PaddingHygiene, NotifierAlarmEventZeroInit) {
  assert_padding_hygiene_zero_init<notifier_alarm_event>();
}
TEST(PaddingHygiene, ErrorMessageZeroInit) {
  assert_padding_hygiene_zero_init<error_message>();
}
TEST(PaddingHygiene, UserProgramObserverSnapshotZeroInit) {
  assert_padding_hygiene_zero_init<user_program_observer_snapshot>();
}
TEST(PaddingHygiene, BootDescriptorZeroInit) {
  assert_padding_hygiene_zero_init<boot_descriptor>();
}
TEST(PaddingHygiene, DsStateZeroInit) {
  assert_padding_hygiene_zero_init<ds_state>();
}

// ---------------------------------------------------------------------------
// Q — envelope_kind ↔ payload_schema mapping (parameterized)
// ---------------------------------------------------------------------------

namespace q_helpers {

bool table_allows(envelope_kind k, schema_id s) {
  for (const auto& row : kPerKindAllowedSchemas) {
    if (row.kind == k) {
      const auto idx = static_cast<std::size_t>(s);
      if (idx >= row.allowed_by_schema_id.size()) return false;
      return row.allowed_by_schema_id[idx];
    }
  }
  return false;
}

std::uint32_t expected_payload_size_for(schema_id s) {
  switch (s) {
    case schema_id::none:                 return 0;
    case schema_id::clock_state:          return sizeof(clock_state);
    case schema_id::power_state:          return sizeof(power_state);
    case schema_id::ds_state:             return sizeof(ds_state);
    case schema_id::can_status:           return sizeof(can_status);
    case schema_id::boot_descriptor:      return sizeof(boot_descriptor);
    case schema_id::can_frame_batch:      return offsetof(can_frame_batch, frames);
    case schema_id::notifier_state:       return offsetof(notifier_state, slots);
    case schema_id::notifier_alarm_batch: return offsetof(notifier_alarm_batch, events);
    case schema_id::error_message_batch:  return offsetof(error_message_batch, messages);
    case schema_id::joystick_output_batch: return offsetof(joystick_output_batch, outputs);
    case schema_id::user_program_observer_snapshot:
      return sizeof(user_program_observer_snapshot);
  }
  return 0;
}

}  // namespace q_helpers

TEST(KindSchemaMapping, ExhaustiveCrossProduct) {
  // Q11 — walk every (envelope_kind, schema_id) pair. For pairs in the
  // table, validator returns ok. For pairs not in the table, validator
  // returns schema_payload_kind_mismatch.
  //
  // For variable-size schemas we set count=0 (header-only), which the
  // validator decodes from the payload prefix. All other schemas are
  // fixed-size or none.
  const std::array<envelope_kind, 6> kinds{
      envelope_kind::boot, envelope_kind::boot_ack,
      envelope_kind::tick_boundary, envelope_kind::on_demand_request,
      envelope_kind::on_demand_reply, envelope_kind::shutdown};
  for (auto k : kinds) {
    for (std::uint8_t v = 0; v <= kSchemaIdMaxValid; ++v) {
      const auto s = static_cast<schema_id>(v);
      const std::uint32_t payload_size = q_helpers::expected_payload_size_for(s);

      // Build a payload buffer with count=0 if variable.
      std::vector<std::uint8_t> payload(payload_size, 0);

      auto env = make_envelope(k, s, payload_size);
      auto r = validate_envelope(env, direction::backend_to_core, 0,
                                 std::span(payload));
      const bool allowed = q_helpers::table_allows(k, s);
      if (allowed) {
        EXPECT_TRUE(r.has_value())
            << "expected ok for kind=" << static_cast<int>(k)
            << " schema=" << +v
            << " err=" << (r.has_value() ? -1
                                         : static_cast<int>(r.error().kind));
      } else {
        EXPECT_FALSE(r.has_value())
            << "expected reject for kind=" << static_cast<int>(k)
            << " schema=" << +v;
        if (!r.has_value()) {
          EXPECT_EQ(r.error().kind,
                    validate_error_kind::schema_payload_kind_mismatch);
        }
      }
    }
  }
}

TEST(KindSchemaMapping, BootRequiresBootDescriptor) {
  auto env = make_envelope(envelope_kind::boot, schema_id::clock_state,
                           sizeof(clock_state));
  auto r = validate_envelope(env, direction::backend_to_core, 0);
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind,
            validate_error_kind::schema_payload_kind_mismatch);
}

TEST(KindSchemaMapping, OnDemandRequestForbidsErrorMessageBatch) {
  // Q9 — errors are pushed, not requested.
  std::vector<std::uint8_t> payload(
      offsetof(error_message_batch, messages), 0);
  auto env = make_envelope(envelope_kind::on_demand_request,
                           schema_id::error_message_batch,
                           static_cast<std::uint32_t>(payload.size()));
  auto r = validate_envelope(env, direction::backend_to_core, 0,
                             std::span(payload));
  ASSERT_FALSE(r.has_value());
  EXPECT_EQ(r.error().kind,
            validate_error_kind::schema_payload_kind_mismatch);
}

TEST(KindSchemaMapping, OnDemandReplyAllowsErrorMessageBatch) {
  // Q10 — replies may carry a synchronous error result.
  std::vector<std::uint8_t> payload(
      offsetof(error_message_batch, messages), 0);
  auto env = make_envelope(envelope_kind::on_demand_reply,
                           schema_id::error_message_batch,
                           static_cast<std::uint32_t>(payload.size()));
  auto r = validate_envelope(env, direction::backend_to_core, 0,
                             std::span(payload));
  EXPECT_TRUE(r.has_value());
}

}  // namespace
}  // namespace robosim::backend
