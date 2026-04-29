#include "shared_memory_transport.h"

#include <gtest/gtest.h>

#include <sys/mman.h>
#include <sys/stat.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <type_traits>
#include <unistd.h>
#include <vector>

namespace robosim::backend::tier1 {
namespace {

int create_test_memfd(std::size_t size) {
  const int fd = memfd_create("robosim-tier1-test", MFD_CLOEXEC);
  EXPECT_GE(fd, 0);
  EXPECT_EQ(ftruncate(fd, static_cast<off_t>(size)), 0);
  return fd;
}

bool fd_is_valid(int fd) {
  struct stat st {};
  return fstat(fd, &st) == 0;
}

template <typename T>
std::vector<std::uint8_t> bytes_of(const T& value) {
  const auto bytes = std::as_bytes(std::span{&value, std::size_t{1}});
  std::vector<std::uint8_t> out(bytes.size());
  std::ranges::transform(
      bytes, out.begin(), [](std::byte b) { return static_cast<std::uint8_t>(b); });
  return out;
}

void expect_fd_closed(int fd) {
  errno = 0;
  EXPECT_EQ(fcntl(fd, F_GETFD), -1);
  EXPECT_EQ(errno, EBADF);
}

void expect_default_lane(const tier1_lane& lane) {
  EXPECT_EQ(lane.state.load(), static_cast<std::uint32_t>(tier1_lane_state::empty));
  EXPECT_EQ(lane.envelope, sync_envelope{});
  EXPECT_EQ(lane.payload_bytes, 0u);
  for (const auto byte : lane.payload) {
    EXPECT_EQ(byte, 0u);
  }
}

void complete_round_trip(tier1_shared_mapping& owner, tier1_shared_mapping& peer) {
  auto backend = tier1_endpoint::make(owner.region(), direction::backend_to_core);
  ASSERT_TRUE(backend.has_value());
  auto core = tier1_endpoint::make(peer.region(), direction::core_to_backend);
  ASSERT_TRUE(core.has_value());

  boot_descriptor boot{};
  boot.runtime = runtime_type::roborio_2;
  boot.team_number = 971;
  const auto boot_bytes = bytes_of(boot);
  ASSERT_TRUE(backend->send(envelope_kind::boot, schema_id::boot_descriptor, boot_bytes, 0));
  auto boot_msg = core->try_receive();
  ASSERT_TRUE(boot_msg.has_value());
  EXPECT_EQ(boot_msg->envelope.sequence, 0u);
  EXPECT_EQ(owner.region().backend_to_core.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));

  ASSERT_TRUE(core->send(envelope_kind::boot_ack, schema_id::none, {}, 0));
  auto ack_msg = backend->try_receive();
  ASSERT_TRUE(ack_msg.has_value());
  EXPECT_EQ(ack_msg->envelope.sequence, 0u);
  EXPECT_EQ(peer.region().core_to_backend.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));

  clock_state tick{};
  tick.sim_time_us = 20'000;
  tick.system_active = 1;
  tick.system_time_valid = 1;
  const auto tick_bytes = bytes_of(tick);
  ASSERT_TRUE(
      backend->send(envelope_kind::tick_boundary, schema_id::clock_state, tick_bytes, 20'000));
  auto tick_msg = core->try_receive();
  ASSERT_TRUE(tick_msg.has_value());
  EXPECT_EQ(tick_msg->envelope.sequence, 1u);
  EXPECT_EQ(owner.region().backend_to_core.state.load(),
            static_cast<std::uint32_t>(tier1_lane_state::empty));
}

}  // namespace

TEST(Tier1SharedMapping, CreateMapsZeroInitializedRegionOfExactRegionSize) {
  auto mapping = tier1_shared_mapping::create();
  ASSERT_TRUE(mapping.has_value());
  ASSERT_TRUE(mapping->valid());
  EXPECT_EQ(mapping->size_bytes(), sizeof(tier1_shared_region));

  struct stat st {};
  ASSERT_EQ(fstat(mapping->fd(), &st), 0);
  EXPECT_EQ(st.st_size, static_cast<off_t>(sizeof(tier1_shared_region)));
  expect_default_lane(mapping->region().backend_to_core);
  expect_default_lane(mapping->region().core_to_backend);

  auto endpoint = tier1_endpoint::make(mapping->region(), direction::backend_to_core);
  EXPECT_TRUE(endpoint.has_value());
}

TEST(Tier1SharedMapping, DuplicateFdMapsPeerThatObservesSameRegionAndOwnsMappingFdIndependently) {
  auto owner = tier1_shared_mapping::create();
  ASSERT_TRUE(owner.has_value());

  std::optional<tier1_shared_mapping> peer;
  {
    auto dup = owner->duplicate_fd();
    ASSERT_TRUE(dup.has_value());
    auto mapped = tier1_shared_mapping::map_existing(*dup);
    ASSERT_TRUE(mapped.has_value());
    EXPECT_TRUE(dup->valid());
    EXPECT_TRUE(fd_is_valid(dup->get()));
    peer.emplace(std::move(*mapped));
  }

  ASSERT_TRUE(peer.has_value());
  EXPECT_NE(&owner->region(), &peer->region());
  owner->region().backend_to_core.payload_bytes = 123;
  EXPECT_EQ(peer->region().backend_to_core.payload_bytes, 123u);
  peer->region().core_to_backend.payload_bytes = 456;
  EXPECT_EQ(owner->region().core_to_backend.payload_bytes, 456u);
  EXPECT_TRUE(fd_is_valid(peer->fd()));
  peer->region().backend_to_core.payload_bytes = 789;
  EXPECT_EQ(owner->region().backend_to_core.payload_bytes, 789u);
}

TEST(Tier1SharedMapping, SupportsEndpointRoundTripAcrossTwoMappings) {
  auto owner = tier1_shared_mapping::create();
  ASSERT_TRUE(owner.has_value());
  auto dup = owner->duplicate_fd();
  ASSERT_TRUE(dup.has_value());
  auto peer = tier1_shared_mapping::map_existing(*dup);
  ASSERT_TRUE(peer.has_value());

  complete_round_trip(*owner, *peer);
}

TEST(Tier1SharedMapping, MapExistingRejectsInvalidFdWithoutSyscallErrno) {
  unique_fd invalid;
  auto mapped = tier1_shared_mapping::map_existing(invalid);
  ASSERT_FALSE(mapped.has_value());
  EXPECT_EQ(mapped.error().kind, tier1_mapping_error_kind::invalid_fd);
  EXPECT_NE(mapped.error().offending_field_name.find("fd"), std::string::npos);
  EXPECT_EQ(mapped.error().errno_value, 0);
}

TEST(Tier1SharedMapping, MapExistingRejectsFdWithWrongSizeBeforeMmap) {
  for (const auto size : {sizeof(tier1_shared_region) - 1, sizeof(tier1_shared_region) + 1}) {
    unique_fd fd{create_test_memfd(size)};
    ASSERT_TRUE(fd.valid());
    auto mapped = tier1_shared_mapping::map_existing(fd);
    ASSERT_FALSE(mapped.has_value());
    EXPECT_EQ(mapped.error().kind, tier1_mapping_error_kind::wrong_size);
    EXPECT_NE(mapped.error().offending_field_name.find("size"), std::string::npos);
    EXPECT_EQ(mapped.error().errno_value, 0);
    EXPECT_TRUE(fd_is_valid(fd.get()));
  }
}

TEST(Tier1SharedMapping, DuplicateFdReturnsIndependentValidFdOwnership) {
  auto owner = tier1_shared_mapping::create();
  ASSERT_TRUE(owner.has_value());
  const int owner_fd = owner->fd();
  {
    auto dup = owner->duplicate_fd();
    ASSERT_TRUE(dup.has_value());
    EXPECT_TRUE(dup->valid());
    EXPECT_NE(dup->get(), owner_fd);
    EXPECT_TRUE(fd_is_valid(dup->get()));
  }
  EXPECT_TRUE(fd_is_valid(owner_fd));
  auto endpoint = tier1_endpoint::make(owner->region(), direction::backend_to_core);
  EXPECT_TRUE(endpoint.has_value());
}

TEST(UniqueFd, IsMoveOnlyAndMoveTransfersOwnership) {
  static_assert(std::is_move_constructible_v<unique_fd>);
  static_assert(std::is_move_assignable_v<unique_fd>);
  static_assert(!std::is_copy_constructible_v<unique_fd>);
  static_assert(!std::is_copy_assignable_v<unique_fd>);

  const int raw = create_test_memfd(1);
  int old_target = -1;
  {
    unique_fd a{raw};
    unique_fd b{std::move(a)};
    EXPECT_FALSE(a.valid());
    ASSERT_TRUE(b.valid());
    EXPECT_EQ(b.get(), raw);

    unique_fd c{create_test_memfd(1)};
    old_target = c.get();
    ASSERT_NE(old_target, raw);
    c = std::move(b);
    EXPECT_FALSE(b.valid());
    ASSERT_TRUE(c.valid());
    EXPECT_EQ(c.get(), raw);
    expect_fd_closed(old_target);
  }
  expect_fd_closed(raw);
}

TEST(Tier1SharedMapping, IsMoveOnlyAndMoveTransfersMappingOwnership) {
  static_assert(std::is_move_constructible_v<tier1_shared_mapping>);
  static_assert(std::is_move_assignable_v<tier1_shared_mapping>);
  static_assert(!std::is_copy_constructible_v<tier1_shared_mapping>);
  static_assert(!std::is_copy_assignable_v<tier1_shared_mapping>);

  auto original = tier1_shared_mapping::create();
  ASSERT_TRUE(original.has_value());
  original->region().backend_to_core.payload_bytes = 321;
  const int original_fd = original->fd();

  tier1_shared_mapping moved{std::move(*original)};
  EXPECT_FALSE(original->valid());
  ASSERT_TRUE(moved.valid());
  EXPECT_EQ(moved.fd(), original_fd);
  EXPECT_EQ(moved.region().backend_to_core.payload_bytes, 321u);

  auto target = tier1_shared_mapping::create();
  ASSERT_TRUE(target.has_value());
  const int old_target_fd = target->fd();
  const int incoming_fd = moved.fd();
  ASSERT_NE(old_target_fd, incoming_fd);
  *target = std::move(moved);
  EXPECT_FALSE(moved.valid());
  ASSERT_TRUE(target->valid());
  EXPECT_EQ(target->fd(), incoming_fd);
  EXPECT_EQ(target->region().backend_to_core.payload_bytes, 321u);
  target->region().backend_to_core.payload_bytes = 654;
  EXPECT_EQ(target->region().backend_to_core.payload_bytes, 654u);
  expect_fd_closed(old_target_fd);

  auto endpoint = tier1_endpoint::make(target->region(), direction::backend_to_core);
  EXPECT_TRUE(endpoint.has_value());
}

TEST(Tier1SharedMapping, PeerRemainsValidAfterOwnerMappingIsDestroyed) {
  std::optional<tier1_shared_mapping> peer;
  {
    auto owner = tier1_shared_mapping::create();
    ASSERT_TRUE(owner.has_value());
    auto dup = owner->duplicate_fd();
    ASSERT_TRUE(dup.has_value());
    auto mapped = tier1_shared_mapping::map_existing(*dup);
    ASSERT_TRUE(mapped.has_value());
    peer.emplace(std::move(*mapped));
    peer->region().backend_to_core.payload_bytes = 444;
  }

  ASSERT_TRUE(peer.has_value());
  ASSERT_TRUE(peer->valid());
  EXPECT_EQ(peer->region().backend_to_core.payload_bytes, 444u);
  peer->region().backend_to_core.payload_bytes = 555;
  EXPECT_EQ(peer->region().backend_to_core.payload_bytes, 555u);
  auto endpoint = tier1_endpoint::make(peer->region(), direction::backend_to_core);
  EXPECT_TRUE(endpoint.has_value());
}

}  // namespace robosim::backend::tier1
