#include "shared_memory_transport.h"

#include <sys/mman.h>
#include <sys/stat.h>

#include <algorithm>
#include <cerrno>
#include <cstddef>
#include <cstring>
#include <fcntl.h>
#include <new>
#include <unistd.h>
#include <utility>

namespace robosim::backend::tier1 {
namespace {

[[nodiscard]] tier1_transport_error make_error(tier1_transport_error_kind kind,
                                               std::string field,
                                               std::string message) {
  return tier1_transport_error{kind, std::nullopt, std::move(field), std::move(message)};
}

[[nodiscard]] tier1_mapping_error make_mapping_error(tier1_mapping_error_kind kind,
                                                     int errno_value,
                                                     std::string field,
                                                     std::string message) {
  return tier1_mapping_error{kind, errno_value, std::move(field), std::move(message)};
}

[[nodiscard]] tier1_transport_error wrap_session_error(session_error error) {
  std::string field = error.offending_field_name;
  return tier1_transport_error{tier1_transport_error_kind::session_rejected_envelope,
                               std::move(error),
                               std::move(field),
                               "protocol session rejected envelope"};
}

[[nodiscard]] bool is_empty(std::uint32_t state) {
  return state == static_cast<std::uint32_t>(tier1_lane_state::empty);
}

[[nodiscard]] bool is_full(std::uint32_t state) {
  return state == static_cast<std::uint32_t>(tier1_lane_state::full);
}

[[nodiscard]] bool is_in_progress(std::uint32_t state) {
  return state == static_cast<std::uint32_t>(tier1_lane_state::writing) ||
         state == static_cast<std::uint32_t>(tier1_lane_state::reading);
}

}  // namespace

unique_fd::unique_fd(int fd) : fd_(fd) {}

unique_fd::~unique_fd() {
  reset();
}

unique_fd::unique_fd(unique_fd&& other) noexcept : fd_(other.release()) {}

unique_fd& unique_fd::operator=(unique_fd&& other) noexcept {
  if (this != &other) {
    reset(other.release());
  }
  return *this;
}

int unique_fd::get() const {
  return fd_;
}

bool unique_fd::valid() const {
  return fd_ >= 0;
}

int unique_fd::release() {
  const int fd = fd_;
  fd_ = -1;
  return fd;
}

void unique_fd::reset(int fd) noexcept {
  if (fd_ >= 0) {
    (void)::close(fd_);
  }
  fd_ = fd;
}

tier1_shared_mapping::~tier1_shared_mapping() {
  reset();
}

tier1_shared_mapping::tier1_shared_mapping(tier1_shared_mapping&& other) noexcept
    : fd_(std::move(other.fd_)),
      region_(std::exchange(other.region_, nullptr)),
      size_bytes_(std::exchange(other.size_bytes_, 0)) {}

tier1_shared_mapping& tier1_shared_mapping::operator=(tier1_shared_mapping&& other) noexcept {
  if (this != &other) {
    reset();
    fd_ = std::move(other.fd_);
    region_ = std::exchange(other.region_, nullptr);
    size_bytes_ = std::exchange(other.size_bytes_, 0);
  }
  return *this;
}

std::expected<tier1_shared_mapping, tier1_mapping_error> tier1_shared_mapping::create() {
  const int raw_fd = ::memfd_create("robosim-tier1", MFD_CLOEXEC);
  if (raw_fd < 0) {
    return std::unexpected(make_mapping_error(tier1_mapping_error_kind::memfd_create_failed,
                                              errno,
                                              "memfd_create",
                                              "memfd_create failed"));
  }

  unique_fd fd{raw_fd};
  constexpr auto size = static_cast<off_t>(sizeof(tier1_shared_region));
  if (::ftruncate(fd.get(), size) != 0) {
    return std::unexpected(make_mapping_error(
        tier1_mapping_error_kind::ftruncate_failed, errno, "size", "ftruncate failed"));
  }

  void* mapped =
      ::mmap(nullptr, sizeof(tier1_shared_region), PROT_READ | PROT_WRITE, MAP_SHARED, fd.get(), 0);
  if (mapped == MAP_FAILED) {
    return std::unexpected(
        make_mapping_error(tier1_mapping_error_kind::mmap_failed, errno, "mmap", "mmap failed"));
  }

  auto* region = static_cast<tier1_shared_region*>(mapped);
  std::construct_at(region);
  return tier1_shared_mapping(std::move(fd), region, sizeof(tier1_shared_region));
}

std::expected<tier1_shared_mapping, tier1_mapping_error> tier1_shared_mapping::map_existing(
    const unique_fd& fd) {
  if (!fd.valid()) {
    return std::unexpected(
        make_mapping_error(tier1_mapping_error_kind::invalid_fd, 0, "fd", "fd is invalid"));
  }

  struct stat st {};
  if (::fstat(fd.get(), &st) != 0) {
    return std::unexpected(
        make_mapping_error(tier1_mapping_error_kind::invalid_fd, errno, "fd", "fstat failed"));
  }
  if (st.st_size != static_cast<off_t>(sizeof(tier1_shared_region))) {
    return std::unexpected(make_mapping_error(
        tier1_mapping_error_kind::wrong_size, 0, "size", "shared mapping size mismatch"));
  }

  const int dup_fd = ::fcntl(fd.get(), F_DUPFD_CLOEXEC, 0);
  if (dup_fd < 0) {
    return std::unexpected(make_mapping_error(
        tier1_mapping_error_kind::dup_failed, errno, "fd", "fd duplication failed"));
  }
  unique_fd owned_fd{dup_fd};

  void* mapped = ::mmap(
      nullptr, sizeof(tier1_shared_region), PROT_READ | PROT_WRITE, MAP_SHARED, owned_fd.get(), 0);
  if (mapped == MAP_FAILED) {
    return std::unexpected(
        make_mapping_error(tier1_mapping_error_kind::mmap_failed, errno, "mmap", "mmap failed"));
  }

  return tier1_shared_mapping(
      std::move(owned_fd), static_cast<tier1_shared_region*>(mapped), sizeof(tier1_shared_region));
}

std::expected<unique_fd, tier1_mapping_error> tier1_shared_mapping::duplicate_fd() const {
  if (!fd_.valid()) {
    return std::unexpected(
        make_mapping_error(tier1_mapping_error_kind::invalid_fd, 0, "fd", "mapping fd is invalid"));
  }
  const int dup_fd = ::fcntl(fd_.get(), F_DUPFD_CLOEXEC, 0);
  if (dup_fd < 0) {
    return std::unexpected(make_mapping_error(
        tier1_mapping_error_kind::dup_failed, errno, "fd", "fd duplication failed"));
  }
  return unique_fd{dup_fd};
}

tier1_shared_region& tier1_shared_mapping::region() {
  return *region_;
}

const tier1_shared_region& tier1_shared_mapping::region() const {
  return *region_;
}

std::size_t tier1_shared_mapping::size_bytes() const {
  return size_bytes_;
}

int tier1_shared_mapping::fd() const {
  return fd_.get();
}

bool tier1_shared_mapping::valid() const {
  return fd_.valid() && region_ != nullptr && size_bytes_ == sizeof(tier1_shared_region);
}

tier1_shared_mapping::tier1_shared_mapping(unique_fd fd,
                                           tier1_shared_region* region,
                                           std::size_t size_bytes)
    : fd_(std::move(fd)), region_(region), size_bytes_(size_bytes) {}

void tier1_shared_mapping::reset() noexcept {
  if (region_ != nullptr && size_bytes_ > 0) {
    (void)::munmap(region_, size_bytes_);
  }
  region_ = nullptr;
  size_bytes_ = 0;
  fd_ = unique_fd{};
}

std::expected<tier1_endpoint, tier1_transport_error> tier1_endpoint::make(
    tier1_shared_region& region, direction local_direction) {
  auto session = protocol_session::make(local_direction);
  if (!session.has_value()) {
    return std::unexpected(make_error(tier1_transport_error_kind::invalid_endpoint_direction,
                                      session.error().offending_field_name,
                                      session.error().message));
  }
  return tier1_endpoint(region, std::move(*session));
}

tier1_endpoint::tier1_endpoint(tier1_shared_region& region, protocol_session session)
    : region_(&region), session_(std::move(session)) {}

std::expected<void, tier1_transport_error> tier1_endpoint::send(
    envelope_kind kind,
    schema_id payload_schema,
    std::span<const std::uint8_t> payload,
    std::uint64_t sim_time_us) {
  if (payload.size() > kTier1MaxPayloadBytes) {
    return std::unexpected(make_error(tier1_transport_error_kind::payload_too_large,
                                      "payload",
                                      "payload exceeds T1 lane capacity"));
  }

  tier1_lane& lane = outbound_lane();
  const auto state = lane.state.load(std::memory_order_acquire);
  if (is_full(state)) {
    return std::unexpected(
        make_error(tier1_transport_error_kind::lane_busy, "lane", "outbound lane is full"));
  }
  if (is_in_progress(state)) {
    return std::unexpected(make_error(
        tier1_transport_error_kind::lane_in_progress, "state", "outbound lane is in progress"));
  }

  auto env = session_.build_envelope(
      kind, payload_schema, static_cast<std::uint32_t>(payload.size()), sim_time_us, payload);
  if (!env.has_value()) {
    return std::unexpected(wrap_session_error(env.error()));
  }

  lane.state.store(static_cast<std::uint32_t>(tier1_lane_state::writing),
                   std::memory_order_release);
  lane.envelope = *env;
  lane.payload_bytes = static_cast<std::uint32_t>(payload.size());
  std::copy(payload.begin(), payload.end(), lane.payload.begin());
  lane.state.store(static_cast<std::uint32_t>(tier1_lane_state::full), std::memory_order_release);
  return {};
}

std::expected<tier1_message, tier1_transport_error> tier1_endpoint::try_receive() {
  tier1_lane& lane = inbound_lane();
  const auto state = lane.state.load(std::memory_order_acquire);
  if (is_empty(state)) {
    return std::unexpected(
        make_error(tier1_transport_error_kind::no_message, "lane", "inbound lane is empty"));
  }
  if (is_in_progress(state)) {
    return std::unexpected(make_error(
        tier1_transport_error_kind::lane_in_progress, "state", "inbound lane is in progress"));
  }

  if (lane.payload_bytes > kTier1MaxPayloadBytes) {
    return std::unexpected(make_error(tier1_transport_error_kind::payload_too_large,
                                      "payload_bytes",
                                      "payload_bytes exceeds lane capacity"));
  }

  tier1_message message{
      lane.envelope,
      std::vector<std::uint8_t>(
          lane.payload.begin(),
          lane.payload.begin() + static_cast<std::ptrdiff_t>(lane.payload_bytes)),
  };

  auto accepted = session_.accept_envelope(message.envelope, message.payload);
  if (!accepted.has_value()) {
    return std::unexpected(wrap_session_error(accepted.error()));
  }

  lane.state.store(static_cast<std::uint32_t>(tier1_lane_state::reading),
                   std::memory_order_release);
  lane.payload_bytes = 0;
  lane.state.store(static_cast<std::uint32_t>(tier1_lane_state::empty), std::memory_order_release);
  return message;
}

tier1_lane& tier1_endpoint::outbound_lane() {
  return session_.local_direction() == direction::backend_to_core ? region_->backend_to_core
                                                                  : region_->core_to_backend;
}

tier1_lane& tier1_endpoint::inbound_lane() {
  return session_.local_direction() == direction::backend_to_core ? region_->core_to_backend
                                                                  : region_->backend_to_core;
}

}  // namespace robosim::backend::tier1
