#include "timed_robot_smoke_host.h"

#include <sys/wait.h>

#include <cerrno>
#include <fcntl.h>
#include <sstream>
#include <string_view>

namespace robosim::backend::shim {
namespace {

constexpr std::string_view kTier1FdEnvName = "ROBOSIM_TIER1_FD";

[[nodiscard]] bool is_no_message(const tier1::tier1_transport_error& error) {
  return error.kind == tier1::tier1_transport_error_kind::no_message;
}

[[nodiscard]] timed_robot_smoke_host_step map_runtime_pump_result(
    shim_host_runtime_pump_result result) {
  switch (result) {
    case shim_host_runtime_pump_result::clock_published:
      return timed_robot_smoke_host_step::clock_published;
    case shim_host_runtime_pump_result::notifier_alarm_published:
      return timed_robot_smoke_host_step::notifier_alarm_published;
    case shim_host_runtime_pump_result::no_due_alarm:
      return timed_robot_smoke_host_step::no_due_alarm;
  }
  return timed_robot_smoke_host_step::no_due_alarm;
}

}  // namespace

std::expected<timed_robot_child_fd_handoff, int> prepare_tier1_child_handoff_fd(
    const tier1::unique_fd& fd) {
  if (!fd.valid()) {
    return std::unexpected(EBADF);
  }
  const int flags = ::fcntl(fd.get(), F_GETFD);
  if (flags < 0) {
    return std::unexpected(errno);
  }
  if (::fcntl(fd.get(), F_SETFD, flags & ~FD_CLOEXEC) != 0) {
    return std::unexpected(errno);
  }
  return timed_robot_child_fd_handoff{fd.get(), std::to_string(fd.get())};
}

std::vector<std::string> build_timed_robot_child_environment(
    std::span<const std::string> base_environment, int fd) {
  std::vector<std::string> result;
  result.reserve(base_environment.size() + 1);
  for (const auto& entry : base_environment) {
    if (entry.starts_with(std::string{kTier1FdEnvName} + "=")) {
      continue;
    }
    result.push_back(entry);
  }
  result.push_back(std::string{kTier1FdEnvName} + "=" + std::to_string(fd));
  return result;
}

int timed_robot_child_exit_code(int wait_status) noexcept {
  if (WIFEXITED(wait_status)) {
    return WEXITSTATUS(wait_status);
  }
  if (WIFSIGNALED(wait_status)) {
    return 128 + WTERMSIG(wait_status);
  }
  return 1;
}

std::string format_timed_robot_smoke_host_status(
    int child_fd, const timed_robot_smoke_host_counters& counters) {
  std::ostringstream out;
  out << "robosim smoke host: attached ROBOSIM_TIER1_FD=" << child_fd
      << " boot=" << counters.boot_accept_count
      << " robot_outputs=" << counters.robot_output_count
      << " ignored_outputs=" << counters.ignored_robot_output_count
      << " clocks=" << counters.clock_publish_count
      << " alarms=" << counters.notifier_alarm_publish_count
      << " no_due=" << counters.no_due_count;
  return out.str();
}

std::optional<std::uint64_t> timed_robot_smoke_host_resume_time_after_robot_output(
    std::optional<std::uint64_t> next_trigger_time_us,
    std::uint64_t current_sim_time_us) noexcept {
  if (next_trigger_time_us.has_value() && *next_trigger_time_us > current_sim_time_us) {
    return next_trigger_time_us;
  }
  return std::nullopt;
}

std::uint64_t timed_robot_smoke_host_initial_resume_time_after_robot_output(
    std::optional<std::uint64_t> next_trigger_time_us,
    std::uint64_t current_sim_time_us) noexcept {
  if (next_trigger_time_us.has_value() && *next_trigger_time_us > current_sim_time_us) {
    return *next_trigger_time_us;
  }
  return current_sim_time_us;
}

timed_robot_smoke_host_runtime::timed_robot_smoke_host_runtime(
    shim_host_runtime_driver runtime)
    : runtime_(std::move(runtime)) {}

std::expected<timed_robot_smoke_host_runtime, tier1::tier1_transport_error>
timed_robot_smoke_host_runtime::make(tier1::tier1_shared_region& region) {
  auto runtime = shim_host_runtime_driver::make(region);
  if (!runtime.has_value()) {
    return std::unexpected(std::move(runtime.error()));
  }
  return timed_robot_smoke_host_runtime{std::move(*runtime)};
}

std::expected<timed_robot_smoke_host_step, tier1::tier1_transport_error>
timed_robot_smoke_host_runtime::step(std::uint64_t sim_time_us) {
  if (!boot_accepted_) {
    auto boot = runtime_.accept_boot_and_send_ack(sim_time_us);
    if (!boot.has_value()) {
      if (is_no_message(boot.error())) {
        return timed_robot_smoke_host_step::waiting_for_boot;
      }
      return std::unexpected(std::move(boot.error()));
    }
    boot_accepted_ = true;
    ++counters_.boot_accept_count;
    return timed_robot_smoke_host_step::boot_accepted;
  }

  auto robot_output = runtime_.poll_robot_outputs();
  if (!robot_output.has_value()) {
    if (!is_no_message(robot_output.error())) {
      return std::unexpected(std::move(robot_output.error()));
    }
  } else if (*robot_output == shim_host_runtime_robot_output::notifier_state_received) {
    ++counters_.robot_output_count;
    return timed_robot_smoke_host_step::robot_output_received;
  } else {
    ++counters_.ignored_robot_output_count;
    return timed_robot_smoke_host_step::ignored_robot_output;
  }

  const auto next_trigger = next_active_notifier_trigger_time();
  if (next_trigger.has_value() && *next_trigger > sim_time_us) {
    return timed_robot_smoke_host_step::waiting_for_notifier_trigger;
  }

  auto pumped = runtime_.pump_to(sim_time_us);
  if (!pumped.has_value()) {
    return std::unexpected(std::move(pumped.error()));
  }
  const auto step = map_runtime_pump_result(*pumped);
  switch (step) {
    case timed_robot_smoke_host_step::clock_published:
      ++counters_.clock_publish_count;
      break;
    case timed_robot_smoke_host_step::notifier_alarm_published:
      ++counters_.notifier_alarm_publish_count;
      break;
    case timed_robot_smoke_host_step::no_due_alarm:
      ++counters_.no_due_count;
      break;
    case timed_robot_smoke_host_step::waiting_for_boot:
    case timed_robot_smoke_host_step::boot_accepted:
    case timed_robot_smoke_host_step::robot_output_received:
    case timed_robot_smoke_host_step::ignored_robot_output:
    case timed_robot_smoke_host_step::waiting_for_notifier_trigger:
      break;
  }
  return step;
}

std::expected<timed_robot_smoke_host_step, tier1::tier1_transport_error>
timed_robot_smoke_host_runtime::poll_robot_outputs_only() {
  auto robot_output = runtime_.poll_robot_outputs();
  if (!robot_output.has_value()) {
    if (is_no_message(robot_output.error())) {
      return timed_robot_smoke_host_step::no_due_alarm;
    }
    return std::unexpected(std::move(robot_output.error()));
  }
  if (*robot_output == shim_host_runtime_robot_output::notifier_state_received) {
    ++counters_.robot_output_count;
    return timed_robot_smoke_host_step::robot_output_received;
  }
  ++counters_.ignored_robot_output_count;
  return timed_robot_smoke_host_step::ignored_robot_output;
}

bool timed_robot_smoke_host_runtime::boot_accepted() const noexcept {
  return boot_accepted_;
}

const timed_robot_smoke_host_counters& timed_robot_smoke_host_runtime::counters() const noexcept {
  return counters_;
}

std::optional<std::uint64_t> timed_robot_smoke_host_runtime::next_active_notifier_trigger_time()
    const noexcept {
  const auto& state = runtime_.latest_notifier_state();
  if (!state.has_value()) {
    return std::nullopt;
  }
  std::optional<std::uint64_t> next;
  for (std::uint32_t i = 0; i < state->count; ++i) {
    const auto& slot = state->slots[i];
    if (slot.alarm_active == 0 || slot.canceled != 0) {
      continue;
    }
    if (!next.has_value() || slot.trigger_time_us < *next) {
      next = slot.trigger_time_us;
    }
  }
  return next;
}

}  // namespace robosim::backend::shim
