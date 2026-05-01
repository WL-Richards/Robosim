#pragma once

#include "shared_memory_transport.h"
#include "shim_host_driver.h"

#include <cstdint>
#include <expected>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace robosim::backend::shim {

struct timed_robot_child_fd_handoff {
  int fd = -1;
  std::string env_value;
};

struct timed_robot_smoke_host_counters {
  std::uint64_t boot_accept_count = 0;
  std::uint64_t robot_output_count = 0;
  std::uint64_t ignored_robot_output_count = 0;
  std::uint64_t clock_publish_count = 0;
  std::uint64_t notifier_alarm_publish_count = 0;
  std::uint64_t no_due_count = 0;

  bool operator==(const timed_robot_smoke_host_counters&) const = default;
};

enum class timed_robot_smoke_host_step {
  waiting_for_boot,
  boot_accepted,
  robot_output_received,
  ignored_robot_output,
  waiting_for_notifier_trigger,
  clock_published,
  notifier_alarm_published,
  no_due_alarm,
};

/**
 * Clears FD_CLOEXEC on the single fd selected for child-process Tier 1 handoff.
 */
[[nodiscard]] std::expected<timed_robot_child_fd_handoff, int>
prepare_tier1_child_handoff_fd(const tier1::unique_fd& fd);

/**
 * Returns a child environment with exactly one ROBOSIM_TIER1_FD entry.
 */
[[nodiscard]] std::vector<std::string> build_timed_robot_child_environment(
    std::span<const std::string> base_environment, int fd);

/**
 * Maps a waitpid(2) status to the smoke launcher's process exit code.
 */
[[nodiscard]] int timed_robot_child_exit_code(int wait_status) noexcept;

/**
 * Formats a compact one-line attached smoke runtime status.
 */
[[nodiscard]] std::string format_timed_robot_smoke_host_status(
    int child_fd, const timed_robot_smoke_host_counters& counters);

/**
 * Returns the future Notifier trigger that should release the launcher from
 * waiting for robot output, or nullopt when it should keep waiting.
 */
[[nodiscard]] std::optional<std::uint64_t>
timed_robot_smoke_host_resume_time_after_robot_output(
    std::optional<std::uint64_t> next_trigger_time_us,
    std::uint64_t current_sim_time_us) noexcept;

/**
 * Returns the sim time that should release the initial post-boot wait.
 */
[[nodiscard]] std::uint64_t timed_robot_smoke_host_initial_resume_time_after_robot_output(
    std::optional<std::uint64_t> next_trigger_time_us,
    std::uint64_t current_sim_time_us) noexcept;

/**
 * Nonblocking host runtime stepper used by the timed-robot smoke launcher.
 */
class timed_robot_smoke_host_runtime {
 public:
  [[nodiscard]] static std::expected<timed_robot_smoke_host_runtime,
                                     tier1::tier1_transport_error>
  make(tier1::tier1_shared_region& region);

  [[nodiscard]] std::expected<timed_robot_smoke_host_step, tier1::tier1_transport_error>
  step(std::uint64_t sim_time_us);

  [[nodiscard]] std::expected<timed_robot_smoke_host_step, tier1::tier1_transport_error>
  poll_robot_outputs_only();

  [[nodiscard]] bool boot_accepted() const noexcept;
  [[nodiscard]] const timed_robot_smoke_host_counters& counters() const noexcept;
  [[nodiscard]] std::optional<std::uint64_t> next_active_notifier_trigger_time() const noexcept;

 private:
  explicit timed_robot_smoke_host_runtime(shim_host_runtime_driver runtime);

  shim_host_runtime_driver runtime_;
  bool boot_accepted_ = false;
  timed_robot_smoke_host_counters counters_{};
};

}  // namespace robosim::backend::shim
