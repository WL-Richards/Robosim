#include "timed_robot_smoke_host.h"

#include <sys/wait.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <atomic>
#include <fcntl.h>
#include <thread>
#include <unistd.h>
#include <vector>

using robosim::backend::shim::prepare_tier1_child_handoff_fd;
using robosim::backend::shim::timed_robot_child_exit_code;
using robosim::backend::shim::timed_robot_smoke_host_initial_resume_time_after_robot_output;
using robosim::backend::shim::timed_robot_smoke_host_resume_time_after_robot_output;
using robosim::backend::shim::timed_robot_smoke_host_runtime;
using robosim::backend::shim::format_timed_robot_smoke_host_status;
using robosim::backend::tier1::tier1_shared_mapping;

namespace {

constexpr std::uint64_t kTimedRobotPeriodUs = 20'000;
std::atomic<bool> g_stop_requested{false};

void request_stop(int) {
  g_stop_requested.store(true, std::memory_order_release);
}

[[nodiscard]] bool is_retryable_runtime_error(
    const robosim::backend::tier1::tier1_transport_error& error) {
  using robosim::backend::tier1::tier1_transport_error_kind;
  return error.kind == tier1_transport_error_kind::lane_busy ||
         error.kind == tier1_transport_error_kind::lane_in_progress;
}

[[nodiscard]] int find_child_command_index(int argc, char** argv) {
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--") == 0) {
      return i + 1;
    }
  }
  return -1;
}

}  // namespace

int main(int argc, char** argv) {
  const int command_index = find_child_command_index(argc, argv);
  if (command_index <= 0 || command_index >= argc) {
    std::fprintf(stderr, "usage: %s -- <robot-command> [args...]\n", argv[0]);
    return 2;
  }
  std::signal(SIGTERM, request_stop);
  std::signal(SIGINT, request_stop);

  auto mapping = tier1_shared_mapping::create();
  if (!mapping.has_value()) {
    std::fprintf(stderr, "robosim smoke host: failed to create Tier 1 mapping: %s\n",
                 mapping.error().message.c_str());
    return 1;
  }
  auto child_fd = mapping->duplicate_fd();
  if (!child_fd.has_value()) {
    std::fprintf(stderr, "robosim smoke host: failed to duplicate Tier 1 fd: %s\n",
                 child_fd.error().message.c_str());
    return 1;
  }
  auto handoff = prepare_tier1_child_handoff_fd(*child_fd);
  if (!handoff.has_value()) {
    std::fprintf(stderr, "robosim smoke host: failed to prepare child fd: errno=%d\n",
                 handoff.error());
    return 1;
  }
  std::fprintf(stderr,
               "%s\n",
               format_timed_robot_smoke_host_status(handoff->fd, {}).c_str());

  auto runtime = timed_robot_smoke_host_runtime::make(mapping->region());
  if (!runtime.has_value()) {
    std::fprintf(stderr, "robosim smoke host: failed to create runtime: %s\n",
                 runtime.error().message.c_str());
    return 1;
  }

  const pid_t child = ::fork();
  if (child < 0) {
    std::perror("robosim smoke host: fork failed");
    return 1;
  }
  if (child == 0) {
    if (::setenv("ROBOSIM_TIER1_FD", handoff->env_value.c_str(), 1) != 0) {
      std::perror("robosim smoke host: setenv failed");
      _exit(127);
    }
    ::execvp(argv[command_index], &argv[command_index]);
    std::perror("robosim smoke host: execvp failed");
    _exit(127);
  }

  child_fd = robosim::backend::tier1::unique_fd{};

  std::uint64_t sim_time_us = kTimedRobotPeriodUs;
  auto next_tick =
      std::chrono::steady_clock::now() + std::chrono::microseconds{kTimedRobotPeriodUs};
  bool tick_complete = false;
  bool waiting_for_robot_output = false;
  bool waiting_requires_future_trigger = false;

  for (;;) {
    if (g_stop_requested.load(std::memory_order_acquire)) {
      (void)::kill(child, SIGTERM);
      int interrupted_status = 0;
      (void)::waitpid(child, &interrupted_status, 0);
      std::fprintf(stderr,
                   "%s\n",
                   format_timed_robot_smoke_host_status(handoff->fd, runtime->counters()).c_str());
      return 128 + SIGTERM;
    }

    int wait_status = 0;
    const pid_t waited = ::waitpid(child, &wait_status, WNOHANG);
    if (waited == child) {
      std::fprintf(stderr,
                   "%s\n",
                   format_timed_robot_smoke_host_status(handoff->fd, runtime->counters()).c_str());
      return timed_robot_child_exit_code(wait_status);
    }
    if (waited < 0) {
      std::perror("robosim smoke host: waitpid failed");
      return 1;
    }

    if (!runtime->boot_accepted()) {
      const auto step = runtime->step(0);
      if (!step.has_value() && !is_retryable_runtime_error(step.error())) {
        std::fprintf(stderr, "robosim smoke host: boot step failed: %s\n",
                     step.error().message.c_str());
        return 1;
      }
      if (step.has_value() &&
          *step == robosim::backend::shim::timed_robot_smoke_host_step::boot_accepted) {
        std::fprintf(stderr,
                     "%s\n",
                     format_timed_robot_smoke_host_status(handoff->fd, runtime->counters())
                         .c_str());
        waiting_for_robot_output = true;
        waiting_requires_future_trigger = false;
      }
      std::this_thread::yield();
      continue;
    }

    if (waiting_for_robot_output) {
      const auto step = runtime->poll_robot_outputs_only();
      if (!step.has_value()) {
        if (is_retryable_runtime_error(step.error())) {
          std::this_thread::yield();
          continue;
        }
        std::fprintf(stderr, "robosim smoke host: robot-output poll failed: %s\n",
                     step.error().message.c_str());
        return 1;
      }
      if (*step == robosim::backend::shim::timed_robot_smoke_host_step::robot_output_received) {
        if (waiting_requires_future_trigger) {
          const auto resume_time = timed_robot_smoke_host_resume_time_after_robot_output(
              runtime->next_active_notifier_trigger_time(), sim_time_us);
          if (resume_time.has_value()) {
            sim_time_us = *resume_time;
            waiting_for_robot_output = false;
          }
        } else {
          const auto resume_time = timed_robot_smoke_host_initial_resume_time_after_robot_output(
              runtime->next_active_notifier_trigger_time(), sim_time_us);
          sim_time_us = resume_time;
          waiting_for_robot_output = false;
        }
      }
      std::this_thread::yield();
      continue;
    }

    const auto now = std::chrono::steady_clock::now();
    if (tick_complete) {
      if (now < next_tick) {
        std::this_thread::sleep_until(next_tick);
      }
      sim_time_us += kTimedRobotPeriodUs;
      next_tick += std::chrono::microseconds{kTimedRobotPeriodUs};
      tick_complete = false;
    }

    const auto step = runtime->step(sim_time_us);
    if (!step.has_value()) {
      if (is_retryable_runtime_error(step.error())) {
        std::this_thread::yield();
        continue;
      }
      std::fprintf(stderr, "robosim smoke host: runtime step failed: %s\n",
                   step.error().message.c_str());
      return 1;
    }
    if (*step == robosim::backend::shim::timed_robot_smoke_host_step::robot_output_received) {
      const auto resume_time = timed_robot_smoke_host_resume_time_after_robot_output(
          runtime->next_active_notifier_trigger_time(), sim_time_us);
      if (resume_time.has_value()) {
        sim_time_us = *resume_time;
      }
    } else if (*step ==
               robosim::backend::shim::timed_robot_smoke_host_step::
                   waiting_for_notifier_trigger) {
      const auto resume_time = timed_robot_smoke_host_resume_time_after_robot_output(
          runtime->next_active_notifier_trigger_time(), sim_time_us);
      if (resume_time.has_value()) {
        sim_time_us = *resume_time;
      }
    } else if (*step ==
               robosim::backend::shim::timed_robot_smoke_host_step::notifier_alarm_published) {
      waiting_for_robot_output = true;
      waiting_requires_future_trigger = true;
    } else if (*step == robosim::backend::shim::timed_robot_smoke_host_step::no_due_alarm) {
      const auto next_trigger = runtime->next_active_notifier_trigger_time();
      if (next_trigger.has_value() && *next_trigger > sim_time_us) {
        sim_time_us = *next_trigger;
      } else if (!waiting_for_robot_output) {
        tick_complete = true;
      }
    }
    std::this_thread::yield();
  }
}
