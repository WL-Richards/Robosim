#include "hal_c.h"
#include "jni_minimal.h"
#include "shim_core.h"

#include <algorithm>
#include <atomic>
#include <array>
#include <charconv>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <mutex>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <thread>

namespace robosim::backend::shim {

namespace {

constexpr std::uint64_t kJniFallbackBootSimTimeUs = 0;
constexpr std::uint64_t kJniFallbackInitialClockSimTimeUs = 20'000;
constexpr const char* kRobosimTier1FdEnv = "ROBOSIM_TIER1_FD";

void production_fallback_sleep(std::uint64_t delay_us) {
  std::this_thread::sleep_for(std::chrono::microseconds{delay_us});
}

void production_attached_poll_idle() {
  std::this_thread::sleep_for(std::chrono::milliseconds{1});
}

struct hal_jni_launch_state {
  tier1::tier1_shared_region fallback_region{};
  std::optional<tier1::tier1_shared_mapping> attached_mapping;
  std::optional<tier1::tier1_endpoint> core_endpoint;
  std::optional<shim_core> shim;
  bool fallback_owned = false;
  bool core_boot_drained = false;
  std::atomic<bool> attached_poll_stop{false};
  std::atomic<bool> attached_poll_active{false};
  std::atomic<std::uint64_t> attached_poll_count{0};
  std::thread attached_poll_thread;
  std::optional<std::uint64_t> last_paced_sim_time_us;
  void (*sleep_for)(std::uint64_t delay_us) = production_fallback_sleep;
  void (*attached_poll_idle)() = production_attached_poll_idle;
};

std::optional<hal_jni_launch_state> g_launch_state;

boot_descriptor make_fallback_boot_descriptor() {
  boot_descriptor desc{};
  desc.runtime = runtime_type::roborio_2;
  desc.team_number = 0;
  desc.vendor_capabilities = 0;
  constexpr char kWpilibVersion[] = "2026.2.1";
  std::memcpy(desc.wpilib_version.data(), kWpilibVersion, sizeof(kWpilibVersion));
  return desc;
}

std::optional<int> parse_tier1_fd_env(std::string_view text) {
  if (text.empty()) {
    return std::nullopt;
  }

  long long value = -1;
  const auto* begin = text.data();
  const auto* end = text.data() + text.size();
  const auto [ptr, ec] = std::from_chars(begin, end, value);
  if (ec != std::errc{} || ptr != end || value < 0 ||
      value > std::numeric_limits<int>::max()) {
    return std::nullopt;
  }
  return static_cast<int>(value);
}

bool ensure_jni_fallback_shim_installed() {
  if (shim_core::current() != nullptr) {
    return true;
  }

  auto sleep_for = production_fallback_sleep;
  if (g_launch_state.has_value()) {
    sleep_for = g_launch_state->sleep_for;
  }
  g_launch_state.emplace();
  g_launch_state->sleep_for = sleep_for;
  g_launch_state->fallback_owned = true;
  auto endpoint =
      tier1::tier1_endpoint::make(g_launch_state->fallback_region, direction::backend_to_core);
  if (!endpoint.has_value()) {
    g_launch_state.reset();
    return false;
  }

  auto shim = shim_core::make(
      std::move(*endpoint), make_fallback_boot_descriptor(), kJniFallbackBootSimTimeUs);
  if (!shim.has_value()) {
    g_launch_state.reset();
    return false;
  }

  auto core_endpoint =
      tier1::tier1_endpoint::make(g_launch_state->fallback_region, direction::core_to_backend);
  if (!core_endpoint.has_value()) {
    g_launch_state.reset();
    return false;
  }

  g_launch_state->core_endpoint = std::move(*core_endpoint);
  g_launch_state->shim = std::move(*shim);
  shim_core::install_global(&*g_launch_state->shim);
  return true;
}

void attached_poll_loop(hal_jni_launch_state* state) {
  state->attached_poll_active.store(true, std::memory_order_release);
  while (!state->attached_poll_stop.load(std::memory_order_acquire)) {
    if (state->shim.has_value()) {
      (void)state->shim->poll();
      state->attached_poll_count.fetch_add(1, std::memory_order_relaxed);
    }
    if (state->attached_poll_idle != nullptr) {
      state->attached_poll_idle();
    }
  }
  state->attached_poll_active.store(false, std::memory_order_release);
}

void start_attached_poll_loop(hal_jni_launch_state& state) {
  state.attached_poll_stop.store(false, std::memory_order_release);
  state.attached_poll_active.store(true, std::memory_order_release);
  state.attached_poll_count.store(0, std::memory_order_relaxed);
  state.attached_poll_thread = std::thread(attached_poll_loop, &state);
}

void stop_attached_poll_loop(hal_jni_launch_state& state) {
  state.attached_poll_stop.store(true, std::memory_order_release);
  if (state.attached_poll_thread.joinable()) {
    state.attached_poll_thread.join();
  }
  state.attached_poll_active.store(false, std::memory_order_release);
}

bool ensure_jni_attached_shim_installed(std::string_view fd_text) {
  if (shim_core::current() != nullptr) {
    return true;
  }

  auto parsed_fd = parse_tier1_fd_env(fd_text);
  if (!parsed_fd.has_value()) {
    return false;
  }

  tier1::unique_fd inherited_fd{*parsed_fd};
  auto mapping = tier1::tier1_shared_mapping::map_existing(inherited_fd);
  if (!mapping.has_value()) {
    return false;
  }

  auto sleep_for = production_fallback_sleep;
  auto attached_poll_idle = production_attached_poll_idle;
  if (g_launch_state.has_value()) {
    sleep_for = g_launch_state->sleep_for;
    attached_poll_idle = g_launch_state->attached_poll_idle;
  }
  g_launch_state.emplace();
  g_launch_state->sleep_for = sleep_for;
  g_launch_state->attached_poll_idle = attached_poll_idle;
  g_launch_state->attached_mapping = std::move(*mapping);
  g_launch_state->fallback_owned = false;

  auto endpoint =
      tier1::tier1_endpoint::make(g_launch_state->attached_mapping->region(),
                                  direction::backend_to_core);
  if (!endpoint.has_value()) {
    g_launch_state.reset();
    return false;
  }

  auto shim = shim_core::make(
      std::move(*endpoint), make_fallback_boot_descriptor(), kJniFallbackBootSimTimeUs);
  if (!shim.has_value()) {
    g_launch_state.reset();
    return false;
  }

  g_launch_state->shim = std::move(*shim);
  shim_core::install_global(&*g_launch_state->shim);
  start_attached_poll_loop(*g_launch_state);
  return true;
}

std::uint64_t attached_notifier_flush_time_us(const shim_core& shim,
                                              std::uint64_t fallback_time_us) {
  const auto& clock = shim.latest_clock_state();
  if (clock.has_value()) {
    return clock->sim_time_us;
  }
  return fallback_time_us;
}

bool attached_notifier_handle_exists(std::int32_t handle) {
  if (!g_launch_state.has_value() || g_launch_state->fallback_owned ||
      !g_launch_state->shim.has_value() || shim_core::current() != &*g_launch_state->shim) {
    return false;
  }
  const notifier_state state = g_launch_state->shim->current_notifier_state();
  for (std::uint32_t i = 0; i < state.count; ++i) {
    if (state.slots[i].handle == handle) {
      return true;
    }
  }
  return false;
}

void flush_attached_notifier_state(std::uint64_t fallback_time_us) {
  if (!g_launch_state.has_value() || g_launch_state->fallback_owned ||
      !g_launch_state->shim.has_value() || shim_core::current() != &*g_launch_state->shim) {
    return;
  }
  shim_core& shim = *g_launch_state->shim;
  (void)shim.flush_notifier_state(attached_notifier_flush_time_us(shim, fallback_time_us));
}

void pace_fallback_to(hal_jni_launch_state& state, std::uint64_t sim_time_us) {
  if (!state.last_paced_sim_time_us.has_value()) {
    state.last_paced_sim_time_us = sim_time_us;
    return;
  }

  const std::uint64_t last = *state.last_paced_sim_time_us;
  if (sim_time_us > last) {
    state.sleep_for(sim_time_us - last);
  }
  state.last_paced_sim_time_us = sim_time_us;
}

std::span<const std::uint8_t> active_notifier_alarm_prefix(const notifier_alarm_batch& batch) {
  const std::size_t active_size =
      offsetof(notifier_alarm_batch, events) +
      static_cast<std::size_t>(batch.count) * sizeof(notifier_alarm_event);
  return {reinterpret_cast<const std::uint8_t*>(&batch), active_size};
}

std::span<const std::uint8_t> bytes_of_clock_state(const clock_state& state) {
  return {reinterpret_cast<const std::uint8_t*>(&state), sizeof(clock_state)};
}

clock_state make_jni_fallback_clock_state(std::uint64_t sim_time_us) {
  clock_state state{};
  state.sim_time_us = sim_time_us;
  state.system_active = 1;
  state.system_time_valid = 1;
  state.rsl_state = 1;
  return state;
}

std::optional<notifier_alarm_event> active_alarm_for_handle(shim_core& shim,
                                                            std::int32_t notifier_handle) {
  const notifier_state state = shim.current_notifier_state();
  for (std::uint32_t i = 0; i < state.count; ++i) {
    const notifier_slot& slot = state.slots[i];
    if (slot.handle == notifier_handle && slot.alarm_active != 0) {
      notifier_alarm_event event{};
      event.fired_at_us = slot.trigger_time_us;
      event.handle = notifier_handle;
      return event;
    }
  }
  return std::nullopt;
}

bool ensure_fallback_core_handshake(hal_jni_launch_state& state) {
  if (!state.shim.has_value() || !state.core_endpoint.has_value()) {
    return false;
  }
  if (state.shim->is_connected()) {
    state.core_boot_drained = true;
    return true;
  }

  tier1::tier1_endpoint& core = *state.core_endpoint;
  if (!state.core_boot_drained) {
    auto boot = core.try_receive();
    if (!boot.has_value() && boot.error().kind != tier1::tier1_transport_error_kind::no_message) {
      return false;
    }
    state.core_boot_drained = true;
  }

  auto ack = core.send(envelope_kind::boot_ack, schema_id::none, {}, kJniFallbackBootSimTimeUs);
  if (!ack.has_value()) {
    return false;
  }

  auto polled = state.shim->poll();
  return polled.has_value() && state.shim->is_connected();
}

bool is_retryable_send_failure(const tier1::tier1_transport_error& error) {
  return error.kind == tier1::tier1_transport_error_kind::lane_busy ||
         error.kind == tier1::tier1_transport_error_kind::lane_in_progress;
}

bool send_and_poll_fallback_tick(hal_jni_launch_state& state,
                                 schema_id payload_schema,
                                 std::span<const std::uint8_t> payload,
                                 std::uint64_t sim_time_us) {
  if (!state.shim.has_value() || !state.core_endpoint.has_value()) {
    return false;
  }

  tier1::tier1_endpoint& core = *state.core_endpoint;
  const auto send_tick = [&]() {
    return core.send(envelope_kind::tick_boundary, payload_schema, payload, sim_time_us);
  };

  auto sent = send_tick();
  if (!sent.has_value()) {
    if (!is_retryable_send_failure(sent.error())) {
      return false;
    }
    if (!state.shim->poll().has_value()) {
      return false;
    }
    sent = send_tick();
    if (!sent.has_value()) {
      return false;
    }
  }

  return state.shim->poll().has_value();
}

}  // namespace

void reset_hal_jni_launch_state_for_test() {
  if (g_launch_state.has_value()) {
    stop_attached_poll_loop(*g_launch_state);
  }
  if (g_launch_state.has_value() && g_launch_state->shim.has_value() &&
      shim_core::current() == &*g_launch_state->shim) {
    shim_core::install_global(nullptr);
  }
  g_launch_state.reset();
}

const char* hal_jni_tier1_fd_env_var_for_test() {
  return kRobosimTier1FdEnv;
}

bool hal_jni_attached_poll_loop_active_for_test() {
  return g_launch_state.has_value() &&
         g_launch_state->attached_poll_active.load(std::memory_order_acquire);
}

std::uint64_t hal_jni_attached_poll_count_for_test() {
  if (!g_launch_state.has_value()) {
    return 0;
  }
  return g_launch_state->attached_poll_count.load(std::memory_order_relaxed);
}

void set_hal_jni_attached_poll_idle_for_test(void (*idle)()) {
  if (!g_launch_state.has_value()) {
    g_launch_state.emplace();
  }
  g_launch_state->attached_poll_idle = idle == nullptr ? production_attached_poll_idle : idle;
}

bool hal_jni_attached_poll_idle_hook_is_default_for_test() {
  if (!g_launch_state.has_value()) {
    return true;
  }
  return g_launch_state->attached_poll_idle == production_attached_poll_idle;
}

void set_hal_jni_fallback_sleep_for_test(void (*sleep_for)(std::uint64_t delay_us)) {
  if (!g_launch_state.has_value()) {
    g_launch_state.emplace();
  }
  g_launch_state->sleep_for = sleep_for == nullptr ? production_fallback_sleep : sleep_for;
}

bool hal_jni_fallback_sleep_hook_is_default_for_test() {
  if (!g_launch_state.has_value()) {
    return true;
  }
  return g_launch_state->sleep_for == production_fallback_sleep;
}

std::optional<std::uint64_t> hal_jni_fallback_last_paced_sim_time_for_test() {
  if (!g_launch_state.has_value()) {
    return std::nullopt;
  }
  return g_launch_state->last_paced_sim_time_us;
}

bool hal_jni_launch_state_has_fallback_for_test() {
  return g_launch_state.has_value() && g_launch_state->fallback_owned &&
         g_launch_state->shim.has_value();
}

bool hal_jni_launch_state_has_attached_mapping_for_test() {
  return g_launch_state.has_value() && !g_launch_state->fallback_owned &&
         g_launch_state->attached_mapping.has_value() && g_launch_state->shim.has_value();
}

shim_core* hal_jni_launch_state_shim_for_test() {
  if (!g_launch_state.has_value() || !g_launch_state->shim.has_value()) {
    return nullptr;
  }
  return &*g_launch_state->shim;
}

tier1::tier1_shared_region* hal_jni_launch_state_region_for_test() {
  if (!g_launch_state.has_value()) {
    return nullptr;
  }
  if (g_launch_state->attached_mapping.has_value()) {
    return &g_launch_state->attached_mapping->region();
  }
  return &g_launch_state->fallback_region;
}

tier1::tier1_endpoint* hal_jni_launch_state_core_endpoint_for_test() {
  if (!g_launch_state.has_value() || !g_launch_state->core_endpoint.has_value()) {
    return nullptr;
  }
  return &*g_launch_state->core_endpoint;
}

bool pump_jni_fallback_notifier_alarm(std::int32_t notifier_handle) {
  if (!g_launch_state.has_value() || !g_launch_state->shim.has_value() ||
      !g_launch_state->core_endpoint.has_value() || !g_launch_state->fallback_owned) {
    return false;
  }

  shim_core& fallback = *g_launch_state->shim;
  if (shim_core::current() != &fallback) {
    return false;
  }

  const auto event = active_alarm_for_handle(fallback, notifier_handle);
  if (!event.has_value()) {
    return false;
  }

  if (!ensure_fallback_core_handshake(*g_launch_state)) {
    return false;
  }

  pace_fallback_to(*g_launch_state, event->fired_at_us);

  const clock_state clock = make_jni_fallback_clock_state(event->fired_at_us);
  if (!send_and_poll_fallback_tick(*g_launch_state,
                                   schema_id::clock_state,
                                   bytes_of_clock_state(clock),
                                   event->fired_at_us)) {
    return false;
  }

  notifier_alarm_batch batch{};
  batch.count = 1;
  batch.events[0] = *event;

  return send_and_poll_fallback_tick(*g_launch_state,
                                     schema_id::notifier_alarm_batch,
                                     active_notifier_alarm_prefix(batch),
                                     event->fired_at_us);
}

bool pump_jni_fallback_clock_state(std::uint64_t sim_time_us) {
  if (!g_launch_state.has_value() || !g_launch_state->shim.has_value() ||
      !g_launch_state->core_endpoint.has_value() || !g_launch_state->fallback_owned) {
    return false;
  }

  shim_core& fallback = *g_launch_state->shim;
  if (shim_core::current() != &fallback) {
    return false;
  }
  if (!ensure_fallback_core_handshake(*g_launch_state)) {
    return false;
  }

  pace_fallback_to(*g_launch_state, sim_time_us);

  const clock_state clock = make_jni_fallback_clock_state(sim_time_us);
  return send_and_poll_fallback_tick(
      *g_launch_state, schema_id::clock_state, bytes_of_clock_state(clock), sim_time_us);
}

}  // namespace robosim::backend::shim

extern "C" {

using JNIEnv = robosim::backend::shim::jni::JNIEnv;
using jclass = robosim::backend::shim::jni::jclass;
using jstring = robosim::backend::shim::jni::jstring;
using jboolean = robosim::backend::shim::jni::jboolean;
using jbyte = robosim::backend::shim::jni::jbyte;
using jint = robosim::backend::shim::jni::jint;
using jlong = robosim::backend::shim::jni::jlong;
using jfloatArray = robosim::backend::shim::jni::jfloatArray;
using jintArray = robosim::backend::shim::jni::jintArray;
using jshortArray = robosim::backend::shim::jni::jshortArray;
using jobject = robosim::backend::shim::jni::jobject;

extern "C++" {

namespace {

std::int32_t bounded_count(std::int16_t count, std::int32_t max_count) {
  return std::clamp(static_cast<std::int32_t>(count), 0, max_count);
}

void copy_float_array(JNIEnv* env, jfloatArray dest, const float* values, std::int32_t count) {
  if (env == nullptr || env->functions == nullptr ||
      env->functions->SetFloatArrayRegion == nullptr || dest == nullptr || count <= 0) {
    return;
  }
  env->functions->SetFloatArrayRegion(env, dest, 0, count, values);
}

void copy_int_array(JNIEnv* env, jintArray dest, const jint* values, std::int32_t count) {
  if (env == nullptr || env->functions == nullptr || env->functions->SetIntArrayRegion == nullptr ||
      dest == nullptr || count <= 0) {
    return;
  }
  env->functions->SetIntArrayRegion(env, dest, 0, count, values);
}

void copy_short_array(JNIEnv* env,
                      jshortArray dest,
                      const std::int16_t* values,
                      std::int32_t count) {
  if (env == nullptr || env->functions == nullptr ||
      env->functions->SetShortArrayRegion == nullptr || dest == nullptr || count <= 0) {
    return;
  }
  env->functions->SetShortArrayRegion(
      env, dest, 0, count, reinterpret_cast<const robosim::backend::shim::jni::jshort*>(values));
}

std::uint8_t* direct_buffer_byte(JNIEnv* env, jobject buffer) {
  if (env == nullptr || env->functions == nullptr ||
      env->functions->GetDirectBufferAddress == nullptr || buffer == nullptr) {
    return nullptr;
  }
  return static_cast<std::uint8_t*>(env->functions->GetDirectBufferAddress(env, buffer));
}

std::string nul_terminated_field(const char* data, std::size_t capacity) {
  const void* nul = std::memchr(data, '\0', capacity);
  const std::size_t size =
      nul == nullptr ? capacity : static_cast<std::size_t>(static_cast<const char*>(nul) - data);
  return std::string(data, size);
}

std::string sized_nul_terminated_field(const std::uint8_t* data,
                                       std::size_t explicit_size,
                                       std::size_t capacity) {
  const std::size_t bounded_size = std::min(explicit_size, capacity);
  const void* nul = std::memchr(data, '\0', bounded_size);
  const std::size_t size =
      nul == nullptr ? bounded_size
                     : static_cast<std::size_t>(static_cast<const std::uint8_t*>(nul) - data);
  return std::string(reinterpret_cast<const char*>(data), size);
}

bool can_call_match_info_set_data(JNIEnv* env, jobject info) {
  return env != nullptr && info != nullptr && env->functions != nullptr &&
         env->functions->GetObjectClass != nullptr && env->functions->GetMethodID != nullptr &&
         env->functions->NewStringUTF != nullptr && env->functions->CallVoidMethod != nullptr;
}

class scoped_utf_chars {
 public:
  scoped_utf_chars(JNIEnv* env, jstring value) : env_(env), value_(value) {
    if (env_ == nullptr || value_ == nullptr || env_->functions == nullptr ||
        env_->functions->GetStringUTFChars == nullptr) {
      return;
    }
    chars_ = env_->functions->GetStringUTFChars(env_, value_, nullptr);
  }

  ~scoped_utf_chars() {
    if (chars_ != nullptr && env_ != nullptr && env_->functions != nullptr &&
        env_->functions->ReleaseStringUTFChars != nullptr) {
      env_->functions->ReleaseStringUTFChars(env_, value_, chars_);
    }
  }

  scoped_utf_chars(const scoped_utf_chars&) = delete;
  scoped_utf_chars& operator=(const scoped_utf_chars&) = delete;

  const char* c_str() const noexcept { return chars_; }

 private:
  JNIEnv* env_ = nullptr;
  jstring value_ = nullptr;
  const char* chars_ = nullptr;
};

}  // namespace

}  // extern "C++"

jboolean Java_edu_wpi_first_hal_HAL_initialize(JNIEnv*, jclass, jint timeout, jint mode) {
  const char* tier1_fd = std::getenv(robosim::backend::shim::kRobosimTier1FdEnv);
  const bool installed =
      tier1_fd != nullptr
          ? robosim::backend::shim::ensure_jni_attached_shim_installed(tier1_fd)
          : robosim::backend::shim::ensure_jni_fallback_shim_installed();
  if (!installed) {
    return 0;
  }
  return HAL_Initialize(static_cast<std::int32_t>(timeout), static_cast<std::int32_t>(mode)) != 0
             ? static_cast<jboolean>(1)
             : static_cast<jboolean>(0);
}

void Java_edu_wpi_first_hal_HAL_shutdown(JNIEnv*, jclass) {
  auto* jni_owned = robosim::backend::shim::hal_jni_launch_state_shim_for_test();
  HAL_Shutdown();
  if (jni_owned != nullptr) {
    robosim::backend::shim::reset_hal_jni_launch_state_for_test();
  }
}

jboolean Java_edu_wpi_first_hal_HAL_hasMain(JNIEnv*, jclass) {
  return 0;
}

void Java_edu_wpi_first_hal_HAL_runMain(JNIEnv*, jclass) {}

void Java_edu_wpi_first_hal_HAL_exitMain(JNIEnv*, jclass) {}

void Java_edu_wpi_first_hal_HAL_terminate(JNIEnv*, jclass) {}

jboolean Java_edu_wpi_first_hal_DriverStationJNI_refreshDSData(JNIEnv*, jclass) {
  return HAL_RefreshDSData() != 0 ? static_cast<jboolean>(1) : static_cast<jboolean>(0);
}

void Java_edu_wpi_first_hal_DriverStationJNI_observeUserProgramStarting(JNIEnv*, jclass) {
  HAL_ObserveUserProgramStarting();
}

void Java_edu_wpi_first_hal_DriverStationJNI_observeUserProgramDisabled(JNIEnv*, jclass) {
  HAL_ObserveUserProgramDisabled();
}

void Java_edu_wpi_first_hal_DriverStationJNI_observeUserProgramAutonomous(JNIEnv*, jclass) {
  HAL_ObserveUserProgramAutonomous();
}

void Java_edu_wpi_first_hal_DriverStationJNI_observeUserProgramTeleop(JNIEnv*, jclass) {
  HAL_ObserveUserProgramTeleop();
}

void Java_edu_wpi_first_hal_DriverStationJNI_observeUserProgramTest(JNIEnv*, jclass) {
  HAL_ObserveUserProgramTest();
}

jint Java_edu_wpi_first_hal_DriverStationJNI_sendError(JNIEnv* env,
                                                       jclass,
                                                       jboolean is_error,
                                                       jint error_code,
                                                       jboolean is_lv_code,
                                                       jstring details,
                                                       jstring location,
                                                       jstring call_stack,
                                                       jboolean print_msg) {
  scoped_utf_chars details_chars{env, details};
  scoped_utf_chars location_chars{env, location};
  scoped_utf_chars call_stack_chars{env, call_stack};

  return static_cast<jint>(HAL_SendError(is_error != 0 ? 1 : 0,
                                         static_cast<std::int32_t>(error_code),
                                         is_lv_code != 0 ? 1 : 0,
                                         details_chars.c_str(),
                                         location_chars.c_str(),
                                         call_stack_chars.c_str(),
                                         print_msg != 0 ? 1 : 0));
}

jint Java_edu_wpi_first_hal_DriverStationJNI_nativeGetControlWord(JNIEnv*, jclass) {
  HAL_ControlWord word{};
  (void)HAL_GetControlWord(&word);
  std::uint32_t bits = 0;
  std::memcpy(&bits, &word, sizeof(bits));
  return static_cast<jint>(bits);
}

jint Java_edu_wpi_first_hal_DriverStationJNI_nativeGetAllianceStation(JNIEnv*, jclass) {
  std::int32_t status = 0;
  return static_cast<jint>(HAL_GetAllianceStation(&status));
}

jint Java_edu_wpi_first_hal_DriverStationJNI_getJoystickAxes(JNIEnv* env,
                                                             jclass,
                                                             jbyte joystick_num,
                                                             jfloatArray axes_array) {
  HAL_JoystickAxes axes{};
  (void)HAL_GetJoystickAxes(static_cast<std::int32_t>(joystick_num), &axes);
  const std::int32_t count = bounded_count(axes.count, HAL_kMaxJoystickAxes);
  copy_float_array(env, axes_array, axes.axes, count);
  return static_cast<jint>(count);
}

jint Java_edu_wpi_first_hal_DriverStationJNI_getJoystickAxesRaw(JNIEnv* env,
                                                                jclass,
                                                                jbyte joystick_num,
                                                                jintArray raw_axes_array) {
  HAL_JoystickAxes axes{};
  (void)HAL_GetJoystickAxes(static_cast<std::int32_t>(joystick_num), &axes);
  const std::int32_t count = bounded_count(axes.count, HAL_kMaxJoystickAxes);

  std::array<jint, HAL_kMaxJoystickAxes> raw{};
  for (std::int32_t i = 0; i < count; ++i) {
    raw[static_cast<std::size_t>(i)] = static_cast<jint>(axes.raw[static_cast<std::size_t>(i)]);
  }
  copy_int_array(env, raw_axes_array, raw.data(), count);
  return static_cast<jint>(count);
}

jint Java_edu_wpi_first_hal_DriverStationJNI_getJoystickPOVs(JNIEnv* env,
                                                             jclass,
                                                             jbyte joystick_num,
                                                             jshortArray povs_array) {
  HAL_JoystickPOVs povs{};
  (void)HAL_GetJoystickPOVs(static_cast<std::int32_t>(joystick_num), &povs);
  const std::int32_t count = bounded_count(povs.count, HAL_kMaxJoystickPOVs);
  copy_short_array(env, povs_array, povs.povs, count);
  return static_cast<jint>(count);
}

jint Java_edu_wpi_first_hal_DriverStationJNI_getJoystickButtons(JNIEnv* env,
                                                                jclass,
                                                                jbyte joystick_num,
                                                                jobject count_buffer) {
  HAL_JoystickButtons buttons{};
  (void)HAL_GetJoystickButtons(static_cast<std::int32_t>(joystick_num), &buttons);
  if (auto* count = direct_buffer_byte(env, count_buffer); count != nullptr) {
    *count = buttons.count;
  }
  return static_cast<jint>(buttons.buttons);
}

jint Java_edu_wpi_first_hal_DriverStationJNI_getMatchInfo(JNIEnv* env, jclass, jobject info) {
  HAL_MatchInfo match{};
  const std::int32_t status = HAL_GetMatchInfo(&match);
  if (!can_call_match_info_set_data(env, info)) {
    return static_cast<jint>(status);
  }

  jclass info_class = env->functions->GetObjectClass(env, info);
  if (info_class == nullptr) {
    return static_cast<jint>(status);
  }
  robosim::backend::shim::jni::jmethodID set_data = env->functions->GetMethodID(
      env, info_class, "setData", "(Ljava/lang/String;Ljava/lang/String;III)V");
  if (set_data == nullptr) {
    return static_cast<jint>(status);
  }

  const std::string event_name = nul_terminated_field(match.eventName, sizeof(match.eventName));
  const std::string game_specific_message =
      sized_nul_terminated_field(match.gameSpecificMessage,
                                 static_cast<std::size_t>(match.gameSpecificMessageSize),
                                 sizeof(match.gameSpecificMessage));
  jstring event = env->functions->NewStringUTF(env, event_name.c_str());
  jstring game = env->functions->NewStringUTF(env, game_specific_message.c_str());
  env->functions->CallVoidMethod(env,
                                 info,
                                 set_data,
                                 event,
                                 game,
                                 static_cast<jint>(match.matchNumber),
                                 static_cast<jint>(match.replayNumber),
                                 static_cast<jint>(match.matchType));
  return static_cast<jint>(status);
}

jint Java_edu_wpi_first_hal_DriverStationJNI_report(
    JNIEnv*, jclass, jint resource, jint instance_number, jint context, jstring) {
  return static_cast<jint>(HAL_Report(static_cast<std::int32_t>(resource),
                                      static_cast<std::int32_t>(instance_number),
                                      static_cast<std::int32_t>(context),
                                      nullptr));
}

jboolean Java_edu_wpi_first_hal_NotifierJNI_setHALThreadPriority(JNIEnv*,
                                                                 jclass,
                                                                 jboolean real_time,
                                                                 jint priority) {
  std::int32_t status = 0;
  return HAL_SetNotifierThreadPriority(
             real_time != 0 ? 1 : 0, static_cast<std::int32_t>(priority), &status) != 0
             ? static_cast<jboolean>(1)
             : static_cast<jboolean>(0);
}

jint Java_edu_wpi_first_hal_NotifierJNI_initializeNotifier(JNIEnv*, jclass) {
  std::int32_t status = 0;
  return static_cast<jint>(HAL_InitializeNotifier(&status));
}

void Java_edu_wpi_first_hal_NotifierJNI_setNotifierName(JNIEnv* env,
                                                        jclass,
                                                        jint notifier_handle,
                                                        jstring name) {
  scoped_utf_chars name_chars{env, name};
  std::int32_t status = 0;
  HAL_SetNotifierName(
      static_cast<HAL_NotifierHandle>(notifier_handle), name_chars.c_str(), &status);
}

void Java_edu_wpi_first_hal_NotifierJNI_stopNotifier(JNIEnv*, jclass, jint notifier_handle) {
  std::int32_t status = 0;
  HAL_StopNotifier(static_cast<HAL_NotifierHandle>(notifier_handle), &status);
  if (status == robosim::backend::shim::kHalSuccess) {
    robosim::backend::shim::flush_attached_notifier_state(0);
  }
}

void Java_edu_wpi_first_hal_NotifierJNI_cleanNotifier(JNIEnv*, jclass, jint notifier_handle) {
  const bool existed =
      robosim::backend::shim::attached_notifier_handle_exists(
          static_cast<std::int32_t>(notifier_handle));
  HAL_CleanNotifier(static_cast<HAL_NotifierHandle>(notifier_handle));
  if (existed) {
    robosim::backend::shim::flush_attached_notifier_state(0);
  }
}

void Java_edu_wpi_first_hal_NotifierJNI_updateNotifierAlarm(JNIEnv*,
                                                            jclass,
                                                            jint notifier_handle,
                                                            jlong trigger_time) {
  std::int32_t status = 0;
  HAL_UpdateNotifierAlarm(static_cast<HAL_NotifierHandle>(notifier_handle),
                          static_cast<std::uint64_t>(trigger_time),
                          &status);
  if (status == robosim::backend::shim::kHalSuccess) {
    robosim::backend::shim::flush_attached_notifier_state(
        static_cast<std::uint64_t>(trigger_time));
  }
}

void Java_edu_wpi_first_hal_NotifierJNI_cancelNotifierAlarm(JNIEnv*, jclass, jint notifier_handle) {
  std::int32_t status = 0;
  HAL_CancelNotifierAlarm(static_cast<HAL_NotifierHandle>(notifier_handle), &status);
  if (status == robosim::backend::shim::kHalSuccess) {
    robosim::backend::shim::flush_attached_notifier_state(0);
  }
}

jlong Java_edu_wpi_first_hal_NotifierJNI_waitForNotifierAlarm(JNIEnv*,
                                                              jclass,
                                                              jint notifier_handle) {
  robosim::backend::shim::pump_jni_fallback_notifier_alarm(
      static_cast<std::int32_t>(notifier_handle));
  std::int32_t status = 0;
  return static_cast<jlong>(
      HAL_WaitForNotifierAlarm(static_cast<HAL_NotifierHandle>(notifier_handle), &status));
}

jint Java_edu_wpi_first_hal_HALUtil_getHALRuntimeType(JNIEnv*, jclass) {
  return static_cast<jint>(HAL_GetRuntimeType());
}

jlong Java_edu_wpi_first_hal_HALUtil_getFPGATime(JNIEnv*, jclass) {
  auto* fallback = robosim::backend::shim::hal_jni_launch_state_shim_for_test();
  if (fallback != nullptr && robosim::backend::shim::hal_jni_launch_state_has_fallback_for_test() &&
      robosim::backend::shim::shim_core::current() == fallback &&
      !fallback->latest_clock_state().has_value()) {
    robosim::backend::shim::pump_jni_fallback_clock_state(
        robosim::backend::shim::kJniFallbackInitialClockSimTimeUs);
  }
  std::int32_t status = 0;
  return static_cast<jlong>(HAL_GetFPGATime(&status));
}
}
