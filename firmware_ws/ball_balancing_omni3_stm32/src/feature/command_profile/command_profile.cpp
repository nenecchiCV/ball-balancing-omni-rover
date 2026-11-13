#include "feature/command_profile/command_profile.hpp"

#include <etl/algorithm.h>
#include <math.h>

namespace bb {

void command_profile::init(const config& cfg) {
  p_ = cfg.params;
  prev_velocity_ = vec2{};
  prev_time_ = 0.0F;
}

command_profile::output command_profile::update(float now_sec) {
  const auto& cmd = p_->command;
  output out{};
  out.enable = cmd.enable;
  vec2 target = cmd.velocity_world;
  float yaw_rate = cmd.yaw_rate;

  if (cmd.cycles_enabled && cmd.cycle_count > 0.0F) {
    // 1 サイクル = move → stop（→必要に応じて yaw パルス）
    const float cycle_period = cmd.cycle_move_duration +
                               cmd.cycle_stop_duration + cmd.cycle_yaw_duration;
    const float elapsed = etl::max(0.0F, now_sec - cmd.cycle_start);
    const float total_cycles =
        etl::min(cmd.cycle_count, 1.0F + floorf(elapsed / cycle_period));
    const float t = fmodf(elapsed, cycle_period);
    const bool in_cycle =
        elapsed >= 0.0F && (elapsed / cycle_period) < cmd.cycle_count;
    if (in_cycle) {
      const float move_end = cmd.cycle_move_duration;
      const float stop_end = move_end + cmd.cycle_stop_duration;
      if (t < move_end) {
        target = cmd.profile_velocity_world;
      } else if (t < stop_end) {
        target = vec2{};
      } else {
        target = vec2{};
        // cycle_yaw_every 回目のサイクルでのみヨーパルス
        const float cycle_index = floorf(elapsed / cycle_period) + 1.0F;
        if (cmd.cycle_yaw_every > 0.0F &&
            fmodf(cycle_index, cmd.cycle_yaw_every) == 0.0F &&
            t >= stop_end + cmd.cycle_yaw_delay &&
            t < stop_end + cmd.cycle_yaw_delay + cmd.cycle_yaw_duration) {
          yaw_rate = cmd.profile_yaw_rate;
        }
      }
    }
    (void)total_cycles;
  } else if (cmd.profile_enabled) {
    if (now_sec >= cmd.profile_start && now_sec < cmd.profile_stop) {
      target = cmd.profile_velocity_world;
      yaw_rate =
          cmd.yaw_pulse_duration > 0.0F ? cmd.profile_yaw_rate : yaw_rate;
    }
  }

  // 台形整形: 加速度/減速度リミット
  const float dt = etl::max(1e-4F, now_sec - prev_time_);
  prev_time_ = now_sec;
  for (int i = 0; i < 2; ++i) {
    const float diff = target[i] - prev_velocity_[i];
    const float limit =
        (fabsf(diff) > 0.0F && fabsf(target[i]) < fabsf(prev_velocity_[i]))
            ? cmd.deceleration_limit
            : cmd.acceleration_limit;
    const float step = etl::clamp(diff, -limit * dt, limit * dt);
    prev_velocity_[i] += step;
  }
  out.command = {prev_velocity_[0], prev_velocity_[1], yaw_rate};
  return out;
}

}  // namespace bb
