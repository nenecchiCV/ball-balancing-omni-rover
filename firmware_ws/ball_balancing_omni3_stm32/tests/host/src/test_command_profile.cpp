// command_profile の台形整形・サイクル位相テスト。
#include <math.h>

#include "common/control_params/control_params.hpp"
#include "feature/command_profile/command_profile.hpp"
#include "test_util.hpp"

namespace bb {
namespace {

void test_zero_before_cycle_start() {
  const auto& p = get_params();
  command_profile::init({&p});
  // 初回呼び出しは内部 dt = now - 0 に制限されるため指令は微小に留まる。
  // （cycle_start 前でも elapsed が 0 にクランプされて「サイクル先頭」
  // 扱いになるのは MATLAB 由来の既知挙動）
  const auto out = command_profile::update(0.0F);
  CHECK_TRUE(out.enable == p.command.enable);
  CHECK_NEAR(out.command[0], 0.0F, 1e-3F);
  CHECK_NEAR(out.command[1], 0.0F, 1e-6F);
  CHECK_NEAR(out.command[2], 0.0F, 1e-6F);
}

void test_acceleration_limit_respected() {
  // サイクル走行は既定 OFF のため、検査用にローカルコピーで有効化する。
  param_set p = get_params();
  p.command.cycles_enabled = true;
  command_profile::init({&p});
  // t=0 から 5ms 周期で連続更新し、移動フェーズの変化率を検査する。
  // （初回呼び出し以外は内部 dt = 実時間差になる）
  const float dt = 0.005F;
  float prev = 0.0F;
  for (int i = 0; i <= 1000; ++i) {
    const float t = dt * i;
    const auto out = command_profile::update(t);
    CHECK_FINITE(out.command[0]);
    const float dv = fabsf(out.command[0] - prev);
    // 1 ステップの変化は加速リミット * dt 以下（丸め誤差余裕あり）
    CHECK_TRUE(dv <= p.command.acceleration_limit * dt + 1e-5F);
    prev = out.command[0];
    // 目標速度を超えて発散しない
    CHECK_TRUE(out.command[0] <= p.command.profile_velocity_world[0] + 1e-4F);
  }
  // 加速後は目標速度に到達している
  CHECK_NEAR(prev, p.command.profile_velocity_world[0], 0.01F);
}

void test_stop_phase_decelerates() {
  param_set p = get_params();
  p.command.cycles_enabled = true;
  command_profile::init({&p});
  const float dt = 0.005F;
  const float stop_begin =
      p.command.cycle_start + p.command.cycle_move_duration + 0.1F;
  // 移動フェーズで速度を上げてから停止フェーズへ
  for (float t = 0.0F; t < stop_begin; t += dt)
    (void)command_profile::update(t);
  float prev = command_profile::update(stop_begin).command[0];
  for (int i = 0; i < 200; ++i) {
    const auto out = command_profile::update(stop_begin + dt * (i + 1));
    // 減速リミット以下で単調減少
    CHECK_TRUE(out.command[0] <= prev + 1e-6F);
    const float dv = fabsf(out.command[0] - prev);
    CHECK_TRUE(dv <= p.command.deceleration_limit * dt + 1e-5F);
    prev = out.command[0];
  }
}

}  // namespace

void command_profile_tests() {
  test_zero_before_cycle_start();
  test_acceleration_limit_respected();
  test_stop_phase_decelerates();
}

}  // namespace bb
