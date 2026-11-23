// state_estimator の静止入力健全性テスト。
#include <math.h>

#include "common/control_params/control_params.hpp"
#include "feature/state_estimator/state_estimator.hpp"
#include "test_util.hpp"

namespace bb {
namespace {

imu_sample still_sample() {
  imu_sample s{};
  // 静止時、加速度計は +g の比力を測る（Z 軸上向き取り付け前提）
  s.specific_force = {0.0F, 0.0F, 9.80665F};
  s.gyro = vec3{};
  s.valid = true;
  return s;
}

void test_still_stays_valid_and_bounded() {
  const auto& p = get_params();
  state_estimator::init({&p});
  state_estimator::output out{};
  for (int i = 0; i < 400; ++i) {
    out =
        state_estimator::step(still_sample(), vec3{}, p.estimator.sample_time);
    CHECK_FINITE(out.euler[0]);
    CHECK_FINITE(out.euler[1]);
    CHECK_FINITE(out.euler[2]);
  }
  CHECK_TRUE(out.valid);
  // 静止入力で姿勢が暴走しない（小さい値に収束）
  const float tilt =
      sqrtf(out.euler[0] * out.euler[0] + out.euler[1] * out.euler[1]);
  CHECK_TRUE(tilt < 0.2F);
  // 速度・輪速は有限で暴走しない
  for (int i = 0; i < 3; ++i) {
    CHECK_FINITE(out.velocity_world[i]);
    CHECK_FINITE(out.ball_rate_world[i]);
  }
}

void test_quaternion_stays_normalized() {
  const auto& p = get_params();
  state_estimator::init({&p});
  // 回転しているようなジャイロ入力でもクォータニオンは正規化される
  imu_sample s = still_sample();
  s.gyro = {0.1F, -0.05F, 0.2F};
  state_estimator::output out{};
  for (int i = 0; i < 400; ++i) {
    out = state_estimator::step(s, vec3{}, p.estimator.sample_time);
  }
  const float n = sqrtf(out.quaternion[0] * out.quaternion[0] +
                        out.quaternion[1] * out.quaternion[1] +
                        out.quaternion[2] * out.quaternion[2] +
                        out.quaternion[3] * out.quaternion[3]);
  CHECK_NEAR(n, 1.0F, 0.01F);
}

// 床置き（大傾斜）起動: identity 初期姿勢だと残差ゲートで加速度更新が
// 永久拒否され、ジャイロバイアスで傾斜が積算され続ける。加速度シードで
// 初期姿勢が実姿勢に合い、バイアス学習が始まることを確認する。
void test_tilted_start_seeds_attitude() {
  const auto& p = get_params();
  state_estimator::init({&p});
  // ロール +60° で静置: 比力 = R^T {0,0,g} = g(0, sin60, cos60)
  imu_sample s{};
  const float g = p.physical.gravity;
  s.specific_force = {0.0F, 0.8660254F * g, 0.5F * g};
  s.gyro = {0.017F, 0.0F, 0.0F};  // ~1°/s の未学習ロールバイアス
  s.valid = true;
  state_estimator::output out{};
  for (int i = 0; i < 1200; ++i) {  // 6 s
    out = state_estimator::step(s, vec3{}, p.estimator.sample_time);
  }
  // 60° ≈ 1.047 rad 付近に収束し、ドリフトで逃走しない
  CHECK_NEAR(out.euler[0], 1.0472F, 0.15F);
  CHECK_TRUE(fabsf(out.euler[1]) < 0.1F);
}

// 固着復帰: 水平で初期化後に床へ倒す（加速度が一貫して傾斜方向）。
// 残差ゲートを超えた拒否が続いても、静止証明後は加速度で再シードされ
// 姿勢が実値へ戻ることを確認する。
void test_stuck_filter_reseeds_from_accel() {
  const auto& p = get_params();
  state_estimator::init({&p});
  const float g = p.physical.gravity;
  imu_sample level{};
  level.specific_force = {0.0F, 0.0F, g};
  level.valid = true;
  for (int i = 0; i < 200; ++i) {  // 1 s 水平で収束
    (void)state_estimator::step(level, vec3{}, p.estimator.sample_time);
  }
  // ロール +60° で静置（ジャイロバイアス付き、quasi_static 外だが
  // inertial 条件内）
  imu_sample tilted = level;
  tilted.specific_force = {0.0F, 0.8660254F * g, 0.5F * g};
  tilted.gyro = {0.03F, 0.0F, 0.0F};
  state_estimator::output out{};
  for (int i = 0; i < 800; ++i) {  // 4 s（再シード待ち 2 s 込み）
    out = state_estimator::step(tilted, vec3{}, p.estimator.sample_time);
  }
  CHECK_NEAR(out.euler[0], 1.0472F, 0.15F);
  CHECK_TRUE(fabsf(out.euler[1]) < 0.1F);
}

void test_invalid_sample_marks_output() {
  const auto& p = get_params();
  state_estimator::init({&p});
  imu_sample s = still_sample();
  s.valid = false;
  const auto out = state_estimator::step(s, vec3{}, p.estimator.sample_time);
  CHECK_TRUE(!out.valid);
}

}  // namespace

void state_estimator_tests() {
  test_still_stays_valid_and_bounded();
  test_quaternion_stays_normalized();
  test_tilted_start_seeds_attitude();
  test_stuck_filter_reseeds_from_accel();
  test_invalid_sample_marks_output();
}

}  // namespace bb
