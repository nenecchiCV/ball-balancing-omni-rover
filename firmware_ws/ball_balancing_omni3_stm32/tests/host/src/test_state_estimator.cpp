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
  test_invalid_sample_marks_output();
}

}  // namespace bb
