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

// 初期位置確定（SET_POSE）由来の推定リセット: 移動中に溜まった動的状態
// をクリアし、学習済みバイアスを保持したまま姿勢が再シードされることを
// 確認する。
void test_placement_reset_reseeds_and_keeps_bias() {
  const auto& p = get_params();
  state_estimator::init({&p});
  // 水平加速度を与えて速度を持たせる（搬送中を模擬）
  imu_sample mv = still_sample();
  mv.specific_force = {1.0F, 0.0F, 9.8F};
  for (int i = 0; i < 400; ++i) {
    (void)state_estimator::step(mv, vec3{}, p.estimator.sample_time);
  }
  state_estimator::set_bias(0.05F, 0.01F, -0.02F);
  state_estimator::request_reset();
  state_estimator::output out = state_estimator::step(
      still_sample(), vec3{}, p.estimator.sample_time);
  // リセット消費直後: バイアスは保持（ゼロクリアされない）
  float bz = 0.0F, bx = 0.0F, by = 0.0F;
  state_estimator::get_bias(&bz, &bx, &by);
  CHECK_NEAR(bz, 0.05F, 1e-3F);
  CHECK_NEAR(bx, 0.01F, 1e-3F);
  CHECK_NEAR(by, -0.02F, 1e-3F);
  // 速度は初期化され、以後の静止サンプルで姿勢が再シードされる
  for (int i = 0; i < 400; ++i) {
    out = state_estimator::step(still_sample(), vec3{},
                                p.estimator.sample_time);
  }
  const float tilt =
      sqrtf(out.euler[0] * out.euler[0] + out.euler[1] * out.euler[1]);
  CHECK_TRUE(tilt < 0.1F);
  CHECK_TRUE(fabsf(out.velocity_world[0]) < 0.05F);
}

// 裏返し起動: 比力が -z（チップの測る上向きが機体 -Z 側）の場合も
// 姿勢が 180° へシードされることを確認する（tilt=0 にならない回帰）。
void test_inverted_start_seeds_180deg() {
  const auto& p = get_params();
  state_estimator::init({&p});
  imu_sample s = still_sample();
  s.specific_force = {0.0F, 0.0F, -9.80665F};
  state_estimator::output out{};
  for (int i = 0; i < 600; ++i) {
    out = state_estimator::step(s, vec3{}, p.estimator.sample_time);
  }
  const float tilt =
      sqrtf(out.euler[0] * out.euler[0] + out.euler[1] * out.euler[1]);
  CHECK_NEAR(tilt, 3.14159F, 0.1F);
}

void test_invalid_sample_marks_output() {
  const auto& p = get_params();
  state_estimator::init({&p});
  imu_sample s = still_sample();
  s.valid = false;
  const auto out = state_estimator::step(s, vec3{}, p.estimator.sample_time);
  CHECK_TRUE(!out.valid);
}

// 動作開始時の推定リセット（FR-12）: 蓄積した yaw を保持したまま動的
// 状態をリセットし、keep_qualification で資格時間も保持されることを
// 確認する。yaw が跳ぶとオドメトリの yaw オフセットと不整合になる。
void test_start_reset_keeps_yaw_and_qualification() {
  const auto& p = get_params();
  state_estimator::init({&p});
  // まず静止で資格を成立させる（START 受理条件と同じ）。
  state_estimator::output out{};
  for (int i = 0; i < 400; ++i) {
    out =
        state_estimator::step(still_sample(), vec3{}, p.estimator.sample_time);
  }
  CHECK_TRUE(out.updated_qualification_time >=
             p.estimator.bias_qualification_time);
  // 搬送中: yaw 回転しつつ水平加速度で速度も蓄積する。
  // 残差が residual_gate を超えるため EKF 更新は走らず、バイアスが
  // 汚染されない（この期間の資格は失格側へ落ちる）。
  imu_sample mv = still_sample();
  mv.specific_force = {3.0F, 0.0F, 9.8F};
  mv.gyro = {0.0F, 0.0F, 0.5F};
  for (int i = 0; i < 200; ++i) {  // 1 s
    (void)state_estimator::step(mv, vec3{}, p.estimator.sample_time);
  }
  // 設置して再び静止させ、資格を再成立させる。
  for (int i = 0; i < 400; ++i) {
    out =
        state_estimator::step(still_sample(), vec3{}, p.estimator.sample_time);
  }
  CHECK_TRUE(out.updated_qualification_time >=
             p.estimator.bias_qualification_time);
  const float yaw_at_start = out.euler[2];
  CHECK_TRUE(fabsf(yaw_at_start) > 0.3F);  // ドリフトした yaw がある前提

  // 動作開始リセット（資格保持）。yaw は維持され、速度・資格の扱いが
  // 仕様どおりであることを確認する。
  state_estimator::reset_estimate(true);
  out = state_estimator::step(still_sample(), vec3{}, p.estimator.sample_time);
  CHECK_NEAR(out.euler[2], yaw_at_start, 0.05F);  // yaw は跳ばない
  CHECK_TRUE(fabsf(out.velocity_world[0]) < 0.1F);
  CHECK_TRUE(out.updated_qualification_time >=
             p.estimator.bias_qualification_time);

  // 既定（資格非保持）では資格が 0 に戻り再証明が必要。
  state_estimator::reset_estimate();
  out = state_estimator::step(still_sample(), vec3{}, p.estimator.sample_time);
  CHECK_TRUE(out.updated_qualification_time <
             p.estimator.bias_qualification_time);
}

// CALIBRATE_IMU（reset_bias）後の静止ウィンドウで、生ジャイロの時間
// 平均が gyro_bias_z / ekf_bias_xy の確定値として採用されることを確認
// する（FR-13: 単一サンプルではなく区間平均。失格サンプルで取り直し）。
void test_calib_capture_adopts_window_mean() {
  const auto& p = get_params();
  state_estimator::init({&p});
  imu_sample s = still_sample();
  s.gyro = {0.010F, -0.008F, 0.012F};  // 静止 + 未学習バイアス
  state_estimator::reset_bias();

  const int window = static_cast<int>(p.estimator.bias_qualification_time /
                                      p.estimator.sample_time);
  // ウィンドウ途中ではまだ採用されない（区間平均が未完成）
  for (int i = 0; i < window / 2; ++i) {
    (void)state_estimator::step(s, vec3{}, p.estimator.sample_time);
  }
  float bz = 0.0F, bx = 0.0F, by = 0.0F;
  state_estimator::get_bias(&bz, &bx, &by);
  CHECK_NEAR(bz, 0.0F, 0.005F);

  // 失格サンプル（yaw 閾値超過）でウィンドウを取り直す
  imu_sample bad = still_sample();
  bad.gyro = {0.0F, 0.0F, 0.5F};
  (void)state_estimator::step(bad, vec3{}, p.estimator.sample_time);

  for (int i = 0; i < window; ++i) {
    (void)state_estimator::step(s, vec3{}, p.estimator.sample_time);
  }
  state_estimator::get_bias(&bz, &bx, &by);
  CHECK_NEAR(bz, 0.012F, 0.002F);
  CHECK_NEAR(bx, 0.010F, 0.002F);
  CHECK_NEAR(by, -0.008F, 0.002F);
}

// 推定速度の物理包絡: 持続した水平加速（搬送中・幻影加速度相当）で
// velocity_world が理論上限に飽和し、leak 減衰を待たず暴走しない
// ことを確認する。
void test_velocity_world_speed_capped() {
  const auto& p = get_params();
  state_estimator::init({&p});
  // 水平加速度 + yaw ジャイロ。ジャイロで assembly_still を外して
  // 静止ドレインを無効化し、輪速 0 でアンカーも効かない条件にする
  // （純粋に加速度積分が伸びる）。
  imu_sample mv = still_sample();
  mv.specific_force = {3.0F, 0.0F, 9.8F};
  mv.gyro = {0.0F, 0.0F, 0.5F};
  state_estimator::output out{};
  for (int i = 0; i < 600; ++i) {  // 3 s: 包絡が無ければ ~9 m/s 級
    out = state_estimator::step(mv, vec3{}, p.estimator.sample_time);
  }
  const float v = hypotf(out.velocity_world[0], out.velocity_world[1]);
  CHECK_TRUE(v <= 1.5F + 1e-6F);  // K_VELOCITY_WORLD_LIMIT
  CHECK_TRUE(v > 1.0F);           // 包絡へ達している（飽和の確認）
  CHECK_TRUE(fabsf(out.velocity_world2[0]) <= 1.5F + 1e-6F);
  CHECK_TRUE(fabsf(out.velocity_world2[1]) <= 1.5F + 1e-6F);
}

}  // namespace

void state_estimator_tests() {
  test_still_stays_valid_and_bounded();
  test_quaternion_stays_normalized();
  test_tilted_start_seeds_attitude();
  test_stuck_filter_reseeds_from_accel();
  test_placement_reset_reseeds_and_keeps_bias();
  test_inverted_start_seeds_180deg();
  test_invalid_sample_marks_output();
  test_start_reset_keeps_yaw_and_qualification();
  test_calib_capture_adopts_window_mean();
  test_velocity_world_speed_capped();
}

}  // namespace bb
