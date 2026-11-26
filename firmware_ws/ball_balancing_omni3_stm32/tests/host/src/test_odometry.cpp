// odometry ホストテスト: 積分・原点オフセット・リセット。
#include "feature/odometry/odometry.hpp"
#include "test_util.hpp"

namespace bb {
namespace {

void integrates_velocity() {
  odometry::init();
  for (int i = 0; i < 10; ++i) {
    odometry::update(0.1F, -0.2F, false, 0.005F);  // 50 ms 分
  }
  const odometry::pose p = odometry::raw();
  CHECK_NEAR(p.x, 0.005F, 1e-6F);
  CHECK_NEAR(p.y, -0.010F, 1e-6F);
}

void origin_offsets() {
  odometry::init();
  odometry::update(0.5F, 0.5F, false, 0.1F);  // raw=(0.05,0.05)
  odometry::reset_origin(0.05F, 0.05F, 0.5F);
  odometry::pose p = odometry::get();
  CHECK_NEAR(p.x, 0.0F, 1e-6F);
  CHECK_NEAR(p.y, 0.0F, 1e-6F);
  CHECK_NEAR(odometry::yaw_of(0.7F), 0.2F, 1e-6F);
  // リセット後の積分はオフセット後座標で反映される。
  odometry::update(0.0F, 0.5F, false, 0.1F);
  p = odometry::get();
  CHECK_NEAR(p.y, 0.05F, 1e-6F);
}

void init_resets() {
  odometry::init();
  odometry::update(1.0F, 0.0F, false, 1.0F);
  odometry::reset_origin(1.0F, 0.0F, 0.0F);
  odometry::init();
  const odometry::pose p = odometry::get();
  CHECK_NEAR(p.x, 0.0F, 1e-6F);
  CHECK_NEAR(p.y, 0.0F, 1e-6F);
}

// START_OPERATION 受理時の宣言位置再適用（FR-12）: 待機中に生積分が
// ドリフト（手で動かした等）しても、開始時に宣言位置へ再アンカーされる。
void declared_pose_reapplied() {
  odometry::init();
  odometry::set_pose(1.0F, 2.0F, 0.5F, 10.0F, 20.0F, 0.3F);
  for (int i = 0; i < 100; ++i) {  // 500 ms 分のドリフト
    odometry::update(0.5F, -0.3F, false, 0.005F);
  }
  const odometry::pose raw = odometry::raw();
  odometry::reapply_declared(raw.x, raw.y, 0.7F);
  const odometry::pose p = odometry::get();
  CHECK_NEAR(p.x, 1.0F, 1e-4F);
  CHECK_NEAR(p.y, 2.0F, 1e-4F);
  CHECK_NEAR(odometry::yaw_of(0.7F), 0.5F, 1e-6F);
}

// RESET_ODOMETRY は (0,0,0) の宣言位置とみなされ、以後の再適用でも
// 原点が維持される。
void reset_origin_declares_zero() {
  odometry::init();
  odometry::set_pose(5.0F, 6.0F, 1.0F, 0.0F, 0.0F, 0.0F);
  odometry::update(0.5F, 0.5F, false, 0.1F);  // raw=(0.05,0.05)
  odometry::reset_origin(0.05F, 0.05F, 0.2F);
  odometry::update(0.5F, 0.0F, false, 0.1F);  // raw=(0.10,0.05)
  odometry::reapply_declared(0.10F, 0.05F, 0.4F);
  const odometry::pose p = odometry::get();
  CHECK_NEAR(p.x, 0.0F, 1e-6F);
  CHECK_NEAR(p.y, 0.0F, 1e-6F);
  CHECK_NEAR(odometry::yaw_of(0.4F), 0.0F, 1e-6F);
}

// 待機中慣性オドメトリ（FR-11）: 確定静止で速度が張り付き（ZUPT）、
// 加速度バーストで位置が進み、再静止で固定される。
void inertial_zupt_and_motion() {
  odometry::init();
  const float dt = 0.005F;
  // 静止残差 0.3 m/s² で 500 ms 静止 → ZUPT で速度はゼロへ張り付く
  for (int i = 0; i < 100; ++i) {
    odometry::update_inertial(0.3F, 0.0F, true, dt);
  }
  CHECK_NEAR(odometry::inertial_velocity().x, 0.0F, 1e-6F);
  const odometry::pose p0 = odometry::raw();
  CHECK_NEAR(p0.x, 0.0F, 0.02F);  // ZUPT 確定前の微少積分のみ

  // 200 ms の加速度バーストで位置が前進し速度が残る
  for (int i = 0; i < 40; ++i) {
    odometry::update_inertial(0.5F, 0.0F, false, dt);
  }
  const odometry::pose p1 = odometry::raw();
  CHECK_TRUE(odometry::inertial_velocity().x > 0.05F);
  CHECK_TRUE(p1.x - p0.x > 0.005F);

  // 再び確定静止すると速度はゼロへ戻り、以後位置は完全に固定される
  for (int i = 0; i < 100; ++i) {
    odometry::update_inertial(0.3F, 0.0F, true, dt);
  }
  CHECK_NEAR(odometry::inertial_velocity().x, 0.0F, 1e-6F);
  const odometry::pose p2 = odometry::raw();
  for (int i = 0; i < 40; ++i) {
    odometry::update_inertial(0.3F, 0.0F, true, dt);
  }
  CHECK_NEAR(odometry::raw().x, p2.x, 1e-6F);
}

// 動作中パス（update）は待機慣性速度を無効化し、待機再突入時に
// 張り残した速度を持ち込まない。
void inertial_velocity_cleared_by_update() {
  odometry::init();
  for (int i = 0; i < 40; ++i) {
    odometry::update_inertial(0.5F, 0.0F, false, 0.005F);
  }
  CHECK_TRUE(odometry::inertial_velocity().x > 0.05F);
  odometry::update(0.0F, 0.0F, false, 0.005F);  // 動作中パス
  CHECK_NEAR(odometry::inertial_velocity().x, 0.0F, 1e-6F);
}

// 積分速度包絡: 物理上限 (1.0 m/s) を超える推定速度は同方向のまま
// クランプされ、幻影速度（姿勢残差の重力混入・輪速高値化）の積分率が
// 上限で飽和する。
void velocity_clamped_to_limit() {
  odometry::init();
  // ノルム 5 m/s 入力 → 1 m/s へクランプ（方向保持で 3:4 = 0.6:0.8）
  odometry::update(3.0F, 4.0F, false, 0.1F);
  const odometry::pose p = odometry::raw();
  CHECK_NEAR(p.x, 0.06F, 1e-6F);
  CHECK_NEAR(p.y, 0.08F, 1e-6F);
  // 上限ちょうどの入力はクランプしない
  odometry::init();
  odometry::update(1.0F, 0.0F, false, 0.1F);
  CHECK_NEAR(odometry::raw().x, 0.1F, 1e-6F);
}

// settle 保持（上限時間）: arm 後 K_SETTLE_HOLD_TIME=1.0 s の間は
// 速度を積分せず宣言位置へ張り付き、期限後はクランプ込みで再開する。
// 解除判定したティック自体は積分しない（解放境界の ±1 tick を
// 吸収するため範囲で評価する）。
void settle_hold_discards_phantom() {
  odometry::init();
  odometry::set_pose(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F);
  odometry::arm_settle_hold();
  // 配置トランジェント相当の幻影速度 2 m/s: 保持中（0.9 s）は積分しない
  for (int i = 0; i < 180; ++i) {
    odometry::update(2.0F, 0.0F, false, 0.005F);
  }
  const odometry::pose p = odometry::get();
  CHECK_NEAR(p.x, 0.0F, 1e-6F);
  CHECK_NEAR(p.y, 0.0F, 1e-6F);
  // 上限（1.0 s）超過後は積分が再開する（2 m/s 入力は 1 m/s にクランプ）。
  for (int i = 0; i < 40; ++i) {  // さらに 0.2 s → ~0.1 s 分の積分
    odometry::update(2.0F, 0.0F, false, 0.005F);
  }
  CHECK_TRUE(odometry::get().x > 0.08F);
  CHECK_TRUE(odometry::get().x < 0.12F);
}

// settle 保持（早期解除）: still が K_SETTLE_STILL_TIME=0.2 s 連続
// すれば上限を待たず解除する。still が途切れると連続時間を積み直す。
void settle_hold_released_by_still() {
  odometry::init();
  odometry::arm_settle_hold();
  // still 連続 0.15 s（30 tick）では解除されない
  for (int i = 0; i < 30; ++i) {
    odometry::update(2.0F, 0.0F, true, 0.005F);
  }
  CHECK_NEAR(odometry::raw().x, 0.0F, 1e-6F);
  // still が途切れると連続時間がリセットされる
  odometry::update(2.0F, 0.0F, false, 0.005F);
  for (int i = 0; i < 30; ++i) {  // 積み直し 0.15 s < 0.2 s → まだ保持
    odometry::update(2.0F, 0.0F, true, 0.005F);
  }
  CHECK_NEAR(odometry::raw().x, 0.0F, 1e-6F);
  // さらに 0.1 s still（連続計 0.25 s > 0.2 s）で解除され、
  // 残りのティックは 1 m/s クランプで積分される（解放境界の
  // ±1 tick を吸収するため範囲で評価）。
  for (int i = 0; i < 20; ++i) {
    odometry::update(2.0F, 0.0F, true, 0.005F);
  }
  CHECK_TRUE(odometry::raw().x > 0.03F);
  CHECK_TRUE(odometry::raw().x < 0.07F);
}

// settle 保持は開始ごとに再武装される（OPERATING 再入サイクル）。
void settle_hold_rearmed_each_start() {
  odometry::init();
  odometry::arm_settle_hold();
  for (int i = 0; i < 220; ++i) {  // 保持 200 tick + 積分 ~20 tick
    odometry::update(1.0F, 0.0F, false, 0.005F);
  }
  CHECK_TRUE(odometry::raw().x > 0.08F);
  CHECK_TRUE(odometry::raw().x < 0.12F);
  odometry::arm_settle_hold();
  const odometry::pose r0 = odometry::raw();
  for (int i = 0; i < 100; ++i) {  // 0.5 s は再保持中
    odometry::update(1.0F, 0.0F, false, 0.005F);
  }
  CHECK_NEAR(odometry::raw().x, r0.x, 1e-6F);
}

// settle 保持中も update は動作中パスとして待機慣性速度を無効化する。
void settle_hold_clears_inertial() {
  odometry::init();
  for (int i = 0; i < 40; ++i) {
    odometry::update_inertial(0.5F, 0.0F, false, 0.005F);
  }
  CHECK_TRUE(odometry::inertial_velocity().x > 0.05F);
  odometry::arm_settle_hold();
  odometry::update(0.0F, 0.0F, false, 0.005F);
  CHECK_NEAR(odometry::inertial_velocity().x, 0.0F, 1e-6F);
}

}  // namespace

void odometry_tests() {
  integrates_velocity();
  origin_offsets();
  init_resets();
  declared_pose_reapplied();
  reset_origin_declares_zero();
  inertial_zupt_and_motion();
  inertial_velocity_cleared_by_update();
  velocity_clamped_to_limit();
  settle_hold_discards_phantom();
  settle_hold_released_by_still();
  settle_hold_rearmed_each_start();
  settle_hold_clears_inertial();
}

}  // namespace bb
