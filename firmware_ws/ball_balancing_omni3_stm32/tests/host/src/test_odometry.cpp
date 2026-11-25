// odometry ホストテスト: 積分・原点オフセット・リセット。
#include "feature/odometry/odometry.hpp"
#include "test_util.hpp"

namespace bb {
namespace {

void integrates_velocity() {
  odometry::init();
  for (int i = 0; i < 10; ++i) {
    odometry::update(0.1F, -0.2F, 0.005F);  // 50 ms 分
  }
  const odometry::pose p = odometry::raw();
  CHECK_NEAR(p.x, 0.005F, 1e-6F);
  CHECK_NEAR(p.y, -0.010F, 1e-6F);
}

void origin_offsets() {
  odometry::init();
  odometry::update(1.0F, 1.0F, 0.1F);  // raw=(0.1,0.1)
  odometry::reset_origin(0.1F, 0.1F, 0.5F);
  odometry::pose p = odometry::get();
  CHECK_NEAR(p.x, 0.0F, 1e-6F);
  CHECK_NEAR(p.y, 0.0F, 1e-6F);
  CHECK_NEAR(odometry::yaw_of(0.7F), 0.2F, 1e-6F);
  // リセット後の積分はオフセット後座標で反映される。
  odometry::update(0.0F, 1.0F, 0.1F);
  p = odometry::get();
  CHECK_NEAR(p.y, 0.1F, 1e-6F);
}

void init_resets() {
  odometry::init();
  odometry::update(1.0F, 0.0F, 1.0F);
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
    odometry::update(0.5F, -0.3F, 0.005F);
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
  odometry::update(1.0F, 1.0F, 0.1F);  // raw=(0.1,0.1)
  odometry::reset_origin(0.1F, 0.1F, 0.2F);
  odometry::update(0.5F, 0.0F, 0.1F);  // raw=(0.15,0.1)
  odometry::reapply_declared(0.15F, 0.1F, 0.4F);
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
  odometry::update(0.0F, 0.0F, 0.005F);  // 動作中パス
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
}

}  // namespace bb
