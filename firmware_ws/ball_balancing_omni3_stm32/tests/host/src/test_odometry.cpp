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

}  // namespace

void odometry_tests() {
  integrates_velocity();
  origin_offsets();
  init_resets();
  declared_pose_reapplied();
  reset_origin_declares_zero();
}

}  // namespace bb
