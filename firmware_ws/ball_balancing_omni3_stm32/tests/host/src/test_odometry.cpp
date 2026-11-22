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

}  // namespace

void odometry_tests() {
  integrates_velocity();
  origin_offsets();
  init_resets();
}

}  // namespace bb
