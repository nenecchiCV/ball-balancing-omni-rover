#include "feature/odometry/odometry.hpp"

namespace bb {

void odometry::init() {
  raw_x_ = 0.0F;
  raw_y_ = 0.0F;
  origin_x_ = 0.0F;
  origin_y_ = 0.0F;
  yaw_offset_ = 0.0F;
  decl_x_ = 0.0F;
  decl_y_ = 0.0F;
  decl_yaw_ = 0.0F;
}

void odometry::update(float vx_w, float vy_w, float dt) {
  raw_x_ += vx_w * dt;
  raw_y_ += vy_w * dt;
}

odometry::pose odometry::raw() {
  return {raw_x_, raw_y_};
}

odometry::pose odometry::get() {
  return {raw_x_ - origin_x_, raw_y_ - origin_y_};
}

float odometry::yaw_of(float raw_yaw) {
  return raw_yaw - yaw_offset_;
}

void odometry::reset_origin(float raw_x, float raw_y, float raw_yaw) {
  origin_x_ = raw_x;
  origin_y_ = raw_y;
  yaw_offset_ = raw_yaw;
  // 原点宣言は (0,0,0) の宣言位置とみなす。
  decl_x_ = 0.0F;
  decl_y_ = 0.0F;
  decl_yaw_ = 0.0F;
}

void odometry::set_pose(float x, float y, float yaw, float raw_x, float raw_y,
                        float raw_yaw) {
  origin_x_ = raw_x - x;
  origin_y_ = raw_y - y;
  yaw_offset_ = raw_yaw - yaw;
  decl_x_ = x;
  decl_y_ = y;
  decl_yaw_ = yaw;
}

void odometry::reapply_declared(float raw_x, float raw_y, float raw_yaw) {
  set_pose(decl_x_, decl_y_, decl_yaw_, raw_x, raw_y, raw_yaw);
}

}  // namespace bb
