#pragma once

namespace bb {

// ワールド速度の積分による 2 次元位置推定と原点オフセット管理
// （FR-10, FR-11、static class）。純粋計算のみで I/O を持たない。
// 呼出し側（外側タスク・リンクスレッド）が排他を担う。
class odometry {
 public:
  odometry() = delete;

  struct pose {
    float x;
    float y;
  };

  static void init();
  // 推定器の velocity_world2 を dt で前進 Euler 積分する（外側タスク呼出し）。
  static void update(float vx_w, float vy_w, float dt);

  // オフセット適用前の生積分位置。
  static pose raw();
  // 原点オフセット適用後の現在ワールド位置。
  static pose get();
  // raw_yaw - yaw_offset の現在ワールド yaw。
  static float yaw_of(float raw_yaw);
  // 現在の生位置・生 yaw を新しいワールド原点とする（RESET_ODOMETRY）。
  // 外側タスクと同時実行され得るため呼出し側が排他する。
  static void reset_origin(float raw_x, float raw_y, float raw_yaw);
  // 現在の get()/yaw_of が (x, y, yaw) を返すようオフセットを設定する
  // （SET_POSE、FR-13）。呼出し側が排他する。
  static void set_pose(float x, float y, float yaw, float raw_x, float raw_y,
                       float raw_yaw);

 private:
  static inline float raw_x_ = 0.0F;
  static inline float raw_y_ = 0.0F;
  static inline float origin_x_ = 0.0F;
  static inline float origin_y_ = 0.0F;
  static inline float yaw_offset_ = 0.0F;
};

}  // namespace bb
