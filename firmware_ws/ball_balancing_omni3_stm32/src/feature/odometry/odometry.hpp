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
  // 推定器の velocity_world2 を dt で前進 Euler 積分する（外側タスク呼出し、
  // 動作中用）。待機中慣性チャネルの速度をリセットし、再突入時に
  // 張り残した速度を持ち込まない。
  static void update(float vx_w, float vy_w, float dt);
  // 待機中専用の IMU 慣性オドメトリ（FR-11、検証用途）。ワールド水平
  // 加速度を 2 重積分して位置を進め、静止候補の継続（ZUPT）で速度を
  // ゼロへ張り付ける。確定した静止の間は水平加速度残差をバイアスと
  // して緩学習し、非静止中の積分入力から差し引く。等速運動は原理上
  // 検知できず、搬送中の精度は参考値（動作開始時に宣言位置へ再
  // アンカーされるため推定誤差は動作へ持ち越さない）。
  static void update_inertial(float ax_w, float ay_w, bool still, float dt);
  // 待機中慣性チャネルの現在速度 {vx, vy}（EVT_POSE 表示用）。
  static pose inertial_velocity();

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
  // （SET_POSE、FR-13）。指定値は宣言位置として記憶し、
  // reapply_declared で再適用できる。呼出し側が排他する。
  static void set_pose(float x, float y, float yaw, float raw_x, float raw_y,
                       float raw_yaw);
  // 最後に宣言した位置・向き（set_pose の指定値、reset_origin なら原点）
  // を現在の生位置・生 yaw へ再適用する（START_OPERATION 受理時、FR-12）。
  // 待機中に蓄積した生積分ドリフトを動作開始へ持込まないための再アンカー。
  static void reapply_declared(float raw_x, float raw_y, float raw_yaw);

 private:
  static inline float raw_x_ = 0.0F;
  static inline float raw_y_ = 0.0F;
  static inline float origin_x_ = 0.0F;
  static inline float origin_y_ = 0.0F;
  static inline float yaw_offset_ = 0.0F;
  // 宣言位置（set_pose の指定値。reset_origin/初期値は原点）
  static inline float decl_x_ = 0.0F;
  static inline float decl_y_ = 0.0F;
  static inline float decl_yaw_ = 0.0F;
  // 待機中慣性オドメトリ状態（update_inertial で更新）。
  static inline float inertial_vx_ = 0.0F;
  static inline float inertial_vy_ = 0.0F;
  static inline float accel_bias_x_ = 0.0F;
  static inline float accel_bias_y_ = 0.0F;
  static inline float still_time_ = 0.0F;
};

}  // namespace bb
