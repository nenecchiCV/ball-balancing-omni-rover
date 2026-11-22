#include "manager/robot_application/robot_application.hpp"

// ハードウェア初期化（クロック・ピンmux・ペリフェラル）は
// devicetree + Zephyr ドライバの SYS 初期化で完了している。
// ここでは各 static class を初期化して制御ループを開始するだけ。

int main() {
  // 実機で確認した IMU 実装向き（hardware_config.md 参照）:
  // 加速度は機体軸に対して全軸反転。角速度はセンサ軸まわりで左ネジ
  // 正転のため、軸反転後の機体座標では無変換で右ネジ正転になる。
  const bb::robot_application::config cfg = {
      &bb::get_params(),
      {{{-1.0F, 0.0F, 0.0F}, {0.0F, -1.0F, 0.0F}, {0.0F, 0.0F, -1.0F}}},
      bb::mat_identity3()};
  // IMU 未検出でも制御ループは回し続ける。
  // 推定器が valid=false を返し続け、制御器は mode 0（停止）へ向かう。
  bb::robot_application::init(cfg);
  bb::robot_application::run();
}
