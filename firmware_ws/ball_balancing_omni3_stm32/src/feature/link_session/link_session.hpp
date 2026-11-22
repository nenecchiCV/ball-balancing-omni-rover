#pragma once

#include <zephyr/kernel.h>

#include <cstdint>

#include "common/control_params/control_params.hpp"
#include "common/link_protocol/link_protocol.hpp"
#include "common/vector_math/vector_math.hpp"

namespace bb {

// リンク制御モード（SET_CONTROL_MODE の mode 値）。
namespace link_mode {
constexpr uint8_t STANDBY = 0;
constexpr uint8_t VELOCITY = 1;
constexpr uint8_t POSITION = 2;
}  // namespace link_mode

// ロボットリンクのセッション管理（FR-09, FR-11、static class）。
// 受信フレームの振分け・モード/指令スロット・購読・安全ウォッチドッグ・
// イベント生成を担う。スレッドモデル:
//   リンクスレッド: run_once() を繰返す。受信排水・振分け・応答/イベント送出。
//   制御スレッド:   publish_state / command / generate_events のみ
//                  （整合コピーか K_NO_WAIT の msgq 投入でブロックしない）。
class link_session {
 public:
  link_session() = delete;

  struct config {
    const param_set* params;
  };

  // 制御スレッド → リンクスレッドの状態メールボックス。
  struct robot_state {
    bool enable;
    bool estop;
    bool imu_ok;
    bool qualified;               // bias qualification 完了（位置制御成立条件）
    float raw_x, raw_y, raw_yaw;  // RESET_ODOMETRY / STOP 用の生値
    float x, y, yaw;              // イベント用オフセット済 pose
    float vx, vy, yaw_rate;
  };

  struct command_output {
    uint8_t mode = link_mode::STANDBY;
    vec3 velocity_cmd{};       // mode=VELOCITY 時のワールド速度指令
    bool link_active = false;  // 一度でもリンク確立したか（false なら
                               // 呼出し側は command_profile を使う）
  };

  struct stats {
    uint32_t rx_frames;
    uint32_t rx_crc_drops;
    uint32_t rx_overruns;
    uint32_t tx_drops;
    uint32_t last_rx_ms;
  };

  static void init(const config& cfg);
  // リンクスレッドのループ本体。rx_sem/tx_msgq の k_poll で起床する。
  static void run_once();

  // 以下は制御スレッドから呼ぶ（短い整合コピー / K_NO_WAIT 投入）。
  static void publish_state(const robot_state& s);
  static command_output command();
  static void generate_events(uint32_t now_ms);

  // EVT_LOG 生成（両スレッドから呼べる）。level: 1=info 2=warn 3=error。
  static void log(uint8_t level, const char* text);
  static stats get_stats();

 private:
  struct tx_item {  // k_msgq 要素（エンコード済みフレーム）
    uint8_t len;
    uint8_t data[link_protocol::MAX_FRAME];
  };

  // complete-class 規則を避けるためメンバ初期化子を持たない POD。
  struct subscriptions {
    uint8_t pose_rate_hz;
    bool log;
    bool status;
  };

  static void dispatch(const link_protocol::request& req);
  static void send_frame(uint8_t type, uint8_t seq, const uint8_t* payload,
                         uint8_t len);
  static void send_ack(uint8_t req_type, uint8_t seq, uint8_t result);
  static void queue_frame(uint8_t type, const uint8_t* payload, uint8_t len);
  static void clear_link_state_locked();  // irq_lock 保持中に呼ぶ

  static inline const param_set* p_ = nullptr;

  static inline link_protocol::frame_decoder decoder_;
  static inline uint8_t rx_scratch_[128] = {};

  // 指令スロット（irq_lock 保護）。リンクスレッドが書・制御スレッドが読。
  static inline uint8_t mode_ = link_mode::STANDBY;
  static inline vec3 vel_cmd_{};
  static inline uint32_t last_vel_ms_ = 0;
  static inline uint32_t last_rx_ms_ = 0;
  static inline bool link_active_ = false;
  static inline bool idle_safe_ = false;  // アイドル失効を一度だけ適用する

  static inline subscriptions subs_{};
  static inline robot_state state_box_{};

  static inline struct k_msgq tx_msgq_ = {};
  static inline char tx_msgq_buf_[8 * sizeof(tx_item)] = {};
  static inline struct k_poll_event poll_events_[2] = {};

  // イベント分周・変化検出（制御スレッドのみアクセス）。
  static inline uint32_t pose_div_count_ = 0;
  static inline uint8_t prev_status_[4] = {};
  static inline uint32_t last_status_tx_ms_ = 0;
  static inline uint8_t prev_log_mode_ = 0xFF;
  static inline bool prev_estop_ = false;

  static inline stats stats_{};
};

}  // namespace bb
