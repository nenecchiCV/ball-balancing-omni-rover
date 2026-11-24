#pragma once

#include <zephyr/kernel.h>

#include <cstdint>

#include "common/control_params/control_params.hpp"
#include "common/link_protocol/link_protocol.hpp"
#include "common/vector_math/vector_math.hpp"
#include "feature/tune_deadzone/tune_deadzone.hpp"

namespace bb {

// リンク制御モード（SET_CONTROL_MODE の mode 値）。
namespace link_mode {
constexpr uint8_t STANDBY = 0;
constexpr uint8_t VELOCITY = 1;
constexpr uint8_t POSITION = 2;
}  // namespace link_mode

// ロボットリンクのセッション管理（FR-09, FR-11, FR-14、static class）。
// 受信フレームの振分け・モード/指令スロット・購読・安全ウォッチドッグ・
// イベントバッチ生成を担う。スレッドモデル:
//   リンクスレッド: run_once() を繰返す。rx_sem/evt_sem の k_poll を
//                   バッチ期限付きタイムアウトで待ち、受信排水・振分け・
//                   単発応答・バッチフラッシュを行う。
//   制御スレッド:   publish_state / command / generate_events /
//                   notify_calib_result（整合コピー・irq_lock 下の短い
//                   バッファ投入でブロックしない）。
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
    float tilt;                   // 推定姿勢の傾斜角（START 受理判定用）
    float raw_x, raw_y, raw_yaw;  // RESET_ODOMETRY / SET_POSE / STOP 用の生値
    float x, y, yaw;              // イベント用オフセット済 pose
    float vx, vy, yaw_rate;
    // EVT_SENSORS 用のセンサ生値（FR-18）。imu_* は推定処理前の
    // 読出し値そのまま、enc_count は累積生カウントの i32 切詰め。
    vec3 imu_accel;
    vec3 imu_gyro;
    int32_t enc_count[3];
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
  // リンクスレッドのループ本体。rx_sem/evt_sem の k_poll を
  // バッチ期限付きタイムアウトで待つ。
  static void run_once();

  // 以下は制御スレッドから呼ぶ（短い整合コピー / irq_lock 下の投入）。
  static void publish_state(const robot_state& s);
  static command_output command();
  static void generate_events(uint32_t now_ms);

  // CALIB_RESULT / TUNE_RESULT の即時送出（制御スレッド、FR-13/FR-17）。
  // バッチを経由しない。
  static void notify_calib_result(uint8_t result, uint8_t seq,
                                  float gyro_bias_z);
  static void notify_tune_result(uint8_t result, uint8_t seq,
                                 const tune_deadzone::measure& m);

  // EVT_LOG 生成（両スレッドから呼べる）。level: 1=info 2=warn 3=error。
  static void log(uint8_t level, const char* text);
  static stats get_stats();

 private:
  // complete-class 規則を避けるためメンバ初期化子を持たない POD。
  struct subscriptions {
    uint8_t pose_rate_hz;
    uint8_t sensors_rate_hz;
    uint8_t status_rate_hz;
    bool log;
    bool status;
  };

  // エンコード済みフレーム 1 件分のスロット（バッチバッファ用）。
  struct evt_slot {
    uint8_t len;
    uint8_t data[link_protocol::MAX_FRAME];
  };

  static constexpr uint8_t LOG_FIFO_DEPTH = 4;     // EVT_LOG リング深さ
  static constexpr uint32_t FLUSH_THRESHOLD = 192; // 早期フラッシュ閾値 [B]
  static constexpr uint32_t BATCH_BUF_SIZE = 384;  // フラッシュ連結バッファ

  static void dispatch(const link_protocol::request& req);
  static void send_frame(uint8_t type, uint8_t seq, const uint8_t* payload,
                         uint8_t len);
  static void send_ack(uint8_t req_type, uint8_t seq, uint8_t result);
  // START_OPERATION 受理条件の充足ビット（ready_mask::*、STATUS 拡張部用）。
  static uint8_t start_ready_mask(const robot_state& s);
  // バッチバッファへの投入（irq_lock 使用、どちらのスレッドからも可）。
  static void queue_slot(evt_slot& slot, bool& dirty, uint8_t type,
                         const uint8_t* payload, uint8_t len);
  static void queue_log_frame(const uint8_t* payload, uint8_t len);
  // フラッシュ判定・一括送出（リンクスレッドのみ）。
  static void flush_batch(uint32_t now_ms);
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

  // イベントバッチバッファ（irq_lock 保護、FR-14）。
  // POSE/STATUS は最新値スロット、LOG は FIFO。
  static inline evt_slot pose_slot_{};
  static inline evt_slot sensors_slot_{};
  static inline evt_slot status_slot_{};
  static inline bool pose_dirty_ = false;
  static inline bool sensors_dirty_ = false;
  static inline bool status_dirty_ = false;
  static inline evt_slot log_fifo_[LOG_FIFO_DEPTH] = {};
  static inline uint8_t log_head_ = 0;  // 最古要素の index
  static inline uint8_t log_count_ = 0;
  static inline uint32_t pending_bytes_ = 0;  // 滞留バイト見積
  static inline uint32_t next_flush_ms_ = 0;

  static inline struct k_sem evt_sem_ = {};
  static inline struct k_poll_event poll_events_[2] = {};

  // イベント分周・変化検出（制御スレッドのみアクセス）。
  static inline uint32_t pose_div_count_ = 0;
  static inline uint32_t sensors_div_count_ = 0;
  static inline uint8_t prev_status_[11] = {};
  static inline uint32_t last_status_tx_ms_ = 0;
  static inline uint8_t prev_log_mode_ = 0xFF;
  static inline bool prev_estop_ = false;

  static inline stats stats_{};
};

}  // namespace bb
