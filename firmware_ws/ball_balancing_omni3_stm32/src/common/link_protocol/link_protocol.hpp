#pragma once

#include <cstdint>

// ロボットリンクプロトコルのフレーム符号化・ストリーム復号・メッセージ型
// （FR-09, FR-11, FR-17）。ワイヤ仕様の正本は
// app_ws/robot_controller_tauri/spec/basic_design/robot_link_protocol.md。
// 他層に依存しない純粋ロジックで、ホストテスト対象。
namespace bb::link_protocol {

// フレーム（rev C）: SYNC(0xA5) | LEN | PAYLOAD[LEN] | CRC16-LE。
// PAYLOAD = TYPE u8 | SEQ u8 | DATA[LEN-2]。
// CRC16-CCITT-FALSE（poly 0x1021, init 0xFFFF）を LEN..PAYLOAD に掛ける。
constexpr uint8_t SYNC = 0xA5;
constexpr uint8_t HEADER_LEN = 2;    // SYNC | LEN
constexpr uint8_t META_LEN = 2;      // PAYLOAD 内の TYPE | SEQ
constexpr uint8_t CRC_LEN = 2;
constexpr uint8_t MAX_DATA = 253;    // DATA 部最大（LEN ≤ 255）
// 258B。uint8_t では表せないため uint16_t とする。
constexpr uint16_t MAX_FRAME = HEADER_LEN + META_LEN + MAX_DATA + CRC_LEN;
constexpr uint8_t PROTO_VER = 3;
// PONG 能力ビット: bit0 位置制御 / bit1 オドメトリ / bit2 ログ /
// bit3 テストビルド / bit4 運転管理（START/STOP_OPERATION, CALIB, SET_POSE）。
constexpr uint8_t ROBOT_CAPS = 0x17;

// リクエスト種別（ホスト → ロボット）。
namespace req {
constexpr uint8_t PING = 0x01;
constexpr uint8_t SET_VELOCITY = 0x10;
constexpr uint8_t SET_POSE_TARGET = 0x11;
constexpr uint8_t STOP = 0x12;
constexpr uint8_t SET_CONTROL_MODE = 0x13;
constexpr uint8_t RESET_ODOMETRY = 0x14;
constexpr uint8_t START_OPERATION = 0x15;
constexpr uint8_t STOP_OPERATION = 0x16;
constexpr uint8_t SET_POSE = 0x17;
constexpr uint8_t CALIBRATE_IMU = 0x18;
constexpr uint8_t SET_PARAM = 0x19;
constexpr uint8_t SAVE_PARAMS = 0x1A;
constexpr uint8_t RESET_PARAMS = 0x1B;
constexpr uint8_t TUNE_DEADZONE = 0x1C;
constexpr uint8_t SUBSCRIBE = 0x20;
constexpr uint8_t UNSUBSCRIBE = 0x21;
constexpr uint8_t GET_STATUS = 0x30;
constexpr uint8_t TEST_ECHO = 0x31;
constexpr uint8_t GET_PARAMS = 0x32;
}  // namespace req

// レスポンス種別（ロボット → ホスト、seq はリクエストのエコー）。
namespace resp {
constexpr uint8_t ACK = 0x80;
constexpr uint8_t PONG = 0x81;
constexpr uint8_t STATUS = 0x82;
constexpr uint8_t CALIB_RESULT = 0x83;
constexpr uint8_t TEST_ECHO_RESP = 0x84;
constexpr uint8_t PARAMS = 0x85;
constexpr uint8_t TUNE_RESULT = 0x86;
}  // namespace resp

// イベント種別（ロボット → ホスト、seq = 0）。
namespace evt {
constexpr uint8_t POSE = 0x90;
constexpr uint8_t LOG = 0x91;
constexpr uint8_t STATUS = 0x92;
constexpr uint8_t SENSORS = 0x93;
}  // namespace evt

// ACK result コード。
namespace ack_result {
constexpr uint8_t OK = 0;
constexpr uint8_t INVALID_PARAM = 1;
constexpr uint8_t REJECTED_STATE = 2;
constexpr uint8_t NOT_READY = 3;
constexpr uint8_t UNSUPPORTED = 4;
}  // namespace ack_result

// STATUS/EVT_STATUS 拡張部（7 バイト目）の ready_mask ビット。
// START_OPERATION の受理条件を bit=1 で充足として表す。
namespace ready_mask {
constexpr uint8_t IMU_OK = 0x01;       // IMU 正常
constexpr uint8_t ESTOP_FREE = 0x02;   // E-Stop 解除
constexpr uint8_t QUALIFIED = 0x04;    // 推定器 bias qualification 完了
constexpr uint8_t TILT_OK = 0x08;      // tilt < controller.start_tilt
}  // namespace ready_mask

// デコード済み受信フレーム。payload は DATA 部のみ（TYPE/SEQ は別フィールド）。
// payload は decoder 内部バッファ内を指し、次回の push()/poll()/reset()
// 呼出しまで有効（消費済み領域の解放は呼出し側の使用後に遅延される）。
struct request {
  uint8_t type = 0;
  uint8_t seq = 0;
  uint8_t payload_len = 0;
  const uint8_t* payload = nullptr;
};

uint16_t crc16(const uint8_t* data, uint32_t len);

// out（MAX_FRAME バイト以上）へフレームを書き、総バイト数を返す。
uint32_t encode(uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t len,
                uint8_t* out);

// LE f32 / u32 の読み書き。
float read_f32(const uint8_t* p);
void write_f32(uint8_t* p, float v);
uint32_t read_u32(const uint8_t* p);
void write_u32(uint8_t* p, uint32_t v);

// バイトストリーム → フレーム切出し。SYNC 走査で再同期し、CRC 不一致・
// LEN 異常のフレームは SYNC バイトのみ捨てて次候補を走査する。
class frame_decoder {
 public:
  void reset();
  // 受信バイトを内部バッファへ積む。バッファ満杯時は最古 1 バイトを捨てる。
  void push(uint8_t byte);
  // バッファ先頭からフレームを 1 つ切り出す。確定したら true。
  bool poll(request* out);
  // CRC 不一致・LEN 異常で捨てたフレーム候補の数（診断用）。
  [[nodiscard]] uint32_t dropped() const {
    return dropped_;
  }

 private:
  void drop_front(uint16_t n);
  void compact();

  // 最大フレーム 2 本分。SYNC 欠落ゴミが溜まっても poll 側で掃く。
  uint8_t buf_[2 * MAX_FRAME] = {};
  uint16_t n_ = 0;
  // poll() が返した payload 参照を保護するため消費を遅延した先頭バイト数。
  uint16_t pending_ = 0;
  uint32_t dropped_ = 0;
};

}  // namespace bb::link_protocol
