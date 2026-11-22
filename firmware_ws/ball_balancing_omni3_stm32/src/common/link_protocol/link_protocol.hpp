#pragma once

#include <cstdint>

// ロボットリンクプロトコルのフレーム符号化・ストリーム復号・メッセージ型
// （FR-09, FR-11）。ワイヤ仕様の正本は
// app_ws/robot_controller_tauri/spec/basic_design/robot_link_protocol.md。
// 他層に依存しない純粋ロジックで、ホストテスト対象。
namespace bb::link_protocol {

// フレーム: SYNC(0xA5) | TYPE | SEQ | LEN | PAYLOAD[LEN] | CRC16-LE。
// CRC16-CCITT-FALSE（poly 0x1021, init 0xFFFF）を TYPE..PAYLOAD に掛ける。
constexpr uint8_t SYNC = 0xA5;
constexpr uint8_t HEADER_LEN = 4;
constexpr uint8_t CRC_LEN = 2;
constexpr uint8_t MAX_PAYLOAD = 64;  // EVT_LOG 本文を 60B までに抑えるため
constexpr uint8_t MAX_FRAME = HEADER_LEN + MAX_PAYLOAD + CRC_LEN;
constexpr uint8_t PROTO_VER = 1;
// PONG 能力ビット: bit0 位置制御 / bit1 オドメトリ / bit2 ログ。
constexpr uint8_t ROBOT_CAPS = 0x07;

// リクエスト種別（ホスト → ロボット）。
namespace req {
constexpr uint8_t PING = 0x01;
constexpr uint8_t SET_VELOCITY = 0x10;
constexpr uint8_t SET_POSE_TARGET = 0x11;
constexpr uint8_t STOP = 0x12;
constexpr uint8_t SET_CONTROL_MODE = 0x13;
constexpr uint8_t RESET_ODOMETRY = 0x14;
constexpr uint8_t SUBSCRIBE = 0x20;
constexpr uint8_t UNSUBSCRIBE = 0x21;
constexpr uint8_t GET_STATUS = 0x30;
}  // namespace req

// レスポンス種別（ロボット → ホスト、seq はリクエストのエコー）。
namespace resp {
constexpr uint8_t ACK = 0x80;
constexpr uint8_t PONG = 0x81;
constexpr uint8_t STATUS = 0x82;
}  // namespace resp

// イベント種別（ロボット → ホスト、seq = 0）。
namespace evt {
constexpr uint8_t POSE = 0x90;
constexpr uint8_t LOG = 0x91;
constexpr uint8_t STATUS = 0x92;
}  // namespace evt

// ACK result コード。
namespace ack_result {
constexpr uint8_t OK = 0;
constexpr uint8_t INVALID_PARAM = 1;
constexpr uint8_t REJECTED_STATE = 2;
constexpr uint8_t NOT_READY = 3;
constexpr uint8_t UNSUPPORTED = 4;
}  // namespace ack_result

// デコード済み受信フレーム。ペイロードの意味づけは呼出し側が行う。
struct request {
  uint8_t type = 0;
  uint8_t seq = 0;
  uint8_t payload_len = 0;
  uint8_t payload[MAX_PAYLOAD] = {};
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
// 過大 LEN のフレームは SYNC バイトのみ捨てて次候補を走査する。
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
  void drop_front(uint8_t n);

  // 最大フレーム 2 本分。SYNC 欠落ゴミが溜まっても poll 側で掃く。
  uint8_t buf_[2 * MAX_FRAME] = {};
  uint8_t n_ = 0;
  uint32_t dropped_ = 0;
};

}  // namespace bb::link_protocol
