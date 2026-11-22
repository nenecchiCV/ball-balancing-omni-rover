# link_protocol

ロボットリンクプロトコルのフレーム符号化・ストリーム復号・メッセージ型定義。プロトコル仕様の正本は `app_ws/robot_controller_tauri/spec/basic_design/robot_link_protocol.md`。他層に依存しない純粋ロジックで、ホストテスト対象。

## 公開型

```cpp
namespace link_protocol {
  // メッセージ種別（namespace 定数群）
  namespace req { PING=0x01, SET_VELOCITY=0x10, SET_POSE_TARGET=0x11, STOP=0x12,
                  SET_CONTROL_MODE=0x13, RESET_ODOMETRY=0x14, SUBSCRIBE=0x20,
                  UNSUBSCRIBE=0x21, GET_STATUS=0x30 }
  namespace resp { ACK=0x80, PONG=0x81, STATUS=0x82 }
  namespace evt { POSE=0x90, LOG=0x91, STATUS=0x92 }
  namespace ack_result { OK=0, INVALID_PARAM=1, REJECTED_STATE=2, NOT_READY=3, UNSUPPORTED=4 }
  constexpr uint8_t PROTO_VER = 1;
  constexpr uint8_t ROBOT_CAPS = 0x07;       // 位置制御・オドメトリ・ログ全対応
  constexpr uint8_t MAX_PAYLOAD = 64;         // EVT_LOG 本文 ≤ 60B

  struct request {           // デコード済み受信フレーム
    uint8_t type;
    uint8_t seq;
    uint8_t payload_len;
    uint8_t payload[MAX_PAYLOAD];
  };

  class frame_decoder {      // バイトストリーム → フレーム切出し
   public:
    void reset();
    void push(uint8_t byte);          // 内部バッファへ積む（満杯時は最古を捨てる）
    bool poll(request* out);          // フレーム確定で true（複数回呼んで排水）
  };
}
```

## 公開関数

- `uint16_t crc16(const uint8_t* data, uint32_t len)` — CRC16-CCITT-FALSE（poly 0x1021、init 0xFFFF）。
- `uint32_t encode(uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t len, uint8_t* out)` — `SYNC|TYPE|SEQ|LEN|PAYLOAD|CRC16-LE` を `out` に書き、総バイト数（`len + 6`）を返す。
- `float read_f32(const uint8_t* p)` / `void write_f32(uint8_t* p, float v)` — LE `f32` の読み書き（`memcpy`/`bit_cast`）。
- `uint32_t read_u32(const uint8_t* p)` / `void write_u32(uint8_t* p, uint32_t v)` — LE `u32` 同上。

## 内部状態（frame_decoder）

- `buf_[2*MAX_FRAME]`/`n_`: 受信バイトの保持バッファ。`poll()` が `0xA5` 位置まで先頭を捨て、ヘッダ 3B・`LEN<=MAX_PAYLOAD`・`PAYLOAD+CRC` 到着・CRC 照合の順に検査する。
- `LEN` 異常・CRC 不一致は SYNC 1 バイトのみ捨てて次の `0xA5` 候補を走査し再同期する（アプリ側実装と同じ規則）。
- CRC は `TYPE|SEQ|LEN|PAYLOAD` に掛け、LE で格納する。

## 備考

- エンコード出力バッファは `MAX_FRAME`（=`MAX_PAYLOAD + 6`）バイト要求。イベント送出は `seq = 0`。
- ペイロードの意味づけ（vx の符号等）は呼び出し側責務で、本コンポーネントは生バイトレイアウトのみ扱う。
