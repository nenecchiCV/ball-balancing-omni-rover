# frame / crc16

ロボットリンクプロトコルのフレーム符号化・ストリーム復号。`robot_link_protocol.md` のフレーム形式を実装する。他層に依存しない。

## 公開型

```rust
pub const SYNC: u8 = 0xA5;
pub const MAX_PAYLOAD: usize = 255;

pub struct Frame {
    pub msg_type: u8,
    pub seq: u8,
    pub payload: Vec<u8>,
}

pub struct FrameDecoder { /* 内部バッファ */ }
```

`crc16` は `pub fn crc16_ccitt(data: &[u8]) -> u16`（poly `0x1021`、init `0xFFFF`）を提供する。

## 公開関数

- `Frame::encode(&self) -> Vec<u8>` — `SYNC,TYPE,SEQ,LEN,PAYLOAD,CRC16(LE)` の並びで符号化する。
- `FrameDecoder::new() -> Self` — 空バッファのデコーダを生成する。
- `FrameDecoder::push(&mut self, data: &[u8]) -> Vec<Frame>` — 受信バイトを追加し、取り出せた完全フレームをすべて返す。`0xA5` 走査で再同期し、CRC 不一致フレームは破棄する。
- `crc16_ccitt(data: &[u8]) -> u16` — `TYPE` 先頭から `PAYLOAD` 末尾までの区間に対して計算する。

## 内部状態

- `FrameDecoder` は未確定バイト列を `Vec<u8>` で保持する。`SYNC` 発見→ヘッダ 4 B 確定→`LEN`+CRC 到着待ちの段階処理とし、途中で `0xA5` が現れた場合は新候補へ切り替える。

## 備考

- リンクはバイトストリームのため、1 通知で複数フレーム・フレーム分割の両方が発生し得る。`push` は常に抽出可能な全フレームを返す。
