//! フレーム符号化・ストリーム復号 — robot_link_protocol.md「フレーム形式」準拠。
//!
//! SYNC(0xA5) | TYPE | SEQ | LEN | PAYLOAD[LEN] | CRC16-LE(TYPE..PAYLOAD)

use super::crc16::crc16_ccitt;

pub const SYNC: u8 = 0xA5;
pub const HEADER_LEN: usize = 4;
pub const CRC_LEN: usize = 2;
pub const MAX_PAYLOAD: usize = 255;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    pub msg_type: u8,
    pub seq: u8,
    pub payload: Vec<u8>,
}

impl Frame {
    pub fn new(msg_type: u8, seq: u8, payload: Vec<u8>) -> Self {
        debug_assert!(payload.len() <= MAX_PAYLOAD);
        Self {
            msg_type,
            seq,
            payload,
        }
    }

    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(HEADER_LEN + self.payload.len() + CRC_LEN);
        out.push(SYNC);
        out.push(self.msg_type);
        out.push(self.seq);
        out.push(self.payload.len() as u8);
        out.extend_from_slice(&self.payload);
        let crc = crc16_ccitt(&out[1..]);
        out.extend_from_slice(&crc.to_le_bytes());
        out
    }
}

/// バイトストリームからフレームを切り出す蓄積型デコーダ。
/// SYNC 走査で再同期し、CRC 不一致フレームは破棄する。
#[derive(Default)]
pub struct FrameDecoder {
    buf: Vec<u8>,
}

impl FrameDecoder {
    pub fn new() -> Self {
        Self { buf: Vec::new() }
    }

    pub fn push(&mut self, data: &[u8]) -> Vec<Frame> {
        self.buf.extend_from_slice(data);
        let mut frames = Vec::new();
        loop {
            match self.buf.iter().position(|&b| b == SYNC) {
                Some(pos) if pos > 0 => {
                    self.buf.drain(..pos);
                }
                Some(_) => {}
                None => {
                    self.buf.clear();
                    break;
                }
            }
            if self.buf.len() < HEADER_LEN {
                break;
            }
            let len = self.buf[3] as usize;
            let total = HEADER_LEN + len + CRC_LEN;
            if self.buf.len() < total {
                break;
            }
            let crc_expected =
                u16::from_le_bytes([self.buf[total - 2], self.buf[total - 1]]);
            if crc16_ccitt(&self.buf[1..HEADER_LEN + len]) == crc_expected {
                frames.push(Frame {
                    msg_type: self.buf[1],
                    seq: self.buf[2],
                    payload: self.buf[HEADER_LEN..HEADER_LEN + len].to_vec(),
                });
                self.buf.drain(..total);
            } else {
                // SYNC バイトのみ捨てて次候補を走査する。
                self.buf.drain(..1);
            }
        }
        frames
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip() {
        let f = Frame::new(0x10, 7, vec![1, 2, 3]);
        let mut dec = FrameDecoder::new();
        let out = dec.push(&f.encode());
        assert_eq!(out, vec![f]);
    }

    #[test]
    fn split_delivery() {
        let f = Frame::new(0x01, 1, vec![]);
        let bytes = f.encode();
        let mut dec = FrameDecoder::new();
        assert!(dec.push(&bytes[..3]).is_empty());
        let out = dec.push(&bytes[3..]);
        assert_eq!(out, vec![f]);
    }

    #[test]
    fn bad_crc_resyncs() {
        let good = Frame::new(0x30, 9, vec![]).encode();
        let mut garbage = vec![0xA5, 0x10, 0x00, 0x02, 0xAA, 0xBB, 0xCC];
        garbage.extend_from_slice(&good);
        let mut dec = FrameDecoder::new();
        let out = dec.push(&garbage);
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].msg_type, 0x30);
    }
}
