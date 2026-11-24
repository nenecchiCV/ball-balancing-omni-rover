#include "common/link_protocol/link_protocol.hpp"

#include <string.h>

namespace bb::link_protocol {

uint16_t crc16(const uint8_t* data, uint32_t len) {
  uint16_t crc = 0xFFFF;
  for (uint32_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000U) != 0 ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                                 : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

// rev C: SYNC | LEN(=META+data) | TYPE | SEQ | DATA | CRC16-LE。
// CRC は LEN 先頭から DATA 末尾まで（out[1..3+len]）に掛ける。
uint32_t encode(uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t len,
                uint8_t* out) {
  out[0] = SYNC;
  out[1] = static_cast<uint8_t>(META_LEN + len);
  out[2] = type;
  out[3] = seq;
  if (len > 0) {
    memcpy(&out[4], payload, len);
  }
  const uint16_t crc = crc16(&out[1], 3 + len);
  out[4 + len] = static_cast<uint8_t>(crc & 0xFF);
  out[4 + len + 1] = static_cast<uint8_t>(crc >> 8);
  return 4 + len + CRC_LEN;
}

float read_f32(const uint8_t* p) {
  float v = 0.0F;
  memcpy(&v, p, sizeof(v));
  return v;
}

void write_f32(uint8_t* p, float v) {
  memcpy(p, &v, sizeof(v));
}

uint32_t read_u32(const uint8_t* p) {
  uint32_t v = 0;
  memcpy(&v, p, sizeof(v));
  return v;
}

void write_u32(uint8_t* p, uint32_t v) {
  memcpy(p, &v, sizeof(v));
}

void frame_decoder::reset() {
  n_ = 0;
  pending_ = 0;
}

void frame_decoder::push(uint8_t byte) {
  compact();
  if (n_ == sizeof(buf_)) {
    drop_front(1);
  }
  buf_[n_++] = byte;
}

bool frame_decoder::poll(request* out) {
  compact();
  while (true) {
    // SYNC 位置まで先頭を捨てる。
    uint16_t pos = 0;
    while (pos < n_ && buf_[pos] != SYNC) {
      ++pos;
    }
    if (pos == n_) {
      n_ = 0;
      return false;
    }
    drop_front(pos);
    if (n_ < HEADER_LEN) {
      return false;
    }
    const uint8_t len = buf_[1];
    if (len < META_LEN) {
      // LEN 異常は SYNC のみ捨てて次の 0xA5 候補を走査する。
      ++dropped_;
      drop_front(1);
      continue;
    }
    const uint16_t total = static_cast<uint16_t>(HEADER_LEN + len + CRC_LEN);
    if (n_ < total) {
      return false;
    }
    const uint16_t want = crc16(&buf_[1], 1 + len);
    const uint16_t got =
        static_cast<uint16_t>(buf_[HEADER_LEN + len]) |
        (static_cast<uint16_t>(buf_[HEADER_LEN + len + 1]) << 8);
    if (want != got) {
      ++dropped_;
      drop_front(1);
      continue;
    }
    out->type = buf_[2];
    out->seq = buf_[3];
    out->payload_len = static_cast<uint8_t>(len - META_LEN);
    // DATA 部は buf_ 内参照を返し、消費は次回呼出しの compact() まで遅延する。
    out->payload = &buf_[4];
    pending_ = total;
    return true;
  }
}

void frame_decoder::compact() {
  if (pending_ > 0) {
    drop_front(pending_);
    pending_ = 0;
  }
}

void frame_decoder::drop_front(uint16_t n) {
  if (n >= n_) {
    n_ = 0;
    return;
  }
  memmove(&buf_[0], &buf_[n], n_ - n);
  n_ = static_cast<uint16_t>(n_ - n);
}

}  // namespace bb::link_protocol
