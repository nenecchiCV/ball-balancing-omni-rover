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

uint32_t encode(uint8_t type, uint8_t seq, const uint8_t* payload, uint8_t len,
                uint8_t* out) {
  out[0] = SYNC;
  out[1] = type;
  out[2] = seq;
  out[3] = len;
  if (len > 0) {
    memcpy(&out[HEADER_LEN], payload, len);
  }
  const uint16_t crc = crc16(&out[1], 3 + len);
  out[HEADER_LEN + len] = static_cast<uint8_t>(crc & 0xFF);
  out[HEADER_LEN + len + 1] = static_cast<uint8_t>(crc >> 8);
  return HEADER_LEN + len + CRC_LEN;
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
}

void frame_decoder::push(uint8_t byte) {
  if (n_ == sizeof(buf_)) {
    drop_front(1);
  }
  buf_[n_++] = byte;
}

bool frame_decoder::poll(request* out) {
  while (true) {
    // SYNC 位置まで先頭を捨てる。
    uint8_t pos = 0;
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
    const uint8_t len = buf_[3];
    if (len > MAX_PAYLOAD) {
      // LEN 異常は SYNC のみ捨てて次の 0xA5 候補を走査する。
      ++dropped_;
      drop_front(1);
      continue;
    }
    const uint8_t total = HEADER_LEN + len + CRC_LEN;
    if (n_ < total) {
      return false;
    }
    const uint16_t want = crc16(&buf_[1], 3 + len);
    const uint16_t got =
        static_cast<uint16_t>(buf_[HEADER_LEN + len]) |
        (static_cast<uint16_t>(buf_[HEADER_LEN + len + 1]) << 8);
    if (want != got) {
      ++dropped_;
      drop_front(1);
      continue;
    }
    out->type = buf_[1];
    out->seq = buf_[2];
    out->payload_len = len;
    if (len > 0) {
      memcpy(out->payload, &buf_[HEADER_LEN], len);
    }
    drop_front(total);
    return true;
  }
}

void frame_decoder::drop_front(uint8_t n) {
  if (n >= n_) {
    n_ = 0;
    return;
  }
  memmove(&buf_[0], &buf_[n], n_ - n);
  n_ = static_cast<uint8_t>(n_ - n);
}

}  // namespace bb::link_protocol
