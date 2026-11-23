// link_protocol ホストテスト: CRC16・符号化・ストリーム復号・再同期。
#include "common/link_protocol/link_protocol.hpp"
#include "test_util.hpp"

namespace bb {
namespace {

using link_protocol::frame_decoder;
using link_protocol::request;
using link_protocol::SYNC;

// フレームをエンコードして返す。
uint32_t enc(uint8_t type, uint8_t seq, const uint8_t* pl, uint8_t len,
             uint8_t* out) {
  return link_protocol::encode(type, seq, pl, len, out);
}

void feed(frame_decoder& d, const uint8_t* buf, uint32_t n) {
  for (uint32_t i = 0; i < n; ++i) {
    d.push(buf[i]);
  }
}

void crc16_known_vector() {
  // "123456789" → 0x29B1（CRC16-CCITT-FALSE）。
  const uint8_t s[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  CHECK_TRUE(link_protocol::crc16(s, 9) == 0x29B1);
  CHECK_TRUE(link_protocol::crc16(s, 0) == 0xFFFF);
}

void encode_layout() {
  // rev C: SYNC | LEN | TYPE | SEQ | DATA | CRC16-LE。
  const uint8_t pl[3] = {0x11, 0x22, 0x33};
  uint8_t buf[link_protocol::MAX_FRAME];
  const uint32_t n = enc(0x10, 7, pl, 3, buf);
  CHECK_TRUE(n == 4 + 3 + 2);
  CHECK_TRUE(buf[0] == SYNC && buf[1] == 5 && buf[2] == 0x10 && buf[3] == 7);
  const uint16_t crc = link_protocol::crc16(&buf[1], 1 + 5);
  CHECK_TRUE(buf[7] == (crc & 0xFF));
  CHECK_TRUE(buf[8] == (crc >> 8));
}

void roundtrip() {
  const uint8_t pl[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
  uint8_t buf[link_protocol::MAX_FRAME];
  const uint32_t n = enc(link_protocol::req::SET_VELOCITY, 42, pl, 12, buf);
  frame_decoder d;
  feed(d, buf, n);
  request r{};
  CHECK_TRUE(d.poll(&r));
  CHECK_TRUE(r.type == 0x10 && r.seq == 42 && r.payload_len == 12);
  CHECK_TRUE(r.payload[11] == 11);
  CHECK_TRUE(!d.poll(&r));  // 1 フレームのみ
}

void split_push() {
  const uint8_t pl[1] = {9};
  uint8_t buf[link_protocol::MAX_FRAME];
  const uint32_t n = enc(link_protocol::req::SET_CONTROL_MODE, 1, pl, 1, buf);
  frame_decoder d;
  request r{};
  // 先頭 3 バイトだけでは確定しない。
  feed(d, buf, 3);
  CHECK_TRUE(!d.poll(&r));
  feed(d, buf + 3, n - 3);
  CHECK_TRUE(d.poll(&r));
  CHECK_TRUE(r.payload_len == 1 && r.payload[0] == 9);
}

void garbage_resync() {
  const uint8_t pl[1] = {2};
  uint8_t buf[link_protocol::MAX_FRAME];
  const uint32_t n = enc(link_protocol::req::UNSUBSCRIBE, 5, pl, 1, buf);
  frame_decoder d;
  // ゴミ → フレームの順で投入。
  const uint8_t garbage[5] = {0x00, 0xFF, 0x13, 0x55, 0x77};
  feed(d, garbage, sizeof(garbage));
  feed(d, buf, n);
  request r{};
  CHECK_TRUE(d.poll(&r));
  CHECK_TRUE(r.type == 0x21 && r.seq == 5);
}

void bad_crc_resyncs() {
  const uint8_t pl[1] = {1};
  uint8_t good[link_protocol::MAX_FRAME];
  const uint32_t n = enc(link_protocol::req::STOP, 9, pl, 0, good);
  frame_decoder d;
  // CRC 破損フレーム → 正常フレーム。
  uint8_t bad[link_protocol::MAX_FRAME];
  const uint32_t m = enc(link_protocol::req::STOP, 1, pl, 0, bad);
  bad[m - 1] ^= 0xFF;
  feed(d, bad, m);
  feed(d, good, n);
  request r{};
  CHECK_TRUE(d.poll(&r));
  CHECK_TRUE(r.seq == 9);
}

void embedded_sync_resyncs() {
  // 破損フレームのペイロード領域に正常フレーム先頭が食い込んでも、
  // CRC 不一致で SYNC を捨てて再スキャンし回復する。
  uint8_t good[link_protocol::MAX_FRAME];
  const uint32_t n = enc(link_protocol::req::PING, 3, nullptr, 0, good);
  frame_decoder d;
  // SYNC|LEN=8|TYPE|SEQ + ゴミ 4B + 正常フレーム。
  // 破損フレームはゴミ+正常フレーム先頭を payload/CRC として吸収し、
  // CRC 照合に失敗 → 先頭 SYNC のみ捨てて正常フレームへ再同期する。
  const uint8_t bad_head[8] = {SYNC, 8, 0x12, 0x00, 0x11, 0x22, 0x33, 0x44};
  feed(d, bad_head, sizeof(bad_head));
  feed(d, good, n);
  request r{};
  CHECK_TRUE(d.poll(&r));
  CHECK_TRUE(r.type == 0x01 && r.seq == 3);
}

void undersize_len_dropped() {
  // LEN < META_LEN(2) のフレームは SYNC のみ捨てて再同期する。
  const uint8_t bad[7] = {SYNC, 1, 0x10, 0x01, 0x00, 0x00, 0x00};
  const uint8_t pl[1] = {4};
  uint8_t good[link_protocol::MAX_FRAME];
  const uint32_t n = enc(link_protocol::req::STOP, 8, pl, 0, good);
  frame_decoder d;
  feed(d, bad, sizeof(bad));
  feed(d, good, n);
  request r{};
  CHECK_TRUE(d.poll(&r));
  CHECK_TRUE(r.seq == 8);
}

void f32_u32_helpers() {
  uint8_t b[8];
  link_protocol::write_f32(b, 0.5F);
  CHECK_NEAR(link_protocol::read_f32(b), 0.5F, 1e-6F);
  link_protocol::write_u32(&b[4], 0xDEADBEEF);
  CHECK_TRUE(link_protocol::read_u32(&b[4]) == 0xDEADBEEF);
}

}  // namespace

void link_protocol_tests() {
  crc16_known_vector();
  encode_layout();
  roundtrip();
  split_push();
  garbage_resync();
  bad_crc_resyncs();
  embedded_sync_resyncs();
  undersize_len_dropped();
  f32_u32_helpers();
}

}  // namespace bb
