#include "feature/link_session/link_session.hpp"

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>

#include "feature/odometry/odometry.hpp"
#include "feature/position_controller/position_controller.hpp"
#include "peripheral/link_uart/link_uart.hpp"

namespace bb {

namespace {

constexpr uint32_t OUTER_RATE_HZ = 200;  // generate_events の呼出し周期
constexpr uint8_t LOG_INFO = 1;
constexpr uint8_t LOG_WARN = 2;

}  // namespace

void link_session::init(const config& cfg) {
  p_ = cfg.params;
  (void)link_uart::init({DEVICE_DT_GET(DT_ALIAS(link_uart))});

  k_msgq_init(&tx_msgq_, tx_msgq_buf_, sizeof(tx_item), 8);
  k_poll_event_init(&poll_events_[0], K_POLL_TYPE_SEM_AVAILABLE,
                    K_POLL_MODE_NOTIFY_ONLY, link_uart::rx_sem());
  k_poll_event_init(&poll_events_[1], K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                    K_POLL_MODE_NOTIFY_ONLY, &tx_msgq_);
}

void link_session::run_once() {
  (void)k_poll(poll_events_, 2, K_FOREVER);

  if (poll_events_[0].state == K_POLL_STATE_SEM_AVAILABLE) {
    // Zephyr k_poll_event のイベント種別に応じた共用体メンバ参照。
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-union-access)
    (void)k_sem_take(poll_events_[0].sem, K_NO_WAIT);
    // RX リングを排水してフレーム切出し・振分け。
    for (;;) {
      const uint32_t n = link_uart::rx_read(rx_scratch_, sizeof(rx_scratch_));
      if (n == 0) {
        break;
      }
      for (uint32_t i = 0; i < n; ++i) {
        decoder_.push(rx_scratch_[i]);
      }
      link_protocol::request req{};
      while (decoder_.poll(&req)) {
        ++stats_.rx_frames;
        const uint32_t now = k_uptime_get_32();
        const unsigned int key = irq_lock();
        last_rx_ms_ = now;
        link_active_ = true;
        idle_safe_ = false;
        irq_unlock(key);
        dispatch(req);
      }
    }
    poll_events_[0].state = K_POLL_STATE_NOT_READY;
  }

  if (poll_events_[1].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE) {
    tx_item it{};
    while (k_msgq_get(&tx_msgq_, &it, K_NO_WAIT) == 0) {
      (void)link_uart::write(it.data, it.len);
    }
    poll_events_[1].state = K_POLL_STATE_NOT_READY;
  }
}

void link_session::publish_state(const robot_state& s) {
  const unsigned int key = irq_lock();
  state_box_ = s;
  irq_unlock(key);
}

link_session::command_output link_session::command() {
  const uint32_t now = k_uptime_get_32();
  bool clear_target = false;
  const unsigned int key = irq_lock();
  command_output o{};
  o.link_active = link_active_;
  if (link_active_) {
    const bool vel_alive = (now - last_vel_ms_) <= p_->link.watchdog_ms;
    const bool link_alive = (now - last_rx_ms_) <= p_->link.idle_ms;
    // UART アイドル失効: ゼロ指令＋購読・目標を解除（一度だけ）。
    if (!link_alive && !idle_safe_) {
      subs_ = {};
      mode_ = link_mode::STANDBY;
      idle_safe_ = true;
      clear_target = true;
    }
    o.mode = mode_;
    o.velocity_cmd = vel_alive ? vel_cmd_ : vec3{};
  }
  irq_unlock(key);
  // irq_lock を内側で取る関数のためロック外で呼ぶ。
  if (clear_target) {
    position_controller::clear_target();
  }
  return o;
}

void link_session::generate_events(uint32_t now_ms) {
  const unsigned int key = irq_lock();
  const robot_state s = state_box_;
  const subscriptions subs = subs_;
  const uint8_t mode = mode_;
  irq_unlock(key);

  // EVT_POSE: 購読中はレート分周で送出。
  if (subs.pose_rate_hz > 0) {
    const uint32_t div =
        (OUTER_RATE_HZ + subs.pose_rate_hz - 1) / subs.pose_rate_hz;
    if (++pose_div_count_ >= div) {
      pose_div_count_ = 0;
      uint8_t pl[28];
      link_protocol::write_u32(&pl[0], now_ms);
      link_protocol::write_f32(&pl[4], s.x);
      link_protocol::write_f32(&pl[8], s.y);
      link_protocol::write_f32(&pl[12], s.yaw);
      link_protocol::write_f32(&pl[16], s.vx);
      link_protocol::write_f32(&pl[20], s.vy);
      link_protocol::write_f32(&pl[24], s.yaw_rate);
      queue_frame(link_protocol::evt::POSE, pl, sizeof(pl));
    }
  }

  // EVT_STATUS: 変化検出 + 100 ms ハートビート。
  if (subs.status) {
    const uint8_t cur[4] = {mode, static_cast<uint8_t>(s.enable ? 1 : 0),
                            static_cast<uint8_t>(s.estop ? 1 : 0),
                            static_cast<uint8_t>(s.imu_ok ? 1 : 0)};
    const bool changed = memcmp(cur, prev_status_, sizeof(cur)) != 0;
    const bool heartbeat = (now_ms - last_status_tx_ms_) >= 100;
    if (changed || heartbeat) {
      memcpy(prev_status_, cur, sizeof(cur));
      last_status_tx_ms_ = now_ms;
      queue_frame(link_protocol::evt::STATUS, cur, sizeof(cur));
    }
  }

  // 主要イベントの EVT_LOG。
  if (mode != prev_log_mode_) {
    const char* m = "mode standby";
    if (mode == link_mode::POSITION) {
      m = "mode position";
    } else if (mode == link_mode::VELOCITY) {
      m = "mode velocity";
    }
    log(LOG_INFO, m);
    prev_log_mode_ = mode;
  }
  if (s.estop != prev_estop_) {
    log(s.estop ? LOG_WARN : LOG_INFO,
        s.estop ? "estop asserted" : "estop released");
    prev_estop_ = s.estop;
  }
}

void link_session::log(uint8_t level, const char* text) {
  const size_t tlen = strlen(text);
  const uint8_t n = (tlen > 60) ? 60 : static_cast<uint8_t>(tlen);
  uint8_t pl[1 + 60];
  pl[0] = level;
  memcpy(&pl[1], text, n);
  queue_frame(link_protocol::evt::LOG, pl, static_cast<uint8_t>(n + 1));
}

link_session::stats link_session::get_stats() {
  stats s{};
  const unsigned int key = irq_lock();
  s = stats_;
  s.last_rx_ms = last_rx_ms_;
  irq_unlock(key);
  // decoder_ はリンクスレッドのみ触るが、get_stats はシェルスレッド
  // から来るため近似的な診断値として扱う。
  s.rx_crc_drops = decoder_.dropped();
  s.rx_overruns = link_uart::rx_dropped();
  s.tx_drops += link_uart::tx_dropped();
  return s;
}

void link_session::dispatch(const link_protocol::request& req) {
  const uint8_t* pl = req.payload;
  const uint8_t n = req.payload_len;
  const uint32_t now = k_uptime_get_32();

  // 状態メールボックスの整合コピー。
  unsigned int key = irq_lock();
  const robot_state s = state_box_;
  const uint8_t mode = mode_;
  irq_unlock(key);

  switch (req.type) {
    case link_protocol::req::PING: {
      const uint8_t r[3] = {req.seq, link_protocol::PROTO_VER,
                            link_protocol::ROBOT_CAPS};
      send_frame(link_protocol::resp::PONG, 0, r, sizeof(r));
      break;
    }
    case link_protocol::req::SET_VELOCITY: {
      if (n != 12) {
        send_ack(req.type, req.seq, link_protocol::ack_result::INVALID_PARAM);
        break;
      }
      if (mode != link_mode::VELOCITY) {
        send_ack(req.type, req.seq, link_protocol::ack_result::REJECTED_STATE);
        break;
      }
      const vec3 v{link_protocol::read_f32(&pl[0]),
                   link_protocol::read_f32(&pl[4]),
                   link_protocol::read_f32(&pl[8])};
      key = irq_lock();
      vel_cmd_ = v;
      last_vel_ms_ = now;
      irq_unlock(key);
      send_ack(req.type, req.seq, link_protocol::ack_result::OK);
      break;
    }
    case link_protocol::req::SET_POSE_TARGET: {
      if (n != 12) {
        send_ack(req.type, req.seq, link_protocol::ack_result::INVALID_PARAM);
        break;
      }
      if (!s.imu_ok || !s.qualified) {
        send_ack(req.type, req.seq, link_protocol::ack_result::NOT_READY);
        break;
      }
      position_controller::set_target(link_protocol::read_f32(&pl[0]),
                                      link_protocol::read_f32(&pl[4]),
                                      link_protocol::read_f32(&pl[8]));
      key = irq_lock();
      mode_ = link_mode::POSITION;
      irq_unlock(key);
      send_ack(req.type, req.seq, link_protocol::ack_result::OK);
      break;
    }
    case link_protocol::req::STOP: {
      key = irq_lock();
      vel_cmd_ = vec3{};
      const uint8_t m = mode_;
      const float cx = s.x;
      const float cy = s.y;
      irq_unlock(key);
      if (m == link_mode::POSITION) {
        position_controller::hold_at(cx, cy);
      }
      send_ack(req.type, req.seq, link_protocol::ack_result::OK);
      break;
    }
    case link_protocol::req::SET_CONTROL_MODE: {
      if (n != 1 || pl[0] > link_mode::POSITION) {
        send_ack(req.type, req.seq, link_protocol::ack_result::INVALID_PARAM);
        break;
      }
      if (pl[0] == link_mode::POSITION && (!s.imu_ok || !s.qualified)) {
        send_ack(req.type, req.seq, link_protocol::ack_result::NOT_READY);
        break;
      }
      key = irq_lock();
      mode_ = pl[0];
      if (pl[0] == link_mode::STANDBY) {
        vel_cmd_ = vec3{};
      }
      irq_unlock(key);
      if (pl[0] == link_mode::STANDBY) {
        position_controller::clear_target();
      }
      send_ack(req.type, req.seq, link_protocol::ack_result::OK);
      break;
    }
    case link_protocol::req::RESET_ODOMETRY: {
      // 外側タスクの積分と同時実行され得るため短く排他する。
      key = irq_lock();
      odometry::reset_origin(s.raw_x, s.raw_y, s.raw_yaw);
      irq_unlock(key);
      send_ack(req.type, req.seq, link_protocol::ack_result::OK);
      break;
    }
    case link_protocol::req::SUBSCRIBE: {
      if (n != 2) {
        send_ack(req.type, req.seq, link_protocol::ack_result::INVALID_PARAM);
        break;
      }
      const uint8_t et = pl[0];
      const uint8_t rate = pl[1];
      uint8_t result = link_protocol::ack_result::OK;
      key = irq_lock();
      if (et == link_protocol::evt::POSE) {
        if (rate >= 1 && rate <= 50) {
          subs_.pose_rate_hz = rate;
        } else if (rate == 0) {
          subs_.pose_rate_hz = 0;  // イベントのみ（周期送出なし）
        } else {
          result = link_protocol::ack_result::INVALID_PARAM;
        }
      } else if (et == link_protocol::evt::LOG) {
        subs_.log = true;
      } else if (et == link_protocol::evt::STATUS) {
        subs_.status = true;
      } else {
        result = link_protocol::ack_result::UNSUPPORTED;
      }
      irq_unlock(key);
      send_ack(req.type, req.seq, result);
      break;
    }
    case link_protocol::req::UNSUBSCRIBE: {
      if (n != 1) {
        send_ack(req.type, req.seq, link_protocol::ack_result::INVALID_PARAM);
        break;
      }
      key = irq_lock();
      if (pl[0] == link_protocol::evt::POSE) {
        subs_.pose_rate_hz = 0;
      } else if (pl[0] == link_protocol::evt::LOG) {
        subs_.log = false;
      } else if (pl[0] == link_protocol::evt::STATUS) {
        subs_.status = false;
      }
      irq_unlock(key);
      send_ack(req.type, req.seq, link_protocol::ack_result::OK);
      break;
    }
    case link_protocol::req::GET_STATUS: {
      uint8_t r[9];
      r[0] = req.seq;
      r[1] = mode;
      r[2] = s.enable ? 1 : 0;
      r[3] = s.estop ? 1 : 0;
      r[4] = s.imu_ok ? 1 : 0;
      link_protocol::write_u32(&r[5], now);
      send_frame(link_protocol::resp::STATUS, 0, r, sizeof(r));
      break;
    }
    default:
      send_ack(req.type, req.seq, link_protocol::ack_result::UNSUPPORTED);
      break;
  }
}

void link_session::send_frame(uint8_t type, uint8_t seq, const uint8_t* payload,
                              uint8_t len) {
  uint8_t buf[link_protocol::MAX_FRAME];
  const uint32_t n = link_protocol::encode(type, seq, payload, len, buf);
  (void)link_uart::write(buf, n);
}

void link_session::send_ack(uint8_t req_type, uint8_t seq, uint8_t result) {
  const uint8_t pl[3] = {req_type, seq, result};
  send_frame(link_protocol::resp::ACK, 0, pl, sizeof(pl));
}

void link_session::queue_frame(uint8_t type, const uint8_t* payload,
                               uint8_t len) {
  tx_item it{};
  it.len = static_cast<uint8_t>(
      link_protocol::encode(type, 0, payload, len, it.data));
  if (k_msgq_put(&tx_msgq_, &it, K_NO_WAIT) != 0) {
    const unsigned int key = irq_lock();
    ++stats_.tx_drops;
    irq_unlock(key);
  }
}

void link_session::clear_link_state_locked() {
  subs_ = {};
  mode_ = link_mode::STANDBY;
  vel_cmd_ = vec3{};
}

}  // namespace bb
