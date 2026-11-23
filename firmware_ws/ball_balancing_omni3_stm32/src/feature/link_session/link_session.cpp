#include "feature/link_session/link_session.hpp"

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>

#include "common/param_store/param_store.hpp"
#include "feature/odometry/odometry.hpp"
#include "feature/operation_manager/operation_manager.hpp"
#include "feature/position_controller/position_controller.hpp"
#include "feature/state_estimator/state_estimator.hpp"
#include "feature/tune_deadzone/tune_deadzone.hpp"
#include "peripheral/flash_store/flash_store.hpp"
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

  k_sem_init(&evt_sem_, 0, 1);
  next_flush_ms_ = k_uptime_get_32() + p_->link.event_batch_ms;
  k_poll_event_init(&poll_events_[0], K_POLL_TYPE_SEM_AVAILABLE,
                    K_POLL_MODE_NOTIFY_ONLY, link_uart::rx_sem());
  k_poll_event_init(&poll_events_[1], K_POLL_TYPE_SEM_AVAILABLE,
                    K_POLL_MODE_NOTIFY_ONLY, &evt_sem_);
}

void link_session::run_once() {
  // バッチ期限をタイムアウトとする。期限到達時は両イベント NOT_READY の
  // まま起床し、rx 排水をスキップしてフラッシュ判定のみ行う。
  (void)k_poll(poll_events_, 2, K_MSEC(p_->link.event_batch_ms));

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

  if (poll_events_[1].state == K_POLL_STATE_SEM_AVAILABLE) {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-union-access)
    (void)k_sem_take(poll_events_[1].sem, K_NO_WAIT);
    poll_events_[1].state = K_POLL_STATE_NOT_READY;
  }

  // バッチ期限到達 or 早期起床（閾値超過）でフラッシュ判定。
  flush_batch(k_uptime_get_32());
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

  const uint8_t op_state = operation_manager::state();
  const uint8_t calib = operation_manager::calib();

  // EVT_POSE: 購読中はレート分周で送出（バッチバッファへ投入）。
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
      queue_slot(pose_slot_, pose_dirty_, link_protocol::evt::POSE, pl,
                 sizeof(pl));
    }
  }

  // EVT_STATUS: 変化検出 + 100 ms ハートビート（バッチバッファへ投入）。
  // 先頭 6 B は従来互換で、7 B 以降は START 受理条件の可視化拡張
  // （ready_mask u8 + tilt f32）。tilt は連続変化するため変化検出は
  // 先頭 7 B のみで行い、拡張部はハートビート周期で更新される。
  if (subs.status) {
    const uint8_t ready = start_ready_mask(s);
    uint8_t cur[11] = {op_state, mode,
                       static_cast<uint8_t>(s.enable ? 1 : 0),
                       static_cast<uint8_t>(s.estop ? 1 : 0),
                       static_cast<uint8_t>(s.imu_ok ? 1 : 0), calib, ready};
    link_protocol::write_f32(&cur[7], s.tilt);
    const bool changed = memcmp(cur, prev_status_, 7) != 0;
    const bool heartbeat = (now_ms - last_status_tx_ms_) >= 100;
    if (changed || heartbeat) {
      memcpy(prev_status_, cur, sizeof(cur));
      last_status_tx_ms_ = now_ms;
      queue_slot(status_slot_, status_dirty_, link_protocol::evt::STATUS, cur,
                 sizeof(cur));
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

void link_session::notify_calib_result(uint8_t result, uint8_t seq,
                                       float gyro_bias_z) {
  uint8_t pl[6];
  pl[0] = seq;
  pl[1] = result;
  link_protocol::write_f32(&pl[2], gyro_bias_z);
  send_frame(link_protocol::resp::CALIB_RESULT, 0, pl, sizeof(pl));
}

// TUNE_RESULT: seq u8 | result u8 | onset f32x3 | sustain f32x3 | suggest f32x3
void link_session::notify_tune_result(uint8_t result, uint8_t seq,
                                      const tune_deadzone::measure& m) {
  uint8_t pl[2 + 36];
  pl[0] = seq;
  pl[1] = result;
  for (int i = 0; i < 3; ++i) {
    link_protocol::write_f32(&pl[2 + i * 4], m.onset[i]);
    link_protocol::write_f32(&pl[14 + i * 4], m.sustain[i]);
    link_protocol::write_f32(&pl[26 + i * 4], m.suggest[i]);
  }
  send_frame(link_protocol::resp::TUNE_RESULT, 0, pl, sizeof(pl));
}

void link_session::log(uint8_t level, const char* text) {
  const size_t tlen = strlen(text);
  const uint8_t n = (tlen > 60) ? 60 : static_cast<uint8_t>(tlen);
  uint8_t pl[1 + 60];
  pl[0] = level;
  memcpy(&pl[1], text, n);
  queue_log_frame(pl, static_cast<uint8_t>(n + 1));
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

  const uint8_t op_state = operation_manager::state();
  const bool operating = (op_state == operation_manager::op_state::OPERATING);

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
      if (!operating || mode != link_mode::VELOCITY) {
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
      if (!operating) {
        send_ack(req.type, req.seq, link_protocol::ack_result::REJECTED_STATE);
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
      if (!operating) {
        send_ack(req.type, req.seq, link_protocol::ack_result::REJECTED_STATE);
        break;
      }
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
      if (!operating) {
        send_ack(req.type, req.seq, link_protocol::ack_result::REJECTED_STATE);
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
    case link_protocol::req::START_OPERATION: {
      const operation_manager::start_cond c{s.imu_ok, s.estop, s.qualified,
                                            s.tilt};
      send_ack(req.type, req.seq, operation_manager::request_start(c));
      break;
    }
    case link_protocol::req::STOP_OPERATION: {
      // TUNING 中は掃引を中断（done 化）し、完了通知で STANDBY へ戻る。
      if (operation_manager::tuning()) {
        tune_deadzone::request_cancel();
      }
      operation_manager::request_stop();
      key = irq_lock();
      mode_ = link_mode::STANDBY;
      vel_cmd_ = vec3{};
      irq_unlock(key);
      position_controller::clear_target();
      send_ack(req.type, req.seq, link_protocol::ack_result::OK);
      break;
    }
    case link_protocol::req::SET_POSE: {
      if (n != 12) {
        send_ack(req.type, req.seq, link_protocol::ack_result::INVALID_PARAM);
        break;
      }
      if (op_state != operation_manager::op_state::STANDBY) {
        send_ack(req.type, req.seq, link_protocol::ack_result::REJECTED_STATE);
        break;
      }
      key = irq_lock();
      odometry::set_pose(link_protocol::read_f32(&pl[0]),
                         link_protocol::read_f32(&pl[4]),
                         link_protocol::read_f32(&pl[8]), s.raw_x, s.raw_y,
                         s.raw_yaw);
      irq_unlock(key);
      position_controller::clear_target();
      send_ack(req.type, req.seq, link_protocol::ack_result::OK);
      break;
    }
    case link_protocol::req::CALIBRATE_IMU: {
      send_ack(req.type, req.seq, operation_manager::request_calib(req.seq));
      break;
    }
    case link_protocol::req::SET_PARAM: {
      if (n != 5) {
        send_ack(req.type, req.seq, link_protocol::ack_result::INVALID_PARAM);
        break;
      }
      const bool ok = param_store::set_key(pl[0],
                                           link_protocol::read_f32(&pl[1]));
      send_ack(req.type, req.seq,
               ok ? link_protocol::ack_result::OK
                  : link_protocol::ack_result::INVALID_PARAM);
      // IMU 特性 key（0x50-0x52）は推定器の実行状態へも即時反映する。
      if (ok && pl[0] >= 0x50 && pl[0] <= 0x52) {
        const param_set& mp = param_store::params();
        state_estimator::set_bias(mp.imu.gyro_bias_z, mp.imu.ekf_bias_x,
                                  mp.imu.ekf_bias_y);
      }
      break;
    }
    case link_protocol::req::SAVE_PARAMS: {
      if (op_state != operation_manager::op_state::STANDBY) {
        send_ack(req.type, req.seq, link_protocol::ack_result::REJECTED_STATE);
        break;
      }
      {
        static uint32_t save_seq = 0;
        static uint8_t blob[param_store::BLOB_SIZE];
        param_store::build_blob(++save_seq, blob);
        // 旧 seq のスロットを選んで交互書き（seq % 2）。
        const uint32_t slot = save_seq % flash_store::SLOT_COUNT;
        const bool ok =
            flash_store::write_slot(slot, blob, param_store::BLOB_SIZE);
        send_ack(req.type, req.seq,
                 ok ? link_protocol::ack_result::OK
                    : link_protocol::ack_result::NOT_READY);
      }
      break;
    }
    case link_protocol::req::RESET_PARAMS: {
      if (op_state != operation_manager::op_state::STANDBY) {
        send_ack(req.type, req.seq, link_protocol::ack_result::REJECTED_STATE);
        break;
      }
      const bool ok = flash_store::erase_all();
      param_store::init_defaults();
      send_ack(req.type, req.seq,
               ok ? link_protocol::ack_result::OK
                  : link_protocol::ack_result::NOT_READY);
      break;
    }
    case link_protocol::req::TUNE_DEADZONE: {
      // dir i8 | max_duty f32 | ramp_s f32（省略時は既定値）
      if (n != 0 && n != 9) {
        send_ack(req.type, req.seq, link_protocol::ack_result::INVALID_PARAM);
        break;
      }
      const float dir = (n >= 1) ? static_cast<float>(
                                       static_cast<int8_t>(pl[0]))
                                 : 1.0F;
      const float max_duty = (n >= 9) ? link_protocol::read_f32(&pl[1]) : 0.25F;
      const float ramp_s = (n >= 9) ? link_protocol::read_f32(&pl[5]) : 15.0F;
      const uint8_t r = operation_manager::request_tune(req.seq);
      if (r == link_protocol::ack_result::OK) {
        (void)tune_deadzone::request_start(dir, max_duty, ramp_s);
      }
      send_ack(req.type, req.seq, r);
      break;
    }
    case link_protocol::req::GET_PARAMS: {
      // {start_key u8} → PARAMS {seq u8, total u8, count u8, [key u8, f32]*}
      const uint8_t start_key = (n >= 1) ? pl[0] : 0;
      uint8_t r[3 + 5 * 30];
      r[0] = req.seq;
      r[1] = static_cast<uint8_t>(param_store::KEY_COUNT);
      uint8_t count = 0;
      // key 昇順に start_key 以降を詰める。
      for (uint32_t k = 0; k < param_store::KEY_COUNT; ++k) {
        const uint8_t key_of_k = param_store::key_at(k);
        if (key_of_k < start_key) {
          continue;
        }
        float v = 0.0F;
        (void)param_store::get_key(key_of_k, &v);
        r[3 + count * 5] = key_of_k;
        link_protocol::write_f32(&r[3 + count * 5 + 1], v);
        ++count;
        if (count >= 30) {
          break;
        }
      }
      r[2] = count;
      send_frame(link_protocol::resp::PARAMS, 0, r,
                 static_cast<uint8_t>(3 + count * 5));
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
      // 11 B 以降は EVT_STATUS と同じ拡張部（ready_mask + tilt）。
      uint8_t r[16];
      r[0] = req.seq;
      r[1] = op_state;
      r[2] = mode;
      r[3] = s.enable ? 1 : 0;
      r[4] = s.estop ? 1 : 0;
      r[5] = s.imu_ok ? 1 : 0;
      r[6] = operation_manager::calib();
      link_protocol::write_u32(&r[7], now);
      r[11] = start_ready_mask(s);
      link_protocol::write_f32(&r[12], s.tilt);
      send_frame(link_protocol::resp::STATUS, 0, r, sizeof(r));
      break;
    }
    case link_protocol::req::TEST_ECHO: {
      uint8_t r[1 + link_protocol::MAX_DATA];
      r[0] = req.seq;
      memcpy(&r[1], pl, n);
      send_frame(link_protocol::resp::TEST_ECHO_RESP, 0, r,
                 static_cast<uint8_t>(n + 1));
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
  // link_uart::write は内部で排他を持たないため、リンクスレッドの応答と
  // 制御スレッドの CALIB_RESULT の競合・TX ISR との整合を irq_lock で
  // 保護する（コピーは最大 70B と短い）。
  const unsigned int key = irq_lock();
  (void)link_uart::write(buf, n);
  irq_unlock(key);
}

uint8_t link_session::start_ready_mask(const robot_state& s) {
  return static_cast<uint8_t>(
      (s.imu_ok ? link_protocol::ready_mask::IMU_OK : 0) |
      (!s.estop ? link_protocol::ready_mask::ESTOP_FREE : 0) |
      (s.qualified ? link_protocol::ready_mask::QUALIFIED : 0) |
      (s.tilt < p_->controller.start_tilt ? link_protocol::ready_mask::TILT_OK
                                          : 0));
}

void link_session::send_ack(uint8_t req_type, uint8_t seq, uint8_t result) {
  const uint8_t pl[3] = {req_type, seq, result};
  send_frame(link_protocol::resp::ACK, 0, pl, sizeof(pl));
}

void link_session::queue_slot(evt_slot& slot, bool& dirty, uint8_t type,
                              const uint8_t* payload, uint8_t len) {
  const unsigned int key = irq_lock();
  const uint32_t n = link_protocol::encode(type, 0, payload, len, slot.data);
  slot.len = static_cast<uint8_t>(n);
  dirty = true;
  pending_bytes_ += n;
  const bool early = pending_bytes_ >= FLUSH_THRESHOLD;
  irq_unlock(key);
  if (early) {
    k_sem_give(&evt_sem_);
  }
}

void link_session::queue_log_frame(const uint8_t* payload, uint8_t len) {
  const unsigned int key = irq_lock();
  bool early = false;
  if (log_count_ < LOG_FIFO_DEPTH) {
    evt_slot& slot = log_fifo_[(log_head_ + log_count_) % LOG_FIFO_DEPTH];
    const uint32_t n = link_protocol::encode(link_protocol::evt::LOG, 0,
                                             payload, len, slot.data);
    slot.len = static_cast<uint8_t>(n);
    ++log_count_;
    pending_bytes_ += n;
    early = pending_bytes_ >= FLUSH_THRESHOLD;
  } else {
    ++stats_.tx_drops;  // FIFO 満杯は新規ドロップして計数
  }
  irq_unlock(key);
  if (early) {
    k_sem_give(&evt_sem_);
  }
}

void link_session::flush_batch(uint32_t now_ms) {
  uint8_t buf[BATCH_BUF_SIZE];
  uint32_t n = 0;

  const unsigned int key = irq_lock();
  const bool due = (now_ms >= next_flush_ms_);
  if (pending_bytes_ == 0 || (!due && pending_bytes_ < FLUSH_THRESHOLD)) {
    // 送出対象なし、または期限未到来。
    if (due) {
      next_flush_ms_ = now_ms + p_->link.event_batch_ms;
    }
    irq_unlock(key);
    return;
  }
  if (pose_dirty_) {
    memcpy(&buf[n], pose_slot_.data, pose_slot_.len);
    n += pose_slot_.len;
    pose_dirty_ = false;
  }
  if (status_dirty_) {
    memcpy(&buf[n], status_slot_.data, status_slot_.len);
    n += status_slot_.len;
    status_dirty_ = false;
  }
  while (log_count_ > 0 && n + log_fifo_[log_head_].len <= sizeof(buf)) {
    memcpy(&buf[n], log_fifo_[log_head_].data, log_fifo_[log_head_].len);
    n += log_fifo_[log_head_].len;
    log_head_ = (log_head_ + 1) % LOG_FIFO_DEPTH;
    --log_count_;
  }
  pending_bytes_ = 0;
  next_flush_ms_ = now_ms + p_->link.event_batch_ms;
  irq_unlock(key);

  if (n > 0) {
    (void)link_uart::write(buf, n);
  }
}

void link_session::clear_link_state_locked() {
  subs_ = {};
  mode_ = link_mode::STANDBY;
  vel_cmd_ = vec3{};
}

}  // namespace bb
