#include "feature/operation_manager/operation_manager.hpp"

#include <zephyr/irq.h>

#include "common/link_protocol/link_protocol.hpp"

namespace bb {

void operation_manager::init(const config& cfg) {
  p_ = cfg.params;
  const unsigned int key = irq_lock();
  state_ = op_state::STANDBY;
  calib_ = calib_state::NONE;
  calib_seq_ = 0;
  calib_deadline_ms_ = 0;
  bias_reset_req_ = false;
  irq_unlock(key);
}

uint8_t operation_manager::state() {
  const unsigned int key = irq_lock();
  const uint8_t s = state_;
  irq_unlock(key);
  return s;
}

uint8_t operation_manager::calib() {
  const unsigned int key = irq_lock();
  const uint8_t c = calib_;
  irq_unlock(key);
  return c;
}

bool operation_manager::drives_motors() {
  const unsigned int key = irq_lock();
  const bool d = (state_ == op_state::OPERATING);
  irq_unlock(key);
  return d;
}

uint8_t operation_manager::request_start(const start_cond& c) {
  const unsigned int key = irq_lock();
  uint8_t result = link_protocol::ack_result::OK;
  if (state_ != op_state::STANDBY) {
    result = link_protocol::ack_result::REJECTED_STATE;
  } else if (!c.imu_ok || c.estop || !c.qualified ||
             c.tilt >= p_->controller.fallen_tilt) {
    result = link_protocol::ack_result::NOT_READY;
  } else {
    state_ = op_state::OPERATING;
  }
  irq_unlock(key);
  return result;
}

void operation_manager::request_stop() {
  const unsigned int key = irq_lock();
  // OPERATING からのみ STANDBY へ。FAULT は終端のため維持する。
  if (state_ == op_state::OPERATING) {
    state_ = op_state::STANDBY;
  }
  irq_unlock(key);
}

uint8_t operation_manager::request_calib(uint8_t seq) {
  const unsigned int key = irq_lock();
  uint8_t result = link_protocol::ack_result::OK;
  if (state_ != op_state::STANDBY) {
    result = link_protocol::ack_result::REJECTED_STATE;
  } else {
    calib_ = calib_state::RUNNING;
    calib_seq_ = seq;
    calib_deadline_ms_ = 0;  // 最初の calib_update が now で初期化する
    bias_reset_req_ = true;
  }
  irq_unlock(key);
  return result;
}

int8_t operation_manager::calib_update(uint32_t now_ms, bool qualified,
                                       bool imu_ok) {
  const unsigned int key = irq_lock();
  int8_t result = -1;
  if (calib_ == calib_state::RUNNING) {
    if (calib_deadline_ms_ == 0) {
      calib_deadline_ms_ = now_ms + p_->link.calib_timeout_ms;
    }
    if (!imu_ok) {
      calib_ = calib_state::FAILED;
      result = 2;
    } else if (qualified) {
      calib_ = calib_state::DONE;
      result = 0;
    } else if (now_ms >= calib_deadline_ms_) {
      calib_ = calib_state::FAILED;
      result = 1;
    }
  }
  irq_unlock(key);
  return result;
}

uint8_t operation_manager::calib_seq() {
  const unsigned int key = irq_lock();
  const uint8_t s = calib_seq_;
  irq_unlock(key);
  return s;
}

bool operation_manager::take_bias_reset_req() {
  const unsigned int key = irq_lock();
  const bool r = bias_reset_req_;
  bias_reset_req_ = false;
  irq_unlock(key);
  return r;
}

uint8_t operation_manager::request_tune(uint8_t seq) {
  const unsigned int key = irq_lock();
  uint8_t result = link_protocol::ack_result::OK;
  if (state_ != op_state::STANDBY || calib_ == calib_state::RUNNING) {
    result = link_protocol::ack_result::REJECTED_STATE;
  } else {
    state_ = op_state::TUNING;
    tune_seq_ = seq;
  }
  irq_unlock(key);
  return result;
}

void operation_manager::notify_tune_done(uint8_t result) {
  (void)result;  // result は link_session 側の TUNE_RESULT で返す
  const unsigned int key = irq_lock();
  if (state_ == op_state::TUNING) {
    state_ = op_state::STANDBY;
  }
  irq_unlock(key);
}

uint8_t operation_manager::tune_seq() {
  const unsigned int key = irq_lock();
  const uint8_t s = tune_seq_;
  irq_unlock(key);
  return s;
}

bool operation_manager::tuning() {
  const unsigned int key = irq_lock();
  const bool t = (state_ == op_state::TUNING);
  irq_unlock(key);
  return t;
}

void operation_manager::notify_controller_terminal() {
  const unsigned int key = irq_lock();
  if (state_ == op_state::OPERATING) {
    state_ = op_state::FAULT;
  }
  irq_unlock(key);
}

}  // namespace bb
