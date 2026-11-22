#include "manager/robot_application/robot_application.hpp"

#include <stdio.h>
#include <string.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>

namespace bb {

namespace {
#define PINS_NODE DT_NODELABEL(bb_pins)

// リンクスレッド: init 完了をセマフォで待ってから link_session の
// ポンプを回す。制御スレッド（main）より低優先度とし、受信処理が
// 1 kHz/200 Hz タスクをプリエンプトしないようにする。
// NOLINTBEGIN: Zephyr の K_SEM_DEFINE / K_THREAD_DEFINE マクロ内部の
// グローバル変数・初期化子に由来する警告
K_SEM_DEFINE(link_init_sem, 0, 1);
// NOLINTEND

void link_thread_entry(void* a, void* b, void* c) {
  ARG_UNUSED(a);
  ARG_UNUSED(b);
  ARG_UNUSED(c);
  (void)k_sem_take(&link_init_sem, K_FOREVER);
  for (;;) {
    link_session::run_once();
  }
}

// NOLINTNEXTLINE: 同上（スレッドオブジェクト・スタックの静的生成）。
K_THREAD_DEFINE(link_thread, 2048, link_thread_entry, NULL, NULL, NULL,
                K_PRIO_PREEMPT(7), 0, 0);

int status_cmd_cb(const struct shell* sh, size_t argc, char** argv) {
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);
  const telemetry_snapshot s = robot_application::snapshot();
  shell_print(sh, "uptime_ms=%lu mode=%u imu_ok=%d enable=%d estop_enable=%d",
              static_cast<unsigned long>(timebase::millis()),
              static_cast<unsigned>(s.ctrl.mode), static_cast<int>(s.imu_ok),
              static_cast<int>(s.ctrl.enable), static_cast<int>(s.enable));
  shell_print(
      sh, "euler_rad=[%.4f %.4f %.4f] wheel_rate=[%.3f %.3f %.3f]",
      static_cast<double>(s.est.euler[0]), static_cast<double>(s.est.euler[1]),
      static_cast<double>(s.est.euler[2]), static_cast<double>(s.wheel_rate[0]),
      static_cast<double>(s.wheel_rate[1]),
      static_cast<double>(s.wheel_rate[2]));
  shell_print(sh, "torque=[%.3f %.3f %.3f] vel_integral=[%.3f %.3f]",
              static_cast<double>(s.ctrl.attitude_torque[0]),
              static_cast<double>(s.ctrl.attitude_torque[1]),
              static_cast<double>(s.ctrl.attitude_torque[2]),
              static_cast<double>(s.ctrl.velocity_integral[0]),
              static_cast<double>(s.ctrl.velocity_integral[1]));
  shell_print(sh,
              "yaw_bias_ready=%d settle_witness=%d outer_idx=%lu tick=%lu "
              "overrun=%lu",
              static_cast<int>(s.ctrl.yaw_bias_ready),
              static_cast<int>(s.ctrl.settle_witness),
              static_cast<unsigned long>(s.outer_index),
              static_cast<unsigned long>(s.tick_count),
              static_cast<unsigned long>(s.overrun_count));
  shell_print(sh, "max_inner_us=%lu max_outer_us=%lu telem=%d",
              static_cast<unsigned long>(s.max_inner_us),
              static_cast<unsigned long>(s.max_outer_us),
              static_cast<int>(robot_application::telemetry_enabled()));
  return 0;
}

int telem_cmd_cb(const struct shell* sh, size_t argc, char** argv) {
  if (argc < 2) {
    shell_print(sh, "telem=%d",
                static_cast<int>(robot_application::telemetry_enabled()));
    return 0;
  }
  const bool on = strcmp(argv[1], "on") == 0;
  if (!on && strcmp(argv[1], "off") != 0) {
    shell_error(sh, "usage: bb telem [on|off]");
    return -EINVAL;
  }
  robot_application::set_telemetry(on);
  shell_print(sh, "telem=%d", static_cast<int>(on));
  return 0;
}

int link_cmd_cb(const struct shell* sh, size_t argc, char** argv) {
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);
  const link_session::stats s = robot_application::link_stats();
  shell_print(sh,
              "rx_frames=%lu crc_drops=%lu rx_overruns=%lu tx_drops=%lu "
              "last_rx_ms=%lu now_ms=%lu",
              static_cast<unsigned long>(s.rx_frames),
              static_cast<unsigned long>(s.rx_crc_drops),
              static_cast<unsigned long>(s.rx_overruns),
              static_cast<unsigned long>(s.tx_drops),
              static_cast<unsigned long>(s.last_rx_ms),
              static_cast<unsigned long>(timebase::millis()));
  return 0;
}

// NOLINTBEGIN: Zephyr の SHELL_* マクロ内部の初期化子・分岐に由来する警告
SHELL_STATIC_SUBCMD_SET_CREATE(
    SUB_BB, SHELL_CMD(status, NULL, "show controller status", status_cmd_cb),
    SHELL_CMD(telem, NULL, "telemetry CSV stream on/off", telem_cmd_cb),
    SHELL_CMD(link, NULL, "robot link stats", link_cmd_cb),
    SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(bb, &SUB_BB, "ball balancing robot commands", NULL);
// NOLINTEND

}  // namespace

bool robot_application::init(const config& cfg) {
  p_ = cfg.params;
  imu_accel_axis_map_ = cfg.imu_accel_axis_map;
  imu_gyro_axis_map_ = cfg.imu_gyro_axis_map;

  // peripheral
  led_pin_.init({GPIO_DT_SPEC_GET(PINS_NODE, led_gpios), true});
  estop_ext_pin_.init({GPIO_DT_SPEC_GET(PINS_NODE, estop_gpios)});
  estop_btn_pin_.init({GPIO_DT_SPEC_GET(PINS_NODE, estop_btn_gpios)});
  pwm_a_[0].init({PWM_DT_SPEC_GET_BY_IDX(PINS_NODE, 0)});
  pwm_b_[0].init({PWM_DT_SPEC_GET_BY_IDX(PINS_NODE, 1)});
  pwm_a_[1].init({PWM_DT_SPEC_GET_BY_IDX(PINS_NODE, 2)});
  pwm_b_[1].init({PWM_DT_SPEC_GET_BY_IDX(PINS_NODE, 3)});
  pwm_a_[2].init({PWM_DT_SPEC_GET_BY_IDX(PINS_NODE, 4)});
  pwm_b_[2].init({PWM_DT_SPEC_GET_BY_IDX(PINS_NODE, 5)});
  enc_count_[0].init(
      {reinterpret_cast<TIM_TypeDef*>(DT_REG_ADDR(DT_NODELABEL(timers2)))});
  enc_count_[1].init(
      {reinterpret_cast<TIM_TypeDef*>(DT_REG_ADDR(DT_NODELABEL(timers1)))});
  enc_count_[2].init(
      {reinterpret_cast<TIM_TypeDef*>(DT_REG_ADDR(DT_NODELABEL(timers3)))});

  // device
  imu_ok_ = lsm6dso::init(
      {DEVICE_DT_GET(DT_ALIAS(imu0)), imu_accel_axis_map_, imu_gyro_axis_map_});
  motors_[0].init({&pwm_a_[0], &pwm_b_[0]});
  motors_[1].init({&pwm_a_[1], &pwm_b_[1]});
  motors_[2].init({&pwm_a_[2], &pwm_b_[2]});
  encoders_[0].init({&enc_count_[0], p_->encoder.quadrature_factor,
                     p_->encoder.counts_per_rev});
  encoders_[1].init({&enc_count_[1], p_->encoder.quadrature_factor,
                     p_->encoder.counts_per_rev});
  encoders_[2].init({&enc_count_[2], p_->encoder.quadrature_factor,
                     p_->encoder.counts_per_rev});
  (void)estop_input::init({&estop_ext_pin_, &estop_btn_pin_});
  status_led::init({&led_pin_});
  status_led::set(imu_ok_ ? status_led::pattern::SLOW_BLINK
                          : status_led::pattern::FAST_BLINK);

  // feature
  wheel_speed_meter::init({{&encoders_[0], &encoders_[1], &encoders_[2]},
                           p_->controller.sample_time});
  state_estimator::init({p_});
  balance_controller::init({p_});
  wheel_servo::init({p_});
  command_profile::init({p_});
  odometry::init();
  position_controller::init({p_});
  link_session::init({p_});
  k_sem_give(&link_init_sem);

  // manager
  task_scheduler::init();
  control_tick::init();
  control_tick::start();
  return imu_ok_;
}

void robot_application::run() {
  const task_scheduler::hooks hooks = {&inner_task_cb, &outer_task_cb,
                                       &diag_task_cb};
  task_scheduler::run(hooks);
}

void robot_application::outer_task_cb() {
  const float dt = p_->controller.sample_time;
  const lsm6dso::sample imu_sample = lsm6dso::read();
  wheel_rate_ = wheel_speed_meter::update();
  est_out_ = state_estimator::step(imu_sample, wheel_rate_, dt);
  imu_ok_ = imu_ok_ && imu_sample.valid;

  // オドメトリ: 推定器のワールド速度を積分し、オフセット適用済の
  // pose を得る（FR-10/11）。
  odometry::update(est_out_.velocity_world2[0], est_out_.velocity_world2[1],
                   dt);
  const odometry::pose pose = odometry::get();
  const float pose_yaw = odometry::yaw_of(est_out_.euler[2]);
  const odometry::pose raw = odometry::raw();

  enable_ = !estop_input::requested() && imu_ok_;

  // 指令選択: UART リンクが一度でも確立していれば link_session の
  // モード/指令を優先し、未接続なら従来の command_profile を使う（FR-09）。
  const link_session::command_output lc = link_session::command();
  vec3 cmd{};
  bool cmd_enable = true;
  if (!lc.link_active) {
    const command_profile::output cp =
        command_profile::update(static_cast<float>(timebase::millis()) * 1e-3F);
    cmd = cp.command;
    cmd_enable = cp.enable;
  } else if (lc.mode == link_mode::VELOCITY) {
    cmd = lc.velocity_cmd;
  } else if (lc.mode == link_mode::POSITION) {
    cmd = position_controller::update(pose.x, pose.y, dt).command;
  }
  // STANDBY は零指令のまま。
  ctrl_out_ = balance_controller::step(est_out_, cmd, enable_ && cmd_enable,
                                       wheel_rate_);

  // リンク向け状態メールボックス更新＋イベント生成（FR-11）。
  const link_session::robot_state rs{enable_,
                                     estop_input::requested(),
                                     imu_ok_,
                                     est_out_.updated_qualification_time >=
                                         p_->estimator.bias_qualification_time,
                                     raw.x,
                                     raw.y,
                                     est_out_.euler[2],
                                     pose.x,
                                     pose.y,
                                     pose_yaw,
                                     est_out_.velocity_world2[0],
                                     est_out_.velocity_world2[1],
                                     est_out_.gyro_body[2]};
  link_session::publish_state(rs);
  link_session::generate_events(timebase::millis());

  ++outer_index_;
}

void robot_application::inner_task_cb() {
  wheel_servo::input in{};
  in.attitude_torque = ctrl_out_.attitude_torque;
  in.wheel_rate_command = ctrl_out_.wheel_rate_command;
  in.settle_witness = ctrl_out_.settle_witness;
  in.enable = ctrl_out_.enable;
  in.wheel_rate = wheel_speed_meter::rate();
  in.mode = ctrl_out_.mode;
  servo_out_ = wheel_servo::step(in);
  for (int i = 0; i < 3; ++i) {
    motors_[i].set_duty(servo_out_.duty[i]);
  }
}

link_session::stats robot_application::link_stats() {
  return link_session::get_stats();
}

telemetry_snapshot robot_application::snapshot() {
  telemetry_snapshot s{};
  // 制御スレッド（1 kHz タスク）とプリエンプションし得るため、
  // 短い irq クリティカル区間で整合コピーを取る。
  const unsigned int key = irq_lock();
  s.ctrl = ctrl_out_;
  s.est = est_out_;
  s.wheel_rate = wheel_rate_;
  s.enable = enable_;
  s.imu_ok = imu_ok_;
  s.outer_index = outer_index_;
  s.tick_count = task_scheduler::tick_count();
  s.overrun_count = task_scheduler::overrun_count();
  s.max_inner_us = task_scheduler::max_inner_us();
  s.max_outer_us = task_scheduler::max_outer_us();
  irq_unlock(key);
  return s;
}

void robot_application::diag_task_cb() {
  status_led::update(timebase::millis());
  if (ctrl_out_.mode == 0) {
    status_led::set(status_led::pattern::OFF);
  } else if (ctrl_out_.mode == 2) {
    status_led::set(status_led::pattern::FAST_BLINK);
  }
  if (!telem_enabled_) return;
  char line[220];
  const int n = snprintf(
      line, sizeof(line),
      "%lu,%u,%d,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
      "%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.4f\r\n",
      static_cast<unsigned long>(timebase::millis()),
      static_cast<unsigned>(ctrl_out_.mode),
      static_cast<int>(ctrl_out_.yaw_bias_ready),
      static_cast<double>(est_out_.euler[0]),
      static_cast<double>(est_out_.euler[1]),
      static_cast<double>(est_out_.euler[2]),
      static_cast<double>(est_out_.gyro_body[0]),
      static_cast<double>(est_out_.gyro_body[1]),
      static_cast<double>(est_out_.gyro_body[2]),
      static_cast<double>(est_out_.velocity_world2[0]),
      static_cast<double>(est_out_.velocity_world2[1]),
      static_cast<double>(est_out_.ball_rate_world[0]),
      static_cast<double>(est_out_.ball_rate_world[1]),
      static_cast<double>(wheel_rate_[0]), static_cast<double>(wheel_rate_[1]),
      static_cast<double>(wheel_rate_[2]),
      static_cast<double>(ctrl_out_.attitude_torque[0]),
      static_cast<double>(ctrl_out_.attitude_torque[1]),
      static_cast<double>(ctrl_out_.attitude_torque[2]),
      static_cast<double>(est_out_.gyro_bias_z));
  debug_uart::write(line, static_cast<uint32_t>(n));
}

}  // namespace bb
