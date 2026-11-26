#include "manager/robot_application/robot_application.hpp"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/printk.h>

#include "common/param_store/param_store.hpp"
#include "feature/tune_deadzone/tune_deadzone.hpp"
#include "peripheral/flash_store/flash_store.hpp"

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
  shell_print(sh,
              "uptime_ms=%lu op_state=%u calib_state=%u mode=%u imu_ok=%d "
              "enable=%d estop_enable=%d",
              static_cast<unsigned long>(timebase::millis()),
              static_cast<unsigned>(operation_manager::state()),
              static_cast<unsigned>(operation_manager::calib()),
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

int start_cmd_cb(const struct shell* sh, size_t argc, char** argv) {
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);
  const telemetry_snapshot s = robot_application::snapshot();
  const bool qualified = s.est.updated_qualification_time >=
                         get_params().estimator.bias_qualification_time;
  const float tilt = sqrtf(s.est.euler[0] * s.est.euler[0] +
                           s.est.euler[1] * s.est.euler[1]);
  const operation_manager::start_cond c{s.imu_ok, estop_input::requested(),
                                        qualified, tilt};
  const uint8_t r = operation_manager::request_start(c);
  shell_print(sh, "start result=%u op_state=%u", static_cast<unsigned>(r),
              static_cast<unsigned>(operation_manager::state()));
  return (r == 0) ? 0 : -EPERM;
}

int stop_cmd_cb(const struct shell* sh, size_t argc, char** argv) {
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);
  operation_manager::request_stop();
  shell_print(sh, "op_state=%u",
              static_cast<unsigned>(operation_manager::state()));
  return 0;
}

int calib_cmd_cb(const struct shell* sh, size_t argc, char** argv) {
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);
  const uint8_t r = operation_manager::request_calib(0);
  shell_print(sh, "calib result=%u calib_state=%u", static_cast<unsigned>(r),
              static_cast<unsigned>(operation_manager::calib()));
  return (r == 0) ? 0 : -EPERM;
}

int param_cmd_cb(const struct shell* sh, size_t argc, char** argv) {
  // bb param            : 調整対象キーを全表示
  // bb param <key>      : 1 キー表示
  // bb param <key> <v>  : 設定（RAM のみ、bb save で不揮発化）
  if (argc == 1) {
    for (uint32_t i = 0; i < param_store::KEY_COUNT; ++i) {
      const uint8_t k = param_store::key_at(i);
      float v = 0.0F;
      (void)param_store::get_key(k, &v);
      shell_print(sh, "0x%02X = %.6f", static_cast<unsigned>(k),
                  static_cast<double>(v));
    }
    return 0;
  }
  const long key = strtol(argv[1], nullptr, 0);
  if (argc == 2) {
    float v = 0.0F;
    if (!param_store::get_key(static_cast<uint8_t>(key), &v)) {
      shell_error(sh, "unknown key 0x%02lX", key);
      return -EINVAL;
    }
    shell_print(sh, "0x%02lX = %.6f", key, static_cast<double>(v));
    return 0;
  }
  const float v = strtof(argv[2], nullptr);
  if (!param_store::set_key(static_cast<uint8_t>(key), v)) {
    shell_error(sh, "set rejected (unknown key or out of range)");
    return -EINVAL;
  }
  shell_print(sh, "0x%02lX = %.6f (volatile; 'bb save' to persist)", key,
              static_cast<double>(v));
  return 0;
}

int save_cmd_cb(const struct shell* sh, size_t argc, char** argv) {
  ARG_UNUSED(argc);
  ARG_UNUSED(argv);
  if (operation_manager::state() != operation_manager::op_state::STANDBY) {
    shell_error(sh, "save allowed in standby only");
    return -EPERM;
  }
  static uint8_t blob[param_store::BLOB_SIZE];
  const uint32_t save_seq = param_store::next_save_seq();
  param_store::build_blob(save_seq, blob);
  const bool ok = flash_store::write_slot(
      save_seq % flash_store::SLOT_COUNT, blob, param_store::BLOB_SIZE);
  shell_print(sh, "save %s (seq=%lu)", ok ? "ok" : "failed",
              static_cast<unsigned long>(save_seq));
  return ok ? 0 : -EIO;
}

int tune_cmd_cb(const struct shell* sh, size_t argc, char** argv) {
  // bb tune [dir] [max_duty] [ramp_s]
  float dir = 1.0F;
  float max_duty = 0.25F;
  float ramp_s = 15.0F;
  if (argc >= 2) dir = strtof(argv[1], nullptr);
  if (argc >= 3) max_duty = strtof(argv[2], nullptr);
  if (argc >= 4) ramp_s = strtof(argv[3], nullptr);
  const uint8_t r = operation_manager::request_tune(0);
  if (r != 0) {
    shell_error(sh, "tune rejected result=%u", static_cast<unsigned>(r));
    return -EPERM;
  }
  (void)tune_deadzone::request_start(dir, max_duty, ramp_s);
  shell_print(sh, "tune started: wheels unloaded, hold the chassis");
  return 0;
}

// NOLINTBEGIN: Zephyr の SHELL_* マクロ内部の初期化子・分岐に由来する警告
SHELL_STATIC_SUBCMD_SET_CREATE(
    SUB_BB, SHELL_CMD(status, NULL, "show controller status", status_cmd_cb),
    SHELL_CMD(telem, NULL, "telemetry CSV stream on/off", telem_cmd_cb),
    SHELL_CMD(link, NULL, "robot link stats", link_cmd_cb),
    SHELL_CMD(start, NULL, "start operation (standby -> operating)",
              start_cmd_cb),
    SHELL_CMD(stop, NULL, "stop operation (-> standby)", stop_cmd_cb),
    SHELL_CMD(calib, NULL, "calibrate IMU gyro bias (standby only)",
              calib_cmd_cb),
    SHELL_CMD(param, NULL, "param [key] [value] get/set tunable params",
              param_cmd_cb),
    SHELL_CMD(save, NULL, "persist params to flash (standby only)",
              save_cmd_cb),
    SHELL_CMD(tune, NULL, "tune [dir] [max_duty] [ramp_s] deadzone sweep",
              tune_cmd_cb),
    SHELL_SUBCMD_SET_END);
SHELL_CMD_REGISTER(bb, &SUB_BB, "ball balancing robot commands", NULL);
// NOLINTEND

}  // namespace

bool robot_application::init(const config& cfg) {
  // FR-15: 既定値で動作パラメータを初期化し、保存 blob があれば上書き。
  // get_params() は param_store::params() を返すため、cfg.params 以降の
  // 全コンポーネントが可変パラメータを共有する。
  param_store::init_defaults();
  {
    static uint8_t blob[param_store::BLOB_SIZE];
    uint32_t seq = 0;
    if (flash_store::load_latest(blob, sizeof(blob), &seq) &&
        param_store::parse_blob(blob, sizeof(blob), nullptr)) {
      // command.* は実行時に変更する手段がないため、過去の既定値で書かれた
      // blob の値は採用せずコンパイル時既定へ戻す。
      param_store::mutable_params().command = param_set{}.command;
      char line[64];
      const int n = snprintf(line, sizeof(line),
                             "params: restored from flash (seq=%lu)\r\n",
                             static_cast<unsigned long>(seq));
      debug_uart::write(line, static_cast<uint32_t>(n));
    }
  }
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
  // FR-15: 保存済み IMU 特性を推定器へ反映（初期値として使用し、
  // バイアス学習は起動後も継続される）。
  state_estimator::set_bias(p_->imu.gyro_bias_z, p_->imu.ekf_bias_x,
                            p_->imu.ekf_bias_y);
  balance_controller::init({p_});
  wheel_servo::init({p_});
  command_profile::init({p_});
  odometry::init();
  position_controller::init({p_});
  operation_manager::init({p_});
  tune_deadzone::init({p_});
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

  // STANDBY → OPERATING エッジ（FR-12）: 待機中に蓄積した滞留状態を
  // 制御開始へ持込まない。推定器の動的状態はリセットして次の静止
  // サンプルで姿勢を再シードし（受理時点の資格は保持、yaw も保持）、
  // オドメトリは SET_POSE/RESET_ODOMETRY の宣言位置を現在の生値へ
  // 再適用する。step() より前に行い、最初の OPERATING ティックから
  // リセット済みの推定値を使う。さらにオドメトリの settle 保持を武装し、
  // ボールへの配置トランジェント（手振れ・ボール転がり等）に由来する
  // 幻影速度を開始直後の積分へ持ち込まない。
  const bool operating = operation_manager::drives_motors();
  if (operating && !prev_operating_) {
    balance_controller::reset();
    state_estimator::reset_estimate(true);
    const odometry::pose raw0 = odometry::raw();
    odometry::reapply_declared(raw0.x, raw0.y, est_out_.euler[2]);
    odometry::arm_settle_hold();
  }
  prev_operating_ = operating;

  const lsm6dso::sample imu_sample = lsm6dso::read();
  wheel_rate_ = wheel_speed_meter::update();
  est_out_ = state_estimator::step(imu_sample, wheel_rate_, dt);
  imu_ok_ = imu_ok_ && imu_sample.valid;

  // オドメトリ（FR-10/11）。動作中は推定器のワールド速度を積分する
  // （開始直後の settle 保持と速度包絡クランプは odometry 側が担う）、
  // 待機中は車輪が動かないため IMU 専用の慣性オドメトリで追跡する
  // （検証用途。動作開始時に宣言位置へ再アンカーされる）。
  if (operating) {
    odometry::update(est_out_.velocity_world2[0], est_out_.velocity_world2[1],
                     est_out_.still, dt);
  } else if (imu_sample.valid) {
    odometry::update_inertial(est_out_.accel_world[0], est_out_.accel_world[1],
                              est_out_.still, dt);
  }
  const odometry::pose pose = odometry::get();
  const float pose_yaw = odometry::yaw_of(est_out_.euler[2]);
  const odometry::pose raw = odometry::raw();
  // EVT_POSE の速度表示も待機中は慣性チャネルを使う。
  const odometry::pose iv = odometry::inertial_velocity();
  const float vx = operating ? est_out_.velocity_world2[0] : iv.x;
  const float vy = operating ? est_out_.velocity_world2[1] : iv.y;

  // E-Stop 要求はワンショット消費型フラグのため、観測した要求は非
  // STANDBY 中ラッチして保持する（他消費者＝掃引中断・状態報告への
  // 取りこぼし防止）。STANDBY では毎ティック解除し、単発押下を持ち越さない。
  if (operation_manager::state() == operation_manager::op_state::STANDBY) {
    estop_latch_ = false;
  }
  const bool estop = estop_input::requested() || estop_latch_;
  estop_latch_ = estop;
  enable_ = !estop && imu_ok_;

  const uint32_t now_ms = timebase::millis();
  const bool qualified = est_out_.updated_qualification_time >=
                         p_->estimator.bias_qualification_time;
  const float tilt = sqrtf(est_out_.euler[0] * est_out_.euler[0] +
                           est_out_.euler[1] * est_out_.euler[1]);

  // 動作状態管理（FR-12/FR-13）。キャリブレーションの推定器リセットと
  // 完了報告をここで処理する。
  if (operation_manager::take_bias_reset_req()) {
    state_estimator::reset_bias();
  }
  const int8_t cr =
      operation_manager::calib_update(now_ms, qualified, imu_ok_);
  if (cr >= 0) {
    link_session::notify_calib_result(static_cast<uint8_t>(cr),
                                      operation_manager::calib_seq(),
                                      est_out_.gyro_bias_z);
    // FR-15: キャリブレーション完了時は学習済み IMU 特性を
    // 動作パラメータへ反映する（SAVE_PARAMS で不揮発化される）。
    if (cr == 0) {
      param_set& mp = param_store::mutable_params();
      state_estimator::get_bias(&mp.imu.gyro_bias_z, &mp.imu.ekf_bias_x,
                                &mp.imu.ekf_bias_y);
    }
  }

  // FR-17: 同定セッションの完了検出 → TUNE_RESULT 送出 → STANDBY 復帰。
  if (operation_manager::tuning() && tune_deadzone::done()) {
    link_session::notify_tune_result(tune_deadzone::done_result(),
                                     operation_manager::tune_seq(),
                                     tune_deadzone::result());
    tune_deadzone::consume_done();
    operation_manager::notify_tune_done(0);
  }

  if (operating) {
    // 指令選択: UART リンクが一度でも確立していれば link_session の
    // モード/指令を優先し、未接続なら従来の command_profile を使う（FR-09）。
    const link_session::command_output lc = link_session::command();
    vec3 cmd{};
    bool cmd_enable = true;
    if (!lc.link_active) {
      const command_profile::output cp = command_profile::update(
          static_cast<float>(timebase::millis()) * 1e-3F);
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
    // OPERATING 中の mode 0 到達は終端 fault（FR-12）。
    if (ctrl_out_.mode == 0) {
      operation_manager::notify_controller_terminal();
    }
  } else {
    // STANDBY/FAULT は制御器を回さずモータ無駆動（mode=0, enable=false）。
    ctrl_out_ = balance_controller::output{};
  }

  // リンク向け状態メールボックス更新＋イベント生成（FR-11, FR-18）。
  const link_session::robot_state rs{enable_,
                                     estop,
                                     imu_ok_,
                                     qualified,
                                     tilt,
                                     raw.x,
                                     raw.y,
                                     est_out_.euler[2],
                                     pose.x,
                                     pose.y,
                                     pose_yaw,
                                     vx,
                                     vy,
                                     est_out_.gyro_body[2],
                                     imu_sample.specific_force,
                                     imu_sample.gyro,
                                     {static_cast<int32_t>(enc_count_[0].count()),
                                      static_cast<int32_t>(enc_count_[1].count()),
                                      static_cast<int32_t>(enc_count_[2].count())}};
  link_session::publish_state(rs);
  link_session::generate_events(now_ms);

  ++outer_index_;
}

void robot_application::inner_task_cb() {
  // FR-17: TUNING 中はサーボを回さず掃引セッションの duty を直接適用。
  if (operation_manager::tuning()) {
    // outer で先に消費された E-Stop 要求もラッチ経由で拾う
    // （inner 側の観測もラッチへ反映し、状態報告へ残す）。
    const bool estop = estop_input::requested() || estop_latch_;
    estop_latch_ = estop;
    const tune_deadzone::output td = tune_deadzone::step(
        wheel_speed_meter::rate(), estop, task_scheduler::tick_count());
    for (int i = 0; i < 3; ++i) {
      motors_[i].set_duty(td.duty[i]);
    }
    return;
  }
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
  } else {
    // mode 1（通常バランス中）は常時点灯で回復モード（mode 2）と区別する。
    status_led::set(status_led::pattern::ON);
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
