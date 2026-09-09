# --- front-matter:toml ---
model = "ball_balancing_omni3_multibody_lqi_custom_contact_fullplant.slx"
component = "ball_balancing_omni3_multibody_lqi_custom_contact_fullplant/MultibodyPlant"
[inputs]
T1 = "WheelTorque(1)"
T2 = "WheelTorque(2)"
T3 = "WheelTorque(3)"
[outputs]
Ground = "ContactStatus(1)"
Wheel1 = "ContactStatus(2)"
Wheel2 = "ContactStatus(3)"
Wheel3 = "ContactStatus(4)"
# --- end front-matter ---

Feature: custom contact full-plant の直立接触
  3輪トルクをゼロに固定した短時間実プラントで4接触を確認する。

Scenario: 直立短時間で4接触を維持する
  Given inputs
    * T1 = const(0)
    * T2 = const(0)
    * T3 = const(0)
  When simulate for 100ms in Normal mode
  Then outputs
    * GroundContact: Ground == 1 when t > 90ms
    * Wheel1Contact: Wheel1 == 1 when t > 90ms
    * Wheel2Contact: Wheel2 == 1 when t > 90ms
    * Wheel3Contact: Wheel3 == 1 when t > 90ms
