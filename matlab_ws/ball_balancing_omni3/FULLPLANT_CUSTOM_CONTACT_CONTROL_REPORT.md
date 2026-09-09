# Custom contact full-plant 制御設計・検証報告

## 結論

対象は `ball_balancing_omni3_multibody_lqi_custom_contact_fullplant.slx` である。実 Simscape Multibody プラント上の階層型速度・姿勢制御により、X、Y、XY の有限時間試験で倒立と4接触を維持した。X/Y のクロス速度は単独指令成分の約3–7.5%であり、今回の低速域では独立SISO構造で十分と判断した。

## Custom contact

3輪すべてで `FrictionType=ProvidedByInput`、`SenseTangentialVelocity=on`、`SenseNormalForceMagnitude=on`、`SenseContactFollowerRotation=on` を確認した。実配線は各輪で `vt_out / Rf_out / fnm_out → ballbotCustomFriction → ffrxy_in` である。単位は順に `m/s`、無次元9要素回転、`N`、出力は `N` の2要素である。

`Rf_out` は contact 座標から follower wheel 座標への回転である。wheel 座標の axle `[0 0 1]` と接触法線から drive 軸を算出し、`Rf'` で contact 座標へ戻す。実測回転は3輪とも直交誤差 `5e-15` 以下、行列式1であった。drive/roller 飽和力比は `0.75/0.02=37.5`、非接触入力の摩擦力はゼロ、実試験の最大 `F・v` は0で正の仕事をしなかった。

0.1 s 静止試験は4接触、最小法線力 `[1.9437 1.9437 1.9437] N`、終了法線力の3輪差 `1.1e-6 N`、最大 slip `0.00413 m/s`、最大摩擦力 `0.04272 N` であった。

## Operating Point と設計モデル

現在の custom-contact full-plant から0.1 s snapshot Operating Pointを再生成した。既定 `operspec/findop` の厳密 trim は実行可能解を得ず、最大状態微分は `0.016167` だった。snapshot 再適用後の残差指標は `0.005611`、roll/pitch は `[5.28e-5, -6.84e-5] deg`、4接触を再現した。この残差と厳密 trim 不成立は remaining risk とする。

解析線形化境界は wheel-speed command の直後から、`speed PI → DC motor → wheels → custom contact → ball → rover → estimator` を含む feedback state までとした。解析結果は0次・全ゼロで、出力引数付き `advisorResult=advise(info.Advisor)` は `SpeedPiAndDcMotor` のゼロI/O対を特定したため不採用とした。

代替同定は同じ実 full-plant に `±0.001/0.002/0.004 m/s` 相当のXパルスを与えた。sample timeは `0.005 s`、状態は `[vx, pitch, pitchRate, equivalentWheelSpeedX]`、同定時 torque limit は `0.002 N m` で全接触を維持した。one-step fit は `[62.3, 90.9, 63.1, 62.9]%`、離散極は複素対に加えて `1.047` を含み、直立不安定性を示した。近似関数 `ballbotReducedPlantDynamics.m` は使用していない。

## Controller

採用構造は `velocity reference → tilt reference → tilt PD → wheel-speed command → speed PI/DC motor` である。sample timeは `0.005 s`、velocity `Kp=[2.3 2.3]`、`Ki=[0 0]`、tilt `Kp=[16 16]`、`Kd=[1.6 1.6]`、command scaleは0.60である。位置積分はない。速度誤差積分状態は実装上保持するが、採用調整では `Ki=0` とした。

制約は motor torque `0.010 N m/輪`、wheel speed `[14.6241 14.6241 14.6241] rad/s`、tilt reference 4 deg、速度・加速度制限、飽和時の積分保持と speed PI back-calculation である。想定帯域は約1–5 Hzである。speed PI は nominal gain の0.25倍とした。torque limit を `0.015 N m` へ上げると接触喪失し、`0.020 N m` では転倒したため、0.010を採用した。

同定モデルからLQRと速度誤差のみを積分するLQIも算出し、Q/R、gain、閉ループ極を `fullplant_controller_design.mat` に保存した。解析線形化が全ゼロで局所同定fitも限定的なため正式採用せず、非線形 full-plant で合格した階層型を採用した。旧LQIゲインは使用していない。

## 実測結果

指令は0.5 sで投入し、2.5 sでゼロへ戻し、StopTimeは4 sとした。

| ケース | 定速区間速度 `[vx vy]` m/s | 停止後速度 `[vx vy]` m/s | 最大傾斜 deg | 最大wheel command rad/s | 最小法線力 N |
|---|---:|---:|---:|---:|---:|
| zero | `[-0.00010 0.00007]` | `[0.00020 -0.00012]` | 0.039 | 11.466 | `[0.905 1.944 0.903]` |
| X | `[0.02268 0.00009]` | `[-0.00261 -0.00120]` | 0.319 | 14.624 | `[0.285 0.610 0.903]` |
| Y | `[0.00008 0.02258]` | `[0.00031 -0.00062]` | 0.277 | 11.466 | `[0.905 1.028 0.903]` |
| XY | `[0.02112 0.02145]` | `[-0.00056 -0.00051]` | 0.363 | 13.447 | `[0.905 1.199 0.695]` |

全ケースで転倒なし、4接触維持、normal force非負、最大motor torque `0.010 N m` であった。X指令→Y速度は `0.00009 m/s`、Y指令→X速度は `0.00008 m/s` である。XY同時誤差は `[+0.00112, +0.00145] m/s` だった。3輪の法線力・slip差は存在するが接触喪失には至らなかった。

## 回帰と再現

`tests/run_fullplant_regression.m` は model update、upright、X、Y、XY を有限 `SimulationInput` で実行し、倒立、接触、normal force、torque、追従、クロス応答を数値 assertion する。Simulink Test API は環境に製品がなく実行できなかったため、実 full-plant を直接 simulation する回帰へフォールバックした。`reproduce_fullplant_control.m` は Operating Point 再取得と回帰結果再生成を行う。

## Remaining risks

- 厳密 trim は未成立で、snapshot Operating Point の残差が残る。
- torque PI は接触保護上限へ長く滞在する。上限緩和は不安全だったため、実機では速度PI再設計が必要である。
- 0.05 m/s、高外乱、センサ雑音、パラメータばらつき、長時間 drift は未確認である。
- 同定fitは姿勢以外で約62–63%であり、高精度MIMO設計の authority には不足する。
