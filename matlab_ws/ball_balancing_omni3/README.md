# 玉乗り3WDオムニローバー

> このブランチは MATLAB R2024b 対応版です。`ball_balancing_omni3_multibody_lqi_rev1.slx` は R2024b 形式で保存されており、R2024b (Update 9) で開いて利用できます。

`ball_balancing_omni3_multibody_lqi_rev1.slx` は、直径100 mm、質量285 gの薄肉球（ゴムスプレー被覆を想定）、3個のServoCity/goBILDA 48 mmオムニホイール（φ48 mm、32 g、全幅18 mm、50Aデュロメータのゴムローラー2列）、3個のDFRobot FIT0521エンコーダー付きDCモータ、2枚のCytron MDD3A、上部ペイロードで構成する閉ループSimscape Multibodyモデルです。本モデルをシステムの正本とします。

## 正本モデル

`ball_balancing_omni3_multibody_lqi_rev1.slx` を現行の正式開発モデルとします。実機rev1ハードウェア構成を再現し、車輪駆動はFIT0521モータ＋MDD3AドライバのSimscape Electrical回路（PWM duty、エンコーダー量子化341.2 PPR）としてモデル化しています。搭載電源は Zeee 7.2 V 5000 mAh NiMH パック（0.423 kg、1層目中央配置）で、ドライバ供給電圧は公称 7.2 V、PWM duty 上限は FIT0521 の定格 6 V 平均を超えない値（6/7.2≈0.83）に制限しています。機体上部のペイロードは質量0.50 kg（公称設計点）、フレーム構造は `ball_balancing_omnirover3wd_assembly` をABS素材で簡易再現しています。

球―地面と3組のホイール―球はすべて `Spatial Contact Force` で接続し、ホイール―球接触は `ballbotCustomFriction.m` による異方性摩擦を `Provided by Input` で与えます。

パラメーターは `ballbotRev1Parameters` が生成する構造体 `ballbotParams` をモデルワークスペースへ設定します（`PreLoadFcn` が未設定時に自動で呼び出します）。`ballbotRev1Parameters` は `ballbotFullPlantParameters` → `ballbotParameters` の順で基礎値を引き継ぎ、rev1向けに電気系アクチュエータ量（電機子インダクタンス、MDD3A論理・ブリッジ抵抗）と移動/停止/ヨー繰り返し指令プロファイルを追加します。

| モデル | パラメーター関数 |
|---|---|
| `ball_balancing_omni3_multibody_lqi_rev1.slx` | `ballbotRev1Parameters` |

## モデル階層

```mermaid
flowchart LR
  CMD[CommandSource] --> CTRL[Controller]
  CTRL --> PLANT[MultibodyPlant]
  PLANT --> IMU[IdealIMU<br/>比力・角速度]
  PLANT --> ENC[エンコーダー<br/>車輪回転変位]
  ENC --> WS[WheelSpeedSensing<br/>後退差分]
  IMU --> EST[StateEstimator]
  WS --> EST
  WS --> CTRL
  EST --> BC[BalanceController<br/>カスケード制御]
  BC --> WD[WheelDrive<br/>トルク→duty変換+DCモータ]
  WD --> PLANT
  PLANT --> LOG[Logging]
```

### Controller 内サブシステム

| サブシステム | 役割 | 主な内容 |
|---|---|---|
| `WheelSpeedSensing` | 車輪回転変位の後退差分で輪速を算出 | `WheelRateInputs`、`WheelRateCalculation`、`PreviousWheelDisplacement` |
| `StateEstimator` | 14状態推定器の更新と状態保持 | `EstimatorInputs`、`EstimatorUpdate`、`EstimatorStateMemory`、`SelectEstimate`、`SelectNextEstimatorState`、`SelectYawBiasDiagnostics` |
| `BalanceController` | 指令・推定値から3輪トルク指令を生成 | `ControlInputs`、`LqiController`、`LqiFeedbackInputs`、`LqiFeedbackState`、`VelocityIntegralMemory`、`SelectMode`、`SelectWheelSpeedCommand`、`SelectYawBiasReady` |
| `WheelDrive` | トルク→duty変換＋MDD3A電圧制限＋DCモータで輪トルクを生成 | `SelectDutyCommand`、`SpeedPiDuty`、`SpeedPiMotorInputs` |

### MultibodyPlant 内サブシステム

| サブシステム | 役割 | 主な内容 |
|---|---|---|
| `Environment` | 世界座標・ソルバー・床面 | `World`、`SolverConfig`、`MechanismConfig`、`GroundFrame`、`GroundPlane` |
| `Ball` | 球剛体と球―床接触 | `BallFreeJoint`、`BallSolid`、`BallGroundContact`、位置・姿勢の物理信号計測、実行時追加の転がり抵抗負荷（`ballbotRev1PlantAugment`） |
| `RoverChassis` | 機体・ペイロード・モータハウジング | `RoverFreeJoint`、`ChassisSolid`、`PayloadRodMount`＋`PayloadRod500g`、`Motor1..3Solid`、位置・加速度・角速度・姿勢の物理信号計測 |
| `WheelAssembly1..3` | 車輪回転自由度・車輪固体・輪―球接触 | `WheelNMount`、`WheelNJoint`、`WheelNSolid`、`WheelNBallContact`、回転変位・接触状態・法線力・すべりの計測変換 |

## 制御構成

制御器はトルクLQI経路を既定とします（`p.controller.torqueDrive.enabled`）。外側のボール速度→リーン変換と内側のロバスト固定ゲインLQI、プラント誤差を抑え込む輪速サーボ推進の構造です。

1. `ballbotLqiControllerUpdate` は `p.controller.cascade.enabled` が真のとき `ballbotCascadeUpdate` へ委譲し、偽のとき既定のトルクLQI経路を実行します。
2. トルクLQI経路は `ballbotRobustTorqueGains` の固定ゲインでフィードバックし、速度誤差→指令リーンの外ループ（`velocityLeanGain`）で移動を生成します。
3. プラント誤差抑制機構として、輪速整合サーボ推進（`propulsionGain`＋トラクション配分）、フィルタ済み輪速の滑り減衰（エンコーダー量子化フリッカー対策のLPF＋小デッドゾーン）、コモンモード残差除去、ボール垂直スピンブレーキ（`ballYawGain`）、並進ゲート付きヨーダンパーを持ちます。
4. `ballbotTorqueDutyStep` が輪トルクをモータ式 `duty = (R*I + Ke*w)/Vs` でdutyへ変換し、ドライバ連続限界とトラクション限界で飽和します。

LQIゲインは `ballbotRobustTorqueGains` が設計します。実測トルクステップB行列をペイロード族0〜1 kgへスケーリングし、族全体で保持床を満たすロバスト固定ゲインを選択します。

`p.controller.cascade.enabled = true` の構成ではカスケード経路（MiaPURE方式準拠の速度→リーン外層＋6状態LQR＋ボール速度PI、`ballbotCascadeGains` 設計）へ切り替わります。保持性能はトルクLQI経路が優位（5 s保持の最大傾斜 ~0.007 rad対 ~0.04 rad）なためトルクLQIを既定としています。

`p.controller.ballforce.enabled = true` の構成ではボールフォース経路（`ballbotBallforceUpdate`）へ切り替わります。姿勢・モード管理・滑り/スピン/ヨーの各レールはトルクLQIと同一で、並進チャネルだけを輪速Pサーボから「ボール速度PI → 平面力要求 → `pinv(Bvel)` で最小ノルム輪トルクへ配分」へ置き換えます。輪速サーボは接触拘束で輪速=ボール転動速度にピンされるため到達不能なレート目標に飽和して失速しますが、力配分は実ボール速度誤差に比例する要求なのでその失速が起きません。静止摩擦のプリロードとして指令輪速方向への Coulomb フィードフォワードを持ちます。実測ではトルクLQIが全く動かせなかった 0.10 m/s ステップへボールを実際に追従させます（整定誤差 +0.011 m/s）。サイクル耐久はトルクLQIのほうが長いため既定はトルクLQIのままです。

## MATLAB関数

| ファイル | モデル内の呼出元 | 役割 |
|---|---|---|
| `ballbotRev1Parameters.m` | rev1モデルのパラメーター初期化 | rev1モデル用パラメーター `ballbotParams` を生成 |
| `ballbotFullPlantParameters.m` | `ballbotRev1Parameters` から呼出し | ペイロード・接触・幾何の共通パラメーター |
| `ballbotParameters.m` | `ballbotFullPlantParameters` から呼出し | 幾何・モータ・接触・推定・制御の基礎パラメーター |
| `ballbotWheelGeometry.m` | パラメーター初期化 | 球面接触点、転動方向、車軸方向、輪速変換行列 |
| `ballbotDesignLqi.m` | パラメーター初期化 | 縮約10状態プラントのLQIゲイン設計 |
| `ballbotDesignTorqueLqi.m` | `ballbotRev1Parameters` から呼出し | トルク入力プラントのLQR設計 |
| `ballbotRobustTorqueGains.m` | `ballbotRev1Parameters` から呼出し | 実測B行列の質量族スケーリングとロバスト固定ゲイン選択 |
| `ballbotCascadeGains.m` | `ballbotRev1Parameters` から呼出し | カスケード6状態LQRゲイン設計（保持床制約付き） |
| `ballbotReducedPlantDynamics.m` | `ballbotDesignLqi` から呼出し | 縮約10状態プラントの非線形動力学 |
| `ballbotTorquePlantDynamics.m` | `ballbotDesignTorqueLqi` から呼出し | トルク入力縮約プラントの非線形動力学 |
| `ballbotPayloadRoverProperties.m` | パラメーター初期化 | ペイロード→質量/重心/慣性の換算 |
| `ballbotRev1CommandProfile.m` | `CommandSource` | 移動/停止繰り返し＋ヨー時間分離指令プロファイル（加減速別レート制限） |
| `ballbotRev1PlantAugment.m` | `ballbotRev1Parameters` から呼出し（実行時） | 球―床転がり抵抗（`External Force and Torque`）と輪軸 Coulomb/静摩擦（`Ideal Torque Source`＋`ballbotShaftFrictionTorque`）を実行時追加（.slx は R2024b 形式を維持・冪等・フラグoffで生成ブロック除去と粘性係数復元） |
| `ballbotShaftFrictionTorque.m` | augment 生成の `WheelShaftFricFcn` から呼出し | 輪軸速度→滑らかな Coulomb/breakaway 摩擦トルク |
| `ballbotRollingResistance.m` | `Ball/BallRollResFcn` | 球速度・球スピン・法線力から転がり抵抗の力/トルクを算出 |
| `ballbotCustomFriction.m` | `WheelAssembly/WheelContact` | 駆動方向とローラー方向を分離した異方性接触摩擦 |
| `ballbotIdealImu.m` / `ballbotImuFromJoint.m` | `MultibodyPlant/IdealIMU` | 6-DOF Joint真値から比力・角速度を生成 |
| `ballbotWheelRateFromDisplacement.m` | `Controller/WheelSpeedSensing` | 車輪回転変位を5 ms後退差分して回転速度を算出 |
| `ballbotEstimatorStep.m` / `ballbotEstimatorUpdate.m` | `Controller/StateEstimator` | IMU・エンコーダー融合、ボール回転・相対位置・ヨー軸ジャイロバイアス推定 |
| `ballbotEstimatorInitialState.m` | 初期値式 | 推定器14状態の初期値 |
| `ballbotTruthGate.m` | `Controller/TruthGate`（診断用） | `p.estimator.truthFeedbackEnabled` 時に推定値をプラント真値で置換 |
| `ballbotYawBiasStartupGuard.m` | `BalanceController` 内関数から呼出し | バイアス収束までヨー制御だけを抑止する準備完了ラッチ |
| `ballbotLqiControllerUpdate.m` | `Controller/BalanceController/LqiController` | モード管理・トルクLQI本体・fullplant/カスケード/ボールフォース経路のディスパッチ |
| `ballbotBallforceUpdate.m` | `ballbotLqiControllerUpdate` から呼出し | ボールフォース制御器（ボール速度PI→力配分の新規並進チャネル＋LQI姿勢レール共有） |
| `ballbotFullPlantHierarchyUpdate.m` | `ballbotLqiControllerUpdate` から呼出し | 旧階層制御（速度外側ループ＋傾斜安定化＋輪速変換） |
| `ballbotCascadeUpdate.m` | `ballbotLqiControllerUpdate` から呼出し | カスケード制御器本体 |
| `ballbotLqiFeedbackState.m` | `Controller/BalanceController` | LQIフィードバック状態ベクトルの組立て |
| `ballbotSpeedPiMotorStep.m` | `ballbotSpeedPiDutyStep` から呼出し | 輪速PI・MDD3A電圧制限・DCモータトルク |
| `ballbotSpeedPiDutyStep.m` | `Controller/WheelDrive` | 輪速PIからPWM dutyを生成（トルクモード無効時のフォールバック） |
| `ballbotTorqueDutyStep.m` | `Controller/WheelDrive` | 輪トルク→duty変換（逆起電力補償・飽和） |
| `ballbotMdd3aVoltageController.m` | 輪速PI関数から呼出し | 輪速PIからMDD3A電圧と積分器更新を生成 |
| `ballbotMdd3aGateDrive.m` | `WheelDrive` | duty指令をMDD3AのPWM/方向ロジック信号へ変換 |
| `ballbotPoseFromJoint.m` | `Logging` | 位置・クォータニオンからxyz/RPY真値を生成 |
| `ballbotEulerToQuaternion.m` | 補助 | オイラー角からクォータニオン変換 |
| `scratch_cycle.m` | 検証用エントリポイント | サイクルシミュレーション実行と `cycle_result_<tag>.mat` 保存 |
| `ballbotBenchmark.m` | 検証用エントリポイント | シナリオ（hold/stepX/yaw/cycle/bias）を実行し G-01〜G-11 指標を採点・要約行を出力 |

推定器の内部状態は14要素で、末尾にヨー軸ジャイロバイアス推定値と低運動継続時間を保持します。制御器へ渡す`estimate(14)`の幅と順序は維持し、4～6番目をバイアス補正後の機体角速度とします。バイアス推定値、学習許可、継続時間は診断信号として扱います。起動時はバイアス収束までヨートルクだけを抑止し、明示的なヨー指令は抑止をバイパスします。

## 実行

モデルを開くだけで `PreLoadFcn` が `ballbotRev1Parameters` を呼び出すため、`ballbotParams` をベースワークスペースへ明示設定する必要はありません。`InitFcn` が不一致・未設定の `ballbotParams` をシミュレーション開始時に自動で再生成します。

```matlab
cd matlab_ws/ball_balancing_omni3

% rev1モデル（トルクLQI制御・既定）
ballbotRev1SimConfig;  % MaxStep と MDD3A 回生モードを適用（~4倍高速化）
p = ballbotRev1Parameters;
assignin("base", "ballbotParams", p);
in = Simulink.SimulationInput( ...
    "ball_balancing_omni3_multibody_lqi_rev1");
in = in.setModelParameter("StopTime", "4");
out = sim(in);
```

`.slx` は R2024b 形式を維持する必要があるため（新しい MATLAB では Simscape Electrical ブロックを旧形式へエクスポートできない）、ソルバー・ドライバ設定は `ballbotRev1SimConfig` で適用します。検証スクリプトは `scratch_cycle.m`（ペイロードとフィールドオーバーライドを受理）、`scratch_hold.m`（ゼロ指令保持）、`scratch_move.m`（移動ランプ）、`scratch_perf.m`（実行時間計測）です。`p.command.cyclesEnabled` の移動/停止繰り返しプロファイルが既定で有効です。各スクリプトは `sim` 前に `clear functions` を実行します。モデル内の解釈実行 MATLAB Fcn ブロックの `persistent` 状態は同一 MATLAB セッションのシミュレーション間で残留し、同一パラメーターでも転倒時刻が変わることを確認したためです。

## 検証状態

| シナリオ | 結果 |
|---|---|
| シミュレーション速度 | `ballbotRev1SimConfig` 適用で実行時間 ~4倍改善（MDD3A 回生モードの方程式切替排除が主因） |
| ゼロ指令直立保持（バッテリー搭載・ペイロードなし） | 8 s 完走・転倒なし（tilt 最大 ~0.000 rad — 転がり抵抗が微小ドリフトまで吸収し完全静止） |
| ペイロード 0.5–1.0 kg 保持 | 8 s 完走・転倒なし（0.5 kg: ~0.000 rad、1.0 kg: ~0.010 rad）。静止判定に ~1 s 連続保持の滞留ゲートを追加し、ブラインド帯境界の低速クリープ（~0.025 m/s）を実運動として制御へ残すように改善 |
| 移動フェーズ（ペイロードなし） | 指令 0.12 m/s・μr=0.008 で移動＋停止ウィンドウを完走した実績あり（傾斜 ≤0.011 rad）。実行差で減速コーナー（指令 → 0 近傍）のボール追い越しが ~0.2 m/s 接触壁に達して転倒する既知限界は残る |
| LQI再設計（飽和織り込み lean 帯域・R=4000・輪速オブザーバ）後のサイクル試験 | 指令 0.12 m/s の移動/停止繰り返しで t=25.6 s（cycle 2 減速区間の脱輪）まで生存。旧値への単独復旧はいずれも早期転倒（`tiltStiffenGain=4.0`: 23.9 s、`leanSetpointLimit=0.016`: 21.6 s、輪速オブザーバ無効: 23.2 s）で、再設計3要素が揃った構成が最長。回復モード（mode 2）は各ランで複数回作動し一部復帰。duty は ~0.83 クランプに対し p99≈0.35 で飽和型 bang-bang は解消済み |
| 0.306 m/s 指令サイクル | t=9.3 s で転倒。ボール速度が接触壁を超えて伸びるため、`leanSetpointLimit` ではなく脱輪が限界を支配 |
| プラント実機化（IMUノイズ・量子化・輪軸静/Coulomb摩擦）＋制御器A/B | 同一プラント・同一シナリオで `ballbotBenchmark` 比較。保持: LQI 0.780° vs ballforce 0.754°（同等）。stepX（0.10 m/s）: LQI は整定誤差 -0.105 m/s で全く動かず（輪速サーボが接触拘束に失速）、ballforce は +0.011 m/s へ追従（ボールが実際に指令速度へ到達）。yaw: ballforce 7/8、LQI 6/8。cycle（0.12 m/s 移動/停止）: LQI t=36.6 s、ballforce t=14.9 s でともに転倒。トラクション上限 0.078→0.10 N・m で ballforce は 22.9 s まで改善するが LQI は 13.1 s へ悪化（追加トルクがリミットサイクルを増幅）。ballforce は追従力で優位・cycle 耐久で劣後のため実験経路として保持し、既定はトルクLQI |
| MATLAB単体試験 | `ballbotEstimatorStepTest` 16件、`ballbotParametersTest` 3件が合格 |

既知の残課題:

- 0.306 m/s 定速走行・移動/停止繰り返しは未達成。ボール速度 ~0.2 m/s が接触包絡の壁（1輪の脱輪→接触崩壊→転倒）で、加速は追従しても減速時の運動量移行（車体ピッチバック→ボール前射出）で壁へ到達する。球―床転がり抵抗（μr=0.008、実機想定値域 0.005–0.02 内）で惰行散逸経路を実装し、μr=0.008 は移動＋停止ウィンドウ完走の実績がある（0.005 は減速コーナーの追い越しを通し、0.01 はボール追従遅れで相殺されることを実測確認済み）。真値フィードバック・接触プリロード増量・全補助チャネルoffでも同一の壁が出ることを確認済み。
- 有効速度帯は狭い: 下はエンコーダー量子化由来の静止判定閾値（フィルタ済み輪速 0.8 rad/s＝ボール ~0.026 m/s未満で速度推定がドレインへ切替わる。境界フリッカー対策として ~1 s 連続静止の滞留ゲート `stillVelocityDrainHoldTime` を追加し、ブラインド帯縁の実クリープを推定速度へ残す）、上は ~0.2 m/sの接触壁。
- LQRの速度状態列は非最小位相経路（ボール逆蹴りで車体前傾）を選択するため既定で無効（`velocityStateFeedback=false`）。速度追従は外側リーンループと輪速サーボ推進が担当。
- ボール鉛直スピンのポンプ機構はコモンモード残差除去＋スピンブレーキ（`ballYawGain=0.06`）で抑制済み（巡航中の傾斜歩行を消去）。
