# 3WDオムニローバー

`omnirover3wd_multibody.slx` は、ヴイストン オムニローバー3WDの仕様を基にした Simscape Multibody モデルです。車体フレームは簡易剛体で表現し、3個の48 mmオムニホイールとサーボの質量・外形・速度・トルク制限を反映します。

## モデル化方針

- 本体外形は直径160 mm、高さ105 mm、総質量462 gです。
- ホイールは直径48 mm、幅25.1 mm、質量39 gです。
- サーボは41.7 mm × 19.7 mm × 42.9 mm、質量55 g、最大トルク13 kgf·cm、速度53～62 rpmです。
- 各ホイールは駆動方向の接触力のみを発生し、回転軸に垂直なローラー方向の摩擦係数を0とします。
- 接触力はサーボ最大トルクから求めた上限で飽和します。
- ロンリウム相当の床面は `Infinite Plane` として表現し、`Custom Tire Force and Torque` により接触判定、半径方向変形、法線ばね・ダンパ力を計算します。
- 駆動方向の摩擦力はサーボ上限と `摩擦係数 × 法線荷重` の小さい方で制限します。ローラー方向の摩擦力は0です。
- 指令 `[vx_W, vy_W, yawRate_W]` はワールド座標系で与え、現在のヨー角を使って各輪速度へ変換します。
- モデル内の `WorldToWheelKinematics` ブロックは `omni3wdInverseKinematics.m` を呼び出し、3輪の回転速度ベクトルを行列演算で計算します。

ロンリウムとローラーの組み合わせによる摩擦係数は表面状態に依存します。`parameters.m` の `floorLongitudinalStaticFriction` と `floorLongitudinalDynamicFriction` を実測値に合わせて調整できます。

## 実行

MATLAB でこのフォルダーを現在のフォルダーにして、次を実行します。

```matlab
run_demo
```

`parameters.m` の `commandVxWorld`、`commandVyWorld`、`commandYawRateWorld` を変更すると、ワールド座標系の並進・回転指令を変更できます。

運動学単体テストは次のコマンドで実行します。

```matlab
runtests("test_kinematics.m")
```
