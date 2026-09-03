# Codex依頼計画：100 mm PLAY/リジッド球へのSimulink適合

## 目的

`matlab_ws/ball_balancing_omni3/ball_balancing_omni3_multibody.slx`を、直径100 mm、基準質量285 gのPLAY/リジッド球を使用するモデルへ変更する。

## Codexへの依頼内容

1. Simulink Agentic Toolkitでモデル構造と球パラメータの参照箇所を確認する。
2. `ballbotParameters.m`を次の値へ変更する。
   - `p.ball.radius = 0.050` m
   - `p.ball.mass = 0.285` kg
   - `p.wheel.contactLatitude = deg2rad(55)`
3. 球慣性は現在の薄肉球モデルを維持し、次式から再計算する。
   - `p.ball.inertia = (2/3)*p.ball.mass*p.ball.radius^2*eye(3)`
   - 対角成分の基準値は`4.75e-4 kg*m^2`
4. 球半径から導出される次の値が自動更新されることを確認する。
   - 球とホイールの接触位置
   - ホイール幾何行列
   - 機体初期高さ
   - ホイール法線荷重
   - 接触摩擦によるトルク上限
5. `.slx`内に直径150 mmまたは旧質量を直接記述した定数があれば、`ballbotParams`参照へ置き換える。
6. 球のSolid設定、初期配置および3輪の接触位置を更新し、モデル構造を検査する。
7. READMEと関連仕様から「直径150 mm」「300 g」「0.075 m」「0.300 kg」の旧記述を削除し、100 mm、285 gへ統一する。

## 検証

1. `ballbotParameters`を実行し、半径、質量、慣性および接触緯度を数値確認する。
2. モデル更新時にエラーがないことを確認する。
3. 静止シミュレーションで球―地面1点、球―ホイール3点の接触を維持する。
4. 初期状態で不自然な貫通、浮き上がり、NaNまたはInfがないことを確認する。
5. 既存MATLABテストを実行し、失敗があれば球寸法変更によるものだけを修正する。
6. `run_demo.m`を実行し、姿勢、球位置およびホイール速度を確認する。

## 完了条件

- モデルの球直径が100 mm、質量が0.285 kgである。
- 球慣性の対角成分が`4.75e-4 kg*m^2`である。
- 接触緯度が55 degである。
- 3輪が球へ接触し、静止シミュレーションが有限値で完了する。
- 旧150 mm・300 g条件が実行対象ファイルと説明文に残っていない。
- 変更ファイル、テスト結果および未調整の制御ゲインを報告する。

## 対象ファイル

- `matlab_ws/ball_balancing_omni3/ballbotParameters.m`
- `matlab_ws/ball_balancing_omni3/ball_balancing_omni3_multibody.slx`
- `matlab_ws/ball_balancing_omni3/README.md`
- `specs/hardware_design/controll/ball_balancing_omnirover3wd/ball_balancing_omnirover3wd-implementation-plan.md`
- `specs/hardware_design/controll/ball_balancing_omnirover3wd/ball_balancing_omnirover3wd-test-plan.md`
