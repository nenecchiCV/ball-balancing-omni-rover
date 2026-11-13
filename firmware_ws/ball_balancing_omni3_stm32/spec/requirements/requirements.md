# 要件仕様書

`ball_balancing_omni3_stm32` ファームウェアの機能要求・非機能要求を ID 付きで管理する。正本となる制御・推定アルゴリズムは `matlab_ws/ball_balancing_omni3` の rev2 構成（`ballbotLqiControllerUpdateRev2.m` / `ballbotEstimatorStep.m` / `ballbotWheelServoDutyStep.m`）であり、ファームウェアはその挙動を `float`・組込み環境へ移植する。

## 機能要求

| ID | 要求 | 個別仕様 |
|---|---|---|
| FR-01 | IMU・輪速から姿勢・速度・ボール状態を推定する | [functional/FR-01.md](functional/FR-01.md) |
| FR-02 | エンコーダ変位から輪速を計測する | [functional/FR-02.md](functional/FR-02.md) |
| FR-03 | 200 Hz で LQI 姿勢トルク制御を実行する | [functional/FR-03.md](functional/FR-03.md) |
| FR-04 | 1 kHz で車輪サーボと PWM デューティ生成を実行する | [functional/FR-04.md](functional/FR-04.md) |
| FR-05 | 制御モード遷移と安全監視を行う | [functional/FR-05.md](functional/FR-05.md) |
| FR-06 | 非常停止入力でモータを即時遮断する | [functional/FR-06.md](functional/FR-06.md) |
| FR-07 | 移動指令（速度・ヨーレート）を生成・適用する | [functional/FR-07.md](functional/FR-07.md) |
| FR-08 | 診断情報を UART で出力する | [functional/FR-08.md](functional/FR-08.md) |

## 非機能要求

| ID | 要求 | 個別仕様 |
|---|---|---|
| NFR-01 | 制御周期を厳守する | [nonfunctional/NFR-01.md](nonfunctional/NFR-01.md) |
| NFR-02 | 静的メモリ管理と計算量を守る | [nonfunctional/NFR-02.md](nonfunctional/NFR-02.md) |
| NFR-03 | 言語・アーキテクチャ規約を守る | [nonfunctional/NFR-03.md](nonfunctional/NFR-03.md) |
| NFR-04 | 静的解析を常時クリーンに保つ | [nonfunctional/NFR-04.md](nonfunctional/NFR-04.md) |
| NFR-05 | ビルド・デバッグを AI が自動実行できる | [nonfunctional/NFR-05.md](nonfunctional/NFR-05.md) |
