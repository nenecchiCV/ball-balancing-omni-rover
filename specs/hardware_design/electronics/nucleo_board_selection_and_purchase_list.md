# 玉乗り3WDオムニローバー向けNucleoボード選定・購入リスト

調査日: 2026-09-15（2026-10-27 改訂: ボードをNUCLEO-G474RE、IMUをLSM6DSOへ変更）

## 結論

制御ボードには **NUCLEO-G474RE**、IMUには **X-NUCLEO-IKS01A3** 上の **LSM6DSO** を採用する。

NUCLEO-G474REは、STM32 Microcontroller Blockset（STM32CubeMXワークフロー）対応のNucleo-64ボードであり、170 MHz Cortex-M4F（単精度FPU・DSP命令）、512 KB Flash、128 KB SRAMを備える。TIM2～TIM5の直交エンコーダーモード、3チャネル20 kHz PWM、I2C IMU、および5 ms周期のEKF推定・LQI制御（状態数は5誤差状態＋既存チャネルでM4に十分な演算量）を実装できる。Nucleo-64はNucleo-144より小型・安価で、本機に必要な入出力数に対し過剰でない。最終的な余裕率は、生成コードを実機でPILまたは実行時間計測して確認する。

IMUはNucleo Arduinoコネクタ互換のセンサーシールドを積層する一体構成とする。X-NUCLEO-IKS01A3のLSM6DSOは6軸IMUで、高性能モード時のデータシート代表値はジャイロノイズ密度3.8 mdps/√Hz、加速度ノイズ密度70〜80 µg/√Hz（フルスケール依存）である。共分散設計済みの誤差状態EKF（`p.estimator.ekf.*`）はこのデータシート値を前提に調整されている。

## 機体から決まる入出力要件

| 機能 | 必要数 | 周期・電気条件 | 実装資源 |
|---|---:|---|---|
| FIT0521直交エンコーダー | 3組、6信号 | 3.3 V駆動、341.2 PPR公称 | エンコーダーモード対応タイマ3個 |
| MDD3A PWM | 3出力 | 最大20 kHz | PWMタイマ3チャネル |
| MDD3A DIR | 3出力 | 3.3 V GPIOで制御可能 | GPIO 3本 |
| IMU | 1 | I2C、加速度3軸・角速度3軸 | I2C 1系統、任意で割込み1本 |
| 状態推定・制御 | 1タスク | 5 ms、200 Hz | 浮動小数点演算、固定周期割込み |
| 非常停止 | 1 | PWMを即時0へ設定 | GPIO入力または外部割込み1本 |

FIT0521は6 V、ストール電流3.2 A、エンコーダー電源3.3 Vまたは5 Vである。エンコーダーは3.3 Vで給電し、Nucleo入力へ5 V信号を加えない構成とする。

## 対応ボード比較

本機が要求するのは3個の直交エンコーダー、3チャネルPWM、I2C IMU、200 HzのEKF+LQI演算であり、次の2機種を比較した。

| 候補 | CPU | メモリ | 評価 | 判断 |
|---|---|---|---|---|
| NUCLEO-G474RE | Cortex-M4F、170 MHz、単精度FPU/DSP | Flash 512 KB、SRAM 128 KB | Nucleo-64で小型・安価。TIM2～TIM5は直交エンコーダー対応、エンコーダ3組＋PWM3ch＋I2Cが収まる。200 HzのEKF（5誤差状態）＋LQIはM4Fで十分な演算余裕 | **採用** |
| NUCLEO-F767ZI | Cortex-M7、216 MHz、倍精度FPU/DSP | Flash 2 MB、SRAM 512 KBほか | I/O余裕が大きい。EKF拡張やロギングを増やすなら余力あり | 本機の入出力には過剰であり第一候補としない |

## IMU選定

| 候補 | 長所 | 短所 | 判断 |
|---|---|---|---|
| X-NUCLEO-IKS01A3 / LSM6DSO | 6軸、高性能モードでジャイロ3.8 mdps/√Hz・加速度70～80 µg/√Hz、最大6.7 kHz級ODR、磁気センサーLIS2MDLも搭載。Arduino積層でNucleo-64に適合 | LSM6DSO専用Simulinkブロックは無く、I2Cレジスタドライバを作る必要がある | **採用** |
| X-NUCLEO-IKS01A2 / LSM6DSL | MathWorks公式の姿勢推定例とLSM6DSLブロックがあり、立上げが容易 | 旧世代で、ジャイロノイズ密度4.5 mdps/√Hz | 開発工数優先の代替案 |

本機は磁気モータを3個搭載するため、オンボード磁気センサーを姿勢ヨー角へ無条件に融合しない。最初はLSM6DSOの加速度・角速度と車輪エンコーダーを用い、磁気センサーはモータ電流ごとの磁場校正と外れ値判定が成立した場合に限り補助観測として使用する。

LSM6DSOの組込みフュージョン出力は演算負荷の削減に有効だが、本機の最終姿勢精度は取付剛性、振動絶縁、温度ドリフト、軸ずれ、加速度外乱、および推定器調整にも依存する。センサー単体仕様だけで姿勢精度を保証しない。姿勢推定は機体側の誤差状態EKF（ジャイロ伝播＋加速度計重力観測のKalman更新、運動モデル＝車輪エンコーダ由来のボール加速度・相対加速度・回転加速度の3証人で重力補正の有効性を判定し、非慣性区間を構造的に除外）を使用し、共分散はLSM6DSOのデータシートノイズ密度から設定する。

## 実装方針

**STM32 Microcontroller BlocksetとSTM32CubeMXワークフロー** を使用する。CubeMXでTIM2、TIM3、TIM4をEncoder Modeへ設定し、別タイマの3チャネルを20 kHz PWMへ割り当てる。I2C、DIR、非常停止を含む具体的なピン割付は、X-NUCLEO-IKS01A3を積層した状態でCubeMXの競合検査を通して確定する。Nucleo-64はArduinoコネクタ側のみの積層となるため、Zio/Morpho側を使う配線は採用しない。

## 購入リスト

### 制御・センサーの新規購入

| 優先度 | 品名・型番 | 数量 | 用途 | 調査時の参考単価 | 小計目安 |
|---|---|---:|---|---:|---:|
| 必須 | STMicroelectronics NUCLEO-G474RE | 1 | 主制御、PWM、エンコーダー、I2C | 税込約3,500円 | 約3,500円 |
| 必須 | STMicroelectronics X-NUCLEO-IKS01A3 | 1 | LSM6DSO 6軸IMU | 税込約4,000円 | 約4,000円 |
| 必須 | 2.54 mmピンソケットまたはジャンパ線セット | 1式 | 使用ピンの引出し、試験配線 | 販売店見積 | - |
| 必須 | USB Type-A－Micro-Bデータケーブル | 1 | ST-LINK給電・書込み（G474REはMicro-B） | 手持ち確認 | - |
| 推奨 | 非常停止用ラッチスイッチ | 1 | PWMハードウェア停止入力 | 販売店見積 | - |
| 推奨 | 6 V、12 A以上の電流制限付き試験電源 | 1 | 3モータの立上げ試験 | 販売店見積 | - |
| 推奨 | 10 A級ヒューズ、ホルダー、主電源スイッチ | 1式 | 電源保護 | 販売店見積 | - |
| 推奨 | 0.1 µFセラミックコンデンサ | 6 | 各モータ端子－ケース/端子間のノイズ抑制を実測調整 | 販売店見積 | - |

制御ボードとIMUの確定小計目安は **約7,500円** である。価格、在庫、送料、消費税は発注時に再確認する。

### 機体部品を未購入の場合

| 品名・型番 | 数量 | 仕様・選定理由 | 参考価格 |
|---|---:|---|---:|
| DFRobot FIT0521 | 3 | 6 V、210 rpm、エンコーダー内蔵 | 税込約3,719円/個、約11,157円 |
| Cytron MDD3A | 2 | 2チャネル、4～16 V、3 A連続/5 Aピーク、PWM/DIR | 販売店見積 |
| Nexus Robot 14108 48 mmオムニホイール | 3 | 現行機構仕様 | 販売店見積 |
| 直径100 mm、質量約285 gの高グリップリジッドボール | 1 | 現行プラント仕様 | 販売店見積 |
| モータ・基板用コネクタ、電源線、信号線 | 1式 | モータ線はストール電流と配線長で線径決定 | 販売店見積 |

## 発注前チェック

- MATLABの使用リリースを確定する。実装経路はSTM32 Microcontroller Blocksetを前提とする。
- NUCLEO-G474REとX-NUCLEO-IKS01A3のArduino積層時に、TIM2、TIM3、TIM4の各CH1/CH2、PWM 3本、I2C、DIR 3本、非常停止1本が同時配置できることをCubeMXで確認する。Nucleo-64はモルフォヘッダ非対応のため、Arduino列だけで入出力を完結させる。
- X-NUCLEO-IKS01A3のLSM6DSOをI2Cモードで使用するためのジャンパ設定とI2Cアドレスを回路図で確定する。
- MDD3A入力、Nucleo、IMU、エンコーダーのGNDを共通化し、モータ電力配線とIMU信号配線を分離する。
- IMUは機体中心へ剛体固定しつつ、高周波モータ振動を伝えにくい取付を試験する。柔らかすぎる防振材は位相遅れを生むため、200 Hz閉ループ試験で決定する。

## 参照資料

- [MathWorks: Nucleoサポートのセットアップと移行案内](https://www.mathworks.com/help/stm32b/setup-and-configuration-nucleo.html)
- [MathWorks: STM32 Encoderブロック](https://www.mathworks.com/help/stm32b/ref/encoder.html)
- [MathWorks: Nucleoのエンコーダーカスタムドライバ例](https://www.mathworks.com/help/stm32b/ug/control-rotary-encoder-knob-using-stmicroelectronics-nucleo-board.html)
- [STMicroelectronics: NUCLEO-G474RE](https://www.st.com/en/evaluation-tools/nucleo-g474re.html)
- [STMicroelectronics: STM32G474REデータシート](https://www.st.com/resource/en/datasheet/stm32g474re.pdf)
- [STMicroelectronics: X-NUCLEO-IKS01A3](https://www.st.com/ja/evaluation-tools/x-nucleo-iks01a3.html)
- [STMicroelectronics: LSM6DSOデータシート](https://www.st.com/resource/en/datasheet/lsm6dso.pdf)
- [MathWorks: LSM6DSLとAHRSの姿勢推定例](https://www.mathworks.com/help/stm32b/ug/estimate-orientation-ahrs-imu.html)
- [DFRobot: FIT0521公式仕様](https://wiki.dfrobot.com/fit0521/)
- [Cytron: MDD3A公式仕様](https://www.cytron.io/cytron/p-3amp-4v-16v-dc-motor-driver-2-channels)
- [DigiKey: NUCLEO-F767ZI価格・在庫](https://www.digikey.jp/ja/products/detail/stmicroelectronics/NUCLEO-F767ZI/497-16525-ND/6004740)
- [Mouser: X-NUCLEO-IKS4A1価格・在庫](https://www.mouser.jp/ja/ProductDetail/STMicroelectronics/X-NUCLEO-IKS4A1)
- [DigiKey: FIT0521価格・在庫](https://www.digikey.jp/en/products/detail/dfrobot/FIT0521/7682226)
