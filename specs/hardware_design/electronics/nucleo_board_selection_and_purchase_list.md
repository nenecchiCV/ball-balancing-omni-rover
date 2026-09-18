# 玉乗り3WDオムニローバー向けNucleoボード選定・購入リスト

調査日: 2026-09-15

## 結論

制御ボードには **NUCLEO-F767ZI**、IMUには **X-NUCLEO-IKS4A1** 上の **LSM6DSV16X** を採用する。

NUCLEO-F767ZIは、指定されたSimulink Coder Support Package for STMicroelectronics Nucleo Boardsの対応機種であり、216 MHz Cortex-M7、倍精度FPU、DSP命令、2 MB Flash、512 KB SRAMを備える。3個の直交エンコーダー、3チャネルの20 kHz PWM、3本の方向信号、I2C IMU、および5 ms周期の推定・制御を実装するための演算性能と周辺機能を十分に持つ。最終的な余裕率は、生成コードを実機でPILまたは実行時間計測して確認する。

指定アドオンの対応NucleoボードにはIMUを基板上に搭載した機種がない。このため、「IMU内蔵」はNucleo互換センサーシールドを積層した一体構成として実現する。X-NUCLEO-IKS4A1のLSM6DSV16Xは6軸IMU、組込みセンサーフュージョン、代表値2.8 mdps/√Hzのジャイロノイズ密度を備え、旧世代LSM6DSLの代表値4.5 mdps/√Hzより低ノイズである。

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

指定アドオンの対応機種はNUCLEO-F031K6、F103RB、F302R8、F401RE、F411RE、F746ZG、F767ZI、L053R8、L476RG、H743ZI、H743ZI2である。このうち、本機の演算余裕、ピン数、タイマ数を重視して次の4機種を比較した。

| 候補 | CPU | メモリ | 評価 | 判断 |
|---|---|---|---|---|
| NUCLEO-F767ZI | Cortex-M7、216 MHz、倍精度FPU/DSP | Flash 2 MB、SRAM 512 KBほか | Nucleo-144でI/O余裕が大きい。TIM2～TIM5は直交エンコーダー対応。現行流通在庫も多い | **採用** |
| NUCLEO-H743ZI2 | Cortex-M7、480 MHz、倍精度FPU/DSP | Flash 2 MB、RAM 1 MB | 性能は最良。TIM2～TIM5は直交エンコーダー対応 | 販売店で生産中止表示があり、新規購入の標準品にはしない |
| NUCLEO-F746ZG | Cortex-M7、216 MHz | Flash 1 MB、SRAM 320 KB級 | 200 Hz制御には有力 | F767ZIよりメモリ余裕が小さいため第二候補 |
| NUCLEO-F411RE | Cortex-M4、100 MHz | Flash 512 KB、SRAM 128 KB | 基本制御は可能だが、タイマ・ピン割付と推定器拡張の余裕が小さい | 小型化を最優先する場合だけ再評価 |

H743ZI2のメーカー推奨代替品として流通店が示すNUCLEO-H753ZIは、指定アドオンの対応機種一覧に含まれない。動作確認なしに代替購入しない。

## IMU選定

| 候補 | 長所 | 短所 | 判断 |
|---|---|---|---|
| X-NUCLEO-IKS4A1 / LSM6DSV16X | 6軸、組込みセンサーフュージョン、2.8 mdps/√Hz、加速度60 µg/√Hz、最大7.68 kHz級ODR、磁気センサーLIS2MDLも搭載 | LSM6DSV16X専用Simulinkブロックは確認できず、I2Cレジスタドライバを作る必要がある | **精度優先で採用** |
| X-NUCLEO-IKS01A2 / LSM6DSL | MathWorks公式の姿勢推定例とLSM6DSLブロックがあり、立上げが容易 | 旧世代で、ジャイロノイズ密度4.5 mdps/√Hz | 開発工数優先の代替案 |

本機は磁気モータを3個搭載するため、オンボード磁気センサーを姿勢ヨー角へ無条件に融合しない。最初はLSM6DSV16Xの加速度・角速度と車輪エンコーダーを用い、磁気センサーはモータ電流ごとの磁場校正と外れ値判定が成立した場合に限り補助観測として使用する。

LSM6DSV16Xの組込みフュージョン出力は演算負荷の削減に有効だが、本機の最終姿勢精度は取付剛性、振動絶縁、温度ドリフト、軸ずれ、加速度外乱、および推定器調整にも依存する。センサー単体仕様だけで姿勢精度を保証しない。

## 実装方針

指定の旧Nucleoサポートパッケージは将来削除予定と案内されている。また、旧パッケージには3個の直交エンコーダーを直接設定する標準ブロックがなく、MathWorksの例もカスタムデバイスドライバ方式である。

新規実装は **STM32 Microcontroller BlocksetとSTM32CubeMXワークフロー** を使用する。CubeMXでTIM2、TIM3、TIM4をEncoder Modeへ設定し、別タイマの3チャネルを20 kHz PWMへ割り当てる。I2C、DIR、非常停止を含む具体的なピン割付は、X-NUCLEO-IKS4A1を積層した状態でCubeMXの競合検査を通して確定する。

旧アドオンを継続使用する場合は、MathWorksのエンコーダーカスタムドライバ例を3タイマへ拡張する。購入ボードはどちらの経路でもNUCLEO-F767ZIを共通利用できる。

## 購入リスト

### 制御・センサーの新規購入

| 優先度 | 品名・型番 | 数量 | 用途 | 調査時の参考単価 | 小計目安 |
|---|---|---:|---|---:|---:|
| 必須 | STMicroelectronics NUCLEO-F767ZI | 1 | 主制御、PWM、エンコーダー、I2C | 税込約6,378円 | 約6,378円 |
| 必須 | STMicroelectronics X-NUCLEO-IKS4A1 | 1 | LSM6DSV16X 6軸IMU | 税込約5,867円 | 約5,867円 |
| 必須 | 2.54 mmピンソケットまたはジャンパ線セット | 1式 | 使用ピンの引出し、試験配線 | 販売店見積 | - |
| 必須 | USB Type-A－Micro-Bデータケーブル | 1 | ST-LINK給電・書込み | 手持ち確認 | - |
| 推奨 | 非常停止用ラッチスイッチ | 1 | PWMハードウェア停止入力 | 販売店見積 | - |
| 推奨 | 6 V、12 A以上の電流制限付き試験電源 | 1 | 3モータの立上げ試験 | 販売店見積 | - |
| 推奨 | 10 A級ヒューズ、ホルダー、主電源スイッチ | 1式 | 電源保護 | 販売店見積 | - |
| 推奨 | 0.1 µFセラミックコンデンサ | 6 | 各モータ端子－ケース/端子間のノイズ抑制を実測調整 | 販売店見積 | - |

制御ボードとIMUの確定小計目安は **約12,245円** である。価格、在庫、送料、消費税は発注時に再確認する。

### 機体部品を未購入の場合

| 品名・型番 | 数量 | 仕様・選定理由 | 参考価格 |
|---|---:|---|---:|
| DFRobot FIT0521 | 3 | 6 V、210 rpm、エンコーダー内蔵 | 税込約3,719円/個、約11,157円 |
| Cytron MDD3A | 2 | 2チャネル、4～16 V、3 A連続/5 Aピーク、PWM/DIR | 販売店見積 |
| Nexus Robot 14108 48 mmオムニホイール | 3 | 現行機構仕様 | 販売店見積 |
| 直径100 mm、質量約285 gの高グリップリジッドボール | 1 | 現行プラント仕様 | 販売店見積 |
| モータ・基板用コネクタ、電源線、信号線 | 1式 | モータ線はストール電流と配線長で線径決定 | 販売店見積 |

## 発注前チェック

- MATLABの使用リリースを確定する。指定旧アドオンの公開互換範囲はR2016b～R2025bであり、R2026a以降はSTM32 Microcontroller Blocksetを前提とする。
- NUCLEO-F767ZIとX-NUCLEO-IKS4A1のArduino/Zio積層時に、TIM2、TIM3、TIM4の各CH1/CH2、PWM 3本、I2C、DIR 3本、非常停止1本が同時配置できることをCubeMXで確認する。
- X-NUCLEO-IKS4A1のLSM6DSV16XをI2Cモードで使用するためのジャンパ設定とI2Cアドレスを回路図で確定する。
- MDD3A入力、Nucleo、IMU、エンコーダーのGNDを共通化し、モータ電力配線とIMU信号配線を分離する。
- IMUは機体中心へ剛体固定しつつ、高周波モータ振動を伝えにくい取付を試験する。柔らかすぎる防振材は位相遅れを生むため、200 Hz閉ループ試験で決定する。

## 参照資料

- [MathWorks: Simulink Coder Support Package for STMicroelectronics Nucleo Boards](https://jp.mathworks.com/matlabcentral/fileexchange/58942-simulink-coder-support-package-for-stmicroelectronics-nucleo-boards)
- [MathWorks: Nucleoサポートのセットアップと移行案内](https://www.mathworks.com/help/stm32b/setup-and-configuration-nucleo.html)
- [MathWorks: STM32 Encoderブロック](https://www.mathworks.com/help/stm32b/ref/encoder.html)
- [MathWorks: Nucleoのエンコーダーカスタムドライバ例](https://www.mathworks.com/help/stm32b/ug/control-rotary-encoder-knob-using-stmicroelectronics-nucleo-board.html)
- [STMicroelectronics: NUCLEO-F767ZI](https://www.st.com/en/evaluation-tools/nucleo-f767zi.html)
- [STMicroelectronics: STM32F767ZIデータシート](https://www.st.com/resource/en/datasheet/stm32f767zi.pdf)
- [STMicroelectronics: X-NUCLEO-IKS4A1](https://www.st.com/ja/evaluation-tools/x-nucleo-iks4a1.html)
- [STMicroelectronics: LSM6DSV16Xデータシート](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf)
- [MathWorks: LSM6DSLとAHRSの姿勢推定例](https://www.mathworks.com/help/stm32b/ug/estimate-orientation-ahrs-imu.html)
- [DFRobot: FIT0521公式仕様](https://wiki.dfrobot.com/fit0521/)
- [Cytron: MDD3A公式仕様](https://www.cytron.io/cytron/p-3amp-4v-16v-dc-motor-driver-2-channels)
- [DigiKey: NUCLEO-F767ZI価格・在庫](https://www.digikey.jp/ja/products/detail/stmicroelectronics/NUCLEO-F767ZI/497-16525-ND/6004740)
- [Mouser: X-NUCLEO-IKS4A1価格・在庫](https://www.mouser.jp/ja/ProductDetail/STMicroelectronics/X-NUCLEO-IKS4A1)
- [DigiKey: FIT0521価格・在庫](https://www.digikey.jp/en/products/detail/dfrobot/FIT0521/7682226)
