# ハードウェア構成仕様

本ファイルは `ball_balancing_omni3_stm32` ファームウェアが対象とする基板のハードウェア仕様を定義する。ピンアサインの確定は本ファイルを正本とし、`boards/nucleo_g474re.overlay`（devicetree）をその機械実行可能な写像として同期させる。

## 構成要素

| 要素 | 部品 | 概要 |
|---|---|---|
| MCU ボード | NUCLEO-G474RE | STM32G474RE、Cortex-M4F 170 MHz、単精度 FPU、Flash 512 KB、SRAM 128 KB（SRAM1 80 KB + SRAM2 16 KB + CCM 32 KB） |
| IMU | LSM6DSO(X) ブレークアウト（ユニバーサルシールド実装） | 3軸加速度 + 3軸ジャイロ、I2C 接続 |
| モータ | DFRobot FIT0521 ×3 | 6 V ギヤード DC モータ、正交ホールエンコーダ 341.2 PPR（出力軸換算） |
| モータドライバ | Cytron MDD3A ×2 | 3 チャネル・デュアル DC モータドライバ、PWM/DIR 制御 |
| 電源 | NiMH 7.2 V | MDD3A モータ電源直結。5 V 系は MDD3A-A の 5VO のみから Nucleo E5V へ供給（MDD3A-B の 5VO は未接続）。全 GND 共通 |

## MCU リソース割当

| 機能 | ペリフェラル | ピン | Arduino 端子 | 備考 |
|---|---|---|---|---|
| ENC0 A/B | TIM2 CH1/CH2（エンコーダモード） | PA0 / PA1 | A0 / A1 | ホイール0（P0、+X 方向） |
| ENC1 A/B | TIM1 CH1/CH2（エンコーダモード） | PA8 / PA9 | D7 / D8 | ホイール1 |
| ENC2 A/B | TIM3 CH1/CH2（エンコーダモード） | PA6 / PA7 | D12 / D11 | ホイール2 |
| PWM0 | TIM17 CH1 | PB5 | D4 | MDD3A-A M1A |
| PWM1 | TIM16 CH1 | PB4 | D5 | MDD3A-B M1A |
| PWM2 | TIM4 CH1 | PB6 | D10 | MDD3A-B M2A |
| DIR0 | GPIO 出力 | PB3 | D3 | MDD3A-A M1B。PWM0=D4 と隣接するよう配置 |
| DIR1 | GPIO 出力 | PB10 | D6 | MDD3A-B M1B。PWM1=D5 と隣接 |
| DIR2 | GPIO 出力 | PC7 | D9 | MDD3A-B M2B。PWM2=D10 と隣接 |
| I2C1 SCL/SDA | I2C1 | PB8 / PB9 | D15 / D14 | IMU 接続。アドレス 0x6B/0x6A を起動時に走査 |
| 非常停止（外部） | GPIO 入力（プルアップ、EXTI） | PC0 | A5 | 低レベル = 停止要求 |
| 非常停止（基板ボタン） | GPIO 入力（EXTI） | PC13 | B1 | ユーザボタン、押下 = 停止要求 |
| 状態表示 LED | GPIO 出力 | PA5 | D13 | Nucleo 緑 LED |
| 診断 UART | USART2 TX/RX | PA2 / PA3 | （ST-LINK VCP） | 115200 8N1、テレメトリ出力 |
| 制御タイムベース | TIM6 | — | — | 1 kHz 周期割込（内側タスク）。外側 200 Hz は 5 カウント分周 |

## タイミング要件

| タスク | 周期 | 根拠 |
|---|---|---|
| 外側タスク（推定 + LQI + モード） | 5 ms（200 Hz） | `p.controller.sampleTime = 0.005` |
| 内側タスク（車輪サーボ + duty） | 1 ms（1 kHz） | `p.controller.servoSampleTime = 0.001` |
| 輪速計測窓 | 5 ms | 1 ms 後退差分では量子化フリッカ ~18.5 rad/s が発生するため |

## 周辺 IC 仕様

### LSM6DSO(X)（IMU）

- I2C アドレス: 0x6B（SDO プルアップ）/ 0x6A（SDO 接地）。WHO_AM_I = 0x6C で照合して確定する。
- 加速度: ±4 g、ODR 833 Hz、感度 0.122 mg/LSB（1.19625e-3 m/s²/LSB）
- ジャイロ: ±2000 dps、ODR 833 Hz、感度 70 mdps/LSB（1.22173e-3 rad/s/LSB）
- 出力は比力（specific force）。静止時は +1 g が鉛直上向きに観測される。
- 機体座標系: +X は P0 モータ方向。IMU 実装向きから機体軸への回転は設定定数とする。

### FIT0521（モータ + エンコーダ）

- 定格 6 V、無負荷 210 rpm、ストール 3.2 A、エンコーダ電源 3.3 V
- エンコーダ分解能: 341.2 PPR（出力軸 1 回転あたり）。タイマの x4 デコードで得る生カウントとの換算係数は `countsPerRev` 定数に集約し、実機で確認する。

### MDD3A（モータドライバ）

- 制御方式: MxA = PWM、MxB = DIR（正転: DIR=H、逆転: DIR=L）
- ロジック: 3.3 V 対応。モータ電源はバッテリ直結 7.2 V。
- 連続トルク上限、デューティ上限 0.8333 はソフト定数側で強制する。

## 未確定事項

- エンコーダの生カウント/回転（x4 デコード込み）: 実機で 1 回転分のカウントを測り `countsPerRev` を確定する。
- IMU 実装向きの機体軸マッピング: シールド実装時に確認して軸変換定数を確定する。
- 5 V 系の実測電流: 200 mA 予算に対して起動後に実測する。
