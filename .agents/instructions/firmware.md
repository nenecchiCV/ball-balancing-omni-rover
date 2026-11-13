# ファームウェア開発の指示

`firmware_ws/` 配下の STM32 ファームウェアを作成・編集する作業に適用する。

## 開発方針

- 仕様駆動開発を行う。要求 → 基本設計 → 詳細設計 → 実装 → 検証の順で整合を保ち、実装だけを先に変更しない。
- 変更の起点は要求とする。要求 ID を採番し、基本設計・詳細設計・実装へトレースできる状態を維持する。
- 設計書と実装は常に同期させる。実装を変える場合は対応する詳細設計を同じ変更内で更新し、構造を変える場合はコンポーネント一覧まで遡って更新する。

## 仕様書の構成

各ファームウェアプロジェクトはプロジェクト直下の `spec/` に Markdown で仕様書を置く。

- `spec/requirements/hardware_config.md`: 対象基板のハードウェア仕様。MCU、周辺 IC、電源、ピンアサインを記載する。ピンアサインの確定は本ファイルの責務とする。
- `spec/requirements/requirements.md`: 機能要求（FR-xx）と非機能要求（NFR-xx）を ID 付きで一覧管理する。各要求は 1 要求 1 ファイルとして下位ディレクトリに分け、一覧からリンクする。
- `spec/basic_design/basic_design.md`: ソフト構成、層構造、タスク管理、通信、メモリ管理、実装方針を記載する。
- `spec/basic_design/component_catalog.md`: コンポーネントを層ごとに一覧化する。1 コンポーネント = 1 クラスとする。
- `spec/detail_design/main.md`: 詳細設計の索引。層とソース構造の対応を記載する。
- `spec/detail_design/<層>/<component>.md`: クラスインターフェース形式で記述し、各関数の詳細は文章で説明する。
- `spec/test/`: オンボードテスト層の仕様。`test_plan.md` に実行モデルと一覧を、`<テスト名>.md` に手順・ログ形式・合否基準を記載する。テストは実機ソフトとは別アプリ（`tests/onboard/`）としてビルドする。

## 実装の構成

- `src/` のディレクトリ構造は `spec/detail_design/` の層構造に対応させる。
- `src/main.cpp` は `src/` 直下に置く。
- 1 コンポーネントごとにコンポーネント名と同名のディレクトリを作り、同名のヘッダとソースを配置する。
- オンボードテストは `tests/onboard/` の別 Zephyr アプリに実装し、コンポーネントは `tests/onboard/src/<component>/` に同じ規約で配置する。下位層の `src/common`〜`src/feature` をそのまま再利用し、倒立制御系は組み込まない。
- HAL・CMSIS・サードパーティは `src/` の外（`third_party/` など）に置き、アプリ側のインクルードパスで利用する。

## アーキテクチャルール

- 層構造は下から `common`（純粋ユーティリティ・定数・データ構造）→ `peripheral`（ペリフェラル制御）→ `device`（IC・センサ/アクチュエータ）→ `feature`（機能）→ `manager`（機能管理・タスク統合）とする。
- 依存は上位層から下位層へのみ許可する。下位層は上位層を知らない。同一層内の参照は循環しない範囲で許可する。
- クラスの継承は原則禁止とする。差し替え可能性が必要な場合もコンポジションとコンセプト/テンプレートで実現する。
- RTOS は Zephyr を使用する。ハードウェア構成（ピン・タイマ・I2C・PWM・QDEC・IMU・UART）は devicetree overlay で宣言し、コードは `DT_*` マクロ経由でのみ参照する。ペリフェラル制御は Zephyr ドライバ API（または LL レジスタ読取）を `peripheral` 層に閉じ込める。
- 浮動小数点は MCU の単精度 FPU に合わせ `float` で統一する。

## 言語規約（制限付き C++）

- C++23 を使うが、**STL は禁止**（`-nostdinc++`）。STL 相当が必要な箇所は ETL（`third_party/etl`、git submodule、`ETL_NO_STL` 定義）を使う。
- **例外・RTTI・動的確保は禁止**（`-fno-exceptions -fno-rtti`、`new`/`delete`/`malloc` 不使用）。カーネルヒープも `CONFIG_HEAP_MEM_POOL_SIZE=0` で無効化する。
- **静的初期化の制限**: `CONFIG_STATIC_INIT_GNU=n` により実行時コンストラクタは走らない。static メンバ・グローバルはゼロ/定数初期化のみで成り立たせ、非ゼロの値は `init()` で設定する（`-fno-threadsafe-statics` も付与）。
- **static class ルール**: システムに 1 つ常駐するコンポーネント（タスク・機能・デバイス・アプリケーション等、インスタンス化が不要なもの）は全メンバを `static` とする static class で実装し、デフォルトコンストラクタを `= delete` してインスタンス化を禁止する。static メンバは `static inline` で宣言する。モータ・エンコーダ・PWM・GPIO のように複数インスタンスが必要なクラスのみインスタンス化を許可する。
- **ライフサイクル**: コンストラクタ・デストラクタは使わない。設定の注入は `init(const config&)`、動作の開始・停止が必要なものは `start()`/`stop()` メンバ関数で行う。
- **命名規則**: 識別子は原則すべて `snake_case`（クラス・関数・変数・ファイル名）。定数のみ `UPPER_SNAKE_CASE` とし `constexpr` で定義する（定数目的のマクロは使わない）。Zephyr API・マクロ識別子はそのまま使用してよい。
- **コールバック命名**: ISR・shell ハンドラ・タスクフック等、登録されて呼び返される関数は `*_cb` サフィックスを付け、名前からコールバックと分かるようにする。static class 化によりインスタンス参照用のグローバルポインタは使わない。
- **オンボードテストの規約**: テストコンポーネントは `<name>_test` の static class とし、`init(config)` + `start_cb`/`update_cb`/`stop_cb` を持たせて `test_runner` の登録テーブルに追加する（継承は使わず関数ポインタで結合）。モータ駆動テストは `estop_input::requested()` を監視し `stop_cb` で必ずデューティをゼロにする。ログは ST-Link 仮想 COM へ `debug_uart`/`printk` で出力し、行先頭にテスト名接頭辞を付ける。

## ツールと自動実行

- ビルド・デバッグ・書き込み・静的解析は VSCode タスク（`.vscode/tasks.json`）として定義し、タスクが呼ぶコマンドをターミナルからそのまま実行できる形にする。AI エージェントは VSCode GUI なしで同等コマンドを実行して検証する。
- リンター・静的解析は clang-tidy を標準とし、`.clang-tidy` にポリシーを置く。整形は clang-format（`.clang-format`）とする。
- 制御アルゴリズムの正本は `matlab_ws/` の Simulink/MATLAB コードとする。移植時に挙動を変えた部分は詳細設計に「適応事項」として記録する。

## 検証

- 変更後は少なくとも Zephyr ビルド（`nucleo_g474re`）と clang-tidy を実行し、結果を報告する。
- feature 層のロジック変更時は `tests/host` のホスト単体テストを更新・実行する。
- 実機依存の検証（書き込み・デバッグ実行）はツールとボードが利用できる場合に実施し、未実施の項目は明示する。
