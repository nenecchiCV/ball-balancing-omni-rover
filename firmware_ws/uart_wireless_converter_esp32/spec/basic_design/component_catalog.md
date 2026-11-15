# コンポーネント一覧

1 コンポーネント = 1 クラス。層は下から `common → peripheral → device → feature → manager` の順で、依存は上位→下位のみ。
システムに 1 つ常駐するコンポーネントは **static class**（全メンバ `static`、ctor `= delete`）とし、複数インスタンスが必要なもののみインスタンス化可能なクラスとする。

## common（純粋ユーティリティ・定数・データ構造）

現時点では該当なし。無線プロトコル上のフレーミング定数を導入する場合に追加する。

## peripheral（ペリフェラル制御）

| コンポーネント | クラス | 形態 | 責務 |
|---|---|---|---|
| host_uart | `host_uart` | static | UART0（ホスト UART）の割込駆動送受信。RX→リング・TX 送出の薄いラッパ |
| debug_console | `debug_console` | static | `usb_serial` console へのテキスト送出（`printk` ラッパ） |

## device（無線トランスポート）

| コンポーネント | クラス | 形態 | 責務 |
|---|---|---|---|
| link | `link` | static | 統一無線インターフェース（`init`/`connected`/`max_payload`/`send`）。`link_ble.cpp`（BLE）/`link_wifi.cpp`（将来 Wi-Fi）を Kconfig で選択 |

## feature（機能）

| コンポーネント | クラス | 形態 | 責務 |
|---|---|---|---|
| uart_bridge | `uart_bridge` | static | 双方向のリングバッファ排出と `link::send` 呼出し。ブリッジスレッドで稼働（FR-01） |
| link_monitor | `link_monitor` | static | 接続遷移・転送統計・最終活動時刻の記録と snapshot 提供（FR-02, FR-03） |

## manager（機能管理）

| コンポーネント | クラス | 形態 | 責務 |
|---|---|---|---|
| converter_application | `converter_application` | static | コンポーネント設定注入・初期化・shell コマンド・`run()` ループ（FR-03） |

## src/main

- `main.cpp`: `converter_application::init(cfg)` → `run()` の呼出のみ
