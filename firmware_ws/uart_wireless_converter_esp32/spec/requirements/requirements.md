# 要件仕様書

`uart_wireless_converter_esp32` ファームウェアの機能要求・非機能要求を ID 付きで管理する。本ファームウェアは XIAO ESP32-C6 上で動作し、ホスト MCU（NUCLEO-G474RE）の UART と無線リンク（初版は BLE）の間でバイトストリームを透過する UART-無線変換器である。

## 機能要求

| ID | 要求 | 個別仕様 |
|---|---|---|
| FR-01 | BLE で UART バイトストリームを双方向透過する（NUS 相当） | [functional/FR-01.md](functional/FR-01.md) |
| FR-02 | 無線リンクの接続ライフサイクルを管理する | [functional/FR-02.md](functional/FR-02.md) |
| FR-03 | 診断・統計を USB シリアルの shell で提供する | [functional/FR-03.md](functional/FR-03.md) |
| FR-04 | 無線トランスポートを BLE/Wi-Fi で切替可能とする | [functional/FR-04.md](functional/FR-04.md) |
| FR-05 | 無線リンクの状態を基板ユーザー LED で表示する | [functional/FR-05.md](functional/FR-05.md) |

## 非機能要求

| ID | 要求 | 個別仕様 |
|---|---|---|
| NFR-01 | 転送の応答性を確保する | [nonfunctional/NFR-01.md](nonfunctional/NFR-01.md) |
| NFR-02 | 言語・アーキテクチャ規約を守る | [nonfunctional/NFR-02.md](nonfunctional/NFR-02.md) |
| NFR-03 | 静的解析を常時クリーンに保つ | [nonfunctional/NFR-03.md](nonfunctional/NFR-03.md) |
| NFR-04 | メモリ管理方針を守る | [nonfunctional/NFR-04.md](nonfunctional/NFR-04.md) |
| NFR-05 | ビルド・書き込みを自動実行できる | [nonfunctional/NFR-05.md](nonfunctional/NFR-05.md) |
