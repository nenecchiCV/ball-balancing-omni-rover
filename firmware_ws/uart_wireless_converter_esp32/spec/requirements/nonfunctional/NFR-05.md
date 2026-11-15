# NFR-05 ビルド・書き込みの自動実行

## 内容

VSCode GUI なしでビルド・静的解析・書き込みをコマンド実行できる形にする。

## 仕様

- Zephyr アプリケーションとして `cmake -S . -B build-zephyr -DBOARD=xiao_esp32c6/esp32c6/hpcore` → `cmake --build build-zephyr` でビルドする。
- 対象ボードは `xiao_esp32c6/esp32c6/hpcore`。ブートは既定の Simple Boot（単一イメージ、ブートローダなし）とする。
- `hal_espressif` モジュールとバイナリ blob（`west update hal_espressif` → `west blobs fetch hal_espressif`）をワークスペースに導入する手順を README に明記する。
- 書き込みは USB 経由の `esptool`（タスク `flash`）とする。実行確認は実機接続時に行う。

## 測定方法

クリーン環境で README 記載の手順通りにビルドが完走すること。
