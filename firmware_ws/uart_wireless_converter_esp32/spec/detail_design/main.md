# 詳細設計索引

詳細設計書は `spec/detail_design/<層>/<component>.md` に 1 コンポーネント 1 ファイルで配置する。`src/` のディレクトリ構造はこの層構造に対応する。

## 層とソース構造の対応

| 層 | 詳細設計ディレクトリ | ソースディレクトリ |
|---|---|---|
| common | `common/` | `src/common/` |
| peripheral | `peripheral/` | `src/peripheral/` |
| device | `device/` | `src/device/` |
| feature | `feature/` | `src/feature/` |
| manager | `manager/` | `src/manager/` |

`src/main.cpp` は `src/` 直下。

## コンポーネント一覧

### peripheral
- [host_uart](peripheral/host_uart.md)
- [debug_console](peripheral/debug_console.md)

### device
- [link](device/link.md)

### feature
- [uart_bridge](feature/uart_bridge.md)
- [link_monitor](feature/link_monitor.md)

### manager
- [converter_application](manager/converter_application.md)

## インターフェース記法

各ファイルは「公開型」「公開関数（シグネチャ + 説明）」「内部状態」「備考」の節で構成する。関数の実装詳細は文章で説明し、コード断片は必要最小限とする。
