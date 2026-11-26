# 詳細設計索引

詳細設計書は `spec/detail_design/<領域>/<component>.md` に 1 コンポーネント 1 ファイルで配置する。領域はソースモジュール構造に対応し、firmware の層モデルは適用しない（`basic_design.md` 参照）。

## 領域とソース構造の対応

### バックエンド（`src-tauri/src/`）

| 詳細設計ディレクトリ | ソース | Tauri 依存 |
|---|---|---|
| `protocol/` | `protocol/` | なし |
| `ble/` | `ble/` | なし |
| `robot/` | `robot/` | なし |
| `ipc/` | `commands.rs`・`error.rs`・`state.rs`・`lib.rs` | あり |

### フロントエンド（`src/`）

| 詳細設計ディレクトリ | ソース | 備考 |
|---|---|---|
| `frontend/` | `src/lib/`・`src/stores/` | IPC 境界と状態管理 |
| `ui/` | `src/components/`・`src/features/`・`src/app/` | 画面 |

エントリは `src-tauri/src/main.rs` と `src/main.tsx`（`src/app/app.tsx` でルート定義）。

## コンポーネント一覧

### protocol
- [frame](protocol/frame.md)
- [message](protocol/message.md)

### ble
- [nus](ble/nus.md)

### robot
- [session](robot/session.md)

### ipc
- [commands](ipc/commands.md)

### frontend
- [ipc](frontend/ipc.md)
- [stores](frontend/stores.md)

### ui
- [pages](ui/pages.md)

## インターフェース記法

各ファイルは「公開型」「公開関数（シグネチャ + 説明）」「内部状態」「備考」の節で構成する。関数の実装詳細は文章で説明し、コード断片は必要最小限とする。
