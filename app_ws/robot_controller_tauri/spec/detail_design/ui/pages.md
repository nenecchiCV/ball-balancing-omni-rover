# ui ページ・共通部品

画面実体は `src/features/<domain>/`、ルート定義は `src/app/<route>/page.tsx`（feature コンポーネントを返す薄い層）。共通部品は `src/components/`、共有状態は `src/stores/`（zustand）。

## 共通部品

- `app-shell`（`components/app-shell.tsx`）— 左ナビ（Connect / RC / Position / Odometry / Logs / Test）、右上に接続バッジ＋動作状態バッジ（`OP_STATE` 対応の STANDBY/OPERATING/FAULT）、ヘッダー右端に緊急停止ボタンを配置する。起動時に `watchLinkState`/`watchTelemetry`/`watchStatus`/`watchLog` を一度だけ張り、受信値を各 store へ配る。緊急停止は `robotStop()` を送る（運動指令ゼロ化＝その場保持であり、動作停止 `robotStopOperation` とは別操作）。
- `joystick`（`features/rc/joystick.tsx`）— `pointerdown/move/up` で動く円形スティック。中心からの正規化オフセット `(x, y) ∈ [-1, 1]` を `onChange` で返し、解放で `(0, 0)` を返す。`label` と軸の向き（`axes: "xy" | "x"`）を props で指定する。
- `odometry-map`（`features/odometry/odometry-map.tsx`）— canvas 描画。props: `pose: PoseTelemetry | null`、`trail: [x, y][]`、`target?: [x, y] | null`。原点中心・+X 右・+Y 上のワールド座標を画面座標へ写像し、グリッド・ロボット三角マーカ・軌跡線・目標点を描く。`pose` の位置系フィールドは `null` 許容で、欠落時はマーカを描かない。

## ページ（feature → ルート）

- `home-page`（`/`）— 接続状態・ロボット `StatusTelemetry` サマリ（telemetry store）・各機能ページへのリンクカードに加え、**運転準備パネル**を持つ（FR-07）。準備パネルは `opState`/`calibState` の表示、`robotCalibrateImu` 実行ボタン（実行中は結果待ち表示、`CalibResultInfo` の result/gyroBiasZ を表示）、`SET_POSE` 用の x/y/yaw 数値入力＋`robotSetPose` 送信ボタン（待機中のみ有効）、`robotStartOperation`/`robotStopOperation` ボタンを含む。`REJECTED_STATE`/`NOT_READY` の拒否応答はユーザーへメッセージ表示する。`STOP_OPERATION` は「STOP → 静止確認 → 停止」ガイドに従い、確認ダイアログ付きとする。未接続時は `/connect` への導線を強調する。
- `connection-page`（`/connect`）— `bleScan` 実行と結果リスト（name/address/rssi、connection store の `devices`）、選択して `bleConnect`。接続後に `robotPing` で `protoVer`/`robotCaps` を表示し、`EVT_STATUS` を 1 Hz で常時購読する（FR-01）。
- `rc-page`（`/rc`）— 左 `joystick`（上下→vx、左右→vy）、右 `joystick`（左右→yaw_rate）を配置。操作中は 20 Hz `setInterval` で `robotSetVelocity` を送り、現在の送信値を数値表示する。速度指令モード遷移ボタン（`robotSetMode(1)`）を併設する（FR-02）。`opState==="operating"` でない間は操作 UI を disabled とし「動作開始が必要」を表示する。
- `position-page`（`/position`）— x, y, speed_limit の数値入力と「目標送信」ボタン（`robotSetPoseTarget`）。滞在中 `EVT_POSE` を購読し、telemetry store の `pose` を `odometry-map` へ渡して目標点・残距離を表示する（FR-03）。`opState` ガードは rc-page と同様。
- `odometry-page`（`/odometry`）— `EVT_POSE` 購読（`robotSubscribe(0x90, 20)`）。telemetry store の `pose`/`trail` を `odometry-map` へ渡し、数値表示と `robotResetOdometry`（確認ダイアログ付き、成功時 `clearTrail`）を提供する（FR-04）。
- `logs-page`（`/logs`）— `robotSubscribe(0x91, 0)` でロボットの `EVT_LOG` を購読する。表示は logs store の `entries`（robot/app ソース混在）で、レベルフィルタ・一時停止・クリアを提供する（FR-05）。
- `test-page`（`/test`）— テストコマンド送受信画面（FR-08）。`robotPing`（PongInfo 表示）、`robotGetStatus`（RobotStatusInfo 全項目表示）、`robotTestEcho`（任意バイト列またはテキスト入力→エコー内容と rttMs 表示）の実行ボタンと結果リストを持つ。本番ファームとテストファーム（`robotCaps` bit3）の両方で使い、`robotCaps` も併せて表示する。

## 内部状態

- 共有データ（接続状態・pose・status・軌跡・ログ）は zustand store。ページローカル state（`useState`/`useRef`）は入力値・送信値・表示制御のみ。
- ロボット向け購読（`SUBSCRIBE` フレーム）はページ滞在中のみ有効とし、離脱時に `robotUnsubscribe` してロボット側送出を止める。

## 備考

- 未接続時は指令系 UI を `disabled` とし、操作不能を見た目で示す（NFR-02）。
- スタイルは Tailwind v4 のユーティリティと `globals.css` の共通クラス（`.card`/`.row`/`.btn`/`.btn-secondary`/`.num`/`.muted`/`.list`）を使う。
