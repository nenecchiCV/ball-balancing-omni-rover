# ui ページ・共通部品

画面実体は `src/features/<domain>/`、ルート定義は `src/app/<route>/page.tsx`（feature コンポーネントを返す薄い層）。共通部品は `src/components/`、共有状態は `src/stores/`（zustand）。

## 共通部品

- `app-shell`（`components/app-shell.tsx`）— 左ナビ（Connect / RC / Position / Odometry / Logs / Test / Tuning）、右上に接続バッジ＋動作状態バッジ（`OP_STATE` 対応の STANDBY/OPERATING/FAULT/TUNING）、ヘッダー右端に緊急停止ボタンを配置する。起動時に `watchLinkState`/`watchTelemetry`/`watchSensors`/`watchStatus`/`watchLog` を一度だけ張り、受信値を各 store へ配る。緊急停止は `robotStopOperation()` を送る（`STOP_OPERATION`：モータ即遮断→待機。故障状態からの待機復帰も兼ね、ロボット側は常時受理する。運動指令ゼロのみの `STOP` とは別機能）。
- `joystick`（`features/rc/joystick.tsx`）— `pointerdown/move/up` で動く円形スティック。中心からの正規化オフセット `(x, y) ∈ [-1, 1]` を `onChange` で返し、解放で `(0, 0)` を返す。`label` と軸の向き（`axes: "xy" | "x"`）を props で指定する。
- `odometry-map`（`features/odometry/odometry-map.tsx`）— canvas 描画。props: `pose: PoseTelemetry | null`、`trail: [x, y][]`、`target?: [x, y] | null`。原点中心・+X 右・+Y 上のワールド座標を画面座標へ写像し、グリッド・ロボット三角マーカ・軌跡線・目標点を描く。`pose` の位置系フィールドは `null` 許容で、欠落時はマーカを描かない。

## ページ（feature → ルート）

- `home-page`（`/`）— 接続状態・ロボット `StatusTelemetry` サマリ（telemetry store）・各機能ページへのリンクカードに加え、**運転準備パネル**を持つ（FR-07）。準備パネルは `opState`/`calibState` の表示、`robotCalibrateImu` 実行ボタン（実行中は結果待ち表示、`CalibResultInfo` の result/gyroBiasZ を表示）、`SET_POSE` 用の x/y/yaw 数値入力＋`robotSetPose` 送信ボタン（待機中のみ有効）、`robotStartOperation`/`robotStopOperation` ボタンを含む。各操作には実機操作ガイドを併記する（FR-10: キャリブ=水平静置、SET_POSE=所定位置へ設置、動作開始=ボール上で保持、動作停止=転倒警告）。`REJECTED_STATE`/`NOT_READY` の拒否応答はユーザーへメッセージ表示する。ホーム側の「動作停止」は確認ダイアログ付きとし、ヘッダの緊急 STOP は確認なし即時送信とする（いずれも `STOP_OPERATION`）。未接続時は `/connect` への導線を強調する。
- `connection-page`（`/connect`）— `bleScan` 実行と結果リスト（name/address/rssi、connection store の `devices`）、選択して `bleConnect`。接続後に `robotPing` で `protoVer`/`robotCaps` を表示し、`EVT_STATUS` を 1 Hz で常時購読する（FR-01）。
- `rc-page`（`/rc`）— 左 `joystick`（上下→vx、左右→vy）、右 `joystick`（左右→yaw_rate）を配置。操作中は 20 Hz `setInterval` で `robotSetVelocity` を送り、現在の送信値を数値表示する。速度指令モード遷移ボタン（`robotSetMode(1)`）を併設する（FR-02）。`opState==="operating"` でない間は操作 UI を disabled とし「動作開始が必要」を表示する。
- `position-page`（`/position`）— x, y, speed_limit の数値入力と「目標送信」ボタン（`robotSetPoseTarget`）。滞在中 `EVT_POSE` を購読し、telemetry store の `pose` を `odometry-map` へ渡して目標点・残距離を表示する（FR-03）。`opState` ガードは rc-page と同様。
- `odometry-page`（`/odometry`）— `EVT_POSE` 購読（`robotSubscribe(0x90, 20)`）。telemetry store の `pose`/`trail` を `odometry-map` へ渡し、数値表示と `robotResetOdometry`（確認ダイアログ付き、成功時 `clearTrail`）を提供する（FR-04）。
- `logs-page`（`/logs`）— `robotSubscribe(0x91, 0)` でロボットの `EVT_LOG` を購読する。表示は logs store の `entries`（robot/app ソース混在）で、レベルフィルタ・一時停止・クリアを提供する（FR-05）。
- `test-page`（`/test`）— テストコマンド送受信画面（FR-08, FR-11）。`robotPing`（PongInfo 表示）、`robotGetStatus`（RobotStatusInfo 全項目表示）、`robotTestEcho`（任意バイト列またはテキスト入力→エコー内容と rttMs 表示）の実行ボタンと結果リストを持つ。加えて**センサ生値カード**を持ち、滞在中 `robotSubscribe(EVT_SENSORS, 20)` で IMU 加速度・ジャイロ・エンコーダ値（sensors store の `latest`、20 Hz）を表示し、離脱時に `robotUnsubscribe` する。エンコーダは累積生カウントを起動時 0° 起点の符号付き累積角度 [deg]（`ENC_DEG_PER_COUNT` 換算、±360° 超も可）と deg/s レートで表示し、生カウントも併記する。本番ファームとテストファーム（`robotCaps` bit3）の両方で使い、`robotCaps` も併せて表示する。テストファーム使用時は「STM32 側で `test link` を起動してから実行」のガイドを併記する（FR-10）。
- `tuning-page`（`/tuning`）— パラメータ調整画面（FR-09, FR-10）。`robotGetParams` で調整対象パラメータを key 昇順・名称付きで一覧表示し、各行を `robotSetParam` で編集・即時反映する（RAM のみ、未保存注記あり）。「保存」=`robotSaveParams`（待機中のみ）、「既定値に戻す」=`robotResetParams`（確認ダイアログ付き）。不感帯掃引は「車体を手で保持し 3 輪を無負荷にして実行（掃引中モータ回転）」のガイド + 確認のうえ `robotTuneDeadzone` を実行し、`TuneResultInfo` の立上り・維持・推奨値を車輪ごとに表示する。方向は正転/逆転/両方向（順次 2 回掃引）のセレクトで指定し、「推奨値を適用」で方向に対応する `deadzone_comp_pos`/`deadzone_comp_neg` へ `SET_PARAM` する。IMU 特性は現在値表示 + `robotCalibrateImu` 実行 + 保存を提供する。`opState!==standby` 時は各操作を disabled とする。

## 内部状態

- 共有データ（接続状態・pose・status・軌跡・ログ）は zustand store。ページローカル state（`useState`/`useRef`）は入力値・送信値・表示制御のみ。
- ロボット向け購読（`SUBSCRIBE` フレーム）はページ滞在中のみ有効とし、離脱時に `robotUnsubscribe` してロボット側送出を止める。

## 備考

- 未接続時は指令系 UI を `disabled` とし、操作不能を見た目で示す（NFR-02）。
- スタイルは Tailwind v4 のユーティリティと `globals.css` の共通クラス（`.card`/`.row`/`.btn`/`.btn-secondary`/`.num`/`.muted`/`.list`）を使う。
