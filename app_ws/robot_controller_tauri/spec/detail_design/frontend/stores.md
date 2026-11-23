# stores（zustand）

UI 状態は zustand store に集約する。`app-shell` の `watch_*` 購読が唯一の書き込み起点で、ページ・共通部品は selector で購読する。高頻度テレメトリとログを別 store に分け、無関係な再レンダを抑える。

## connection.ts

```ts
interface ConnectionStore {
  linkState: LinkState;              // watchLinkState が更新
  devices: DiscoveredDevice[];       // スキャン結果（connect ページが書込）
  setLinkState(s): void;
  setDevices(d): void;
}
export const useConnected = () => /* linkState === "connected" 専用 selector */
```

## telemetry.ts

```ts
interface TelemetryStore {
  pose: PoseTelemetry | null;        // 最新 EVT_POSE
  status: StatusTelemetry | null;    // 最新 EVT_STATUS（opState/calibState を含む。動作状態バッジ・準備パネル・操作ガードはここから導出）
  trail: [number, number][];         // 軌跡リング（20000 点上限）
  pushPose(p): void;   // x/y 欠落フレームは軌跡へ追加しない
  setStatus(s): void;
  clearTrail(): void;  // 原点リセット時
}
```

## params.ts

```ts
interface ParamsStore {
  maxSpeed: number;             // ロボット controller.max_speed（key 0x60）。指令 UI の速度クランプに使用。未取得時は既定 0.4 m/s
  maxSpeedTheory: number | null; // key 0x61 の無滑り理論上限（表示用）
  setLimits(maxSpeed, theory): void;
}
```

- 書き込み起点は `app-shell` の接続時 `robotGetParams` および tuning ページのパラメータ取得・`SET_PARAM(0x60)` 反映時。
- `watch_*` 購読ではなくコマンド応答駆動のため、「`watch_*` が唯一の書き込み起点」の原則の例外とする。

## logs.ts

```ts
interface LogStore {
  entries: LogMessage[];             // 5000 件リング、robot/app ソース混在
  push(level, text, source = "app"): void;  // UI 側エラー記録
  pushMsg(msg): void;                // watchLog 受信
  clear(): void;
}
```

## 備考

- 購読解除は store 側では管理しない（`app-shell` がアプリ寿命で張り続ける）。ページ個別のロボット購読（`robotSubscribe` フレーム）はページの `useEffect` で制御する。
- React Context は使わない（高頻度更新の再レンダ制御が困難なため）。
