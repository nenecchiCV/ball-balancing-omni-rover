import { create } from "zustand";

import type { SensorTelemetry } from "@/lib/types";

// i32 ラップを吸収した累積カウント差分。
const wrapDelta32 = (d: number) => {
  const m = ((d % 4294967296) + 4294967296) % 4294967296;
  return m >= 2147483648 ? m - 4294967296 : m;
};

interface SensorsStore {
  /** 最新の EVT_SENSORS フレーム。 */
  latest: SensorTelemetry | null;
  /** 受信フレーム数（接続ごとにリセットせず累積）。 */
  count: number;
  /** フレーム間差分から求めたエンコーダカウントレート [count/s]。 */
  encRate: [number, number, number];
  push: (s: SensorTelemetry) => void;
}

export const useSensorsStore = create<SensorsStore>((set) => ({
  latest: null,
  count: 0,
  encRate: [0, 0, 0],
  push: (s) =>
    set((st) => {
      let encRate = st.encRate;
      const prev = st.latest;
      if (prev) {
        const dt = (s.tMs - prev.tMs) / 1000;
        if (dt > 0) {
          encRate = s.enc.map(
            (c, i) => wrapDelta32(c - prev.enc[i]) / dt,
          ) as [number, number, number];
        }
      }
      return { latest: s, count: st.count + 1, encRate };
    }),
}));
