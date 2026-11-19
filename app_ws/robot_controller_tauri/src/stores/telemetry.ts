import { create } from "zustand";

import type { PoseTelemetry, StatusTelemetry } from "@/lib/types";

const TRAIL_CAP = 20000;

interface TelemetryStore {
  pose: PoseTelemetry | null;
  status: StatusTelemetry | null;
  /** 軌跡 [x, y]。原点リセット・切断時にクリアする。 */
  trail: [number, number][];
  pushPose: (p: PoseTelemetry) => void;
  setStatus: (s: StatusTelemetry) => void;
  clearTrail: () => void;
}

export const useTelemetryStore = create<TelemetryStore>((set) => ({
  pose: null,
  status: null,
  trail: [],
  pushPose: (p) =>
    set((s) => {
      // 位置フィールド欠落フレームは軌跡へ追加しない
      const trail =
        p.x == null || p.y == null
          ? s.trail
          : [...s.trail, [p.x, p.y] as [number, number]].slice(-TRAIL_CAP);
      return { pose: p, trail };
    }),
  setStatus: (status) => set({ status }),
  clearTrail: () => set({ trail: [], pose: null }),
}));
