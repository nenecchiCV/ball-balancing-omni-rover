import { create } from "zustand";

// ロボット調整パラメータ由来の指令上限（FR-09）。
// GET_PARAMS で取得した controller.max_speed（key 0x60）と派生の
// 無滑り理論上限（key 0x61）を保持し、指令 UI のクランプに使う。
// 未接続・未取得時はシム検証値 0.35 m/s を既定とする。
export const PARAM_KEY_MAX_SPEED = 0x60;
export const PARAM_KEY_THEORY_MAX_SPEED = 0x61;
export const DEFAULT_MAX_SPEED = 0.35; // m/s（シム検証値）

interface ParamsStore {
  maxSpeed: number;
  maxSpeedTheory: number | null;
  setLimits: (maxSpeed: number, theory: number | null) => void;
}

export const useParamsStore = create<ParamsStore>((set) => ({
  maxSpeed: DEFAULT_MAX_SPEED,
  maxSpeedTheory: null,
  setLimits: (maxSpeed, maxSpeedTheory) =>
    set({
      maxSpeed:
        Number.isFinite(maxSpeed) && maxSpeed > 0
          ? maxSpeed
          : DEFAULT_MAX_SPEED,
      maxSpeedTheory,
    }),
}));
