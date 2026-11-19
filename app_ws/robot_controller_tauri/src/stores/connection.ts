import { create } from "zustand";

import type { DiscoveredDevice, LinkState } from "@/lib/types";

interface ConnectionStore {
  linkState: LinkState;
  devices: DiscoveredDevice[];
  setLinkState: (s: LinkState) => void;
  setDevices: (d: DiscoveredDevice[]) => void;
}

export const useConnectionStore = create<ConnectionStore>((set) => ({
  linkState: "disconnected",
  devices: [],
  setLinkState: (linkState) => set({ linkState }),
  setDevices: (devices) => set({ devices }),
}));

/** linkState === "connected" の購読用セレクタ */
export const useConnected = () =>
  useConnectionStore((s) => s.linkState === "connected");
