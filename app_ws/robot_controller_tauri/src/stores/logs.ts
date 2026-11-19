import { create } from "zustand";

import type { LogMessage, LogSource } from "@/lib/types";

const LOG_CAP = 5000;

interface LogStore {
  entries: LogMessage[];
  push: (level: number, text: string, source?: LogSource) => void;
  pushMsg: (msg: LogMessage) => void;
  clear: () => void;
}

export const useLogStore = create<LogStore>((set) => ({
  entries: [],
  push: (level, text, source = "app") =>
    set((s) => ({
      entries: [...s.entries, { level, text, source }].slice(-LOG_CAP),
    })),
  pushMsg: (msg) =>
    set((s) => ({ entries: [...s.entries, msg].slice(-LOG_CAP) })),
  clear: () => set({ entries: [] }),
}));
