"use client";

// 全画面共通シェル: ナビ・接続バッジ・緊急停止・バックエンド購読の一元起動。
// watch_* はここで 1 回だけ張り、受信データは各 zustand store へ配る。

import Link from "next/link";
import { usePathname } from "next/navigation";
import { useEffect, useRef, type ReactNode } from "react";

import {
  robotStop,
  watchLinkState,
  watchLog,
  watchStatus,
  watchTelemetry,
} from "@/lib/ipc";
import type { LinkState } from "@/lib/types";
import { useConnectionStore } from "@/stores/connection";
import { useLogStore } from "@/stores/logs";
import { useTelemetryStore } from "@/stores/telemetry";

const NAV = [
  { href: "/connect", label: "Connect" },
  { href: "/rc", label: "RC" },
  { href: "/position", label: "Position" },
  { href: "/odometry", label: "Odometry" },
  { href: "/logs", label: "Logs" },
];

const STATE_LABEL: Record<LinkState, string> = {
  disconnected: "切断",
  scanning: "スキャン中",
  connecting: "接続中",
  connected: "接続済",
};

export function AppShell({ children }: { children: ReactNode }) {
  const pathname = usePathname();
  const linkState = useConnectionStore((s) => s.linkState);
  const pushLog = useLogStore((s) => s.push);
  const stopping = useRef(false);

  // バックエンド broadcast の購読を起動時に一度だけ張る。
  useEffect(() => {
    const { setLinkState } = useConnectionStore.getState();
    const { pushPose, setStatus } = useTelemetryStore.getState();
    const { pushMsg } = useLogStore.getState();
    watchLinkState(setLinkState);
    watchTelemetry(pushPose);
    watchStatus(setStatus);
    watchLog(pushMsg);
  }, []);

  const emergencyStop = async () => {
    if (stopping.current) return;
    stopping.current = true;
    try {
      await robotStop();
    } catch (e) {
      pushLog(3, `STOP 送信失敗: ${String(e)}`);
    } finally {
      stopping.current = false;
    }
  };

  return (
    <div className="flex h-screen">
      <aside className="flex w-[180px] shrink-0 flex-col gap-1 border-r border-line bg-panel p-3">
        <div className="px-2.5 pb-3 pt-1.5 text-[13px] font-bold text-muted">
          Robot Controller
        </div>
        {NAV.map((n) => (
          <Link
            key={n.href}
            href={n.href}
            className={`rounded-md px-2.5 py-2 text-sm text-text no-underline hover:bg-[#1f2630] ${
              pathname === n.href ? "bg-[#23314a] text-white" : ""
            }`}
          >
            {n.label}
          </Link>
        ))}
      </aside>
      <div className="flex min-w-0 flex-1 flex-col">
        <header className="flex items-center justify-between border-b border-line bg-panel px-4 py-2.5">
          <span
            className={`rounded-[10px] px-2.5 py-1 text-[13px] ${
              linkState === "connected"
                ? "bg-[#1d4d2b] text-ok"
                : linkState === "disconnected"
                  ? "bg-[#333a44] text-text"
                  : "bg-[#4d3d1d] text-warn"
            }`}
          >
            {STATE_LABEL[linkState]}
          </span>
          <button
            className="cursor-pointer rounded-md border border-[#a33] bg-[#7a1f1f] px-7 py-2 text-[15px] font-bold text-white disabled:cursor-default disabled:opacity-40"
            onClick={emergencyStop}
            disabled={linkState !== "connected"}
          >
            STOP
          </button>
        </header>
        <main className="flex-1 overflow-auto p-5">{children}</main>
      </div>
    </div>
  );
}
