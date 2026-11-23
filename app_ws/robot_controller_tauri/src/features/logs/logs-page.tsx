"use client";

// FR-05: EVT_LOG 購読 → ログリスト表示。レベルフィルタ・一時停止・クリア。
// app ソース（接続/切断/エラー）も同一リストに記録される（watchLog が一元収集）。

import { useEffect, useRef, useState } from "react";

import { formatError } from "@/lib/error";
import { robotSubscribe, robotUnsubscribe } from "@/lib/ipc";
import { EVT_LOG, LOG_LEVEL_NAMES } from "@/lib/limits";
import { useConnected } from "@/stores/connection";
import { useLogStore } from "@/stores/logs";

const LEVEL_COLOR = ["text-muted", "text-text", "text-warn", "text-err"];

export function LogsPage() {
  const connected = useConnected();
  const { entries, push, clear } = useLogStore();
  const [minLevel, setMinLevel] = useState(0);
  const [paused, setPaused] = useState(false);
  const snapshot = useRef<typeof entries>([]);
  const bottomRef = useRef<HTMLDivElement>(null);

  // ロボットの EVT_LOG ストリームを滞在中のみ購読する。
  useEffect(() => {
    if (!connected) return;
    robotSubscribe(EVT_LOG, 0).catch((e) => push(3, `LOG 購読失敗: ${formatError(e)}`));
    return () => {
      robotUnsubscribe(EVT_LOG).catch(() => {});
    };
  }, [connected, push]);

  // 一時停止は受信保持のまま表示を固定する。
  const shown = paused ? snapshot.current : entries;
  useEffect(() => {
    if (paused) snapshot.current = entries;
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [paused]);

  useEffect(() => {
    bottomRef.current?.scrollIntoView({ behavior: "auto" });
  }, [shown.length]);

  const filtered = shown.filter((l) => l.level >= minLevel);

  return (
    <div>
      <h1 className="mb-4 text-xl">ログ</h1>
      <div className="card">
        <div className="row">
          <label>
            レベル{" "}
            <select
              className="num w-auto"
              value={minLevel}
              onChange={(e) => setMinLevel(Number(e.target.value))}
            >
              {LOG_LEVEL_NAMES.map((n, i) => (
                <option key={n} value={i}>
                  {n}+
                </option>
              ))}
            </select>
          </label>
          <button className="btn-secondary" onClick={() => setPaused((p) => !p)}>
            {paused ? "再開" : "一時停止"}
          </button>
          <button className="btn-secondary" onClick={clear}>
            クリア
          </button>
          <span className="muted">{filtered.length} 件</span>
        </div>
      </div>
      <div className="h-[60vh] overflow-y-auto whitespace-pre-wrap rounded-lg border border-line bg-[#0e1218] p-2 font-mono text-[13px]">
        {filtered.map((l, i) => (
          <div key={i}>
            <span className="text-accent">[{l.source.toUpperCase()}]</span>{" "}
            <span className={LEVEL_COLOR[l.level] ?? "text-text"}>
              [{LOG_LEVEL_NAMES[l.level] ?? l.level}] {l.text}
            </span>
          </div>
        ))}
        <div ref={bottomRef} />
      </div>
    </div>
  );
}
