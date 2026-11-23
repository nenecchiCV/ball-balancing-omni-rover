"use client";

// FR-01: BLE スキャン・接続・切断・PING 照合。

import { useState } from "react";

import { formatError } from "@/lib/error";
import {
  bleConnect,
  bleDisconnect,
  bleScan,
  robotPing,
  robotSubscribe,
} from "@/lib/ipc";
import { EVT_STATUS } from "@/lib/limits";
import type { PongInfo } from "@/lib/types";
import { useConnectionStore } from "@/stores/connection";
import { useLogStore } from "@/stores/logs";

export function ConnectionPage() {
  const { linkState, devices, setDevices } = useConnectionStore();
  const pushLog = useLogStore((s) => s.push);
  const [pong, setPong] = useState<PongInfo | null>(null);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const connected = linkState === "connected";

  const scan = async () => {
    setBusy(true);
    setError(null);
    try {
      setDevices(await bleScan(5000));
    } catch (e) {
      setError(formatError(e));
      pushLog(3, `スキャン失敗: ${formatError(e)}`);
    } finally {
      setBusy(false);
    }
  };

  const connect = async (address: string) => {
    setBusy(true);
    setError(null);
    try {
      await bleConnect(address);
      setPong(await robotPing().catch(() => null));
      // EVT_STATUS は常時購読（ホームのステータス表示用、1 Hz で軽量）
      robotSubscribe(EVT_STATUS, 1).catch(() => {});
    } catch (e) {
      setError(formatError(e));
      pushLog(3, `接続失敗: ${formatError(e)}`);
    } finally {
      setBusy(false);
    }
  };

  return (
    <div>
      <h1 className="mb-4 text-xl">接続</h1>
      <div className="card">
        <div className="row">
          <button className="btn" onClick={scan} disabled={busy || connected}>
            スキャン
          </button>
          <button
            className="btn-secondary"
            onClick={() =>
              bleDisconnect().catch((e) => pushLog(3, formatError(e)))
            }
            disabled={!connected}
          >
            切断
          </button>
          <span className="muted">既定フィルタ: uwc-esp32c6</span>
        </div>
        {error && <p className="mt-2 text-err">{error}</p>}
      </div>

      <div className="card">
        <table className="list">
          <thead>
            <tr>
              <th>デバイス名</th>
              <th>アドレス</th>
              <th>RSSI</th>
              <th></th>
            </tr>
          </thead>
          <tbody>
            {devices.map((d) => (
              <tr key={d.address}>
                <td>{d.name}</td>
                <td>{d.address}</td>
                <td>{d.rssi ?? "-"}</td>
                <td>
                  <button
                    className="btn"
                    onClick={() => connect(d.address)}
                    disabled={busy || connected}
                  >
                    接続
                  </button>
                </td>
              </tr>
            ))}
            {devices.length === 0 && (
              <tr>
                <td colSpan={4} className="muted">
                  スキャン結果なし
                </td>
              </tr>
            )}
          </tbody>
        </table>
      </div>

      {connected && (
        <div className="card">
          <p>
            接続済。proto_ver: {pong ? pong.protoVer : "(PING 未応答/照合中)"}
            {pong && ` / caps: 0x${pong.robotCaps.toString(16)}`}
          </p>
        </div>
      )}
    </div>
  );
}
