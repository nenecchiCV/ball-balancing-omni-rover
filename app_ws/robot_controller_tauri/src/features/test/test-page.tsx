"use client";

// FR-08: テストコマンド送受信。PING / GET_STATUS / TEST_ECHO の往復を検証する。
// FR-11: EVT_SENSORS 購読で IMU・エンコーダの生データを表示する。

import { useEffect, useState } from "react";

import { formatError } from "@/lib/error";
import {
  robotGetStatus,
  robotPing,
  robotSubscribe,
  robotTestEcho,
  robotUnsubscribe,
} from "@/lib/ipc";
import {
  ENC_DEG_PER_COUNT,
  EVT_SENSORS,
  SENSORS_SUBSCRIBE_HZ,
} from "@/lib/limits";
import { useConnected } from "@/stores/connection";
import { useLogStore } from "@/stores/logs";
import { useSensorsStore } from "@/stores/sensors";

const CAPS_NAMES: [number, string][] = [
  [0x01, "位置制御"],
  [0x02, "オドメトリ"],
  [0x04, "ログ"],
  [0x08, "テストビルド"],
  [0x10, "運転管理"],
];

const OP_NAMES = ["待機", "動作中", "故障", "調整中"] as const;
const MODE_NAMES = ["スタンバイ", "速度指令", "位置制御"] as const;

// ASCII 文字列 ⇔ バイト列（ペイロードは 1-63 バイト）。
const toBytes = (s: string) => Array.from(new TextEncoder().encode(s));
const toText = (b: number[]) => new TextDecoder().decode(new Uint8Array(b));

export function TestPage() {
  const connected = useConnected();
  const pushLog = useLogStore((s) => s.push);
  const [echoText, setEchoText] = useState("hello");
  const [result, setResult] = useState<string[]>([]);
  const note = (s: string) => setResult((r) => [...r.slice(-19), s]);

  // FR-11: 滞在中は EVT_SENSORS を購読する（受信値はグローバル store 経由）。
  const [sensorsErr, setSensorsErr] = useState<string | null>(null);
  const sensors = useSensorsStore((s) => s.latest);
  const sensorsCount = useSensorsStore((s) => s.count);
  const encRate = useSensorsStore((s) => s.encRate);
  useEffect(() => {
    if (!connected) {
      setSensorsErr(null);
      return;
    }
    robotSubscribe(EVT_SENSORS, SENSORS_SUBSCRIBE_HZ).catch((e) =>
      setSensorsErr(formatError(e)),
    );
    return () => {
      robotUnsubscribe(EVT_SENSORS).catch(() => {});
    };
  }, [connected]);

  const ping = async () => {
    try {
      const t0 = performance.now();
      const p = await robotPing();
      const rtt = (performance.now() - t0).toFixed(0);
      const caps = CAPS_NAMES.filter(([bit]) => p.robotCaps & bit)
        .map(([, name]) => name)
        .join(", ");
      note(
        `PONG (${rtt} ms): proto_ver=${p.protoVer} caps=0x${p.robotCaps
          .toString(16)
          .padStart(2, "0")} [${caps}]`,
      );
    } catch (e) {
      note(`PING 失敗: ${formatError(e)}`);
      pushLog(3, `PING 失敗: ${formatError(e)}`);
    }
  };

  const getStatus = async () => {
    try {
      const s = await robotGetStatus();
      note(
        `STATUS: op=${OP_NAMES[s.opState] ?? s.opState} mode=${
          MODE_NAMES[s.mode] ?? s.mode
        } enable=${s.enable} estop=${s.estop} imu=${s.imuOk} calib=${
          s.calibState
        } uptime=${s.uptimeMs} ms`,
      );
    } catch (e) {
      note(`GET_STATUS 失敗: ${formatError(e)}`);
      pushLog(3, `GET_STATUS 失敗: ${formatError(e)}`);
    }
  };

  const testEcho = async () => {
    const data = toBytes(echoText);
    if (data.length === 0 || data.length > 63) {
      note(`エコー送信不可: ${data.length} バイト（1-63 バイト）`);
      return;
    }
    try {
      const t0 = performance.now();
      const r = await robotTestEcho(data);
      const rtt = (performance.now() - t0).toFixed(0);
      const match = data.length === r.data.length && data.every((b, i) => b === r.data[i]);
      note(
        `ECHO (${rtt} ms): ${match ? "一致" : "不一致"} "${toText(r.data)}"`,
      );
    } catch (e) {
      note(`TEST_ECHO 失敗: ${formatError(e)}`);
      pushLog(3, `TEST_ECHO 失敗: ${formatError(e)}`);
    }
  };

  return (
    <div>
      <h1 className="mb-4 text-xl">テストコマンド</h1>
      <div className="card">
        <div className="row">
          <button className="btn" onClick={ping} disabled={!connected}>
            PING
          </button>
          <button className="btn" onClick={getStatus} disabled={!connected}>
            GET_STATUS
          </button>
          <input
            className="num w-40"
            value={echoText}
            onChange={(e) => setEchoText(e.target.value)}
            placeholder="echo 文字列"
          />
          <button className="btn" onClick={testEcho} disabled={!connected}>
            TEST_ECHO
          </button>
        </div>
        <p className="muted mt-2">
          TEST_ECHO は ASCII 文字列（1-63 バイト）を往復させる。STM32
          テストファーム使用時は、先に STM32 側シェルで
          <code>test link</code> を起動してから実行すること
          （`caps` のテストビルドビットで識別可能）。
        </p>
      </div>
      <div className="card">
        <h2 className="mb-2 text-base">
          センサ生データ（EVT_SENSORS, {SENSORS_SUBSCRIBE_HZ} Hz）
        </h2>
        {sensorsErr != null && (
          <p className="muted">
            購読失敗: {sensorsErr}（EVT_SENSORS 未対応のファームの可能性）
          </p>
        )}
        {sensors == null ? (
          <p className="muted">{connected ? "受信待ち" : "未接続"}</p>
        ) : (
          <>
            <table className="list mb-2">
              <thead>
                <tr>
                  <th></th>
                  <th>x</th>
                  <th>y</th>
                  <th>z</th>
                </tr>
              </thead>
              <tbody>
                <tr>
                  <td>accel [m/s²]</td>
                  {sensors.accel.map((v, i) => (
                    <td key={i} className="font-mono">
                      {v?.toFixed(4) ?? "-"}
                    </td>
                  ))}
                </tr>
                <tr>
                  <td>gyro [rad/s]</td>
                  {sensors.gyro.map((v, i) => (
                    <td key={i} className="font-mono">
                      {v?.toFixed(4) ?? "-"}
                    </td>
                  ))}
                </tr>
              </tbody>
            </table>
            <table className="list mb-2">
              <thead>
                <tr>
                  <th>wheel</th>
                  <th>angle [deg]</th>
                  <th>rate [deg/s]</th>
                  <th>count</th>
                </tr>
              </thead>
              <tbody>
                {sensors.enc.map((c, i) => (
                  <tr key={i}>
                    <td>{i}</td>
                    <td className="font-mono">
                      {(c * ENC_DEG_PER_COUNT).toFixed(1)}
                    </td>
                    <td className="font-mono">
                      {(encRate[i] * ENC_DEG_PER_COUNT).toFixed(0)}
                    </td>
                    <td className="font-mono">{c}</td>
                  </tr>
                ))}
              </tbody>
            </table>
            <p className="muted">
              t_ms={sensors.tMs} ・ 受信 {sensorsCount} フレーム ・
              accel/gyro は機体軸写像後・推定処理前の読出し値 ・
              angle は起動時 0° 起点の符号付き累積角（±360° 超も可）
            </p>
          </>
        )}
      </div>
      <div className="card">
        <h2 className="mb-2 text-base">結果</h2>
        {result.length === 0 ? (
          <p className="muted">未実行</p>
        ) : (
          <pre className="whitespace-pre-wrap text-[13px] leading-6">
            {result.join("\n")}
          </pre>
        )}
      </div>
    </div>
  );
}
