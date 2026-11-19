"use client";

import Link from "next/link";

import { useConnected } from "@/stores/connection";
import { useTelemetryStore } from "@/stores/telemetry";

const MODE_NAMES = ["スタンバイ", "速度指令", "位置制御"] as const;

export function HomePage() {
  const connected = useConnected();
  const status = useTelemetryStore((s) => s.status);

  return (
    <div>
      <h1 className="mb-4 text-xl">ホーム</h1>
      <div className="card">
        {connected ? (
          status ? (
            <table className="list">
              <tbody>
                <tr>
                  <th>制御モード</th>
                  <td>{MODE_NAMES[status.mode] ?? `不明 (${status.mode})`}</td>
                </tr>
                <tr>
                  <th>イネーブル</th>
                  <td>{status.enable ? "ON" : "OFF"}</td>
                </tr>
                <tr>
                  <th>E-Stop</th>
                  <td>{status.estop ? "要求中" : "なし"}</td>
                </tr>
                <tr>
                  <th>IMU</th>
                  <td>{status.imuOk ? "正常" : "異常"}</td>
                </tr>
              </tbody>
            </table>
          ) : (
            <p className="muted">STATUS 受信待ち（EVT_STATUS 購読で更新）</p>
          )
        ) : (
          <p className="muted">
            ロボット未接続。<Link href="/connect">Connect</Link> から接続する。
          </p>
        )}
      </div>
      <div className="flex gap-4">
        {[
          { href: "/rc", title: "RC", desc: "2 本スティックで速度指令" },
          { href: "/position", title: "Position", desc: "目標位置への移動" },
          { href: "/odometry", title: "Odometry", desc: "自己位置・軌跡" },
          { href: "/logs", title: "Logs", desc: "ログメッセージ" },
        ].map((c) => (
          <Link key={c.href} href={c.href} className="card flex-1 no-underline">
            <h3 className="text-text">{c.title}</h3>
            <p className="muted">{c.desc}</p>
          </Link>
        ))}
      </div>
    </div>
  );
}
