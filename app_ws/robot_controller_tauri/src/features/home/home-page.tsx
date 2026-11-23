"use client";

// FR-07: 運転開始・停止と開始前準備（IMU キャリブレーション・位置設定）。

import Link from "next/link";
import { useState } from "react";

import { formatError } from "@/lib/error";
import {
  robotCalibrateImu,
  robotSetPose,
  robotStartOperation,
  robotStopOperation,
} from "@/lib/ipc";
import { CALIB_STATE, OP_STATE, READY_MASK } from "@/lib/types";
import { confirmAsync } from "@/stores/confirm";
import { useConnected } from "@/stores/connection";
import { useLogStore } from "@/stores/logs";
import { useTelemetryStore } from "@/stores/telemetry";

const MODE_NAMES = ["スタンバイ", "速度指令", "位置制御"] as const;
const OP_NAMES = ["待機", "動作中", "故障", "調整中"] as const;
const CALIB_NAMES = ["未実施", "実行中", "完了", "失敗"] as const;

export function HomePage() {
  const connected = useConnected();
  const status = useTelemetryStore((s) => s.status);
  const pushLog = useLogStore((s) => s.push);
  const opState = status?.opState;
  const standby = opState === OP_STATE.standby;
  const [px, setPx] = useState("0.0");
  const [py, setPy] = useState("0.0");
  const [pyaw, setPyaw] = useState("0.0");
  const [calibNote, setCalibNote] = useState<string | null>(null);

  const calibrate = async () => {
    setCalibNote("キャリブレーション実行中…（ロボットを静止させる）");
    try {
      const r = await robotCalibrateImu();
      const ok = r.result === 0;
      setCalibNote(
        `${ok ? "完了" : "失敗"}: gyro_bias_z=${r.gyroBiasZ?.toFixed(6) ?? "?"} rad/s`,
      );
      pushLog(ok ? 1 : 3, `IMU キャリブレーション${ok ? "完了" : "失敗"}`);
    } catch (e) {
      setCalibNote(null);
      pushLog(3, `キャリブレーション失敗: ${formatError(e)}`);
    }
  };

  const setPose = async () => {
    const [x, y, yaw] = [px, py, pyaw].map(parseFloat);
    if ([x, y, yaw].some(Number.isNaN)) {
      pushLog(2, "位置設定値が数値ではない");
      return;
    }
    try {
      await robotSetPose(x, y, yaw);
      pushLog(1, `初期位置設定: (${x.toFixed(2)}, ${y.toFixed(2)}, ${yaw.toFixed(2)})`);
    } catch (e) {
      pushLog(3, `位置設定拒否: ${formatError(e)}`);
    }
  };

  const startOp = async () => {
    if (!(await confirmAsync("モータ駆動を開始します。よろしいですか？"))) return;
    try {
      await robotStartOperation();
      pushLog(1, "動作開始を受理");
    } catch (e) {
      pushLog(3, `動作開始拒否: ${formatError(e)}${startBlockers()}`);
    }
  };

  // START_OPERATION 拒否時に未充足条件を列挙する（新ファームの
  // startReady 拡張がある場合のみ。旧ファームは空文字）。
  const startBlockers = (): string => {
    const r = status?.startReady;
    if (r == null || r === 0x0f) return "";
    const missing: string[] = [];
    if (!(r & READY_MASK.imuOk)) missing.push("IMU 正常");
    if (!(r & READY_MASK.estopFree)) missing.push("E-Stop 解除");
    if (!(r & READY_MASK.qualified))
      missing.push("静止判定（0.5 秒静止が必要）");
    if (!(r & READY_MASK.tiltOk)) {
      const deg =
        status?.tilt != null ? `現在 ${((status.tilt * 180) / Math.PI).toFixed(0)}°` : "";
      missing.push(`傾斜 35° 未満（${deg}）`);
    }
    return missing.length ? `（未充足: ${missing.join("・")}）` : "";
  };

  const stopOp = async () => {
    if (
      !(await confirmAsync(
        "モータ駆動を停止します。ロボットは姿勢を保持できず転倒します。よろしいですか？",
      ))
    )
      return;
    try {
      await robotStopOperation();
      pushLog(1, "動作停止を受理");
    } catch (e) {
      pushLog(3, `動作停止失敗: ${formatError(e)}`);
    }
  };

  return (
    <div>
      <h1 className="mb-4 text-xl">ホーム</h1>
      <div className="card">
        {connected ? (
          status ? (
            <table className="list">
              <tbody>
                <tr>
                  <th>動作状態</th>
                  <td>{OP_NAMES[status.opState] ?? `不明 (${status.opState})`}</td>
                </tr>
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
                <tr>
                  <th>キャリブレーション</th>
                  <td>
                    {CALIB_NAMES[status.calibState] ??
                      `不明 (${status.calibState})`}
                  </td>
                </tr>
                <tr>
                  <th>静止判定</th>
                  <td>
                    {status.startReady == null
                      ? "—"
                      : status.startReady & READY_MASK.qualified
                        ? "成立"
                        : "未成立（0.5 秒静止が必要）"}
                  </td>
                </tr>
                <tr>
                  <th>傾斜</th>
                  <td>
                    {status.tilt == null
                      ? "—"
                      : `${((status.tilt * 180) / Math.PI).toFixed(1)}°（動作開始は 35° 未満）`}
                  </td>
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
      <div className="card">
        <h2 className="mb-2 text-base">運転準備（待機中のみ実行可）</h2>
        <p className="muted mb-2 text-[13px]">
          実機準備: ①ロボットを水平な場所に置き静止させて IMU
          キャリブレーション → ②所定の初期位置・向きに設置して位置設定
          → ③ボール上に置き手で保持しながら動作開始。
        </p>
        <div className="row">
          <button
            className="btn"
            onClick={calibrate}
            disabled={
              !connected ||
              !standby ||
              status?.calibState === CALIB_STATE.running
            }
          >
            IMU キャリブレーション
          </button>
          {calibNote && <span className="muted">{calibNote}</span>}
        </div>
        <p className="muted mt-1 text-[13px]">
          ※ 実行中はロボットを動かさないこと（約 5 秒以内に完了/失敗が返る）。
        </p>
        <div className="row mt-2">
          <label>
            x [m]{" "}
            <input
              className="num"
              value={px}
              onChange={(e) => setPx(e.target.value)}
            />
          </label>
          <label>
            y [m]{" "}
            <input
              className="num"
              value={py}
              onChange={(e) => setPy(e.target.value)}
            />
          </label>
          <label>
            yaw [rad]{" "}
            <input
              className="num"
              value={pyaw}
              onChange={(e) => setPyaw(e.target.value)}
            />
          </label>
          <button
            className="btn"
            onClick={setPose}
            disabled={!connected || !standby}
          >
            位置設定
          </button>
        </div>
        <p className="muted mt-1 text-[13px]">
          ※ 送信前にロボットを設定したい座標・向きへ実際に置いておく。
        </p>
        <div className="row mt-2">
          {opState !== OP_STATE.operating ? (
            <button
              className="btn"
              onClick={startOp}
              disabled={!connected || !standby}
            >
              動作開始
            </button>
          ) : (
            <button className="btn" onClick={stopOp} disabled={!connected}>
              動作停止
            </button>
          )}
          <span className="muted">
            動作開始には IMU 正常・E-Stop 解除・キャリブレーション完了・姿勢正常が必要。
            開始時は手で保持して傾きを最小にすること。動作停止はモータ遮断=
            転倒するため、先に STOP で静止させてから実行すること。
          </span>
        </div>
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
