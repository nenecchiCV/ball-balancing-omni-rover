"use client";

// FR-03: 目標位置 (x, y) と速度上限の入力 → SET_POSE_TARGET 送信。マップに目標を描画。

import { useEffect, useState } from "react";

import { OdometryMap } from "@/features/odometry/odometry-map";
import { robotSetPoseTarget, robotSubscribe, robotUnsubscribe } from "@/lib/ipc";
import { EVT_POSE, POSE_SUBSCRIBE_HZ } from "@/lib/limits";
import { OP_STATE } from "@/lib/types";
import { useConnected } from "@/stores/connection";
import { useLogStore } from "@/stores/logs";
import { useParamsStore } from "@/stores/params";
import { useTelemetryStore } from "@/stores/telemetry";

export function PositionPage() {
  const connected = useConnected();
  const operating = useTelemetryStore(
    (s) => s.status?.opState === OP_STATE.operating,
  );
  const pushLog = useLogStore((s) => s.push);
  const pose = useTelemetryStore((s) => s.pose);
  const maxSpeed = useParamsStore((s) => s.maxSpeed);
  const [x, setX] = useState("0.0");
  const [y, setY] = useState("0.0");
  const [speed, setSpeed] = useState("0.2");
  const [target, setTarget] = useState<[number, number] | null>(null);

  // 滞在中は EVT_POSE を購読する（受信値はグローバル store に流れ込む）。
  useEffect(() => {
    if (!connected) return;
    robotSubscribe(EVT_POSE, POSE_SUBSCRIBE_HZ).catch((e) =>
      pushLog(3, `POSE 購読失敗: ${String(e)}`),
    );
    return () => {
      robotUnsubscribe(EVT_POSE).catch(() => {});
    };
  }, [connected, pushLog]);

  const sendTarget = async () => {
    const tx = parseFloat(x);
    const ty = parseFloat(y);
    const sp = Math.min(Math.max(parseFloat(speed) || 0, 0), maxSpeed);
    if (Number.isNaN(tx) || Number.isNaN(ty)) {
      pushLog(2, "目標値が数値ではない");
      return;
    }
    try {
      await robotSetPoseTarget(tx, ty, sp);
      setTarget([tx, ty]);
      pushLog(1, `位置目標送信: (${tx.toFixed(2)}, ${ty.toFixed(2)}) v≤${sp.toFixed(2)}`);
    } catch (e) {
      pushLog(3, `位置目標拒否: ${String(e)}`);
    }
  };

  const remaining =
    pose?.x != null && pose.y != null && target
      ? Math.hypot(target[0] - pose.x, target[1] - pose.y)
      : null;

  return (
    <div>
      <h1 className="mb-4 text-xl">位置制御</h1>
      <div className="card">
        <div className="row">
          <label>
            x [m]{" "}
            <input
              className="num"
              value={x}
              onChange={(e) => setX(e.target.value)}
            />
          </label>
          <label>
            y [m]{" "}
            <input
              className="num"
              value={y}
              onChange={(e) => setY(e.target.value)}
            />
          </label>
          <label>
            速度上限 [m/s]{" "}
            <input
              className="num"
              value={speed}
              onChange={(e) => setSpeed(e.target.value)}
            />
          </label>
          <button
            className="btn"
            onClick={sendTarget}
            disabled={!connected || !operating}
          >
            目標送信
          </button>
        </div>
        {connected && !operating && (
          <p className="muted mt-2">
            動作中ではないため目標位置は送信できません。ホームで動作開始してください。
          </p>
        )}
        <p className="muted mt-2">
          ワールド座標（起動点原点）。速度上限は {maxSpeed} m/s まで。
          {remaining !== null && ` 残距離: ${remaining.toFixed(2)} m`}
        </p>
      </div>
      <div className="card">
        <OdometryMap
          pose={pose}
          trail={pose?.x != null && pose.y != null ? [[pose.x, pose.y]] : []}
          target={target}
        />
      </div>
    </div>
  );
}
