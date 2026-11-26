// FR-04: EVT_POSE 購読 → 位置・向き・軌跡・数値表示。RESET_ODOMETRY で原点再設定。
// 受信値・軌跡はグローバル store（app-shell の watchTelemetry が集約）。

import { useEffect } from "react";

import { OdometryMap } from "@/features/odometry/odometry-map";
import { formatError } from "@/lib/error";
import {
  robotResetOdometry,
  robotSubscribe,
  robotUnsubscribe,
} from "@/lib/ipc";
import { EVT_POSE, POSE_SUBSCRIBE_HZ } from "@/lib/limits";
import { confirmAsync } from "@/stores/confirm";
import { useConnected } from "@/stores/connection";
import { useLogStore } from "@/stores/logs";
import { useTelemetryStore } from "@/stores/telemetry";

const num = (v: number | null) => (v == null ? "-" : v.toFixed(3));

export function OdometryPage() {
  const connected = useConnected();
  const pushLog = useLogStore((s) => s.push);
  const { pose, trail, clearTrail } = useTelemetryStore();

  useEffect(() => {
    if (!connected) return;
    robotSubscribe(EVT_POSE, POSE_SUBSCRIBE_HZ).catch((e) =>
      pushLog(3, `POSE 購読失敗: ${formatError(e)}`),
    );
    return () => {
      robotUnsubscribe(EVT_POSE).catch(() => {});
    };
  }, [connected, pushLog]);

  const resetOrigin = async () => {
    if (!(await confirmAsync("現在位置をワールド原点に再設定する。よろしいか?"))) {
      return;
    }
    try {
      await robotResetOdometry();
      clearTrail();
      pushLog(1, "オドメトリ原点を再設定");
    } catch (e) {
      pushLog(3, `原点リセット失敗: ${formatError(e)}`);
    }
  };

  return (
    <div>
      <h1 className="mb-4 text-xl">オドメトリ</h1>
      <div className="card">
        <div className="row">
          <button
            className="btn-secondary"
            onClick={resetOrigin}
            disabled={!connected}
          >
            原点リセット
          </button>
          <span className="muted">
            {pose
              ? `x=${num(pose.x)} m, y=${num(pose.y)} m, yaw=${num(pose.yaw)} rad, vx=${num(pose.vx)}, vy=${num(pose.vy)}, yaw_rate=${num(pose.yawRate)}`
              : "POSE 未受信"}
          </span>
        </div>
      </div>
      <div className="card">
        <OdometryMap pose={pose} trail={trail} />
      </div>
    </div>
  );
}
