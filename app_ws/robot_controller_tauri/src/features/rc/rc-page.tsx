// FR-02: 2 本ジョイスティックによる速度指令。左=並進 (vx,vy)、右=回転 (yaw_rate)。

import { useCallback, useEffect, useRef, useState } from "react";

import { Joystick } from "@/features/rc/joystick";
import { formatError } from "@/lib/error";
import { robotSetMode, robotSetVelocity } from "@/lib/ipc";
import { MAX_YAW_RATE, RC_SEND_HZ } from "@/lib/limits";
import { CONTROL_MODE, OP_STATE } from "@/lib/types";
import { useConnected } from "@/stores/connection";
import { useLogStore } from "@/stores/logs";
import { useParamsStore } from "@/stores/params";
import { useTelemetryStore } from "@/stores/telemetry";

export function RcPage() {
  const connected = useConnected();
  const operating = useTelemetryStore(
    (s) => s.status?.opState === OP_STATE.operating,
  );
  const maxSpeed = useParamsStore((s) => s.maxSpeed);
  const pushLog = useLogStore((s) => s.push);
  const cmd = useRef({ vx: 0, vy: 0, yawRate: 0 });
  const active = useRef(false);
  const [sent, setSent] = useState({ vx: 0, vy: 0, yawRate: 0 });
  const [modeSet, setModeSet] = useState(false);

  // 20 Hz で最新指令を送出する。操作中のみ送信する（解放時はゼロを 1 回送出）。
  useEffect(() => {
    if (!connected || !modeSet || !operating) return;
    const id = setInterval(() => {
      if (!active.current) return;
      const c = cmd.current;
      robotSetVelocity(c.vx, c.vy, c.yawRate).catch((e) =>
        pushLog(3, `速度指令失敗: ${formatError(e)}`),
      );
      setSent({ ...c });
    }, 1000 / RC_SEND_HZ);
    return () => clearInterval(id);
  }, [connected, modeSet, operating, pushLog]);

  const onStick = useCallback(
    (which: "trans" | "rot") => (x: number, y: number) => {
      const c = cmd.current;
      if (which === "trans") {
        c.vx = y * maxSpeed;
        c.vy = x * maxSpeed;
      } else {
        c.yawRate = x * MAX_YAW_RATE;
      }
      const nonzero = c.vx !== 0 || c.vy !== 0 || c.yawRate !== 0;
      if (!nonzero && active.current) {
        // 全軸解放: ゼロ指令を 1 回送出
        robotSetVelocity(0, 0, 0).catch(() => {});
        setSent({ vx: 0, vy: 0, yawRate: 0 });
      }
      active.current = nonzero;
    },
    [maxSpeed],
  );

  const enterVelocityMode = async () => {
    try {
      await robotSetMode(CONTROL_MODE.velocity);
      setModeSet(true);
      pushLog(1, "速度指令モードへ移行");
    } catch (e) {
      pushLog(3, `モード移行失敗: ${formatError(e)}`);
    }
  };

  return (
    <div>
      <h1 className="mb-4 text-xl">RC（速度指令）</h1>
      <div className="card">
        <div className="row">
          <button
            className="btn"
            onClick={enterVelocityMode}
            disabled={!connected || !operating}
          >
            速度指令モードへ
          </button>
          <span className="muted">
            送信: vx={sent.vx.toFixed(2)} m/s, vy={sent.vy.toFixed(2)} m/s,
            yaw={sent.yawRate.toFixed(2)} rad/s（20 Hz, 最大 {maxSpeed} m/s /{" "}
            {MAX_YAW_RATE} rad/s）
          </span>
        </div>
        {connected && !operating && (
          <p className="muted mt-2">
            動作中ではないため速度指令は送信されません。ホームで動作開始してください。
          </p>
        )}
      </div>
      <div className="card">
        <Joystick label="並進 vx(↑)/vy(↔)" axes="xy" onChange={onStick("trans")} />
        <Joystick label="回転 yaw_rate(↔)" axes="x" onChange={onStick("rot")} />
        <p className="muted">
          スティック解放でゼロ指令を送出。ロボット側は 500 ms 無指令で自動停止する。
        </p>
      </div>
    </div>
  );
}
