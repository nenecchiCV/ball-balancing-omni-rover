// FR-09/FR-10: パラメータ調整ページ。調整対象パラメータの表示・編集・
// フラッシュ保存と、不感帯掃引・IMU 特性確認を行う。

import { useCallback, useEffect, useState } from "react";

import { formatError } from "@/lib/error";
import {
  robotCalibrateImu,
  robotGetParams,
  robotResetParams,
  robotSaveParams,
  robotSetParam,
  robotTuneDeadzone,
} from "@/lib/ipc";
import { OP_STATE, PARAM_DEFS, paramName, type TuneResultInfo } from "@/lib/types";
import { confirmAsync } from "@/stores/confirm";
import { useConnected } from "@/stores/connection";
import { useLogStore } from "@/stores/logs";
import {
  PARAM_KEY_MAX_SPEED,
  PARAM_KEY_THEORY_MAX_SPEED,
  useParamsStore,
} from "@/stores/params";
import { useTelemetryStore } from "@/stores/telemetry";

export function TuningPage() {
  const connected = useConnected();
  const status = useTelemetryStore((s) => s.status);
  const pushLog = useLogStore((s) => s.push);
  const standby = status?.opState === OP_STATE.standby;
  const enabled = connected && standby;

  const [params, setParams] = useState<Map<number, number | null> | null>(null);
  const [dirty, setDirty] = useState<Set<number>>(new Set());
  const [note, setNote] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);
  const [tuneDir, setTuneDir] = useState<"pos" | "neg" | "both">("pos");
  const [tuneMax, setTuneMax] = useState("0.25");
  const [tuneRamp, setTuneRamp] = useState("15");
  const [tuneResultPos, setTuneResultPos] = useState<TuneResultInfo | null>(null);
  const [tuneResultNeg, setTuneResultNeg] = useState<TuneResultInfo | null>(null);

  const reload = useCallback(async () => {
    try {
      const info = await robotGetParams();
      setParams(new Map(info.entries.map((e) => [e.key, e.value])));
      setDirty(new Set());
      setNote(null);
      // 指令上限ストアも最新値へ同期する。
      const ms = info.entries.find(
        (e) => e.key === PARAM_KEY_MAX_SPEED,
      )?.value;
      const theory = info.entries.find(
        (e) => e.key === PARAM_KEY_THEORY_MAX_SPEED,
      )?.value;
      useParamsStore.getState().setLimits(ms ?? 0.4, theory ?? null);
    } catch (e) {
      setNote(`パラメータ取得失敗: ${formatError(e)}`);
    }
  }, []);

  useEffect(() => {
    if (connected) void reload();
  }, [connected, reload]);

  const applyParam = async (key: number, raw: string) => {
    const v = parseFloat(raw);
    if (Number.isNaN(v)) {
      setNote(`${paramName(key)}: 数値ではない`);
      return;
    }
    try {
      await robotSetParam(key, v);
      setParams((m) => new Map(m).set(key, v));
      if (key === PARAM_KEY_MAX_SPEED) {
        useParamsStore.getState().setLimits(v, null);
      }
      setDirty((d) => new Set(d).add(key));
      setNote(`${paramName(key)} = ${v}（RAM 反映、未保存）`);
    } catch (e) {
      setNote(`${paramName(key)} 設定拒否: ${formatError(e)}`);
    }
  };

  const save = async () => {
    try {
      await robotSaveParams();
      setDirty(new Set());
      setNote("フラッシュへ保存しました");
      pushLog(1, "パラメータをフラッシュへ保存");
    } catch (e) {
      setNote(`保存失敗: ${formatError(e)}`);
    }
  };

  const resetDefaults = async () => {
    if (
      !(await confirmAsync(
        "保存領域を消去し、全パラメータを既定値へ戻します。よろしいですか？",
      ))
    )
      return;
    try {
      await robotResetParams();
      pushLog(2, "パラメータを既定値へ復元");
      await reload();
      setNote("既定値へ復元しました");
    } catch (e) {
      setNote(`復元失敗: ${formatError(e)}`);
    }
  };

  const tune = async () => {
    const maxDuty = parseFloat(tuneMax);
    const rampS = parseFloat(tuneRamp);
    if (Number.isNaN(maxDuty) || Number.isNaN(rampS)) {
      setNote("掃引パラメータが数値ではない");
      return;
    }
    // 両方向は正転→逆転の順に 2 セッション連続実行（時間は 2 倍）。
    const dirs: number[] =
      tuneDir === "both" ? [1, -1] : [tuneDir === "pos" ? 1 : -1];
    const sweepSecs = Math.ceil(2 * rampS * dirs.length);
    if (
      !(await confirmAsync(
        `不感帯掃引を開始します（約 ${sweepSecs} 秒）。` +
          "車体を手で保持し、3 輪すべて地面から浮かせた無負荷状態にしてください。" +
          "掃引中モータが回転します。よろしいですか？",
      ))
    )
      return;
    setBusy(true);
    if (dirs.includes(1)) setTuneResultPos(null);
    if (dirs.includes(-1)) setTuneResultNeg(null);
    try {
      for (const [i, dir] of dirs.entries()) {
        const label = dir > 0 ? "正転" : "逆転";
        setNote(
          `掃引実行中（${label} ${i + 1}/${dirs.length}、約 ${Math.ceil(2 * rampS)} 秒、完了まで操作しない）`,
        );
        const r = await robotTuneDeadzone(dir, maxDuty, rampS);
        if (dir > 0) setTuneResultPos(r);
        else setTuneResultNeg(r);
        pushLog(r.result === 0 ? 1 : 2, `不感帯掃引(${label}) result=${r.result}`);
        if (r.result !== 0) {
          setNote(`掃引（${label}）が中断されました（E-Stop/キャンセル）`);
          return;
        }
      }
      setNote("掃引完了。推奨値を確認して「推奨値を適用」してください");
    } catch (e) {
      setNote(`掃引失敗: ${formatError(e)}`);
      pushLog(3, `不感帯掃引失敗: ${formatError(e)}`);
    } finally {
      setBusy(false);
    }
  };

  const applySuggest = async () => {
    const entries: [TuneResultInfo | null, number][] = [
      [tuneResultPos, 0x00],
      [tuneResultNeg, 0x03],
    ];
    const targets = entries.filter(([r]) => r?.result === 0);
    if (targets.length === 0) return;
    try {
      const applied: string[] = [];
      for (const [r, base] of targets) {
        for (let i = 0; i < 3; ++i) {
          await robotSetParam(base + i, r!.suggest[i] ?? 0);
        }
        setDirty((d) => new Set([...d, base, base + 1, base + 2]));
        applied.push(base === 0 ? "正転" : "逆転");
      }
      await reload();
      setNote(
        `推奨 deadzone_comp（${applied.join("・")}）を適用しました（保存は「保存」ボタン）`,
      );
    } catch (e) {
      setNote(`推奨値適用失敗: ${formatError(e)}`);
    }
  };

  const calibrate = async () => {
    if (
      !(await confirmAsync(
        "IMU キャリブレーションを実行します。" +
          "ロボットを水平な場所に置き、静止させてください。よろしいですか？",
      ))
    )
      return;
    setNote("キャリブレーション実行中…（静止を維持）");
    try {
      const r = await robotCalibrateImu();
      setNote(
        r.result === 0
          ? `キャリブレーション完了: gyro_bias_z=${r.gyroBiasZ?.toFixed(6) ?? "?"} rad/s（保存は「保存」ボタン）`
          : "キャリブレーション失敗",
      );
      pushLog(r.result === 0 ? 1 : 3, `IMU キャリブ result=${r.result}`);
    } catch (e) {
      setNote(`キャリブレーション失敗: ${formatError(e)}`);
    }
  };

  return (
    <div>
      <h1 className="mb-4 text-xl">パラメータ調整</h1>

      {!standby && connected && (
        <div className="card">
          <p className="muted">
            調整操作はロボット待機中（STANDBY）のみ有効です。
            {status?.opState === OP_STATE.tuning && " 現在: 同定セッション実行中。"}
          </p>
        </div>
      )}

      <div className="card">
        <h2 className="mb-2 text-base">実機準備ガイド</h2>
        <ul className="muted ml-4 list-disc text-[13px] leading-6">
          <li>パラメータ編集・保存・既定値復元: ロボットは待機中のままでよい（転倒しない）</li>
          <li>不感帯掃引: 車体を手で保持し、3 輪すべて地面から浮かせた無負荷状態で実行（掃引中モータが回転する）</li>
          <li>IMU キャリブレーション: ロボットを水平な場所に置き、実行中は静止を維持する</li>
          <li>変更は「設定」時点で RAM に反映（揮発）。電源を切っても残すには「保存」が必要</li>
        </ul>
      </div>

      <div className="card">
        <h2 className="mb-2 text-base">調整対象パラメータ</h2>
        <div className="row mb-2">
          <button className="btn" onClick={reload} disabled={!connected}>
            再読込
          </button>
          <button className="btn" onClick={save} disabled={!enabled}>
            保存
          </button>
          <button className="btn" onClick={resetDefaults} disabled={!enabled}>
            既定値に戻す
          </button>
          {dirty.size > 0 && (
            <span className="muted">未保存の変更 {dirty.size} 件</span>
          )}
        </div>
        {params == null ? (
          <p className="muted">{connected ? "読込中…" : "未接続"}</p>
        ) : (
          <div className="max-h-96 overflow-y-auto">
            <table className="list">
              <thead>
                <tr>
                  <th>パラメータ</th>
                  <th>key</th>
                  <th>値</th>
                  <th></th>
                </tr>
              </thead>
              <tbody>
                {[...params.entries()].map(([key, value]) => (
                  <ParamRow
                    key={key}
                    kkey={key}
                    value={value}
                    dirty={dirty.has(key)}
                    disabled={!enabled}
                    onApply={applyParam}
                  />
                ))}
              </tbody>
            </table>
          </div>
        )}
      </div>

      <div className="card">
        <h2 className="mb-2 text-base">不感帯掃引（TUNE_DEADZONE）</h2>
        <p className="muted mb-2 text-[13px]">
          車体を手で保持し 3 輪を浮かせた状態で、デューティを三角波掃引し、
          各車輪が回り始める duty（立上り）と回り続ける最小 duty（維持）を同定します。
        </p>
        <div className="row">
          <label>
            方向{" "}
            <select
              className="num w-32"
              value={tuneDir}
              onChange={(e) =>
                setTuneDir(e.target.value as "pos" | "neg" | "both")
              }
            >
              <option value="pos">正転</option>
              <option value="neg">逆転</option>
              <option value="both">両方向（順次）</option>
            </select>
          </label>
          <label>
            max duty{" "}
            <input
              className="num w-20"
              value={tuneMax}
              onChange={(e) => setTuneMax(e.target.value)}
            />
          </label>
          <label>
            ramp [s]{" "}
            <input
              className="num w-20"
              value={tuneRamp}
              onChange={(e) => setTuneRamp(e.target.value)}
            />
          </label>
          <button className="btn" onClick={tune} disabled={!enabled || busy}>
            {busy ? "掃引中…" : "掃引実行"}
          </button>
        </div>
        {(tuneResultPos || tuneResultNeg) && (
          <div className="mt-2 space-y-3">
            {(
              [
                { label: "正転", r: tuneResultPos },
                { label: "逆転", r: tuneResultNeg },
              ] as { label: string; r: TuneResultInfo | null }[]
            )
              .filter((e): e is { label: string; r: TuneResultInfo } => e.r != null)
              .map(({ label, r }) => (
                <div key={label}>
                  <h3 className="mb-1 text-[14px]">{label}</h3>
                  <table className="list">
                    <thead>
                      <tr>
                        <th>車輪</th>
                        <th>立上り duty</th>
                        <th>維持 duty</th>
                        <th>推奨 comp</th>
                      </tr>
                    </thead>
                    <tbody>
                      {[0, 1, 2].map((i) => (
                        <tr key={i}>
                          <th>{i}</th>
                          <td>{r.onset[i]?.toFixed(4) ?? "-"}</td>
                          <td>{r.sustain[i]?.toFixed(4) ?? "-"}</td>
                          <td>{r.suggest[i]?.toFixed(4) ?? "-"}</td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                  {r.result !== 0 && (
                    <p className="muted mt-1">中断/失敗のため適用対象外</p>
                  )}
                </div>
              ))}
            <button
              className="btn"
              onClick={applySuggest}
              disabled={
                !enabled ||
                (tuneResultPos?.result !== 0 && tuneResultNeg?.result !== 0)
              }
            >
              推奨値を deadzone_comp へ適用
            </button>
          </div>
        )}
      </div>

      <div className="card">
        <h2 className="mb-2 text-base">IMU 特性</h2>
        <p className="muted mb-2 text-[13px]">
          現在値は上のパラメータ表（imu.* 行）を参照。ロボットを水平に置いて
          静止させた状態で実行してください。結果は動作パラメータへ反映され、
          「保存」で不揮発化されます。
        </p>
        <button className="btn" onClick={calibrate} disabled={!enabled}>
          IMU キャリブレーション実行
        </button>
      </div>

      {note && <div className="card">{note}</div>}
    </div>
  );
}

function ParamRow({
  kkey,
  value,
  dirty,
  disabled,
  onApply,
}: {
  kkey: number;
  value: number | null;
  dirty: boolean;
  disabled: boolean;
  onApply: (key: number, raw: string) => void;
}) {
  const [text, setText] = useState(value == null ? "" : String(value));
  useEffect(() => setText(value == null ? "" : String(value)), [value]);
  const def = PARAM_DEFS.find((d) => d.key === kkey);
  const ro = def?.readOnly === true;
  return (
    <tr>
      <td>{paramName(kkey)}</td>
      <td className="muted">0x{kkey.toString(16).toUpperCase().padStart(2, "0")}</td>
      <td>
        <input
          className="num w-28"
          value={text}
          onChange={(e) => setText(e.target.value)}
          disabled={disabled || ro}
        />
        {def?.unit && <span className="muted ml-1">{def.unit}</span>}
      </td>
      <td>
        {ro ? (
          <span className="muted">参照値</span>
        ) : (
          <button
            className="btn"
            onClick={() => onApply(kkey, text)}
            disabled={disabled}
          >
            設定{dirty ? "*" : ""}
          </button>
        )}
      </td>
    </tr>
  );
}
