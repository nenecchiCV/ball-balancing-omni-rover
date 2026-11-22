#!/usr/bin/env python3
"""オンボードテスト自動実行スクリプト。

ST-Link 仮想 COM 経由で onboard ファームウェアの `test` shell コマンドを
test_plan.md の推奨順に実行し、各テストの生ログと結果サマリを
spec/test/results/<timestamp>/ に記録する。

使い方:
    python run_onboard_tests.py [--port COM4] [--out DIR]
        [--skip-motor] [--skip-interactive]

前提:
    - onboard ファームウェアが書き込まれ、LED が 1 秒周期で点滅していること。
    - TeraTerm 等の他のシリアルクライアントは閉じておくこと。
    - モータ駆動テスト (enc spin / omni / deadzone) はバッテリー接続・
      車輪を浮かせた状態で実施する。対話プロンプトで確認してから進む。
"""

import argparse
import datetime
import re
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

BAUD = 115200
ST_LINK_VID = 0x0483

ANSI_RE = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]|\x1b[()][0-9AB]")


def strip_ansi(text: str) -> str:
    return ANSI_RE.sub("", text).replace("\r\n", "\n").replace("\r", "\n")


def find_port() -> str:
    """ST-Link VCP (VID 0483) を探す。見つからなければ先頭のポート。"""
    ports = list(list_ports.comports())
    for p in ports:
        if p.vid == ST_LINK_VID:
            return p.device
    if ports:
        return ports[0].device
    raise RuntimeError("シリアルポートが見つかりません")


class SerialSession:
    def __init__(self, port: str, baud: int = BAUD):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.buf = ""
        self.debug = False

    def close(self):
        self.ser.close()

    def drain(self, duration: float) -> str:
        """duration 秒だけ受信し、蓄積済み分を含めて返す。"""
        end = time.monotonic() + duration
        while time.monotonic() < end:
            chunk = self.ser.read(4096)
            if chunk:
                self.buf += chunk.decode("utf-8", errors="replace")
        return self.buf

    def collect(self, markers=(), timeout: float = 10.0) -> str:
        """マーカー文字列のいずれかが現れるか timeout まで受信。"""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            chunk = self.ser.read(4096)
            if chunk:
                self.buf += chunk.decode("utf-8", errors="replace")
                if self.debug:
                    print(f"[dbg] +{len(chunk)}B: {chunk[:80]!r}")
            if any(m in self.buf for m in markers):
                time.sleep(0.3)
                self.drain(0.3)
                return self.buf
        return self.buf

    def send(self, cmd: str):
        self.ser.write((cmd + "\r").encode())

    def take(self) -> str:
        """蓄積バッファを取り出してクリア。"""
        out = self.buf
        self.buf = ""
        return out


def save(log_dir: Path, name: str, text: str):
    path = log_dir / f"{name}.log"
    path.write_text(strip_ansi(text), encoding="utf-8")
    return path


def wait_enter(msg: str):
    input(f"\n>>> {msg} Enter で続行...")


def countdown(msg: str, sec: float):
    print(f">>> {msg} ({sec:.0f} s)", flush=True)
    time.sleep(sec)


def evaluate_tick(text: str):
    m = re.search(
        r"tick n=(\d+) mean_us=(\d+) min_us=(\d+) max_us=(\d+) missed=(\d+)",
        text,
    )
    if not m:
        return "FAIL", "tick 報告行が見つからない"
    n, mean, mn, mx, missed = (int(v) for v in m.groups())
    ok = 990 <= mean <= 1010 and missed == 0
    return ("PASS" if ok else "FAIL"), m.group(0)


def evaluate_estop(text: str):
    asserted = "requested=1" in text
    released = "requested=0" in text
    if asserted and released:
        return "PASS", "requested=1 と 0 への遷移を確認"
    if asserted:
        return "REVIEW", "requested=1 のみ確認（非押下復帰が未確認）"
    return "REVIEW", "requested=1 が未確認（ボタン操作が要検証）"


def evaluate_imu(text: str):
    lines = re.findall(r"imu t=.*valid=(\d)", text)
    if not lines:
        return "FAIL", "imu ログ行なし"
    valid_ratio = sum(int(v) for v in lines) / len(lines)
    if valid_ratio >= 0.9:
        return "PASS", f"imu ログ {len(lines)} 行, valid 率 {valid_ratio:.0%}"
    return "REVIEW", f"imu ログ {len(lines)} 行, valid 率 {valid_ratio:.0%}"


def evaluate_enc(text: str):
    lines = re.findall(r"enc t=\d+ .*cnt=\[(-?\d+) (-?\d+) (-?\d+)\]", text)
    if not lines:
        return "FAIL", "enc ログ行なし"
    last = lines[-1]
    return "REVIEW", f"{len(lines)} 行, 最終 cnt={last}（方向は目視確認）"


def evaluate_enc_spin(text: str):
    if "enc spin: done" not in text:
        return "FAIL", "enc spin が完了しなかった"
    rows = re.findall(
        r"enc t=\d+ duty=(-?[\d.]+) .*rate=\[(-?[\d.]+) (-?[\d.]+) (-?[\d.]+)\]",
        text,
    )
    pos = [r[1:] for r in rows if float(r[0]) > 0.0]
    neg = [r[1:] for r in rows if float(r[0]) < 0.0]
    if not pos or not neg:
        return "REVIEW", "duty 正負双方の rate ログが不足"
    pos_rate = [float(v) for v in pos[len(pos) // 2]]
    neg_rate = [float(v) for v in neg[len(neg) // 2]]
    ok = all(r > 0 for r in pos_rate) and all(r < 0 for r in neg_rate)
    note = f"+duty rate={pos_rate} / -duty rate={neg_rate}"
    return ("PASS" if ok else "REVIEW"), note


def evaluate_deadzone(text: str):
    if "dz,done" not in text:
        return "FAIL", "dz,done が現れず掃引未完了"
    rows = re.findall(
        r"dz,([\d.]+),(-?[\d.]+),(-?[\d.]+),(-?[\d.]+),(-?[\d.]+)", text
    )
    if not rows:
        return "REVIEW", "CSV 行なし"
    # 各輪で |w|>0.5 rad/s となる最小 |duty| を不感帯推定値とする。
    estimates = []
    for wi in range(3):
        onset = next(
            (abs(float(r[1])) for r in rows if abs(float(r[2 + wi])) > 0.5),
            None,
        )
        estimates.append(f"{onset:.3f}" if onset is not None else "不動")
    return "REVIEW", f"不感帯推定 |duty| w0={estimates[0]} w1={estimates[1]} w2={estimates[2]}（最大値超過時は不動）"


def evaluate_omni(text: str):
    lines = re.findall(
        r"omni t=\d+ .*w=\[(-?[\d.]+) (-?[\d.]+) (-?[\d.]+)\]", text
    )
    if not lines:
        return "FAIL", "omni ログ行なし"
    moved = any(any(abs(float(v)) > 0.5 for v in row) for row in lines)
    last = [float(v) for v in lines[-1]]
    return ("PASS" if moved else "REVIEW"), f"{len(lines)} 行, 最終 w={last}"


def run(session: SerialSession, log_dir: Path):
    results = []

    def record(name, outcome):
        verdict, note = outcome
        results.append((name, verdict, note))
        print(f"    [{verdict}] {name}: {note}")

    # 0. shell 疎通 + テスト一覧
    session.send("test")
    log = session.collect(markers=("uart:~$", "usage"), timeout=3.0)
    save(log_dir, "00_list", session.take())
    if "usage" not in log and "deadzone" not in log:
        print("FAIL: shell `test` コマンドが応答しません。onboard ファームウェアが"
              "起動しているか確認してください。")
        return results

    # 1. tick（自動で1秒ごとに報告）
    print("\n== test tick ==")
    session.debug = True
    log = ""
    for attempt in (1, 2):
        session.send("test tick")
        session.drain(0.6)
        session.send("test")  # 診断1: 窓の初期に isr/loop/upd 採取
        session.drain(1.5)
        session.send("test")  # 診断2: 差分で発火率を測定
        log = session.collect(markers=("tick n="), timeout=5.0)
        session.send("test stop")
        log += session.collect(markers=("stopped",), timeout=3.0)
        if "tick n=" in log:
            break
        session.drain(1.0)
    session.debug = False
    save(log_dir, "01_tick", session.take())
    verdict, note = evaluate_tick(log)
    if attempt == 2 and "tick n=" in log:
        note += "（2 回目の起動で取得成功）"
    record("tick", (verdict, note))

    # 2. estop（対話）
    print("\n== test estop ==")
    wait_enter("E-stop テスト。外部 E-stop (PC0) と B1 ボタンを操作します。")
    session.send("test estop")
    session.drain(1.0)
    countdown("PC0 を GND に短絡してください", 8.0)
    countdown("PC0 を解放してください", 4.0)
    countdown("B1（青ボタン）を押してください", 6.0)
    countdown("B1 を離してください", 4.0)
    session.send("test stop")
    log = session.collect(markers=("stopped",), timeout=3.0)
    save(log_dir, "02_estop", session.take())
    record("estop", evaluate_estop(log))

    # 3. imu（10 秒記録）
    print("\n== test imu ==")
    wait_enter("IMU テスト。車体を水平に静止させてから開始します"
               "（開始後にゆっくり傾けて euler 変化も記録）。")
    session.send("test imu")
    countdown("10 秒間記録します", 10.0)
    session.send("test stop")
    log = session.collect(markers=("stopped",), timeout=3.0)
    save(log_dir, "03_imu", session.take())
    record("imu", evaluate_imu(log))

    # 4. enc（手回し 6 秒）
    print("\n== test enc ==")
    wait_enter("エンコーダテスト。開始後に各車輪を正方向（モーターフェイス側から"
               "見て CW）へ手で回すと cnt/rate が記録されます。")
    session.send("test enc")
    countdown("6 秒間記録します（車輪を手で回してください）", 6.0)
    session.send("test stop")
    log = session.collect(markers=("stopped",), timeout=3.0)
    save(log_dir, "04_enc", session.take())
    record("enc", evaluate_enc(log))

    return results


def run_motor(session: SerialSession, log_dir: Path, results):
    def record(name, outcome):
        verdict, note = outcome
        results.append((name, verdict, note))
        print(f"    [{verdict}] {name}: {note}")

    # 5. enc spin（+duty → -duty、自動終了。既定 0.40 は不感帯超え）
    print("\n== test enc spin ==")
    session.send("test enc spin")
    log = session.collect(markers=("enc spin: done", "E-stop"), timeout=20.0)
    session.send("test stop")
    session.collect(markers=("stopped",), timeout=3.0)
    save(log_dir, "05_enc_spin", session.take())
    record("enc spin", evaluate_enc_spin(log))

    # 6. deadzone（+方向 0.6 まで 15 s 掃引、自動終了）
    print("\n== test deadzone ==")
    session.send("test deadzone 1 0.6 15")
    log = session.collect(markers=("dz,done", "E-stop"), timeout=45.0)
    session.send("test stop")
    session.collect(markers=("stopped",), timeout=3.0)
    save(log_dir, "06_deadzone", session.take())
    record("deadzone", evaluate_deadzone(log))

    # 7. omni（10 秒駆動後に手動停止）
    print("\n== test omni ==")
    session.send("test omni 0.1 0 0")
    countdown("10 秒間駆動します", 10.0)
    session.send("test stop")
    log = session.collect(markers=("stopped",), timeout=5.0)
    save(log_dir, "07_omni", session.take())
    record("omni", evaluate_omni(log))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", help="COM ポート（省略時は ST-Link を自動検出）")
    ap.add_argument("--out", type=Path, default=None, help="結果出力先")
    ap.add_argument("--skip-motor", action="store_true",
                    help="モータ駆動テスト (enc spin/deadzone/omni) をスキップ")
    args = ap.parse_args()

    port = args.port or find_port()
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = args.out or Path(__file__).resolve().parents[1] / "results" / ts
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"ポート: {port} @ {BAUD}")
    print(f"結果出力先: {out_dir}")
    try:
        session = SerialSession(port)
    except serial.SerialException as e:
        sys.exit(f"ポートを開けません: {e}\n"
                 "TeraTerm 等でポートを占有していないか確認してください。")

    session.drain(0.5)
    session.send("")          # プロンプト起こし
    session.drain(0.5)
    session.send("test stop")  # 残留テストの停止
    session.drain(0.8)
    session.take()

    results = run(session, out_dir)

    if not args.skip_motor:
        wait_enter("モータ駆動テストへ進みます。バッテリー接続・車輪を浮かせた"
                   "状態を確認してください（中止は Ctrl+C）。")
        run_motor(session, out_dir, results)

    session.send("test stop")
    session.drain(0.5)
    session.close()

    # サマリ出力
    lines = [
        f"# オンボードテスト結果 {ts}",
        "",
        f"- ポート: {port}",
        "",
        "| テスト | 判定 | 備考 |",
        "|---|---|---|",
    ]
    for name, verdict, note in results:
        lines.append(f"| {name} | {verdict} | {note} |")
    lines += [
        "",
        "PASS=自動判定合格 / REVIEW=ログ目視確認 / FAIL=要調査",
        "各テストの生ログは同ディレクトリの *.log を参照。",
    ]
    summary = "\n".join(lines) + "\n"
    (out_dir / "summary.md").write_text(summary, encoding="utf-8")
    print("\n" + summary)
    print(f"サマリ保存: {out_dir / 'summary.md'}")


if __name__ == "__main__":
    main()
