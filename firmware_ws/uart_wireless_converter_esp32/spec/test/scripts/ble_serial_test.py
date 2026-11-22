#!/usr/bin/env python3
"""BLE シリアル通信テスト自動実行スクリプト。

uart_wireless_converter_esp32（NUS 相当 GATT）へ PC 側 BLE セントラルとして
接続し、TX Notify / RX Write のシリアル透過を検証する。
検証パスは PC <-> BLE <-> ESP32 UART0 <-> STM32 USART1 で、STM32 側は
onboard テストファームのバイトエコー（uart_echo_test、ブート時から常時
有効）で折り返す。

使い方:
    python ble_serial_test.py [--name uwc-esp32c6] [--stream-bytes 1024]
        [--out DIR] [--shell-port COM4]

前提:
    - bleak (pip install bleak) がインストール済みであること。
    - ESP32 (uwc-esp32c6) が給電され advertising していること。
    - STM32 に USART1 エコーを含む onboard テストファームが
      書き込まれていること（モータ不要）。
    - --shell-port を指定するとテスト後に `test echo` の転送統計を
      ST-Link シェルから採取する（ポートが空いている場合のみ）。
"""

import argparse
import asyncio
import datetime
import sys
import time
from pathlib import Path

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic

DEVICE_NAME = "uwc-esp32c6"
NUS_SVC_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # PC -> ESP32 (Write)
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # ESP32 -> PC (Notify)

SCAN_TIMEOUT = 10.0
ECHO_TIMEOUT = 5.0


class NotifySink:
    """TX Notify の受信蓄積。asyncio.Event で指定バイト数到達を待つ。"""

    def __init__(self):
        self.data = bytearray()
        self._event = asyncio.Event()

    def handler(self, _char: BleakGATTCharacteristic, data: bytearray):
        self.data += data
        self._event.set()

    def clear(self):
        self.data.clear()
        self._event.clear()

    async def wait_for(self, nbytes: int, timeout: float) -> bool:
        """nbytes 以上受信するか timeout で打ち切る。到達で True。"""
        end = time.monotonic() + timeout
        while len(self.data) < nbytes:
            self._event.clear()
            remaining = end - time.monotonic()
            if remaining <= 0:
                break
            try:
                await asyncio.wait_for(self._event.wait(), remaining)
            except asyncio.TimeoutError:
                break
        return len(self.data) >= nbytes


def save(log_dir: Path, name: str, text: str):
    path = log_dir / f"{name}.log"
    path.write_text(text, encoding="utf-8")
    return path


async def find_device(name: str, timeout: float):
    """名前一致 → NUS UUID 順でデバイスを探す。

    戻り値は (BLEDevice|None, {addr: (BLEDevice, AdvertisementData)}) 。
    AdvertisementData から RSSI・local_name・service_uuids を参照する。
    """
    found = await BleakScanner.discover(timeout=timeout, return_adv=True)
    exact = [
        d for d, adv in found.values()
        if (adv.local_name or d.name) == name
    ]
    if exact:
        return exact[0], found
    by_uuid = [
        d for d, adv in found.values()
        if NUS_SVC_UUID.lower()
        in [u.lower() for u in (adv.service_uuids or [])]
    ]
    if by_uuid:
        return by_uuid[0], found
    return None, found


async def roundtrip(client: BleakClient, rx_char: BleakGATTCharacteristic,
                    sink: NotifySink, payload: bytes, chunk: int,
                    timeout: float) -> tuple[bool, float, bytes]:
    """payload を chunk 分割で RX Write し、同一バイト列の折返しを待つ。"""
    sink.clear()
    t0 = time.monotonic()
    for i in range(0, len(payload), chunk):
        await client.write_gatt_char(rx_char, payload[i:i + chunk],
                                     response=False)
    ok = await sink.wait_for(len(payload), timeout)
    rtt_ms = (time.monotonic() - t0) * 1000.0
    return ok and bytes(sink.data[:len(payload)]) == payload, rtt_ms, \
        bytes(sink.data)


async def run(args) -> int:
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = args.out or Path(__file__).resolve().parents[1] / "results" / ts
    out_dir.mkdir(parents=True, exist_ok=True)

    results = []

    def record(name, verdict, note):
        results.append((name, verdict, note))
        print(f"    [{verdict}] {name}: {note}", flush=True)

    # ---- 0. スキャン ----------------------------------------------------
    print(f"== scan ({SCAN_TIMEOUT:.0f} s) ==", flush=True)
    device, found = await find_device(args.name, SCAN_TIMEOUT)
    lines = [
        f"# BLE scan results {ts}",
        "",
        f"見つかったデバイス数: {len(found)}",
        "",
        "| addr | name | rssi | uuids |",
        "|---|---|---|---|",
    ]
    for addr, (d, adv) in found.items():
        uuids = ",".join(adv.service_uuids or [])
        lines.append(
            f"| {addr} | {adv.local_name or d.name} | {adv.rssi} "
            f"| {uuids[:60]} |")
    if device is not None:
        lines += ["", f"対象: {device.address} ({device.name})"]
    save(out_dir, "00_scan", "\n".join(lines) + "\n")
    if device is None:
        record("scan", "FAIL",
               f"{args.name} / NUS UUID を持つデバイスが見つからない")
        summarize(results, out_dir, ts)
        return 1
    adv = found[device.address][1] if device.address in found else None
    record("scan", "PASS",
           f"{device.address} name={device.name} "
           f"rssi={adv.rssi if adv else '?'}")

    # ---- 1. 接続 + GATT -------------------------------------------------
    print("== connect ==", flush=True)
    sink = NotifySink()
    gatt_lines = [f"# GATT dump {device.address}", ""]
    client = None
    last_err = None
    for _attempt in range(2):
        client = BleakClient(device)
        try:
            await client.connect()
            break
        except Exception as e:  # noqa: BLE001
            last_err = e
            client = None
            await asyncio.sleep(1.0)
    if client is None:
        record("gatt", "FAIL", f"接続失敗: {last_err}")
        summarize(results, out_dir, ts)
        return 1
    try:
        await asyncio.sleep(0.5)  # MTU 交換・接続パラメータ更新待ち
        mtu = getattr(client, "mtu_size", None) or 23
        payload_max = max(1, mtu - 3)
        rx_char = client.services.get_characteristic(NUS_RX_UUID)
        tx_char = client.services.get_characteristic(NUS_TX_UUID)
        for svc in client.services:
            gatt_lines.append(f"service {svc.uuid}")
            for c in svc.characteristics:
                gatt_lines.append(
                    f"  char {c.uuid} props={','.join(c.properties)}"
                    f" handle={c.handle}")
        save(out_dir, "01_gatt", "\n".join(gatt_lines) + "\n")
        if rx_char is None or tx_char is None:
            record("gatt", "FAIL", "NUS RX/TX 特性が見つからない")
            summarize(results, out_dir, ts)
            return 1
        record("gatt", "PASS", f"mtu={mtu} payload_max={payload_max}")

        await client.start_notify(tx_char, sink.handler)
        await asyncio.sleep(0.3)  # CCC 反映待ち

        # ---- 2. 小メッセージ往復 ---------------------------------------
        print("== echo small ==", flush=True)
        msg = b"uwc-echo-test\n"
        ok, rtt, got = await roundtrip(client, rx_char, sink, msg,
                                       payload_max, ECHO_TIMEOUT)
        save(out_dir, "02_echo_small",
             f"send={msg!r}\nrtt_ms={rtt:.1f}\nrecv={got!r}\n")
        record("echo small", "PASS" if ok else "FAIL",
               f"{len(msg)} B roundtrip rtt={rtt:.1f} ms recv={len(got)} B")

        # ---- 3. 全バイト値パターン往復 ----------------------------------
        print("== echo 256B pattern ==", flush=True)
        pat = bytes(range(256))
        ok, rtt, got = await roundtrip(client, rx_char, sink, pat,
                                       payload_max, ECHO_TIMEOUT)
        save(out_dir, "03_echo_pattern",
             f"send_len=256\nrtt_ms={rtt:.1f}\n"
             f"recv_len={len(got)}\nmatch={got[:256] == pat}\n"
             f"recv_hex={got.hex()}\n")
        record("echo 256B", "PASS" if ok else "FAIL",
               f"256 B pattern rtt={rtt:.1f} ms recv={len(got)} B")

        # ---- 4. ストリーム往復 ------------------------------------------
        print(f"== echo stream {args.stream_bytes} B ==", flush=True)
        stream = bytes((i * 31 + 7) % 256 for i in range(args.stream_bytes))
        ok, rtt, got = await roundtrip(client, rx_char, sink, stream,
                                       payload_max, args.stream_timeout)
        kbps = len(stream) * 8 / rtt if rtt > 0 else 0.0
        save(out_dir, "04_echo_stream",
             f"send_len={len(stream)}\nrtt_ms={rtt:.1f}\n"
             f"recv_len={len(got)}\nmatch={got[:len(stream)] == stream}\n"
             f"throughput_kbps={kbps:.1f}\n")
        verdict = "PASS" if ok else "FAIL"
        record("echo stream", verdict,
               f"{len(stream)} B rtt={rtt:.1f} ms recv={len(got)} B "
               f"~{kbps:.1f} kbps")

        # MTU 交換が遅延完了した場合の最終値を記録する。
        mtu_late = getattr(client, "mtu_size", None) or mtu
        if mtu_late != mtu:
            record("mtu", "REVIEW",
                   f"mtu が {mtu}→{mtu_late} に更新（payload_max "
                   f"{max(1, mtu_late - 3)} まで拡大可）")

        await client.stop_notify(tx_char)
    finally:
        await client.disconnect()

    # ---- 5. 切断→再 advertising → 再接続 --------------------------------
    print("== reconnect ==", flush=True)
    await asyncio.sleep(2.0)  # 切断検知・再 advertising 待ち
    device2, _ = await find_device(args.name, SCAN_TIMEOUT)
    if device2 is None:
        record("reconnect", "FAIL", "切断後に再 advertising が確認できない")
        summarize(results, out_dir, ts)
        return 1
    async with BleakClient(device2) as client:
        await asyncio.sleep(0.5)
        rx_char = client.services.get_characteristic(NUS_RX_UUID)
        tx_char = client.services.get_characteristic(NUS_TX_UUID)
        await client.start_notify(tx_char, sink.handler)
        await asyncio.sleep(0.3)
        ok, rtt, got = await roundtrip(client, rx_char, sink,
                                       b"reconnect-check\n", payload_max,
                                       ECHO_TIMEOUT)
        save(out_dir, "05_reconnect",
             f"addr={device2.address}\nrtt_ms={rtt:.1f}\nrecv={got!r}\n")
        record("reconnect", "PASS" if ok else "FAIL",
               f"再接続+エコー rtt={rtt:.1f} ms recv={len(got)} B")
        await client.stop_notify(tx_char)

    # ---- 6. STM32 側統計（任意） ----------------------------------------
    if args.shell_port:
        try:
            import serial
            s = serial.Serial(args.shell_port, 115200, timeout=0.1)
            time.sleep(0.3)
            s.read(4096)
            s.write(b"test echo\r")
            time.sleep(2.5)
            stat = s.read(8192).decode(errors="replace")
            s.write(b"test stop\r")
            time.sleep(0.4)
            stat += s.read(4096).decode(errors="replace")
            s.close()
            save(out_dir, "06_stm32_echo_stats", stat)
            line = next((x for x in stat.splitlines() if x.startswith("echo ")),
                        "")
            record("stm32 echo stats", "REVIEW", line or "統計行なし")
        except Exception as e:  # noqa: BLE001
            record("stm32 echo stats", "REVIEW", f"シェル取得失敗: {e}")

    summarize(results, out_dir, ts)
    return 0 if all(v == "PASS" for _, v, _ in results) else 1


def summarize(results, out_dir: Path, ts: str):
    lines = [
        f"# BLE シリアル通信テスト結果 {ts}",
        "",
        "| テスト | 判定 | 備考 |",
        "|---|---|---|",
    ]
    for name, verdict, note in results:
        lines.append(f"| {name} | {verdict} | {note} |")
    lines += [
        "",
        "PASS=自動判定合格 / REVIEW=要目視 / FAIL=要調査",
        "各テストの生ログは同ディレクトリの *.log を参照。",
    ]
    summary = "\n".join(lines) + "\n"
    (out_dir / "summary.md").write_text(summary, encoding="utf-8")
    print("\n" + summary)
    print(f"サマリ保存: {out_dir / 'summary.md'}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--name", default=DEVICE_NAME, help="BLE デバイス名")
    ap.add_argument("--stream-bytes", type=int, default=1024,
                    help="ストリーム往復テストのバイト数")
    ap.add_argument("--stream-timeout", type=float, default=15.0,
                    help="ストリーム往復の打切り秒")
    ap.add_argument("--shell-port", default=None,
                    help="ST-Link COM ポート（指定時は STM32 統計を採取）")
    ap.add_argument("--out", type=Path, default=None, help="結果出力先")
    args = ap.parse_args()
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        print("中断")
        return 2
    except Exception as e:  # noqa: BLE001
        print(f"FAIL: {type(e).__name__}: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
