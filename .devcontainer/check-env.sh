#!/usr/bin/env bash
# check-env.sh — devcontainer 内の開発環境を点検する
set -u

echo "=== Zephyr environment ==="
echo "ZEPHYR_BASE            = ${ZEPHYR_BASE:-<unset>}"
echo "ZEPHYR_WS_DIR          = ${ZEPHYR_WS_DIR:-<unset>}"
echo "ZEPHYR_SDK_INSTALL_DIR = ${ZEPHYR_SDK_INSTALL_DIR:-<unset>}"
echo "ZEPHYR_TOOLCHAIN_VARIANT = ${ZEPHYR_TOOLCHAIN_VARIANT:-<unset>}"
echo

echo "=== Tool versions ==="
for cmd in cmake ninja west clang clang++ clang-tidy clang-format clangd \
           arm-zephyr-eabi-gcc riscv64-zephyr-elf-gcc \
           arm-zephyr-eabi-gdb arm-none-eabi-gdb riscv64-zephyr-elf-gdb \
           openocd git python3; do
  if command -v "$cmd" >/dev/null 2>&1; then
    printf '%-26s %s\n' "$cmd" "$("$cmd" --version 2>&1 | head -n1)"
  else
    printf '%-26s %s\n' "$cmd" "NOT FOUND"
  fi
done
printf '%-26s %s\n' "esptool" "$(esptool version 2>&1 | head -n1 || echo 'NOT FOUND')"
printf '%-26s %s\n' "openocd-esp32" "$(/opt/openocd-esp32/bin/openocd --version 2>&1 | head -n1 || echo 'NOT FOUND')"
echo

echo "=== Zephyr workspace ==="
if [ -d "${ZEPHYR_WS_DIR:-/nonexistent}/zephyr" ]; then
  (cd "$ZEPHYR_WS_DIR" && west list manifest 2>/dev/null | head -n3)
  for m in hal/stm32 hal/st hal/cmsis_6 hal/espressif; do
    [ -d "$ZEPHYR_WS_DIR/modules/$m" ] && echo "module ok: $m" || echo "module MISSING: $m"
  done
else
  echo "zephyr workspace not found"
fi
echo

echo "=== App tools (robot_controller_tauri) ==="
for cmd in node npm cargo rustc rustup; do
  if command -v "$cmd" >/dev/null 2>&1; then
    printf '%-26s %s\n' "$cmd" "$("$cmd" --version 2>&1 | head -n1)"
  else
    printf '%-26s %s\n' "$cmd" "NOT FOUND"
  fi
done
if pkg-config --exists webkit2gtk-4.1 2>/dev/null; then
  printf '%-26s %s\n' "webkit2gtk-4.1" "$(pkg-config --modversion webkit2gtk-4.1)"
else
  printf '%-26s %s\n' "webkit2gtk-4.1" "NOT FOUND"
fi
if command -v bluetoothctl >/dev/null 2>&1; then
  printf '%-26s %s\n' "bluez" "installed (BLE は Linux ホストの BlueZ/D-Bus 経由)"
else
  printf '%-26s %s\n' "bluez" "NOT FOUND"
fi
echo

echo "=== USB devices (usbipd passthrough) ==="
if command -v lsusb >/dev/null 2>&1; then
  lsusb | grep -Ei '0483:37|303a:1001|st-link|espressif' || echo "(no target devices attached)"
else
  echo "lsusb not available"
fi
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || true
