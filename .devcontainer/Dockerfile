# ball-balancing-omni-rover ファームウェア開発コンテナ
#
# 内容:
#   Zephyr 4.3 ワークスペース (hal_stm32 / hal_st / cmsis_6 / hal_espressif+blob)
#   Zephyr SDK 0.17.4 (arm-zephyr-eabi + riscv64-zephyr-elf のみ)
#   LLVM 21 (clang++ ホストテスト / clang-tidy / clang-format / clangd)
#   OpenOCD (STM32 ST-LINK 用) + openocd-esp32 (ESP32-C6 内蔵 USB-JTAG 用)
#   esptool / west / cmake / ninja / ccache
#
# ビルド:
#   docker build -f .devcontainer/Dockerfile -t bbor-devcontainer .

FROM ubuntu:24.04

ARG DEBIAN_FRONTEND=noninteractive
ARG ZEPHYR_VERSION=v4.3.0
ARG ZEPHYR_SDK_VERSION=0.17.4
ARG OPENOCD_ESP_VERSION=0.12.0-esp32-20260831
ARG LLVM_VERSION=21

# ---------------------------------------------------------------- 基本ツール
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates wget curl gnupg xz-utils file less vim-tiny locales \
    git cmake ninja-build device-tree-compiler gperf ccache \
    python3 python3-pip python3-venv \
    libusb-1.0-0 usbutils dfu-util \
    openocd \
    sudo \
 && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# ---------------------------------------------------------------- LLVM
# apt.llvm.org から clang/clang++/clang-tidy/clang-format/clangd のみ導入。
RUN wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key | gpg --dearmor -o /usr/share/keyrings/llvm.gpg \
 && echo "deb [signed-by=/usr/share/keyrings/llvm.gpg] http://apt.llvm.org/noble/ llvm-toolchain-noble-${LLVM_VERSION} main" \
      > /etc/apt/sources.list.d/llvm.list \
 && apt-get update && apt-get install -y --no-install-recommends \
    clang-${LLVM_VERSION} clang-format-${LLVM_VERSION} clang-tidy-${LLVM_VERSION} clangd-${LLVM_VERSION} \
 && for t in clang clang++ clang-format clang-tidy clangd; do \
      update-alternatives --install /usr/bin/$t $t /usr/bin/$t-${LLVM_VERSION} 100; \
    done \
 && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------- Python (west / esptool)
ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv ${VIRTUAL_ENV}
ENV PATH="${VIRTUAL_ENV}/bin:${PATH}"
RUN pip install --no-cache-dir west esptool

# ---------------------------------------------------------------- Zephyr ワークスペース
ENV ZEPHYR_WS_DIR=/opt/zephyr_ws
ENV ZEPHYR_BASE=${ZEPHYR_WS_DIR}/zephyr
RUN west init -m https://github.com/zephyrproject-rtos/zephyr --mr ${ZEPHYR_VERSION} ${ZEPHYR_WS_DIR} \
 && cd ${ZEPHYR_WS_DIR} \
 && west update \
 && west zephyr-export

# requirements.txt 側が blobs fetch の https fetcher に必要な requests を含むため先に導入する。
RUN pip install --no-cache-dir -r ${ZEPHYR_BASE}/scripts/requirements.txt \
 && cd ${ZEPHYR_WS_DIR} \
 && west blobs fetch hal_espressif

# ---------------------------------------------------------------- Zephyr SDK
# minimal + arm-zephyr-eabi (nucleo_g474re) + riscv64-zephyr-elf (xiao_esp32c6)
ARG SDK_URL=https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v${ZEPHYR_SDK_VERSION}
ENV ZEPHYR_SDK_INSTALL_DIR=/opt/zephyr-sdk-${ZEPHYR_SDK_VERSION}
RUN mkdir -p ${ZEPHYR_SDK_INSTALL_DIR} && cd ${ZEPHYR_SDK_INSTALL_DIR} \
 && wget -qO- ${SDK_URL}/zephyr-sdk-${ZEPHYR_SDK_VERSION}_linux-x86_64_minimal.tar.xz | tar -xJ --strip-components=1 \
 && wget -qO- ${SDK_URL}/toolchain_linux-x86_64_arm-zephyr-eabi.tar.xz | tar -xJ \
 && wget -qO- ${SDK_URL}/toolchain_linux-x86_64_riscv64-zephyr-elf.tar.xz | tar -xJ \
 && ./setup.sh -y >/dev/null 2>&1 || true

# ---------------------------------------------------------------- Espressif OpenOCD
# ESP32-C6 は SoC 内蔵 USB-JTAG でデバッグする (board/esp32c6-builtin.cfg)。
# Debian 同梱の openocd は STM32 用に使い、ESP32 用は Espressif fork を分離導入する。
ARG ESP_OCD_URL=https://github.com/espressif/openocd-esp32/releases/download/v${OPENOCD_ESP_VERSION}
ENV OPENOCD_ESP32=/opt/openocd-esp32
RUN mkdir -p ${OPENOCD_ESP32} \
 && wget -qO- ${ESP_OCD_URL}/openocd-esp32-linux-amd64-${OPENOCD_ESP_VERSION}.tar.gz | tar -xz --strip-components=1 -C ${OPENOCD_ESP32}

# ---------------------------------------------------------------- Tauri アプリ開発環境
# app_ws/robot_controller_tauri のビルド・検証環境。
#   Node.js (公式 tarball) + Rust toolchain (ubuntu ユーザ) + Tauri/BlueZ Linux 依存
# コンテナ内ビルドの成果物は Linux 向け。Windows 向けインストーラはホスト側で
# `npm run tauri build` を実行する。BLE 実行は Linux ホストの BlueZ/D-Bus 経由に限る。
ARG NODE_VERSION=24.19.0
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential pkg-config libssl-dev \
    libwebkit2gtk-4.1-dev libgtk-3-dev libayatana-appindicator3-dev \
    librsvg2-dev patchelf \
    libdbus-1-dev libudev-dev bluez dbus \
    rpm libfuse2t64 desktop-file-utils \
 && rm -rf /var/lib/apt/lists/* \
 && mkdir -p /opt/node \
 && wget -qO- https://nodejs.org/dist/v${NODE_VERSION}/node-v${NODE_VERSION}-linux-x64.tar.xz \
      | tar -xJ --strip-components=1 -C /opt/node
ENV PATH="/opt/node/bin:${PATH}"

# ---------------------------------------------------------------- Zephyr 環境変数
ENV ZEPHYR_TOOLCHAIN_VARIANT=zephyr
# ツールチェーンを PATH に入れ、cortex-debug 既定名 arm-none-eabi-gdb にも応える。
ENV PATH="${ZEPHYR_SDK_INSTALL_DIR}/arm-zephyr-eabi/bin:${ZEPHYR_SDK_INSTALL_DIR}/riscv64-zephyr-elf/bin:${PATH}"
RUN for t in gcc g++ gdb objcopy objdump nm readelf size strings addr2line; do \
      ln -sf ${ZEPHYR_SDK_INSTALL_DIR}/arm-zephyr-eabi/bin/arm-zephyr-eabi-$t /usr/local/bin/arm-none-eabi-$t; \
    done

# ---------------------------------------------------------------- 非 root ユーザ
# ubuntu:24.04 には UID/GID 1000 の ubuntu ユーザが同梱済みのためそれを利用する。
# ツールチェーン群は読み取り・実行のみで足りるため root 所有のままにし、
# 書き込みが必要な場合はパスワードなし sudo で対処する。
ARG USERNAME=ubuntu
# Zephyr は configure 時に ${ZEPHYR_BASE}/.cache へ書き込むため同所だけ ubuntu 所有にする。
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && mkdir -p ${ZEPHYR_BASE}/.cache \
 && chown -R ${USERNAME}:${USERNAME} ${ZEPHYR_BASE}/.cache
USER ${USERNAME}
# bind mount したリポジトリを git が "dubious ownership" で拒否しないよう許可する。
# ホスト Windows 側が autocrlf で CRLF チェックアウトしているため、コンテナ側も
# autocrlf=true に揃えて「全文差分」表示を防ぐ（.gitattributes で *.sh は LF 固定）。
RUN git config --global --add safe.directory '*' \
 && git config --global core.autocrlf true

# ---------------------------------------------------------------- Rust toolchain
# Tauri バックエンド用。remoteUser (ubuntu) のホームに rustup で導入する。
ARG RUST_TOOLCHAIN=stable
RUN export HOME=/home/${USERNAME} \
 && curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs \
      | sh -s -- -y --default-toolchain "${RUST_TOOLCHAIN}" --profile minimal --no-modify-path \
 && "$HOME/.cargo/bin/rustup" component add clippy rustfmt
ENV PATH="/home/${USERNAME}/.cargo/bin:${PATH}"

WORKDIR /workspaces
