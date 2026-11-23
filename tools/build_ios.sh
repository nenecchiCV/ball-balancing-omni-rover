#!/usr/bin/env bash
# robot_controller_tauri を iOS アプリとして Xcode でビルド・デプロイする。
#
# 使い方:
#   tools/build_ios.sh init     # ツールチェーン確認 + iOS ターゲット + npm ci
#   tools/build_ios.sh sim      # iOS シミュレータ向けビルド（署名不要、BLE 非動作）
#   tools/build_ios.sh device   # 実機向けビルド＋接続デバイスへインストール
#   tools/build_ios.sh open     # 生成済み Xcode プロジェクトを Xcode で開く
#
# 前提: Xcode + Command Line Tools。初回は `init` を実行する。
# 実機: Apple Development 証明書がキーチェーンにあること（Xcode → Settings →
#   Accounts → Apple ID 追加 → Manage Certificates → + → Apple Development）。
#   Team ID は TAURI_DEV_TEAM 環境変数または tauri.conf.json の
#   bundle.iOS.developmentTeam で指定する（単一チームなら自動検出を試みる）。
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
APP_DIR="$REPO_ROOT/app_ws/robot_controller_tauri"
GEN_DIR="$APP_DIR/src-tauri/gen/apple"
# Homebrew vendored ruby 4.x（CocoaPods の実行用。macOS 付属 ruby 2.6 は非対応）
PORTABLE_RUBY_BIN="/opt/homebrew/Library/Homebrew/vendor/portable-ruby/current/bin"
GEM_BIN="$HOME/.gem/ruby/4.0.0/bin"

# 最新の Xcode*.app を使う（XCODE_APP=/Applications/Xcode-XX.app で上書き可）。
# 注意: 実機署名では xcode-select がプロジェクト生成に使った Xcode と同系を
# 指している必要がある（sudo xcode-select -s /Applications/Xcode-XX.app）。
# 旧 Xcode が指されたままだと "No Accounts" エラーになる。DEVELOPER_DIR は
# ビルドツール選択のみを強制し、プロビジョニング解決には効かない。
if [ -z "${XCODE_APP:-}" ]; then
  XCODE_APP="$(ls -d /Applications/Xcode*.app 2>/dev/null | sort -V | tail -1)"
fi
export DEVELOPER_DIR="${DEVELOPER_DIR:-$XCODE_APP/Contents/Developer}"
XCODE_TOOLCHAIN_BIN="$DEVELOPER_DIR/Toolchains/XcodeDefault.xctoolchain/usr/bin"

# Xcode toolchain を PATH 先頭に置く（CLT の swift だと iOS 用 swift ライブラリを解決できずリンク失敗する）
export PATH="$XCODE_TOOLCHAIN_BIN:$HOME/.local/bin:$HOME/.local/node/bin:$HOME/.cargo/bin:$GEM_BIN:$PORTABLE_RUBY_BIN:$PATH"
export LANG="${LANG:-en_US.UTF-8}"

log() { printf '\033[1;34m==>\033[0m %s\n' "$*"; }
die() { printf '\033[1;31merror:\033[0m %s\n' "$*" >&2; exit 1; }

ensure_toolchain() {
  command -v rustup >/dev/null || die "rustup がありません。https://rustup.rs から導入するか brew install rustup"
  command -v node >/dev/null || die "node がありません。~/.local/node に展開するか brew install node"
  command -v xcodegen >/dev/null || die "xcodegen がありません。https://github.com/yonaskolb/XcodeGen/releases のバイナリを ~/.local に展開"
  command -v pod >/dev/null || die "pod がありません。gem install --user-install cocoapods（brew vendored ruby 利用可）"
  xcodebuild -version
  log "rust targets: aarch64-apple-ios / aarch64-apple-ios-sim"
  rustup target add aarch64-apple-ios aarch64-apple-ios-sim
  log "npm ci"
  (cd "$APP_DIR" && npm ci)
}

ensure_ios_project() {
  if [ -z "$(find "$GEN_DIR" -maxdepth 1 -name '*.xcodeproj' 2>/dev/null)" ]; then
    log "tauri ios init（Xcode プロジェクト生成）"
    (cd "$APP_DIR" && npm run tauri -- ios init)
  fi
  # BLE 用 CoreBluetooth をリンク対象に追加（tauri の project.yml テンプレートは
  # bundle.iOS.frameworks を反映しないため、再生成のたびにここで差し戻す）
  if ! grep -q "CoreBluetooth.framework" "$GEN_DIR/project.yml"; then
    log "project.yml に CoreBluetooth.framework を追加"
    sed -i '' 's/      - sdk: WebKit.framework/      - sdk: WebKit.framework\n      - sdk: CoreBluetooth.framework/' "$GEN_DIR/project.yml"
    (cd "$GEN_DIR" && xcodegen generate)
  fi
}

resolve_team() {
  TAURI_DEV_TEAM="${TAURI_DEV_TEAM:-${APPLE_DEVELOPMENT_TEAM:-}}"
  if [ -z "$TAURI_DEV_TEAM" ]; then
    # キーチェーンの署名証明書から Team ID を推定（組織単位フィールドの値）
    TAURI_DEV_TEAM="$(security find-identity -v -p codesigning 2>/dev/null \
      | grep -o 'Apple Development[^"]*\|iPhone Developer[^"]*' >/dev/null \
      && security find-certificate -c "Apple Development" -p 2>/dev/null \
      | openssl x509 -noout -subject 2>/dev/null | grep -o 'OU=[A-Z0-9]*' | cut -d= -f2 || true)"
  fi
  [ -n "$TAURI_DEV_TEAM" ] || die "Team ID 不明。TAURI_DEV_TEAM=<Team ID> を指定するか、Xcode に Apple ID を登録して Apple Development 証明書を作成してください"
  export APPLE_DEVELOPMENT_TEAM="$TAURI_DEV_TEAM"
  log "development team: $TAURI_DEV_TEAM"
}

deploy_to_device() {
  local ipa="$GEN_DIR/build/arm64/robot-controller.ipa"
  local app="$GEN_DIR/build/arm64/robot-controller.app"
  local artifact
  if [ -f "$ipa" ]; then
    artifact="$ipa"
  elif [ -d "$app" ]; then
    artifact="$app"
  else
    die "$ipa / $app が見つかりません"
  fi
  local udid
  udid="$(xcrun devicectl list devices 2>/dev/null | awk '$0 !~ /simulated/ && /connected|available/ {for(i=1;i<=NF;i++) if ($i ~ /^[0-9A-F-]{20,}/) {gsub(/\(UDID\)/,"",$i); print $i; exit}}')"
  [ -n "$udid" ] || die "接続済み iOS デバイスが見つかりません"
  log "devicectl install → $udid"
  xcrun devicectl device install app --device "$udid" "$artifact"
  log "インストール完了。デバイス上で robot-controller を起動してください"
}

case "${1:-sim}" in
  init) ensure_toolchain ;;
  sim)
    ensure_toolchain
    ensure_ios_project
    log "tauri ios build --target aarch64-sim（シミュレータ向け .app）"
    (cd "$APP_DIR" && npm run tauri -- ios build --target aarch64-sim --debug)
    log "完了: $GEN_DIR/build/arm64-sim/robot-controller.app"
    ;;
  device)
    ensure_toolchain
    ensure_ios_project
    resolve_team
    log "tauri ios build（実機向け, team=${TAURI_DEV_TEAM}）"
    (cd "$APP_DIR" && npm run tauri -- ios build)
    deploy_to_device
    ;;
  open)
    ensure_ios_project
    open "$GEN_DIR"/robot_controller_tauri_lib.xcodeproj
    ;;
  *) die "usage: $0 [init|sim|device|open]" ;;
esac
