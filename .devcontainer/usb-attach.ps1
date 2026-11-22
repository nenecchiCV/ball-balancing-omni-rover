# usb-attach.ps1 — ST-LINK / ESP32-C6 USB-JTAG を usbipd 経由で devcontainer へ渡す
#
# 使い方（管理者 PowerShell で実行）:
#   .devcontainer\usb-attach.ps1            # 検出した全ターゲットを attach
#   .devcontainer\usb-attach.ps1 -Detach    # 全ターゲットを detach
#   .devcontainer\usb-attach.ps1 -BusId 2-3 # 個別バス ID を指定して attach
#
# 前提: winget install usbipd / コンテナは --privileged で起動済み。

param(
    [switch]$Detach,
    [string]$BusId
)

# 対象デバイスの VID:PID パターン
#   0483:37xx  STMicroelectronics ST-LINK (V2/V2-1/V3 全系)
#   303a:1001  Espressif ESP32-C6 内蔵 USB-Serial-JTAG
$TargetPattern = '^(?<busid>\d+-\d+)\s+(?<vidpid>(0483:37..|303a:1001))\s+(?<device>.+?)\s{2,}(?<state>.+?)\s*$'

function Get-Targets {
    usbipd list | Select-String -Pattern $TargetPattern | ForEach-Object {
        $m = $_.Matches[0].Groups
        [PSCustomObject]@{
            BusId  = $m['busid'].Value
            VidPid = $m['vidpid'].Value
            Device = $m['device'].Value.Trim()
            State  = $m['state'].Value
        }
    }
}

$targets = if ($BusId) {
    usbipd list | Select-String -Pattern "^(?<busid>$([regex]::Escape($BusId)))\s+(?<vidpid>\S+)\s+(?<device>.+?)\s{2,}(?<state>.+?)\s*$" | ForEach-Object {
        $m = $_.Matches[0].Groups
        [PSCustomObject]@{ BusId = $m['busid'].Value; VidPid = $m['vidpid'].Value; Device = $m['device'].Value.Trim(); State = $m['state'].Value }
    }
} else {
    Get-Targets
}

if (-not $targets) {
    Write-Host "対象デバイスが見つかりません。usbipd list の出力を確認し -BusId を指定してください:"
    usbipd list
    exit 1
}

foreach ($t in $targets) {
    Write-Host "$($t.BusId)  $($t.VidPid)  $($t.Device)  [$($t.State)]"
    if ($Detach) {
        usbipd detach --busid $t.BusId
        continue
    }
    if ($t.State -eq 'Not shared') {
        usbipd bind --busid $t.BusId
    }
    usbipd attach --wsl --busid $t.BusId
}

Write-Host ""
Write-Host "コンテナ内では以下でデバイスを確認できます:"
Write-Host "  lsusb    # 0483:37xx (ST-LINK) / 303a:1001 (ESP32-C6)"
Write-Host "  ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null"
