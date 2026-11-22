# usb-autowatch.ps1 — ST-LINK / ESP32-C6 を usbipd 経由で自動 attach（常駐プロセスなし）
#
# 仕組み:
#   Scheduled Task が次のタイミングで本スクリプトを -Once で 1 回だけ実行します。
#     - USB/PNP デバイス開始イベント（Microsoft-Windows-Kernel-PnP/Configuration, ID 410）
#     - ログオン時
#     - -FallbackMin 分周期（WSL 再起動で detach された場合の復帰用）
#   常駐プロセス・ポーリングループはタスク登録時には使いません。
#
# 初回セットアップ（管理者 PowerShell で 1 回だけ）:
#   .devcontainer\usb-autowatch.ps1 -Install    # Scheduled Task 登録
#
# 運用:
#   .devcontainer\usb-autowatch.ps1 -Suspend    # 自動 attach を一時停止（ホスト側で COM ポートを使うとき等）
#   .devcontainer\usb-autowatch.ps1 -Resume     # 再開
#   .devcontainer\usb-autowatch.ps1 -Uninstall  # Scheduled Task を削除（管理者 PowerShell）
#
# 手動実行・テスト:
#   .devcontainer\usb-autowatch.ps1 -Once       # 1 回だけスキャン
#   .devcontainer\usb-autowatch.ps1             # フォアグラウンド常駐モード（タスクを使わない手動運用向け）
#
# 前提: winget install usbipd / WSL2 / コンテナは --privileged で起動済み。

param(
    [switch]$Install,
    [switch]$Uninstall,
    [switch]$Suspend,
    [switch]$Resume,
    [switch]$Once,
    [int]$DelaySec = 0,        # -Once 実行前の待機（PnP イベント直後の列挙完了待ち）
    [int]$FallbackMin = 15,    # -Install: 周期スキャンの間隔（分）
    [int]$IntervalSec = 2,     # 常駐モードのポーリング間隔
    [string]$Distribution = '' # 既定以外の WSL ディストロへ attach する場合に指定
)

$TaskName    = 'usbipd-autowatch'
$SuspendFile = Join-Path $env:LOCALAPPDATA 'usb-autowatch.suspend'

# 対象デバイスの VID:PID パターン（usb-attach.ps1 と共通）
#   0483:37xx  STMicroelectronics ST-LINK (V2/V2-1/V3 全系)
#   303a:1001  Espressif ESP32-C6 内蔵 USB-Serial-JTAG
$TargetPattern = '^(?<busid>\d+-\d+)\s+(?<vidpid>(0483:37..|303a:1001))\s+(?<device>.+?)\s{2,}(?<state>.+?)\s*$'

function Test-Admin {
    ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()
    ).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

# 未 attach の対象デバイスを bind + attach する 1 回分のスキャン。
# $Seen に前回の状態・エラーを持たせ、同一内容の重複出力を抑止する。
function Invoke-UsbScan {
    param([hashtable]$Seen)
    $found = $false
    usbipd list | Select-String -Pattern $TargetPattern | ForEach-Object {
        $found  = $true
        $g      = $_.Matches[0].Groups
        $busid  = $g['busid'].Value
        $vidpid = $g['vidpid'].Value
        $device = $g['device'].Value.Trim()
        $state  = $g['state'].Value.Trim()
        if ($state -match '^Attached') { return }
        if ($Seen[$busid] -ne $state) {
            Write-Host "$(Get-Date -Format 'HH:mm:ss')  $busid  $vidpid  $device  [$state] -> attach"
            $Seen[$busid] = $state
        }
        if ($state -match '^Not') {
            usbipd bind --busid $busid | Out-Null
        }
        $wslArgs = @('--wsl')
        if ($Distribution) { $wslArgs += $Distribution }
        $out = usbipd attach @wslArgs --busid $busid 2>&1 | Out-String
        if ($LASTEXITCODE -eq 0) {
            Write-Host '  attached'
            $Seen[$busid] = 'Attached'
            $Seen["err-$busid"] = $null
        }
        else {
            $msg = $out.Trim()
            if ($Seen["err-$busid"] -ne $msg) {
                Write-Host "  attach 失敗: $msg"
                $Seen["err-$busid"] = $msg
            }
        }
    }
    return $found
}

if ($Suspend -or $Resume) {
    if ($Suspend) {
        New-Item -Path $SuspendFile -ItemType File -Force | Out-Null
        Write-Host "自動 attach を一時停止しました（再開: -Resume）"
    } else {
        Remove-Item -Path $SuspendFile -Force -ErrorAction SilentlyContinue
        Write-Host "自動 attach を再開しました"
    }
    exit 0
}

if ($Uninstall) {
    Stop-ScheduledTask -TaskName $TaskName -ErrorAction SilentlyContinue
    Unregister-ScheduledTask -TaskName $TaskName -Confirm:$false -ErrorAction SilentlyContinue
    Write-Host "Scheduled Task '$TaskName' を削除しました"
    exit 0
}

if ($Install) {
    if (-not (Test-Admin)) {
        Write-Error "Scheduled Task の登録には管理者権限が必要です。管理者 PowerShell で再実行してください。"
        exit 1
    }
    if (-not (Get-Command usbipd -ErrorAction SilentlyContinue)) {
        Write-Error "usbipd が見つかりません。先に 'winget install usbipd' を実行してください。"
        exit 1
    }
    $arg = "-NoProfile -WindowStyle Hidden -ExecutionPolicy Bypass -File `"$PSCommandPath`" -Once -DelaySec 3"
    if ($Distribution) { $arg += " -Distribution `"$Distribution`"" }
    $action    = New-ScheduledTaskAction -Execute 'powershell.exe' -Argument $arg
    $principal = New-ScheduledTaskPrincipal -UserId "$env:USERDOMAIN\$env:USERNAME" -LogonType Interactive -RunLevel Highest
    $settings  = New-ScheduledTaskSettingsSet -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries `
        -MultipleInstances IgnoreNew -ExecutionTimeLimit (New-TimeSpan -Minutes 5)

    $triggers = @(
        # ログオン時（既にデバイスが挿さっている場合に対応）
        (New-ScheduledTaskTrigger -AtLogOn -User "$env:USERDOMAIN\$env:USERNAME"),
        # 周期スキャン（WSL 再起動による detach は PnP イベントが出ないためフォールバック）
        (New-ScheduledTaskTrigger -Once -At (Get-Date) `
            -RepetitionInterval (New-TimeSpan -Minutes $FallbackMin) -RepetitionDuration (New-TimeSpan -Days 3650))
    )
    # USB/PNP デバイス開始イベント（ID 410）で即時スキャン
    $subscription = '<QueryList><Query Id="0" Path="Microsoft-Windows-Kernel-PnP/Configuration">' +
        '<Select Path="Microsoft-Windows-Kernel-PnP/Configuration">*[System[(EventID=410)]]</Select></Query></QueryList>'
    $evtClass = Get-CimClass -ClassName MSFT_TaskEventTrigger -Namespace 'Root/Microsoft/Windows/TaskScheduler'
    $triggers += New-CimInstance -CimClass $evtClass -ClientOnly -Property @{
        Enabled      = $true
        Subscription = $subscription
    }

    Register-ScheduledTask -TaskName $TaskName -Action $action -Trigger $triggers -Principal $principal `
        -Settings $settings -Description 'usbipd: ST-LINK/ESP32-C6 を WSL へ自動 attach（イベント駆動・常駐なし）' `
        -Force | Out-Null
    Write-Host "Scheduled Task '$TaskName' を登録しました（トリガ: PnP デバイス開始 / ログオン / ${FallbackMin}分周期）"
    Write-Host "動作確認: $PSCommandPath -Once    一時停止: -Suspend    削除: -Uninstall"
    exit 0
}

if (-not (Get-Command usbipd -ErrorAction SilentlyContinue)) {
    Write-Error "usbipd が見つかりません。'winget install usbipd' を実行してください。"
    exit 1
}

if ($Once) {
    if (Test-Path $SuspendFile) { exit 0 }
    if ($DelaySec -gt 0) { Start-Sleep -Seconds $DelaySec }
    if (-not (Invoke-UsbScan -Seen @{})) {
        Write-Host "対象デバイスは見つかりませんでした（usbipd list を確認してください）"
    }
    exit 0
}

# 常駐モード（タスクを使わない手動運用・デバッグ用）
Write-Host "usbipd 自動 attach 常駐モード: ST-LINK(0483:37xx) / ESP32-C6(303a:1001), ${IntervalSec}s 周期"
Write-Host "一時停止: -Suspend / Ctrl+C で終了"

$seen = @{}
while ($true) {
    if (Test-Path $SuspendFile) {
        if ($seen['_suspend'] -ne $true) {
            Write-Host "$(Get-Date -Format 'HH:mm:ss')  一時停止中（-Resume で再開）"
            $seen['_suspend'] = $true
        }
    }
    else {
        $seen['_suspend'] = $false
        if (-not (Invoke-UsbScan -Seen $seen) -and -not $seen['_notfound']) {
            Write-Host "対象デバイスは見つかりませんでした（挿入を待っています）"
            $seen['_notfound'] = $true
        }
    }
    Start-Sleep -Seconds $IntervalSec
}
