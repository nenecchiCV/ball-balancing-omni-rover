Write-Output '=== ESP32 複合ノード Status 20秒監視 ==='
$prev = $null; $chg = 0
1..10 | ForEach-Object {
    $d = Get-PnpDevice -ErrorAction SilentlyContinue | Where-Object { $_.InstanceId -eq 'USB\VID_303A&PID_1001\9C:CC:01:18:DC:98' }
    $cur = if ($d) { $d.Status } else { 'GONE' }
    if ($null -ne $prev -and $cur -ne $prev) { $chg++; Write-Output ("{0}  {1} -> {2}" -f (Get-Date -Format 'HH:mm:ss.ff'), $prev, $cur) }
    $prev = $cur
    Start-Sleep -Seconds 2
}
Write-Output "ESP32 変化回数: $chg"

Write-Output ''
Write-Output '=== 直近10分の Kernel-PnP 420(削除)/410(開始) イベント（全デバイス） ==='
$since = (Get-Date).AddMinutes(-10)
$evs = Get-WinEvent -FilterHashtable @{LogName='Microsoft-Windows-Kernel-PnP/Configuration'; StartTime=$since} -ErrorAction SilentlyContinue
$evs | Where-Object { $_.Id -in 400,410,420 } | Group-Object Id | Select-Object Name,Count
Write-Output '--- 時系列（直近20件） ---'
$evs | Where-Object { $_.Id -in 400,410,420 } | Select-Object -First 20 TimeCreated,Id,@{n='Dev';e={($_.Properties[0].Value -split '\\')[0..1] -join '\'}} | Format-Table -AutoSize

Write-Output ''
Write-Output '=== usbipd list（303a/0483 行） ==='
usbipd list | Select-String '303a|0483'
