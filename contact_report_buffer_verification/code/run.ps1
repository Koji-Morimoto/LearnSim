# 検証スクリプトを GUI とは別プロセスの headless Isaac Sim で実行する。
# 使い方: powershell -File run.ps1 b01_pull_semantics.py [b02_substep.py ...]   引数なしなら b*.py を全部
#   ログ: ../results/<スクリプト名>.log / 結果: ../results/<スクリプト名>.json
param([string[]]$Scripts)
$ErrorActionPreference = 'Continue'
# システムの PYTHONPATH (C:\BuiltOpenUSD) は Isaac の pxr を隠すので空にする
$env:PYTHONPATH = ''
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $here
$results = Join-Path (Split-Path -Parent $here) 'results'
New-Item -ItemType Directory -Force $results | Out-Null
if (-not $Scripts) { $Scripts = (Get-ChildItem -Filter 'b*.py' | Sort-Object Name).Name }
foreach ($s in $Scripts) {
    $log = Join-Path $results ([IO.Path]::GetFileNameWithoutExtension($s) + '.log')
    $t0 = Get-Date
    cmd /c "C:\isaacsim-6.2.0\python.bat `"$s`" --no-ros-env > `"$log`" 2>&1"
    Write-Output "script=$s exit=$LASTEXITCODE elapsed=$((Get-Date) - $t0)"
    Select-String -Path $log -Encoding UTF8 -Pattern '\[RESULT\]|\[DONE\]|\[WATCHDOG\]|!EXC|Traceback|Error' | ForEach-Object { $_.Line.Substring(0, [Math]::Min(1500, $_.Line.Length)) }
}
