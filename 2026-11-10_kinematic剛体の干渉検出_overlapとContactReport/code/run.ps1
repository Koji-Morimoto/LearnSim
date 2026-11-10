# 検証スクリプトを GUI とは別プロセスの headless Isaac Sim で実行する。
# 使い方: powershell -File run.ps1 verify_snippets.py   /   powershell -File run.ps1 p01_play_speed.py
param([Parameter(Mandatory = $true)][string]$Script, [string]$ScriptArgs = '')
$ErrorActionPreference = 'Continue'
$env:PYTHONPATH = ''  # システムの PYTHONPATH は Isaac の pxr を隠すので空にする
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $here
$results = Join-Path (Split-Path -Parent $here) 'results'
New-Item -ItemType Directory -Force $results | Out-Null
$log = Join-Path $results ([IO.Path]::GetFileNameWithoutExtension($Script) + '.log')
$t0 = Get-Date
cmd /c "C:\isaacsim-6.2.0\python.bat `"$Script`" $ScriptArgs --no-ros-env > `"$log`" 2>&1"
Write-Output "script=$Script args=$ScriptArgs exit=$LASTEXITCODE elapsed=$((Get-Date) - $t0)"
Select-String -Path $log -Encoding UTF8 -Pattern '\[RESULT\]|\[DONE\]|\[WATCHDOG\]|!EXC|Traceback|^x=' | ForEach-Object { $_.Line.Substring(0, [Math]::Min(2500, $_.Line.Length)) }
