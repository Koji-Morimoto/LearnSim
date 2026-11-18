# テストを全部回す(headless の Isaac Sim を別プロセスで 1 つ起動する。2〜3 分)。失敗が 1 つでもあれば終了コード 1。
#   powershell -ExecutionPolicy Bypass -File run_tests.ps1
#   powershell -ExecutionPolicy Bypass -File run_tests.ps1 -Only franka     名前に franka を含むテストだけ
param([string]$Only = '')
$ErrorActionPreference = 'Continue'
[Console]::OutputEncoding = [Text.Encoding]::UTF8
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $here
$env:PYTHONPATH = ''      # システムの PYTHONPATH は Isaac Sim の pxr を隠すので空にする
$env:PYTHONUTF8 = '1'
$isaac = if ($env:ISAAC_SIM_DIR) { $env:ISAAC_SIM_DIR } else { 'C:\isaacsim-6.2.0' }
$log = Join-Path $here 'tests_kit\last_run.log'
$extra = if ($Only) { "--only=$Only" } else { '' }
cmd /c "`"$isaac\python.bat`" tests_kit\run_tests.py $extra > `"$log`" 2>&1"
$code = $LASTEXITCODE
# Kit が例外を握りつぶして 0 で終わることがあるので、[SUMMARY] が無いか failed が 0 でなければ失敗にする
$summary = Select-String -Path $log -Encoding UTF8 -Pattern '^\[SUMMARY\] passed \d+ / failed 0$'
if (-not $summary -and $code -eq 0) { $code = 1 }
Select-String -Path $log -Encoding UTF8 -Pattern '^\[PASS\]|^\[FAIL\]|^\[SUMMARY\]|^\[WATCHDOG\]' | ForEach-Object { $_.Line }
Write-Output "== exit=$code (ログ: tests_kit\last_run.log、結果: tests_kit\last_run.json)"
exit $code
