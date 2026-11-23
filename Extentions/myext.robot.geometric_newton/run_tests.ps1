# テストを全部回す。失敗が 1 つでもあれば終了コード 1。
#   powershell -ExecutionPolicy Bypass -File run_tests.ps1            両方
#   powershell -ExecutionPolicy Bypass -File run_tests.ps1 -Pure      pure テスト(numpy + Newton GeometricScene)
#   powershell -ExecutionPolicy Bypass -File run_tests.ps1 -Kit       USD / Fabric / PhysX / Newton Play=ON テスト(headless Isaac Sim)
#   powershell -ExecutionPolicy Bypass -File run_tests.ps1 -Kit -Only newton      名前に newton を含むテストだけ
param([switch]$Pure, [switch]$Kit, [string]$Only = '')
$ErrorActionPreference = 'Continue'
[Console]::OutputEncoding = [Text.Encoding]::UTF8
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $here
$env:PYTHONPATH = ''      # システムの PYTHONPATH は Isaac Sim の pxr を隠すので空にする
$env:PYTHONUTF8 = '1'
if (-not $Pure -and -not $Kit) { $Pure = $true; $Kit = $true }
$isaac = if ($env:ISAAC_SIM_DIR) { $env:ISAAC_SIM_DIR } else { 'C:\isaacsim-6.2.0' }
$failed = @()

if ($Pure) {
    # Isaac Sim 同梱の Python で pure テスト(numpy モデル + OpenUSD/Newton GeometricScene)を回す
    $log = Join-Path $here 'tests_pure\last_run.log'
    cmd /c "`"$isaac\python.bat`" -m unittest discover -s tests_pure -v > `"$log`" 2>&1"
    $code = $LASTEXITCODE
    Get-Content -Encoding UTF8 $log | Select-Object -Last 16
    if ($code -ne 0) { $failed += 'pure' }
    Write-Output "== tests_pure (model + Newton GeometricScene): exit=$code (ログ: tests_pure\last_run.log)"
}

if ($Kit) {
    $log = Join-Path $here 'tests_kit\last_run.log'
    $extra = if ($Only) { "--only=$Only" } else { '' }
    cmd /c "`"$isaac\python.bat`" tests_kit\run_kit_tests.py $extra --no-ros-env > `"$log`" 2>&1"
    $code = $LASTEXITCODE
    Select-String -Path $log -Encoding UTF8 -Pattern '^\[PASS\]|^\[FAIL\]|^\[SUMMARY\]|^\[WATCHDOG\]' | ForEach-Object { $_.Line }
    if ($code -ne 0) { $failed += 'kit' }
    Write-Output "== tests_kit (Kit / Fabric / Play=ON Newton): exit=$code (ログ: tests_kit\last_run.log、測った値: tests_kit\last_run.json)"
}

if ($failed.Count) { Write-Output "FAILED: $($failed -join ', ')"; exit 1 }
Write-Output 'ALL PASSED'
exit 0
