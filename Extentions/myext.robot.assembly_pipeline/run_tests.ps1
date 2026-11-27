<#
.SYNOPSIS
    myext.robot.assembly_pipeline の統合テストスイート（tests_kit\run_tests.py）を Headless 実行するラッパー。

.DESCRIPTION
    1. 外部の PYTHONPATH が Isaac Sim 同梱の pxr / numpy を隠す事故を防ぐため PYTHONPATH を空にする。
    2. C:\isaacsim-6.2.0\tools\myext\run_sim.bat 経由で Headless の Isaac Sim を 1 つだけ起動し、
       tests_kit\run_tests.py の全テストを順次実行する（ログは tests_kit\last_run.log に保存）。
    3. 実行結果の要約（[PASS] / [FAIL] / [SUMMARY] / [WATCHDOG]）を端末へ抜き出して表示し、
       失敗が 1 件でもあるか [SUMMARY] に到達しなかった場合は終了コード 1 を返す。

    公式アセットとの定量的な PhysX 動作一致テスト（全 6 テスト）を実行する場合は、
    同じ環境で tests_kit\test_official_motion_match.py を直接 run_sim.bat で実行してください。
    詳細は tests_kit\README.md を参照してください。

.PARAMETER Only
    テスト関数名に含まれる部分文字列で実行対象を絞り込む（省略時は全テストを実行）。

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File run_tests.ps1
.EXAMPLE
    powershell -ExecutionPolicy Bypass -File run_tests.ps1 -Only franka
#>

param(
    [string]$Only = ''
)

# Kit / run_sim.bat の標準エラー出力で PowerShell が途中停止しないよう Continue にする
$ErrorActionPreference = 'Continue'
[Console]::OutputEncoding = [Text.Encoding]::UTF8

# スクリプト自身のフォルダ（拡張ルート）をカレントディレクトリにする
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $here

# システムの PYTHONPATH に BuiltOpenUSD 等が入っていると Isaac Sim の pxr を隠すため必ず空にする
$env:PYTHONPATH = ''
$env:PYTHONUTF8 = '1'

# 環境変数 ISAAC_SIM_DIR が未設定なら既定の C:\isaacsim-6.2.0 を使う
$isaac = if ($env:ISAAC_SIM_DIR) { $env:ISAAC_SIM_DIR } else { 'C:\isaacsim-6.2.0' }
$runner = Join-Path $isaac 'tools\myext\run_sim.bat'

# 全出力（Kit の起動ログ + テスト出力）の保存先
$log = Join-Path $here 'tests_kit\last_run.log'
$extra = if ($Only) { "--only=$Only" } else { '' }

# Headless の Isaac Sim を 1 回起動して tests_kit\run_tests.py を実行する
cmd /c "`"$runner`" tests_kit\run_tests.py $extra > `"$log`" 2>&1"
$code = $LASTEXITCODE

# Kit が例外を握りつぶして終了コード 0 で終わる場合があるため、
# ログ内に "[SUMMARY] passed <N> / failed 0" が無ければ失敗（終了コード 1）として扱う
$summary = Select-String -Path $log -Encoding UTF8 -Pattern '^\[SUMMARY\] passed \d+ / failed 0$'
if (-not $summary -and $code -eq 0) {
    $code = 1
}

# テストごとの成否とサマリだけを端末に表示する
Select-String -Path $log -Encoding UTF8 -Pattern '^\[PASS\]|^\[FAIL\]|^\[SUMMARY\]|^\[WATCHDOG\]' | ForEach-Object { $_.Line }
Write-Output "== exit=$code (ログ: tests_kit\last_run.log、結果: tests_kit\last_run.json)"
exit $code
