# 検証スクリプトを GUI とは別プロセスの headless Isaac Sim で実行する。
# 使い方: powershell -File run.ps1 n01_namespace.py [m01_play_matrix.py ...]   引数なしなら <英字><数字>_*.py を全部
#   追加引数は -ScriptArgs で渡す(例: -ScriptArgs '--physx-fabric' / -ScriptArgs '--gpu')
#   ログ: ../results/<スクリプト名><接尾辞>.log / 結果: ../results/<スクリプト名><接尾辞>.json
[CmdletBinding(PositionalBinding = $false)]  # -ScriptArgs は名前付きでだけ受け取り、位置引数はすべて $Scripts に入れる
param(
    [string]$ScriptArgs = '',
    [Parameter(ValueFromRemainingArguments = $true)][string[]]$Scripts
)
$ErrorActionPreference = 'Continue'
# システムの PYTHONPATH (C:\BuiltOpenUSD) は Isaac の pxr を隠すので空にする
$env:PYTHONPATH = ''
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $here
$results = Join-Path (Split-Path -Parent $here) 'results'
New-Item -ItemType Directory -Force $results | Out-Null
# -Filter はファイルシステムのワイルドカードで [...] を解釈しないので、正規表現で絞る
if (-not $Scripts) { $Scripts = (Get-ChildItem -File | Where-Object { $_.Name -match '^[a-z]\d+_.*\.py$' } | Sort-Object Name).Name }
# bplib.SUFFIX と同じ規則でログ名の接尾辞を作る
$suffix = ''
if ($ScriptArgs -match 'physx-fabric') { $suffix += '__physx_fabric' }
if ($ScriptArgs -match '--gpu') { $suffix += '__gpu' }
if ($ScriptArgs -match '--no-fsd') { $suffix += '__no_fsd' }
foreach ($s in $Scripts) {
    $log = Join-Path $results ([IO.Path]::GetFileNameWithoutExtension($s) + $suffix + '.log')
    # 既存のログが他プロセス(開いている Isaac Sim の GUI など)にロックされていたら、別名に書く
    if (Test-Path $log) {
        try { $f = [IO.File]::Open($log, 'Open', 'ReadWrite', 'None'); $f.Close() }
        catch { $log = Join-Path $results ([IO.Path]::GetFileNameWithoutExtension($s) + $suffix + '_run2.log') }
    }
    $t0 = Get-Date
    cmd /c "C:\isaacsim-6.2.0\python.bat `"$s`" $ScriptArgs --no-ros-env > `"$log`" 2>&1"
    Write-Output "script=$s args=$ScriptArgs exit=$LASTEXITCODE elapsed=$((Get-Date) - $t0)"
    Select-String -Path $log -Encoding UTF8 -Pattern '\[RESULT\]|\[DONE\]|\[WATCHDOG\]|!EXC|Traceback' | ForEach-Object { $_.Line.Substring(0, [Math]::Min(600, $_.Line.Length)) }
}
