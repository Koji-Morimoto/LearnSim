# 2026-11-18  やり方 B: Isaac Sim を headless で起動し、デバッガの接続を待ってから b_kit_exec.py を動かす
#
# 使い方 (PowerShell):
#   .\run_b.ps1                                  # 下の既定値で起動
#   .\run_b.ps1 -IsaacSimRoot D:\isaacsim-6.2.0   # Isaac Sim の場所を指定して起動
#   .\run_b.ps1 -Port 3001 -WaitSeconds 60       # ポートと待ち時間を変える
# 起動ログが止まったら、VSCode で「B: Attach (port 3000)」を選んで F5 を押す。
#
# 【環境に合わせて書き換えるところ】
#   $IsaacSimRoot の既定値 … Isaac Sim をインストールしたフォルダ（isaac-sim.bat が入っているフォルダ）
#   $Port の既定値         … .vscode/launch.json の B の port と同じ値にする
param(
    [string]$IsaacSimRoot = "C:\isaacsim-6.2.0",
    [int]$Port = 3000,
    # デバッガの接続を待つ秒数。過ぎると接続を待たずに先へ進む（-1 で無限に待つ）
    [double]$WaitSeconds = 30
)

$launcher = Join-Path $IsaacSimRoot "isaac-sim.bat"
if (-not (Test-Path $launcher)) {
    Write-Error "isaac-sim.bat が見つかりません: $launcher  (-IsaacSimRoot でインストール先を指定してください)"
    exit 1
}

# このスクリプトと同じフォルダの b_kit_exec.py を、/ 区切りの絶対パスで渡す
$script = (Join-Path $PSScriptRoot "b_kit_exec.py") -replace '\\', '/'

& $launcher `
    --no-window `
    --enable omni.kit.debug.python `
    "--/exts/omni.kit.debug.python/port=$Port" `
    "--/exts/omni.kit.debug.python/waitForClient=true" `
    "--/exts/omni.kit.debug.python/waitTimeout=$WaitSeconds" `
    --exec "$script"
