# Isaac Sim 6.2.0 バックエンドデータフロー回路図（CPU/GPU分割・実機検証付）

Isaac Sim 6.2.0 のバックエンドを **CPU DOMAIN（Host RAM）** と **GPU DOMAIN（VRAM / CUDA / Vulkan）** に PCIe 境界で分割し、各配線の駆動方式（`① CLOCK` / `② DIRTY` / `③ EVENT`）と通電状態をゆっくりとした連続信号流で可視化する単一 HTML ツールです。すべての主要経路について、Isaac Sim 6.2.0 の公式ドキュメント・ソース出典と実機実測（T1〜T9）の両輪で裏付けを取っています。

## ファイル

- `isaacsim_backend_circuit.html` — ブラウザでそのまま開けるスタンドアロン HTML（外部依存なし）
- `verify_backend_circuit.py` — Isaac Sim 6.2.0 実機検証スクリプト（`tools\myext\run_sim.bat verify_backend_circuit.py` で再現実行可能）
- `verify_backend_circuit_results.json` — RTX 3060 / Isaac Sim 6.2.0 での T1〜T9 実測生データ

## 主な特徴

- **CPU / GPU ノード分割と PCIe 境界**:
  - `Fabric (CPU Mirror)`（`cpuValid`）と `Fabric (GPU Mirror)`（`gpuValid`）を分離し、Dirty-bit PCIe ミラー経路を明示
  - `Physics Engine [CPU]` と `Physics Engine [GPU/CUDA]` を分離し、`/physics/suppressReadback` による PCIe Readback 遮断と `GPU Interop` / `Newton Warp` / `CUDA Tensor Zero-Copy` の VRAM 完結経路を対比
- **目で追える連続信号アニメーション（速度スライダー・再生/一時停止付）**:
  - `① CLOCK 毎フレーム`（● 丸粒子が等間隔で連続移動）
  - `② DIRTY 差分同期`（■ 四角バッチがラッチ蓄積→フラッシュ移動）
  - `③ EVENT 単発`（◆ ダイヤ粒子がイベント発火時のみ 1 回移動）
  - 既定速度 `0.6x`（`0.2x`〜`2.0x` で調整可能）
- **ドキュメント × 実機実測（T1〜T9）の両輪根拠**:
  - 各 IC ブロック／配線クリック時のプローブ詳細と本文レポートに、一次出典パスと T1〜T9 の実測数値を記載
