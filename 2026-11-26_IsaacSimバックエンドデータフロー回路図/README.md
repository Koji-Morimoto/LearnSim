# Isaac Sim 6.2.0 バックエンドデータフロー回路図

Isaac Sim 6.2.0 のバックエンド（USD Stage / Fabric / PhysX 5 / Newton / Hydra Scene Delegate / RTX Renderer / Viewport / OmniGraph / Tensor API / Sensors / ROS 2 Bridge）のデータ経路を、電子回路の通電状態として可視化する単一 HTML ツールです。

## ファイル

- `isaacsim_backend_circuit.html` — ブラウザでそのまま開けるスタンドアロン HTML（外部依存なし）

## 主な機能

- **スイッチ盤によるリアルタイム通電切替**:
  - Timeline 状態（`STOP` / `PLAY` / `PAUSE`）、物理エンジン（`PhysX 5` / `Newton`）、Render Gate（`SimulationContext.step(render=...)`）
  - `/physics/updateToUsd`、`/physics/fabricEnabled`、`physics/fabricUseGPUInterop`、`/physics/suppressReadback`、`/physics/resetOnStop`
  - `/app/useFabricSceneDelegate`（FSD vs OmniHydra）、`enable_fabric_usd_notice_handler`、`enable_usd_notice_handler`
  - `ext."omni.kit.property.usd".enableAdapter`、`/persistent/omnigraph/updateToUsd`、`/persistent/omni/replicator/captureOnPlay`
- **駆動方式フィルタ（CLOCK / DIRTY / EVENT）**:
  - 毎フレーム実行（`CLOCK`）、イベント記録＋差分バッチ同期（`DIRTY`）、変化時・手動時のみ単発発火（`EVENT`）を切り分けて確認可能
  - `/physics/updateToUsd=true` による `USD Stage → Fabric` の毎フレーム連鎖発火も自動検出
- **1フレーム信号トレース & ワンショットパルス**:
  - `⚡ 1フレーム信号トレース`、`🔄 SynchronizeToFabric()`、`💾 save_to_usd()`
- **実運用プリセット & 回路トラブル再現**:
  - `isaacsim.exp.full.kit` / `isaacsim.exp.full.fabric.kit` / `RL Headless` / `isaacsim.exp.full.newton.kit` / `STOP` / `PAUSE`
  - Property Window 不更新、Readback 遮断インターロック、Viewport 描画断線の 3 事故パターン
