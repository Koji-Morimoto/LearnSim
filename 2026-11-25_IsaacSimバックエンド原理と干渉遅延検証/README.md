# Isaac Sim バックエンド原理と干渉遅延検証

Isaac Sim 6.2.0 における **USD Stage / Fabric（Bucket・SoA・RAM/VRAM 2 面キャッシュ）/ IFabricHierarchy / Hydra / DebugDraw / PhysX（overlap_shape・ContactReport・CPU BVH）/ Newton（eval_fk・CollisionPipeline）/ Warp GPU 非同期同期** の計算原理を、役割別の 2 枚の HTML ドキュメントと 1 コマ確認スクリプトにまとめたディレクトリです。

## 収録ファイル

| ファイル | 役割（類型） | 内容 |
|---|---|---|
| [`explanation_backend_architecture.html`](explanation_backend_architecture.html) | **解説（Explanation）** | 5 層の前提マップ・Fabric 内部構造（Bucket/SoA・2 面キャッシュ）・`app.update()` 4 フェーズと Play=ON/OFF の SVG シーケンス図・1F / 2 update / 0F 遅延と 2 つの視点・3 種剛体（ピンク/黄色/緑）＋USD 属性＋6 組み合わせ表・BVH とバックエンド別対応形状（convexHull/convexDecomposition/trimesh/sdf）・公式出典一覧 |
| [`handson_verify_lag.html`](handson_verify_lag.html) | **ハンズオン（Hands-on）** | 実験 1〜4 の起動コマンド・1 コマ確認チェックポイント・Isaac Sim 6.2.0（RTX 3060）での実測出力 |
| [`verify_backend_lag_step_by_step.py`](verify_backend_lag_step_by_step.py) | **検証スクリプト** | GUI＋Enter キー 1 コマ送り（既定）および `--headless --auto` / `--exp 1..4` 両対応の実機確認スクリプト |
| [`run_verify.bat`](run_verify.bat) | **起動ラッパー** | `C:\isaacsim-6.2.0\tools\myext\run_sim.bat` 経由で UTF-8＋必須拡張つきで起動するバッチファイル |

## クイックスタート

```powershell
# GUI を開き、Enter キーで 1 コマずつ止めながら Viewport と数値を見比べる
.\run_verify.bat

# 特定の実験だけ実行する（1〜4）
.\run_verify.bat --exp 2

# 画面なし・自動進行で全 4 実験を一括検証する
.\run_verify.bat --headless --auto
```
