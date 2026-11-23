# Isaac Sim 6.2.0 大規模点群（〜5,000万点）統一描画API ＆ CPU⇔GPUデータ転送・DrawCall解説

Isaac Sim 6.2.0 上で点群を **3Dの球（Sphere）** として描画する全6方式を、1つの統一関数 `create_pointcloud()` にまとめた実装と、各方式における **CPU⇔GPU間のデータ転送・DrawCall（描画命令）** を可視化した解説動画一式です。

## 解説動画

- [media/pointcloud_cpu_gpu_explained.mp4](media/pointcloud_cpu_gpu_explained.mp4) （1920×1080 / 30fps / 2分12秒 / VOICEVOX ナレーション付き）

## 含まれるファイル

| ファイル | 内容 |
| :--- | :--- |
| `pointcloud_api.py` | 全6方式の統一関数 `create_pointcloud()` / `UnifiedPointCloud`（半径・色・位置・テカリ・影のリアルタイム調整対応） |
| `demo_interactive_gui.py` | `UnifiedPointCloud` の各種パラメータをボタンでリアルタイムに切り替えられる Isaac Sim GUI デモ |
| `octree_50m_full_isaacsim.py` | フル版 Isaac Sim（`isaac-sim.bat`）で 5,000万点 Octree LOD と全点Visibleモードを切り替えて検証するスクリプト |
| `capture_pointcloud_video.py` | 動画用の Isaac Sim 3Dシーン・テレメトリ撮影スクリプト |
| `script_and_voice.py` | VOICEVOX（四国めたん）によるナレーションWAV生成・読み（`kana`）検証スクリプト |
| `compose_video.py` | 実機映像・CPU⇔GPU通信HUD・実行コードハイライト・字幕・音声を合成する動画生成スクリプト |
| `media/` | 解説動画 MP4 および 5,000万点 Octree LOD / Warp CUDA `atomicMin` の検証画像 |

## 統一関数インターフェース（`pointcloud_api.py`）の使い方

```python
from pointcloud_api import create_pointcloud

# 描画方式 (method) を変えるだけで6パターンを統一的に切り替え可能:
# "octree_lod" | "usd_points" | "point_instancer" | "hybrid_compute_rtx" | "warp_compute_atomic64" | "naive_spheres"
pc = create_pointcloud(
    stage,
    positions=pts_np,               # (N, 3) float32 座標配列
    colors=rgb_np,                  # (N, 3) float32 色配列 または単色 (R, G, B)
    method="octree_lod",
    point_radius=0.045,             # 球の半径 [m]
    world_offset=(0.0, 0.0, 0.0),   # 点群全体のオフセット座標 [m]
    glossy=True,                    # True=ツヤあり (roughness=0.08), False=マット (roughness=0.92)
    cast_shadows=True,              # True=影あり, False=影なし (doNotCastShadows=True)
)

# 実行中のリアルタイム見た目調整（座標配列の再転送なし）
pc.set_point_radius(0.085)
pc.set_colors((0.18, 0.85, 0.55))
pc.set_world_offset((0.0, 0.0, 0.5))
pc.set_appearance(glossy=False)
pc.set_cast_shadows(False)

# カメラ移動時の更新（octree_lod / hybrid_compute_rtx / warp_compute_atomic64 用）
pc.update_camera(cam_eye=(5.0, -5.0, 2.0), cam_target=(0.0, 0.0, 0.0))
```

## 6方式の CPU⇔GPU 転送データ・DrawCall・実測FPS 比較

実測環境: NVIDIA GeForce RTX 3060 (12 GB VRAM) / Intel Core i7-13700K / Isaac Sim 6.2.0（$10\text{ m} \times 10\text{ m} \times 5\text{ m}$ ランダム3D点群・4方向カメラ移動）

| `method` 引数 | 初回 (`Frame 0`: CPU → GPU) | カメラ操作時 (`Frame 1+`: 毎フレーム転送) | 毎フレームの命令 / `DrawCall` | 実測 FPS (100万点 / 5,000万点) |
| :--- | :--- | :--- | :--- | :--- |
| `'naive_spheres'` | $N$ 個の球Primを定義・転送 | 座標転送 **0 B** | **[渋滞]** $N$ 回の `DrawCall` | 5,000点で **14.4 FPS** (上限) |
| `'usd_points'` | 座標・色・直径を一括転送 (100万点分割) | **64 B** カメラ行列のみ (座標 0 B) | 1 Chunk = 1 `DrawCall` | 1M: **38.4 FPS** / 50M: **6.1 FPS** |
| `'point_instancer'` | 球1個 + $N$ 個の配置配列を転送 | **64 B** カメラ行列のみ (座標 0 B) | 1 Instanced `DrawCall` (RTコア加速) | 1M: **55.8 FPS** / >10M VRAM上限 |
| `'hybrid_compute_rtx'` | 全座標を CUDA VRAM へ転送 | **[罠]** 毎フレーム **11.8 MB** (GPU→CPU→GPU) | 毎フレーム `VtArray` 再転送 + BVH更新 | 1M: **32.0 FPS** / 50M: **11.1 FPS** |
| `'octree_lod'` | 全32ブロック LOD0/1/2 を VRAM 常駐 | **64 B** カメラ行列 + `visibility` 1bitフラグ | 座標転送ゼロ・視野外&遠方を自動間引き | 1M: **61.9 FPS** / 50M: **35.3 FPS** |
| `'warp_compute_atomic64'` | 全5,000万点を CUDA VRAM 常駐 | **64 B** カメラ行列のみ (座標 0 B) | USD `DrawCall` ゼロ (1 CUDA Kernel起動) | 1M: **1,711 FPS** / 50M: **118.8 FPS** |
