# 実測ベンチマーク・Red → Green 検証知見ログ

このファイルは、`myext.robot.geometric_newton` の各設計・最適化・不具合修正について、**公式ドキュメントからの推測ではなく Isaac Sim 6.2.0 実機で Red（問題・劣化の再現）→ Green（解消・改善の確認）を計測した知見**を記録するものです。

- 検証環境: Windows 11 Pro / Intel Core i7-13700K / NVIDIA GeForce RTX 3060 (12 GiB) / Isaac Sim 6.2.0 / Newton 1.6.1 / Warp 1.17.0
- 比較スクリプト配置先: `tests_red_green/`（各組の `*_Red.py` と `*_Green.py` は切替フラグ 1 行のみが異なるため、WinMerge で差分を直接比較できます）
- 実行コマンド: `C:\isaacsim-6.2.0\tools\myext\run_sim.bat extsUser\Extentions\myext.robot.geometric_newton\tests_red_green\<スクリプト名> --headless`

---

## 01. USD 書込直後の Fabric 1 フレーム戻り（ちらつき）検証

- **比較ペア**: `01_fabric_repush_flicker_Red.py` vs `01_fabric_repush_flicker_Green.py`
- **差分箇所**: `ENABLE_REPUSH_GUARD = False`（Red）vs `True`（Green）
- **対象ロボット**: UR10e（＋`Gripper` VariantSet `Robotiq_2F_85`）

| ケース | Red（ガード無効） | 修正前（`_repush_frames` のみ） | Green（`_usd_sync_pending` ＋ `_repush_frames`） |
|---|---|---|---|
| **Case A**: `flush_to_usd()` → 直後に Fabric 移動 | frame +1〜+4: **`1.323236 m`**（旧姿勢へ戻って固着） | frame +1 のみ **`1.323236 m`**、frame +2〜+4: `0.000000 m` | frame +0〜+4: **全フレーム `0.000000 m`** |
| **Case B**: `set_variant()` → 直後に Fabric 移動 | frame +1〜+4: **`1.323236 m`**（旧姿勢へ戻って固着） | frame +1 のみ **`1.323236 m`**、frame +2〜+4: `0.000000 m` | frame +0〜+4: **全フレーム `0.000000 m`** |
| **Case C**: `layer.apply()` → 直後に Fabric 移動 | frame +1〜+4: **`1.477654 m`**（旧姿勢へ戻って固着） | frame +1 のみ **`1.477654 m`**、frame +2〜+4: `0.000000 m` | frame +0〜+4: **全フレーム `0.000000 m`** |

### 得られた知見
1. USD Sublayer へ姿勢を書いた直後に `write_target="fabric"` で新姿勢を書き込むと、次の `app.update()` で走る USD→Fabric 同期が Fabric 側の新姿勢を旧 USD 値で上書きしてしまい、**ガードなしでは旧姿勢へ戻ったまま永続的に固着します**。
2. `_repush_frames` による `_on_update` 再送だけでは、Kit の `update_event_stream`（`order=0`）が USD→Fabric 同期より先に実行されるため、**frame +1 の 1 フレームだけ `1.32〜1.48 m` のちらつきが残っていた**ことが実測で判明しました。
3. `_usd_sync_pending` フラグを導入し、「USD 側がすでに dirty で次フレームに同期が予定されている同フレーム内」に限り `_push()` で最新姿勢を USD Sublayer へも追随させることで、**定常フレームの USD 書込 0 回を維持したまま frame +0〜+4 のすべてで誤差 `0.000000 m`** を達成しました。

---

## 02. Newton Broad Phase への `collision_filter` 事前登録（焼き込み）

- **比較ペア**: `02_newton_broadphase_filter_Red.py` vs `02_newton_broadphase_filter_Green.py`
- **差分箇所**: `BAKE_FILTER_IN_BROADPHASE = False`（Red）vs `True`（Green・既定）
- **対象ロボット**: UR10e ＋ `Robotiq_2F_85`（17 リンク、`ignore_touching_pairs()` 適用済み、200 反復平均）

| 指標 | Red（Broad Phase 未登録） | Green（Broad Phase 登録済み・既定） | 差分・効果 |
|---|---:|---:|---|
| Newton 登録済み除外ペア数 | `0` | `21` | `robot.collision_filter` の 17 組を全コライダー組へ展開 |
| Broad Phase 候補ペア数（`shape_pairs_filtered`） | `151` | `130` | 除外ペアが事前リストから完全削除 |
| Narrow Phase 生接触数（`rigid_contact_count`） | **`42`**（無駄計算） | **`0`** | Narrow Phase の無駄な接触計算が完全にゼロ |
| ユーザーへの報告ヒット数 | `0` | `0` | 結果一致 |
| `set_dof_positions` ＋ `check_collisions` 平均時間 | `2.709 ms/iter` | **`2.188 ms/iter`** | `-0.521 ms/iter`（約 20% 高速化） |

### 得られた知見
1. 単体ロボット 1 台（17 形状・単純凸包）では、固定費（Fabric 書込約 `0.38 ms` ＋ GPU 起動および CPU-GPU 同期約 `1.8 ms` = 約 `2.19 ms`）が支配的なため全体では約 20% の短縮ですが、**Narrow Phase の無駄計算分（約 `0.52 ms`）は 100% 削減**されています。
2. 凹メッシュ（`convex_meshes=False`）や多軸複合ロボット、あるいは `BatchedNewtonScene` で 64〜1024 台を並列評価する場面では固定費が薄まり、未登録だと `42 接触 × 1024 台 = 43,008 件/step` の無駄接触が Narrow Phase と接触バッファを圧迫するため、Broad Phase 段階での除外が必須です。

---

## 03. Session Sublayer の `collision_mode="none"` vs `"static"`

- **比較ペア**: `03_collision_mode_none_vs_static_Red.py` vs `03_collision_mode_none_vs_static_Green.py`
- **差分箇所**: `COLLISION_MODE = "static"`（Red）vs `"none"`（Green・既定）
- **対象ロボット**: UR10e（instanceable な `collisions` prim を含む）＋ 環境障害物 `/World/Obs`

| 指標 | Red（`collision_mode="static"`） | Green（`collision_mode="none"`・既定） |
|---|---:|---:|
| PhysX 空間の有効ロボットコライダー数（`overlap_sphere` 照会） | **`7`**（PhysX に残存） | **`0`**（PhysX コライダー生成ゼロ） |
| Newton が認識するロボットコライダー数 | `7` | `7` |
| Newton 環境干渉検知数 | `3` | `3` |

### 得られた知見
1. UR10e のように `collisions` prim が `instanceable=True` のアセットでは、そのままでは `physics:collisionEnabled` を上書きできません。`GeometricLayer` が Session Sublayer 上のみ一時的に `instanceable=False` へ展開して `physics:collisionEnabled=False` を書くことで、PhysX のロボットコライダーを確実に 0 個にできます。
2. Newton 側は `is_collision_enabled_ignoring_geometric_layer()` で `LAYER_MARKER` 付き Sublayer の上書きだけを無視するため、元のコライダー構成（ユーザーが Root Layer 等で無効化したものは除外）をそのまま読み取れます。
3. `layer.remove()`（または Stop 中に GUI の Layers パネルで無名 Sublayer をミュート）により、instanceable 状態を含め 100% 元の Articulation 構成へ復元されます。

---

## 04. 毎フレーム書込先 `write_target="usd"` vs `"fabric"`

- **比較ペア**: `04_write_target_usd_vs_fabric_Red.py` vs `04_write_target_usd_vs_fabric_Green.py`
- **差分箇所**: `WRITE_TARGET = "usd"`（Red）vs `"fabric"`（Green・既定）
- **対象ロボット**: UR10e ＋ `Robotiq_2F_85`（120 フレーム平均）

| 指標 | Red（`write_target="usd"`） | Green（`write_target="fabric"`・既定） | 差分・効果 |
|---|---:|---:|---|
| USD 書込回数 / 120 フレーム | `120 回` | **`0 回`** | 定常フレームで USD 非書込 |
| `set_dof_positions()` 平均時間 | `0.847 ms/frame` | **`0.380 ms/frame`** | **2.23 倍高速** |
| `app.update()` 平均時間 | `4.725 ms/frame` | `4.686 ms/frame` | — |
| 合計（`set + update`） | `5.573 ms/frame` | **`5.066 ms/frame`** | `-0.507 ms/frame` |

---

## 05. 54 DOF 4 アームロボットでの一括配列 vs `with robot.batch_update():` 部位別セット

- **比較ペア**: `05_batch_update_components_Red.py` vs `05_batch_update_components_Green.py`
- **差分箇所**: `USE_BATCH_UPDATE = False`（Red）vs `True`（Green）
- **対象ロボット**: `multi_arm`（54 DOF・59 リンク・台車＋並列 4 アーム＋各アーム先端 VariantSet、300 フレーム平均）

| 書き込み方式 | FK ＋ Fabric 書込回数 / frame | 平均時間 / frame | 一括配列との差 |
|---|---:|---:|---|
| 一括フラット配列（54 DOF を 1 回渡し） | `1 回` | **`0.527〜0.560 ms`** | 基準（`1.00x`） |
| **Red**: `batch_update()` なしで 5 Component 個別セット | **`5 回`** | **`2.700 ms`** | **`+2.173 ms`（`5.13x` 遅い）** |
| **Green**: `with robot.batch_update():` ＋ 5 Component セット | **`1 回`** | **`0.627 ms`** | **`+0.067 ms`（`1.12x`）** |
| **Green**: `with robot.batch_update():` ＋ 5 `GeometricDofBinder` | **`1 回`** | **`0.615 ms`** | **`+0.055 ms`（`1.10x`）** |
| 参考: 54 キー辞書を 1 回渡し（`set_dof_positions(dict)`） | `1 回` | `0.548〜0.553 ms` | `+0.01〜0.03 ms` |

### 得られた知見
1. 並列複数アームではロボット全体の `dof_names` が幅優先（階層順）で入り組むため、長い 1 本の配列を組み立てる負担が大きくなります。
2. `robot.get_component(...)` で部位ごとに取り出して **`with robot.batch_update():`** の中でセットすれば、一括配列との速度差はわずか **`+0.055〜0.067 ms/frame`**（60 fps 予算 16.6 ms の 0.4% 未満）に収まり、書きやすさと速度を両立できます。
3. VariantSet 切替でアームごとに軸数が変わる場合も、ダミー軸（`lower==upper`）でモデルや IK を汚さず、部位ごとの `robot.bind_dofs(..., ignore_unknown=True)` を `with robot.batch_update():` 内で呼ぶことで **`0.615 ms/frame`** のまま安全に駆動できます。
4. 逆に `with robot.batch_update():` を付け忘れると FK と Fabric 書込が部位数ぶん（5 回）重複実行され、**5.13 倍遅く**なります。

---

## 06. 凹メッシュ精度実測（`convex_meshes=True` vs `False`）

- **対象形状**: U 字型水密凹メッシュ対（28 三角形、溝幅 0.4 m、溝内への小球配置および 5 cm 貫通、CUDA）
- `convex_meshes=True`（単一凸包・既定）: 溝内の非接触位置で誤検知 `1 件`、5 cm 貫通時の符号付き距離 `-0.45 m`、処理時間 `2.35 ms`
- `convex_meshes=False`（元の三角形メッシュ）: 溝内の非接触位置で `0 件`、5 cm 貫通時の符号付き距離 **`-0.0500 m`**（誤差 4 nm）、処理時間 `2.39 ms`
