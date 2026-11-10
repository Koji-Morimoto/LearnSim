---
type: moc
created: 2026-11-08
updated: 2026-11-09
tags: [isaac-sim-6.2.0, moc, open-questions]
---

# MOC-未確認事項

ノートに【未確認】【要観測】【未調査】と書いていた事項と、その確定までの記録である。2026-11-09 に、すべての事項をソースか実測で確定させ、元のノートの記述も直した。新しく分からないことが出てきたら、この表に行を足し、確定したら「状態」と元のノートを直す。

下の2つの表のうち、上は 2026-11-08 から載っていた事項、下は 2026-11-09 にノートの見直しで見つけて確定させた事項である。

| 事項                                                                                         | 出てくるノート                                                                                   | 確かめる方法                                                                                                           | 状態                                                                                                                                                                               |
| ------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| 1回の更新で実行する物理ステップ数が、「余りに進みを足し、刻み幅で割って切り捨て、割り切れない分を持ち越す」手順に従うか                               | [時間と物理ステップ（「timeline 時刻とシミュレーション時刻のずれ」の節）](../03_時間と物理ステップ/01_時間と物理ステップ.md)               | [clock_probe](../05_観測Snippet/03_clock_probe.md) を、S と C を変えて実行する                                          | 確定（2026-11-09。ソース `PhysXScene::computeSubstepping` と実測で一致。ただし minFrameRate による上限がある）                                                                                             |
| timeline が開始時刻から進んだ量とシミュレーション時刻の差が、0 以上、刻み幅未満に収まるか                                         | 同上                                                                                        | clock_probe の `in_range` 列を見る                                                                                 | 訂正（2026-11-09 実測。ウォームアップの2回のため、正しくは −2h 以上 −h 未満）                                                                                                                                |
| Play 直後のウォームアップの2回が、物理ステップ数に数えられるか                                                         | 同上                                                                                        | clock_probe と step_probe で、Play 直後の値を見る                                                                       | 確定（2026-11-09 実測。シミュレーション時刻にも累計回数にも数えられる）                                                                                                                                        |
| `minFrameRate` で打ち切った物理ステップを、次の更新に持ち越すか捨てるか                                                | 同上                                                                                        | 重いシーンで clock_probe を実行する                                                                                      | 確定（2026-11-09。上限×刻み幅までは持ち越し、超えた分は捨てる。ソースと実測）                                                                                                                                     |
| `isaacsim.core.simulation_manager.SimulationManager.step(n)` で timeline が進まないか、描画されないか     | [時間と物理ステップ（「手動ステップ」の節）](../03_時間と物理ステップ/01_時間と物理ステップ.md)                                  | Pause 中に `isaacsim.core.simulation_manager.SimulationManager.step(steps=3)` を実行し、clock_probe と step_probe で見る | 確定（2026-11-09 実測。timeline は進まず描画もされない。ずれは Stop まで残り、余りは 0 に戻る）                                                                                                                   |
| simulate の2つ目の引数を物理エンジンが何に使うか                                                              | [時間と物理ステップ（「物理ステップ1回の中で起きること（simulate）」の節）](../03_時間と物理ステップ/01_時間と物理ステップ.md)              | 公式ドキュメントか omni.physics のソースで調べる                                                                                  | 確定（2026-11-09。USD の time sample を読む時刻に使う。ソース）                                                                                                                                    |
| simulate の内部で結果の取り出しまで行うか                                                                  | 同上                                                                                        | 同上                                                                                                               | 確定（2026-11-09。行う。バインディングの説明と実測）                                                                                                                                                  |
| Stop で timeline 時刻が開始時刻に戻るか                                                                | [時間と物理ステップ（「Play・Pause・Stop で時計がどう動くか」の節）](../03_時間と物理ステップ/01_時間と物理ステップ.md)               | clock_probe を Stop の前後で見る                                                                                     | 確定（2026-11-09 実測。開始時刻に戻る）                                                                                                                                                        |
| omni.physx.fabric が実行時に有効になっているか                                                           | [設定一覧](../04_設定/01_設定一覧.md)                                                               | [env_fingerprint](../05_観測Snippet/01_env_fingerprint.md)                                                         | 確定（2026-11-09。GUI の既定では無効。env_fingerprint の結果）                                                                                                                                   |
| `/physics/updateToUsd` など、未記載の設定の既定値                                                       | 同上、[設定一覧](../04_設定/01_設定一覧.md)                                                            | env_fingerprint                                                                                                  | 確定（2026-11-09。updateToUsd・Velocities・Particles は True、ForceSensors は None、playSimulations は True、cudaDevice は -1、suppressReadback は False、minFrameRate は 30。env_fingerprint の結果） |
| Play 中に、`isaacsim.core.experimental.prims.XformPrim` の usd / usdrt / fabric のどれが物理の最新値を返すか | [バックエンド選択の仕組み（「Play 中に XformPrim が読む値と、物理の出力先」の節）](../02_バックエンド選択の仕組み/01_バックエンド選択の仕組み.md) | [pose_freshness_probe](../05_観測Snippet/05_pose_freshness_probe.md)                                               | 確定（2026-11-09 実測。物理の出力先で決まる。USD へ書く既定では3つとも最新、Fabric へ書くと usdrt / fabric だけ最新、どちらにも書かないと3つとも古い）                                                                                  |
| `carb.log_warn` が標準出力にも出るか                                                                 | [バックエンド選択の仕組み（「フォールバックの通知のされ方」の節）](../02_バックエンド選択の仕組み/01_バックエンド選択の仕組み.md)                 | strict を外して `isaacsim.core.experimental.utils.backend.use_backend()` の警告を出し、出力先を確かめる                             | 確定（2026-11-09。Python の sys.stdout も Script Editor のログ欄も通らない。Kit のログ（Console・ログファイル・端末）に出る。ソースと実測）                                                                                |
| fabric 経路は、N が小さいとき usd より遅いか                                                              | [複数プリム対応とPythonループの有無](../01_用語と概念/05_複数プリム対応とPythonループの有無.md)                            | [bench_probe](../05_観測Snippet/04_bench_probe.md)                                                                 | **確定（2026-11-09 実測。N = 1 で fabric 208 µs、usd 100 µs と遅い。N = 10 で逆転する）**                                                                                                          |
| view の getter が返す配列を書き換えても、シミュレーションが変わらないか                                                 | [viewとは何か](../01_用語と概念/03_viewとは何か.md)                                                    | tensor の段階で観測する                                                                                                  | 確定（2026-11-09 実測。変わらない。呼ぶたびに別の配列が返る）                                                                                                                                             |
| simstate の中身                                                                               | [バックエンド選択の仕組み（「候補リストとメソッドの対応表」の節）](../02_バックエンド選択の仕組み/01_バックエンド選択の仕組み.md)                 | ソースを読む                                                                                                           | 確定（2026-11-09。書き込み先の isaacsim.mega.sim_state.client は 6.2.0 のインストールにない。mirror はエラーのログだけ、exclusive は目標値がどこにも書かれない。ソースと実測）                                                         |
| 1つのプロセスで複数のステージを同時に物理シミュレーションできるか                                                          | [viewとは何か](../01_用語と概念/03_viewとは何か.md)                                                    | 調べる                                                                                                              | 確定（2026-11-09。できない。attach_stage は前のステージを切り離す。ソースと実測）                                                                                                                             |
| 描画フレームのイベント NEW_FRAME が、更新1回ごとに1回出るか                                                       | [clock_probe](../05_観測Snippet/03_clock_probe.md)                                    | clock_probe の `d_tau` 列を見る                                                                                    | 確定（2026-11-09 実測。平均1回。1回の更新だけ見ると 0 回や 2 回のこともある）                                                                                                                                 |

## 2026-11-09 に見つけて確定させた事項

| 事項 | 出てくるノート | 確かめた方法 | 状態 |
|---|---|---|---|
| Stop 中に予約した step_probe の処理が、なぜ1ステップ遅れて実行されるか | [step_probe](../05_観測Snippet/02_step_probe.md) | ソース（`register_callback` の `_simulation_view_created` の条件）と実測 | 確定（ウォームアップの2回は累計に数えられるが、PRE / POST のコールバックは呼ばれない） |
| Play 中だけ上限周波数が 60、manual mode が True になる理由 | [env_fingerprint](../05_観測Snippet/01_env_fingerprint.md) | ヘッドレスで、目標フレームレートと上限周波数の初期値を変えて実測 | 確定（上限周波数は timeline が目標フレームレートに書き換え、Pause・Stop で戻す。manual mode は isaacsim.core.throttling が切り替える） |
| Stop 中・Pause 中に描画フレームのイベントが2回ずつまとまって届く理由 | [clock_probe](../05_観測Snippet/03_clock_probe.md) | ソース（isaacsim.core.throttling）と実測 | 確定（Stop・Pause の10回後から asyncRendering が true になる） |
| `gpu dynamics: True` なのに `physics device: 'cpu'` になる理由 | [env_fingerprint](../05_観測Snippet/01_env_fingerprint.md) | ソース（`SimulationManager.get_device`） | 確定（device が cuda になるのは /physics/suppressReadback が true のときだけ） |
| env_fingerprint が Physics Scene のないステージに `/PhysicsScene` を作ってしまうか | [env_fingerprint](../05_観測Snippet/01_env_fingerprint.md) | ソースと実測 | 確定（作っていた。get_physics_dt と is_gpu_dynamics_enabled が作る。コードを直した） |
| Stop 中に `Articulation.get_dof_positions` が usd にフォールバックするか | [backend_spy](../05_観測Snippet/06_backend_spy.md) | ソースと実測 | 確定（しない。tensor 専用のメソッドで、AssertionError になる） |
| Script Editor の print が重い理由 | [bench_probe](../05_観測Snippet/04_bench_probe.md) | ソース（`log_view.py`）と実測 | 確定（ログ欄が1行ごとに表示を作り直す。ヘッドレスでは 13.3 µs/行） |
| Isaac Sim のログ出力が Play 中の更新を遅くするか | [重い処理と軽くする方法](../05_観測Snippet/08_重い処理と軽くする方法.md) | 実測 | 確定（Play 中はログがほぼ出ず、ログレベルを変えても速さは変わらない） |
| コードから Play / Pause / Stop したとき、いつ反映されるか | [MOC-観測Snippet](../05_観測Snippet/00_MOC-観測Snippet.md) | 実測 | 確定（次の更新で反映される。commit() でその場で反映される） |
| GPU の配列を測るとき sync が必要か | [bench_probe](../05_観測Snippet/04_bench_probe.md) | 実測 | 確定（RigidPrim.get_world_poses では差が小さい（240 / 255 µs）が、API によるので sync=True にする） |
| `/app/useFabricSceneDelegate` を起動後に変えると use_backend の判定が変わるか | [設定一覧](../04_設定/01_設定一覧.md) | ソースと実測 | 確定（変わらない。モジュールの読み込み時に1回だけ読む） |
| use_backend の with の中で登録したコールバックに指定が効くか | [コンテキストマネージャとuse_backend](../01_用語と概念/04_コンテキストマネージャとuse_backend.md) | 実測 | 確定（後の更新で呼ばれると効かない。with の中で step() を呼んだ場合は効く） |
| 旧 Core API の置き換え先の根拠 | [旧CoreAPIと新APIの対応](../01_用語と概念/06_旧CoreAPIと新APIの対応.md) | 拡張の非推奨表記 | 確定（isaacsim.core.utils → isaacsim.core.experimental.utils。isaacsim.core.api は isaacsim.core.experimental.* とだけ書かれている） |

## 他のカテゴリ

- [README（ルート目次）](../README.md)

## このノートに出てくる API の import

このノートでは、関数とクラスを `モジュール名.クラス名.メソッド名` の完全修飾名で書いている。Script Editor で使うときは、次の import を先に実行する。import した後は、完全修飾名からモジュール名の部分を除いた名前で呼べる。たとえば `isaacsim.core.simulation_manager.SimulationManager.get_num_physics_steps()` は `SimulationManager.get_num_physics_steps()`、`isaacsim.core.experimental.utils.backend.use_backend()` は `backend_utils.use_backend()` と書く。

Script Editor に貼って実行するコード（ファイルではない）
```python
# このノートに出てくる API の import
from isaacsim.core.simulation_manager import SimulationManager  # シミュレーションの状態・物理ステップ・コールバックを扱うクラス
from isaacsim.core.experimental.prims import XformPrim  # Xform のラッパー
import isaacsim.core.experimental.utils.backend as backend_utils  # use_backend / get_current_backend などを持つモジュール
import carb  # Kit のログ・設定の基盤
```

## 前後のノート

| 前 | このフォルダの目次 | 次 |
|---|---|---|
| [学習方針](../06_学習方針/01_学習方針.md) | [MOC-未確認事項](00_MOC-未確認事項.md) | （なし。ここで最後） |
