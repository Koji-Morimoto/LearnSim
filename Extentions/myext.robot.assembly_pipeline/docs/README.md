# myext.robot.assembly_pipeline

2026-11-19 / Isaac Sim 6.2.0(URDF Importer 3.11.12、Robot Assembler 3.3.9)で作成・確認

URDF から usda を作り、usda どうしを結合した**結合ファイル**を作る拡張機能です。何度実行しても同じ結果になります(冪等)。結合ファイルは次の結合の親にも子にもできるので、中間の状態がそのまま 1 つの usda として残ります。

あわせて、URDF と usda を先に調べて既知の罠に当たるかを判定します(**事前チェック**)。判定が出たら止めるか進めるかを選べます。進めた場合も、最後にその症状が**実際に起きたか**を測ります(**事後確認**)。測るのは「公式のまま」と「修正後」の両方です。公式のままで症状が出なくなれば、報告に「公式のままでも起きなかった(公式側の修正の可能性)」と出ます。

## 言葉

この拡張の中では、次の言葉をこの意味でだけ使います。

| 言葉 | 意味 |
|---|---|
| usda | default prim を持つ USD ファイル。URDF から作ったもの、または結合ファイル |
| 結合 | 子の usda を、親の usda のリンクに固定する 1 回の操作。中身は下の「結合の中身」の 7 つの書き込み |
| 結合ファイル | 親の usda に 1 回以上の結合を書いた、新しい usda。次の結合の親にも子にもできる |
| 公式のまま | URDF Importer の出力を Robot Assembler でそのまま結合した場合。比較用に、パイプラインの中で実際に結合して測る |
| 修正後 | このパイプラインの出力 |
| 修正 | このパイプラインが入れる直し(規則ごとに `fix = "skip"` で外せる) |

## すぐ試す

```powershell
cd C:\isaacsim-6.2.0\extsUser\Extentions\myext.robot.assembly_pipeline\examples
C:\isaacsim-6.2.0\python.bat ..\run_pipeline.py multi_arm.toml
```

1〜2 分で `examples\out\multi_arm\` に次のものができます。

| ファイル | 中身 |
|---|---|
| `arm_unit.usda` | 結合ファイル:アーム + エンドエフェクタ(VariantSet `EndEffector`: gripper / rod / None)。単体で使える |
| `robot.usda` | 結合ファイル:ベース + `arm_unit.usda` を 4 本(arm1〜arm4)。アームごとにエンドエフェクタを選んでいる |
| `urdf\<名前>\<URDF名>.usda` | URDF Importer の生成物(手を入れない) |
| `urdf\<名前>\<名前>_fixed.usda` | 正規化層(生成物を参照し、over だけを書く) |
| `report.md` / `report.json` | 規則ごとの判定と、測った症状 |
| `report_previous.json` | 1 つ前の報告(前回からの変化を出すため) |

終了コード: 0 = 問題なし、1 = 修正後も起きた症状か判定外の症状がある、2 = 判定で止めた、3 = 例外。

ほかの例: `arm_gripper.toml`(結合 1 回)、`franka\franka.toml`(Isaac Sim 同梱の Franka の腕に手を z 軸まわり -45° で結合)。

## 仕様(TOML)

```toml
output_dir = "out/multi_arm"           # 相対パスは TOML ファイルから見る

[checks]
on_violation = "continue"              # "stop" = 判定が出たら結合ファイルを作る前に止める / "continue" = 進めて最後に答え合わせする
fix = "apply"                          # "apply" = 修正を入れる / "skip" = 入れない(修正が要らなくなったかを確かめるとき)

[checks.rules.R02]                     # 規則ごとに上書きできる
on_violation = "stop"

[urdf.arm]                             # URDF から作る usda。名前は URDF と結合ファイルで共通の名前の空間
path = "urdf/arm.urdf"
# collision_from_visuals = false       # 省略時: URDF に collision が 1 つでもあれば false、無ければ true
# physics_variant = "auto"             # "auto"(physx があれば physx、無ければ physics) / "physx" / "physics" / "mujoco"
# ros_package_paths = [{ name = "pkg", path = "..." }]
# joint_drive_stiffness = 100.0        # URDF Importer の一括 gain 指定(mimic の follower にも入るので、修正で follower は 0 に戻す)
# joint_drive_damping = 5.0

[join_file.arm_unit]                   # 結合ファイル。出力は <output_dir>/arm_unit.usda
parent = "arm"                         # 親の usda(urdf か join_file の名前)

[[join_file.arm_unit.join]]            # 結合 1 回
name = "gripper"                       # 結合の名前(結合ファイルの中で一意)
child = "gripper"                      # 子の usda(urdf か join_file の名前)
parent_link = "flange"                 # 親のリンク(形も慣性も無い空のリンクでもよい)
child_link = "gripper_base"            # 子のリンク。この 2 つのリンクの URDF の座標系を一致させる(offset を挟む)
prim_name = "end_effector"             # 子の prim の名前(省略時は name)
variant_set = "EndEffector"            # 結合を VariantSet の選択肢にするとき。結合ファイルの一番上の prim に付く
variant = "gripper"                    # 選択肢の名前。同じ variant_set の結合は排他で、最初が既定。"None" は自動で足す
# parent_prim = "link2"                # 子を Reference で置く親側の prim。省略時は parent_link の prim
#                                      # (stage の親子が実際のリンクの親子に揃う)。"." = 結合ファイルの一番上の prim
# offset_xyz = [0, 0, 0]               # parent_link の座標系で見た child_link の位置(m)
# offset_rpy = [0, 0, 0]               # 同じく向き(rad、URDF と同じ固定軸 XYZ)
# select = { EndEffector = "rod" }     # 子の usda の VariantSet の選択

[probe]                                # 症状を測る条件(省略可)
steps = 240
revolute_target = 0.3                  # 関節を動かす目標(rad。可動範囲の 8 割に切り詰める)
angle_tol_deg = 1.0
length_tol_m = 0.001
```

**リンクの書き方**: 親(または子)の usda の一番上の URDF のリンクは名前だけ(`flange`)。結合ファイルの中で結合した子のリンクは「結合の名前/リンク名」(`arm1/flange`、入れ子なら `arm1/gripper/finger_l`)。

Python から組み立てる場合は `PipelineSpec` / `UrdfSource` / `JoinFileSpec` / `JoinSpec` / `RuleSetting` / `ProbeSettings`(`spec.py`)。

## 結合の中身(7 つの書き込み)

| # | USD への書き込み | Robot Assembler との違い(= 修正。規則) |
|---|---|---|
| 1 | 親の usda の指定した prim の子として、子の usda を Reference で置く。その prim が instanceable なら外す | Assembler は置き場所を選ばせない(置いた場所のまま)。instanceable な prim の下には書けず失敗する(R05) |
| 2 | 子の prim の Transform を、親のリンクと子のリンクの URDF の座標系(+ offset)が一致する値にする | Assembler は prim の位置で合わせる。prim が URDF のリンク座標からずれていると、結合面がずれる(R04)。offset を指定する機能も無い |
| 3 | 親と子の剛体をつなぐ固定関節を、決まったパス `<子>/JoinJoint` に 1 本作る。子のリンクに剛体が無ければ、fixed joint でつながる子孫の剛体に固定する | Assembler はやり直すたびに固定関節が増える(R14)。子のリンクに剛体が無いと結合できない(R08) |
| 4 | 子のワールド固定 joint(子の外につながる joint)を、名前でなく構造で探して無効にする | Assembler は body0 か body1 が空の joint しか無効にしない。URDF Importer の root_joint は body0 が埋まっているので残る(R02、剛体の子に置いたときは閉ループ = R09) |
| 5 | 子の Articulation Root を Physics・Newton・Physx の全部外す | Assembler は Physics の方だけ外す。NewtonArticulationRootAPI が Physics の方を内蔵しているので残る(R07) |
| 6 | 親の剛体と子の剛体の組の衝突を除外する | Assembler は何もしない(R11) |
| 7 | 固定関節に IsaacJointAPI を付け、結合ファイルのロボットスキーマの関係の末尾に子を足し、無効にしたワールド固定 joint を子の robotJoints から消す | Assembler は先頭に足すので、RobotPoser の木の根が子になる。RobotPoser は無効にした joint も使う(R12) |

VariantSet を使う結合は、1〜7 を結合ファイルの一番上の prim の VariantSet の選択肢の中に書きます。

## 何をするか

1. **URDF の判定**(import の前)。
2. **import**: 入力(URDF・メッシュ・設定・importer の版)のハッシュが前回と同じなら飛ばします。違えば一時フォルダに import してから `urdf\<名前>\` を置き換えます(importer に任せると、出力先が空でなければ `_1` 付きの別フォルダになる = R13)。
3. **正規化層**: Physics variant を選び(R06)、mimic の follower の drive を stiffness = damping = 0 にします(R03)。
4. **usda の判定**(結合ファイルを作る前)。
5. **結合ファイル**を依存の順に作ります。毎回まるごと作り直し、中身が同じならファイルは書き換えません。各ファイルを 2 回作って、2 回目に変化が無いことも確かめます。
6. **症状の確認**:
   - 公式のまま:結合 1 回ごとに、親と子の一番上の URDF の生成物を Robot Assembler で実際に結合して測ります(リンクの書き方に `/` が無い結合だけ)。
   - 修正後:結合ファイルごとに、VariantSet の既定の選択と、既定でない選択肢 1 つずつで測ります。
   - どちらも Play して関節を drive で動かし、関節の到達、結合のずれ、親との接触、Play 中のログ(閉ループなど)を測ります。Stop 中には RobotPoser の `apply_joint_state` で teleport して、子が付いてくるかと、親の剛体が URDF の順運動学どおりに来たかを測ります。
   - mimic は URDF ごとに、leader を柔らかい drive で動かして測ります。
7. **幾何モード**: 結合ファイルごとに GeometricRobot を作り、同じ関節値で PhysX とリンク位置を比べます。構成要素は IsaacRobotAPI から自動で作り、名前は結合の名前をつないだもの(`arm1`、`arm1/end_effector`)。
8. **報告**: 規則ごとに、判定と、公式のまま・修正後の症状から最終判定を出します。

## Python から使う

```python
from myext.robot.assembly_pipeline import load_spec, run_pipeline, PipelineStopped

try:
    result = run_pipeline(load_spec("robot.toml"))   # headless の Isaac Sim の中で呼ぶ(run_pipeline.py を参照)
    print(result.report["status"], result.outputs["robot"])
except PipelineStopped as exc:
    print(exc.report["stopped_by"])                  # 止めた理由。報告は書いてある
```

結合ファイルを幾何モードにする:

```python
from myext.robot.assembly_pipeline import make_geometric_robot

robot = make_geometric_robot(stage, "/robot")        # GeometricRobot.from_articulation と同じ引数を渡せる
robot.get_component("arm1/end_effector")
```

`run_pipeline` は Kit の更新を自分で回すので、GUI の Isaac Sim の Script Editor からは呼べません(headless 専用)。できた usda は GUI でもそのまま開けます。

## 規則

| ID | 対象 | 判定(静的) | 予想する症状 | 測るもの |
|---|---|---|---|---|
| R01 | URDF | ルートリンクが空(visual・collision 無し、inertial 無しか質量 0) | ワールド固定 joint が root_joint という名前にならない | 生成物のワールド固定 joint の名前 / 子として結合した後に残ったワールド固定 joint |
| R02 | 結合 | 子のワールド固定 joint の body0・body1 がどちらも空でなく、子を剛体の子に置かない | 子がワールドに固定されて、親の関節が動かない | 親の関節の到達誤差、結合のずれ |
| R03 | URDF | mimic follower の drive に stiffness ≠ 0 か damping ≠ 0 | leader が目標に届かない・遅れる | 柔らかい drive で動かした leader の位置(10・30・60 ステップ目と最後)、mimic の関係の誤差 |
| R04 | URDF | 子の形状が 1 つだけの末端リンク | prim の位置が URDF のリンク座標からずれる | prim と URDF の順運動学の差、結合面のずれ |
| R05 | 結合 | 子を置く prim か子のリンクが instanceable | instance proxy への書き込みで結合が失敗する | 結合の成否 |
| R06 | URDF | 出力 USD の Physics variant が未選択 | 剛体も関節も無い | 剛体の数 |
| R07 | 結合 | 子に NewtonArticulationRootAPI | Articulation Root が残る | 結合後の子の Articulation Root |
| R08 | 結合 | 子のリンクの位置にも上にも剛体が無い | 結合できない(Assembler 3.3.8 より前は分離) | 結合の成否、固定関節の子の側の剛体 |
| R09 | 結合 | 子を剛体の子に置き、body0 が空でないワールド固定 joint がある | 閉ループのエラー | Play 中のログ |
| R10 | URDF | visual と collision が同じメッシュで collision_from_visuals = False | 衝突メッシュが見える | purpose が guide でなく visual と同じ位置・大きさの collider |
| R11 | 結合 | (結合のたびに) | 子と親の接触 | Play 中の接触の報告 |
| R12 | 結合 | 子を剛体の子に置かない | RobotPoser の teleport で子が置いて行かれる | teleport 後の結合のずれ、親の剛体の順運動学との差 |
| R13 | URDF | 出力先が空でない | 別名(`_1`)のフォルダに出力される | importer の出力先の決め方、実際の出力先 |
| R14 | 結合 | (結合のたびに) | やり直すと固定関節が増える | Assembler で 2 回結合した後の本数 / 結合ファイルの本数と、作り直しで変化が無いか |

規則の一覧は `kinds.py` の `RuleId` と `rules.py` の `RULES` にあり、食い違うと読み込み時に止まります。

### 最終判定

| verdict | 意味 |
|---|---|
| `ok` | 判定なし・症状なし |
| `fixed` | 判定あり。公式のままでは症状が出た。修正後は出なかった |
| `upstream_fixed` | 判定あり。公式のままでも症状が出なかった(公式側の修正の可能性。R11 のように形の配置で起きないだけのこともあり、規則ごとの説明を出す) |
| `still_broken` | 修正後(または修正を skip した出力)でも症状が出た |
| `unexpected` | 判定なしなのに、公式のままで症状が出た(新しい原因の可能性) |
| `not_measured` | 測れなかった(結合できなかった、公式のままでは再現しない結合) |

報告の文面の例:

```
[R09] robot/arm1: 子 arm_unit を剛体の子に置き、body0 が空でないワールド固定 joint ['root_joint'] が残ると、
その body0 が親の剛体に解決されて閉ループになる と判定した → 公式のままでは、やはり「閉ループのエラー」が起きた
(Play 中の閉ループのログ 1 件)。修正後は起きなかった(Play 中の閉ループのログ 0 件)
[R10] arm: ... と判定した → 公式のままでも、最終的には「衝突メッシュが見える」は起きなかった(...)。
Asset Transformer rules 1.7.7 以降の修正で、衝突側が guide のまま
```

## 確かめたこと(テストで毎回確認している)

`run_tests.ps1`(13 本、約 3 分)。

| 場面 | 確かめること |
|---|---|
| 結合 1 回・親のリンクの子に置く | 状態 ok。R06・R07・R09・R14 が修正で解消、R10 は公式のままでも起きない。GeometricRobot と PhysX のリンク位置の差 1e-4 m 未満 |
| 同じ仕様でもう一度 | import を飛ばす、結合ファイルは 1 回目も 2 回目も変化なし、USD ファイルのハッシュが前と同じ、`_1` のフォルダができない |
| 結合 1 回・一番上の prim の子に置く(`parent_prim = "."`) | R02(公式のままでは親の関節が動かない)と R12(teleport で子が置いて行かれる)が修正で解消 |
| R02・R07・R12 を skip | 3 つとも「修正を skip した出力でも起きた」になり、状態が problems |
| R02 を stop | 結合ファイルを作る前に止まる |
| 空のルートリンクの子(子のリンクも空のリンク) | R08(公式のままでは ValueError)と R01(ワールド固定 joint の名前が mount_joint)が修正で解消 |
| mimic | URDF の `<dynamics damping>` だけなら判定も症状も無し(PhysX の drive に入らない)。importer の一括 damping 指定なら修正で解消 |
| 4 本アーム(結合ファイル 2 段) | 状態 ok。Articulation はベースの 1 つ、ワールド固定 joint は 1 本、DOF 13 個が全部目標に届く。arm2 は rod、arm4 は None。`arm_unit.usda` は単体で開ける。構成要素は robot・arm1〜arm4・各 end_effector |
| 4 本アームでもう一度 | USD ファイルのハッシュが前と同じ |
| Franka(同梱の腕と手) | 状態 ok、offset(-45°)込みで結合面が一致、構成要素は franka と hand |
| 仕様の確認 | 存在しない usda の参照と、結合ファイルの循環を読み込み時に止める |

## 分かっている制約

- headless 専用(Kit の更新を自分で回すため)。
- 公式のままの比較は、親・子とも一番上の URDF のリンクどうしの結合だけ(子が結合ファイルなら、その一番上の URDF の生成物で代わりに測る)。それ以外は `not_measured`。
- 修正後の測定は、VariantSet ごとに既定の選択と、既定でない選択肢を 1 つずつ(全部の組み合わせではない)。
- 既存の USD(URDF から作っていないもの)はまだ入力にできない。
- スケールの付いたリンクは想定していない(URDF から作ったものには付かない)。
- 症状を測る条件(drive の強さ、許容誤差)はロボットに合わせて `[probe]` で変える。

## ファイル構成

| ファイル | 中身 |
|---|---|
| `run_pipeline.py` | コマンドラインの入口 |
| `myext/robot/assembly_pipeline/kinds.py` | 決まった値しか取らないものの型(Literal) |
| `spec.py` | 仕様と TOML の読み込み |
| `urdf_model.py` | URDF の読み取り、空のリンクの判定、順運動学 |
| `usd_util.py` | 剛体の解決、ワールド固定 joint の構造判定、リンク prim の対応 |
| `sources.py` | usda の中身の地図(どの URDF から作った部分がどこにあるか)、リンクの書き方の解決 |
| `part_import.py` | import(ハッシュで飛ばす、出力先の置き換え)と正規化層 |
| `compose.py` | 結合ファイルを作る(結合の 7 つの書き込み) |
| `official.py` | 公式のままの比較用に、Robot Assembler で結合する |
| `probes.py` | 症状を測る |
| `rules.py` | 規則 R01〜R14 の判定と評価 |
| `geometric_bridge.py` | GeometricRobot を作る、PhysX と突き合わせる |
| `report.py` | 報告の書き出しと前回との比較 |
| `pipeline.py` | 全体の流れ |
| `examples/` | 検証用の URDF・メッシュ・仕様 |
| `tests_kit/run_tests.py` | テスト |
