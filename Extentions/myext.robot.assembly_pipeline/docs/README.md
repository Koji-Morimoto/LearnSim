# myext.robot.assembly_pipeline

2026-11-19 / Isaac Sim 6.2.0(URDF Importer 3.11.12、Robot Assembler 3.3.9)で作成・確認

URDF から usda を作り、usda どうしを結合した**結合ファイル**を作る拡張機能です。何度実行しても同じ結果になります(冪等)。結合ファイルは次の結合の親にも子にもできるので、中間の状態がそのまま 1 つの usda として残ります。

あわせて、URDF と usda を先に調べて既知の罠に当たるかを判定します(**事前チェック**)。判定が出たら止めるか進めるかを選べます。進めた場合も、最後にその症状が**実際に起きたか**を測ります(**事後確認**)。測るのは「公式のまま」と「修正後」の両方です。公式のままで症状が出なくなれば、報告に「公式のままでも起きなかった(公式側の修正の可能性)」と出ます。

## 言葉

この拡張の中では、次の言葉をこの意味でだけ使います。

| 言葉 | 意味 |
|---|---|
| usda | default prim を持つ USD ファイル。URDF から作ったもの、または結合ファイル。どこでもファイル名(`arm.usda` のように拡張子まで)で指す |
| 結合 | 子の usda を、親の usda のリンクに固定する 1 回の操作。中身は下の「結合の中身」の 7 つの書き込み |
| 結合ファイル | 親の usda に 1 回以上の結合を書いた、新しい usda。次の結合の親にも子にもできる |
| 公式のまま | URDF Importer の出力を Robot Assembler でそのまま結合した場合。比較用に、パイプラインの中で実際に結合して測る |
| 修正後 | このパイプラインの出力 |
| 修正 | このパイプラインが入れる直し(規則ごとに `fix = "skip"` で外せる) |
| 一番上の prim | usda の default prim。名前は usda のファイル名の `.usda` より前(`robot.usda` なら `/robot`) |
| 一番上の URDF | usda の一番上の prim の中身を作った URDF。URDF から作った usda ならその URDF、結合ファイルなら `parent_usda` を何段でもたどって行き着いた URDF |

TOML のキー名にも `usda` があります(`[[urdf_to_usda]]` と `[[join_file]]` の `usda` = 作るファイルの名前)。文章で「usda」と書くときは上の表の意味です。

ほかに出てくる用語:

| 用語 | 意味 |
|---|---|
| 剛体 | 物理で動く物体(UsdPhysics の RigidBodyAPI が付いた prim)。URDF のリンクは、形も慣性も無い空のリンクを除いて剛体になる |
| joint の body の解決 | joint の body0・body1 が指す prim から祖先へたどり、最初に見つかった剛体をその側の物体とする(剛体が無ければ最初に見つかったコライダー)。body が空のときと、たどっても何も見つからないときは、その側は**ワールド**(どこにも動かない空間)。PhysX と同じ決め方 |
| ワールド固定 joint | 上の解決で、片側が剛体、もう片側がワールドになる joint。URDF Importer はロボットを空間に固定するために root_joint を作る。root_joint は body0 に一番上の prim(剛体ではない)、body1 にルートリンクを書くので、body0 側がワールドになる。body0・body1 のどちらかが空の joint だけを指すのではない |
| 子の外につながる joint | 結合で子を置いた後、子の prim の下にある joint のうち、上の解決で片側だけが子の中の剛体になるもの(もう片側はワールドか、子の外の剛体)。子の中のワールド固定 joint は、子を剛体の子に置くと、ワールド側が親の剛体に解決されてこれになる |
| 剛体の子に置く | 子を置く prim(`place_child_under` で決まる prim)自身か、その祖先のどれかが剛体であること。既定の置き場所(parent_link の prim)は、そのリンクの prim か祖先のリンクの prim に剛体があれば(リンクの prim は入れ子なので、ふつうはある)剛体の子に置くことになる。`place_child_under = "."`(一番上の prim)は自身も祖先も剛体ではないので、剛体の子に置かない |
| 子のルートリンク | 子の usda の一番上の URDF のルートリンク(URDF の木の根のリンク)。子が結合ファイルでも、その中で結合したリンクは含まない |
| Articulation Root | 関節でつながった剛体の集まり(Articulation)の根を示す印。1 つの Articulation に 1 つだけにする |
| mimic の leader / follower | URDF の `<mimic>`。follower の関節は leader の関節の値に比例して動く |
| 結合面 | 結合で一致させる 2 つのリンク(parent_link と child_link)の座標系 |
| teleport | 物理を回さずに、関節の値から各リンクの位置を直接書き換えること(RobotPoser の `apply_joint_state` が Stop 中にする) |
| 判定 / 症状 / 最終判定 | 判定 = 事前チェックで「このままだと○○が起きる」と出たもの。症状 = 実際に測った結果。最終判定 = 両方から決める結論(下の「最終判定」) |

## すぐ試す

```powershell
cd C:\isaacsim-6.2.0\extsUser\Extentions\myext.robot.assembly_pipeline\examples
C:\isaacsim-6.2.0\python.bat ..\run_pipeline.py multi_arm.toml --check
C:\isaacsim-6.2.0\python.bat ..\run_pipeline.py multi_arm.toml
```

- `--check` は Isaac Sim を起動せずに(数秒)、仕様を検査して、参照関係の一覧(どの usda をどこで作り、どこで使うか)を表示します。終了コードは 0(誤りなし)か 4(誤りあり)。Isaac Sim の `python.bat` で動かします(標準ライブラリと numpy だけを使うので、numpy の入った Python 3.11 以上でも動く)。実行するときも、Isaac Sim を起動する前に同じ検査をします。
- 実行すると、1〜2 分で `examples\out\multi_arm\` に次のものができます。

| ファイル | 中身 |
|---|---|
| `base.usda`、`arm.usda`、`two_finger.usda`、`rod.usda` | URDF から作った usda([[urdf_to_usda]] の usda で決めた名前) |
| `arm_unit.usda` | 結合ファイル:arm.usda + エンドエフェクタ(VariantSet `EndEffector`: TwoFinger / Rod / None)。単体で使える |
| `robot.usda` | 結合ファイル:base.usda + arm_unit.usda を 4 本(arm1〜arm4)。アームごとにエンドエフェクタを選んでいる |
| `_import\base\`、`_import\arm\` など | URDF Importer の生成物(手を入れない)。フォルダ名は URDF から作った usda のファイル名の `.usda` より前。その usda はこれを参照している |
| `report.md` / `report.json` | 参照関係の一覧、規則ごとの判定、測った症状 |
| `report_previous.json` | 1 つ前の報告(前回からの変化を出すため) |

実行の終了コード: 0 = 問題なし、1 = 最終判定に `still_broken` か `unexpected` がある、または幾何モードと PhysX が食い違った、2 = 判定で止めた、3 = 例外、4 = 仕様の誤り。

ほかの例: `arm_gripper.toml`(結合 1 回)、`franka\franka.toml`(Isaac Sim 同梱の Franka の腕に手を z 軸まわり -45° で結合)。

## TOML の書き方

### 参照のしくみ

usda は**どこでもファイル名(拡張子 `.usda` まで)**で指します。ファイル名を書くキーは 4 つだけで、そのうち 2 つが「作る」側、2 つが「使う」側です。

| キー | セクション | 向き | 書く値 |
|---|---|---|---|
| `usda` | `[[urdf_to_usda]]` | 作る | output_dir に作る usda のファイル名 |
| `usda` | `[[join_file]]` | 作る | output_dir に作る結合ファイルのファイル名 |
| `parent_usda` | `[[join_file]]` | 使う | 上の 2 つの `usda` のどれか(親) |
| `child_usda` | `[[join_file.join]]` | 使う | 上の 2 つの `usda` のどれか(子) |

```
[[urdf_to_usda]]  urdf = "urdf/arm.urdf"      usda = "arm.usda" ─────────────┐
[[urdf_to_usda]]  urdf = "urdf/gripper.urdf"  usda = "two_finger.usda" ──┐   │
                                                                         │   │
[[join_file]]  usda = "arm_unit.usda" ◀── 作る        parent_usda = "arm.usda" ◀┘
  [[join_file.join]]  name = "ee_two_finger"   child_usda = "two_finger.usda" ◀┘
        │
        └─ arm_unit.usda は次の [[join_file]] の parent_usda / child_usda に書ける
```

同じファイル名を 2 か所で作ることはできません(エラーになる)。使う側に、どこでも作っていない名前を書いてもエラーになります(作っている名前の一覧が出る)。

### セクションとキー

TOML の中のパス(`output_dir`、`urdf`、`ros_package_paths` の `path`)は、絶対パスか、TOML ファイルのフォルダからの相対パスで書きます。

**一番上**

| キー | 必須 | 値 | 意味 |
|---|---|---|---|
| `output_dir` | 必須 | フォルダのパス | 出力先。全部の usda と報告はここに作る |
| `compare_official` | | true / false(既定 true) | 公式のまま(Robot Assembler)でも結合して症状を測るか |
| `check_geometric` | | true / false(既定 true) | 結合ファイルごとに GeometricRobot を作って PhysX と比べるか |

**`[[urdf_to_usda]]`**(URDF 1 本から usda を 1 つ作る。何個でも書ける)

| キー | 必須 | 値 | 意味 |
|---|---|---|---|
| `urdf` | 必須 | URDF のパス | 入力 |
| `usda` | 必須 | ファイル名(`.usda` まで。`.usda` より前は英数字と `_` で、先頭は数字以外) | 出力。`<output_dir>/<この名前>` に作る。ほかの場所ではこの名前で指す。一番上の prim の名前は `.usda` より前(`arm.usda` なら `/arm`)。URDF Importer の生成物は `<output_dir>/_import/<.usda より前>/` に出る(`arm.usda` なら `_import/arm/`。URDF のファイル名は使わない) |
| `collision_from_visuals` | | true / false | URDF Importer の同名の設定。省略時は、URDF に collision が 1 つでもあれば false、無ければ true |
| `collision_type` | | 文字列(既定 "Convex Hull") | collision_from_visuals のときの近似 |
| `physics_variant` | | "auto" / "physx" / "physics" / "mujoco" | 使う Physics variant。auto は physx があれば physx、無ければ physics |
| `ros_package_paths` | | `[{ name = "パッケージ名", path = "フォルダ" }]` | package:// の解決先 |
| `joint_drive_stiffness` / `joint_drive_damping` | | 数値 | URDF Importer の一括 gain 指定(全関節。mimic の follower にも入るので、修正で follower は 0 に戻す) |

**`[[join_file]]`**(結合ファイルを 1 つ作る。何個でも書ける。順番は自由で、依存の順に作る)

| キー | 必須 | 値 | 意味 |
|---|---|---|---|
| `usda` | 必須 | ファイル名(`.usda` まで。`.usda` より前は英数字と `_` で、先頭は数字以外) | 出力。`<output_dir>/<この名前>` に作る。一番上の prim の名前は `.usda` より前 |
| `parent_usda` | 必須 | 作っている usda のファイル名 | 入力。親 |

**`[[join_file.join]]`**(直前の `[[join_file]]` の中の結合 1 回。何個でも書ける)

| キー | 必須 | 値 | 意味 |
|---|---|---|---|
| `name` | 必須 | 英数字と `_`(先頭は数字以外) | この結合の名前(この結合ファイルの中で一意)。使われる場所:報告の表示(`arm_unit.usda/ee_two_finger`)、`child_prim_name` の既定値、この結合ファイルを親か子にした後の結合でのリンクの前置き(`arm1/flange`)。リンクの前置きには child_prim_name ではなく常にこの name を使う |
| `child_usda` | 必須 | 作っている usda のファイル名 | 入力。子 |
| `parent_link` | 必須 | リンクの書き方(下) | 親の usda のリンク。子のリンクの座標系をここに一致させる |
| `child_link` | 必須 | リンクの書き方(下) | 子の usda のリンク。ふつうは子のルートリンク。ルートリンクでないリンクでも物理(PhysX)と幾何モードでは正しく結合されるが、RobotPoser の teleport(Stop 中の `apply_joint_state`)では子がずれる(実測で 2.1 m。規則 R12 で判定する。このパイプラインの修正でも直らないので、測ると修正後も症状が出て、最終判定は `still_broken`、終了コードは 1 になる) |
| `place_child_under` | | リンクの書き方 / `"."` / `"/..."` | 子を Reference で置く親側の prim。子の prim のパスは `<この prim>/<child_prim_name>` になる。省略時は parent_link の prim(親の usda の中のそのリンクの prim。stage の親子が実際のリンクの親子に揃う)。`"."` はこの結合ファイルの一番上の prim。`"/"` で始まれば、この結合ファイルの中の prim のパス(一番上の prim から書く。`robot.usda` なら `/robot/...`) |
| `child_prim_name` | | 英数字と `_` | 出力。子を置く prim の名前(省略時は name)。VariantSet の選択肢どうしで同じにすると、どれを選んでもパスが同じ。幾何モードの構成要素の名前にも使う(下の「何をするか」の 8) |
| `offset_xyz` | | `[x, y, z]`(m) | parent_link の座標系で見た child_link の位置 |
| `offset_rpy` | | `[r, p, y]`(rad) | 同じく向き(URDF と同じ固定軸 XYZ) |
| `variant_set` | | 文字列 | 出力。この結合を選択肢にする VariantSet の名前。この結合ファイルの一番上の prim に付く |
| `variant` | variant_set を書いたら必須 | 文字列(`None` 以外。大文字小文字を区別する) | 出力。その選択肢の名前。同じ variant_set の結合は排他で、TOML に先に書いたものが既定の選択。選択肢 `None`(何も結合しない)は自動で足す。既定の選択を `None` にはできない。`None` を選ぶには、この結合ファイルを子にする別の結合ファイルの `[[join_file.join]]` で `child_variant_selection` に書く |
| `child_variant_selection` | | `{ VariantSet の名前 = "選択肢の名前" }` | 子の usda の VariantSet で選ぶもの。書けるのは、子の usda(結合ファイル)の一番上の prim の VariantSet だけ = 子を作った `[[join_file.join]]` の `variant_set` と `variant`(と `None`)。孫の VariantSet は、子の結合ファイルを作るときにその中の `child_variant_selection` で選ぶ。書かなかった VariantSet は既定の選択。結合ファイルの中に VariantSet の選択として書くので、できた usda を開いた後でも USD の上で切り替えられる(ただし、切り替えた後の状態はパイプラインでは測っていない) |

**`[checks]`**、**`[checks.rules.R01]`** など

| キー | 値 | 意味 |
|---|---|---|
| `on_violation` | "continue"(既定) / "stop" | 判定が出たとき、進めて最後に答え合わせするか、止めるか。止める場所は、URDF だけで判定する R01・R04・R10 は import の前、ほかの規則は import の後で結合ファイルを作る前。止めたときも報告は書く(終了コード 2) |
| `fix` | "apply"(既定) / "skip" | 修正を入れるか。skip で、修正が要らなくなったかを確かめられる |

`[checks.rules.R02]` のように規則ごとに書いた値が、`[checks]` の値より優先されます。

R11・R14 は結合が 1 回でもあれば必ず判定が出ます(Robot Assembler の動きによるもので、TOML の書き方に関係しない)。`[checks]` で `on_violation = "stop"` にすると、結合ファイルがある限り毎回止まるので、止めたい規則だけを `[checks.rules.Rxx]` に書きます。

自分の修正を持たない規則があります。その規則に `fix` を書いても出力は変わりません(報告の文面が「修正を skip した出力」になるだけ)。

| 規則 | 修正 |
|---|---|
| R01・R09 | R02 の修正(ワールド固定 joint を構造で探して無効にする)で直る。切り替えるのは R02 の `fix` |
| R10 | 修正しない(Asset Transformer rules 1.7.7 以降は公式のままで起きないため、測るだけ) |
| R14 | 外せない(固定関節を常に決まったパスに 1 本作る) |
| ほか | 規則ごとの修正。`fix = "skip"` で外れる |

**`[probe]`**(症状を測る条件。省略可)

| キー | 既定 | 意味 |
|---|---|---|
| `steps` | 240 | Play して関節を動かすステップ数 |
| `drive_stiffness` / `drive_damping` | 1e4 / 1e3 | 関節を動かす drive の強さ。単位は USD の drive と同じで、回転は N·m/度・N·m·s/度、直動は N/m・N·s/m |
| `revolute_target` | 0.5 | 回転関節の目標(rad)。可動範囲 [lower, upper](URDF の limit。rad)のある回転関節では、[0.8 × lower, 0.8 × upper] に収まるように切り詰める(0 を中心に範囲を 8 割に縮めたもの)。continuous はそのまま |
| `prismatic_ratio` | 0.5 | 直動関節の目標 = lower + prismatic_ratio × (upper − lower)。可動範囲が無ければ lower = 0、upper = 0.1 m とする |
| `mimic_leader_stiffness_linear` / `mimic_leader_damping_linear` | 20 / 2 | mimic の確認で leader(直動)を動かす柔らかい drive(N/m・N·s/m) |
| `mimic_leader_stiffness_angular` / `mimic_leader_damping_angular` | 0.05 / 0.005 | 同じく leader(回転)(N·m/度・N·m·s/度) |
| `mimic_steps` | 300 | mimic の確認のステップ数。R03 の「最後」はこのステップ |
| `angle_tol_deg` / `length_tol_m` | 1.0 / 0.001 | 症状ありとする差の許容(度 / m)。差が許容より大きいと症状あり(等しければ症状なし)。比べる差:関節の到達誤差(目標と最終値)、結合のずれ(結合面の位置と向き)、teleport 後の結合のずれと順運動学との差、幾何モードと PhysX のリンク位置の差(長さだけ) |

### リンクの書き方

- usda の一番上の URDF のリンク:リンク名だけ(`flange`)。URDF から作った usda ならその URDF、結合ファイルなら `parent_usda` をたどった先の URDF。
- 結合ファイルの中で結合した子のリンク:`結合の name/リンク名`(`arm1/flange`)。入れ子なら `arm1/ee_two_finger/finger_l`。
- parent_link と、リンクの書き方で書いた place_child_under は親の usda の中での書き方、child_link は子の usda の中での書き方です(place_child_under の `"."` と `"/..."` は、作る結合ファイルの中の prim を指す)。子が結合ファイルなら、child_link にも前置きを書けます(子 arm_unit.usda のエンドエフェクタのリンクなら `ee_two_finger/finger_l`)。
- 前置きのある結合は、公式のまま(Robot Assembler)では再現しないので、その結合の公式のままの比較は `not_measured` になります。
- 前置きでたどる結合が VariantSet の選択肢の中にあるときは、その選択肢が選ばれている場合だけ書けます。選ばれていなければ仕様の誤り(終了コード 4)。選ばれているかは、child_link なら この結合の `child_variant_selection`(書かなければ既定の選択)、parent_link・place_child_under なら親の usda の既定の選択で決まります。
  - 例:arm_unit.usda の VariantSet `EndEffector` の既定が TwoFinger のとき、`child_link = "ee_two_finger/finger_l"` は書ける。`child_variant_selection = { EndEffector = "Rod" }` や `"None"` と一緒に書くとエラー。
  - できた usda を USD の上で別の選択肢に切り替えると、その結合の固定関節の相手の prim が無くなります(パイプラインでは確かめない)。

リンクの prim は、URDF Importer の出力ではリンクの木のとおりに入れ子になり、`/<一番上の prim>/Geometry/<ルートリンク>/<その子のリンク>/…` に置かれます(例:base.usda の port1 を robot.usda の中で見ると `/robot/Geometry/base_body/turret/port1`)。TOML にはこのパスでなく、上のリンクの書き方で書きます。

`--check` の「結合」の表に、リンクがどの usda のどのリンクに解決されたかが出ます。

### 例(examples/multi_arm.toml の一部)

```toml
output_dir = "out/multi_arm"

[[urdf_to_usda]]
urdf = "urdf/multi_base.urdf"     # 入力: URDF
usda = "base.usda"                # 出力: output_dir/base.usda

[[urdf_to_usda]]
urdf = "urdf/arm.urdf"            # 入力: URDF
usda = "arm.usda"                 # 出力: output_dir/arm.usda

[[urdf_to_usda]]
urdf = "urdf/gripper.urdf"
usda = "two_finger.usda"

[[join_file]]
usda = "arm_unit.usda"            # 出力: output_dir/arm_unit.usda
parent_usda = "arm.usda"          # 入力: 親

[[join_file.join]]
name = "ee_two_finger"            # この結合の名前
child_usda = "two_finger.usda"    # 入力: 子
parent_link = "flange"            # arm.urdf のリンク名
child_link = "gripper_base"       # gripper.urdf のリンク名
child_prim_name = "end_effector"  # 出力: 子を置く prim の名前
variant_set = "EndEffector"       # 出力: VariantSet の名前
variant = "TwoFinger"             # 出力: 選択肢の名前

[[join_file]]
usda = "robot.usda"
parent_usda = "base.usda"

[[join_file.join]]
name = "arm2"
child_usda = "arm_unit.usda"                       # 入力: 上で作った結合ファイル
parent_link = "port2"
child_link = "base_link"                           # arm_unit.usda の一番上の URDF(arm.urdf)のリンク
child_variant_selection = { EndEffector = "Rod" }  # arm_unit.usda の VariantSet "EndEffector" で "Rod" を選ぶ
```

### エラーの例

```
[SPEC ERROR] usda = 'arm.usda' を 2 か所で作ろうとしている: [[urdf_to_usda]] 1 番目 と [[urdf_to_usda]] 3 番目。出力のファイル名は 1 つずつ別にする
[SPEC ERROR] [[join_file]] usda = 'unit.usda' の parent_usda = 'arms.usda' は、この TOML のどこでも作っていない。作っている usda: ['arm.usda', 'two_finger.usda', 'unit.usda']
[SPEC ERROR] [[join_file]] usda = 'unit.usda' の [[join_file.join]] name = 'ee' の parent_link = 'flang': arm.usda(URDF arm.urdf)にリンク 'flang' が無い。ある名前: ['base_link', 'flange', 'link1', 'link2']
[SPEC ERROR] [[join_file]] usda = 'robot.usda' の [[join_file.join]] name = 'u1' の child_variant_selection: 子 unit.usda の VariantSet 'EndEffector' に選択肢 'Rod' が無い。ある選択肢: ['TwoFinger', 'None']
[SPEC ERROR] [[join_file]] 1 番目(usda = 'unit'): [[join_file]] の usda: usda のファイル名は、フォルダを含まず拡張子 .usda まで書く(例 "arm.usda"): 'unit'
[SPEC ERROR] [[join_file]] 1 番目(usda = 'unit.usda'): 知らないキー ['parent']。書けるキー: ['join', 'parent_usda', 'usda']
```

## 結合の中身(7 つの書き込み)

| # | USD への書き込み | Robot Assembler との違い(= 修正。規則) |
|---|---|---|
| 1 | 親の usda の指定した prim の子として、子の usda を Reference で置く。その prim が instanceable なら外す | Assembler は置き場所を選ばせない(置いた場所のまま)。instanceable な prim の下には書けず失敗する(R05) |
| 2 | 子の prim の Transform を、親のリンクと子のリンクの URDF の座標系(+ offset)が一致する値にする | Assembler は prim の位置で合わせる。prim が URDF のリンク座標からずれていると、結合面がずれる(R04)。offset を指定する機能も無い |
| 3 | 親と子の剛体をつなぐ固定関節を、決まったパス `<子の prim のパス>/JoinJoint` に 1 本作る。子のリンクに剛体が無ければ、fixed joint でつながる子孫の剛体に固定する | Assembler はやり直すたびに固定関節が増える(R14)。子のリンクに剛体が無いと結合できない(R08) |
| 4 | 子の外につながる joint(上の「言葉」の定義)を、名前でなく構造で探して無効にする | Assembler は body0 か body1 が空の joint しか無効にしない。URDF Importer の root_joint は body0 に prim が書いてある(一番上の prim。剛体ではないのでワールドに解決される)ので残る(R02。剛体の子に置いたときは、body0 が親の剛体に解決されて閉ループ = R09) |
| 5 | 子の Articulation Root を Physics・Newton・Physx の全部外す | Assembler は Physics の方だけ外す。NewtonArticulationRootAPI が Physics の方を内蔵しているので残る(R07) |
| 6 | 親の剛体と子の剛体の組の衝突を除外する | Assembler は何もしない(R11) |
| 7 | 固定関節に IsaacJointAPI を付け、結合ファイルのロボットスキーマの関係の末尾に子を足し、無効にしたワールド固定 joint を子の robotJoints から消す | Assembler は先頭に足すので、RobotPoser の木の根が子になる。RobotPoser は無効にした joint も使う(R12) |

VariantSet を使う結合は、1〜7 を結合ファイルの一番上の prim の VariantSet の選択肢の中に書きます。

## 何をするか

1. **仕様の検査**(Isaac Sim を起動する前):TOML のキー、usda のファイル名の参照、リンク名、子の VariantSet の選択。
2. **URDF の判定**(import の前)。
3. **import**: 入力(URDF・メッシュ・設定・importer の版)のハッシュが前回と同じなら飛ばします。違えば一時フォルダ `<output_dir>\.staging\<.usda より前>\` に import してから `<output_dir>\_import\<.usda より前>\` を置き換えます(importer に任せると、出力先が空でなければ `_1` 付きの別フォルダになる = R13)。
4. **URDF から作る usda**(`<output_dir>/<usda>`): 生成物を参照し、Physics variant を選び(R06)、mimic の follower の drive を stiffness = damping = 0 にします(R03)。
5. **usda の判定**(結合ファイルを作る前)。
6. **結合ファイル**を依存の順に作ります。毎回まるごと作り直し、中身が同じならファイルは書き換えません。各ファイルを 2 回作って、2 回目に変化が無いことも確かめます。
7. **症状の確認**:
   - 公式のまま:結合 1 回ごとに、親と子の一番上の URDF の生成物を Robot Assembler で実際に結合して測ります(リンクの書き方に `/` が無い結合だけ)。
   - 修正後:結合ファイルごとに、その結合ファイルの一番上の prim の VariantSet について、まず全部を既定の選択にして 1 回、次に既定でない選択肢(`None` を除く)を全部、1 つずつ選んで(ほかの VariantSet は既定のまま)測ります。子の中の VariantSet は、`child_variant_selection` で選んだもの(書かなければ既定)のまま測ります。
   - どちらも Play して関節を drive で動かし、関節の到達、結合のずれ、親との接触、Play 中のログ(閉ループなど)を測ります。Stop 中には RobotPoser の `apply_joint_state` で teleport して、子が付いてくるかと、親の剛体が URDF の順運動学どおりに来たかを測ります。
   - mimic は URDF ごとに、leader を柔らかい drive で動かして測ります。
8. **幾何モード**: 結合ファイルごとに GeometricRobot を作り、同じ関節値で PhysX とリンク位置を比べます。構成要素は IsaacRobotAPI から自動で作ります。親の部分の構成要素の名前は結合ファイルの一番上の prim の名前(`robot`)。結合した子の構成要素の名前は、子の prim の名前(`child_prim_name`。省略時は name)で、入れ子なら外側から `/` でつなぐ(`arm1`、`arm1/end_effector`。先頭に `robot/` は付けない)。リンクの前置き(常に結合の name)とは別なので、child_prim_name を変えるとこちらだけが変わります。
9. **報告**: 規則ごとに、判定と、公式のまま・修正後の症状から最終判定を出します。

## Python から使う

```python
from myext.robot.assembly_pipeline import load_spec, run_pipeline, PipelineStopped

try:
    result = run_pipeline(load_spec("robot.toml"))   # headless の Isaac Sim の中で呼ぶ(run_pipeline.py を参照)
    print(result.report["status"], result.outputs["robot.usda"])
except PipelineStopped as exc:
    print(exc.report["stopped_by"])                  # 止めた理由。報告は書いてある
```

Python で仕様を組み立てる場合は `PipelineSpec` / `UrdfSource` / `JoinFileSpec` / `JoinSpec` / `RuleSetting` / `ProbeSettings`(`spec.py`)。キーは TOML と同じ名前。

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
| R01 | URDF から作る usda | ルートリンクが空(visual・collision 無し、inertial 無しか質量 0) | ワールド固定 joint が root_joint という名前にならない | 生成物のワールド固定 joint の名前 / 子として結合した後に残ったワールド固定 joint |
| R02 | 結合 | 子(子の usda を単体で見たとき)にワールド固定 joint があり、その body0・body1 の両方に prim が書いてあり(Robot Assembler はこれを無効にしない)、子を剛体の子に置かない | 子がワールドに固定されて、親の関節が動かない | 親の関節の到達誤差、結合のずれ |
| R03 | URDF から作る usda | mimic follower の drive に stiffness ≠ 0 か damping ≠ 0 | leader が目標に届かない・遅れる | 柔らかい drive で動かした leader の位置(10・30・60 ステップ目と最後)、mimic の関係の誤差 |
| R04 | URDF から作る usda | 子の形状が 1 つだけの末端リンク | prim の位置が URDF のリンク座標からずれる | prim と URDF の順運動学の差、結合面のずれ |
| R05 | 結合 | 子を置く prim か子のリンクが instanceable | instance proxy への書き込みで結合が失敗する | 結合の成否 |
| R06 | URDF から作る usda | 出力 USD の Physics variant が未選択 | 剛体も関節も無い | 剛体の数 |
| R07 | 結合 | 子に NewtonArticulationRootAPI | Articulation Root が残る | 結合後の子の Articulation Root |
| R08 | 結合 | 子のリンクの位置にも上にも剛体が無い | 結合できない(Assembler 3.3.8 より前は分離) | 結合の成否、固定関節の子の側の剛体 |
| R09 | 結合 | 子を剛体の子に置き、子(子の usda を単体で見たとき)に、body0・body1 の両方に prim が書いてあるワールド固定 joint がある | 閉ループのエラー | Play 中のログ |
| R10 | URDF から作る usda | visual と collision が同じメッシュで collision_from_visuals = False | 衝突メッシュが見える | purpose が guide でなく visual と同じ位置・大きさの collider |
| R11 | 結合 | (結合のたびに) | 子と親の接触 | Play 中の接触の報告 |
| R12 | 結合 | 子を剛体の子に置かない、または child_link が子のルートリンクでない | RobotPoser の teleport で子が置いて行かれる | teleport 後の結合のずれ、親の剛体の順運動学との差 |
| R13 | URDF から作る usda | 出力先が空でない | 別名(`_1`)のフォルダに出力される | importer の出力先の決め方、実際の出力先 |
| R14 | 結合 | (結合のたびに) | やり直すと固定関節が増える | Assembler で 2 回結合した後の本数 / 結合ファイルの本数と、作り直しで変化が無いか |

報告の対象の名前は、URDF から作る usda はそのファイル名(`arm.usda`)、結合は「結合ファイルのファイル名/結合の name」(`robot.usda/arm1`)、R03 は「ファイル名:follower の関節名」。規則の一覧は `kinds.py` の `RuleId` と `rules.py` の `RULES` にあり、食い違うと読み込み時に止まります。

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
[R09] robot.usda/arm1: 子 arm_unit.usda を剛体の子に置き、body0 が空でないワールド固定 joint ['root_joint'] が残ると、
その body0 が親の剛体に解決されて閉ループになる と判定した → 公式のままでは、やはり「閉ループのエラー」が起きた
(Play 中の閉ループのログ 1 件)。修正後は起きなかった(Play 中の閉ループのログ 0 件)
[R10] arm.usda: ... と判定した → 公式のままでも、最終的には「衝突メッシュが見える」は起きなかった(...)。
Asset Transformer rules 1.7.7 以降の修正で、衝突側が guide のまま
```

## 確かめたこと(テストで毎回確認している)

`run_tests.ps1`(13 本、約 3 分)。

| 場面 | 確かめること |
|---|---|
| 仕様の検査 | 同じファイル名を 2 か所で作る、作っていない usda を指す、拡張子の書き忘れ、循環、存在しないリンク名、子に無い VariantSet の選択肢、選ばれていない VariantSet の選択肢の中のリンクを、場所を指したエラーで止める |
| 結合 1 回・親のリンクの子に置く | 状態 ok。R06・R07・R09・R14 が修正で解消、R10 は公式のままでも起きない。GeometricRobot と PhysX のリンク位置の差 1e-4 m 未満 |
| 同じ仕様でもう一度 | import を飛ばす、結合ファイルは 1 回目も 2 回目も変化なし、USD ファイルのハッシュが前と同じ、`_1` のフォルダができない |
| 結合 1 回・一番上の prim の子に置く(`place_child_under = "."`) | R02(公式のままでは親の関節が動かない)と R12(teleport で子が置いて行かれる)が修正で解消 |
| R02・R07・R12 を skip | 3 つとも「修正を skip した出力でも起きた」になり、状態が problems |
| R02 を stop | 結合ファイルを作る前に止まる |
| 空のルートリンクの子(子のリンクも空のリンク) | R08(公式のままでは ValueError)と R01(ワールド固定 joint の名前が mount_joint)が修正で解消 |
| mimic | URDF の `<dynamics damping>` だけなら判定も症状も無し(PhysX の drive に入らない)。importer の一括 damping 指定なら修正で解消 |
| 4 本アーム(結合ファイル 2 段) | 状態 ok。Articulation はベースの 1 つ、ワールド固定 joint は 1 本、DOF 13 個が全部目標に届く。arm2 は Rod、arm4 は None。`arm_unit.usda` は単体で開ける。構成要素は robot・arm1〜arm4・各 end_effector |
| 4 本アームでもう一度 | USD ファイルのハッシュが前と同じ |
| Franka(同梱の腕と手) | 状態 ok、offset(-45°)込みで結合面が一致 |

## 分かっている制約

- 実行は headless 専用(Kit の更新を自分で回すため)。`--check` は Isaac Sim なしで動く。
- 公式のままの比較は、親・子とも一番上の URDF のリンクどうしの結合だけ(子が結合ファイルなら、その一番上の URDF の生成物で代わりに測る)。それ以外は `not_measured`。
- 修正後の測定は、既定の選択と、既定でない選択肢を 1 つずつ(VariantSet が 2 つ以上あっても、選択肢の全部の組み合わせは測らない)。USD の上で後から切り替えた状態も測らない。
- 既存の USD(URDF から作っていないもの)はまだ入力にできない。
- スケールの付いたリンクは想定していない(URDF から作ったものには付かない)。
- 症状を測る条件(drive の強さ、許容誤差)はロボットに合わせて `[probe]` で変える。

## ファイル構成

| ファイル | 中身 |
|---|---|
| `run_pipeline.py` | コマンドラインの入口(`--check` もここ) |
| `myext/robot/assembly_pipeline/kinds.py` | 決まった値しか取らないものの型(Literal) |
| `spec.py` | 仕様と TOML の読み込み(Isaac Sim なしで動く) |
| `spec_check.py` | 仕様の検査(リンク名、子の VariantSet の選択)と参照関係の一覧(Isaac Sim なしで動く) |
| `urdf_model.py` | URDF の読み取り、空のリンクの判定、順運動学 |
| `usd_util.py` | 剛体の解決、ワールド固定 joint の構造判定、リンク prim の対応 |
| `sources.py` | usda の中身の地図(どの URDF から作った部分がどこにあるか)、リンクの書き方の解決 |
| `part_import.py` | import(ハッシュで飛ばす、出力先の置き換え)と、URDF から作る usda |
| `compose.py` | 結合ファイルを作る(結合の 7 つの書き込み) |
| `official.py` | 公式のままの比較用に、Robot Assembler で結合する |
| `probes.py` | 症状を測る |
| `rules.py` | 規則 R01〜R14 の判定と評価 |
| `geometric_bridge.py` | GeometricRobot を作る、PhysX と突き合わせる |
| `report.py` | 報告の書き出しと前回との比較 |
| `pipeline.py` | 全体の流れ |
| `examples/` | 検証用の URDF・メッシュ・仕様 |
| `tests_kit/run_tests.py` | テスト |
