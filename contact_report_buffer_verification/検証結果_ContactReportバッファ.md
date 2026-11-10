# 検証結果：ContactReport の pull 型 API（get_contact_report）のバッファの実態

実施日：2026-11-02
対象：Isaac Sim 6.2.0（`6.2.0-rc.14`）の omni.physx 110.4.4、`IPhysxSimulation.get_contact_report()`、および Robot Self-Collision Detector（`isaacsim.robot_setup.collision_detector` 0.2.3）が使う `get_initial_collider_pairs()`

検証コード：[code/](code/)　生データ：[results/](results/)（`*.json` が結果、`*.log` が実行ログ）

## 1. 結論

検証前の疑問と答え：

| 疑問 | 答え | 根拠 |
| --- | --- | --- |
| pull 型だが、バッファを見に行っているのか | **はい。** omni.physx 側に「直近の報告」を保持するバッファがあり、`get_contact_report()` は呼ばれた時点の中身を**新しい Python オブジェクトにコピーして**返す。読んでも消えない（消費型ではない） | P-02, P-03 |
| 何周期分キャッシュしているか | **1 サブステップ分だけ。** 次の物理ステップの `fetch_results` で丸ごと上書きされ、過去のステップ分は残らない。`update_simulation` が 1 回で 2 サブステップ走ると、**1 つ目のサブステップの分は pull では取れない** | P-04, P-05, S-03, S-04 |
| 物理ステップを進めない限り、何度でも同じ情報にアクセスできるか | **はい。** 同じステップ内の 5 回の pull は接触点の座標まで完全に一致した。USD を書き換えても、`simulate` を呼んでも、`fetch_results` が終わるまでは前の内容のまま。**`detach_stage` しても消えない** | P-02, P-06, P-07 |
| ContactReport のバッファは毎物理ステップ作られているか | **はい（ContactReportAPI が付いた、起きているペアについて）。** pull も push 購読もしていなくても、`fetch_results` の中で毎ステップ作られる。API があるだけで 1 ペアあたり約 0.7〜0.8 µs/ステップ余計にかかり、その増分は読み方によらず同じだった。報告するものが無いステップではバッファは空になる | C-01, P-04 |

### 検出器にとっての意味

検出器は `simulate(1/60)` → `fetch_results()` → `get_contact_report()` を 1 回だけ行い、**`CONTACT_FOUND` のヘッダだけ**を集める（[physicsUtils.py:1665-1676](../../extscache/omni.physx-110.4.4+110.4.0.wx64.r.cp312.u7f4/omni/physx/scripts/physicsUtils.py#L1665-L1676)）。バッファは 1 ステップ分しかないので、この手順は「**アタッチ直後の最初の 1 ステップ**では、接触しているペアがすべて FOUND として出る」ことに依存している。

| 呼んだときの PhysX の状態 | 結果 | 根拠 |
| --- | --- | --- |
| 通常の Stop 状態（IPhysxSimulation 未アタッチ） | **正しく検出する。** 関数が自分でアタッチし、最初のステップで全ペアが FOUND になる | D-01 |
| タイムライン Play 中 / Pause 中 | **検出はできる。** ただし Play が使う経路は IPhysxSimulation のアタッチとは別なので、関数がアタッチし直し、終わると `detach_stage` する。その直後、PhysX の剛体オブジェクトはすべて消えていた（再開後は USD の値から作り直されて動いた） | D-04, D-05 |
| スクリプトなどで IPhysxSimulation にすでにアタッチ済み | **取りこぼす（0 ペア）。** 既存の接触は PERSIST になり、FOUND だけを拾うフィルタで捨てられる。API が無かったペアは、後から API を付けても一度離れるまで報告されない。さらに PhysX の時間だけが 1/60 s 進み、USD とずれる | D-02, D-03, D-06 |

GUI の Stop / Play / Pause の操作だけなら取りこぼしは起きなかった。取りこぼしが起きるのは、誰かが `get_physx_simulation_interface().attach_stage()` でステージをアタッチしたまま検出ボタンを押した場合である。

### そのほかに分かったこと

1. **`IPhysxSimulation.simulate(elapsed)` はサブステップに分けない。** `simulate(4/60)` は dt = 4/60 の 1 ステップとして進む（S-01）。分けるのは `IPhysx.update_simulation` で、この環境では 1 回あたり最大 2 サブステップだった。それ以上の時間は捨てられる（`minFrameRate=30` の環境。2 = 60 / 30 という関係は推論）。
2. **push 購読（`subscribe_contact_report_events`）のコールバックは `fetch_results` の中で同期的に呼ばれ、サブステップごとに届く。** そのため pull では消える 1 つ目のサブステップのイベントも push では受け取れる（S-04）。報告が空のステップではコールバックは呼ばれない（P-08 step3）。
3. **スリープした剛体のペアは報告から消える。LOST も出ない。** 静止した剛体は 24 ステップ目から報告が 0 件になった（C-02）。push と pull は同じステップで止まり、起こすと PERSIST から再開する（§4.5、Z-01〜Z-04）。「報告が無い＝離れた」とは限らない。
4. **pull のコピーのコストはペア数に比例する**（約 0.065 µs/ペア、2,500 ペア・10,000 接触点で約 0.16 ms）。同じステップの 2 回目の pull は 1 回目より速かった（2,500 ペアで 0.018〜0.026 ms）。理由は未確認（1 回目の変換結果を内部でキャッシュしている可能性と、CPU キャッシュが温まっているだけの可能性がある）。
5. threshold = 0 なら、隙間 1 mm（`separation = 0.001`）の非貫通ペアも報告される（P-02 の接触点）。検出器が出す「衝突ペア」には、貫通していなくても contact offset 内に入っている近接ペアが含まれる。

## 2. 仕組み

### 2.1 検出器の呼び出し経路

```text
[Check Collisions] ボタン
 └ widget.py  _on_check_collisions → _detect_collisions
    └ physicsUtils.get_initial_collider_pairs(stage)
       1. 全剛体に PhysxContactReportAPI(threshold=0) を一時サブレイヤーに付ける
       2. updateToUsd と fabricEnabled を一時的に False にする
       3. IPhysxSimulation にステージが未アタッチなら attach_stage(stage_id)
       4. simulate(1/60, 0) → fetch_results()
       5. get_contact_report()  ← pull。CONTACT_FOUND のヘッダだけを集める
       6. 自分でアタッチした場合だけ detach_stage()、設定とサブレイヤーを戻す
```

- 呼び出し元：[widget.py:1301](../../exts/isaacsim.robot_setup.collision_detector/isaacsim/robot_setup/collision_detector/widget.py#L1301)（タイムラインの状態は見ていない：[widget.py:943-967](../../exts/isaacsim.robot_setup.collision_detector/isaacsim/robot_setup/collision_detector/widget.py#L943-L967)）
- 本体：[physicsUtils.py:1607-1688](../../extscache/omni.physx-110.4.4+110.4.0.wx64.r.cp312.u7f4/omni/physx/scripts/physicsUtils.py#L1607-L1688)
- API の説明（[_physx.pyi:1318-1330](../../extscache/omni.physx-110.4.4+110.4.0.wx64.r.cp312.u7f4/omni/physx/bindings/_physx.pyi#L1318-L1330)）は「The contact buffer data are available for one simulation step」の 1 行だけで、「1 ステップ」がサブステップか呼び出し 1 回か、読むと消えるかは書かれていない。C++ ヘッダは同梱されていないので、以下はすべて実測による。

### 2.2 バッファの寿命（実測から組み立てた図）

```text
時間 →
         step k の simulate   step k の fetch_results              step k+1 の fetch_results
 ───────────┬────────────────────┬───────────────────────────────────────┬──────────
            │                    │ ① PhysX が接触を計算                  │ ① …
            │                    │ ② 報告バッファを step k の分で作り直す   │ ② step k+1 の分で上書き
            │                    │ ③ push 購読者のコールバックを同期で呼ぶ  │ ③ …
            │                    │                                       │
  pull の結果: step k-1 の内容    │   step k の内容（何度呼んでも同じ。      │ step k+1 の内容
  （simulate 後も変わらない）     │   USD を書き換えても、detach しても同じ） │
```

- ②は読み手がいなくても毎回行われる（C-01）。報告するものが無いステップでは空になる（P-04 step3）。
- `update_simulation` が 1 回で 2 サブステップ走るときは、①〜③がサブステップごとに 2 回行われる。pull で見えるのは 2 回目の②の結果だけ（S-03）。push は 2 回届く（S-04）。
- pull の戻り値は `(ContactEventHeaderVector, ContactDataVector)` の 2 要素タプル（docstring にある friction anchors は入っていない。P-01 の `tuple_len=2`）。各ヘッダの `contact_data_offset` と `num_contact_data` が、データ配列のどの範囲がそのペアの接触点かを示す。LOST のヘッダは接触点 0 個。

## 3. 検証方法

### 3.1 実行環境

- 描画と UI を外した軽量エクスペリエンス [crb_verify.kit](code/crb_verify.kit) で、headless の `SimulationApp` を GUI とは別のプロセスとして起動する（前回までの検証と同じ方式）。
- PhysicsScene は CPU・MBP、`timeStepsPerSecond` = 60（既定）、`/persistent/simulation/minFrameRate` = 30（S-00）。
- ステップの進め方は 2 通り（[crblib.py](code/crblib.py) の `Stepper`）：
  - `simfetch`：検出器と同じ `attach_stage` → `simulate` → `fetch_results`
  - `update`：`force_load_physics_from_usd` → `update_simulation`（Play 中に Kit が使う経路に近い）

### 3.2 基本シーン（[crblib.make_toggle_scene](code/crblib.py)）

```text
     /World/Box  動的立方体 1 m、ContactReportAPI(threshold=0)
     ┌──────┐
     │      │
     └──────┘   ← 隙間 1 mm（threshold=0 なので接触として報告される）
 ━━━━━━━━━━━━━━━━  /World/Ground  静的な床（上面 z=0）
 重力 0 なので立方体は動かない。USD の translate を z=10 に書き換えると離れ（LOST）、戻すと再接触（FOUND）する
```

テレポートのスケジュール（b01 の `SCHEDULE`）：step2 の直前に離す、step4 の直前に戻す。

### 3.3 スクリプト

| スクリプト | 内容 |
| --- | --- |
| [b01_pull_semantics.py](code/b01_pull_semantics.py) | pull の冪等性、コピーか参照か、何ステップ分残るか、差し替えのタイミング、detach、push との併用、update_simulation 経路 |
| [b02_substep.py](code/b02_substep.py) | 1 回の呼び出しが複数サブステップに分かれたときに何が残るか |
| [b03_per_step_cost.py](code/b03_per_step_cost.py) | 誰も読まなくてもバッファが作られているか（時間で判定）、スリープの影響 |
| [b04_detector_path.py](code/b04_detector_path.py) | `get_initial_collider_pairs()` を PhysX の状態を変えて呼ぶ |
| [b05_sleep.py](code/b05_sleep.py) | 静止シーンで報告が止まる現象を push と pull で並べて測る。wake_up / put_to_sleep で原因を確かめる |

再現：`powershell -File code\run.ps1`（引数なしで b*.py を全部、引数でスクリプト指定）。

## 4. 結果

### 4.1 pull は何を返すか（b01）

#### P-01 ステップ前の pull

| タイミング | pull の結果 |
| --- | --- |
| attach 前 | 空 |
| attach 直後（ステップ前） | 空 |
| 1 ステップ後 | `FOUND Box-Ground`（接触点 4） |

**P-02 同じステップ内で 5 回 pull**：5 回とも同じ。接触点の位置・法線・separation まで一致した（`all_equal_including_contact_points: true`）。タプルもヘッダ配列も毎回別の Python オブジェクトだった。

#### P-03 コピーか参照か

| 操作 | 結果 | 読み取れること |
| --- | --- | --- |
| 返ってきた 2 つの配列を `clear()` してから、もう一度 pull | 長さは (1, 4) のまま | 戻り値を書き換えても内部バッファは変わらない |
| 戻り値を保持したままステップを進める（LOST のステップ） | 保持していたオブジェクトは `PERSIST` のまま、新しい pull は `LOST` | 戻り値は内部バッファへのビューではなく、呼び出し時点のコピー |

#### P-04 毎ステップ pull（基準）

| step | 0 | 1 | 2（直前に離す） | 3 | 4（直前に戻す） | 5 | 6 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| pull | FOUND | PERSIST | LOST | （空） | FOUND | PERSIST | PERSIST |

**P-05 1 回だけ pull する**（実行ごとに 1 回だけ。ほかのステップでは読まない）

| pull したステップ | 結果 | P-04 の基準 |
| --- | --- | --- |
| step1 | PERSIST | PERSIST |
| step2 | LOST | LOST |
| step3 | **空** | 空 |
| step6 | PERSIST（1 件） | PERSIST |

step3 で pull しても step2 の LOST は見えない。step6 で pull しても、それまでの FOUND・LOST は溜まっていない。**バッファは直近 1 ステップ分だけで、読まれなかった分は次のステップで捨てられる。**

#### P-06 差し替えのタイミング

| タイミング | pull の結果 |
| --- | --- |
| step1 の後 | PERSIST |
| USD で立方体を離した直後（simulate 前） | PERSIST |
| `simulate()` の後、`fetch_results()` の前 | PERSIST |
| `fetch_results()` の後 | **LOST** |

バッファが新しくなるのは `fetch_results` の時点である。

#### P-07 detach / 再 attach

| タイミング | pull の結果 |
| --- | --- |
| detach 前 | PERSIST |
| `detach_stage()` 後 | **PERSIST（残っている）** |
| 再 `attach_stage()` 直後（ステップ前） | PERSIST（古いまま） |
| 再 attach 後 1 ステップ | FOUND（作り直されたので初回接触扱い） |

detach してもバッファは消えない。PhysX のオブジェクトが無い状態でも、最後のステップの内容が返る。

**P-08 push 購読と併用**：全ステップで、コールバックは `fetch_results` の中で 1 回呼ばれ（`callback_inside_fetch_results: true`）、その内容は直後の pull と完全に一致した。push が先に受け取っても pull の内容は減らない（消費し合わない）。報告が空の step3 ではコールバックは呼ばれなかった。最後のステップの後に `app.update()` を回しても、追加のコールバックは 0 回だった。

**P-09 update_simulation 経路**：P-04、P-05 と同じ列になった（1 回の呼び出しが 1 サブステップの場合）。

### 4.2 サブステップ（b02）

**S-01 1 回の呼び出しで何サブステップ走るか**（静止から自由落下させた落下量を g·DT² で割った値。DT = 1/60 s）

| elapsed | simfetch：ステップイベント（dt） | simfetch：落下量 | update：ステップイベント | update：落下量 |
| --- | --- | --- | --- | --- |
| 1/60 | 1 回（1/60） | 1.0 | 1 回 | 1.0 |
| 2/60 | 1 回（2/60） | 4.0 | 2 回（1/60 ×2） | 3.0 |
| 3/60 | 1 回（3/60） | 9.0 | 2 回 | 3.0 |
| 4/60 | 1 回（4/60） | 16.0 | 2 回 | 3.0 |
| 8/60 | 1 回（8/60） | 64.0 | 2 回 | 3.0 |

DT を n 回に分けたなら落下量は n(n+1)/2（1, 3, 6, 10, …）、k·DT の 1 ステップなら k²（1, 4, 9, 16, …）になる。simfetch は k² に一致したので分割しない。update は 2 回で頭打ちになり、残りの時間は進まなかった。

**S-02 接触し続けている状態で 1 回進める**：update で 2 サブステップ走った場合も、ヘッダは PERSIST 1 件だけだった（溜まらない）。

#### S-03 / S-04 1 回の呼び出しの中でイベントが起きる場合

秒速 30 m で薄い壁の横を 1 mm の隙間でかすめる立方体（1 サブステップで 0.5 m 進む）。1 サブステップずつ進めた列を正解とし、2 サブステップずつ進めた場合と比べた。開始位置を 0.5 m ずらした 2 通りで測った。

開始 x = −2.5 m：

| サブステップ | 4 | 5 | 6 | 7 |
| --- | --- | --- | --- | --- |
| 正解（1 つずつ） | FOUND | PERSIST | PERSIST | LOST |
| update 2 つずつ：push | FOUND, PERSIST | ← 同じ呼び出し | PERSIST, LOST | ← 同じ呼び出し |
| update 2 つずつ：pull | **PERSIST のみ** | | **LOST のみ** | |

開始 x = −3.0 m：

| サブステップ | 4 | 5 | 6 | 7 | 8 | 9 |
| --- | --- | --- | --- | --- | --- | --- |
| 正解（1 つずつ） | （空） | FOUND | PERSIST | PERSIST | LOST | （空） |
| update 2 つずつ：push | FOUND | | PERSIST, PERSIST | | LOST | |
| update 2 つずつ：pull | FOUND | | PERSIST（1 件） | | **空** | |

pull は呼び出しの最後のサブステップの内容だけを返す。x = −2.5 m では FOUND が、x = −3.0 m では LOST が pull から消えた。push はすべて受け取っている。

### 4.3 毎ステップ作られているか（b03）

**C-01 1 ステップ（simulate + fetch_results）の時間**（中央値。2 周の平均。立方体は全部床に接触し続け、スリープさせない）

| 条件 | N=100 | N=900 | N=2,500 |
| --- | --- | --- | --- |
| no_api（ContactReportAPI なし） | 0.575 ms | 2.997 ms | 8.807 ms |
| api_no_read（API あり、誰も読まない） | +0.034 ms | **+0.650 ms** | **+1.948 ms** |
| api_pull（毎ステップ pull、pull の時間は別） | +0.043 ms | +0.692 ms | +1.979 ms |
| api_subscribe（何もしない push 購読） | +0.051 ms | +0.794 ms | +2.172 ms |
| pull 1 回の時間（api_pull） | 0.012 ms | 0.056 ms | 0.164 ms |
| 210 ステップ後に 1 回だけ pull したヘッダ数 / 接触点数 | 100 / 400 | 900 / 3,600 | 2,500 / 10,000 |

- 誰も読まなくても、API があるだけで N=2,500 では約 1.95 ms（約 0.78 µs/ペア）増えた。その増分は pull・購読の有無でほとんど変わらない。**バッファは読まれるかどうかに関係なく、`fetch_results` の中で毎ステップ作られている。**
- 210 ステップ回した後のヘッダ数はちょうど N 件（1 ステップ分）だった。210 × N 件は溜まっていない。
- 計測は 1 台の Windows 機での実測で、絶対値は環境によって変わる。読み取るべきは条件間の差である。

**C-02 スリープの影響**（既定のスリープ設定、静止した立方体 9 個）

| step | ヘッダ数 | 種類 |
| --- | --- | --- |
| 0 | 9 | FOUND |
| 1〜23 | 9 | PERSIST |
| 24〜119 | **0** | （LOST も出ない） |

最後の時点で立方体はスリープしていた（`is_sleeping: true`）。スリープしたペアは接触が計算されなくなり、報告から黙って消える。

### 4.4 検出器の経路（b04）

シーン：ロボットの代わりに 3 リンク（A と B は隙間 1 mm で接触、C は離れている）と、1 m/s で上昇し続ける剛体 Mover（PhysX の時間が進んだかを見るため）。重力 0。

| ケース | 呼ぶ前のバッファ | 関数が見たバッファ | 返ったペア | 呼んだ後の状態 |
| --- | --- | --- | --- | --- |
| D-01 Stop（未アタッチ） | — | FOUND B-A | **A-B** | 一時 API・サブレイヤーは消えている |
| D-02 IPhysxSimulation アタッチ済み・3 ステップ済み・API なし | 空 | **空** | **なし** | Mover：PhysX 0.0500 → 0.0667、USD 0.0500 のまま |
| D-03 同上・API が最初から付いている | PERSIST B-A | **PERSIST B-A** | **なし** | 同上 |
| D-04 Play → Pause（API なし / あり） | 空 / PERSIST | FOUND B-A | **A-B** | PhysX の剛体オブジェクトが消えた（取得するとエラー）。再開 10 フレーム後の Mover は 0.3333 m（呼ぶ前の 0.1667 m から 10 フレーム × 1/60 m 進んだ値）で、動き続けていた |
| D-05 Play 中（API なし / あり） | 空 / PERSIST | FOUND B-A | **A-B** | D-04 と同じ |

- D-04 / D-05：Play 中・Pause 中も IPhysxSimulation の `get_attached_stage()` はこのステージを指していなかった（`attached_stage_matches: false`）。そのため関数は自分でアタッチし、最初のステップで FOUND が出た。
- D-02 / D-03：アタッチ済みなので関数はアタッチし直さない。D-03 では最初から API が付いていたので報告は PERSIST になり、FOUND だけを拾うフィルタで捨てられた。D-02 では何も報告されなかった。その理由が D-06。

#### D-06 アタッチ済み・接触中のペアに、後から API を付ける

| タイミング | pull の結果 |
| --- | --- |
| API を付けた後の 1〜4 ステップ | すべて空 |
| B を離した後 | 空（LOST も出ない） |
| B を戻した後 | **FOUND B-A** |

すでに存在する接触ペアには、後から付けた ContactReportAPI は効かない。ペアが一度消えて作り直されると報告されるようになる。（PhysX がペアを作る時点で報告するかどうかを決めているため、と考えられる。この部分は推論。）

### 4.5 静止シーンで報告が止まる現象：push と pull の比較（b05）

push 購読で「PERSIST が何回か来たあと来なくなる」現象が pull でも起きるかを調べた。同じ実行の中で、ステップごとに push（コールバック）・pull・`is_sleeping` を同時に記録した。シーンは床の上の立方体 1 個。

#### Z-01 4 通りの比較（80 ステップ、simfetch）

| 条件 | push | pull | スリープ状態 |
| --- | --- | --- | --- |
| 重力 0・スリープ許可（既定） | FOUND → PERSIST（step1〜23）→ **step24 から無し** | push と同じ | step24 から寝ている |
| 重力 9.81・スリープ許可（既定） | FOUND → PERSIST（step1〜23）→ **step24 から無し** | push と同じ | step24 から寝ている |
| 重力 0・スリープ禁止（sleepThreshold=0） | FOUND → PERSIST が 80 ステップ目まで続く | push と同じ | 一度も寝ない |
| 重力 9.81・スリープ禁止 | FOUND → PERSIST が 80 ステップ目まで続く | push と同じ | 一度も寝ない |

4 通りとも、push と pull は全ステップで一致した（`push_equals_pull_every_step: true`）。報告が止まったステップは、剛体がスリープしたステップと一致した。

#### Z-02 寝たあとに起こす（重力あり、step50 の直前に `wake_up`）

| step | push / pull | スリープ状態 |
| --- | --- | --- |
| 0 | FOUND | 起きている |
| 1〜23 | PERSIST | 起きている |
| 24〜49 | 無し | 寝ている |
| 50〜73 | **PERSIST**（FOUND ではない） | 起きている |
| 74〜79 | 無し | また寝た |

起こすと PERSIST から再開した。寝ているあいだも PhysX はペアを「接触中」として覚えている。そのため寝るときに LOST は出ず、起きたときにも FOUND は出ない。

#### Z-03 起きている剛体を強制的に寝かせる（重力 0、step5 の直前に `put_to_sleep`）

PERSIST は step4 まで来て、step5 から push も pull も止まった。止まったのはスリープさせたステップちょうどで、24 ステップ待つ必要はなかった。報告が止まる原因はスリープである。

#### Z-04 タイムライン Play 中（重力あり。1 フレーム = 1 物理ステップ）

Play を始めたフレームで FOUND が出たあと、PERSIST が 23 フレーム続き、24 フレーム目からは push も pull も来なくなった。スリープしたフレームと一致しており、simfetch の場合と同じ挙動だった。

#### 読み取れること

- 静止したシーンで物理ステップだけを進めると、**約 24 ステップ（既定の sleepThreshold = 5e-5 のとき）で剛体が寝て、pull しても何も返らなくなる**。push で起きる現象と同じものである。
- push も pull も、PhysX が 1 ステップごとに作る同じ報告を受け取る口にすぎない（P-08）。報告が作られなければ、どちらにも何も来ない。
- 寝ているあいだは接触が続いていても報告されず、LOST も出ない。「いまどのペアが接触しているか」を報告だけで追う場合は、FOUND で集合に加えて LOST で外す。報告が来なくなったことを「離れた」と扱ってはいけない。毎ステップの報告が必要なら、`physxRigidBody:sleepThreshold = 0` でスリープを止める（Z-01、ただし計算は増える）。
- 検出器（`get_initial_collider_pairs`）は、アタッチした直後の 1 ステップしか見ない。その時点の剛体は起きているので、スリープの影響は受けない（D-01）。

## 5. 判定のまとめ

| 主張 | 判定 | 根拠 |
| --- | --- | --- |
| pull はバッファを読みに行っている | 正 | P-02, P-03, P-06 |
| 読むとバッファが消える（消費型） | **誤** | P-02（5 回同じ）、P-08（push と消費し合わない） |
| 戻り値は内部バッファのビュー | **誤（コピー）** | P-03 |
| 複数ステップ分キャッシュされる | **誤（1 サブステップ分）** | P-05, S-03 |
| ステップを進めなければ何度でも同じ情報が取れる | 正 | P-02, P-06。detach しても残る（P-07） |
| 「1 ステップ」＝ `update_simulation` 1 回 | **誤（PhysX の 1 サブステップ）** | S-03, S-04 |
| 毎物理ステップ ContactReport のバッファが作られる | 正（API 付き・スリープしていないペアについて） | C-01, C-02 |
| 静止シーンで PERSIST が止まるのは push だけの現象 | **誤（pull も同じステップで止まる）** | Z-01, Z-04。原因はスリープ（Z-03）。起こすと PERSIST から再開する（Z-02） |
| 検出器は PhysX の状態によらず正しくペアを取れる | **条件付き** | Stop / Play / Pause なら取れる（D-01, D-04, D-05）。IPhysxSimulation にアタッチ済みだと取りこぼす（D-02, D-03） |

## 6. 未確認・推論の部分

- バッファの実体（C++ 側でどう確保・再利用しているか）は、ヘッダもソースも同梱されていないので直接は見ていない。「毎ステップ作り直している」は時間（C-01）と中身（P-05, C-01 のヘッダ数）からの判断である。メモリを確保し直しているのか、同じ領域を使い回しているのかは区別できていない。
- `update_simulation` の最大サブステップ数が `timeStepsPerSecond / minFrameRate` で決まるというのは、60 / 30 = 2 という 1 組の観測からの推論である。
- 同じステップの 2 回目以降の pull が 1 回目より速い理由（§1 の 4）は未確認。
- D-04 / D-05 で PhysX オブジェクトが USD から作り直された後、USD に書かれない内部状態（ジョイントの内部状態、スリープ状態など）がどうなるかは調べていない。
- GPU dynamics（`enableGPUDynamics=True`）では調べていない。すべて CPU で測った。
