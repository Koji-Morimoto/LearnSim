---
type: moc
created: 2026-11-08
updated: 2026-11-10
tags: [isaac-sim-6.2.0, moc, snippet]
---

# MOC-観測Snippet

`snippets/` の Python ファイルと、その解説ノートの一覧である。Snippet はすべて、Isaac Sim の Script Editor にそのまま貼って実行できる。Snippet の書き方の約束は [学習方針](../06_学習方針/01_学習方針.md) にある。

各ノートは「コード」→「観測手順と期待される出力」→「手順をコードで自動実行する」→「結果」の順に並んでいる。観測手順の「〇〇を実行する」は、そのノートの「コード」の節の全文を Script Editor で実行することを指す。期待される出力は、2026-11-09 に GUI とヘッドレス（GUI と同じ `isaacsim.exp.full.kit`）で実測した値である。測ったときのスクリプトと生データは [measurements/](measurements/) にある。

## 読む順序

番号は、[学習方針](../06_学習方針/01_学習方針.md) で Snippet を使う順に付けた。8番は4番の派生ノートで、Snippet のファイルを持たない。

| 順 | 解説ノート | ファイル | 目的 | 有効な状態 |
|---|---|---|---|---|
| 1 | [env_fingerprint](01_env_fingerprint.md) | [env_fingerprint.py](../snippets/env_fingerprint.py) | 設定値・拡張・状態の一覧（環境指紋）を出す。状態によって変わる設定と、その変え方 | すべて |
| 2 | [step_probe](02_step_probe.md) | [step_probe.py](../snippets/step_probe.py) | 物理ステップの前後のログと、「指定したステップ数の後に実行」の予約 | Play |
| 3 | [clock_probe](03_clock_probe.md) | [clock_probe.py](../snippets/clock_probe.py) | 描画フレームごとの時計の値と、lag（timeline が開始時刻から進んだ量 − シミュレーション時刻）が −2h 以上 −h 未満かの判定 | Play |
| 4 | [bench_probe](04_bench_probe.md) | [bench_probe.py](../snippets/bench_probe.py) | API の呼び出し時間の計測と基準値 | 計測対象による |
| 5 | [pose_freshness_probe](05_pose_freshness_probe.md) | [pose_freshness_probe.py](../snippets/pose_freshness_probe.py) | 4つの経路で剛体の高さを読み比べ、物理の出力先ごとにどの経路が最新値を返すかを見る | Play |
| 6 | [backend_spy](06_backend_spy.md) | [backend_spy.py](../snippets/backend_spy.py) | 呼び出しごとに選ばれたバックエンドとフォールバックの表示 | すべて |
| 7 | [strict_backend](07_strict_backend.md) | [strict_backend.py](../snippets/strict_backend.py) | strict の定義と、2種類の例外のデモ | すべて |
| 8 | [重い処理と軽くする方法](08_重い処理と軽くする方法.md) | （なし。コードはノートの中） | 重い処理の確かめ方、なくし方、軽くなったことの確かめ方と、結果の記入欄 | 項目による |
| 9 | [supported_backends](09_supported_backends.md) | [supported_backends.py](../snippets/supported_backends.py) | メソッドのソースを読んで、対応するバックエンドを一覧表示する（呼ばずに分かる） | すべて |
| 10 | [lab_parts](10_lab_parts.md) | [lab_parts.py](../snippets/lab_parts.py) | 何度も使う処理の部品集（シーン作成・状態の切り替え・物理の出力先・Stop のままの物理・設定を読む場所の検索） | すべて |

## Play / Pause / Stop をコードで切り替える

どのノートにも、そのノートの手順をコードで行う「手順をコードで自動実行する」の節がある。共通する決まりは次のとおりである（2026-11-09 に実測で確かめた）。

- `omni.timeline.get_timeline_interface()` の `play()`・`pause()`・`stop()` が、それぞれのボタンの代わりになる。
- これらは呼んだ時点では状態を変えず、次のメインループの更新で反映される。`play()` の直後に `is_playing()` を呼ぶと False が返る。
- `commit()` を呼ぶと、次の更新を待たずに、その場で反映される。Play の場合は、ウォームアップの2回の物理ステップと view の作成も `commit()` の中で終わる。
- 状態を切り替えてから結果を見るまでを1つのコードで行うには、`async` の関数の中で `await omni.kit.app.get_app().next_update_async()` を使って更新を待ち、その関数を `asyncio.ensure_future()` で走らせる。

## 追加するときの手順

1. `snippets/` に py ファイルを追加する。既存の Snippet を変えるときは、変更前の版を `C:\isaacsim-6.2.0\CodeByClaude\isaac-sim-backend-lab_old\` の下にフォルダごと残してから書き換える。py ファイルとノートのコードが食い違った場合は、ノートの側を正とし、py ファイルをノートに合わせる。
2. このフォルダに、最後の番号の次の番号を付けて解説ノートを追加する。目的、確かめるノート、メタ情報、コードの全文、観測手順と期待される出力、手順をコードで自動実行する方法、期待と違ったときに疑う箇所を、この順に書く。
3. 上の表に1行足す。

## 他のカテゴリ

- [README（ルート目次）](../README.md)

## 前後のノート

| 前 | このフォルダの目次 | 次 |
|---|---|---|
| [設定一覧](../04_設定/01_設定一覧.md) | [MOC-観測Snippet](00_MOC-観測Snippet.md) | [env_fingerprint](01_env_fingerprint.md) |
