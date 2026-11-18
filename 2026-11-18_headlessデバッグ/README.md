# headless の Isaac Sim を VSCode のデバッガで止める（Windows）

2026-11-18

画面を出さない headless の Isaac Sim でも、Python のコードなら VSCode のブレークポイントで止められる。
Isaac Sim の起動のしかたによって、やり方が 2 通りある。

| やり方 | 起動のしかた | VSCode の操作 | 止められるコード |
|---|---|---|---|
| **A** | `SimulationApp({"headless": True})` を使う単体スクリプト | F5 で起動する（launch） | そのスクリプト（試したのは `a_simulation_app.py` だけ） |
| **B** | Isaac Sim 本体を `--no-window` で起動する | 起動している Isaac Sim に後から接続する（attach） | `--exec` で渡したスクリプト。拡張機能のコードも同じ仕組みで止まるはずだが、試していない |

2026-11-18 に下の環境で、ブレークポイントで止まることを確かめた。
確かめたのは、A は `a_simulation_app.py`、B は `--exec` で渡した `b_kit_exec.py` の 2 つ。

- Windows 11 / Isaac Sim 6.2.0（インストール先 `C:\isaacsim-6.2.0`）
- VSCode の拡張機能: Python（ms-python.python 2026.10.0）と Python Debugger（ms-python.debugpy 2026.6.0）
- B で使う Isaac Sim 側の拡張機能: `omni.kit.debug.python` 1.0.2（Isaac Sim に入っている）

## フォルダの中身

| ファイル | 中身 |
|---|---|
| `.vscode/launch.json` | VSCode の起動設定。「A: SimulationApp (headless)」と「B: Attach (port 3000)」の 2 つ |
| `.vscode/settings.json` | このフォルダで使う Python の場所（Isaac Sim 同梱の Python） |
| `a_simulation_app.py` | A 用のスクリプト。headless で箱を落とし、60 フレームの間、高さ z を表示する |
| `b_kit_exec.py` | B 用のスクリプト。Isaac Sim の中で 30 フレーム動かし、終わると Isaac Sim を終了させる |
| `run_b.ps1` | B 用の起動スクリプト。デバッガの接続を待つ設定を付けて、Isaac Sim を headless で起動する |
| `reference/isaacsim_install_launch.jsonc` | Isaac Sim のインストールフォルダに付いてくる `.vscode/launch.json` の写しに、項目ごとの説明を書き足したもの |

## 使う前に書き換えるところ

Isaac Sim をインストールしたフォルダが `C:\isaacsim-6.2.0` でなければ、書き換える。
インストールしたフォルダとは、`isaac-sim.bat` と `python.bat` が入っているフォルダのこと。

書き換えるのは、`C:/isaacsim-6.2.0` という文字列の部分だけ。後ろに続く `/kit/python/python.exe` などはそのまま残す。
例: インストール先が `D:\isim\isaac-sim-6.2.0` なら、次のように書き換える。

```
"python": "C:/isaacsim-6.2.0/kit/python/python.exe",      ← 書き換える前
"python": "D:/isim/isaac-sim-6.2.0/kit/python/python.exe", ← 書き換えた後
```

| ファイル | `C:/isaacsim-6.2.0` が出てくる所 | 数 |
|---|---|---|
| `.vscode/launch.json` | `cwd`、`env` の `CARB_APP_PATH`・`ISAAC_PATH`・`EXP_PATH`・`PYTHONPATH`、`python`、`debugAdapterPython`、`debugLauncherPython` | 8 か所 |
| `.vscode/settings.json` | `python.defaultInterpreterPath` | 1 か所 |
| `run_b.ps1` | `$IsaacSimRoot` の既定値（`C:\isaacsim-6.2.0`）。書き換えずに、起動するとき `-IsaacSimRoot D:\isim\isaac-sim-6.2.0` のように渡してもよい | 1 か所 |

パスの区切りの書き方:

- JSON（launch.json、settings.json）の中では、`/` か `\\` を使う。`\` を 1 個だけ書くと不正な書式になる。
- PowerShell（run_b.ps1 とその引数）では、`\` 1 個でよい。

ポート番号を変える場合（既定は 3000）は、次の 2 つを同じ値にする。

- `run_b.ps1` の `$Port`（または起動時に `-Port 3005` のように渡す）
- `.vscode/launch.json` の B の `"port"`

`"name"` の「B: Attach (port 3000)」は、プルダウンに出る表示名にすぎない。ポートを変えても動作には関係ない。分かりやすさのために変えてもよい。

## 準備

1. VSCode に Python 拡張と Python Debugger 拡張を入れる。Python 拡張を入れると、Python Debugger 拡張も一緒に入る。
2. VSCode の「ファイル → フォルダーを開く」で、**このフォルダ**を開く。`.vscode` の設定は、開いたフォルダの直下にあるものだけが読まれる。

## やり方 A: SimulationApp のスクリプトを F5 で起動する

1. `a_simulation_app.py` を開き、`print(f"frame=...` の行の左端をクリックして、ブレークポイント（赤い丸）を付ける。
2. 左の「実行とデバッグ」を開き、上のプルダウンで「A: SimulationApp (headless)」を選んで F5 を押す。
3. 起動に数十秒〜1 分かかる。そのあと、ブレークポイントの行で止まる。左の変数の欄で、`frame` と `z` の値が見える。
   2 分たっても止まらないときは、VSCode のターミナルに `Traceback` などのエラーが出ていないかを見る。
   何も出ずに終わったときは、下の「つまずいたところ」を見る。

自分のスクリプト（例: `C:\work\my_sim.py`）で使うときは、`launch.json` の `"program"` をそのスクリプトのパスにする（例: `"C:/work/my_sim.py"`）。
`"${file}"` と書くと、エディタでいま開いているファイルが起動される。

- VSCode で開くフォルダは、このフォルダのままでよい。自分のスクリプトがあるフォルダを開いて使いたいときは、この `.vscode` フォルダをそちらにコピーする。
- `cwd` は Isaac Sim のフォルダのままにしておく。スクリプトの中で相対パスでファイルを読むなら、`cwd` をそのスクリプトのフォルダに変えてもよい。

## やり方 B: headless で起動した Isaac Sim に接続する

1. `b_kit_exec.py` を開き、`print(f"frame=...` の行にブレークポイントを付ける。
2. VSCode のターミナル（PowerShell）を開き、`cd` でこのフォルダに移ってから `run_b.ps1` を実行する。
   ```powershell
   .\run_b.ps1
   ```
3. ログの最後に `Waiting for debugger to connect (timeout: 30s)...` と出て止まったら、「B: Attach (port 3000)」を選んで F5 を押す。
   B の F5 は、Isaac Sim を起動するのではなく、動いている Isaac Sim に接続する操作。
4. ブレークポイントの行で止まる。F5（続行）を押すたびに次のフレームへ進み、30 フレーム目のあと Isaac Sim は自分で終了する。

**待ち時間は既定で 30 秒**（`-WaitSeconds` で変えられる）。`Waiting for debugger to connect` が表示されてから待ち時間のうちに接続しないと、Isaac Sim は接続を待つのをやめ、スクリプトを最後まで実行して終了する。

- 「接続を待つ」設定は、`run_b.ps1` が `--/exts/omni.kit.debug.python/waitForClient=true` を付けて有効にしている。
- 待つ秒数は、`-WaitSeconds` に渡した値が `--/exts/omni.kit.debug.python/waitTimeout=` として Isaac Sim に渡される。

- 遅れて接続したときに止まるかどうかは、試していない。スクリプトがブレークポイントの行をもう通り過ぎていれば、止まらない。Isaac Sim がすでに終了していれば、接続もできない。
- 確実に止めたいときは、`.\run_b.ps1 -WaitSeconds 120` のように長めの秒数を渡す。`-1` を渡すと、接続するまで無限に待つ。

`run_b.ps1` のファイルの末尾は、次の 2 行になっている。

```powershell
    "--/exts/omni.kit.debug.python/waitTimeout=$WaitSeconds" `
    --exec "$script"
```

自分の `--exec` スクリプトで使うときは、2 行目の `--exec "$script"` を `--exec "C:/work/my_script.py"` のように書き換える。

自作の拡張機能（例: `my.ext`）のコードを止めたいときは、2 行目の `--exec "$script"` を消して、代わりに `--enable my.ext` と書く。
書き換えた後の末尾の 2 行は、次のようになる。

```powershell
    "--/exts/omni.kit.debug.python/waitTimeout=$WaitSeconds" `
    --enable my.ext
```

そのうえで、拡張機能の .py ファイルにブレークポイントを付けて接続する。
自作の拡張機能を止めるこの使い方は、試していない。接続の仕組みは `--exec` のときと同じなので、止まるはず。
ただし、拡張機能の起動時（`on_startup`）の処理は、接続を待っている間に終わっているかもしれない。
その場合は、起動時の処理では止まらない。
`--exec` を外すと、Isaac Sim は自分では終了しない。終わらせるときは、ターミナルで Ctrl+C を押すか、`kit.exe` を終了させる。

## つまずいたところ

2026-11-18 に試したときに起きた問題と、その対処。
対処はこのフォルダのファイルに入れてある。ほかの PC にコピーして同じ症状が出たときに読む。

### A で F5 を押すと、何も出ずに「デバッグが停止しました」で終わる

**原因**: デバッガ本体（VSCode とプログラムの間を取り持つ部分）を動かす Python の場所が見つからない。

- VSCode のデバッガ本体は、launch.json の `"python"` ではなく、VSCode のユーザー設定 `python.defaultInterpreterPath` の Python で動く。
- 今回は、ユーザー設定がもう存在しない古いパス（`python.bat`）を指していた。そのため、デバッガ本体を起動できなかった。
- 拡張機能のログ（VSCode の「出力」パネル → 「Python Debugger」）には、次のように出ていた。
  ```
  legacyResolveEnvironment: Resolving environment '<存在しないパス>\python.bat'
  createDebugAdapterDescriptor: python command parts=''
  ```

**対処**（このフォルダではどちらも済ませてある）:

- `launch.json` に `"debugAdapterPython"` と `"debugLauncherPython"` を書き、Isaac Sim 同梱の `python.exe` を指す。
  `debugAdapterPython` はデバッガ本体を動かす Python、`debugLauncherPython` はスクリプトを起動する部品を動かす Python。
  どちらも書かなければ、ユーザー設定の Python が使われる。
- `.vscode/settings.json` に `python.defaultInterpreterPath` を書き、このフォルダで使う Python を指定する。
  VSCode では、開いたフォルダの `.vscode/settings.json` の値が、ユーザー設定の同じ名前の値より優先される。

それでも直らないときは、次を試す。

1. コマンドパレット（Ctrl+Shift+P）で「Developer: Reload Window」を実行し、VSCode に設定を読み直させる。
2. コマンドパレットの「Python: Select Interpreter」で、`<Isaac Sim のフォルダ>\kit\python\python.exe` を選ぶ。

### B で Isaac Sim が終了しない

**原因**: スクリプトの中でステージに立方体を足しているので、ステージは「未保存の変更あり」になっている。

- この状態で `omni.kit.app.get_app().post_quit()` を呼ぶと、`omni.kit.window.file` 拡張が終了を取り消し、「保存しますか？」の確認を出す。
- headless では画面が無いので、この確認に答えられない。そのため、Isaac Sim はいつまでも終了しない。
- デバッガを接続していなくても、同じように終了しなかった。

**対処**: `post_quit()` の前に、次の設定を `true` にする（`b_kit_exec.py` では済ませてある）。
`true` にすると、未保存の変更を捨てて終了する。

```python
carb.settings.get_settings().set("/app/file/ignoreUnsavedOnExit", True)
```

### B で `Can't listen for client connections: [WinError 10048]` と出る

**原因**: ポート 3000 を、ほかのプロセスが使っている。

前に起動した headless の Isaac Sim が終了せずに残っていると、このエラーが出る。
今回は、次の 2 つのプロセスが残っていた。

- `kit.exe`（Isaac Sim 本体）
- その子プロセスの `python.exe`（接続を受け付ける debugpy の部品）。コマンドラインに `debugpy/adapter` が含まれている

headless の Isaac Sim は画面が無いので、残っていても気づきにくい。

**対処**: 次のどちらかを行う。

- 残っているプロセスを 2 つとも終了させる。
  - タスクマネージャーの「詳細」タブに「コマンドライン」列を出して探す。
  - `kit.exe` のうち、コマンドラインに `--no-window` が付いているものが headless で起動したもの。
  - `python.exe` のうち、コマンドラインに `debugpy/adapter` が含まれているものがその子プロセス。
  - `kit.exe` だけを終了させたとき、子の `python.exe` も一緒に終わるかは確かめていない。
- 別のポートを使う（`.\run_b.ps1 -Port 3001` と、launch.json の B の `"port": 3001`）。

ポートを使っている相手が Isaac Sim 以外のソフトの場合も、同じエラーになる。そのときは別のポートを使う。

## launch.json に書いた内容の出典

| 項目 | 出典 |
|---|---|
| A の設定の元になった内容（`program`、`console`、`env`、`python`） | Isaac Sim のインストールフォルダの `.vscode/launch.json` の「Python: Current File」。この写しに説明を書き足したものが `reference/isaacsim_install_launch.jsonc` |
| `env` の `CARB_APP_PATH` / `ISAAC_PATH` / `EXP_PATH` | `python.bat` がこの 3 つを同じ値で設定している |
| `env` の `PYTHONPATH=<Isaac Sim>/site` | `setup_python_env.bat`（`python.bat` から呼ばれる）。`site/sitecustomize.py` が `exts/isaacsim.simulation_app` などを import の探索先に足す |
| `env` の `RESOURCE_NAME` | インストールフォルダの `.vscode/launch.json` に書かれている。インストール内の .py と .bat には、この値を読む箇所は見つからなかった |
| `CARB_APP_PATH` と `EXP_PATH` の使われ方 | `exts/isaacsim.simulation_app/isaacsim/simulation_app/app_framework.py`（`CARB_APP_PATH` から Kit 本体の場所を決める）と `simulation_app.py`（docstring に `$EXP_PATH` の `.kit` を読むと書いてある） |
| `"type": "debugpy"`、`python`、`cwd`、`env`、`console`、`justMyCode`、`connect` | Python Debugger 拡張（ms-python.debugpy 2026.6.0）の `package.json` の、設定項目の説明 |
| `"type": "python"` は古い書き方 | Python 拡張（ms-python.python 2026.10.0）の `package.json` に、「間もなく非推奨。`python` を `debugpy` に置き換えて」という案内がある |
| `debugAdapterPython` / `debugLauncherPython` | Python Debugger 拡張の `dist/extension.js` の `getDebugAdapterPython` と `setDebugComponentPythonPaths`。この 2 つが無いと、ユーザー設定の Python がデバッガ本体に使われる。上の「つまずいたところ」のログとも合う |
| B の `port: 3000` と、`run_b.ps1` の `port` / `waitForClient` / `waitTimeout` / `host` | `extscache/omni.kit.debug.python-1.0.2+.../config/extension.toml` の `[settings.exts."omni.kit.debug.python"]`。既定値は host=127.0.0.1、port=3000、waitForClient=false、waitTimeout=30。waitTimeout の説明に「0 = 待たない、-1 = 無限に待つ」とある |
| `run_b.ps1` の `--exec`、`--enable`、`--/設定名=値` | `kit\kit.exe --help` |
| `run_b.ps1` の `--no-window` | `kit.exe --help` には載っていない。Kit に付いてくる `kit/apps/omni.app.full.kit` と、`kit/dev/templates/apps/kit_base_editor/README.md` に使用例がある |
| `isaac-sim.bat` が起動するもの | `isaac-sim.bat` の最終行。`kit\kit.exe` に `apps/isaacsim.exp.full.kit` を渡している |
| `/app/file/ignoreUnsavedOnExit` | `extscache/omni.kit.window.file-2.0.17+.../omni/kit/window/file/file_window.py` の `_on_post_quit` |

## 確かめた結果（2026-11-18）

| 確かめたこと | 結果 |
|---|---|
| A をデバッガなしで実行する | 60 フレームまで動いて終了した |
| A を VSCode の F5 で実行する（`debugAdapterPython` を足した後） | ブレークポイントで止まった |
| B を VSCode から接続して実行する | ブレークポイントで止まった |
| B を接続せずに最後まで実行する（終了処理を直した後。`-WaitSeconds 5` で接続を 5 秒だけ待たせた） | 待ち時間の 5 秒のあと 30 フレーム動き、自分で終了した。`run_b.ps1` の起動から終了まで約 30 秒 |
| B に接続したまま、30 回止めては続行して最後まで実行する（終了処理を直した後） | 30 回止まり、そのあと自分で終了した。接続には、VSCode と同じ手順で接続・ブレークポイント設定・続行を行う自作のテスト用スクリプトを使った |
| B を終了処理を直す前に実行する | 接続してもしなくても、30 フレームのあと終了しなかった |
