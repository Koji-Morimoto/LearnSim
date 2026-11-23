"""Generate VOICEVOX 四国めたん（ノーマル） narration WAV clips + manifest.json with kana audit
for the Point Cloud Unified API & CPU<->GPU DrawCall / Data Transfer Explainer Video."""

from __future__ import annotations

import json
import os
import sys
import time
import urllib.parse
import urllib.request
import wave
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent
AUDIO_DIR = BASE_DIR / "audio"
MANIFEST_PATH = AUDIO_DIR / "manifest.json"
VOICEVOX_URL = os.environ.get("VOICEVOX_URL", "http://127.0.0.1:50021")
SPEAKER_NAME = "四国めたん"
STYLE_NAME = "ノーマル"
DEFAULT_SPEAKER_ID = 2

SEGMENTS = [
    {
        "id": "01_intro_api",
        "scene": "scene1_api_appearance",
        "phase_hints": ["init_create"],
        "title": "Scene 1: 統一関数 create_pointcloud と見た目調整",
        "subtitle": "全6種類の点群描画方式を1つの関数 create_pointcloud に統一！さらに CPU と GPU 間のデータ転送と DrawCall の仕組みを解説します。",
        "speech_text": "全六種類のてんぐん描画方式を、ひとつの関数、クリエイトポイントクラウドに統一しました！さらにシーピーユーとジーピーユー間のデータ転送と、ドローコールの仕組みを解説します。",
        "speed_scale": 1.13,
    },
    {
        "id": "02_s1_radius_color_pos",
        "scene": "scene1_api_appearance",
        "phase_hints": ["resize_color_move"],
        "title": "Scene 1: 点の大きさ・色・位置のリアルタイム変更",
        "subtitle": "set_point_radius で球の半径、set_colors で点ごとの色、set_world_offset で点群の位置を、どの描画方式でも同じ関数で自在に変更できます！",
        "speech_text": "セットポイントラディウスで球の半径、セットカラーズで点ごとの色、セットワールドオフセットでてんぐんの位置を、どの描画方式でもおなじ関数で自在に変更できます！",
        "speed_scale": 1.13,
    },
    {
        "id": "03_s1_glossy_shadow",
        "scene": "scene1_api_appearance",
        "phase_hints": ["glossy_toggle", "glossy_on", "shadow_off", "shadow_on"],
        "title": "Scene 1: テカリ（Roughness）と影（Cast Shadows）の切替",
        "subtitle": "さらに PBR マテリアルと連動し、set_appearance でツヤあり・マットのテカリ調整、set_cast_shadows で床への影のオンオフもワンクリックで切り替わります！",
        "speech_text": "さらにピービーアールマテリアルと連動し、セットアピアランスでツヤあり、マットのテカリ調整、セットキャストシャドウズで床への影のオンオフも、即座に切り替わります！",
        "speed_scale": 1.13,
    },
    {
        "id": "04_s2_naive_drawcalls",
        "scene": "scene2_drawcall_vs_batch",
        "phase_hints": ["naive_spheres"],
        "title": "Scene 2: 個別 UsdGeom.Sphere (N回の DrawCall ボトルネック)",
        "subtitle": "ここからは CPU と GPU 間の通信に注目！個別 Sphere 方式は 1点ごとに USD Prim を作るため、カメラを動かすたび CPU が数千回の DrawCall を発行して大渋滞します。",
        "speech_text": "ここからはシーピーユーとジーピーユー間の通信に注目です！個別スフィア方式は、一点ごとにユーエスディープリムを作るため、カメラを動かすたび、シーピーユーが数千回のドローコールを発行して大渋滞します。",
        "speed_scale": 1.13,
    },
    {
        "id": "05_s2_points_init_vs_cam",
        "scene": "scene2_drawcall_vs_batch",
        "phase_hints": ["usd_points_init", "usd_points_camera"],
        "title": "Scene 2: UsdGeom.Points (初回転送後は 64B 行列 + 1 DrawCall)",
        "subtitle": "一方 UsdGeom.Points は初回フレームだけ座標配列を GPU VRAM へ一括転送！以降はカメラを動かしても点群は一切送らず、64バイトのカメラ行列と 1 DrawCall だけです。",
        "speech_text": "一方、ユーエスディージオムポインツは、初回フレームだけ座標配列をジーピーユーブイラムへ一括転送します！以降はカメラを動かしてもてんぐんは一切送らず、六十四バイトのカメラ行列と、ワンドローコールだけです。",
        "speed_scale": 1.13,
    },
    {
        "id": "06_s2_instancer_rtcore",
        "scene": "scene2_drawcall_vs_batch",
        "phase_hints": ["point_instancer"],
        "title": "Scene 2: UsdGeom.PointInstancer (RTコア 1回インスタンシング)",
        "subtitle": "PointInstancer も同様に初回だけ GPU へ送り、毎フレームはカメラ行列と 1 DrawCall のみ。さらに RTX の RT コアが単一球メッシュをハードウェア高速化します！",
        "speech_text": "ポイントインスタンサーも同様に、初回だけジーピーユーへ送り、毎フレームはカメラ行列とワンドローコールのみ。さらにアールティーエックスのアールティーコアが、たんいつの球メッシュをハードウェア高速化します！",
        "speed_scale": 1.13,
    },
    {
        "id": "07_s3_hybrid_pcie_trap",
        "scene": "scene3_octree_vs_hybrid",
        "phase_hints": ["hybrid_pcie_trap"],
        "title": "Scene 3: 5,000万点での罠 — 毎フレーム PCIe 再転送のボトルネック",
        "subtitle": "では 5,000万点では何が効くのか？Compute Shader で点を間引いても、毎フレーム CPU 側の USD へ書き戻すと PCIe バスに毎フレーム十数MBの再転送が発生し 11 FPS に落ちます。",
        "speech_text": "では、五千万点では何が効くのでしょうか？コンピュートシェーダーで点を間引いても、毎フレームシーピーユー側のユーエスディーへ書き戻すと、ピーシーアイイーバスに毎フレーム十数メガバイトの再転送が発生し、十一エフピーエスに落ちます。",
        "speed_scale": 1.14,
    },
    {
        "id": "08_s3_octree_lod_zero_copy",
        "scene": "scene3_octree_vs_hybrid",
        "phase_hints": ["octree_lod_active"],
        "title": "Scene 3: 32ブロック Octree LOD (座標転送ゼロ・Visibility 1bit 命令)",
        "subtitle": "対して Octree LOD は全ブロックの LOD0・1・2 を初回に VRAM 常駐！カメラ移動時は座標を一切送らず、カメラ行列とブロックの表示・非表示フラグ切替命令だけで 35 FPS で動きます！",
        "speech_text": "対してオクトリーエルオーディーは、全ブロックのエルオーディーゼロ、イチ、ニを、初回にブイラムへ常駐させます！カメラ移動時は座標を一切送らず、カメラ行列とブロックの表示フラグ切替命令だけで、三十五エフピーエスで動きます！",
        "speed_scale": 1.14,
    },
    {
        "id": "09_s4_warp_atomic64",
        "scene": "scene4_warp_atomic64",
        "phase_hints": ["warp_atomic64_run"],
        "title": "Scene 4: Warp CUDA 64-bit atomicMin (1 Kernel Launch で 118 FPS)",
        "subtitle": "最速の Warp CUDA 64ビット atomicMin 方式！5,000万点を CUDA VRAM に完全固定し、毎フレームは 64バイトのカメラ行列と 1回のカーネル起動命令だけで 118 FPS を叩き出します！",
        "speech_text": "そして最速のワープクーダ、六十四ビットアトミックミン方式です！五千万点をクーダブイラムに完全固定し、毎フレームは六十四バイトのカメラ行列と、一回のカーネル起動命令だけで、百十八エフピーエスを叩き出します！",
        "speed_scale": 1.13,
    },
    {
        "id": "10_outro_summary",
        "scene": "outro",
        "phase_hints": [],
        "title": "まとめ: 各描画方式の CPU⇔GPU 転送データと DrawCall 比較一覧",
        "subtitle": "まとめ！高速化の鉄則は「点群座標を初回に VRAM 常駐させ、カメラ移動時は 64B のカメラ行列と最小の命令だけを送ること」。用途に合わせて統一関数で使い分けましょう！",
        "speech_text": "まとめです！高速化の鉄則は、てんぐん座標を初回にブイラムへ常駐させ、カメラ移動時は六十四バイトのカメラ行列と、最小の命令だけを送ること。用途に合わせて統一関数で使い分けましょう！",
        "speed_scale": 1.13,
    },
]


def wait_for_voicevox(timeout_sec: float = 30.0) -> list[dict]:
    deadline = time.time() + timeout_sec
    last_err = None
    while time.time() < deadline:
        try:
            with urllib.request.urlopen(f"{VOICEVOX_URL}/speakers", timeout=5) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except Exception as exc:
            last_err = exc
            time.sleep(0.5)
    raise RuntimeError(f"VOICEVOX Engine not reachable at {VOICEVOX_URL}: {last_err}")


def find_speaker_id(speakers: list[dict]) -> tuple[int, str]:
    for sp in speakers:
        if sp.get("name") == SPEAKER_NAME:
            for st in sp.get("styles", []):
                if st.get("name") == STYLE_NAME:
                    return int(st["id"]), f"{sp['name']}（{st['name']}）"
    return DEFAULT_SPEAKER_ID, f"{SPEAKER_NAME}（{STYLE_NAME}）"


def synthesize_wav(text: str, speaker_id: int, out_path: Path, speed_scale: float = 1.13) -> tuple[float, str]:
    qs = urllib.parse.urlencode({"text": text, "speaker": speaker_id})
    req = urllib.request.Request(f"{VOICEVOX_URL}/audio_query?{qs}", method="POST")
    with urllib.request.urlopen(req, timeout=30) as resp:
        query = json.loads(resp.read().decode("utf-8"))

    kana = str(query.get("kana", ""))
    query["speedScale"] = speed_scale
    query["pitchScale"] = 0.0
    query["intonationScale"] = 1.05
    query["volumeScale"] = 1.05
    query["prePhonemeLength"] = 0.08
    query["postPhonemeLength"] = 0.22
    query["outputSamplingRate"] = 48000
    query["outputStereo"] = False

    synth_qs = urllib.parse.urlencode({"speaker": speaker_id})
    synth_req = urllib.request.Request(
        f"{VOICEVOX_URL}/synthesis?{synth_qs}",
        data=json.dumps(query).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(synth_req, timeout=60) as resp:
        wav_bytes = resp.read()

    out_path.write_bytes(wav_bytes)
    with wave.open(str(out_path), "rb") as wf:
        duration = wf.getnframes() / float(wf.getframerate())
    return duration, kana


def main() -> int:
    AUDIO_DIR.mkdir(parents=True, exist_ok=True)
    keep_wavs = {f"{seg['id']}.wav" for seg in SEGMENTS}
    for old_wav in AUDIO_DIR.glob("*.wav"):
        if old_wav.name not in keep_wavs and old_wav.name != "narration_master.wav":
            old_wav.unlink(missing_ok=True)

    speakers = wait_for_voicevox()
    speaker_id, speaker_label = find_speaker_id(speakers)
    print(f"Using VOICEVOX speaker: {speaker_label} (id={speaker_id})")

    manifest_segments = []
    total_dur = 0.0
    for seg in SEGMENTS:
        wav_name = f"{seg['id']}.wav"
        wav_path = AUDIO_DIR / wav_name
        t0 = time.time()
        dur, kana = synthesize_wav(
            seg["speech_text"],
            speaker_id=speaker_id,
            out_path=wav_path,
            speed_scale=seg.get("speed_scale", 1.13),
        )
        total_dur += dur
        print(f"  [{seg['id']}] {dur:5.2f}s (gen {time.time() - t0:.2f}s) -> {wav_name}")
        print(f"      kana: {kana}")
        manifest_segments.append(
            {
                "id": seg["id"],
                "scene": seg["scene"],
                "phase_hints": seg["phase_hints"],
                "title": seg["title"],
                "subtitle": seg["subtitle"],
                "speech_text": seg["speech_text"],
                "kana": kana,
                "wav": wav_name,
                "duration_sec": round(dur, 4),
            }
        )

    manifest = {
        "speaker": speaker_label,
        "speaker_id": speaker_id,
        "sample_rate": 48000,
        "total_duration_sec": round(total_dur, 4),
        "segments": manifest_segments,
    }
    MANIFEST_PATH.write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\nDone! Generated {len(manifest_segments)} clips, total narration = {total_dur:.2f}s")
    print(f"Manifest saved to: {MANIFEST_PATH}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
