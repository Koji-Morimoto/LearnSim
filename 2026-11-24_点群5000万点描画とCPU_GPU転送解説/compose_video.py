"""Compose the 1920x1080 30fps Japanese explainer video with VOICEVOX 四国めたん narration:
"Unified Point Cloud API & CPU<->GPU DrawCall / Data Transfer Architecture"

Strict 4-color semantic system (shared across 3D scene, CPU<->GPU Bus HUD, and Callouts):
  - USD_BLUE      = CPU / USD Stage / 64B Camera Matrix / Far LOD2
  - FABRIC_PURPLE = GPU RTX Batch / RT-Core PointInstancer / Mid LOD1
  - TENSOR_GREEN  = Zero-Copy Optimal Path / Near LOD0 / Warp CUDA atomic64 / High FPS
  - ALERT_RED     = Bottleneck (N DrawCalls in Naive Sphere / Per-Frame PCIe Copy in Hybrid)

Includes automated pre-render ergonomics audit asserting:
  - Every callout bubble stays continuously visible >= 3.5 seconds
  - active_line_idx never bounces backward within a segment
  - 100% of callouts use the 4-color semantic palette
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import subprocess
import sys
import time
import wave
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image, ImageDraw, ImageFont

BASE_DIR = Path(__file__).resolve().parent
AUDIO_DIR = BASE_DIR / "audio"
CAPTURED_DIR = BASE_DIR / "captured"
MANIFEST_PATH = AUDIO_DIR / "manifest.json"
TELEMETRY_PATH = CAPTURED_DIR / "telemetry.json"
CODE_SNIPPETS_PATH = CAPTURED_DIR / "code_snippets.json"
MASTER_WAV_PATH = AUDIO_DIR / "narration_master.wav"
OUTPUT_MP4_PATH = BASE_DIR / "pointcloud_cpu_gpu_explained.mp4"
TIMELINE_OUT_PATH = BASE_DIR / "render_timeline.json"

FFMPEG_EXE = os.environ.get("FFMPEG_EXE", r"C:\Users\TaskM\.pixi\bin\ffmpeg.exe")
FPS_OUT = 30
WIDTH = 1920
HEIGHT = 1080
TAIL_PAD_SEC = 0.25
MIN_CALLOUT_SEC = 3.5

# Layout rectangles
HUD_H = 180
VP_X, VP_Y, VP_W, VP_H = 16, 192, 1248, 702
CODE_X, CODE_Y, CODE_W, CODE_H = 1278, 192, 626, 702
SUB_Y, SUB_H = 906, 174

# Neutral chrome palette
BG_DARK = (11, 16, 27)
PANEL_BG = (16, 24, 39)
PANEL_BG_ALT = (13, 19, 32)
CARD_BORDER = (38, 56, 84)
TEXT_LIGHT = (241, 245, 249)
TEXT_MUTED = (148, 163, 184)
TEXT_DIM = (100, 116, 139)

# Strict 4-Color Semantic System
USD_BLUE = (72, 152, 255)
FABRIC_PURPLE = (186, 110, 255)
TENSOR_GREEN = (46, 213, 140)
ALERT_RED = (244, 67, 78)

FONT_JP_BOLD_CANDIDATES = [
    r"C:\Windows\Fonts\meiryob.ttc",
    r"C:\Windows\Fonts\YuGothB.ttc",
    r"C:\Windows\Fonts\meiryo.ttc",
    r"C:\Windows\Fonts\msgothic.ttc",
]
FONT_JP_REG_CANDIDATES = [
    r"C:\Windows\Fonts\meiryo.ttc",
    r"C:\Windows\Fonts\YuGothM.ttc",
    r"C:\Windows\Fonts\msgothic.ttc",
]
FONT_MONO_CANDIDATES = [
    r"C:\Windows\Fonts\consola.ttf",
    r"C:\Windows\Fonts\msgothic.ttc",
]
FONT_MONO_BOLD_CANDIDATES = [
    r"C:\Windows\Fonts\consolab.ttf",
    r"C:\Windows\Fonts\consola.ttf",
]


def _pick_font(candidates: list[str]) -> str:
    for p in candidates:
        if os.path.exists(p):
            return p
    raise FileNotFoundError(f"None of fonts found: {candidates}")


FONT_JP_BOLD_PATH = _pick_font(FONT_JP_BOLD_CANDIDATES)
FONT_JP_REG_PATH = _pick_font(FONT_JP_REG_CANDIDATES)
FONT_MONO_PATH = _pick_font(FONT_MONO_CANDIDATES)
FONT_MONO_BOLD_PATH = _pick_font(FONT_MONO_BOLD_CANDIDATES)

_FONT_CACHE: dict[tuple[str, int], ImageFont.FreeTypeFont] = {}


def get_font(path: str, size: int) -> ImageFont.FreeTypeFont:
    key = (path, size)
    if key not in _FONT_CACHE:
        _FONT_CACHE[key] = ImageFont.truetype(path, size=size)
    return _FONT_CACHE[key]


PY_KEYWORDS = {
    "def", "class", "return", "if", "elif", "else", "for", "while", "in", "not",
    "and", "or", "import", "from", "as", "with", "try", "except", "finally",
    "True", "False", "None", "pass", "break", "continue", "lambda", "yield", "is",
}
PY_BUILTINS = {
    "create_pointcloud", "UnifiedPointCloud", "PointCloudConfig", "UsdGeom",
    "Vt", "Gf", "Sdf", "wp", "stage", "pc", "np", "math",
}
TOKEN_RE = re.compile(
    r"(#.*$)"
    r"|('(?:\\.|[^'\\])*'|\"(?:\\.|[^\"\\])*\")"
    r"|(\b\d+(?:\.\d+)?\b)"
    r"|(\b[A-Za-z_][A-Za-z0-9_]*\b)"
    r"|(\s+|.)"
)


def tokenize_python_line(line: str) -> list[tuple[str, tuple[int, int, int]]]:
    tokens: list[tuple[str, tuple[int, int, int]]] = []
    pos = 0
    n = len(line)
    for m in TOKEN_RE.finditer(line):
        if m.start() > pos:
            tokens.append((line[pos : m.start()], TEXT_LIGHT))
        pos = m.end()
        comment, string, number, ident, other = m.groups()
        if comment is not None:
            tokens.append((comment, (110, 128, 152)))
        elif string is not None:
            tokens.append((string, (152, 195, 121)))
        elif number is not None:
            tokens.append((number, (209, 154, 102)))
        elif ident is not None:
            next_ch = line[pos] if pos < n else ""
            if ident in PY_KEYWORDS:
                tokens.append((ident, (198, 120, 221)))
            elif ident in PY_BUILTINS:
                tokens.append((ident, (165, 200, 250)))
            elif next_ch == "(":
                tokens.append((ident, (97, 175, 239)))
            else:
                tokens.append((ident, TEXT_LIGHT))
        elif other is not None:
            tokens.append((other, (180, 195, 215)))
    return tokens


def wrap_japanese_text(text: str, font: ImageFont.FreeTypeFont, max_width: int) -> list[str]:
    tokens = re.findall(r"[A-Za-z0-9_./:+()\-]+|\n|.", text)
    lines: list[str] = []
    cur = ""
    for tok in tokens:
        if tok == "\n":
            lines.append(cur.rstrip())
            cur = ""
            continue
        test = cur + tok
        if font.getlength(test) <= max_width or not cur:
            cur = test
        else:
            if len(tok) == 1 and tok in "。、！？）」』.,!?":
                lines.append((cur + tok).rstrip())
                cur = ""
            else:
                lines.append(cur.rstrip())
                cur = tok.lstrip()
    if cur:
        lines.append(cur.rstrip())
    return lines


def load_telemetry_and_snippets() -> tuple[dict[str, list[dict]], dict[str, dict]]:
    raw = json.loads(TELEMETRY_PATH.read_text(encoding="utf-8"))
    scenes = raw["scenes"]
    snippets = json.loads(CODE_SNIPPETS_PATH.read_text(encoding="utf-8"))
    return scenes, snippets


def build_master_audio(manifest: dict) -> list[dict]:
    sample_rate = int(manifest.get("sample_rate", 48000))
    pcm_chunks: list[np.ndarray] = []
    schedules: list[dict] = []

    current_frame = 0
    for seg in manifest["segments"]:
        wav_path = AUDIO_DIR / seg["wav"]
        with wave.open(str(wav_path), "rb") as wf:
            sr = wf.getframerate()
            nch = wf.getnchannels()
            raw_bytes = wf.readframes(wf.getnframes())
        samples = np.frombuffer(raw_bytes, dtype=np.int16)
        if nch > 1:
            samples = samples.reshape(-1, nch).mean(axis=1).astype(np.int16)
        if sr != sample_rate:
            x_old = np.linspace(0, 1, len(samples))
            new_len = int(round(len(samples) * sample_rate / sr))
            x_new = np.linspace(0, 1, new_len)
            samples = np.interp(x_new, x_old, samples).astype(np.int16)

        dur_sec = len(samples) / float(sample_rate)
        n_video_frames = max(15, int(round((dur_sec + TAIL_PAD_SEC) * FPS_OUT)))
        exact_audio_samples = int(round(n_video_frames * sample_rate / FPS_OUT))

        if len(samples) < exact_audio_samples:
            padded = np.zeros(exact_audio_samples, dtype=np.int16)
            padded[: len(samples)] = samples
        else:
            padded = samples[:exact_audio_samples]
        pcm_chunks.append(padded)

        start_sec = current_frame / float(FPS_OUT)
        end_sec = (current_frame + n_video_frames) / float(FPS_OUT)
        schedules.append(
            {
                **seg,
                "start_frame": current_frame,
                "end_frame": current_frame + n_video_frames,
                "num_frames": n_video_frames,
                "start_sec": round(start_sec, 3),
                "end_sec": round(end_sec, 3),
            }
        )
        current_frame += n_video_frames

    master_pcm = np.concatenate(pcm_chunks) if pcm_chunks else np.zeros(sample_rate, dtype=np.int16)
    with wave.open(str(MASTER_WAV_PATH), "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(master_pcm.tobytes())

    return schedules


def select_frames_for_segment(seg: dict, scenes: dict[str, list[dict]]) -> list[dict]:
    scene_name = seg["scene"]
    if scene_name == "outro":
        return []
    frames = scenes.get(scene_name, [])
    hints = [h.lower() for h in seg.get("phase_hints", [])]
    matched = [f for f in frames if str(f.get("phase", "")).lower() in hints]
    return matched if matched else frames


def _vp_to_canvas(uv: list[float] | tuple[float, float]) -> tuple[float, float]:
    scale_x = VP_W / 1280.0
    scale_y = VP_H / 720.0
    return (VP_X + float(uv[0]) * scale_x, VP_Y + float(uv[1]) * scale_y)


def build_callouts(seg_id: str, rec: dict) -> list[dict]:
    """Return callouts strictly using the 4-color semantic system (USD_BLUE, FABRIC_PURPLE, TENSOR_GREEN, ALERT_RED).
    Titles remain constant across the entire segment so every callout stays visible >= 3.5s."""
    targets = rec.get("targets_2d") or {}
    callouts: list[dict] = []

    sp_uv = targets.get("sphere_cluster") or [640.0, 360.0]
    sh_uv = targets.get("floor_shadow") or [720.0, 520.0]

    if seg_id == "01_intro_api":
        callouts.append(
            {
                "target_xy": _vp_to_canvas(sp_uv),
                "title": "create_pointcloud() 統一関数IF",
                "body": "全6方式を同一引数・同一メソッドで操作",
                "color": USD_BLUE,
            }
        )
    elif seg_id == "02_s1_radius_color_pos":
        callouts.append(
            {
                "target_xy": _vp_to_canvas(sp_uv),
                "title": "半径・色・位置をリアルタイム変更",
                "body": "set_point_radius / set_colors / set_world_offset",
                "color": TENSOR_GREEN,
            }
        )
    elif seg_id == "03_s1_glossy_shadow":
        callouts.append(
            {
                "target_xy": _vp_to_canvas(sp_uv),
                "title": "テカリ調整 (UsdPreviewSurface PBR)",
                "body": "roughness=0.08(ツヤ有) ⇔ 0.92(マット)",
                "color": FABRIC_PURPLE,
            }
        )
        callouts.append(
            {
                "target_xy": _vp_to_canvas(sh_uv),
                "title": "影のON/OFF (doNotCastShadows)",
                "body": "set_cast_shadows(True / False) で即時切替",
                "color": USD_BLUE,
            }
        )
    elif seg_id == "04_s2_naive_drawcalls":
        callouts.append(
            {
                "target_xy": (1400.0, 160.0),
                "title": "[ボトルネック] 毎フレーム N 回の DrawCall!",
                "body": "1点1PrimのためCPU命令発行が大渋滞 (14 FPS)",
                "color": ALERT_RED,
                "prefer_below": True,
            }
        )
        callouts.append(
            {
                "target_xy": _vp_to_canvas(sp_uv),
                "title": "個別 UsdGeom.Sphere (5,000 Prims)",
                "body": "カメラ移動のたびにCPUが全Primを走査",
                "color": ALERT_RED,
            }
        )
    elif seg_id == "05_s2_points_init_vs_cam":
        callouts.append(
            {
                "target_xy": (1400.0, 160.0),
                "title": "カメラ操作時は 64B 行列 + 1 DrawCall のみ!",
                "body": "点群座標は初回のみVRAM転送 (毎フレーム転送 0 B)",
                "color": TENSOR_GREEN,
                "prefer_below": True,
            }
        )
        callouts.append(
            {
                "target_xy": _vp_to_canvas(sp_uv),
                "title": "UsdGeom.Points (RTX 解析的球プリミティブ)",
                "body": "100万点を1回のバッチ命令でレイトレーシング",
                "color": USD_BLUE,
            }
        )
    elif seg_id == "06_s2_instancer_rtcore":
        callouts.append(
            {
                "target_xy": (1400.0, 160.0),
                "title": "RTコア インスタンシング (64B 行列 + 1 DrawCall)",
                "body": "1個の球プロトタイプをGPUハードウェアで複製描画",
                "color": FABRIC_PURPLE,
                "prefer_below": True,
            }
        )
        callouts.append(
            {
                "target_xy": _vp_to_canvas(sp_uv),
                "title": "UsdGeom.PointInstancer (55.8 FPS @ 100万点)",
                "body": "毎フレームの点群再転送ゼロ・RTコアで高速交差判定",
                "color": TENSOR_GREEN,
            }
        )
    elif seg_id == "07_s3_hybrid_pcie_trap":
        callouts.append(
            {
                "target_xy": (1400.0, 160.0),
                "title": "[罠] 毎フレーム GPU→CPU→GPU 再転送 (11.8 MB/frame)",
                "body": "USD Points属性を毎フレーム書き換えるとPCIe帯域が飽和!",
                "color": ALERT_RED,
                "prefer_below": True,
            }
        )
        callouts.append(
            {
                "target_xy": _vp_to_canvas(sp_uv),
                "title": "Hybrid Compute + USD 書戻し (11.1 FPS)",
                "body": "GPUで間引いてもCPU経由の再アップロードが足かせ",
                "color": ALERT_RED,
            }
        )
    elif seg_id == "08_s3_octree_lod_zero_copy":
        near_uv = targets.get("near_lod0") or [560.0, 400.0]
        far_uv = targets.get("far_lod2") or [740.0, 280.0]
        callouts.append(
            {
                "target_xy": (1400.0, 160.0),
                "title": "座標再転送ゼロ! (64B 行列 + Visibility 1bit切替)",
                "body": "全32ブロックのLOD0/1/2を初回にVRAM常駐 (35.3 FPS)",
                "color": TENSOR_GREEN,
                "prefer_below": True,
            }
        )
        callouts.append(
            {
                "target_xy": _vp_to_canvas(near_uv),
                "title": "近距離 LOD0 (緑) / 中距離 LOD1 (紫)",
                "body": "カメラ距離に応じてブロックの表示フラグだけ切替",
                "color": TENSOR_GREEN,
            }
        )
        callouts.append(
            {
                "target_xy": _vp_to_canvas(far_uv),
                "title": "遠距離 LOD2 (青: 面積保存半径で隙間なし)",
                "body": "視野外ブロックはカリングしてGPU負荷を99%削減",
                "color": USD_BLUE,
            }
        )
    elif seg_id == "09_s4_warp_atomic64":
        callouts.append(
            {
                "target_xy": (1400.0, 160.0),
                "title": "64B カメラ行列 + 1 CUDA Kernel Launch のみ!",
                "body": "USD DrawCall ゼロ・VRAM完全常駐で 118.8 FPS!",
                "color": TENSOR_GREEN,
                "prefer_below": True,
            }
        )
        callouts.append(
            {
                "target_xy": _vp_to_canvas(sp_uv),
                "title": "64-bit atomicMin((depth<<32) | color)",
                "body": "5,000万スレッドがL2/VRAM上で直接Zバッファ球描画",
                "color": TENSOR_GREEN,
            }
        )

    return callouts


def audit_timeline_ergonomics(
    schedules: list[dict],
    seg_src_frames: list[list[dict]],
) -> dict[str, Any]:
    allowed_palette = {USD_BLUE, FABRIC_PURPLE, TENSOR_GREEN, ALERT_RED}
    callout_runs: list[dict[str, Any]] = []
    line_bounces: list[dict[str, Any]] = []
    invalid_colors: list[dict[str, Any]] = []

    for seg_idx, seg in enumerate(schedules):
        srcs = seg_src_frames[seg_idx]
        n_out = seg["num_frames"]
        if not srcs:
            continue

        active_runs: dict[str, int] = {}
        prev_code_id: str | None = None
        prev_line_idx: int | None = None

        for fi in range(n_out):
            src_idx = min(len(srcs) - 1, max(0, int(fi * len(srcs) / float(n_out))))
            rec = srcs[src_idx]

            cid = str(rec.get("active_code_id", ""))
            lidx = int(rec.get("active_line_idx", 0))
            if prev_code_id == cid and prev_line_idx is not None and lidx < prev_line_idx:
                line_bounces.append(
                    {
                        "segment": seg["id"],
                        "frame_in_seg": fi,
                        "code_id": cid,
                        "from_line": prev_line_idx + 1,
                        "to_line": lidx + 1,
                    }
                )
            prev_code_id = cid
            prev_line_idx = lidx

            frame_callouts = build_callouts(seg["id"], rec)
            for c in frame_callouts:
                if tuple(c["color"]) not in allowed_palette:
                    invalid_colors.append({"segment": seg["id"], "title": c["title"], "color": c["color"]})
            titles_now = {c["title"] for c in frame_callouts}
            for t in list(active_runs.keys()):
                if t not in titles_now:
                    start_f = active_runs.pop(t)
                    dur = (fi - start_f) / float(FPS_OUT)
                    callout_runs.append(
                        {
                            "segment": seg["id"],
                            "title": t,
                            "duration_sec": round(dur, 2),
                            "start_sec": round((seg["start_frame"] + start_f) / float(FPS_OUT), 2),
                            "end_sec": round((seg["start_frame"] + fi) / float(FPS_OUT), 2),
                        }
                    )
            for t in titles_now:
                if t not in active_runs:
                    active_runs[t] = fi

        for t, start_f in active_runs.items():
            dur = (n_out - start_f) / float(FPS_OUT)
            callout_runs.append(
                {
                    "segment": seg["id"],
                    "title": t,
                    "duration_sec": round(dur, 2),
                    "start_sec": round((seg["start_frame"] + start_f) / float(FPS_OUT), 2),
                    "end_sec": round((seg["start_frame"] + n_out) / float(FPS_OUT), 2),
                }
            )

    short_callouts = [r for r in callout_runs if r["duration_sec"] < MIN_CALLOUT_SEC]
    min_dur = min((r["duration_sec"] for r in callout_runs), default=0.0)

    print("\n=== ERGONOMICS AUDIT REPORT ===")
    print(f"Callout runs checked: {len(callout_runs)} | Min duration: {min_dur:.2f}s (threshold >= {MIN_CALLOUT_SEC:.1f}s)")
    for r in callout_runs:
        flag = " [FAIL < 3.5s]" if r["duration_sec"] < MIN_CALLOUT_SEC else " [OK]"
        print(f"  [{r['segment']:24s}] {r['start_sec']:6.2f}s..{r['end_sec']:6.2f}s ({r['duration_sec']:5.2f}s) {r['title']}{flag}")
    print(f"Code active_line_idx backward bounces: {len(line_bounces)}")

    if short_callouts:
        raise AssertionError(f"Found {len(short_callouts)} callout(s) shorter than {MIN_CALLOUT_SEC}s: {short_callouts}")
    if line_bounces:
        raise AssertionError(f"Found {len(line_bounces)} backward active_line_idx bounce(s): {line_bounces[:5]}")
    if invalid_colors:
        raise AssertionError(f"Found callout(s) outside 4-color semantic palette: {invalid_colors[:5]}")

    print("Semantic 4-color palette check: PASSED (100% in {USD_BLUE, FABRIC_PURPLE, TENSOR_GREEN, ALERT_RED})")
    print("=== ERGONOMICS AUDIT PASSED ===\n")
    return {
        "min_callout_sec": round(min_dur, 2),
        "threshold_sec": MIN_CALLOUT_SEC,
        "callout_runs": callout_runs,
        "line_bounces": line_bounces,
        "palette_violations": len(invalid_colors),
        "passed": True,
    }


def _rects_overlap(a: tuple[int, int, int, int], b: tuple[int, int, int, int], pad: int = 10) -> bool:
    return not (a[2] + pad <= b[0] or b[2] + pad <= a[0] or a[3] + pad <= b[1] or b[3] + pad <= a[1])


def draw_callouts_with_leaders(canvas: Image.Image, callouts: list[dict]) -> None:
    if not callouts:
        return

    overlay = Image.new("RGBA", canvas.size, (0, 0, 0, 0))
    odraw = ImageDraw.Draw(overlay)

    f_title = get_font(FONT_JP_BOLD_PATH, 19)
    f_body = get_font(FONT_JP_REG_PATH, 15)

    placed_rects: list[tuple[int, int, int, int]] = []
    safe_x0, safe_y0 = VP_X + 14, VP_Y + 14
    safe_x1, safe_y1 = VP_X + VP_W - 14, VP_Y + VP_H - 14

    for c in callouts:
        tx, ty = c["target_xy"]
        title = c["title"]
        body = c["body"]
        color = c["color"]
        prefer_below = bool(c.get("prefer_below", False))

        tw = int(max(f_title.getlength(title), f_body.getlength(body))) + 32
        bw = max(270, min(510, tw))
        bh = 68

        if prefer_below:
            candidates = [
                (safe_x1 - bw - 12, VP_Y + 18),
                (int(tx - bw / 2), VP_Y + 18),
                (safe_x0 + 12, VP_Y + 18),
            ]
        else:
            candidates = [
                (safe_x0 + 16, safe_y1 - bh - 16),
                (safe_x1 - bw - 16, safe_y1 - bh - 16),
                (safe_x0 + 16, safe_y0 + 16),
                (int(tx + 55), int(ty - bh - 36)),
                (int(tx - bw - 55), int(ty - bh - 36)),
                (int(tx + 55), int(ty + 36)),
                (int(tx - bw - 55), int(ty + 36)),
            ]

        best_rect = None
        for cx0, cy0 in candidates:
            cx0 = max(safe_x0, min(safe_x1 - bw, cx0))
            cy0 = max(safe_y0, min(safe_y1 - bh, cy0))
            rect = (cx0, cy0, cx0 + bw, cy0 + bh)
            covers_target = (rect[0] - 12 <= tx <= rect[2] + 12) and (rect[1] - 12 <= ty <= rect[3] + 12)
            if not covers_target and not any(_rects_overlap(rect, r) for r in placed_rects):
                best_rect = rect
                break

        if best_rect is None:
            best_rect = (safe_x0 + 16, safe_y1 - bh - 16, safe_x0 + 16 + bw, safe_y1 - 16)

        placed_rects.append(best_rect)
        bx0, by0, bx1, by1 = best_rect

        if ty < by0:
            ax = max(bx0 + 24, min(bx1 - 24, tx))
            ay = float(by0)
            elbow = (ax, (ay + ty) * 0.5)
        elif ty > by1:
            ax = max(bx0 + 24, min(bx1 - 24, tx))
            ay = float(by1)
            elbow = (ax, (ay + ty) * 0.5)
        elif tx < bx0:
            ax = float(bx0)
            ay = max(by0 + 18, min(by1 - 18, ty))
            elbow = ((ax + tx) * 0.5, ay)
        else:
            ax = float(bx1)
            ay = max(by0 + 18, min(by1 - 18, ty))
            elbow = ((ax + tx) * 0.5, ay)

        pts = [(ax, ay), elbow, (tx, ty)]
        odraw.line(pts, fill=(8, 12, 20, 230), width=6, joint="curve")
        odraw.line(pts, fill=(*color, 255), width=3, joint="curve")

        for r, alpha in ((14, 75), (9, 160)):
            odraw.ellipse((tx - r, ty - r, tx + r, ty + r), outline=(*color, alpha), width=2)
        odraw.ellipse((tx - 5, ty - 5, tx + 5, ty + 5), fill=(10, 15, 24, 240), outline=(*color, 255), width=2)

        odraw.rounded_rectangle((bx0 + 3, by0 + 4, bx1 + 3, by1 + 4), radius=10, fill=(0, 0, 0, 140))
        odraw.rounded_rectangle(
            (bx0, by0, bx1, by1),
            radius=10,
            fill=(11, 18, 32, 240),
            outline=(*color, 255),
            width=2,
        )
        odraw.rounded_rectangle((bx0 + 4, by0 + 6, bx0 + 9, by1 - 6), radius=3, fill=(*color, 255))
        odraw.text((bx0 + 18, by0 + 8), title, font=f_title, fill=(255, 255, 255, 255))
        odraw.text((bx0 + 18, by0 + 38), body, font=f_body, fill=(203, 213, 225, 255))

    composited = Image.alpha_composite(canvas.convert("RGBA"), overlay)
    canvas.paste(composited.convert("RGB"))


def draw_top_hud(draw: ImageDraw.ImageDraw, seg: dict, rec: dict) -> None:
    """Draw the 1920x180 top HUD:
    Module 1 (x=16..536)  : Scene + Timeline State + Real-time FPS & Frame Time
    Module 2 (x=548..848) : Physics Clock (sim_time & num_steps)
    Module 3 (x=860..1904): Live CPU <=> GPU PCIe Bus & DrawCall Architecture Diagram
    """
    draw.rectangle((0, 0, WIDTH, HUD_H), fill=PANEL_BG)
    draw.line((0, HUD_H - 1, WIDTH, HUD_H - 1), fill=CARD_BORDER, width=2)

    scene_name = seg["scene"]
    scene_tag_map = {
        "scene1_api_appearance": ("SCENE 1/4", USD_BLUE),
        "scene2_drawcall_vs_batch": ("SCENE 2/4", FABRIC_PURPLE),
        "scene3_octree_vs_hybrid": ("SCENE 3/4", TENSOR_GREEN),
        "scene4_warp_atomic64": ("SCENE 4/4", TENSOR_GREEN),
        "outro": ("SUMMARY", USD_BLUE),
    }
    tag_txt, tag_col = scene_tag_map.get(scene_name, ("DEMO", USD_BLUE))

    # --- MODULE 1: Scene + Timeline + FPS (x=16..536) ---
    draw.rounded_rectangle((16, 10, 536, 170), radius=10, fill=PANEL_BG_ALT, outline=CARD_BORDER, width=2)
    draw.rounded_rectangle((26, 18, 120, 42), radius=5, fill=tag_col, outline=tag_col)
    draw.text((34, 23), tag_txt, font=get_font(FONT_MONO_BOLD_PATH, 14), fill=(10, 14, 22))

    f_title = get_font(FONT_JP_BOLD_PATH, 17)
    title_str = seg.get("title", "")
    while f_title.getlength(title_str) > 395 and len(title_str) > 4:
        title_str = title_str[:-2] + "…"
    draw.text((128, 18), title_str, font=f_title, fill=TEXT_LIGHT)

    tl = str(rec.get("timeline_state", "PLAYING")).upper()
    tl_label, tl_bg, tl_fg = ("▶ PLAY ON", (14, 54, 42), TENSOR_GREEN) if tl == "PLAYING" else ("■ PLAY OFF", (58, 22, 30), ALERT_RED)
    draw.rounded_rectangle((26, 52, 218, 104), radius=8, fill=tl_bg, outline=tl_fg, width=2)
    draw.text((40, 63), tl_label, font=get_font(FONT_JP_BOLD_PATH, 22), fill=tl_fg)

    m_lbl = str(rec.get("method_label", "Unified API"))
    if len(m_lbl) > 23:
        m_lbl = m_lbl[:22] + "…"
    draw.rounded_rectangle((26, 114, 218, 158), radius=7, fill=(20, 30, 48), outline=USD_BLUE, width=1)
    draw.text((34, 127), m_lbl, font=get_font(FONT_JP_BOLD_PATH, 13), fill=TEXT_LIGHT)

    bus_status = str(rec.get("bus_status", "CAMERA_64B_ONLY"))
    is_bottleneck = bus_status in ("DRAWCALL_BOTTLENECK", "PER_FRAME_COPY_TRAP")
    fps_val = float(rec.get("fps", 60.0))
    step_ms = float(rec.get("step_ms", 16.6))
    fps_col = ALERT_RED if is_bottleneck else TENSOR_GREEN

    draw.rounded_rectangle(
        (228, 52, 524, 158),
        radius=8,
        fill=(48, 16, 22) if is_bottleneck else (16, 26, 42),
        outline=fps_col,
        width=2 if is_bottleneck else 1,
    )
    draw.text((238, 58), f"{fps_val:5.1f}", font=get_font(FONT_MONO_BOLD_PATH, 38), fill=fps_col)
    draw.text((366, 74), "FPS", font=get_font(FONT_MONO_BOLD_PATH, 16), fill=TEXT_MUTED)
    draw.rounded_rectangle((412, 62, 514, 94), radius=5, fill=(28, 38, 56), outline=fps_col, width=1)
    draw.text((420, 70), f"{step_ms:5.1f} ms", font=get_font(FONT_MONO_BOLD_PATH, 15), fill=fps_col)

    bx0, by0, bx1, by1 = 238, 112, 512, 126
    draw.rounded_rectangle((bx0, by0, bx1, by1), radius=4, fill=(24, 34, 52))
    frac = max(0.14, min(1.0, fps_val / 120.0))
    draw.rounded_rectangle((bx0, by0, int(bx0 + (bx1 - bx0) * frac), by1), radius=4, fill=fps_col)
    app_desc = str(rec.get("appearance_desc", "RTX RayTraced Spheres"))
    draw.text((238, 133), app_desc[:32], font=get_font(FONT_MONO_PATH, 13), fill=TEXT_MUTED)

    # --- MODULE 2: Physics Clock (x=548..848) ---
    draw.rounded_rectangle((548, 10, 848, 170), radius=10, fill=PANEL_BG_ALT, outline=CARD_BORDER, width=2)
    sim_time = float(rec.get("sim_time", 0.0))
    num_steps = int(rec.get("num_steps", 0))
    cx, cy, r = 612, 90, 50
    draw.ellipse((cx - r, cy - r, cx + r, cy + r), fill=(9, 14, 24), outline=TENSOR_GREEN, width=3)
    sweep_deg = (sim_time % 1.0) * 360.0
    if sim_time > 0.0:
        draw.arc((cx - r + 6, cy - r + 6, cx + r - 6, cy + r - 6), start=-90, end=-90 + max(5.0, sweep_deg), fill=TENSOR_GREEN, width=5)
    ang_fast = math.radians(sweep_deg - 90.0)
    draw.line((cx, cy, cx + (r - 12) * math.cos(ang_fast), cy + (r - 12) * math.sin(ang_fast)), fill=TENSOR_GREEN, width=3)
    draw.ellipse((cx - 4, cy - 4, cx + 4, cy + 4), fill=TEXT_LIGHT)

    draw.text((674, 22), "SIM CLOCK / STEPS", font=get_font(FONT_MONO_BOLD_PATH, 13), fill=TEXT_MUTED)
    draw.text((674, 48), f"{sim_time:6.3f} s", font=get_font(FONT_MONO_BOLD_PATH, 26), fill=TENSOR_GREEN)
    draw.text((674, 88), f"step: {num_steps:<5d}", font=get_font(FONT_MONO_BOLD_PATH, 20), fill=TEXT_LIGHT)
    tot_pts = int(rec.get("total_points", 0))
    act_pts = int(rec.get("active_points", 0))
    draw.text((674, 128), f"Active: {act_pts:,}", font=get_font(FONT_MONO_BOLD_PATH, 13), fill=USD_BLUE)
    draw.text((674, 146), f"Total : {tot_pts:,}", font=get_font(FONT_MONO_PATH, 12), fill=TEXT_MUTED)

    # --- MODULE 3: Live CPU <=> GPU PCIe Bus & DrawCall Monitor (x=860..1904) ---
    draw.rounded_rectangle((860, 10, 1904, 170), radius=10, fill=PANEL_BG_ALT, outline=CARD_BORDER, width=2)
    draw.text(
        (876, 16),
        "CPU ⇔ GPU DATA TRANSFER & DRAWCALL PIPELINE (カメラ操作時の毎フレーム通信)",
        font=get_font(FONT_JP_BOLD_PATH, 14),
        fill=TEXT_MUTED,
    )

    # Left Box: CPU RAM / USD Host (x=876..1130, y=44..156)
    draw.rounded_rectangle((876, 44, 1130, 156), radius=8, fill=(16, 28, 48), outline=USD_BLUE, width=2)
    draw.text((890, 52), "CPU (Host / USD Stage)", font=get_font(FONT_MONO_BOLD_PATH, 15), fill=USD_BLUE)
    init_mb = float(rec.get("pcie_init_mb", 28.0))
    draw.text((890, 78), f"Frame 0 初回: {init_mb:.1f} MB", font=get_font(FONT_JP_BOLD_PATH, 14), fill=TEXT_LIGHT)
    draw.text((890, 102), "Frame 1+ カメラ移動時:", font=get_font(FONT_JP_REG_PATH, 13), fill=TEXT_MUTED)
    if bus_status == "DRAWCALL_BOTTLENECK":
        cpu_act = "N個のPrimを毎フレーム走査!"
        cpu_col = ALERT_RED
    elif bus_status == "PER_FRAME_COPY_TRAP":
        cpu_act = "VtArray生成 + USD再書込!"
        cpu_col = ALERT_RED
    else:
        cpu_act = "カメラ行列(64B)のみ計算"
        cpu_col = TENSOR_GREEN
    draw.text((890, 124), cpu_act, font=get_font(FONT_JP_BOLD_PATH, 14), fill=cpu_col)

    # Center Box: PCIe Bus & DrawCalls per Frame (x=1146..1556, y=44..156)
    bus_col = ALERT_RED if is_bottleneck else (FABRIC_PURPLE if bus_status == "INIT_UPLOAD" else TENSOR_GREEN)
    bus_bg = (48, 16, 24) if is_bottleneck else (16, 36, 34)
    draw.rounded_rectangle((1146, 44, 1556, 156), radius=8, fill=bus_bg, outline=bus_col, width=2)

    pcie_bytes = str(rec.get("pcie_frame_bytes", "64 B (Camera Matrix Only!)"))
    drawcalls = str(rec.get("drawcalls_per_frame", "1 Batched DrawCall"))
    draw.text((1162, 50), "PCIe Bus 毎フレーム転送量:", font=get_font(FONT_JP_BOLD_PATH, 13), fill=TEXT_MUTED)
    draw.text((1162, 71), f">> {pcie_bytes}", font=get_font(FONT_MONO_BOLD_PATH, 16), fill=bus_col)
    draw.line((1160, 100, 1542, 100), fill=CARD_BORDER, width=1)
    draw.text((1162, 106), "CPU → GPU 描画命令 (DrawCall):", font=get_font(FONT_JP_BOLD_PATH, 13), fill=TEXT_MUTED)
    draw.text((1162, 127), f">> {drawcalls}", font=get_font(FONT_MONO_BOLD_PATH, 15), fill=TEXT_LIGHT)

    # Right Box: GPU VRAM & Execution Engine (x=1572..1890, y=44..156)
    gpu_col = FABRIC_PURPLE if "Instancer" in m_lbl or "Octree" in m_lbl else TENSOR_GREEN
    draw.rounded_rectangle((1572, 44, 1890, 156), radius=8, fill=(20, 24, 44), outline=gpu_col, width=2)
    draw.text((1586, 52), "GPU VRAM (座標データ常駐)", font=get_font(FONT_JP_BOLD_PATH, 15), fill=gpu_col)
    gpu_desc = str(rec.get("gpu_work_desc", "RTX Raytracing / CUDA"))
    if "PBR Shader" in gpu_desc:
        gpu_desc = "PBR Shader (Rough/Shadow)"
    elif "atomicMin" in gpu_desc:
        gpu_desc = "64bit atomicMin(depth|rgb)"
    elif "N Prim Traversal" in gpu_desc:
        gpu_desc = "CPU Bottleneck (N Prims)"
    draw.text((1586, 80), gpu_desc[:28], font=get_font(FONT_JP_BOLD_PATH, 13), fill=TEXT_LIGHT)
    ratio_pct = 100.0 * act_pts / float(max(1, tot_pts))
    draw.text(
        (1586, 106),
        f"VRAM常駐: {tot_pts:,} 点",
        font=get_font(FONT_JP_REG_PATH, 13),
        fill=TEXT_MUTED,
    )
    draw.text(
        (1586, 128),
        f"実描画球: {act_pts:,} ({ratio_pct:.1f}%)",
        font=get_font(FONT_JP_BOLD_PATH, 14),
        fill=TENSOR_GREEN if ratio_pct <= 10.0 or fps_val >= 50.0 else TEXT_LIGHT,
    )


def draw_code_panel(draw: ImageDraw.ImageDraw, rec: dict, snippets: dict[str, dict]) -> None:
    draw.rounded_rectangle((CODE_X, CODE_Y, CODE_X + CODE_W, CODE_Y + CODE_H), radius=10, fill=PANEL_BG_ALT, outline=CARD_BORDER, width=2)
    draw.rounded_rectangle((CODE_X, CODE_Y, CODE_X + CODE_W, CODE_Y + 40), radius=10, fill=(22, 32, 51))
    draw.rectangle((CODE_X, CODE_Y + 26, CODE_X + CODE_W, CODE_Y + 40), fill=(22, 32, 51))
    draw.line((CODE_X, CODE_Y + 40, CODE_X + CODE_W, CODE_Y + 40), fill=CARD_BORDER, width=1)

    for i, col in enumerate((USD_BLUE, FABRIC_PURPLE, TENSOR_GREEN)):
        cx = CODE_X + 18 + i * 18
        cy = CODE_Y + 20
        draw.ellipse((cx - 5, cy - 5, cx + 5, cy + 5), fill=col)

    code_id = str(rec.get("active_code_id", ""))
    active_idx = int(rec.get("active_line_idx", 0))
    snip = snippets.get(code_id) or {"title": "pointcloud_api.py", "lines": ["# Unified Point Cloud API"]}
    lines: list[str] = snip.get("lines", [])
    title = snip.get("title", f"{code_id}.py")

    draw.text((CODE_X + 76, CODE_Y + 11), title, font=get_font(FONT_MONO_BOLD_PATH, 15), fill=TEXT_LIGHT)

    line_badge = f"EXEC L{active_idx + 1:02d}"
    draw.rounded_rectangle((CODE_X + CODE_W - 110, CODE_Y + 8, CODE_X + CODE_W - 12, CODE_Y + 32), radius=5, fill=(20, 44, 76), outline=USD_BLUE, width=1)
    draw.text((CODE_X + CODE_W - 99, CODE_Y + 12), line_badge, font=get_font(FONT_MONO_BOLD_PATH, 14), fill=USD_BLUE)

    f_code = get_font(FONT_MONO_PATH, 16)
    f_code_bold = get_font(FONT_MONO_BOLD_PATH, 16)
    f_lnum = get_font(FONT_MONO_PATH, 14)

    body_y = CODE_Y + 52
    line_h = 28
    code_start_x = CODE_X + 52
    max_code_x = CODE_X + CODE_W - 14

    for idx, raw_line in enumerate(lines[:22]):
        y = body_y + idx * line_h
        is_active = idx == active_idx
        if is_active:
            draw.rectangle((CODE_X + 2, y - 3, CODE_X + CODE_W - 2, y + line_h - 4), fill=(24, 48, 82))
            draw.rectangle((CODE_X + 2, y - 3, CODE_X + 8, y + line_h - 4), fill=USD_BLUE)
            draw.text((CODE_X + 12, y), "▶", font=get_font(FONT_JP_BOLD_PATH, 14), fill=TENSOR_GREEN)
        else:
            draw.text((CODE_X + 14, y + 2), f"{idx + 1:2d}", font=f_lnum, fill=TEXT_DIM)

        x = float(code_start_x)
        font_use = f_code_bold if is_active else f_code
        for tok_txt, tok_col in tokenize_python_line(raw_line):
            w = font_use.getlength(tok_txt)
            if x + w > max_code_x:
                break
            draw.text((x, y), tok_txt, font=font_use, fill=tok_col)
            x += w


def draw_outro_summary(draw: ImageDraw.ImageDraw) -> None:
    """Render full-screen comparison table of CPU<->GPU Data Transfer & DrawCalls across all 6 methods."""
    x0, y0, x1, y1 = 16, 192, 1904, 894
    draw.rounded_rectangle((x0, y0, x1, y1), radius=12, fill=PANEL_BG_ALT, outline=USD_BLUE, width=2)

    draw.text(
        (48, 214),
        "点群描画6方式の CPU ⇔ GPU 転送データ・DrawCall・実測FPS 比較まとめ",
        font=get_font(FONT_JP_BOLD_PATH, 30),
        fill=TEXT_LIGHT,
    )
    draw.text(
        (48, 256),
        "統一関数 create_pointcloud(stage, pts, rgb, method=..., point_radius=..., glossy=..., cast_shadows=...) で全方式を切替可能",
        font=get_font(FONT_JP_BOLD_PATH, 18),
        fill=USD_BLUE,
    )
    draw.line((48, 290, 1872, 290), fill=CARD_BORDER, width=2)

    headers = [
        (54, "method 引数 (描画方式)"),
        (390, "初回 (Frame 0: CPU→GPU)"),
        (730, "カメラ操作時 (Frame 1+ 毎フレーム転送)"),
        (1170, "毎フレームの命令 / DrawCall"),
        (1530, "100万点 / 5,000万点 実測"),
    ]
    f_hdr = get_font(FONT_JP_BOLD_PATH, 17)
    for hx, htxt in headers:
        draw.text((hx, 302), htxt, font=f_hdr, fill=TEXT_MUTED)
    draw.line((48, 334, 1872, 334), fill=CARD_BORDER, width=1)

    rows = [
        (
            "'naive_spheres'",
            "個別 UsdGeom.Sphere",
            ALERT_RED,
            "N個のPrim定義を転送",
            "座標再転送なし (0 B)",
            "[渋滞] N回の DrawCall を毎フレーム発行!",
            "5千点で 14.4 FPS (上限)",
        ),
        (
            "'usd_points'",
            "UsdGeom.Points (1M分割)",
            USD_BLUE,
            "座標・色・半径を一括転送",
            "64 B カメラ行列のみ (座標 0 B!)",
            "1 Chunk = 1 DrawCall (50Mで50個)",
            "1M: 38.4 FPS / 50M: 6.1 FPS",
        ),
        (
            "'point_instancer'",
            "UsdGeom.PointInstancer",
            FABRIC_PURPLE,
            "球1個 + N個の配置配列を転送",
            "64 B カメラ行列のみ (座標 0 B!)",
            "1 Instanced DrawCall (RTコア加速)",
            "1M: 55.8 FPS / >10M VRAM上限",
        ),
        (
            "'hybrid_compute_rtx'",
            "Compute間引き + USD書戻し",
            ALERT_RED,
            "全座標をCUDA VRAMへ転送",
            "[罠] 毎フレーム 11.8 MB (GPU→CPU→GPU)",
            "毎フレーム VtArray 再転送 + BVH更新",
            "1M: 32.0 FPS / 50M: 11.1 FPS",
        ),
        (
            "'octree_lod'",
            "32ブロック 3階層 Octree LOD",
            TENSOR_GREEN,
            "全ブロックLOD0/1/2をVRAM常駐",
            "64 B カメラ行列 + Visibility 1bitフラグ!",
            "座標転送ゼロ・視野外&遠方を自動間引き",
            "1M: 61.9 FPS / 50M: 35.3 FPS",
        ),
        (
            "'warp_compute_atomic64'",
            "Warp CUDA 64bit atomicMin",
            TENSOR_GREEN,
            "全5,000万点をCUDA VRAM常駐",
            "64 B カメラ行列のみ (座標 0 B!)",
            "USD DrawCall ゼロ (1 CUDA Kernel起動)",
            "1M: 1711 FPS / 50M: 118.8 FPS",
        ),
    ]

    f_m = get_font(FONT_MONO_BOLD_PATH, 18)
    f_sub = get_font(FONT_JP_BOLD_PATH, 15)
    f_cell = get_font(FONT_JP_BOLD_PATH, 17)

    for idx, (m_code, m_jp, col, init_s, cam_s, dc_s, fps_s) in enumerate(rows):
        ry = 348 + idx * 86
        draw.rounded_rectangle((48, ry, 1872, ry + 74), radius=8, fill=(18, 28, 46), outline=col, width=2)
        draw.rectangle((54, ry + 10, 62, ry + 64), fill=col)
        draw.text((74, ry + 10), m_code, font=f_m, fill=col)
        draw.text((74, ry + 38), m_jp, font=f_sub, fill=TEXT_LIGHT)

        draw.text((390, ry + 24), init_s, font=f_cell, fill=TEXT_LIGHT)
        draw.text((730, ry + 24), cam_s, font=f_cell, fill=ALERT_RED if "[罠]" in cam_s else TENSOR_GREEN)
        draw.text((1170, ry + 24), dc_s, font=f_cell, fill=ALERT_RED if "[渋滞]" in dc_s else TEXT_LIGHT)
        draw.text((1530, ry + 24), fps_s, font=f_cell, fill=col)


def draw_bottom_subtitle_bar(
    draw: ImageDraw.ImageDraw,
    seg: dict,
    global_frame: int,
    total_frames: int,
    schedules: list[dict],
) -> None:
    draw.rectangle((0, SUB_Y, WIDTH, HEIGHT), fill=(9, 14, 24))
    draw.line((0, SUB_Y, WIDTH, SUB_Y), fill=CARD_BORDER, width=2)

    legend_items = [
        (USD_BLUE, "CPU / USD / 64Bカメラ行列 / LOD2 (青)"),
        (FABRIC_PURPLE, "GPU RTX バッチ / PointInstancer / LOD1 (紫)"),
        (TENSOR_GREEN, "座標再転送ゼロ / Warp CUDA / LOD0 / 高FPS (緑)"),
        (ALERT_RED, "ボトルネック: N回DrawCall / 毎フレームPCIe再転送 (赤)"),
    ]
    f_leg = get_font(FONT_JP_BOLD_PATH, 14)
    item_widths = [20 + int(f_leg.getlength(lbl)) for _, lbl in legend_items]
    gap = 32
    total_leg_w = sum(item_widths) + gap * (len(legend_items) - 1)
    lx = (WIDTH - total_leg_w) // 2
    ly = 916
    for (col, lbl), iw in zip(legend_items, item_widths):
        draw.rounded_rectangle((lx, ly + 3, lx + 14, ly + 17), radius=3, fill=col)
        draw.text((lx + 20, ly), lbl, font=f_leg, fill=TEXT_MUTED)
        lx += iw + gap

    draw.line((32, 944, WIDTH - 32, 944), fill=(26, 38, 58), width=1)

    sub_text = seg.get("subtitle", "")
    f_sub = get_font(FONT_JP_BOLD_PATH, 30)
    lines = wrap_japanese_text(sub_text, f_sub, max_width=1820)
    if len(lines) > 2:
        f_sub = get_font(FONT_JP_BOLD_PATH, 26)
        lines = wrap_japanese_text(sub_text, f_sub, max_width=1820)

    line_h = 42 if f_sub.size >= 29 else 36
    total_h = len(lines) * line_h
    start_y = 954 + max(0, (104 - total_h) // 2)
    for li, line in enumerate(lines[:3]):
        ly_text = start_y + li * line_h
        lw = f_sub.getlength(line)
        tx = int((WIDTH - lw) / 2)
        draw.text((tx + 2, ly_text + 2), line, font=f_sub, fill=(0, 0, 0))
        draw.text((tx, ly_text), line, font=f_sub, fill=TEXT_LIGHT)

    draw.rectangle((0, 1068, WIDTH, HEIGHT), fill=(18, 26, 40))
    if total_frames > 0:
        px = int(WIDTH * (global_frame + 1) / float(total_frames))
        draw.rectangle((0, 1068, px, HEIGHT), fill=USD_BLUE)
        seen_scenes: set[str] = set()
        for sc in schedules:
            if sc["scene"] not in seen_scenes:
                seen_scenes.add(sc["scene"])
                dx = int(WIDTH * sc["start_frame"] / float(total_frames))
                if dx > 0:
                    draw.rectangle((dx - 1, 1068, dx + 1, HEIGHT), fill=(9, 14, 24))


def render_video(preview_frames_dir: Path | None = None, audit_only: bool = False) -> None:
    t_start = time.time()
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
    scenes, snippets = load_telemetry_and_snippets()
    schedules = build_master_audio(manifest)

    total_frames = schedules[-1]["end_frame"] if schedules else 0
    print(f"Loaded {len(schedules)} segments, total = {total_frames} frames ({total_frames / FPS_OUT:.2f}s)")

    seg_src_frames: list[list[dict]] = []
    for seg in schedules:
        srcs = select_frames_for_segment(seg, scenes)
        seg_src_frames.append(srcs)
        print(f"  [{seg['id']:24s}] {seg['start_sec']:6.2f}s..{seg['end_sec']:6.2f}s ({seg['num_frames']:3d} out frames) <- {len(srcs):3d} sim frames")

    audit_report = audit_timeline_ergonomics(schedules, seg_src_frames)

    TIMELINE_OUT_PATH.write_text(
        json.dumps(
            {
                "output_mp4": str(OUTPUT_MP4_PATH),
                "fps": FPS_OUT,
                "width": WIDTH,
                "height": HEIGHT,
                "total_frames": total_frames,
                "duration_sec": round(total_frames / float(FPS_OUT), 3),
                "audit": audit_report,
                "segments": schedules,
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )

    if audit_only:
        return

    if preview_frames_dir is not None:
        preview_frames_dir.mkdir(parents=True, exist_ok=True)

    ffmpeg_cmd = [
        FFMPEG_EXE,
        "-y",
        "-f", "rawvideo",
        "-vcodec", "rawvideo",
        "-pix_fmt", "rgb24",
        "-s", f"{WIDTH}x{HEIGHT}",
        "-r", str(FPS_OUT),
        "-i", "-",
        "-i", str(MASTER_WAV_PATH),
        "-c:v", "libx264",
        "-preset", "medium",
        "-crf", "19",
        "-pix_fmt", "yuv420p",
        "-c:a", "aac",
        "-b:a", "192k",
        "-shortest",
        "-movflags", "+faststart",
        str(OUTPUT_MP4_PATH),
    ]

    proc = None
    if preview_frames_dir is None:
        print(f"Launching ffmpeg: {' '.join(ffmpeg_cmd)}")
        proc = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
        assert proc.stdin is not None

    cached_png_path: Path | None = None
    cached_vp_img: Image.Image | None = None

    global_frame = 0
    for seg_idx, seg in enumerate(schedules):
        srcs = seg_src_frames[seg_idx]
        n_out = seg["num_frames"]

        for fi in range(n_out):
            if preview_frames_dir is not None and fi not in (n_out // 2,):
                global_frame += 1
                continue

            canvas = Image.new("RGB", (WIDTH, HEIGHT), BG_DARK)
            draw = ImageDraw.Draw(canvas)

            if seg["scene"] == "outro":
                outro_rec = {
                    "timeline_state": "PLAYING",
                    "method_label": "全6方式 まとめ比較",
                    "fps": 118.8,
                    "step_ms": 8.4,
                    "sim_time": 12.0,
                    "num_steps": 720,
                    "total_points": 50_000_000,
                    "active_points": 385_000,
                    "pcie_init_mb": 1200.0,
                    "pcie_frame_bytes": "64 B (Camera Matrix Only!)",
                    "drawcalls_per_frame": "1 Dispatch / Visibility Flag",
                    "gpu_work_desc": "VRAM常駐 + 最小コマンド転送",
                    "bus_status": "CAMERA_64B_ONLY",
                    "appearance_desc": "Unified API Summary",
                }
                draw_top_hud(draw, seg, outro_rec)
                draw_outro_summary(draw)
                draw_bottom_subtitle_bar(draw, seg, global_frame, total_frames, schedules)
            else:
                src_idx = min(len(srcs) - 1, max(0, int(fi * len(srcs) / float(n_out))))
                rec = srcs[src_idx]

                png_path = CAPTURED_DIR / rec["file"]
                if png_path != cached_png_path or cached_vp_img is None:
                    with Image.open(png_path) as im:
                        cached_vp_img = im.convert("RGB").resize((VP_W, VP_H), Image.Resampling.LANCZOS)
                    cached_png_path = png_path

                canvas.paste(cached_vp_img, (VP_X, VP_Y))
                draw.rectangle((VP_X - 2, VP_Y - 2, VP_X + VP_W + 1, VP_Y + VP_H + 1), outline=(34, 72, 114), width=2)

                draw_top_hud(draw, seg, rec)
                draw_code_panel(draw, rec, snippets)

                callouts = build_callouts(seg["id"], rec)
                draw_callouts_with_leaders(canvas, callouts)

                draw_bottom_subtitle_bar(draw, seg, global_frame, total_frames, schedules)

            if preview_frames_dir is not None:
                out_png = preview_frames_dir / f"{seg['id']}.png"
                canvas.save(out_png)
                print(f"  Saved preview: {out_png.name}")
            else:
                assert proc is not None and proc.stdin is not None
                proc.stdin.write(canvas.tobytes())

            global_frame += 1

    if proc is not None:
        assert proc.stdin is not None and proc.stderr is not None
        proc.stdin.close()
        stderr_out = proc.stderr.read().decode("utf-8", errors="replace")
        ret = proc.wait()
        if ret != 0:
            print(stderr_out[-2000:], file=sys.stderr)
            raise RuntimeError(f"ffmpeg failed with exit code {ret}")
        elapsed = time.time() - t_start
        size_mb = OUTPUT_MP4_PATH.stat().st_size / (1024 * 1024)
        print(f"\nSUCCESS! Rendered {OUTPUT_MP4_PATH} ({size_mb:.2f} MB) in {elapsed:.1f}s")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--preview-dir", type=str, default="", help="Render 1 preview PNG per segment")
    parser.add_argument("--audit-only", action="store_true", help="Run ergonomics audit only")
    args = parser.parse_args()
    preview_dir = Path(args.preview_dir) if args.preview_dir else None
    render_video(preview_frames_dir=preview_dir, audit_only=args.audit_only)
    return 0


if __name__ == "__main__":
    sys.exit(main())
