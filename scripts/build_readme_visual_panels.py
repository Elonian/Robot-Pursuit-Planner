#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont, ImageSequence


ROOT = Path(__file__).resolve().parents[1]
VIS_ROOT = ROOT / "output" / "visualizations"
OUT_ROOT = VIS_ROOT / "readme_panels"

MAPS = ("map1", "map2", "map3", "map4")
PLANNERS = ("hybrid", "multigoal", "bfs", "direct", "greedy")
PLANNER_TITLES = {
    "hybrid": "Hybrid Planner",
    "multigoal": "Multi-Goal A*",
    "bfs": "BFS Intercept",
    "direct": "Direct Intercept",
    "greedy": "Greedy Baseline",
}

BG = (9, 13, 18)
CARD = (17, 24, 39)
HEADER = (24, 34, 53)
LINE = (51, 65, 85)
TEXT = (248, 250, 252)
MUTED = (148, 163, 184)
ACCENT = (45, 212, 191)
WARN = (251, 146, 60)

CELL_W = 720
CELL_H = 405
PAD = 24
TITLE_H = 62
LABEL_H = 42
FOOT_H = 34
COLS = 2
ROWS = 2
CANVAS_W = PAD + COLS * CELL_W + (COLS - 1) * PAD + PAD
CANVAS_H = TITLE_H + ROWS * (LABEL_H + CELL_H) + (ROWS - 1) * PAD + FOOT_H + PAD


@dataclass(frozen=True)
class Metrics:
    caught: str
    time: str
    cost: str
    moves: str


def load_font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    candidates = []
    if bold:
        candidates.extend(
            [
                "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf",
            ]
        )
    else:
        candidates.extend(
            [
                "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
            ]
        )
    for candidate in candidates:
        path = Path(candidate)
        if path.exists():
            return ImageFont.truetype(path, size=size)
    return ImageFont.load_default()


FONT_TITLE = load_font(30, bold=True)
FONT_LABEL = load_font(19, bold=True)
FONT_METRIC = load_font(16, bold=False)
FONT_FOOT = load_font(15, bold=False)


def fit_image(image: Image.Image, width: int, height: int) -> Image.Image:
    src = image.convert("RGB")
    scale = min(width / src.width, height / src.height)
    new_w = max(1, int(src.width * scale))
    new_h = max(1, int(src.height * scale))
    resized = src.resize((new_w, new_h), Image.Resampling.LANCZOS)
    canvas = Image.new("RGB", (width, height), CARD)
    canvas.paste(resized, ((width - new_w) // 2, (height - new_h) // 2))
    return canvas


def sample_indices(src_count: int, dst_count: int) -> list[int]:
    if src_count <= 1:
        return [0] * dst_count
    if dst_count <= 1:
        return [src_count - 1]
    return [round(i * (src_count - 1) / (dst_count - 1)) for i in range(dst_count)]


def load_gif_frames(path: Path, frame_count: int) -> list[Image.Image]:
    with Image.open(path) as gif:
        frames = [frame.convert("RGB").copy() for frame in ImageSequence.Iterator(gif)]
    indices = sample_indices(len(frames), frame_count)
    return [fit_image(frames[i], CELL_W, CELL_H) for i in indices]


def load_final_frame(path: Path) -> Image.Image:
    return fit_image(Image.open(path), CELL_W, CELL_H)


def load_metrics(map_name: str, planner: str) -> Metrics:
    summary = VIS_ROOT / map_name / "summary.csv"
    with summary.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row["planner"] == planner:
                return Metrics(
                    caught=row["caught"],
                    time=row["time"],
                    cost=row["cost"],
                    moves=row["moves"],
                )
    raise ValueError(f"No metrics for {planner} in {summary}")


def metric_line(metrics: Metrics) -> tuple[str, tuple[int, int, int]]:
    status = "caught" if metrics.caught == "1" else "not caught"
    color = ACCENT if metrics.caught == "1" else WARN
    return (
        f"{status} | t={metrics.time} | cost={int(metrics.cost):,} | moves={metrics.moves}",
        color,
    )


def draw_panel_shell(planner: str) -> Image.Image:
    canvas = Image.new("RGB", (CANVAS_W, CANVAS_H), BG)
    draw = ImageDraw.Draw(canvas)
    title = f"{PLANNER_TITLES[planner]} | maps 1-4"
    draw.text((PAD, 18), title, fill=TEXT, font=FONT_TITLE)
    draw.text(
        (CANVAS_W - PAD, 25),
        "robot cyan | target magenta | lower cost preferred",
        fill=MUTED,
        font=FONT_FOOT,
        anchor="ra",
    )

    for i, map_name in enumerate(MAPS):
        row = i // COLS
        col = i % COLS
        x0 = PAD + col * (CELL_W + PAD)
        y0 = TITLE_H + row * (LABEL_H + CELL_H + PAD)
        x1 = x0 + CELL_W
        y1 = y0 + LABEL_H + CELL_H
        draw.rounded_rectangle((x0, y0, x1, y1), radius=8, fill=CARD, outline=LINE, width=2)
        draw.rectangle((x0, y0, x1, y0 + LABEL_H), fill=HEADER)
        draw.line((x0, y0 + LABEL_H, x1, y0 + LABEL_H), fill=ACCENT, width=2)

        metrics = load_metrics(map_name, planner)
        line, color = metric_line(metrics)
        draw.text((x0 + 14, y0 + 9), map_name, fill=TEXT, font=FONT_LABEL)
        draw.text((x1 - 14, y0 + 12), line, fill=color, font=FONT_METRIC, anchor="ra")

    draw.text(
        (PAD, CANVAS_H - 24),
        "Combined README panel built from output/visualizations/map{1..4}/<planner>/ panels.",
        fill=MUTED,
        font=FONT_FOOT,
    )
    return canvas


def paste_cells(canvas: Image.Image, images: dict[str, Image.Image]) -> Image.Image:
    out = canvas.copy()
    for i, map_name in enumerate(MAPS):
        row = i // COLS
        col = i % COLS
        x0 = PAD + col * (CELL_W + PAD)
        y0 = TITLE_H + row * (LABEL_H + CELL_H + PAD) + LABEL_H
        out.paste(images[map_name], (x0, y0))
    return out


def build_planner_panel(planner: str, frames: int, duration_ms: int) -> None:
    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    frame_sets = {
        map_name: load_gif_frames(VIS_ROOT / map_name / planner / "animated_panel.gif", frames)
        for map_name in MAPS
    }
    final_frames = {
        map_name: load_final_frame(VIS_ROOT / map_name / planner / "final_panel.png")
        for map_name in MAPS
    }

    rendered: list[Image.Image] = []
    for frame_idx in range(frames):
        shell = draw_panel_shell(planner)
        cells = {map_name: frame_sets[map_name][frame_idx] for map_name in MAPS}
        rendered.append(paste_cells(shell, cells).quantize(colors=224, method=Image.MEDIANCUT))

    gif_path = OUT_ROOT / f"{planner}_maps1_4_animated_panel.gif"
    rendered[0].save(
        gif_path,
        save_all=True,
        append_images=rendered[1:],
        duration=duration_ms,
        loop=0,
        optimize=False,
        disposal=2,
    )

    final_path = OUT_ROOT / f"{planner}_maps1_4_final_panel.png"
    paste_cells(draw_panel_shell(planner), final_frames).save(final_path, format="PNG", optimize=True)
    print(f"Wrote {gif_path}")
    print(f"Wrote {final_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Build README combined planner panels from map visualizations.")
    parser.add_argument("--planners", nargs="+", default=list(PLANNERS))
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--duration-ms", type=int, default=85)
    args = parser.parse_args()

    for planner in args.planners:
        build_planner_panel(planner, args.frames, args.duration_ms)


if __name__ == "__main__":
    main()
