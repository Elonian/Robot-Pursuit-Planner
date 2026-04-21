#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import os
import re
import shutil
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from PIL import Image


RESULT_RE = {
    "caught": re.compile(r"target caught = (\d+)"),
    "time": re.compile(r"time taken \(s\) = (\d+)"),
    "moves": re.compile(r"moves made = (-?\d+)"),
    "cost": re.compile(r"path cost = (-?\d+)"),
}


@dataclass
class MapData:
    name: str
    x_size: int
    y_size: int
    collision_threshold: int
    robot_start: tuple[int, int]
    target: np.ndarray
    costmap: np.ndarray


@dataclass
class PlannerRun:
    planner: str
    map_name: str
    metrics: dict[str, int | str | float]
    robot: np.ndarray
    cumulative_cost: np.ndarray
    distance_to_target: np.ndarray
    out_dir: Path


def parse_map_file(path: Path) -> MapData:
    with path.open("r", encoding="utf-8") as f:
        assert f.readline().strip() == "N"
        x_size, y_size = [int(v) for v in f.readline().strip().split(",")]
        assert f.readline().strip() == "C"
        collision_threshold = int(f.readline().strip())
        assert f.readline().strip() == "R"
        robot_start = tuple(int(v) for v in f.readline().strip().split(","))
        assert f.readline().strip() == "T"

        target: list[tuple[int, int]] = []
        line = f.readline().strip()
        while line != "M":
            x, y = [int(float(v)) for v in line.split(",")]
            target.append((x, y))
            line = f.readline().strip()

        rows: list[list[float]] = []
        for line in f:
            stripped = line.strip()
            if stripped:
                rows.append([float(v) for v in stripped.split(",")])

    # Map files store x-major rows; transpose for image indexing as [y, x].
    costmap = np.asarray(rows, dtype=np.float32).T
    return MapData(
        name=path.stem,
        x_size=x_size,
        y_size=y_size,
        collision_threshold=collision_threshold,
        robot_start=(int(robot_start[0]), int(robot_start[1])),
        target=np.asarray(target, dtype=np.int32),
        costmap=costmap,
    )


def parse_robot_trajectory(path: Path) -> np.ndarray:
    rows: list[tuple[int, int, int]] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            stripped = line.strip()
            if stripped:
                rows.append(tuple(int(v) for v in stripped.split(",")))
    if not rows:
        raise ValueError(f"No trajectory rows in {path}")
    return np.asarray(rows, dtype=np.int32)


def parse_result(text: str) -> dict[str, int | str]:
    row: dict[str, int | str] = {}
    for key, regex in RESULT_RE.items():
        match = regex.search(text)
        row[key] = int(match.group(1)) if match else "NA"
    return row


def map_cost(map_data: MapData, x: int, y: int) -> int:
    return int(map_data.costmap[y - 1, x - 1])


def target_at(map_data: MapData, t: int) -> tuple[int, int]:
    idx = max(0, min(int(t), len(map_data.target) - 1))
    return int(map_data.target[idx, 0]), int(map_data.target[idx, 1])


def enrich_trajectory(map_data: MapData, robot: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    cumulative = np.zeros(len(robot), dtype=np.int64)
    distances = np.zeros(len(robot), dtype=np.int32)
    for i in range(len(robot)):
        t, x, y = [int(v) for v in robot[i]]
        tx, ty = target_at(map_data, t)
        distances[i] = max(abs(x - tx), abs(y - ty))
        if i + 1 < len(robot):
            duration = max(0, int(robot[i + 1, 0]) - t)
            cumulative[i + 1] = cumulative[i] + duration * map_cost(map_data, x, y)
    return cumulative, distances


def run_planner(
    root: Path,
    executable: Path,
    map_name: str,
    planner: str,
    out_dir: Path,
    timeout_s: float,
) -> tuple[dict[str, int | str | float], np.ndarray]:
    env = os.environ.copy()
    env["STP_PLANNER"] = planner
    started = time.perf_counter()
    proc = subprocess.run(
        [str(executable), map_name],
        cwd=root,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout_s,
        check=False,
    )
    elapsed = time.perf_counter() - started
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "run_output.txt").write_text(proc.stdout, encoding="utf-8")

    metrics = parse_result(proc.stdout)
    metrics.update(
        {
            "planner": planner,
            "map": map_name,
            "exit_code": proc.returncode,
            "wall_s": round(elapsed, 3),
        }
    )

    source_traj = root / "output" / "robot_trajectory.txt"
    if not source_traj.exists():
        raise FileNotFoundError(f"Missing trajectory after running {planner} on {map_name}")
    target_traj = out_dir / "trajectory.csv"
    shutil.copyfile(source_traj, target_traj)
    robot = parse_robot_trajectory(target_traj)
    (out_dir / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    return metrics, robot


def cost_limits(map_data: MapData) -> tuple[float, float]:
    costs = np.asarray(map_data.costmap, dtype=np.float32)
    c_min = float(np.min(costs))
    c_max = float(np.max(costs))
    if c_max <= c_min:
        c_max = c_min + 1.0
    return c_min, c_max


def planner_map_image(map_data: MapData) -> np.ndarray:
    costs = np.asarray(map_data.costmap, dtype=np.float32)
    obstacle = costs >= map_data.collision_threshold
    free = ~obstacle

    img = np.zeros((costs.shape[0], costs.shape[1], 3), dtype=np.float32)
    low = np.array([216, 221, 216], dtype=np.float32)
    mid = np.array([151, 166, 160], dtype=np.float32)
    high = np.array([71, 89, 84], dtype=np.float32)
    blocked = np.array([13, 15, 16], dtype=np.float32)

    img[:, :] = low
    if np.any(free):
        free_costs = costs[free]
        c_min = float(np.min(free_costs))
        c_max = float(np.max(free_costs))
        if c_max <= c_min:
            norm = np.zeros_like(costs, dtype=np.float32)
        else:
            norm = np.clip((costs - c_min) / (c_max - c_min), 0.0, 1.0)
        norm = norm ** 0.72
        first = norm <= 0.55
        mix_first = np.clip(norm / 0.55, 0.0, 1.0)[..., None]
        mix_second = np.clip((norm - 0.55) / 0.45, 0.0, 1.0)[..., None]
        blue_to_teal = (1.0 - mix_first) * low + mix_first * mid
        teal_to_orange = (1.0 - mix_second) * mid + mix_second * high
        img[free & first] = blue_to_teal[free & first]
        img[free & ~first] = teal_to_orange[free & ~first]

    img[obstacle] = blocked
    return np.clip(img, 0, 255).astype(np.uint8)


def relief_map_data(map_data: MapData, max_cells: int = 190) -> dict[str, np.ndarray | float]:
    costs = np.asarray(map_data.costmap, dtype=np.float32)
    step = max(1, int(np.ceil(max(map_data.x_size, map_data.y_size) / max_cells)))
    sampled = costs[::step, ::step]
    h, w = sampled.shape

    obstacle = sampled >= map_data.collision_threshold
    free = ~obstacle
    norm = np.zeros_like(sampled, dtype=np.float32)
    if np.any(free):
        vals = sampled[free]
        c_min = float(np.min(vals))
        c_max = float(np.max(vals))
        if c_max > c_min:
            norm[free] = np.clip((sampled[free] - c_min) / (c_max - c_min), 0.0, 1.0)
    norm[obstacle] = 1.18

    x_edges = np.linspace(1, map_data.x_size, w + 1, dtype=np.float32)
    y_edges = np.linspace(1, map_data.y_size, h + 1, dtype=np.float32)
    xx, yy = np.meshgrid(x_edges, y_edges)
    corner_h = np.pad(norm, ((0, 1), (0, 1)), mode="edge")
    relief_height = 0.085 * max(map_data.x_size, map_data.y_size)
    uu = xx + 0.32 * yy
    vv = 0.58 * yy - relief_height * corner_h
    return {
        "u": uu,
        "v": vv,
        "cost": norm,
        "height": relief_height,
        "x_size": float(map_data.x_size),
        "y_size": float(map_data.y_size),
    }


def relief_project(map_data: MapData, xs: np.ndarray, ys: np.ndarray, lift: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
    xs = np.asarray(xs, dtype=np.float32)
    ys = np.asarray(ys, dtype=np.float32)
    xi = np.clip(np.rint(xs).astype(np.int32) - 1, 0, map_data.x_size - 1)
    yi = np.clip(np.rint(ys).astype(np.int32) - 1, 0, map_data.y_size - 1)
    costs = np.asarray(map_data.costmap, dtype=np.float32)
    obstacle = costs >= map_data.collision_threshold
    free = ~obstacle
    norm_costs = np.zeros_like(costs, dtype=np.float32)
    if np.any(free):
        vals = costs[free]
        c_min = float(np.min(vals))
        c_max = float(np.max(vals))
        if c_max > c_min:
            norm_costs[free] = np.clip((costs[free] - c_min) / (c_max - c_min), 0.0, 1.0)
    norm_costs[obstacle] = 1.18
    relief_height = 0.085 * max(map_data.x_size, map_data.y_size)
    h = norm_costs[yi, xi] * relief_height
    return xs + 0.32 * ys, 0.58 * ys - h - lift


def draw_relief_map(ax: plt.Axes, relief_data: dict[str, np.ndarray | float]) -> None:
    cmap = LinearSegmentedColormap.from_list(
        "planner_relief",
        ["#d8ddd8", "#bec9c4", "#8fa39d", "#546b66", "#151817"],
    )
    ax.pcolormesh(
        relief_data["u"],
        relief_data["v"],
        relief_data["cost"],
        cmap=cmap,
        shading="flat",
        vmin=0.0,
        vmax=1.18,
        rasterized=True,
    )
    ax.set_aspect("equal", adjustable="box")
    ax.set_facecolor("#0b0f14")
    ax.set_xticks([])
    ax.set_yticks([])
    x_size = float(relief_data["x_size"])
    y_size = float(relief_data["y_size"])
    for x in np.linspace(1.0, x_size, 8):
        y = np.linspace(1.0, y_size, 160)
        xx = np.full_like(y, x)
        u = xx + 0.32 * y
        v = 0.58 * y
        ax.plot(u, v, color="white", alpha=0.10, lw=0.55, zorder=1)
    for y in np.linspace(1.0, y_size, 8):
        x = np.linspace(1.0, x_size, 160)
        yy = np.full_like(x, y)
        u = x + 0.32 * yy
        v = 0.58 * yy
        ax.plot(u, v, color="white", alpha=0.10, lw=0.55, zorder=1)


def set_relief_limits(ax: plt.Axes, map_data: MapData, relief_data: dict[str, np.ndarray | float]) -> None:
    u = np.asarray(relief_data["u"])
    v = np.asarray(relief_data["v"])
    pad = 0.035 * max(float(np.ptp(u)), float(np.ptp(v)), 1.0)
    ax.set_xlim(float(np.min(u) - pad), float(np.max(u) + pad))
    ax.set_ylim(float(np.max(v) + pad), float(np.min(v) - pad))


def set_relief_view_limits(
    ax: plt.Axes,
    map_data: MapData,
    view: tuple[float, float, float, float],
) -> None:
    xmin, xmax, ymin, ymax = view
    xs = np.array([xmin, xmax, xmax, xmin], dtype=np.float32)
    ys = np.array([ymin, ymin, ymax, ymax], dtype=np.float32)
    u, v = relief_project(map_data, xs, ys)
    pad = 0.12 * max(float(np.ptp(u)), float(np.ptp(v)), 1.0)
    ax.set_xlim(float(np.min(u) - pad), float(np.max(u) + pad))
    ax.set_ylim(float(np.max(v) + pad), float(np.min(v) - pad))


def display_costmap(map_data: MapData, map_style: str) -> np.ndarray:
    if map_style == "relief":
        return relief_map_data(map_data)
    if map_style == "paper":
        return np.asarray(map_data.costmap, dtype=np.float32)
    return planner_map_image(map_data)


def viewport_for(map_data: MapData, robot: np.ndarray) -> tuple[float, float, float, float]:
    xs = np.concatenate([robot[:, 1], map_data.target[:, 0]])
    ys = np.concatenate([robot[:, 2], map_data.target[:, 1]])
    xmin, xmax = int(xs.min()), int(xs.max())
    ymin, ymax = int(ys.min()), int(ys.max())
    span = max(xmax - xmin + 1, ymax - ymin + 1, 40)
    pad = max(20, int(0.12 * span))
    xmin = max(1, xmin - pad)
    xmax = min(map_data.x_size, xmax + pad)
    ymin = max(1, ymin - pad)
    ymax = min(map_data.y_size, ymax + pad)
    return float(xmin), float(xmax), float(ymin), float(ymax)


def frame_indices(robot: np.ndarray, max_frames: int) -> np.ndarray:
    if len(robot) <= max_frames:
        return np.arange(len(robot), dtype=np.int32)
    return np.unique(np.linspace(0, len(robot) - 1, max_frames, dtype=np.int32))


def figure_to_image(fig: plt.Figure) -> Image.Image:
    fig.canvas.draw()
    rgba = np.asarray(fig.canvas.buffer_rgba())
    return Image.fromarray(rgba).convert("P", palette=Image.Palette.ADAPTIVE, colors=256)


def render_panel(
    map_data: MapData,
    run: PlannerRun,
    frame_id: int,
    base_image: np.ndarray,
    view: tuple[float, float, float, float],
    map_style: str,
) -> Image.Image:
    t = int(run.robot[frame_id, 0])
    rx = int(run.robot[frame_id, 1])
    ry = int(run.robot[frame_id, 2])
    tx, ty = target_at(map_data, t)
    final_caught = int(run.metrics.get("caught", 0)) == 1
    current_gap = int(run.distance_to_target[frame_id])
    current_cost = int(run.cumulative_cost[frame_id])

    fig = plt.figure(figsize=(13.6, 7.65), dpi=110)
    fig.patch.set_facecolor("#090d12")
    grid = fig.add_gridspec(
        3,
        3,
        width_ratios=[1.1, 1.1, 0.92],
        height_ratios=[0.9, 0.9, 0.9],
        left=0.035,
        right=0.975,
        bottom=0.075,
        top=0.895,
        wspace=0.28,
        hspace=0.46,
    )
    ax_map = fig.add_subplot(grid[:, :2])
    ax_focus = fig.add_subplot(grid[0, 2])
    ax_dist = fig.add_subplot(grid[1, 2])
    ax_cost = fig.add_subplot(grid[2, 2])

    xmin, xmax, ymin, ymax = view
    is_relief = map_style == "relief"

    def plot_coords(xs: np.ndarray, ys: np.ndarray, lift: float = 0.0) -> tuple[np.ndarray, np.ndarray]:
        if is_relief:
            return relief_project(map_data, xs, ys, lift=lift)
        return np.asarray(xs), np.asarray(ys)

    if is_relief:
        draw_relief_map(ax_map, base_image)
        set_relief_limits(ax_map, map_data, base_image)
    elif map_style == "paper":
        vmin, vmax = cost_limits(map_data)
        map_image = ax_map.imshow(
            base_image,
            origin="upper",
            extent=(1, map_data.x_size, map_data.y_size, 1),
            cmap="jet",
            interpolation="nearest",
            vmin=vmin,
            vmax=vmax,
        )
        cbar = fig.colorbar(map_image, ax=ax_map, fraction=0.026, pad=0.018)
        cbar.ax.tick_params(labelsize=7, colors="#cbd5e1")
        cbar.outline.set_edgecolor("#475569")
        cbar.set_label("cell cost", color="#cbd5e1", fontsize=8)
    else:
        ax_map.imshow(
            base_image,
            origin="upper",
            extent=(1, map_data.x_size, map_data.y_size, 1),
            interpolation="nearest",
        )
    if not is_relief:
        ax_map.set_xlim(1, map_data.x_size)
        ax_map.set_ylim(map_data.y_size, 1)

    target_until = min(t + 1, len(map_data.target))
    robot_until = frame_id + 1
    sample_robot = np.linspace(0, robot_until - 1, num=min(28, robot_until), dtype=int)
    sample_target = np.linspace(0, target_until - 1, num=min(28, target_until), dtype=int)
    tu, tv = plot_coords(map_data.target[:, 0], map_data.target[:, 1], lift=3.0)
    ru, rv = plot_coords(run.robot[:, 1], run.robot[:, 2], lift=5.0)
    tpu, tpv = plot_coords(map_data.target[:target_until, 0], map_data.target[:target_until, 1], lift=3.0)
    rpu, rpv = plot_coords(run.robot[:robot_until, 1], run.robot[:robot_until, 2], lift=5.0)
    ax_map.plot(tu, tv, color="#ff00ff", lw=1.2, alpha=0.36)
    ax_map.plot(ru, rv, color="#06111f", lw=3.0, alpha=0.42, linestyle="--")
    ax_map.plot(
        ru,
        rv,
        color="#00e5ff",
        lw=1.85,
        alpha=0.76,
        linestyle="--",
    )
    ax_map.plot(
        tpu,
        tpv,
        color="#ff00ff",
        lw=2.4,
        alpha=0.95,
        label="target",
    )
    if target_until > 0:
        stu, stv = plot_coords(map_data.target[sample_target, 0], map_data.target[sample_target, 1], lift=3.2)
        ax_map.scatter(
            stu,
            stv,
            s=10,
            color="#ff66ff",
            edgecolor="none",
            alpha=0.9,
            zorder=5,
        )
    ax_map.plot(rpu, rpv, color="#06111f", lw=3.5, alpha=0.55)
    ax_map.plot(
        rpu,
        rpv,
        color="#00e5ff",
        lw=2.4,
        alpha=0.95,
        label="robot",
    )
    if robot_until > 0:
        sru, srv = plot_coords(run.robot[sample_robot, 1], run.robot[sample_robot, 2], lift=5.2)
        ax_map.scatter(
            sru,
            srv,
            s=10,
            color="#bffaff",
            edgecolor="none",
            alpha=0.9,
            zorder=5,
        )
    r_start_u, r_start_v = plot_coords(np.array([map_data.robot_start[0]]), np.array([map_data.robot_start[1]]), lift=7.0)
    t_start_u, t_start_v = plot_coords(np.array([map_data.target[0, 0]]), np.array([map_data.target[0, 1]]), lift=7.0)
    cur_ru, cur_rv = plot_coords(np.array([rx]), np.array([ry]), lift=8.0)
    cur_tu, cur_tv = plot_coords(np.array([tx]), np.array([ty]), lift=8.0)
    ax_map.text(
        float(r_start_u[0]),
        float(r_start_v[0]),
        "R",
        color="#00ff3b",
        fontsize=10,
        weight="bold",
        ha="center",
        va="center",
        zorder=8,
    )
    ax_map.text(
        float(t_start_u[0]),
        float(t_start_v[0]),
        "T",
        color="#ff00ff",
        fontsize=10,
        weight="bold",
        ha="center",
        va="center",
        zorder=8,
    )
    ax_map.scatter(cur_ru, cur_rv, s=115, color="#00e5ff", edgecolor="black", linewidth=0.7, zorder=7)
    ax_map.scatter(cur_tu, cur_tv, s=115, color="#ff00ff", edgecolor="black", linewidth=0.7, zorder=7)
    ax_map.set_title("Map Movement", loc="left", fontsize=12, color="#e5e7eb", pad=8)
    if not is_relief:
        ax_map.set_xlabel("x cell")
        ax_map.set_ylabel("y cell")
    ax_map.grid(color="white", alpha=0.12, linewidth=0.45)
    ax_map.legend(loc="lower right", frameon=True, framealpha=0.88, fontsize=8)

    focus_heatmap = map_style == "planner"
    if is_relief:
        draw_relief_map(ax_focus, base_image)
    elif map_style == "paper" or focus_heatmap:
        if focus_heatmap:
            vmin, vmax = cost_limits(map_data)
            focus_image = np.asarray(map_data.costmap, dtype=np.float32)
        else:
            focus_image = base_image
        ax_focus.imshow(
            focus_image,
            origin="upper",
            extent=(1, map_data.x_size, map_data.y_size, 1),
            cmap="jet",
            interpolation="nearest",
            vmin=vmin,
            vmax=vmax,
        )
    else:
        ax_focus.imshow(
            base_image,
            origin="upper",
            extent=(1, map_data.x_size, map_data.y_size, 1),
            interpolation="nearest",
        )
    ax_focus.plot(tu, tv, color="#ff00ff", lw=1.4, alpha=0.38)
    ax_focus.plot(ru, rv, color="#06111f", lw=2.8, alpha=0.42, linestyle="--")
    ax_focus.plot(ru, rv, color="#00e5ff", lw=1.75, alpha=0.76, linestyle="--")
    ax_focus.plot(
        tpu,
        tpv,
        color="#ff00ff",
        lw=2.0,
        alpha=0.92,
    )
    ax_focus.plot(rpu, rpv, color="#06111f", lw=3.2, alpha=0.55)
    ax_focus.plot(rpu, rpv, color="#00e5ff", lw=2.2)
    ax_focus.scatter(cur_ru, cur_rv, s=46, color="#00e5ff", edgecolor="black", linewidth=0.7, zorder=4)
    ax_focus.scatter(cur_tu, cur_tv, s=46, color="#ff00ff", edgecolor="black", linewidth=0.7, zorder=4)
    if is_relief:
        set_relief_view_limits(ax_focus, map_data, view)
    else:
        ax_focus.set_xlim(xmin, xmax)
        ax_focus.set_ylim(ymax, ymin)
    ax_focus.set_title("Trajectory Heatmap", loc="left", fontsize=11, color="#e5e7eb")
    ax_focus.set_xticks([])
    ax_focus.set_yticks([])
    ax_focus.grid(color="white", alpha=0.12, linewidth=0.45)

    times = run.robot[:, 0]
    ax_dist.plot(times, run.distance_to_target, color="#475569", lw=1.5, alpha=0.75)
    ax_dist.plot(times[:robot_until], run.distance_to_target[:robot_until], color="#2dd4bf", lw=2.6)
    ax_dist.scatter([t], [run.distance_to_target[frame_id]], s=56, color="#2dd4bf", edgecolor="#0f172a", zorder=4)
    ax_dist.set_title("Intercept Gap", loc="left", fontsize=11, color="#e5e7eb")
    ax_dist.set_xlabel("time")
    ax_dist.set_ylabel("Chebyshev cells")
    ax_dist.grid(color="#334155", alpha=0.55, linewidth=0.6)
    ax_dist.set_ylim(bottom=0)

    ax_cost.plot(times, run.cumulative_cost, color="#475569", lw=1.5, alpha=0.75)
    ax_cost.plot(times[:robot_until], run.cumulative_cost[:robot_until], color="#f59e0b", lw=2.6)
    ax_cost.scatter([t], [run.cumulative_cost[frame_id]], s=56, color="#f59e0b", edgecolor="#0f172a", zorder=4)
    ax_cost.set_title("Cumulative Cost", loc="left", fontsize=11, color="#e5e7eb")
    ax_cost.set_xlabel("time")
    ax_cost.set_ylabel("cost")
    ax_cost.grid(color="#334155", alpha=0.55, linewidth=0.6)
    ax_cost.set_ylim(bottom=0)

    for ax in (ax_map, ax_focus):
        for spine in ax.spines.values():
            spine.set_color("#cbd5e1")
    for ax in (ax_dist, ax_cost):
        ax.set_facecolor("#111827")
        ax.tick_params(colors="#94a3b8", labelsize=8)
        ax.xaxis.label.set_color("#cbd5e1")
        ax.yaxis.label.set_color("#cbd5e1")
        for spine in ax.spines.values():
            spine.set_color("#475569")

    fig.text(
        0.035,
        0.945,
        f"{map_data.name} | {run.planner}",
        color="#f8fafc",
        fontsize=17,
        weight="bold",
        ha="left",
        va="center",
    )
    fig.text(
        0.965,
        0.945,
        f"t={t} | gap={current_gap} | cost={current_cost}/{run.metrics.get('cost')}",
        color="#cbd5e1",
        fontsize=10,
        ha="right",
        va="center",
    )
    final_status = "caught" if final_caught else "not caught"
    fig.text(
        0.035,
        0.027,
        f"Robot path is cyan. Target path is magenta. Final result: {final_status}, time={run.metrics.get('time')}, moves={run.metrics.get('moves')}.",
        color="#94a3b8",
        fontsize=9,
        ha="left",
        va="center",
    )

    image = figure_to_image(fig)
    plt.close(fig)
    return image


def write_static_and_gif(
    map_data: MapData,
    run: PlannerRun,
    frames: int,
    duration_ms: int,
    map_style: str,
) -> None:
    base_image = display_costmap(map_data, map_style)
    view = viewport_for(map_data, run.robot)
    indices = frame_indices(run.robot, frames)
    images = [render_panel(map_data, run, int(idx), base_image, view, map_style) for idx in indices]
    gif_path = run.out_dir / "animated_panel.gif"
    png_path = run.out_dir / "final_panel.png"
    images[-1].convert("RGB").save(png_path)
    images[0].save(
        gif_path,
        save_all=True,
        append_images=images[1:],
        duration=duration_ms,
        loop=0,
        optimize=True,
    )


def write_overview(map_data: MapData, runs: list[PlannerRun], out_dir: Path, map_style: str) -> None:
    if not runs:
        return
    cols = min(3, len(runs))
    rows = int(np.ceil(len(runs) / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(5.1 * cols, 4.25 * rows), dpi=120)
    fig.patch.set_facecolor("#090d12")
    axes_arr = np.atleast_1d(axes).ravel()
    base_image = display_costmap(map_data, map_style)
    if map_style == "paper":
        vmin, vmax = cost_limits(map_data)
    for ax, run in zip(axes_arr, runs):
        xmin, xmax, ymin, ymax = viewport_for(map_data, run.robot)
        if map_style == "paper":
            ax.imshow(
                base_image,
                origin="upper",
                extent=(1, map_data.x_size, map_data.y_size, 1),
                cmap="jet",
                interpolation="nearest",
                vmin=vmin,
                vmax=vmax,
            )
        else:
            ax.imshow(
                base_image,
                origin="upper",
                extent=(1, map_data.x_size, map_data.y_size, 1),
                interpolation="nearest",
            )
        ax.set_xlim(xmin, xmax)
        ax.set_ylim(ymax, ymin)
        ax.plot(map_data.target[:, 0], map_data.target[:, 1], color="#ff00ff", lw=1.8, alpha=0.85)
        ax.plot(run.robot[:, 1], run.robot[:, 2], color="#00e5ff", lw=2.2)
        ax.scatter([run.robot[-1, 1]], [run.robot[-1, 2]], s=60, color="#00e5ff", edgecolor="black", zorder=4)
        ax.set_facecolor("#f8fafc")
        ax.set_title(
            f"{run.planner}: caught={run.metrics.get('caught')} cost={run.metrics.get('cost')}",
            fontsize=10,
            color="#e5e7eb",
            loc="left",
        )
        ax.set_xticks([])
        ax.set_yticks([])
    for ax in axes_arr[len(runs):]:
        ax.axis("off")
    fig.suptitle(f"{map_data.name} Planner Comparison", color="#f8fafc", fontsize=16, weight="bold")
    fig.tight_layout(rect=(0.02, 0.02, 0.98, 0.93))
    fig.savefig(out_dir / "overview.png", facecolor=fig.get_facecolor())
    plt.close(fig)


def write_summary(map_data: MapData, runs: list[PlannerRun], out_dir: Path) -> None:
    cols = ["planner", "caught", "time", "moves", "cost", "wall_s", "exit_code"]
    with (out_dir / "summary.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=cols)
        writer.writeheader()
        for run in runs:
            writer.writerow({col: run.metrics.get(col, "") for col in cols})

    lines = [
        f"# {map_data.name} Planner Visuals",
        "",
        "| planner | caught | time | moves | cost | wall_s | gif | final_png |",
        "| --- | --- | --- | --- | --- | --- | --- | --- |",
    ]
    for run in runs:
        rel_gif = run.out_dir.relative_to(out_dir) / "animated_panel.gif"
        rel_png = run.out_dir.relative_to(out_dir) / "final_panel.png"
        lines.append(
            "| "
            + " | ".join(
                [
                    run.planner,
                    str(run.metrics.get("caught", "")),
                    str(run.metrics.get("time", "")),
                    str(run.metrics.get("moves", "")),
                    str(run.metrics.get("cost", "")),
                    str(run.metrics.get("wall_s", "")),
                    str(rel_gif),
                    str(rel_png),
                ]
            )
            + " |"
        )
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Render per-planner movement and cost GIF panels.")
    parser.add_argument("--executable", type=Path, default=root / "build" / "run_test")
    parser.add_argument("--maps", nargs="+", default=["map9.txt"])
    parser.add_argument("--planners", nargs="+", default=["hybrid", "multigoal", "bfs", "direct", "greedy"])
    parser.add_argument("--frames", type=int, default=72)
    parser.add_argument("--duration-ms", type=int, default=85)
    parser.add_argument("--timeout-s", type=float, default=45.0)
    parser.add_argument("--output-root", type=Path, default=root / "output" / "visualizations")
    parser.add_argument("--map-style", choices=["planner", "paper", "relief"], default="planner")
    parser.add_argument("--write-overview", action="store_true", help="Also write a combined static overview image.")
    args = parser.parse_args()

    args.output_root.mkdir(parents=True, exist_ok=True)
    for map_name in args.maps:
        map_path = root / "maps" / map_name
        map_data = parse_map_file(map_path)
        map_out_dir = args.output_root / map_path.stem
        map_out_dir.mkdir(parents=True, exist_ok=True)

        runs: list[PlannerRun] = []
        for planner in args.planners:
            planner_out_dir = map_out_dir / planner
            print(f"Running {planner} on {map_name}")
            metrics, robot = run_planner(root, args.executable, map_name, planner, planner_out_dir, args.timeout_s)
            cumulative_cost, distance_to_target = enrich_trajectory(map_data, robot)
            run = PlannerRun(
                planner=planner,
                map_name=map_name,
                metrics=metrics,
                robot=robot,
                cumulative_cost=cumulative_cost,
                distance_to_target=distance_to_target,
                out_dir=planner_out_dir,
            )
            write_static_and_gif(map_data, run, args.frames, args.duration_ms, args.map_style)
            runs.append(run)
            print(f"Wrote {planner_out_dir / 'animated_panel.gif'}")

        write_summary(map_data, runs, map_out_dir)
        if args.write_overview:
            write_overview(map_data, runs, map_out_dir, args.map_style)
            print(f"Wrote {map_out_dir / 'overview.png'}")
        print(f"Wrote {map_out_dir / 'summary.md'}")


if __name__ == "__main__":
    main()
