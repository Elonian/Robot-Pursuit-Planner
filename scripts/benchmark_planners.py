#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import os
import re
import subprocess
import time
from pathlib import Path


RESULT_RE = {
    "caught": re.compile(r"target caught = (\d+)"),
    "time": re.compile(r"time taken \(s\) = (\d+)"),
    "moves": re.compile(r"moves made = (-?\d+)"),
    "cost": re.compile(r"path cost = (-?\d+)"),
}


def parse_result(text: str) -> dict[str, int | str]:
    row: dict[str, int | str] = {}
    for key, regex in RESULT_RE.items():
        match = regex.search(text)
        row[key] = int(match.group(1)) if match else "NA"
    return row


def run_case(executable: Path, map_name: str, planner: str, timeout_s: float) -> dict[str, int | str | float]:
    env = os.environ.copy()
    env["STP_PLANNER"] = planner
    started = time.perf_counter()
    try:
        proc = subprocess.run(
            [str(executable), map_name],
            cwd=executable.parent,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout_s,
            check=False,
        )
        elapsed = time.perf_counter() - started
        row = parse_result(proc.stdout)
        row.update({"planner": planner, "map": map_name, "exit_code": proc.returncode, "wall_s": round(elapsed, 3)})
        return row
    except subprocess.TimeoutExpired as exc:
        elapsed = time.perf_counter() - started
        return {
            "planner": planner,
            "map": map_name,
            "caught": "TIMEOUT",
            "time": "TIMEOUT",
            "moves": "TIMEOUT",
            "cost": "TIMEOUT",
            "exit_code": "TIMEOUT",
            "wall_s": round(elapsed, 3),
        }


def write_markdown(rows: list[dict[str, int | str | float]], path: Path) -> None:
    cols = ["planner", "map", "caught", "time", "moves", "cost", "wall_s", "exit_code"]
    lines = [
        "| " + " | ".join(cols) + " |",
        "| " + " | ".join(["---"] * len(cols)) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(str(row.get(col, "")) for col in cols) + " |")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description="Run planner variants and build a comparison table.")
    parser.add_argument("--executable", type=Path, default=root / "build" / "run_test")
    parser.add_argument("--maps", nargs="+", default=[f"map{i}.txt" for i in range(1, 10)])
    parser.add_argument(
        "--planners",
        nargs="+",
        default=["hybrid", "multigoal", "bfs", "direct", "greedy"],
    )
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument("--output-csv", type=Path, default=root / "output" / "planner_benchmark.csv")
    parser.add_argument("--output-md", type=Path, default=root / "output" / "planner_benchmark.md")
    args = parser.parse_args()

    rows: list[dict[str, int | str | float]] = []
    for planner in args.planners:
        for map_name in args.maps:
            row = run_case(args.executable, map_name, planner, args.timeout_s)
            rows.append(row)
            print(row)

    args.output_csv.parent.mkdir(parents=True, exist_ok=True)
    cols = ["planner", "map", "caught", "time", "moves", "cost", "wall_s", "exit_code"]
    with args.output_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=cols)
        writer.writeheader()
        writer.writerows(rows)
    write_markdown(rows, args.output_md)
    print(f"Wrote {args.output_csv}")
    print(f"Wrote {args.output_md}")


if __name__ == "__main__":
    main()
