"""CLI for pure pursuit path-following simulation."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

from pure_pursuit.simulation import Pose, run_simulation


def build_circle_path(radius: float = 20.0, points: int = 60) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2 * math.pi * i / points),
            radius * math.sin(2 * math.pi * i / points),
        )
        for i in range(points)
    ]


def build_s_path() -> list[tuple[float, float]]:
    path: list[tuple[float, float]] = []
    for i in range(40):
        x = i * 1.0
        y = 8.0 * math.sin(i / 6.0)
        path.append((x, y))
    return path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Pure pursuit path-following demo")
    parser.add_argument("--path", choices=["circle", "s"], default="s")
    parser.add_argument("--speed", type=float, default=3.0)
    parser.add_argument("--lookahead", type=float, default=6.0)
    parser.add_argument("--wheelbase", type=float, default=2.5)
    parser.add_argument("--dt", type=float, default=0.1)
    parser.add_argument("--max-steps", type=int, default=800)
    parser.add_argument("--csv", type=Path, default=None, help="Optional trajectory CSV output")
    args = parser.parse_args(argv)

    path = build_circle_path() if args.path == "circle" else build_s_path()
    start = Pose(x=path[0][0], y=path[0][1] - 2.0, yaw=0.0)
    result = run_simulation(
        path,
        start,
        speed=args.speed,
        wheelbase=args.wheelbase,
        lookahead_distance=args.lookahead,
        dt=args.dt,
        max_steps=args.max_steps,
    )

    print(f"path={args.path} reached={result['reached']} steps={result['steps']}")
    print(f"final_distance={result['final_distance']:.3f}")

    if args.csv:
        with args.csv.open("w", newline="") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=["step", "x", "y", "yaw", "steering", "target_x", "target_y"],
            )
            writer.writeheader()
            writer.writerows(result["trajectory"])
        print(f"wrote {args.csv}")

    return 0 if result["reached"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
