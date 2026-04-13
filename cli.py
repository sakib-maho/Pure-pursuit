"""CLI for pure pursuit simulation."""

from __future__ import annotations

import argparse
import json

from pure_pursuit.simulation import run_simulation


def main() -> int:
    parser = argparse.ArgumentParser(description="Run pure pursuit simulation.")
    parser.add_argument("--steps", type=int, default=150)
    args = parser.parse_args()
    result = run_simulation(
        pursuer=(0.0, 0.0),
        target=(10000.0, 10000.0),
        pursuer_speed=60.0,
        target_speed=50.0,
        steps=args.steps,
        dt=2.0,
    )
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
