"""Minimal pure pursuit style chaser-target simulation."""

from __future__ import annotations

import math


def run_simulation(
    pursuer: tuple[float, float],
    target: tuple[float, float],
    pursuer_speed: float,
    target_speed: float,
    steps: int,
    dt: float,
) -> dict[str, float | bool]:
    px, py = pursuer
    tx, ty = target

    for _ in range(steps):
        dx = tx - px
        dy = ty - py
        distance = math.hypot(dx, dy)
        if distance < 1e-6:
            return {"captured": True, "distance": 0.0}

        px += (dx / distance) * pursuer_speed * dt
        py += (dy / distance) * pursuer_speed * dt
        tx -= target_speed * dt

    final_distance = math.hypot(tx - px, ty - py)
    return {"captured": final_distance < 1000.0, "distance": final_distance}
