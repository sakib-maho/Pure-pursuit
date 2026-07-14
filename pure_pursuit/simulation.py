"""Pure pursuit path-following controller and simulation."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float  # radians


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def nearest_waypoint_index(path: list[tuple[float, float]], pose: Pose) -> int:
    return min(range(len(path)), key=lambda i: distance(path[i], (pose.x, pose.y)))


def find_lookahead_point(
    path: list[tuple[float, float]],
    pose: Pose,
    lookahead_distance: float,
) -> tuple[float, float]:
    """Pick a target point approximately lookahead_distance ahead on the path."""
    if not path:
        raise ValueError("path must not be empty")
    if lookahead_distance <= 0:
        raise ValueError("lookahead_distance must be > 0")

    start = nearest_waypoint_index(path, pose)
    # Walk forward until we reach/pass lookahead distance from the vehicle.
    for i in range(start, len(path)):
        if distance(path[i], (pose.x, pose.y)) >= lookahead_distance:
            return path[i]
    return path[-1]


def pure_pursuit_steering(
    pose: Pose,
    target: tuple[float, float],
    wheelbase: float,
    lookahead_distance: float | None = None,
) -> float:
    """
    Compute bicycle-model steering angle (radians) using pure pursuit.

    alpha = angle between vehicle heading and target
    curvature = 2 * sin(alpha) / Ld
    steering = atan(wheelbase * curvature)
    """
    if wheelbase <= 0:
        raise ValueError("wheelbase must be > 0")

    dx = target[0] - pose.x
    dy = target[1] - pose.y
    ld = lookahead_distance if lookahead_distance is not None else math.hypot(dx, dy)
    if ld < 1e-6:
        return 0.0

    alpha = wrap_angle(math.atan2(dy, dx) - pose.yaw)
    curvature = 2.0 * math.sin(alpha) / ld
    return math.atan(wheelbase * curvature)


def step_bicycle(
    pose: Pose,
    speed: float,
    steering: float,
    wheelbase: float,
    dt: float,
) -> Pose:
    """Advance a simple bicycle model one timestep."""
    if dt <= 0:
        raise ValueError("dt must be > 0")
    x = pose.x + speed * math.cos(pose.yaw) * dt
    y = pose.y + speed * math.sin(pose.yaw) * dt
    yaw = wrap_angle(pose.yaw + (speed / wheelbase) * math.tan(steering) * dt)
    return Pose(x=x, y=y, yaw=yaw)


def run_simulation(
    path: list[tuple[float, float]],
    start: Pose,
    *,
    speed: float = 2.0,
    wheelbase: float = 2.5,
    lookahead_distance: float = 5.0,
    dt: float = 0.1,
    max_steps: int = 500,
    goal_tolerance: float = 1.0,
) -> dict:
    """
    Simulate pure pursuit along a waypoint path.

    Returns trajectory samples and whether the final waypoint was reached.
    """
    if len(path) < 2:
        raise ValueError("path needs at least 2 waypoints")

    pose = start
    trajectory: list[dict[str, float]] = []
    goal = path[-1]

    for step in range(max_steps):
        target = find_lookahead_point(path, pose, lookahead_distance)
        steering = pure_pursuit_steering(pose, target, wheelbase, lookahead_distance)
        pose = step_bicycle(pose, speed, steering, wheelbase, dt)
        trajectory.append(
            {
                "step": float(step),
                "x": pose.x,
                "y": pose.y,
                "yaw": pose.yaw,
                "steering": steering,
                "target_x": target[0],
                "target_y": target[1],
            }
        )
        if distance((pose.x, pose.y), goal) <= goal_tolerance:
            return {
                "reached": True,
                "steps": step + 1,
                "final_distance": distance((pose.x, pose.y), goal),
                "trajectory": trajectory,
            }

    return {
        "reached": False,
        "steps": max_steps,
        "final_distance": distance((pose.x, pose.y), goal),
        "trajectory": trajectory,
    }


# Backward-compatible helper used by older CLI/tests.
def run_chaser_demo(
    pursuer: tuple[float, float],
    target: tuple[float, float],
    pursuer_speed: float,
    target_speed: float,
    steps: int,
    dt: float,
) -> dict[str, float | bool]:
    """Legacy chasing demo kept for compatibility."""
    px, py = pursuer
    tx, ty = target
    for _ in range(steps):
        dx = tx - px
        dy = ty - py
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return {"captured": True, "distance": 0.0}
        px += (dx / dist) * pursuer_speed * dt
        py += (dy / dist) * pursuer_speed * dt
        tx -= target_speed * dt
    final = math.hypot(tx - px, ty - py)
    return {"captured": final < 1.0, "distance": final}
