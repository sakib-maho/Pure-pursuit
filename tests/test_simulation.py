import math
import unittest

from pure_pursuit.simulation import (
    Pose,
    find_lookahead_point,
    pure_pursuit_steering,
    run_simulation,
    wrap_angle,
)


class PurePursuitTests(unittest.TestCase):
    def test_wrap_angle(self):
        wrapped = wrap_angle(3 * math.pi)
        self.assertAlmostEqual(abs(wrapped), math.pi, places=6)

    def test_lookahead_moves_forward(self):
        path = [(float(i), 0.0) for i in range(20)]
        pose = Pose(x=0.0, y=0.0, yaw=0.0)
        target = find_lookahead_point(path, pose, lookahead_distance=5.0)
        self.assertGreaterEqual(target[0], 5.0)

    def test_steering_turns_left_for_left_target(self):
        pose = Pose(x=0.0, y=0.0, yaw=0.0)
        steering = pure_pursuit_steering(pose, (5.0, 5.0), wheelbase=2.5, lookahead_distance=5.0)
        self.assertGreater(steering, 0.0)

    def test_simulation_reaches_end_of_straight_path(self):
        path = [(float(i), 0.0) for i in range(0, 31)]
        result = run_simulation(
            path,
            Pose(x=0.0, y=0.5, yaw=0.0),
            speed=4.0,
            lookahead_distance=4.0,
            max_steps=400,
            goal_tolerance=1.5,
        )
        self.assertTrue(result["reached"])
        self.assertLess(result["final_distance"], 1.5)


if __name__ == "__main__":
    unittest.main()
