from subprocess import run
import json
import unittest

from pure_pursuit.simulation import run_simulation


class PurePursuitTests(unittest.TestCase):
    def test_simulation_returns_distance(self) -> None:
        result = run_simulation((0, 0), (1000, 0), 60, 30, steps=50, dt=1)
        self.assertIn("distance", result)
        self.assertIsInstance(result["captured"], bool)

    def test_cli(self) -> None:
        process = run(
            ["python3", "cli.py", "--steps", "120"],
            check=True,
            text=True,
            capture_output=True,
        )
        payload = json.loads(process.stdout)
        self.assertIn("distance", payload)


if __name__ == "__main__":
    unittest.main()
