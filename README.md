# Pure Pursuit Path Following

Python implementation of the **pure pursuit** geometric controller for bicycle-model path following.

## Features

- Lookahead target selection on a waypoint path
- Steering angle from pure pursuit curvature law
- Bicycle-model simulation loop
- CLI demos (`s` curve / circle) with optional CSV trajectory export
- Unit tests for geometry, steering sign, and goal reaching

## Quick start

```bash
python3 -m pip install -e .
python3 cli.py --path s --csv trajectory.csv
python3 -m pytest -q
```

If the package is not installed:

```bash
PYTHONPATH=. python3 cli.py --path circle
PYTHONPATH=. python3 -m unittest discover -s tests -p "test_*.py"
```

## Core idea

For lookahead distance `Ld` and heading error `α`:

`κ = 2 sin(α) / Ld`  
`δ = arctan(L · κ)` where `L` is wheelbase.

## License

MIT
