# Pure Pursuit Simulation

This repository is upgraded into a reproducible pure pursuit simulation project.
The notebook stays available, while the repo now includes reusable simulation logic, CLI, and tests.

## Features

- Simplified pursuer-target simulation model
- Configurable simulation step count via CLI
- JSON output for simulation results
- Unit tests for simulation and CLI

## Usage

```bash
python3 cli.py --steps 150
```

## Tests

```bash
python3 -m unittest discover -s tests -p "test_*.py"
```

## License

MIT License. See `LICENSE`.
