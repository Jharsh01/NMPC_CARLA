# NMPC Overtaking at the Friction Limit

Nonlinear MPC for an automated overtake of a slower lead car, built on a dynamic
bicycle model with tire-force limits and compared against a pure-pursuit
baseline. Simulation is planned locally in Python and in CARLA.

**Status:** project scaffold only. No controller, simulation or results yet.

## Benchmark

The ego car closes on a slower lead car in its lane on a straight two-lane
road, pulls out, passes, and returns to the original lane. Closing speed, road
friction and the gap at which the overtake starts are swept to find the
smallest gap each controller can handle.

## Layout

| Path | Contents |
| --- | --- |
| `overtake_nmpc/model/` | Vehicle and tire models |
| `overtake_nmpc/controllers/` | NMPC and pure-pursuit controllers |
| `overtake_nmpc/sim/` | Closed-loop simulation (local and CARLA) |
| `overtake_nmpc/scenarios/` | Benchmark scenario definitions and metrics |
| `overtake_nmpc/safety/` | Safety mechanisms wrapping the controller |
| `docs/scenarios.md` | Benchmark scenarios and how the pure-pursuit route is built |
| `docs/safety/` | Functional-safety work products (ISO 26262 concepts) |
| `tests/` | Tests |
| `legacy/` | Original course NMPC formulations this project grew from |

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
pytest
```

Requires Python 3.10 or newer. If `python3 -m venv` is unavailable, use
`uv venv .venv && uv pip install -e ".[dev]"`.

If a ROS 2 environment is sourced in the shell, its pytest plugins break test
collection; run `PYTHONPATH= pytest` instead.
