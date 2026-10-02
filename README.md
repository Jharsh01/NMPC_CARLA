# NMPC Overtaking at the Friction Limit

Nonlinear MPC for an automated overtake of a slower lead car, built on a dynamic
bicycle model with tire-force limits and compared against a pure-pursuit
baseline. Simulation is planned locally in Python and in CARLA.

**Status:** vehicle model and simulation plant implemented and validated. No
controller, CARLA integration or benchmark results yet.

## Vehicle model

A dynamic single-track (bicycle) model written for this project
(`overtake_nmpc/model/`). The same code evaluates on NumPy floats (simulation)
and CasADi symbols (NMPC).

- **State:** `[X, Y, ψ, U_x, U_y, r, δ]`. **Input:** steering rate and total
  longitudinal force `F_x`, split front/rear by a fixed drive and brake ratio.
- **Tires:** Fiala brush model per axle, with lateral capacity reduced by the
  friction circle, `F_y,max = sqrt((μ F_z)² − F_x²)`.
- **Load transfer:** quasi-static, longitudinal only,
  `F_zf = (m g b − h F_x) / L`.
- **Drag:** aerodynamic and rolling resistance.

Left out on purpose: roll, pitch and suspension; left/right load transfer;
wheel-spin dynamics. The model is intended for speeds above about 3 m/s.
Parameters describe CARLA 0.9.15's Tesla Model 3; several are estimates still
to be confirmed (see [docs/vehicle_parameters.md](docs/vehicle_parameters.md)).

The simulation plant (`overtake_nmpc/sim/plant.py`) integrates the same
equations with RK4 at 1 ms and adds what the controller model omits: actuator
saturation, a steering actuator lag and tire force build-up (relaxation
length). It can also be given different parameters from the controller.

### Validation against hand calculations

| Check | Simulated | Hand calculation |
| --- | --- | --- |
| Braking distance 30 → 5 m/s, μ = 0.9 | 106.68 m | 106.68 m |
| Braking distance 30 → 5 m/s, μ = 0.5 | 131.90 m | 131.90 m |
| Steady yaw rate, 10 m/s, δ = 0.002 rad | 0.006656 rad/s | 0.006656 rad/s |
| Steady yaw rate, 20 m/s | 0.013302 rad/s | 0.013302 rad/s |
| Steady yaw rate, 30 m/s | 0.019927 rad/s | 0.019928 rad/s |
| Peak lateral acceleration, ramp steer at 25 m/s | 8.48 m/s² | ≤ μ g = 8.83 m/s² |

Yaw rate is compared with `U_x δ / (L + K U_x²)`, where `K` is the understeer
gradient (about zero for the default parameters). Peak deceleration is
4.10 m/s² on a dry road, set by brake torque, and 3.32 m/s² at μ = 0.5, set by
rear-axle grip under the equal brake split.

These checks show the simulation solves the model's equations correctly. They
do not show that the parameters match CARLA's vehicle; that needs the tests
listed in [docs/vehicle_parameters.md](docs/vehicle_parameters.md).

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
