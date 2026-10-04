# NMPC Overtaking at the Friction Limit

Nonlinear MPC for an automated overtake of a slower lead car, built on a dynamic
bicycle model with tire-force limits and compared against a pure-pursuit
baseline. Simulation is planned locally in Python and in CARLA.

**Status:** vehicle model, simulation plant, benchmark scenario, the
pure-pursuit baseline and the NMPC implemented and tested on four overtaking
cases, and the NMPC drives a lap of a circuit with slower traffic. CARLA
integration and benchmark sweeps are not done yet.

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

The scenario and its scoring are defined once
(`overtake_nmpc/scenarios/overtake.py`) and shared by every controller.

| Setting | Nominal value |
| --- | --- |
| Road | Straight, two same-direction lanes, 3.5 m wide |
| Lead car | 50 km/h, constant, centred in the ego lane |
| Ego entry speed | 90 km/h |
| Gap when the overtake starts | 30 m, bumper to bumper |
| Road friction | 0.9 |
| Car bodies | 4.79 × 2.16 m rectangles (CARLA Model 3 bounding box) |

A run passes if all four hold:

- **Clearance:** the car bodies never come closer than 0.3 m.
- **Road:** the ego body stays on the two-lane road.
- **Sideslip:** stays within 10°.
- **Completed:** within 30 s the ego is at least 10 m ahead of the lead car,
  within 0.3 m of its lane centre and 2° of the road heading, and holds that
  for 1 s.

Reported per run: minimum clearance, margin to the road edge, peak sideslip,
and the time and distance to complete the overtake.
`overtake_nmpc/sim/closed_loop.py` runs a controller against the plant at
20 Hz, logs at 100 Hz and scores the result.

## Pure-pursuit baseline

`overtake_nmpc/controllers/` holds the baseline the NMPC is compared against.

- **Path** (`path.py`): planned once when the overtake starts, as a lateral
  offset `Y_ref(X)` along the road, from the lead car's position and speed
  (both cars assumed to keep their speed). It is a quintic lane change out, a
  straight section in the passing lane, and a quintic lane change back. Each
  lane change is sized for 2 m/s² peak lateral acceleration. The lane change
  out is shortened if that is needed to clear the lead car by 0.3 m, and the
  path is flagged infeasible if the result needs more grip than the road has.
  The return starts once the ego's rear will be 3 m ahead of the lead car.
- **Steering** (`pure_pursuit.py`): lookahead point on the path at
  `L_d = max(1.4 s · U_x, 5 m)` from the rear axle,
  `δ = atan(2 L sin α / L_d)`, sent to the plant as a steering-rate command.
- **Speed:** proportional control of `F_x` to hold the entry speed.

Lookahead and steering gains were tuned on the nominal case only. A shorter
lookahead tracks more tightly but overshoots the passing lane and leaves the
road; a longer one cuts the lane change and comes too close to the lead car.

| Case | Result | Min clearance | Completed |
| --- | --- | --- | --- |
| nominal (90 km/h, 30 m, μ 0.9) | pass | 0.97 m | 9.0 s |
| relaxed (80 km/h, 60 m, μ 0.9) | pass | 1.26 m | 13.7 s |
| slow_closing (60 km/h, 30 m, μ 0.9) | pass | 1.34 m | 19.9 s |
| late_wet (110 km/h, 20 m, μ 0.5) | fail: hits the lead car | 0 | – |

In the nominal case the car overshoots the passing lane by about 0.5 m and
passes the road-edge check with only 0.05 m to spare. In `late_wet` the
planned path already needs more grip than the road has; with the tuned
lookahead the car also responds too slowly, and a much shorter lookahead
reaches the grip limit and still hits the lead car.

## NMPC

`overtake_nmpc/controllers/nmpc.py` predicts with the same model the plant
integrates, so both controllers work on one state vector
`[X, Y, ψ, U_x, U_y, r, δ]` and one input `[δ_rate, F_x]`.

- **Model:** `model/bicycle.py` built on CasADi symbols (Fiala tires with
  friction-circle derating, load transfer), one RK4 step per 0.1 s over a 5 s
  horizon (50 steps). The plant's steering lag and tire relaxation are not in
  the controller's model.
- **Constraints:** an ellipse around the lead car (12.7 × 3.1 m semi-axes,
  sized to keep the bodies 0.6 m apart side by side and 1 m nose to tail), the
  front and rear ends of the body 0.3 m inside the road edges, sideslip within
  70 % of the 10° limit, steering angle and rate, and the force range from
  `fx_limits`. The first three are soft.
- **Cost:** hold the entry speed and the ego-lane centre; penalise lateral
  velocity, yaw rate, `F_x` and steering rate. The structure follows the
  course formulation in `legacy/nmpc_takeover_student.py`.
- **Solver:** IPOPT through CasADi, re-solved at 20 Hz and warm-started from
  the previous solution.

| Case | Pure pursuit | NMPC |
| --- | --- | --- |
| nominal | pass, clearance 0.97 m, road margin 0.05 m, 9.0 s | pass, clearance 0.77 m, road margin 0.66 m, 8.9 s |
| relaxed | pass, 1.26 m, 0.17 m, 13.7 s | pass, 0.77 m, 0.67 m, 13.7 s |
| slow_closing | pass, 1.34 m, 0.31 m, 19.9 s | pass, 0.74 m, 0.67 m, 21.0 s |
| late_wet | fail: hits the lead car at 1.2 s | pass, 0.37 m, 0.30 m, 8.5 s, peak sideslip 7.3° |

The NMPC is the only one that gets through `late_wet`, with 0.07 m to spare on
the 0.3 m clearance criterion. Its solves take about 60 ms on average (up to
200 ms) on a laptop, longer than the 50 ms control period, so it is not
real-time yet; the simulation does not model that delay.

### Watching a run

`./run.sh` does everything in one go: creates `.venv` and installs the
package if needed, runs the tests, prints a results table for every scenario
and controller, and animates the scenarios one after another (close a window
to see the next).

```bash
./run.sh                          # setup, tests, all scenarios
./run.sh late_wet --speed 0.25    # one scenario in slow motion
./run.sh --skip-tests --save      # write results/<scenario>.gif instead of windows
./run.sh --setup-only             # only create/update the environment
```

Without a display it writes GIFs to `results/`. The viewer can also be run
directly:

```bash
python scripts/watch_overtake.py nominal            # opens a window
python scripts/watch_overtake.py all --no-animate   # results table only
python scripts/watch_overtake.py nominal --controller nmpc   # or pure_pursuit, both (default)
python scripts/watch_overtake.py relaxed --save results/relaxed.gif
```

Both controllers drive the same scenario in one animation: pure pursuit in
blue, the NMPC in orange, the lead car in grey, the pure-pursuit reference path
dashed. A car that has finished or crashed is left faded where it stopped. The
bottom panel shows lateral position against time.
Scenarios: `nominal`, `relaxed`, `late_wet`, `slow_closing`.

## Circuit

`overtake_nmpc/scenarios/circuit.py` defines a one-lap circuit: cornering and
passing slower traffic, with the speed chosen from the grip limit.

| Setting | Value |
| --- | --- |
| Lap | 1292 m, clockwise; 300 m top and bottom straights, 330 m sides, and a 60 m outward step on the right side (30 m down, 60 m out, 100 m down, 60 m back, 200 m down) |
| Corner radii | 15 m at the two ends of the 30 m leg, 25–40 m elsewhere |
| Road | two 3.5 m lanes, friction 0.9 |
| Grass | 1.75 m (half a lane) on each side, friction 0.3 |
| Traffic | five cars at a constant 50 km/h and one at 30 km/h, 60 m past the start line; all on the right lane, no dynamics |
| Ego speed limit | 80 km/h |

`scenarios/track.py` holds the geometry: a centreline of straights and arcs by
arc length, with conversion between world and track coordinates.

### NMPC lap

`overtake_nmpc/controllers/nmpc_track.py` is the same NMPC with the pose
carried in track coordinates: state `[s, e_y, e_ψ, U_x, U_y, r, δ]`, where `s`
is distance along the centreline, `e_y` the lateral offset and `e_ψ` the
heading relative to the track. The dynamics of `U_x, U_y, r, δ` and the inputs
are the plant's. The curvature of each step of the 4 s horizon is read from the
track at the positions of the previous plan.

The cost asks for the 80 km/h limit and the right-hand lane. No corner speed is
prescribed: the plan brakes because the tire model cannot hold the corner
otherwise, with the asphalt edges and 7° of sideslip as soft limits. The two
nearest traffic cars are kept out with the same ellipse as on the straight
road. `sim/lap.py` runs the lap; the plant's friction follows the surface under
the car's centre of gravity (0.9 on asphalt, 0.3 on grass).

### Pure-pursuit lap

`controllers/pure_pursuit_track.py` is the baseline on the circuit:

- **Steering:** pure pursuit on a lane line, lookahead `max(0.6 s · U_x, 5 m)`.
- **Speed:** a profile computed once for the lap (`controllers/speed_profile.py`):
  the corner limit `sqrt(a / |κ|)` on the tighter of the two lanes, then a
  backward pass for braking and a forward pass for accelerating, sharing the
  budget `a = grip_use · μ g` with cornering and capped by the brake force.
- **Passing:** a rule, since pure pursuit knows nothing about other cars: move
  to the left lane while a traffic car in the right lane is between 15 m behind
  and 30 m ahead, then return. The distance ahead grows with the closing speed
  so that 3.5 s are left for the lane change.

### Comparison

One lap from 50 km/h on the start line, same plant, same traffic. The two cars
do not see each other.

| | NMPC | Pure pursuit |
| --- | --- | --- |
| Lap time | 65.2 s | 90.5 s |
| Speed range | 47–80 km/h | 24–80 km/h |
| Traffic cars passed | 3 of 6 | 1 of 6 (the slow one) |
| Closest to a traffic car | 0.72 m | 1.24 m |
| Closest to the asphalt edge | 0.19 m | 0.12 m |
| Time on the grass | 0 s | 0 s |
| Peak sideslip | 7.0° | 5.8° |
| Compute per step | 53 ms mean, 95 ms max | negligible |

The pure-pursuit profile uses 40 % of the grip (`grip_use = 0.4`). That is the
fastest of the settings tried (0.3 to 0.9 in steps of 0.1) that keeps the whole
body on the asphalt. Above it the car runs wide in corners, because the
steering law assumes the car goes where it points and at speed it slides
outward: at 0.8 the lap takes 75.4 s with the body up to 0.63 m over the edge,
and a shorter lookahead spins. The NMPC plans with the tire model, so it can
corner near the limit, use the full road width and still hold the edge. Both
pass the 30 km/h car on the top straight. Of the 50 km/h cars the NMPC reaches
two in one lap; the pure-pursuit car is too slow in the corners to catch any.

The NMPC is not real-time yet: its solves take longer than the 50 ms control
period, and the simulation does not model that delay.

```bash
python scripts/run_lap.py                     # both controllers, then the animation
python scripts/run_lap.py --controller nmpc   # or pure_pursuit
python scripts/run_lap.py --save results/lap.gif --speed 3
python scripts/show_track.py                  # draw the circuit
python scripts/show_track.py --animate        # traffic circulating
```

## Layout

| Path | Contents |
| --- | --- |
| `overtake_nmpc/model/` | Vehicle and tire models |
| `overtake_nmpc/controllers/` | NMPC and pure-pursuit controllers |
| `overtake_nmpc/sim/` | Closed-loop simulation (local and CARLA) |
| `overtake_nmpc/scenarios/` | Benchmark scenario definitions and metrics |
| `overtake_nmpc/safety/` | Safety mechanisms wrapping the controller |
| `docs/safety/` | Functional-safety work products (ISO 26262 concepts) |
| `run.sh` | One-step setup, tests and animated scenario runs |
| `scripts/` | Command-line tools (animation viewer) |
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
