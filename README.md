# NMPC Overtaking at the Friction Limit

Nonlinear MPC for an automated overtake of a slower lead car, built on a dynamic
bicycle model with tire-force limits and compared against a pure-pursuit
baseline. The model, the plants, the controllers and the scenarios are C++;
Python only draws the animations and drives the tests.

**Status:** on the project's own bicycle plant both controllers are implemented
and tested on four overtaking cases and a lap of a circuit with slower traffic.
The same controllers also run on a multibody sedan from Project Chrono
(Pacejka tires, suspension, left/right load transfer). There the pure-pursuit
baseline completes the lap, but the NMPC does not yet: it passes three of the
four overtaking cases and spins in the wet one and on the circuit (see
[Project Chrono plant](#project-chrono-plant)). CARLA integration is not done.

## Vehicle model

A dynamic single-track (bicycle) model written for this project
(`plant/bicycle.hpp`, `plant/tire.hpp`). The equations are C++ templates, so the
same code evaluates on numbers (simulation) and on CasADi symbols (NMPC).

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

The simulation plant (`plant/bicycle_plant.cpp`) integrates the same
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
(`sim/scenarios/overtake/scenario.cpp`) and shared by every controller.

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
`sim/scenarios/overtake/run.cpp` runs a controller against the plant at
20 Hz, logs at 100 Hz and scores the result.

## Pure-pursuit baseline

`controller/pure_pursuit/` holds the baseline the NMPC is compared against.

- **Path** (`path.cpp`): planned once when the overtake starts, as a lateral
  offset `Y_ref(X)` along the road, from the lead car's position and speed
  (both cars assumed to keep their speed). It is a quintic lane change out, a
  straight section in the passing lane, and a quintic lane change back. Each
  lane change is sized for 2 m/s² peak lateral acceleration. The lane change
  out is shortened if that is needed to clear the lead car by 0.3 m, and the
  path is flagged infeasible if the result needs more grip than the road has.
  The return starts once the ego's rear will be 3 m ahead of the lead car.
- **Steering** (`overtake.cpp`): lookahead point on the path at
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

`controller/nmpc/overtake.cpp` predicts with the same model the plant
integrates, so both controllers work on one state vector
`[X, Y, ψ, U_x, U_y, r, δ]` and one input `[δ_rate, F_x]`.

- **Model:** `plant/bicycle.hpp` built on CasADi symbols (Fiala tires with
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
  course NMPC formulation this project grew from (`legacy/` in the git
  history).
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

`./run.sh` sets up the Python environment and builds the C++ code if needed,
runs a quick check (a few seconds), simulates a scenario with both controllers
and animates it. Without arguments that is the lap of the circuit.

```bash
./run.sh                       # setup, build, quick check, then the lap in a window
./run.sh --plant chrono        # the same on the Project Chrono sedan
./run.sh --save                # write results/circuit.mp4 instead of a window
./run.sh --speed 3             # faster playback
./run.sh overtake late_wet     # an overtaking case, or: overtake all
./run.sh --no-animate          # results only
./run.sh --test                # run every test first, not only the quick check
./run.sh --no-check            # skip the check before the launch
./run.sh --setup-only          # only create the environment and build
```

Without a display it writes the video to `results/`. The two steps can also be
run on their own: `build/simulate` runs the scenario and writes its logs to
`results/logs/`, and `python -m sim.view` animates them.

```bash
build/simulate overtake nominal                  # both controllers, results table, logs
build/simulate overtake all --controller nmpc    # or pure_pursuit, both (default)
python -m sim.view overtake nominal              # opens a window
python -m sim.view overtake relaxed --save results/relaxed.mp4
```

Both controllers drive the same scenario in one animation: pure pursuit in
blue, the NMPC in orange, the lead car in grey, the pure-pursuit reference path
dashed. A car that has finished or crashed is left faded where it stopped. The
bottom panel shows lateral position against time.
Scenarios: `nominal`, `relaxed`, `late_wet`, `slow_closing`.

## Circuit

`sim/scenarios/circuit/scenario.cpp` defines a one-lap circuit: cornering and
passing slower traffic, with the speed chosen from the grip limit.

| Setting | Value |
| --- | --- |
| Lap | 1292 m, clockwise; 300 m top and bottom straights, 330 m sides, and a 60 m outward step on the right side (30 m down, 60 m out, 100 m down, 60 m back, 200 m down) |
| Corner radii | 15 m at the two ends of the 30 m leg, 25–40 m elsewhere |
| Road | two 3.5 m lanes, friction 0.9 |
| Grass | 1.75 m (half a lane) on each side, friction 0.3 |
| Traffic | five cars at a constant 50 km/h and one at 30 km/h, 60 m past the start line; all on the right lane, no dynamics |
| Ego speed limit | 80 km/h |

`track.cpp` holds the geometry: a centreline of straights and arcs by
arc length, with conversion between world and track coordinates.

### NMPC lap

`controller/nmpc/track.cpp` is the same NMPC with the pose
carried in track coordinates: state `[s, e_y, e_ψ, U_x, U_y, r, δ]`, where `s`
is distance along the centreline, `e_y` the lateral offset and `e_ψ` the
heading relative to the track. The dynamics of `U_x, U_y, r, δ` and the inputs
are the plant's. The curvature of each step of the 4 s horizon is read from the
track at the positions of the previous plan.

The cost asks for the 80 km/h limit and the right-hand lane. No corner speed is
prescribed: the plan brakes because the tire model cannot hold the corner
otherwise, with the asphalt edges and 7° of sideslip as soft limits. The two
nearest traffic cars are kept out with the same ellipse as on the straight
road. `sim/scenarios/circuit/run.cpp` runs the lap; the plant's friction follows the surface under
the car's centre of gravity (0.9 on asphalt, 0.3 on grass).

### Pure-pursuit lap

`controller/pure_pursuit/track.cpp` is the baseline on the circuit:

- **Steering:** pure pursuit on a lane line, lookahead `max(0.6 s · U_x, 5 m)`.
- **Speed:** a profile computed once for the lap (`controller/pure_pursuit/speed_profile.cpp`):
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

### Limited, noisy sensing

The table above gives both controllers the exact position and speed of every
traffic car. `sim/perception.cpp` replaces that with detections: a car is only
reported within a range ahead of and behind the ego (measured along the
track), and each report has Gaussian errors along the track, across it and in
speed, drawn afresh every 50 ms. The NMPC predicts each detected car at its
reported speed and offset. The simulation generator uses 60 m ahead, 20 m behind
and errors of 0.5 m, 0.2 m and 0.5 m/s unless `--perfect` is given. The runs
in this table have the exact ego state and the whole road known.

| Sensing | NMPC | Pure pursuit |
| --- | --- | --- |
| exact, all cars | 65.2 s, clearance 0.72 m | 90.5 s, clearance 1.24 m |
| 60 m, default noise (3 seeds) | 65.4–65.5 s, clearance 0.79–0.89 m | 90.5 s, clearance 1.24–1.28 m |
| 60 m, three times the noise | 65.7 s, clearance 0.33 m, 0.07 m to the edge | 90.5 s, clearance 1.00 m |
| 40 m | 65.4 s, clearance 0.81 m | 90.5 s, clearance 0.58 m |
| 30 m | 65.4 s, clearance 0.85 m | hits the slow car at 4.3 s |
| 20 m | 66.0 s, clearance 0.65 m, body 0.31 m over the edge, sideslip 10.8° | hits the slow car at 4.3 s |

Both cope with the default sensor. Pure pursuit needs about 3.5 s of warning
for its lane change, so it fails once the range drops to 30 m. The NMPC still
avoids every car at 20 m but has to swerve hard enough to leave the asphalt.
Not modelled: line of sight round corners, missed or false detections, delay.

### Noisy ego state and a road known only 40 m ahead

The perception also reports the ego's own state with Gaussian errors, drawn
afresh every 50 ms and fed to the controllers unfiltered: 0.1 m in position,
0.005 rad in heading, 0.1 m/s and 0.05 m/s in longitudinal and lateral speed,
0.005 rad/s in yaw rate and 0.002 rad in steering angle. These values are
assumed, not taken from a sensor's data sheet.

The road's shape is known only up to a range ahead of the car, 40 m by
default (`--road-range`). Further on, the controllers take it to continue with the curvature at
the end of the known part, and keep to a speed from which they can still brake
for the tightest corner the circuit has (15 m radius, taken as known in
advance):

- The NMPC limits the speed of the part of its plan beyond the known road to
  what that corner allows at the friction limit (41 km/h).
- Pure pursuit recomputes its speed profile at every step over the known
  road, ending at the speed its grip share allows in that corner (24 km/h).

All runs below use the default traffic sensing (60 m, default noise).

The rows with noise were measured with the earlier Python implementation. The
C++ code draws different random numbers, so its noisy runs differ slightly: the
default run gives 72.2 s for the NMPC and 98.5 s for pure pursuit. Runs with
exact sensing agree with the Python ones to the digits shown.

| Ego state | Road known | NMPC | Pure pursuit |
| --- | --- | --- | --- |
| exact | whole lap | 65.5 s, 46–80 km/h, passes 3 | 90.5 s, 24–80 km/h, passes 1 |
| noisy | whole lap | 65.8 s, 46–80 km/h, passes 3 | 90.5 s, 0.01 m to the asphalt edge |
| exact | 20 m | 83.0 s, 42–59 km/h, passes 2 | 115.9 s, 24–50 km/h, passes 1 |
| noisy (3 seeds) | 20 m | 83.1–83.9 s, clearance 0.80–0.97 m | 115.9–116.0 s, clearance 0.41–0.59 m |
| three times the noise | 20 m | 83.6 s, 0.02 m to the asphalt edge | 116.3 s, body 0.24 m over the edge |
| noisy (the default) | 40 m | 72.2 s, 44–70 km/h, passes 2 | 98.7 s, 24–61 km/h, passes 1 |

No run collides or puts the centre of gravity on the grass. The short road
range costs far more than the noise: the brakes give about 4 m/s², so 20 m of
known road holds the NMPC to 59 km/h and pure pursuit to 50 km/h, the speed of
most of the traffic. The noise mainly eats into the margin to the asphalt
edge. Not modelled: errors that persist from one step to the next (bias,
drift), and the effect of the ego's position error on where it places the
traffic.

```bash
build/simulate circuit                        # both controllers, results, logs
build/simulate circuit --controller nmpc      # or pure_pursuit
build/simulate circuit --perfect              # exact traffic and ego state, whole road known
build/simulate circuit --range 30 --noise 2   # shorter sensor range, twice the noise
build/simulate circuit --road-range 20 --ego-noise 0   # road known 20 m ahead, exact ego state
python -m sim.view circuit --save --speed 3   # writes results/circuit.mp4
python -m sim.view track                      # draw the circuit
python -m sim.view track --animate            # traffic circulating
```

## Project Chrono plant

`plant/chrono_plant.cpp` puts a multibody vehicle from
[Project Chrono](https://projectchrono.org) behind the same plant interface:
the sedan of Chrono's model library on PAC2002 (Pacejka) tires 245/40 R18.
Chrono simulates the chassis in 3D, the double-wishbone and multi-link
suspensions, the steering rack, the driveline, the brakes, each wheel's spin
and each tire's own load, so load transfers left to right as well as front to
rear. Select it with `--plant chrono`.

What the plant class adds to fit the interface:

- **Steering:** the steering rate is integrated to a road-wheel angle, passed
  through a 0.05 s actuator lag and converted to Chrono's steering input.
- **Longitudinal force:** a positive `F_x` becomes a torque on the front axle
  shafts, a negative one a brake command. Chrono's engine and gearbox are left
  out, so the drive force is delivered as asked within the parameter limits.
- **Friction:** the scenario's friction coefficient is passed to Chrono as its
  ground coefficient. Chrono scales the tire's own grip by that value over
  0.8, so 0.9 gives this tire a cornering limit of about 1.1 g, not 0.9 g.

### Identified parameters

The controllers need bicycle-model parameters for this car.
`build/identify_chrono` reads mass, inertia and geometry from the Chrono model
and gets the rest from manoeuvres driven in Chrono, then writes
`parameters_chrono_sedan.json`:

| Parameter | Value | From |
| --- | --- | --- |
| Mass, yaw inertia | 1684 kg, 1324 kg m² | Chrono model |
| Centre of mass to front / rear axle, height | 1.421 m / 1.355 m, 0.419 m | Chrono model |
| Cornering stiffness front / rear | 90.4 / 149.6 kN/rad | steady cornering at 0.1 g |
| Friction coefficient on the dry road | 1.12 | cornering limit on a slippery surface, scaled |
| Rolling resistance | 0.0007 | coasting |
| Peak brake force | 23.3 kN | four brakes at 2000 N m |
| Peak drive force, power, drag area | 6 kN, 150 kW, 0.6 m² | chosen, not from Chrono |

With these parameters the bicycle model's steady yaw rate is within 1–2 % of
Chrono's at 0.2 g, 4–5 % at 0.4 g and 7–10 % at 0.6 g (it turns less than
Chrono does).

### Results on Chrono

| Run | Pure pursuit | NMPC |
| --- | --- | --- |
| Overtake, nominal | pass, clearance 0.47 m | pass, clearance 0.78 m |
| Overtake, relaxed | pass | pass |
| Overtake, late_wet | fails (hits the lead car, as on the bicycle plant) | **fails: spins and leaves the road** |
| Overtake, slow_closing | pass | pass |
| Circuit lap (default sensing) | 98.2 s, clean | **fails: spins off in the first tight corner at 13 s** |

The NMPC as tuned for the bicycle plant is not robust to this car at the
limit. Two things were tried on the circuit:

- A friction margin alone (the model's friction at 0.8, 0.7, 0.6 of the
  identified value) gets further but still ends in a collision or off the
  track at 20–27 s.
- A friction margin of 0.8 together with the brake force limited to 5–8 kN
  completes the lap in 69–71 s, but with sideslip peaks near 30°, the body on
  or over the asphalt edge and about 20 solves that did not converge.

So hard braking while turning is a large part of the problem; the likely cause
is the rear wheels locking under Chrono's equal brake torques, which the
bicycle model's fixed 50/50 force split does not represent. This is not fixed.

## Layout

```
controller/
  interface.hpp           what a simulation needs from a controller
  pure_pursuit/           baseline: overtake (+ path), track (+ speed_profile)
  nmpc/                   NMPC: overtake, track; common.hpp holds the solver set-up
parameters.json           vehicle parameters and the state and input definitions (CARLA Model 3)
parameters_chrono_sedan.json   the same for the Chrono sedan, written by identify_chrono
parameters.hpp/.cpp       loads them
plant/
  interface.hpp           what a simulation needs from a plant
  tire.hpp, bicycle.hpp   tire and vehicle equations, shared with the NMPC
  bicycle_plant.cpp       bicycle plant: those equations plus lags and saturation
  chrono_plant.cpp        plant on the Project Chrono sedan
  identify_chrono.cpp     identifies the bicycle-model parameters of the Chrono sedan
  filters.hpp             first-order lags used by the plants
sim/
  scenarios/overtake/     scenario 1: definition and closed-loop run (C++), animation (Python)
  scenarios/circuit/      scenario 2: track, definition and closed-loop run (C++), animation (Python)
  perception.cpp          what the controllers are told: noise, sensor and road range
  generator.cpp           build/simulate: runs a scenario, reports, writes the logs
  view.py                 animates the logs, writes the video
bindings/                 Python module of the C++ code, for the tests and animations
results/                  logs and videos (not in git)
preflight.py              check to run before launching a simulation
tests/                    tests (pytest, through the Python module); slow ones are marked `slow`
run.sh                    setup, build, check and launch in one step
docs/                     vehicle parameter derivations and the CARLA readout
```

Both controllers implement `Controller` (`controller/interface.hpp`) and every
plant `PlantModel` (`plant/interface.hpp`), so the generator can pair any of them.

## Setup

Needs a C++17 compiler, CMake 3.18 or newer, Eigen 3 and Python 3.10 or newer.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"        # CasADi (with IPOPT), pybind11, matplotlib, pytest
cmake -S . -B build
cmake --build build -j
python preflight.py            # quick check; --full runs every test
```

If `python3 -m venv` is unavailable, use
`uv venv .venv && uv pip install -e ".[dev]"`. `./run.sh --setup-only` does all
of the above.

CasADi is taken from the pip package, whose library is built with the old
libstdc++ string ABI, so the project (and Chrono) is compiled with
`-D_GLIBCXX_USE_CXX11_ABI=0`.

Project Chrono is optional; without it everything but `--plant chrono` works.
The build looks for it in `~/opt/chrono/install`. To build it there (9.0.1,
vehicle module only, about 15 minutes):

```bash
git clone --depth 1 --branch 9.0.1 https://github.com/projectchrono/chrono.git ~/opt/chrono/src
cmake -S ~/opt/chrono/src -B ~/opt/chrono/build -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=$HOME/opt/chrono/install -DENABLE_MODULE_VEHICLE=ON \
    -DBUILD_DEMOS=OFF -DCMAKE_CXX_FLAGS="-D_GLIBCXX_USE_CXX11_ABI=0"
cmake --build ~/opt/chrono/build -j && cmake --install ~/opt/chrono/build
build/identify_chrono          # after building this project: writes parameters_chrono_sedan.json
```

If a ROS 2 environment is sourced in the shell, its pytest plugins break test
collection; run `PYTHONPATH= pytest` instead.
