# Benchmark scenarios

Three reference scenarios live in `overtake_nmpc/scenarios/library.py`, one for
each expected outcome. Each changes as few things as possible from the nominal
pass so a difference in the result has one cause.

| Name | Expected | Road μ | Ego / lead speed | Gap | Loop | What differs |
| --- | --- | --- | --- | --- | --- | --- |
| `pp_edge_fast_loop_model_mismatch` | pure pursuit | 0.8 | 22 / 17 m/s | 35 m | 50 Hz | NMPC assumes μ = 1.0, mass 35 % low, cornering stiffness 30 % high; 80 ms unmodelled actuator delay; any solve over 20 ms fails the run |
| `tie_nominal_highway_pass` | tie | 1.0 | 25 / 20 m/s | 30 m | 10 Hz | nothing: dry road, generous gap, matched model |
| `nmpc_edge_wet_late_pullout_brake_check` | NMPC | 0.5 | 30 / 20 m/s | 12 m | 10 Hz | lead brakes at 4 m/s² for 1.5 s, starting 0.5 s into the pull-out |

## Why each one goes the way it does

**Pure pursuit edge.** The manoeuvre is easy: a lane change within 80 % of the
grip fits well before the lead car. Pure pursuit uses no vehicle model and
runs in microseconds, so it meets every deadline. The NMPC predicts with a
model that is wrong about the car, and at 50 Hz an IPOPT solve is likely to
miss its 20 ms budget. A real-time-iteration NMPC may still pass; report solve-time
percentiles, not only pass/fail.

**Tie.** Both controllers should complete the pass. Compare peak lateral
acceleration, jerk, lane-keeping error, time to complete and compute cost.

**NMPC edge.** At constant speed, even a lane change planned at 100 % of the
grip is not beside the lead car when the bumpers meet (2.1 m of side offset
against the 2.4 m needed). The route was also planned assuming the lead would
hold 20 m/s. A pass that brakes while steering, sharing the friction circle
within 90 % of the grip, does clear the lead car. An NMPC with a
friction-ellipse constraint and a fresh lead-car prediction each step can find
that pass. This needs the dynamic bicycle model with tire limits; the
kinematic model cannot represent μ.

`tests/test_scenarios.py` checks these geometric claims with a point-mass
model, so the scenarios stay discriminating if parameters change.

## Success criteria

A run succeeds when:
- the ego is never alongside the lead car with less than 0.5 m of side
  clearance;
- the ego stays on the road;
- at the end, the ego is back in its lane (|y| < 0.3 m) with its rear 8 m
  ahead of the lead's front;
- no control step misses its solve budget.

## Building the pure-pursuit route

Pure pursuit only tracks a path, so the whole overtake is planned when it
starts:

1. Follow the ego-lane centreline.
2. Blend into the passing-lane centreline over a length `L`.
3. Hold the passing lane until the ego's rear is `return_gap` past the lead's
   front.
4. Blend back over `L` and continue in the ego lane.

The blend weight is the quintic smoothstep `h(τ) = 10τ³ − 15τ⁴ + 6τ⁵`, where
`τ` is the fraction of the blend length covered. Its slope and curvature are
zero at both ends, so steering starts and ends smoothly. Each route point is
`(1 − w)·p_ego_lane + w·p_passing_lane`.

**Lane-change length.** For a lateral offset `d` at speed `v`, the lateral
acceleration peaks at `(10√3/3) · d · v² / L²`. Choosing the friction budget
`a = k · μ · g` (default `k = 0.8`) gives the shortest lane change that stays
within it:

```
L = sqrt(5.77 · d · v² / (k · μ · g))
```

For example, a 3.5 m lane change at 25 m/s on a dry road (μ = 1.0) needs
about 40 m. At 30 m/s on a wet road (μ = 0.5) it needs about 68 m.

**Return point.** With closing speed `v_ego − v_lead` measured when the
overtake starts, the ego reaches the return point after
`(lead centre x + half lengths + return_gap) / (v_ego − v_lead)` seconds. The
route starts blending back at the distance travelled in that time. A planner
that does not model the lead car has to assume its speed stays constant; that
assumption is what the NMPC-edge scenario breaks.

**Speed reference.** `v_ref = min(v_des, sqrt(a / |κ|))`, where `κ` is the
route curvature. On a straight road this is just `v_des`.

Locally:

```python
from overtake_nmpc.scenarios import TIE, plan_overtake_route

route = plan_overtake_route(TIE)  # x, y, s, heading, curvature, v_ref
```

### In CARLA

Take matched lane centrelines from the map, then reuse the same blend.
`get_left_lane()` on each waypoint gives the neighbour in the passing lane,
so the two lists are matched point by point, as `blend_centerlines` requires.
Check that the left lane runs in the same direction (`lane_id` has the same
sign); otherwise it carries oncoming traffic.

```python
import carla
import numpy as np
from overtake_nmpc.scenarios import blend_centerlines, lane_change_length

wp = world.get_map().get_waypoint(ego.get_location())
ego_lane, pass_lane = [], []
for _ in range(int(400 / 1.0)):  # 400 m ahead at 1 m spacing
    left = wp.get_left_lane()
    if (left is None or left.lane_type != carla.LaneType.Driving
            or left.lane_id * wp.lane_id < 0):
        break  # no same-direction passing lane here
    ego_lane.append((wp.transform.location.x, wp.transform.location.y))
    pass_lane.append((left.transform.location.x, left.transform.location.y))
    wp = wp.next(1.0)[0]

v, a_lat = 25.0, 0.8 * mu * 9.81
L = lane_change_length(wp.lane_width, v, a_lat)
route = blend_centerlines(np.array(ego_lane), np.array(pass_lane),
                          s_out=5.0, out_length=L, s_back=s_back, back_length=L,
                          v_des=v, a_lat_max=a_lat)
```

CARLA's map uses a left-handed frame (y points right), so "left lane" is
`get_left_lane()` regardless of the sign of y. Compute `s_back` from the
measured lead-car gap and speeds as above.

### Tracking the route with pure pursuit

At each step:
1. Find the nearest route sample (`route.nearest_index`, searching forward
   from the previous index).
2. Take the point a look-ahead distance `L_d` further along
   (`route.lookahead_index`).
3. Steer with `δ = atan(2 · wheelbase · sin α / L_d)`, where `α` is the angle
   from the car's heading to the look-ahead point, measured at the rear axle.

Schedule the look-ahead with speed, for example
`L_d = clip(0.8 s · v, 6 m, 30 m)`. A fixed look-ahead oscillates at high
speed and cuts corners at low speed. Track `v_ref` with a PI controller on
throttle and brake. Tune both on the tie scenario before comparing, so the
baseline is the best pure pursuit can do.
