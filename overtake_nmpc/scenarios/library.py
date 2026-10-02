"""The three reference scenarios: one per expected outcome.

Each scenario changes as few things as possible relative to the nominal pass,
so a difference in the result can be traced to one cause.

The pure-pursuit baseline is a path tracker on the route from
:func:`overtake_nmpc.scenarios.route.plan_overtake_route` plus a speed
controller on ``v_ref``. It does not react to the lead car once the route is
planned; that is the baseline being compared, not an oversight.
"""

from __future__ import annotations

from .scenario import (
    Conditions,
    Ego,
    Expected,
    Lead,
    NmpcModel,
    Road,
    Scenario,
    SpeedEvent,
    Success,
)

# Pure pursuit should win: an easy manoeuvre, but a fast control loop and an
# NMPC model that is wrong about the car. Pure pursuit needs no model and
# computes in microseconds, so it completes the pass with every deadline met.
# The NMPC's prediction is biased (heavier car, worn tires, lower friction than
# assumed, unmodelled actuator delay), and any solve longer than 20 ms counts
# as a missed deadline, which fails the run.
PURE_PURSUIT_EDGE = Scenario(
    name="pp_edge_fast_loop_model_mismatch",
    expected=Expected.PURE_PURSUIT,
    summary="Easy pass at 50 Hz with a mis-specified NMPC model",
    road=Road(mu=0.8),
    ego=Ego(speed=22.0, v_des=22.0),
    lead=Lead(speed=17.0),
    gap=35.0,
    duration=20.0,
    conditions=Conditions(
        control_dt=0.02,
        actuation_delay=0.08,
        nmpc_model=NmpcModel(mu_assumed=1.0, mass_scale=1.35, cornering_stiffness_scale=0.7),
    ),
    success=Success(max_missed_deadlines=0),
    notes=(
        "NMPC loses on missed deadlines and on prediction error, not on geometry.",
        "Report solve-time percentiles; a real-time-iteration NMPC may still pass.",
    ),
)

# Both should succeed: dry road, moderate closing speed, generous gap, matched
# model and a 10 Hz loop the NMPC can meet. Compare comfort (peak lateral
# acceleration, jerk), lane-keeping error, time to complete and compute cost.
TIE = Scenario(
    name="tie_nominal_highway_pass",
    expected=Expected.TIE,
    summary="Nominal dry-road pass with a generous gap",
    road=Road(mu=1.0),
    ego=Ego(speed=25.0, v_des=25.0),
    lead=Lead(speed=20.0),
    gap=30.0,
    duration=20.0,
    conditions=Conditions(control_dt=0.1),
)

# NMPC should win: wet road, high closing speed, late pull-out, and the lead
# car brakes hard just after the overtake starts. Even a route planned at the
# full friction limit cannot get the ego beside the lead car in time at
# constant speed, and the route assumed the lead would hold its speed. Braking
# while steering (sharing the friction circle) does clear the lead car within
# 90 % of the available grip, and that is what an NMPC with a friction-ellipse
# constraint and a lead-car prediction can find. Needs the dynamic bicycle
# model with tire limits; the kinematic model cannot represent μ.
NMPC_EDGE = Scenario(
    name="nmpc_edge_wet_late_pullout_brake_check",
    expected=Expected.NMPC,
    summary="Wet road, 12 m gap at 10 m/s closing speed, lead brakes mid-pull-out",
    road=Road(mu=0.5),
    ego=Ego(speed=30.0, v_des=30.0),
    lead=Lead(speed=20.0, events=(SpeedEvent(t_start=0.5, duration=1.5, accel=-4.0),)),
    gap=12.0,
    duration=12.0,
    conditions=Conditions(control_dt=0.1),
    notes=(
        "The NMPC must re-predict the lead car from its measured speed every step.",
    ),
)

SCENARIOS: dict[str, Scenario] = {s.name: s for s in (PURE_PURSUIT_EDGE, TIE, NMPC_EDGE)}
