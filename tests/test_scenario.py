from dataclasses import replace

import numpy as np
import pytest

from overtake_nmpc.model.bicycle import IDELTA, IPSI, IUX, IUY, IX, IY, NX
from overtake_nmpc.model.params import VehicleParams
from overtake_nmpc.scenarios.overtake import SCENARIOS, OvertakeScenario
from overtake_nmpc.sim.closed_loop import run_overtake
from overtake_nmpc.sim.plant import Plant

SC = OvertakeScenario()
DT = 0.01


def state(X=0.0, Y=0.0, psi=0.0, Ux=SC.v0, Uy=0.0):
    x = np.zeros(NX)
    x[[IX, IY, IPSI, IUX, IUY]] = X, Y, psi, Ux, Uy
    return x


def smoothstep(s):
    s = np.clip(s, 0.0, 1.0)
    return s * s * (3.0 - 2.0 * s)


def lane_offset(t, t_out=0.0, t_back=6.0, t_change=3.0):
    """Lateral reference: out to the passing lane at `t_out`, back at `t_back`."""
    return SC.lane_width * (smoothstep((t - t_out) / t_change) - smoothstep((t - t_back) / t_change))


def kinematic_log(y_of_t, v=SC.v0, t_end=12.0):
    """States of a car at constant speed following Y = y_of_t(t) without sideslip."""
    t = np.arange(0.0, t_end, DT)
    y = y_of_t(t)
    x = np.zeros((len(t), NX))
    x[:, IX], x[:, IY], x[:, IUX] = v * t, y, v
    x[:, IPSI] = np.arctan(np.gradient(y, t) / v)
    return t, x


def test_initial_gap_is_trigger_distance():
    assert SC.clearance(0.0, SC.initial_state()) == pytest.approx(SC.d_trig)
    assert SC.gap_ahead(0.0, SC.initial_state()) == pytest.approx(-SC.d_trig - 2 * SC.car_length)


def test_clearance_alongside_and_overlapping():
    alongside = state(X=SC.lead_x(0.0) + SC.cg_to_center, Y=SC.lane_width)
    assert SC.clearance(0.0, alongside) == pytest.approx(SC.lane_width - SC.car_width)
    assert SC.clearance(0.0, state(X=SC.lead_x(0.0))) == 0.0


def perimeter(corners, n=400):
    s = np.linspace(0.0, 1.0, n)[:, None]
    return np.vstack([c0 + s * (c1 - c0) for c0, c1 in zip(corners, np.roll(corners, -1, axis=0))])


@pytest.mark.parametrize("X, Y, psi", [(0.0, 0.0, 0.3), (30.0, 2.5, -0.2), (45.0, 3.0, 0.4)])
def test_clearance_of_rotated_car_matches_sampled_outlines(X, Y, psi):
    x = state(X=X, Y=Y, psi=psi)
    ego, lead = perimeter(SC.ego_corners(x)), perimeter(SC.lead_corners(0.0))
    sampled = np.linalg.norm(ego[:, None, :] - lead[None, :, :], axis=2).min()
    assert SC.clearance(0.0, x) == pytest.approx(sampled, abs=0.01)


def test_road_margin():
    assert SC.road_margin(state()) == pytest.approx(0.5 * (SC.lane_width - SC.car_width))
    assert SC.road_margin(state(Y=-1.0)) < 0.0
    assert SC.road_margin(state(Y=SC.lane_width + 1.0)) < 0.0


def test_clean_overtake_passes():
    t, x = kinematic_log(lane_offset)
    res = SC.evaluate(t, x)
    assert res.passed and res.failures == ()
    # the ego draws level while still finishing its lane change, so slightly under the lane-centre value
    assert 1.2 < res.min_clearance <= SC.lane_width - SC.car_width + 1e-9
    # complete when the return gap is reached or the lane change back ends, whichever is later
    closing = SC.v0 - SC.v_lead
    t_gap = (SC.d_trig + 2 * SC.car_length + SC.return_gap) / closing
    assert 6.0 < res.t_complete <= max(t_gap, 9.0) + DT
    assert res.distance == pytest.approx(SC.v0 * res.t_complete)


def test_driving_straight_collides():
    t, x = kinematic_log(np.zeros_like)
    res = SC.evaluate(t, x)
    assert res.min_clearance == 0.0
    assert "clearance" in res.failures


def test_following_the_lead_car_does_not_complete():
    t, x = kinematic_log(np.zeros_like, v=SC.v_lead)
    res = SC.evaluate(t, x)
    assert res.failures == ("completed",)
    assert res.t_complete is None and res.distance is None


def test_returning_too_early_fails_clearance():
    t, x = kinematic_log(lambda t: lane_offset(t, t_back=2.5, t_change=2.0))
    assert "clearance" in SC.evaluate(t, x).failures


def test_leaving_the_road_fails():
    t, x = kinematic_log(lambda t: 1.5 * lane_offset(t))
    assert "road" in SC.evaluate(t, x).failures


def test_excess_sideslip_fails():
    t, x = kinematic_log(lane_offset)
    x[100, IUY] = 0.2 * SC.v0
    res = SC.evaluate(t, x)
    assert res.failures == ("sideslip",)
    assert res.peak_sideslip == pytest.approx(np.arctan(0.2))


def test_return_must_hold_for_settle_time():
    # back in lane only briefly before drifting out again
    def y(t):
        return lane_offset(t) + SC.lane_width * smoothstep((t - 8.7) / 3.0)

    t, x = kinematic_log(y)
    assert "completed" in SC.evaluate(t, x).failures


def test_closed_loop_overtake_with_scripted_steering():
    # feedback steering lags its reference, so give it a generous gap
    sc = replace(SC, d_trig=60.0)
    p = replace(VehicleParams(), mu=sc.mu)

    def controller(t, x):
        # steer toward the lateral reference, hold speed
        delta_des = 0.02 * (lane_offset(t, t_back=8.0) - x[IY]) - 0.5 * x[IPSI]
        return [(delta_des - x[IDELTA]) / 0.1, 2.0 * p.m * (sc.v0 - x[IUX])]

    log, res = run_overtake(sc, controller, Plant(p))
    assert res.passed, res
    assert log.t[-1] == pytest.approx(res.t_complete + sc.settle_time, abs=DT)
    assert len(log.t) == len(log.x) == len(log.u)
    assert np.abs(log.u[:, 0]).max() <= p.delta_rate_max


def test_closed_loop_stops_at_collision():
    p = replace(VehicleParams(), mu=SC.mu)
    log, res = run_overtake(SC, lambda t, x: [0.0, 0.0], Plant(p))
    assert "clearance" in res.failures
    assert log.t[-1] < SC.d_trig / (SC.v0 - SC.v_lead) + 1.0


@pytest.mark.parametrize("name", SCENARIOS)
def test_named_scenarios_start_at_the_trigger_gap(name):
    sc = SCENARIOS[name]
    assert sc.v0 > sc.v_lead
    assert sc.clearance(0.0, sc.initial_state()) == pytest.approx(sc.d_trig)
