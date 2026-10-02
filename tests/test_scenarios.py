import numpy as np
import pytest

from overtake_nmpc.scenarios import (
    NMPC_EDGE,
    PURE_PURSUIT_EDGE,
    SCENARIOS,
    TIE,
    Lead,
    SpeedEvent,
    blend_centerlines,
    lane_change_length,
    plan_overtake_route,
)
from overtake_nmpc.scenarios.scenario import GRAVITY

DT = 0.01


def _overlap(sc, x_ego, t):
    x_lead, _ = sc.lead_state(t)
    return np.abs(x_ego - x_lead) < (sc.ego.body.length + sc.lead.body.length) / 2


def route_clearance(sc, friction_utilization=0.8):
    """Smallest lateral offset while alongside the lead, ego on the route at v_des."""
    route = plan_overtake_route(sc, friction_utilization)
    t = np.arange(0.0, sc.duration, DT)
    x = sc.ego.v_des * t
    y = np.interp(x - route.x[0], route.s, route.y)
    ov = _overlap(sc, x, t)
    return y[ov].min() if ov.any() else np.inf


def best_braking_pass(sc, utilization=0.9):
    """Best side clearance of brake-while-steering passes within the friction circle.

    Point-mass search: constant braking for t_brake seconds, bang-bang lateral
    acceleration into the passing lane with what is left of the grip. Only
    passes that end with the ego well ahead of the lead count.
    """
    a_max = utilization * sc.road.mu * GRAVITY
    t = np.arange(0.0, sc.duration, DT)
    x_lead, _ = sc.lead_state(t)
    best = -np.inf
    for frac in np.linspace(0.0, 0.95, 40):
        a_brake = frac * a_max
        a_lat = np.sqrt(a_max**2 - a_brake**2)
        t_lat = np.sqrt(4 * sc.road.lane_width / a_lat)
        ay = np.where(t < t_lat / 2, a_lat, np.where(t < t_lat, -a_lat, 0.0))
        y = np.cumsum(np.cumsum(ay) * DT) * DT
        for t_brake in np.linspace(0.0, 6.0, 61):
            v = sc.ego.speed - np.cumsum(np.where(t < t_brake, a_brake, 0.0)) * DT
            x = np.cumsum(v) * DT
            ov = _overlap(sc, x, t)
            if ov.any() and x[-1] > x_lead[-1] + 10.0:
                best = max(best, y[ov].min())
    return best


def test_registry_has_one_scenario_per_outcome():
    assert {s.expected for s in SCENARIOS.values()} == {s.expected for s in (PURE_PURSUIT_EDGE, TIE, NMPC_EDGE)}
    assert len(SCENARIOS) == 3


@pytest.mark.parametrize("sc", [PURE_PURSUIT_EDGE, TIE])
def test_pure_pursuit_route_clears_lead_in_easy_scenarios(sc):
    assert route_clearance(sc) >= sc.required_lateral_offset


def test_nmpc_edge_defeats_any_friction_limited_route():
    # Even a route planned at 100 % of the grip is not beside the lead in time.
    assert route_clearance(NMPC_EDGE, friction_utilization=1.0) < NMPC_EDGE.required_lateral_offset


def test_nmpc_edge_has_a_friction_feasible_pass():
    assert best_braking_pass(NMPC_EDGE) >= NMPC_EDGE.required_lateral_offset


def test_route_lateral_acceleration_within_budget():
    sc = TIE
    route = plan_overtake_route(sc, friction_utilization=0.8)
    a_lat = sc.ego.v_des**2 * np.abs(route.curvature)
    assert a_lat.max() <= 0.8 * sc.road.mu * GRAVITY * 1.02
    assert route.y.min() >= -1e-9 and route.y.max() <= sc.road.lane_width + 1e-9
    assert abs(route.y[-1]) < 1e-9


def test_route_returns_after_passing_lead():
    sc = TIE
    route = plan_overtake_route(sc)
    closing = sc.ego.v_des - sc.lead.speed
    rel = sc.lead_x0 + (sc.ego.body.length + sc.lead.body.length) / 2 + sc.success.return_gap
    x_back = route.x[0] + 50.0 + sc.ego.v_des * rel / closing  # route starts 50 m behind the ego
    at = np.searchsorted(route.x, x_back)
    assert route.y[at - 2] == pytest.approx(sc.road.lane_width)


def test_blend_on_curved_road_stays_between_lanes():
    theta = np.linspace(0, np.pi / 2, 400)
    inner = np.column_stack([100 * np.cos(theta), 100 * np.sin(theta)])
    outer = np.column_stack([103.5 * np.cos(theta), 103.5 * np.sin(theta)])
    route = blend_centerlines(inner, outer, 20.0, 40.0, 80.0, 40.0, v_des=15.0, a_lat_max=5.0)
    r = np.hypot(route.x, route.y)
    assert r.min() >= 100 - 1e-3 and r.max() <= 103.5 + 1e-3
    assert route.v_ref.max() <= 15.0


def test_lane_change_length_formula():
    assert lane_change_length(3.5, 20.0, 4.0) == pytest.approx(np.sqrt(10 * np.sqrt(3) / 3 * 3.5 * 400 / 4.0))


def test_lead_trajectory_with_brake_to_stop():
    lead = Lead(speed=10.0, events=(SpeedEvent(1.0, 10.0, -5.0),))
    s, v = lead.trajectory(np.array([0.0, 1.0, 2.0, 3.0, 5.0]))
    np.testing.assert_allclose(v, [10, 10, 5, 0, 0])
    np.testing.assert_allclose(s, [0, 10, 17.5, 20, 20])
