"""Reference route for the pure-pursuit baseline.

Pure pursuit only tracks a path, so the overtake has to be planned up front:

1. stay on the ego-lane centreline,
2. blend into the passing-lane centreline over ``out_length``,
3. hold the passing lane until the ego is ``return_gap`` ahead of the lead car,
4. blend back over ``back_length`` and continue in the ego lane.

The blends use the quintic smoothstep ``h(τ) = 10τ³ − 15τ⁴ + 6τ⁵``, which has
zero slope and curvature at both ends. On a straight road the lateral
acceleration at speed v peaks at ``a = (10√3/3) · d · v² / L²`` for an offset d
over length L, so the shortest lane change that stays within a lateral
acceleration budget is ``L = sqrt((10√3/3) · d · v² / a)``.

:func:`blend_centerlines` works on any pair of matched lane centrelines, so the
same code builds the route in CARLA from map waypoints (see
``docs/scenarios.md``). :func:`plan_overtake_route` builds it for the local
straight-road scenarios.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .scenario import GRAVITY, Scenario

QUINTIC_PEAK_ACCEL = 10 * np.sqrt(3) / 3  # max |h''(τ)| of the quintic smoothstep


def smoothstep5(tau: np.ndarray) -> np.ndarray:
    tau = np.clip(tau, 0.0, 1.0)
    return tau**3 * (10 - 15 * tau + 6 * tau**2)


def lane_change_length(offset: float, speed: float, a_lat_max: float) -> float:
    """Shortest quintic lane change that keeps lateral acceleration ≤ ``a_lat_max``."""
    return float(np.sqrt(QUINTIC_PEAK_ACCEL * abs(offset) * speed**2 / a_lat_max))


@dataclass(frozen=True)
class Route:
    """Densely sampled path with a speed reference."""

    x: np.ndarray
    y: np.ndarray
    s: np.ndarray  # arc length
    heading: np.ndarray
    curvature: np.ndarray
    v_ref: np.ndarray

    def nearest_index(self, px: float, py: float, start: int = 0, window: int = 200) -> int:
        """Closest sample to (px, py), searched forward from ``start``."""
        stop = min(start + window, len(self.x))
        d2 = (self.x[start:stop] - px) ** 2 + (self.y[start:stop] - py) ** 2
        return start + int(np.argmin(d2))

    def lookahead_index(self, nearest: int, distance: float) -> int:
        """First sample at least ``distance`` of arc length past ``nearest``."""
        target = self.s[nearest] + distance
        return min(int(np.searchsorted(self.s, target)), len(self.s) - 1)


def _arc_length(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    return np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))))


def blend_centerlines(
    lane_from: np.ndarray,
    lane_to: np.ndarray,
    s_out: float,
    out_length: float,
    s_back: float,
    back_length: float,
    v_des: float,
    a_lat_max: float,
    ds: float = 0.5,
) -> Route:
    """Route that leaves ``lane_from`` for ``lane_to`` and comes back.

    ``lane_from`` and ``lane_to`` are (N, 2) centreline points matched row by
    row (row i of ``lane_to`` is the neighbour of row i of ``lane_from``, as
    returned by CARLA's ``waypoint.get_left_lane()``). Positions along the
    manoeuvre (``s_out``, ``s_back``) are arc lengths along ``lane_from``.
    """
    lane_from = np.asarray(lane_from, dtype=float)
    lane_to = np.asarray(lane_to, dtype=float)
    if lane_from.shape != lane_to.shape or lane_from.shape[1] != 2:
        raise ValueError("centrelines must both be (N, 2) and matched row by row")
    if s_back < s_out + out_length:
        raise ValueError("return starts before the pull-out has finished")

    s_from = _arc_length(lane_from[:, 0], lane_from[:, 1])
    s = np.arange(0.0, s_from[-1], ds)
    p0 = np.column_stack([np.interp(s, s_from, lane_from[:, i]) for i in range(2)])
    p1 = np.column_stack([np.interp(s, s_from, lane_to[:, i]) for i in range(2)])

    w = smoothstep5((s - s_out) / out_length) - smoothstep5((s - s_back) / back_length)
    p = (1 - w)[:, None] * p0 + w[:, None] * p1
    x, y = p[:, 0], p[:, 1]

    dx, dy = np.gradient(x), np.gradient(y)
    ddx, ddy = np.gradient(dx), np.gradient(dy)
    heading = np.unwrap(np.arctan2(dy, dx))
    curvature = (dx * ddy - dy * ddx) / np.maximum(np.hypot(dx, dy) ** 3, 1e-12)
    v_ref = np.minimum(v_des, np.sqrt(a_lat_max / np.maximum(np.abs(curvature), 1e-9)))
    return Route(x, y, _arc_length(x, y), heading, curvature, v_ref)


def plan_overtake_route(
    scenario: Scenario,
    friction_utilization: float = 0.8,
    s_out: float = 0.0,
    ds: float = 0.5,
) -> Route:
    """Pure-pursuit route for a straight-road scenario, planned at t = 0.

    The pull-out and return lengths are the shortest quintics whose lateral
    acceleration stays within ``friction_utilization · μ · g`` at ``v_des``.
    The return point is predicted from the lead car's *current* speed, which is
    what a path planner without a lead-car model has to assume; if the lead
    later changes speed the route does not adapt.
    """
    road, ego, lead = scenario.road, scenario.ego, scenario.lead
    v = ego.v_des
    closing = v - lead.speed
    if closing <= 0:
        raise ValueError("ego cruise speed must exceed the lead speed to overtake")

    a_lat_max = friction_utilization * road.mu * GRAVITY
    length = lane_change_length(road.lane_width, v, a_lat_max)

    # Ego rear must be return_gap past the lead's front before turning back in.
    rel_distance = scenario.lead_x0 + (ego.body.length + lead.body.length) / 2 + scenario.success.return_gap
    s_back = max(v * rel_distance / closing, s_out + length)

    x = np.arange(-50.0, road.length + ds, ds)
    lane_from = np.column_stack([x, np.zeros_like(x)])
    lane_to = np.column_stack([x, np.full_like(x, road.lane_width)])
    route = blend_centerlines(lane_from, lane_to, s_out + 50.0, length, s_back + 50.0, length, v, a_lat_max, ds)
    return route
