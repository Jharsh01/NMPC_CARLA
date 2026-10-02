"""Data types that describe one benchmark overtaking scenario.

Frame convention (local simulation): the road runs along +x. The ego lane
centre is y = 0 and the passing lane centre is y = +lane_width (the passing lane
is on the left). At t = 0 the ego centre is at x = 0 and the overtake may start
immediately; ``gap`` is the bumper-to-bumper distance to the lead car at that
moment.
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field

import numpy as np

GRAVITY = 9.81


class Expected(enum.Enum):
    """Which controller the scenario is designed to favour."""

    PURE_PURSUIT = "pure_pursuit"
    TIE = "tie"
    NMPC = "nmpc"


@dataclass(frozen=True)
class Road:
    """Straight two-lane road."""

    lane_width: float = 3.5
    length: float = 1000.0
    mu: float = 1.0  # true tire-road friction coefficient

    @property
    def y_min(self) -> float:
        """Right edge of the ego lane."""
        return -self.lane_width / 2

    @property
    def y_max(self) -> float:
        """Left edge of the passing lane."""
        return 1.5 * self.lane_width


@dataclass(frozen=True)
class SpeedEvent:
    """Constant longitudinal acceleration of the lead car over a time window."""

    t_start: float
    duration: float
    accel: float  # m/s^2, negative for braking


@dataclass(frozen=True)
class Vehicle:
    length: float = 4.7
    width: float = 1.9


@dataclass(frozen=True)
class Lead:
    speed: float  # initial speed, m/s
    events: tuple[SpeedEvent, ...] = ()
    body: Vehicle = Vehicle()

    def accel_at(self, t: float) -> float:
        return sum(e.accel for e in self.events if e.t_start <= t < e.t_start + e.duration)

    def trajectory(self, t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Displacement and speed of the lead car at the (sorted) times ``t``.

        Exact for piecewise-constant acceleration; the lead stops at zero speed.
        """
        t = np.asarray(t, dtype=float)
        knots = {0.0, float(t[-1])}
        for e in self.events:
            knots.update((e.t_start, e.t_start + e.duration))
        knots = np.array(sorted(k for k in knots if 0.0 <= k <= t[-1]))

        # Integrate segment by segment between knots.
        s_k, v_k = [0.0], [self.speed]
        for t0, t1 in zip(knots[:-1], knots[1:]):
            a, dt = self.accel_at(t0), t1 - t0
            v0 = v_k[-1]
            if a < 0 and v0 + a * dt < 0:  # stops inside the segment
                s_k.append(s_k[-1] + v0**2 / (-2 * a))
                v_k.append(0.0)
            else:
                s_k.append(s_k[-1] + v0 * dt + 0.5 * a * dt**2)
                v_k.append(v0 + a * dt)

        idx = np.clip(np.searchsorted(knots, t, side="right") - 1, 0, len(knots) - 1)
        tau = t - knots[idx]
        a = np.array([self.accel_at(k) for k in knots])[idx]
        v0 = np.array(v_k)[idx]
        tau = np.where(a < 0, np.minimum(tau, v0 / np.maximum(-a, 1e-12)), tau)
        s = np.array(s_k)[idx] + v0 * tau + 0.5 * a * tau**2
        v = np.maximum(v0 + a * tau, 0.0)
        return s, v


@dataclass(frozen=True)
class Ego:
    speed: float  # initial speed, m/s
    v_des: float  # cruise speed the controllers try to hold
    body: Vehicle = Vehicle()


@dataclass(frozen=True)
class NmpcModel:
    """Parameters the NMPC prediction model *believes*, relative to the truth.

    The simulator uses the true values; the NMPC uses these. A value of 1.0
    (or ``mu_assumed=None``) means the NMPC model matches the plant.
    """

    mu_assumed: float | None = None  # None: use Road.mu
    mass_scale: float = 1.0  # true mass / modelled mass
    cornering_stiffness_scale: float = 1.0  # true C_alpha / modelled C_alpha


@dataclass(frozen=True)
class Conditions:
    """Execution conditions shared by both controllers."""

    control_dt: float = 0.1  # control period, s
    solve_budget: float | None = None  # wall-clock limit per solve; None = control_dt
    actuation_delay: float = 0.0  # unmodelled delay on steering and throttle/brake, s
    nmpc_model: NmpcModel = NmpcModel()

    @property
    def budget(self) -> float:
        return self.control_dt if self.solve_budget is None else self.solve_budget


@dataclass(frozen=True)
class Success:
    """Pass/fail criteria evaluated on the closed-loop run.

    A run succeeds when, throughout, the ego is never alongside the lead car
    with less than ``lateral_margin`` of side clearance and stays on the road;
    at the end it is back in its lane (``final_lane_tolerance``) with its rear
    ``return_gap`` ahead of the lead's front; and it misses no more than
    ``max_missed_deadlines`` control steps.
    """

    lateral_margin: float = 0.5  # extra side clearance to the lead car, m
    return_gap: float = 8.0  # ego rear to lead front before returning, m
    final_lane_tolerance: float = 0.3  # |y| at the end of the run, m
    max_missed_deadlines: int = 0  # control steps whose solve exceeded the budget


@dataclass(frozen=True)
class Scenario:
    name: str
    expected: Expected
    summary: str
    road: Road
    ego: Ego
    lead: Lead
    gap: float  # bumper-to-bumper distance at t = 0, m
    duration: float  # simulated time, s
    conditions: Conditions = Conditions()
    success: Success = Success()
    notes: tuple[str, ...] = field(default=())

    @property
    def lead_x0(self) -> float:
        """Initial lead-car centre in the ego frame."""
        return self.gap + (self.ego.body.length + self.lead.body.length) / 2

    @property
    def nmpc_mu(self) -> float:
        mu = self.conditions.nmpc_model.mu_assumed
        return self.road.mu if mu is None else mu

    @property
    def required_lateral_offset(self) -> float:
        """Centre-to-centre lateral offset needed to be beside the lead car."""
        return (self.ego.body.width + self.lead.body.width) / 2 + self.success.lateral_margin

    def lead_state(self, t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Lead-car centre x and speed at times ``t`` (lead stays at y = 0)."""
        s, v = self.lead.trajectory(t)
        return self.lead_x0 + s, v
