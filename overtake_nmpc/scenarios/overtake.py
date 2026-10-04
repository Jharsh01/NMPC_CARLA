"""Overtaking benchmark: road layout, lead car, pass criteria and metrics.

Road frame: X along the road, Y to the left. The ego lane is centred on Y = 0
and the passing lane on Y = lane_width; both lanes run in the same direction.
The lead car holds the ego-lane centre at constant speed. The ego starts at
X = 0 with its front bumper `d_trig` behind the lead car's rear bumper, and
has to pass and return to the ego lane ahead of the lead car.

Both cars are rectangles of the same size. The ego state [X, Y, psi, ...] is
its centre of gravity, which sits `cg_to_center` ahead of the body centre.
"""

from dataclasses import dataclass

import numpy as np

from ..model.bicycle import IPSI, IUX, IUY, IX, IY, NX


@dataclass(frozen=True)
class OvertakeResult:
    passed: bool
    failures: tuple  # names of the criteria that failed
    min_clearance: float  # smallest distance between the two car bodies [m]
    road_margin: float  # smallest distance from the ego body to a road edge [m], negative if off the road
    peak_sideslip: float  # largest |atan(Uy / Ux)| [rad]
    t_complete: float | None  # time at which the ego is back in lane and stays there [s]
    distance: float | None  # distance travelled by the ego up to t_complete [m]


@dataclass(frozen=True)
class OvertakeScenario:
    v0: float = 90 / 3.6  # ego entry speed [m/s]
    v_lead: float = 50 / 3.6  # lead car speed [m/s]
    d_trig: float = 30.0  # bumper-to-bumper gap at which the overtake starts [m]
    mu: float = 0.9  # road friction coefficient [-]
    lane_width: float = 3.5  # [m]
    car_length: float = 4.79  # CARLA Model 3 bounding box [m]
    car_width: float = 2.16  # CARLA Model 3 bounding box [m]
    cg_to_center: float = 0.45  # centre of gravity ahead of the body centre [m]
    t_max: float = 30.0  # time allowed for the overtake [s]

    # pass criteria
    clearance_min: float = 0.3  # between the car bodies [m]
    sideslip_max: float = np.radians(10.0)  # [rad]
    return_gap: float = 10.0  # ego rear bumper ahead of lead front bumper [m]
    y_tol: float = 0.3  # offset from the ego-lane centre [m]
    psi_tol: float = np.radians(2.0)  # heading [rad]
    settle_time: float = 1.0  # how long the return conditions must hold [s]

    @property
    def road_edges(self):
        """Y of the right and left road edge [m]."""
        return -0.5 * self.lane_width, 1.5 * self.lane_width

    def initial_state(self):
        x = np.zeros(NX)
        x[IUX] = self.v0
        return x

    def lead_x(self, t):
        """X of the lead car's body centre [m]."""
        ego_front = 0.5 * self.car_length - self.cg_to_center
        return ego_front + self.d_trig + 0.5 * self.car_length + self.v_lead * t

    def ego_corners(self, x):
        """Corners of the ego body, shape (4, 2)."""
        c, s = np.cos(x[IPSI]), np.sin(x[IPSI])
        centre = np.array([x[IX] - self.cg_to_center * c, x[IY] - self.cg_to_center * s])
        return centre + self._half_extents() @ np.array([[c, s], [-s, c]])

    def lead_corners(self, t):
        return np.array([self.lead_x(t), 0.0]) + self._half_extents()

    def clearance(self, t, x):
        """Distance between the two car bodies [m], 0 if they overlap."""
        return _polygon_distance(self.ego_corners(x), self.lead_corners(t))

    def road_margin(self, x):
        """Distance from the ego body to the nearer road edge [m], negative if off the road."""
        y = self.ego_corners(x)[:, 1]
        right, left = self.road_edges
        return min(y.min() - right, left - y.max())

    def gap_ahead(self, t, x):
        """Ego rear bumper minus lead front bumper along the road [m]."""
        return self.ego_corners(x)[:, 0].min() - (self.lead_x(t) + 0.5 * self.car_length)

    def returned(self, t, x):
        """True if the ego is ahead of the lead car and aligned with the ego lane."""
        return (
            self.gap_ahead(t, x) >= self.return_gap
            and abs(x[IY]) <= self.y_tol
            and abs(x[IPSI]) <= self.psi_tol
        )

    def evaluate(self, t, x):
        """Score a logged run: `t` of shape (N,), ego states `x` of shape (N, NX)."""
        t, x = np.asarray(t, dtype=float), np.asarray(x, dtype=float)
        min_clearance = min(self.clearance(tk, xk) for tk, xk in zip(t, x))
        road_margin = min(self.road_margin(xk) for xk in x)
        peak_sideslip = np.abs(np.arctan2(x[:, IUY], x[:, IUX])).max()

        t_complete = distance = None
        start = None  # first sample of the current run of `returned`
        for k, (tk, xk) in enumerate(zip(t, x)):
            if not self.returned(tk, xk):
                start = None
                continue
            if start is None:
                start = k
            if tk - t[start] >= self.settle_time - 1e-9:
                t_complete, distance = t[start], x[start, IX] - x[0, IX]
                break

        checks = {
            "clearance": min_clearance >= self.clearance_min,
            "road": road_margin >= 0.0,
            "sideslip": peak_sideslip <= self.sideslip_max,
            "completed": t_complete is not None and t_complete <= self.t_max,
        }
        failures = tuple(name for name, ok in checks.items() if not ok)
        return OvertakeResult(
            not failures, failures, min_clearance, road_margin, peak_sideslip, t_complete, distance
        )

    def _half_extents(self):
        hl, hw = 0.5 * self.car_length, 0.5 * self.car_width
        return np.array([[hl, hw], [-hl, hw], [-hl, -hw], [hl, -hw]])


def _polygon_distance(P, Q):
    """Distance between two convex polygons given as (n, 2) corner arrays."""
    if _overlap(P, Q):
        return 0.0
    return min(
        min(_point_segment_distance(a, B[i], B[i - 1]) for a in A for i in range(len(B)))
        for A, B in ((P, Q), (Q, P))
    )


def _overlap(P, Q):
    """Separating-axis test for two convex polygons."""
    for A in (P, Q):
        edges = A - np.roll(A, 1, axis=0)
        for normal in np.column_stack([-edges[:, 1], edges[:, 0]]):
            p, q = P @ normal, Q @ normal
            if p.max() < q.min() or q.max() < p.min():
                return False
    return True


def _point_segment_distance(a, b0, b1):
    d = b1 - b0
    s = np.clip((a - b0) @ d / (d @ d), 0.0, 1.0)
    return np.linalg.norm(a - b0 - s * d)


# Named cases for the controller comparison. The expected outcomes are
# point-mass estimates, still to be shown in simulation.
SCENARIOS = {
    "nominal": OvertakeScenario(),
    # Both should pass: 7.2 s to contact, so a comfortable 2 m/s^2 lane change fits with room to spare.
    "relaxed": OvertakeScenario(v0=80 / 3.6, d_trig=60.0),
    # NMPC should pass, pure pursuit should not: 1.2 s to contact on a wet road. A smooth
    # quintic lane change that clears the lead car in time needs 5.25 m/s^2 > mu*g = 4.9;
    # steering out hard first and finishing the lane change later needs about 1.0 s.
    "late_wet": OvertakeScenario(v0=110 / 3.6, d_trig=20.0, mu=0.5),
    # Pure pursuit should do as well at a fraction of the compute: 18 s alongside the lead car,
    # far longer than a 3-4 s NMPC horizon, so the NMPC needs extra logic to hold the passing lane.
    "slow_closing": OvertakeScenario(v0=60 / 3.6, d_trig=30.0),
}
