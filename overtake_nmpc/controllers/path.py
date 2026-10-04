"""Reference path for the overtake: lane change out, passing lane, lane change back.

The path is a lateral offset Y_ref(X) in the road frame, planned once at the
trigger from the lead car's position and speed, assuming both cars keep their
speed. Each lane change is a quintic, Y = w (10 s^3 - 15 s^4 + 6 s^5) with
s = (X - X_start) / length, which starts and ends with zero heading and
curvature. Its peak lateral acceleration at speed v is QUINTIC_PEAK * w v^2 / length^2.
"""

from dataclasses import dataclass

import numpy as np

from ..model.bicycle import IUX, IX

QUINTIC_PEAK = 10.0 / np.sqrt(3.0)  # max of the quintic's second derivative, 5.77


def quintic(s):
    s = np.clip(s, 0.0, 1.0)
    return s**3 * (10.0 - 15.0 * s + 6.0 * s**2)


def quintic_inverse(y):
    """s in [0, 1] at which quintic(s) = y, by bisection."""
    lo, hi = 0.0, 1.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if quintic(mid) < y else (lo, mid)
    return 0.5 * (lo + hi)


@dataclass(frozen=True)
class OvertakePath:
    width: float  # lateral offset of the passing lane [m]
    x_out: float  # X where the lane change out starts [m]
    length_out: float  # [m]
    x_back: float  # X where the lane change back starts [m]
    length_back: float  # [m]
    a_lat_out: float  # peak lateral acceleration of the lane change out at the planning speed [m/s^2]
    feasible: bool  # a_lat_out within the road's grip

    def y(self, X):
        """Lateral reference Y_ref [m] at road position X."""
        out = quintic((np.asarray(X) - self.x_out) / self.length_out)
        back = quintic((np.asarray(X) - self.x_back) / self.length_back)
        return self.width * (out - back)


def plan_overtake(scenario, x, a_lat=2.0, gap_back=3.0):
    """Plan the overtake from ego state `x` at the scenario's start (t = 0).

    `a_lat` [m/s^2] is the lateral-acceleration budget for each lane change.
    The lane change out is shortened if that is needed to clear the lead car
    by `scenario.clearance_min` before reaching it, even if the result needs
    more grip than the road has; `feasible` records whether it does. The lane
    change back starts once the ego's rear is `gap_back` ahead of the lead's front.
    """
    sc = scenario
    v, w = x[IUX], sc.lane_width
    closing = v - sc.v_lead
    if closing <= 0.0:
        raise ValueError("ego is not faster than the lead car")

    length_comfort = v * np.sqrt(QUINTIC_PEAK * w / a_lat)
    # distance the ego covers before its front reaches the lead's rear
    x_contact = v * sc.clearance(0.0, x) / closing
    s_clear = quintic_inverse((sc.car_width + sc.clearance_min) / w)
    length_out = min(length_comfort, x_contact / s_clear)
    a_lat_out = QUINTIC_PEAK * w * v**2 / length_out**2

    t_back = (gap_back - sc.gap_ahead(0.0, x)) / closing
    x_back = max(x[IX] + v * t_back, x[IX] + length_out)
    return OvertakePath(
        width=w,
        x_out=x[IX],
        length_out=length_out,
        x_back=x_back,
        length_back=length_comfort,
        a_lat_out=a_lat_out,
        feasible=a_lat_out <= sc.mu * 9.81,
    )
