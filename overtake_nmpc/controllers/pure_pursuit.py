"""Pure-pursuit baseline: track the planned overtake path, hold the entry speed."""

import numpy as np

from ..model.bicycle import IDELTA, IPSI, IUX, IX, IY
from .path import plan_overtake


class PurePursuit:
    """Controller `(t, x) -> [steering rate, Fx]` for `sim.closed_loop.run_overtake`.

    Steering: the lookahead point is on the path at distance
    L_d = max(k_lookahead * U_x, lookahead_min) from the rear axle, and the
    steering angle that puts the rear axle on an arc through it is
    delta = atan(2 L sin(alpha) / L_d). The plant takes a steering rate, so the
    angle is reached with a first-order command of time constant `tau_steer`.
    Speed: proportional control of F_x toward the entry speed.
    """

    def __init__(
        self, scenario, params, k_lookahead=1.4, lookahead_min=5.0, tau_steer=0.05, k_speed=1.0, a_lat=2.0
    ):
        self.sc = scenario
        self.p = params
        self.k_lookahead = k_lookahead
        self.lookahead_min = lookahead_min
        self.tau_steer = tau_steer
        self.k_speed = k_speed
        self.a_lat = a_lat
        self.path = None
        self.v_ref = None

    def __call__(self, t, x):
        p = self.p
        if self.path is None:  # plan once, at the trigger
            self.path = plan_overtake(self.sc, x, self.a_lat)
            self.v_ref = x[IUX]

        psi = x[IPSI]
        rear = np.array([x[IX] - p.b * np.cos(psi), x[IY] - p.b * np.sin(psi)])
        L_d = max(self.k_lookahead * x[IUX], self.lookahead_min)
        target = self.lookahead_point(rear, L_d)
        alpha = np.arctan2(target[1] - rear[1], target[0] - rear[0]) - psi
        delta_cmd = np.clip(np.arctan(2.0 * p.L * np.sin(alpha) / L_d), -p.delta_max, p.delta_max)

        rate = (delta_cmd - x[IDELTA]) / self.tau_steer
        Fx = self.k_speed * p.m * (self.v_ref - x[IUX])
        return np.array([rate, Fx])

    def lookahead_point(self, rear, L_d):
        """First point ahead on the path at distance L_d from `rear`."""
        # the last sample is L_d ahead along X, so at least L_d away
        X = rear[0] + np.linspace(0.0, L_d, 101)
        d = np.hypot(X - rear[0], self.path.y(X) - rear[1])
        k = int(np.argmax(d >= L_d))
        if k == 0:  # already L_d off the path: aim at the nearest-ahead point
            X_la = X[0]
        else:
            X_la = np.interp(L_d, d[k - 1 : k + 1], X[k - 1 : k + 1])
        return np.array([X_la, self.path.y(X_la)])
