"""Pure-pursuit baseline for the circuit: follow a lane, a speed profile and a passing rule.

Steering is the pure-pursuit law of `controllers.pure_pursuit` on a lane line
of the track. Speed follows a grip-limited profile computed once for the lap
(`controllers.speed_profile`). Pure pursuit has no notion of other cars, so
passing is a rule: move to the other lane while a traffic car in the home lane
is within a window around the ego, and come back afterwards.
"""

import numpy as np

from ..model.bicycle import IDELTA, IPSI, IUX, IX, IY
from .speed_profile import grip_speed_profile


class TrackPurePursuit:
    """Controller `(t, x) -> [steering rate, Fx]` for `sim.lap.run_lap`.

    The lookahead point is on the current target line, L_d = max(k_lookahead *
    U_x, lookahead_min) further along the track than the rear axle. The target
    line moves between lane centres at `lateral_rate` [m/s]. A traffic car
    triggers the move when it is between `window_behind` behind and
    `window_ahead` ahead of the ego (centre to centre, along the track). The
    window ahead grows with the closing speed, so that `pass_time` seconds are
    left to change lane before reaching the car.
    `grip_use` is the share of mu*g the speed profile may use and `brake_use`
    the share of the brakes' peak force it plans with.
    """

    def __init__(
        self,
        circuit,
        params,
        lane="right",
        grip_use=0.4,
        a_accel=3.0,
        brake_use=0.85,
        k_lookahead=0.6,
        lookahead_min=5.0,
        tau_steer=0.05,
        k_speed=3.0,
        window_ahead=30.0,
        window_behind=15.0,
        pass_time=3.5,
        lateral_rate=1.2,
    ):
        self.c = circuit
        self.p = params
        self.k_lookahead = k_lookahead
        self.lookahead_min = lookahead_min
        self.tau_steer = tau_steer
        self.k_speed = k_speed
        self.window = (-window_behind, window_ahead)
        self.pass_time = pass_time
        self.lateral_rate = lateral_rate
        self.home = lane
        self.e_home = circuit.lane_offset(lane)
        self.e_pass = -self.e_home
        self.e_target = self.e_home
        self._t = None

        # the profile must hold on whichever lane the car is in
        self.s_grid, self.v_grid = grip_speed_profile(
            circuit.track,
            (self.e_home, self.e_pass),
            circuit.mu_road,
            circuit.v_max,
            grip_use,
            a_accel,
            a_brake=brake_use * params.F_brake_max / params.m,
        )

    def speed_reference(self, s):
        """Profile speed [m/s] and its slope dv/ds [1/s] at arc length `s`."""
        lap = self.c.track.length
        v = np.interp(s % lap, self.s_grid, self.v_grid, period=lap)
        ahead = np.interp((s + 1.0) % lap, self.s_grid, self.v_grid, period=lap)
        return v, ahead - v

    def _traffic_in_the_way(self, t, s, speed):
        lap = self.c.track.length
        for car in self.c.traffic:
            if car.lane != self.home:
                continue
            ahead = (self.c.traffic_s(car, t) - s + 0.5 * lap) % lap - 0.5 * lap
            reach = self.c.car_length + self.pass_time * max(speed - car.speed, 0.0)
            if self.window[0] <= ahead <= max(self.window[1], reach):
                return True
        return False

    def __call__(self, t, x):
        p, track = self.p, self.c.track
        s, _ = track.project(x[IX], x[IY])

        # passing rule: slide the target line toward the lane it should be in
        goal = self.e_pass if self._traffic_in_the_way(t, s, x[IUX]) else self.e_home
        dt = 0.0 if self._t is None else t - self._t
        self._t = t
        self.e_target += np.clip(goal - self.e_target, -self.lateral_rate * dt, self.lateral_rate * dt)

        psi = x[IPSI]
        rear = np.array([x[IX] - p.b * np.cos(psi), x[IY] - p.b * np.sin(psi)])
        L_d = max(self.k_lookahead * x[IUX], self.lookahead_min)
        tx, ty, _ = track.pose(track.project(*rear)[0] + L_d, self.e_target)
        alpha = np.arctan2(ty - rear[1], tx - rear[0]) - psi
        chord = np.hypot(tx - rear[0], ty - rear[1])
        delta_cmd = np.clip(np.arctan(2.0 * p.L * np.sin(alpha) / chord), -p.delta_max, p.delta_max)
        rate = (delta_cmd - x[IDELTA]) / self.tau_steer

        v_ref, dv_ds = self.speed_reference(s)
        accel = v_ref * dv_ds + self.k_speed * (v_ref - x[IUX])
        drag = 0.5 * p.rho * p.CdA * x[IUX] ** 2 + p.Crr * p.m * p.g
        return np.array([rate, p.m * accel + drag])
