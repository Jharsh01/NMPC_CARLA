"""Simulation plant: the bicycle model plus effects the controller model leaves out.

On top of `model.bicycle.dynamics` the plant adds
- actuator saturation (steering angle and rate, tire and engine force limits),
- a first-order steering actuator lag,
- first-order tire force build-up (relaxation length),
and it can be given parameters that differ from the controller's.
"""

import numpy as np

from ..model.bicycle import IDELTA, IFX, IRATE, IUX, NX, dynamics, fx_limits, slip_angles
from .integrate import rk4_step


class Plant:
    def __init__(self, params, dt=1e-3, tau_steer=0.05, relaxation_length=0.3):
        """`tau_steer` [s] and `relaxation_length` [m] of 0 switch the respective lag off."""
        self.p = params
        self.dt = dt
        self.tau_steer = tau_steer
        self.relaxation_length = relaxation_length
        self.reset(np.zeros(NX))

    def reset(self, x0):
        x0 = np.asarray(x0, dtype=float)
        # internal state: model state, steering reference, lagged slip angles
        self.z = np.concatenate([x0, [x0[IDELTA]], slip_angles(x0, self.p)])

    def observe(self):
        """Model state [X, Y, psi, Ux, Uy, r, delta]."""
        return self.z[:NX].copy()

    def saturate(self, u):
        """Input after steering-rate and force limits."""
        p = self.p
        fx_min, fx_max = fx_limits(self.z[IUX], p)
        return np.array(
            [np.clip(u[IRATE], -p.delta_rate_max, p.delta_rate_max), np.clip(u[IFX], fx_min, fx_max)]
        )

    def _deriv(self, z, u):
        p = self.p
        x, delta_ref, alpha_lag = z[:NX], z[NX], z[NX + 1 :]

        ddelta_ref = u[IRATE]
        if abs(delta_ref) >= p.delta_max and ddelta_ref * delta_ref > 0:
            ddelta_ref = 0.0
        ddelta = (delta_ref - x[IDELTA]) / self.tau_steer if self.tau_steer > 0 else ddelta_ref

        alpha_ss = np.array(slip_angles(x, p))
        if self.relaxation_length > 0:
            alpha = alpha_lag
            dalpha = max(x[IUX], p.Ux_min) / self.relaxation_length * (alpha_ss - alpha_lag)
        else:
            alpha = alpha_ss
            dalpha = np.zeros(2)

        dx = dynamics(x, np.array([ddelta, u[IFX]]), p, alpha=alpha)
        return np.concatenate([dx, [ddelta_ref], dalpha])

    def step(self, u, duration):
        """Hold `u` for `duration` seconds and return the new model state."""
        for _ in range(int(round(duration / self.dt))):
            self.z = rk4_step(self._deriv, self.z, self.saturate(u), self.dt)
        return self.observe()
