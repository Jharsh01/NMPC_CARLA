"""Dynamic single-track (bicycle) model.

State  x = [X, Y, psi, Ux, Uy, r, delta]
Input  u = [delta_rate, Fx]

X, Y, psi are the global pose; Ux, Uy are body-frame velocities at the centre of
gravity; r is yaw rate; delta is the road-wheel steering angle; Fx is the total
longitudinal tire force (positive drive, negative brake).
"""

from types import SimpleNamespace

import numpy as np

from .backend import NUMPY
from .tire import fiala_lateral

NX = 7
NU = 2
IX, IY, IPSI, IUX, IUY, IR, IDELTA = range(NX)
IRATE, IFX = range(NU)


def slip_angles(x, p, ops=NUMPY):
    """Front and rear axle slip angles [rad]."""
    Ux = ops.fmax(x[IUX], p.Ux_min)
    alpha_f = ops.atan((x[IUY] + p.a * x[IR]) / Ux) - x[IDELTA]
    alpha_r = ops.atan((x[IUY] - p.b * x[IR]) / Ux)
    return alpha_f, alpha_r


def axle_forces(x, u, p, ops=NUMPY, alpha=None):
    """Normal, longitudinal and lateral force on each axle.

    `alpha` overrides the kinematic slip angles (used by the plant for tire lag).
    """
    Fx = u[IFX]
    front = ops.if_else(Fx >= 0, p.drive_front, p.brake_front)
    Fxf = front * Fx
    Fxr = Fx - Fxf
    # quasi-static longitudinal load transfer
    Fzf = (p.m * p.g * p.b - p.h * Fx) / p.L
    Fzr = (p.m * p.g * p.a + p.h * Fx) / p.L
    alpha_f, alpha_r = slip_angles(x, p, ops) if alpha is None else alpha
    Fyf = fiala_lateral(alpha_f, Fzf, Fxf, p.C_alpha_f, p.mu, ops)
    Fyr = fiala_lateral(alpha_r, Fzr, Fxr, p.C_alpha_r, p.mu, ops)
    return SimpleNamespace(
        Fzf=Fzf, Fzr=Fzr, Fxf=Fxf, Fxr=Fxr, Fyf=Fyf, Fyr=Fyr, alpha_f=alpha_f, alpha_r=alpha_r
    )


def dynamics(x, u, p, ops=NUMPY, alpha=None):
    """Continuous-time state derivative."""
    psi, Ux, Uy, r, delta = x[IPSI], x[IUX], x[IUY], x[IR], x[IDELTA]
    f = axle_forces(x, u, p, ops, alpha)
    drag = 0.5 * p.rho * p.CdA * Ux**2 + p.Crr * p.m * p.g
    Fy_front_body = f.Fyf * ops.cos(delta) + f.Fxf * ops.sin(delta)

    dUx = (f.Fxf * ops.cos(delta) - f.Fyf * ops.sin(delta) + f.Fxr - drag) / p.m + r * Uy
    dUy = (Fy_front_body + f.Fyr) / p.m - r * Ux
    dr = (p.a * Fy_front_body - p.b * f.Fyr) / p.Izz
    dX = Ux * ops.cos(psi) - Uy * ops.sin(psi)
    dY = Ux * ops.sin(psi) + Uy * ops.cos(psi)
    return ops.stack(dX, dY, r, dUx, dUy, dr, u[IRATE])


def fx_limits(Ux, p):
    """Range of total longitudinal force [N] the tires, brakes and engine can deliver in a straight line.

    The fixed front/rear split means one axle reaches mu*Fz first, so the grip
    limits sit below mu*m*g.
    """
    grip = p.mu * p.m * p.g

    def axle_limit(num, den):
        return num / den if den > 0 else np.inf

    fb, fd = p.brake_front, p.drive_front
    brake = min(
        axle_limit(grip * p.b, fb * p.L - p.mu * p.h),
        axle_limit(grip * p.a, (1 - fb) * p.L + p.mu * p.h),
        p.F_brake_max,
    )
    drive = min(
        axle_limit(grip * p.b, fd * p.L + p.mu * p.h),
        axle_limit(grip * p.a, (1 - fd) * p.L - p.mu * p.h),
        p.F_drive_max,
        p.P_max / max(Ux, p.Ux_min),
    )
    return -brake, drive
