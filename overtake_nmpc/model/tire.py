"""Fiala brush tire model with friction-circle derating."""

from .backend import NUMPY

F_MIN = 50.0  # floor on lateral capacity [N], keeps the model smooth when Fx uses all the grip


def lateral_capacity(Fz, Fx, mu, ops=NUMPY):
    """Lateral force left on the friction circle, sqrt((mu Fz)^2 - Fx^2), smoothed near zero."""
    s = (mu * Fz) ** 2 - Fx**2
    return ops.sqrt(0.5 * (s + ops.sqrt(s * s + 4.0 * F_MIN**4)))


def fiala_lateral(alpha, Fz, Fx, C_alpha, mu, ops=NUMPY):
    """Axle lateral force [N] for slip angle alpha [rad], normal load Fz and longitudinal force Fx."""
    Fy_max = lateral_capacity(Fz, Fx, mu, ops)
    z = ops.tan(alpha)
    z_sl = 3.0 * Fy_max / C_alpha  # tan of the slip angle at full sliding
    Fy_grip = (
        -C_alpha * z
        + C_alpha**2 / (3.0 * Fy_max) * ops.fabs(z) * z
        - C_alpha**3 / (27.0 * Fy_max**2) * z**3
    )
    Fy_slide = -Fy_max * ops.sign(z)
    return ops.if_else(ops.fabs(z) < z_sl, Fy_grip, Fy_slide)
