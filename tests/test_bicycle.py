from dataclasses import replace

import casadi as ca
import numpy as np
import pytest

from overtake_nmpc.model.backend import CASADI
from overtake_nmpc.model.bicycle import IUX, IX, NU, NX, axle_forces, dynamics, fx_limits
from overtake_nmpc.model.params import VehicleParams

P = VehicleParams()


def test_casadi_and_numpy_agree():
    x, u = ca.MX.sym("x", NX), ca.MX.sym("u", NU)
    f = ca.Function("f", [x, u], [dynamics(x, u, P, CASADI)])
    rng = np.random.default_rng(0)
    lo = np.array([-10, -10, -1, 5, -2, -0.5, -0.2, -0.6, -8000])
    hi = np.array([10, 10, 1, 35, 2, 0.5, 0.2, 0.6, 4000])
    for _ in range(50):
        s = rng.uniform(lo, hi)
        np.testing.assert_allclose(
            f(s[:NX], s[NX:]).full().ravel(), dynamics(s[:NX], s[NX:], P), rtol=1e-9, atol=1e-9
        )


def test_coasting_straight_without_drag_is_steady():
    p = replace(P, CdA=0.0, Crr=0.0)
    x = np.zeros(NX)
    x[IUX] = 20.0
    expected = np.zeros(NX)
    expected[IX] = 20.0
    np.testing.assert_allclose(dynamics(x, np.zeros(NU), p), expected, atol=1e-12)


def test_load_transfer():
    x = np.zeros(NX)
    x[IUX] = 20.0
    rest = axle_forces(x, np.array([0.0, 0.0]), P)
    braking = axle_forces(x, np.array([0.0, -5000.0]), P)
    assert rest.Fzf == pytest.approx(P.m * P.g * P.b / P.L)
    assert braking.Fzf + braking.Fzr == pytest.approx(P.m * P.g)
    assert braking.Fzf - rest.Fzf == pytest.approx(5000.0 * P.h / P.L)


def test_force_limits_keep_each_axle_within_grip():
    for mu in (0.9, 0.5, 0.3):
        p = replace(P, mu=mu)
        x = np.zeros(NX)
        x[IUX] = 10.0
        for Fx in fx_limits(10.0, p):
            f = axle_forces(x, np.array([0.0, Fx]), p)
            assert abs(f.Fxf) <= mu * f.Fzf * (1 + 1e-9)
            assert abs(f.Fxr) <= mu * f.Fzr * (1 + 1e-9)
            assert abs(Fx) <= mu * p.m * p.g
