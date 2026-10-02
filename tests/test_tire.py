import numpy as np
import pytest

from overtake_nmpc.model.tire import F_MIN, fiala_lateral

C_ALPHA = 160e3
MU = 0.9
FZ = 9000.0


def test_small_slip_matches_cornering_stiffness():
    alpha = 1e-4
    assert fiala_lateral(alpha, FZ, 0.0, C_ALPHA, MU) == pytest.approx(-C_ALPHA * alpha, rel=2e-3)


def test_saturates_at_friction_limit():
    assert fiala_lateral(0.3, FZ, 0.0, C_ALPHA, MU) == pytest.approx(-MU * FZ, rel=1e-6)
    assert fiala_lateral(-0.3, FZ, 0.0, C_ALPHA, MU) == pytest.approx(MU * FZ, rel=1e-6)


def test_continuous_at_sliding_threshold():
    alpha_sl = np.arctan(3 * MU * FZ / C_ALPHA)
    below = fiala_lateral(alpha_sl * (1 - 1e-6), FZ, 0.0, C_ALPHA, MU)
    above = fiala_lateral(alpha_sl * (1 + 1e-6), FZ, 0.0, C_ALPHA, MU)
    assert below == pytest.approx(above, rel=1e-6)


def test_odd_in_slip_angle():
    alpha = np.linspace(0.0, 0.3, 31)
    np.testing.assert_allclose(
        fiala_lateral(alpha, FZ, 0.0, C_ALPHA, MU), -fiala_lateral(-alpha, FZ, 0.0, C_ALPHA, MU)
    )


def test_total_force_stays_in_friction_circle():
    alpha, Fx = np.meshgrid(np.linspace(-0.4, 0.4, 81), np.linspace(-MU * FZ, MU * FZ, 41))
    Fy = fiala_lateral(alpha, FZ, Fx, C_ALPHA, MU)
    assert np.all(Fx**2 + Fy**2 <= (MU * FZ) ** 2 + 2 * F_MIN**2)


def test_longitudinal_force_reduces_lateral_capacity():
    free = abs(fiala_lateral(0.3, FZ, 0.0, C_ALPHA, MU))
    braking = abs(fiala_lateral(0.3, FZ, -0.6 * MU * FZ, C_ALPHA, MU))
    assert braking == pytest.approx(0.8 * free, rel=1e-4)
