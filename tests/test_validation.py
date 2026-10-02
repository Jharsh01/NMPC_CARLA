"""Closed-form checks of the simulated vehicle against hand calculations."""

from dataclasses import replace

import numpy as np
import pytest

from overtake_nmpc.model.bicycle import IDELTA, IR, IUX, IUY, IX, NX, dynamics, fx_limits
from overtake_nmpc.model.params import VehicleParams
from overtake_nmpc.sim.plant import Plant

NO_DRAG = replace(VehicleParams(), CdA=0.0, Crr=0.0)
DT_CTRL = 0.01


def ideal_plant(p):
    return Plant(p, tau_steer=0.0, relaxation_length=0.0)


def hold_speed(x, v_ref, p, gain=2.0):
    return p.m * gain * (v_ref - x[IUX])


def start_state(Ux, delta=0.0):
    x = np.zeros(NX)
    x[IUX] = Ux
    x[IDELTA] = delta
    return x


@pytest.mark.parametrize("mu", [0.9, 0.5])
def test_braking_distance(mu):
    p = replace(NO_DRAG, mu=mu)
    plant = ideal_plant(p)
    v0 = 30.0
    plant.reset(start_state(v0))
    x = plant.observe()
    while x[IUX] > 5.0:
        x = plant.step(np.array([0.0, -1e9]), DT_CTRL)

    decel = -fx_limits(v0, p)[0] / p.m
    assert decel <= min(mu * p.g, p.F_brake_max / p.m)
    assert x[IX] == pytest.approx((v0**2 - x[IUX] ** 2) / (2 * decel), rel=1e-3)


@pytest.mark.parametrize("Ux", [10.0, 20.0, 30.0])
def test_steady_state_yaw_rate_matches_understeer_gradient(Ux):
    p = NO_DRAG
    delta = 0.002
    plant = ideal_plant(p)
    plant.reset(start_state(Ux, delta))
    for _ in range(int(6.0 / DT_CTRL)):
        x = plant.observe()
        x = plant.step(np.array([0.0, hold_speed(x, Ux, p)]), DT_CTRL)

    expected = Ux * delta / (p.L + p.understeer_gradient * Ux**2)
    assert x[IR] == pytest.approx(expected, rel=0.01)


def test_lateral_acceleration_saturates_at_mu_g():
    p = NO_DRAG
    Ux = 25.0
    plant = ideal_plant(p)
    plant.reset(start_state(Ux))
    ay = []
    for _ in range(int(12.0 / DT_CTRL)):
        x = plant.observe()
        rate = 0.02 if x[IDELTA] < 0.2 else 0.0
        u = plant.saturate(np.array([rate, hold_speed(x, Ux, p)]))
        ay.append(dynamics(x, u, p)[IUY] + x[IR] * x[IUX])
        x = plant.step(u, DT_CTRL)

    assert 0.9 * p.mu * p.g <= max(ay) <= 1.001 * p.mu * p.g


def test_lags_delay_response_but_keep_steady_state():
    p = NO_DRAG
    Ux, rate, t_ramp = 20.0, 0.1, 0.1
    ideal, lagged = ideal_plant(p), Plant(p)
    early = {}
    for name, plant in (("ideal", ideal), ("lagged", lagged)):
        plant.reset(start_state(Ux))
        for k in range(int(6.0 / DT_CTRL)):
            x = plant.observe()
            u = np.array([rate if k * DT_CTRL < t_ramp else 0.0, hold_speed(x, Ux, p)])
            x = plant.step(u, DT_CTRL)
            if k == int(0.15 / DT_CTRL):
                early[name] = x[IR]

    assert early["lagged"] < 0.9 * early["ideal"]
    assert lagged.observe()[IR] == pytest.approx(ideal.observe()[IR], rel=1e-3)
