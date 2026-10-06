"""The Project Chrono plant, where the project was built with Chrono."""

import json
from pathlib import Path

import numpy as np
import pytest

from overtake_core import CHRONO_PARAMETERS_PATH, IDELTA, IR, IUX, IUY, IX, IY, NX, ChronoPlant, VehicleParams

pytestmark = [pytest.mark.slow]  # building a Chrono vehicle takes a second or two
try:
    PARAMS = VehicleParams.from_file(CHRONO_PARAMETERS_PATH)
    ChronoPlant(PARAMS)
except RuntimeError as reason:  # built without Chrono, or not identified yet
    pytestmark.append(pytest.mark.skip(reason=str(reason)))


@pytest.fixture(scope="module")
def plant():
    return ChronoPlant(PARAMS)


def start(speed):
    x = np.zeros(NX)
    x[IUX] = speed
    return x


def test_parameters_were_read_from_the_chrono_model(plant):
    facts = plant.facts()
    assert PARAMS.m == pytest.approx(facts["mass"])
    assert PARAMS.a == pytest.approx(facts["a"]) and PARAMS.b == pytest.approx(facts["b"])
    assert PARAMS.F_brake_max == pytest.approx(facts["brake_force_max"])
    sources = {entry["source"] for entry in json.loads(Path(CHRONO_PARAMETERS_PATH).read_text())["vehicle"]["parameters"].values()}
    assert sources == {"chrono", "identified", "design", "constant"}


def test_starts_at_the_requested_state_and_coasts(plant):
    plant.reset(start(20.0))
    assert plant.observe() == pytest.approx(start(20.0))
    x = plant.step(np.zeros(2), 1.0)
    # no jolt from the start: only drag slows it, and it runs straight
    drag = 0.5 * PARAMS.rho * PARAMS.CdA * 20.0**2 / PARAMS.m
    assert x[IUX] == pytest.approx(20.0 - drag, abs=0.05)
    assert x[IX] == pytest.approx(20.0, abs=0.1)
    assert abs(x[IY]) < 0.05 and abs(x[IUY]) < 0.02 and abs(x[IR]) < 0.01


@pytest.mark.parametrize("force", [3000.0, -5000.0])
def test_longitudinal_force_is_delivered(plant, force):
    plant.reset(start(20.0))
    v0 = plant.step(np.array([0.0, force]), 0.5)[IUX]
    v1 = plant.step(np.array([0.0, force]), 1.0)[IUX]
    drag = 0.5 * PARAMS.rho * PARAMS.CdA * v0**2
    assert (v1 - v0) / 1.0 == pytest.approx((force - drag) / PARAMS.m, rel=0.05)


def test_steers_left_for_a_positive_rate_and_reports_the_commanded_angle(plant):
    plant.reset(start(15.0))
    for _ in range(20):
        x = plant.step(np.array([0.02, 0.0]), 0.05)
    assert x[IDELTA] == pytest.approx(0.02 * 1.0, abs=0.002)  # 1 s of 0.02 rad/s, less the actuator lag
    assert x[IR] > 0.05 and x[IY] > 0.0


def test_bicycle_model_with_identified_parameters_matches_steady_cornering(plant):
    from overtake_core import BicyclePlant

    speed, delta = 15.0, 0.03  # about 0.2 g
    rates = []
    for car in (plant, BicyclePlant(PARAMS)):
        car.reset(start(speed))
        x = car.observe()
        for _ in range(400):
            x = car.step(np.array([(delta - x[IDELTA]) / 0.1, 2.0 * PARAMS.m * (speed - x[IUX])]), 0.02)
        rates.append(x[IR])
    assert rates[1] == pytest.approx(rates[0], rel=0.05)
