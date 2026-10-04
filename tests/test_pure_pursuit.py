from dataclasses import replace

import matplotlib
import numpy as np
import pytest

from overtake_nmpc.controllers.path import QUINTIC_PEAK, plan_overtake, quintic, quintic_inverse
from overtake_nmpc.controllers.pure_pursuit import PurePursuit
from overtake_nmpc.model.bicycle import IUX
from overtake_nmpc.model.params import VehicleParams
from overtake_nmpc.scenarios.overtake import SCENARIOS
from overtake_nmpc.sim.closed_loop import run_overtake
from overtake_nmpc.sim.plant import Plant


def test_quintic_shape():
    s = np.linspace(0.0, 1.0, 100001)
    y = quintic(s)
    assert y[0] == 0.0 and y[-1] == pytest.approx(1.0)
    d2 = np.gradient(np.gradient(y, s), s)
    assert np.abs(d2).max() == pytest.approx(QUINTIC_PEAK, rel=1e-3)
    assert quintic(quintic_inverse(0.3)) == pytest.approx(0.3)


@pytest.mark.parametrize("name", ["nominal", "relaxed", "slow_closing"])
def test_comfortable_path_when_there_is_room(name):
    sc = SCENARIOS[name]
    path = plan_overtake(sc, sc.initial_state())
    assert path.a_lat_out == pytest.approx(2.0)
    assert path.feasible
    v = sc.v0
    # the ego rear is gap_back ahead of the lead front when the return starts
    t_back = (path.x_back - path.x_out) / v
    x_back = sc.initial_state()
    x_back[0] = path.x_back
    assert sc.gap_ahead(t_back, x_back) == pytest.approx(3.0)
    assert path.y(path.x_out + path.length_out) == pytest.approx(sc.lane_width)
    assert path.y(path.x_back + path.length_back) == pytest.approx(0.0, abs=1e-12)


def test_late_wet_path_is_shortened_to_clear_and_flagged_infeasible():
    sc = SCENARIOS["late_wet"]
    x0 = sc.initial_state()
    path = plan_overtake(sc, x0)
    assert not path.feasible
    assert path.a_lat_out > sc.mu * 9.81
    # at the moment the ego front would reach the lead rear, the path is clear by clearance_min
    x_contact = x0[IUX] * sc.d_trig / (sc.v0 - sc.v_lead)
    assert path.y(x_contact) == pytest.approx(sc.car_width + sc.clearance_min, rel=1e-6)


def run(name):
    sc = SCENARIOS[name]
    p = replace(VehicleParams(), mu=sc.mu)
    return run_overtake(sc, PurePursuit(sc, p), Plant(p))


def test_pure_pursuit_passes_nominal():
    _, res = run("nominal")
    assert res.passed, res


def test_pure_pursuit_fails_late_wet():
    _, res = run("late_wet")
    assert "clearance" in res.failures


def test_animation_saves_a_gif(tmp_path):
    matplotlib.use("Agg")
    from overtake_nmpc.sim.animate import animate

    sc = SCENARIOS["late_wet"]
    p = replace(VehicleParams(), mu=sc.mu)
    controller = PurePursuit(sc, p)
    log, res = run_overtake(sc, controller, Plant(p))
    out = tmp_path / "run.gif"
    animate(sc, {"pure pursuit": (log, res)}, controller.path, speed=4.0, save=out)
    assert out.stat().st_size > 0
