import matplotlib
import numpy as np
import pytest

from overtake_core import (
    BicyclePlant,
    IUX,
    plan_overtake,
    PurePursuit,
    quintic,
    quintic_inverse,
    QUINTIC_PEAK,
    replace,
    run_overtake,
    SCENARIOS,
    VehicleParams,
)


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
    return run_overtake(sc, PurePursuit(sc, p), BicyclePlant(p))


@pytest.mark.slow
def test_pure_pursuit_passes_nominal():
    _, res = run("nominal")
    assert res.passed, res


def test_pure_pursuit_fails_late_wet():
    _, res = run("late_wet")
    assert "clearance" in res.failures


@pytest.mark.slow
def test_animation_saves_a_video(tmp_path):
    matplotlib.use("Agg")
    from sim.scenarios.overtake.animate import animate

    sc = SCENARIOS["late_wet"]
    p = replace(VehicleParams(), mu=sc.mu)
    controller = PurePursuit(sc, p)
    log, res = run_overtake(sc, controller, BicyclePlant(p))
    out = tmp_path / "run.mp4"
    animate(sc, {"pure pursuit": (log, res)}, controller.path, speed=4.0, save=out)
    assert out.stat().st_size > 0
