from dataclasses import replace

import matplotlib
import numpy as np
import pytest

from overtake_nmpc.controllers.nmpc import NMPC
from overtake_nmpc.controllers.pure_pursuit import PurePursuit
from overtake_nmpc.model.bicycle import IRATE, IX, IY, NX
from overtake_nmpc.model.params import VehicleParams
from overtake_nmpc.scenarios.overtake import SCENARIOS
from overtake_nmpc.sim.closed_loop import run_overtake
from overtake_nmpc.sim.plant import Plant


def run(name, cls=NMPC):
    sc = SCENARIOS[name]
    p = replace(VehicleParams(), mu=sc.mu)
    controller = cls(sc, p)
    return controller, *run_overtake(sc, controller, Plant(p))


def test_ellipse_contains_the_keep_out_rectangle():
    sc = SCENARIOS["nominal"]
    c = NMPC(sc, VehicleParams(), horizon=1.0, margin_lat=0.6, margin_long=1.0)
    # body centres this far apart leave exactly the margins between the bodies
    corner = (sc.car_length + 1.0) ** 2 / c.ell_a**2 + (sc.car_width + 0.6) ** 2 / c.ell_b**2
    assert corner == pytest.approx(1.0)
    assert c.ell_b < sc.lane_width


def test_first_solve_plans_around_the_lead_car():
    sc = SCENARIOS["nominal"]
    c = NMPC(sc, VehicleParams())
    u = c(0.0, sc.initial_state())
    assert c.failures == 0
    assert u[IRATE] > 0.0  # steers left, toward the passing lane
    # the plan uses the plant's state vector and stays outside the ellipse
    assert c.plan.shape == (c.N + 1, NX)
    k = np.arange(c.N + 1)
    dx = c.plan[:, IX] - sc.lead_x(c.h * k) - c.ell_x
    assert dx[-1] > c.ell_a  # it ends ahead of the lead car
    assert np.all((dx[1:] / c.ell_a) ** 2 + (c.plan[1:, IY] / c.ell_b) ** 2 >= 1.0 - 1e-6)


def test_nmpc_passes_nominal():
    c, _, res = run("nominal")
    assert res.passed, res
    assert c.failures == 0


def test_nmpc_passes_late_wet_and_animates_next_to_pure_pursuit(tmp_path):
    matplotlib.use("Agg")
    from overtake_nmpc.sim.animate import animate

    sc = SCENARIOS["late_wet"]
    pp, pp_log, pp_res = run("late_wet", PurePursuit)
    _, log, res = run("late_wet")
    assert "clearance" in pp_res.failures
    assert res.passed, res
    out = tmp_path / "both.gif"
    animate(sc, {"pure pursuit": (pp_log, pp_res), "NMPC": (log, res)}, pp.path, speed=4.0, save=out)
    assert out.stat().st_size > 0
