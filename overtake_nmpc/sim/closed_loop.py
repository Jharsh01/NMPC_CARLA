"""Closed-loop run of a controller on the overtaking scenario."""

from types import SimpleNamespace

import numpy as np


def run_overtake(scenario, controller, plant, dt_ctrl=0.05, dt_log=0.01):
    """Drive `plant` through `scenario` with `controller(t, x) -> u`.

    The controller is called every `dt_ctrl` seconds and its input held in
    between; the state is logged every `dt_log` seconds. The run ends at a
    collision, once the ego has returned to its lane for the settle time, or at
    `scenario.t_max`. `plant` should be built with the scenario's friction.

    Returns the log (t, x, and the saturated input u applied from each sample)
    and the scenario's result.
    """
    substeps = int(round(dt_ctrl / dt_log))
    plant.reset(scenario.initial_state())
    t, x = 0.0, plant.observe()
    ts, xs, us = [], [], []
    returned_since = None
    done = False
    while not done and t < scenario.t_max + scenario.settle_time:
        u = plant.saturate(np.asarray(controller(t, x), dtype=float))
        for _ in range(substeps):
            ts.append(t)
            xs.append(x)
            us.append(u)
            if scenario.clearance(t, x) <= 0.0:
                done = True
                break
            if not scenario.returned(t, x):
                returned_since = None
            elif returned_since is None:
                returned_since = t
            elif t - returned_since >= scenario.settle_time - 1e-9:
                done = True
                break
            x = plant.step(u, dt_log)
            t += dt_log

    log = SimpleNamespace(t=np.array(ts), x=np.array(xs), u=np.array(us))
    return log, scenario.evaluate(log.t, log.x)
