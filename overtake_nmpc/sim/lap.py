"""Closed-loop run of a controller round the circuit."""

from dataclasses import dataclass, replace
from types import SimpleNamespace

import numpy as np

from ..model.bicycle import IUX, IUY


@dataclass(frozen=True)
class LapResult:
    completed: bool
    lap_time: float | None  # [s]
    collided: bool
    left_track: bool  # centre of gravity went beyond the grass
    min_clearance: float  # smallest distance to a traffic car body [m]
    road_margin: float  # smallest distance from the ego body to the asphalt edge [m], negative on the grass
    time_on_grass: float  # time with the centre of gravity on the grass [s]
    peak_sideslip: float  # [rad]
    min_speed: float  # [m/s]
    max_speed: float  # [m/s]


def run_lap(circuit, controller, plant, x0=None, t_max=180.0, dt_ctrl=0.05, dt_log=0.01):
    """Drive `plant` from the start line round `circuit` with `controller(t, x) -> u`.

    The controller is called every `dt_ctrl` seconds and its input held in
    between; the state is logged every `dt_log` seconds. The plant's friction
    follows the surface under the centre of gravity (asphalt or grass). The run
    ends when the lap is complete, at a collision with a traffic car, when the
    car leaves the grass, or at `t_max`.

    Returns the log (t, x, u, and the track coordinates s, e_y and friction mu
    at each sample; s counts up from 0 without wrapping) and the result.
    """
    track = circuit.track
    substeps = int(round(dt_ctrl / dt_log))
    base = plant.p
    plant.reset(circuit.initial_state() if x0 is None else x0)
    t, x = 0.0, plant.observe()
    s_prev, _ = track.project(x[0], x[1])
    progress = 0.0
    log = {key: [] for key in ("t", "x", "u", "s", "e_y", "mu")}
    clearances, margins = [], []
    lap_time, collided, left_track = None, False, False

    while lap_time is None and not collided and not left_track and t < t_max:
        clearances.append(circuit.clearance(t, x))
        margins.append(circuit.road_margin(x))
        if clearances[-1] <= 0.0:
            collided = True
            break
        u = plant.saturate(np.asarray(controller(t, x), dtype=float))
        for _ in range(substeps):
            s, e_y = track.project(x[0], x[1])
            progress += (s - s_prev + 0.5 * track.length) % track.length - 0.5 * track.length
            s_prev = s
            mu = float(circuit.friction(e_y))
            for key, value in zip(log, (t, x, u, progress, e_y, mu)):
                log[key].append(value)
            if np.isnan(mu):
                left_track = True
                break
            if progress >= track.length:
                lap_time = t
                break
            plant.p = replace(base, mu=mu)
            x = plant.step(u, dt_log)
            t += dt_log

    plant.p = base
    log = SimpleNamespace(**{key: np.array(value) for key, value in log.items()})
    result = LapResult(
        completed=lap_time is not None,
        lap_time=lap_time,
        collided=collided,
        left_track=left_track,
        min_clearance=min(clearances),
        road_margin=min(margins),
        time_on_grass=dt_log * np.count_nonzero(log.mu == circuit.mu_grass),
        peak_sideslip=np.abs(np.arctan2(log.x[:, IUY], log.x[:, IUX])).max(),
        min_speed=log.x[:, IUX].min(),
        max_speed=log.x[:, IUX].max(),
    )
    return log, result
