"""Run pure pursuit and the NMPC on a named scenario (or all of them) and animate both together.

    python scripts/watch_overtake.py nominal
    python scripts/watch_overtake.py late_wet --speed 0.5
    python scripts/watch_overtake.py nominal --controller nmpc
    python scripts/watch_overtake.py relaxed --save results/relaxed.gif
    python scripts/watch_overtake.py all                   # one window after another
    python scripts/watch_overtake.py all --save results    # one .gif per scenario
    python scripts/watch_overtake.py all --no-animate      # results table only
"""

import argparse
from dataclasses import replace
from pathlib import Path

import numpy as np

from overtake_nmpc.controllers.nmpc import NMPC
from overtake_nmpc.controllers.pure_pursuit import PurePursuit
from overtake_nmpc.model.params import VehicleParams
from overtake_nmpc.scenarios.overtake import SCENARIOS
from overtake_nmpc.sim.animate import animate
from overtake_nmpc.sim.closed_loop import run_overtake
from overtake_nmpc.sim.plant import Plant

CONTROLLERS = {"pure pursuit": PurePursuit, "NMPC": NMPC}
CHOICES = {"both": list(CONTROLLERS), "pure_pursuit": ["pure pursuit"], "nmpc": ["NMPC"]}


def run(name, labels):
    """Run each controller on scenario `name`: {label: (controller, log, result)}."""
    sc = SCENARIOS[name]
    params = replace(VehicleParams(), mu=sc.mu)
    out = {}
    for label in labels:
        controller = CONTROLLERS[label](sc, params)
        out[label] = (controller, *run_overtake(sc, controller, Plant(params)))
    return out


def report(name, label, controller, result):
    completed = "-" if result.t_complete is None else f"{result.t_complete:.1f} s"
    verdict = "PASS" if result.passed else "FAIL " + ",".join(result.failures)
    if isinstance(controller, PurePursuit):
        path = controller.path
        extra = (
            f"path out {path.length_out:3.0f} m, peak {path.a_lat_out:.2f} m/s^2"
            f" ({'within' if path.feasible else 'EXCEEDS'} mu*g)"
        )
    else:
        ms = 1e3 * np.array(controller.solve_times)
        extra = f"solve mean {ms.mean():.0f} ms, max {ms.max():.0f} ms, {controller.failures} not converged"
    print(
        f"{name:13s} {label:13s} {verdict:34s}"
        f" clearance {result.min_clearance:5.2f} m  road margin {result.road_margin:5.2f} m"
        f"  sideslip {np.degrees(result.peak_sideslip):4.1f} deg  completed {completed:>7s}  | {extra}"
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scenario", choices=["all", *sorted(SCENARIOS)], nargs="?", default="nominal")
    parser.add_argument("--controller", choices=list(CHOICES), default="both")
    parser.add_argument("--speed", type=float, default=1.0, help="playback rate relative to real time")
    parser.add_argument("--save", help="write a .gif instead of opening a window (a directory with 'all')")
    parser.add_argument("--no-animate", action="store_true", help="only print the results")
    args = parser.parse_args()

    names = list(SCENARIOS) if args.scenario == "all" else [args.scenario]
    runs = {}
    for name in names:
        runs[name] = run(name, CHOICES[args.controller])
        for label, (controller, _, result) in runs[name].items():
            report(name, label, controller, result)

    if args.no_animate:
        return
    for name, by_label in runs.items():
        save = args.save
        if save and args.scenario == "all":
            save = str(Path(save) / f"{name}.gif")
        if save:
            Path(save).parent.mkdir(parents=True, exist_ok=True)
            print(f"writing {save}")
        pure_pursuit = by_label.get("pure pursuit")
        animate(
            SCENARIOS[name],
            {label: (log, result) for label, (_, log, result) in by_label.items()},
            path=pure_pursuit[0].path if pure_pursuit else None,
            speed=args.speed,
            save=save,
            name=name,
        )


if __name__ == "__main__":
    main()
