"""Drive one lap of the circuit with the NMPC and with pure pursuit, compare and animate them.

    python scripts/run_lap.py                          # both, opens a window
    python scripts/run_lap.py --controller nmpc        # or pure_pursuit
    python scripts/run_lap.py --speed 3                # faster playback
    python scripts/run_lap.py --save results/lap.gif
    python scripts/run_lap.py --no-animate             # results only
"""

import argparse
from pathlib import Path

import numpy as np

from overtake_nmpc.controllers.nmpc_track import TrackNMPC
from overtake_nmpc.controllers.pure_pursuit_track import TrackPurePursuit
from overtake_nmpc.model.params import VehicleParams
from overtake_nmpc.scenarios.circuit import Circuit
from overtake_nmpc.sim.lap import run_lap
from overtake_nmpc.sim.plant import Plant
from overtake_nmpc.sim.track_plot import animate_lap

CONTROLLERS = {"NMPC": TrackNMPC, "pure pursuit": TrackPurePursuit}
CHOICES = {"both": list(CONTROLLERS), "nmpc": ["NMPC"], "pure_pursuit": ["pure pursuit"]}


def passed(circuit, log):
    """Number of traffic cars the ego got ahead of during the run."""
    lap = circuit.track.length
    t = log.t[::50]
    count = 0
    for car in circuit.traffic:
        s = np.unwrap(circuit.traffic_s(car, t), period=lap)
        ahead = (s[0] - log.s[0]) % lap  # every car starts somewhere ahead on the lap
        count += log.s[-1] - log.s[0] > ahead + s[-1] - s[0]
    return count


def report(circuit, label, controller, log, r):
    outcome = f"{r.lap_time:6.1f} s" if r.completed else "   none"
    note = "collision" if r.collided else "left the track" if r.left_track else ""
    line = (
        f"{label:13s} lap {outcome}   speed {3.6 * r.min_speed:2.0f}-{3.6 * r.max_speed:2.0f} km/h"
        f"   passed {passed(circuit, log)}/{len(circuit.traffic)}   clearance {r.min_clearance:5.2f} m"
        f"   asphalt margin {r.road_margin:5.2f} m   on grass {r.time_on_grass:3.1f} s"
        f"   sideslip {np.degrees(r.peak_sideslip):4.1f} deg"
    )
    if isinstance(controller, TrackNMPC):
        ms = 1e3 * np.array(controller.solve_times)
        line += f"   | solve mean {ms.mean():.0f} ms, max {ms.max():.0f} ms, {controller.failures} not converged"
    print(line + (f"   {note} at {log.t[-1]:.1f} s" if note else ""))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--controller", choices=list(CHOICES), default="both")
    parser.add_argument("--speed", type=float, default=1.0, help="playback rate relative to real time")
    parser.add_argument("--save", help="write a .gif instead of opening a window")
    parser.add_argument("--no-animate", action="store_true", help="only print the results")
    args = parser.parse_args()

    circuit = Circuit()
    params = VehicleParams()
    runs = {}
    for label in CHOICES[args.controller]:
        print(f"driving the lap with {label} ...")
        controller = CONTROLLERS[label](circuit, params)
        log, result = run_lap(circuit, controller, Plant(params))
        report(circuit, label, controller, log, result)
        runs[label] = (log, result)

    if args.no_animate:
        return
    if args.save:
        Path(args.save).parent.mkdir(parents=True, exist_ok=True)
        print(f"writing {args.save}")
    animate_lap(circuit, runs, speed=args.speed, save=args.save, dpi=70 if args.save else 100)


if __name__ == "__main__":
    main()
