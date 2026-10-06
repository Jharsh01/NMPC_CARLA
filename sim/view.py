"""Animate the logs written by the simulation generator (build/simulate).

    python -m sim.view circuit                     # results/logs/circuit, opens a window
    python -m sim.view circuit --save              # write results/circuit.mp4
    python -m sim.view overtake late_wet --speed 0.5
    python -m sim.view overtake all --save         # one video per case in results/
    python -m sim.view track --animate             # draw the circuit, traffic circulating
"""

import argparse
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from overtake_core import NU, NX, SCENARIOS, Circuit, OvertakePath

RESULTS = Path("results")
ORDER = {"overtake": ("pure_pursuit", "nmpc"), "circuit": ("nmpc", "pure_pursuit")}  # drawing order of the runs


def read_run(file):
    """The log (t, x, u and any further columns) and result of one run, from its .csv and .json."""
    with open(file) as lines:
        names = lines.readline().strip().split(",")
    table = np.atleast_2d(np.loadtxt(file, delimiter=",", skiprows=1))
    log = SimpleNamespace(t=table[:, 0], x=table[:, 1 : 1 + NX], u=table[:, 1 + NX : 1 + NX + NU])
    for column, name in enumerate(names[1 + NX + NU :], start=1 + NX + NU):
        setattr(log, name, table[:, column])
    result = json.loads(Path(file).with_suffix(".json").read_text())
    if "failures" in result:
        result["failures"] = tuple(result["failures"])
    return log, SimpleNamespace(**result)


def read_runs(directory, scenario):
    """{controller label: (log, result)} for the runs logged in `directory`."""
    runs = {}
    for key in ORDER[scenario]:
        file = Path(directory) / f"{key}.csv"
        if file.exists():
            log, result = read_run(file)
            runs[result.controller] = (log, result)
    if not runs:
        raise SystemExit(f"no logs in {directory}; run the simulation first (./run.sh, or build/simulate)")
    return runs


def video_path(save, name, suffix=".mp4"):
    """Where to write the video of `name`: `save` is a file, a directory, or "" for results/."""
    if save is None:
        return None
    path = Path(save) if save else RESULTS
    if not path.suffix:
        path = path / f"{name}{suffix}"
    print(f"writing {path}")
    return str(path)


def view_overtake(args):
    from sim.scenarios.overtake.animate import animate

    for name in list(SCENARIOS) if args.case == "all" else [args.case]:
        runs = read_runs(Path(args.logs) / name, "overtake")
        path = None
        if "pure pursuit" in runs and getattr(runs["pure pursuit"][1], "path", None):
            path = OvertakePath(**runs["pure pursuit"][1].path)
        animate(SCENARIOS[name], runs, path=path, speed=args.speed, save=video_path(args.save, name), name=name)


def view_circuit(args):
    from sim.scenarios.circuit.animate import animate_lap

    runs = read_runs(Path(args.logs) / "circuit", "circuit")
    save = video_path(args.save, "circuit")
    animate_lap(Circuit(), runs, speed=args.speed, save=save, dpi=70 if save else 100)


def view_track(args):
    from sim.scenarios.circuit.animate import show_circuit

    save = video_path(args.save, "track", ".mp4" if args.animate else ".png")
    show_circuit(Circuit(), animate=args.animate, speed=args.speed, save=save)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="scenario", required=True)

    def command(name, run, help, speed=1.0):
        sub = commands.add_parser(name, help=help)
        sub.set_defaults(run=run)
        sub.add_argument("--speed", type=float, default=speed, help="playback rate relative to real time")
        sub.add_argument(
            "--save",
            nargs="?",
            const="",
            metavar="PATH",
            help="write a video instead of opening a window: a file (.mp4, .gif) or a directory, default results/",
        )
        sub.add_argument("--logs", default=str(RESULTS / "logs"), help="where the generator wrote its logs")
        return sub

    sub = command("overtake", view_overtake, "scenario 1: overtake a slower car on a straight road")
    sub.add_argument("case", choices=["all", *SCENARIOS], nargs="?", default="nominal")
    command("circuit", view_circuit, "scenario 2: a lap of the circuit with slower traffic")
    sub = command("track", view_track, "draw the circuit", speed=20.0)
    sub.add_argument("--animate", action="store_true", help="animate the traffic")

    args = parser.parse_args(argv)
    args.run(args)


if __name__ == "__main__":
    main()
