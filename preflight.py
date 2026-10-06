"""Checks to run before launching a simulation. They test the C++ code through its
Python module, so build first (cmake -S . -B build && cmake --build build -j).

    python preflight.py          # quick: parameters, vehicle model, plants, scenarios, sensing (a few seconds)
    python preflight.py --full   # everything, including the closed-loop runs of both controllers
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--full", action="store_true", help="also run the slow closed-loop tests")
    args = parser.parse_args()

    command = [sys.executable, "-m", "pytest", "-q"] + ([] if args.full else ["-m", "not slow"])
    # a sourced ROS 2 environment puts pytest plugins on PYTHONPATH that break collection
    env = {key: value for key, value in os.environ.items() if key != "PYTHONPATH"}
    return subprocess.call(command, cwd=Path(__file__).parent, env=env)


if __name__ == "__main__":
    sys.exit(main())
