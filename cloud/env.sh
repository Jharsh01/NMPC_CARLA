# Paths and environment for CARLA 0.9.15 and Bench2Drive. Source it, do not run it:
#
#   source cloud/env.sh
#
# The other scripts in cloud/ source it themselves. Set OV_EXTERNAL before
# sourcing to keep CARLA and Bench2Drive somewhere other than external/.

OV_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export OV_REPO
export OV_EXTERNAL="${OV_EXTERNAL:-$OV_REPO/external}"

export CARLA_ROOT="${CARLA_ROOT:-$OV_EXTERNAL/carla}"
export B2D_ROOT="${B2D_ROOT:-$OV_EXTERNAL/Bench2Drive}"
export SCENARIO_RUNNER_ROOT="$B2D_ROOT/scenario_runner"
export LEADERBOARD_ROOT="$B2D_ROOT/leaderboard"

# Python 3.8 environment for the CARLA client and the Bench2Drive evaluator.
# The repository's own environment (.venv, Python 3.10 or newer) is separate.
export B2D_PYTHON="$OV_REPO/.venv-b2d/bin/python"

# Replaces any PYTHONPATH: a sourced ROS 2 environment would otherwise leak its
# packages in. The CARLA folder supplies the `agents` package, this repository
# the controllers (overtake_core, built for Python 3.8 by cloud/setup.sh).
export PYTHONPATH="$LEADERBOARD_ROOT:$SCENARIO_RUNNER_ROOT:$CARLA_ROOT/PythonAPI/carla:$OV_REPO"

# Vulkan needs a runtime directory, which an ssh session without a desktop does not have
if [[ -z "${XDG_RUNTIME_DIR:-}" || ! -d "${XDG_RUNTIME_DIR:-}" ]]; then
    export XDG_RUNTIME_DIR="/tmp/runtime-$(id -u)"
    mkdir -p "$XDG_RUNTIME_DIR"
    chmod 700 "$XDG_RUNTIME_DIR"
fi
