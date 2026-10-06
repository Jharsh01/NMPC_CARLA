#!/usr/bin/env bash
# Start everything: set up the Python environment and build the C++ code if
# needed, check it, then run a scenario with pure pursuit and the NMPC and
# animate them together.
#
#   ./run.sh                       the lap of the circuit, in a window
#   ./run.sh --plant chrono        the same on the Project Chrono sedan
#   ./run.sh --save                the lap written to results/circuit.mp4
#   ./run.sh --speed 3             faster playback
#   ./run.sh overtake late_wet     an overtaking case (or: overtake all)
#   ./run.sh track --animate       draw the circuit with its traffic
#   ./run.sh --no-animate          results only
#   ./run.sh --test                run every test first, not only the quick check
#   ./run.sh --no-check            skip the check before the launch
#   ./run.sh --setup-only          only create the environment and build
#
# Other options go to the simulation (build/simulate -h): --controller, --perfect,
# --range, --noise, --ego-noise, --road-range, --seed, --params.
set -euo pipefail
cd "$(dirname "$0")"

# a sourced ROS 2 environment puts pytest plugins on PYTHONPATH that break collection
unset PYTHONPATH

check=quick
setup_only=0
animate=1
scenario=circuit
sim_args=()   # for build/simulate
view_args=()  # for python -m sim.view
while [[ $# -gt 0 ]]; do
    case "$1" in
        circuit|overtake|track) scenario="$1" ;;
        --test) check=full ;;
        --no-check) check=none ;;
        --setup-only) setup_only=1 ;;
        --no-animate) animate=0 ;;
        --animate) view_args+=("$1") ;;
        --speed) view_args+=("$1" "$2"); shift ;;
        --save)
            view_args+=("$1")
            if [[ $# -gt 1 && $2 != -* ]]; then view_args+=("$2"); shift; fi ;;
        -h|--help) sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        --*) sim_args+=("$1"); if [[ $# -gt 1 && $2 != -* ]]; then sim_args+=("$2"); shift; fi ;;
        *) sim_args+=("$1"); view_args+=("$1") ;;  # an overtake case
    esac
    shift
done

PY=.venv/bin/python

if [[ ! -x $PY ]]; then
    echo "== creating .venv"
    if python3 -m venv .venv 2>/dev/null; then
        :
    elif command -v uv >/dev/null; then
        rm -rf .venv
        uv venv .venv
    else
        echo "need python3-venv or uv to create the environment" >&2
        exit 1
    fi
fi

if ! $PY -c "import casadi, matplotlib, imageio_ffmpeg, pybind11, pytest" 2>/dev/null; then
    echo "== installing the Python dependencies"
    if $PY -m pip --version >/dev/null 2>&1; then
        $PY -m pip install -e ".[dev]"
    else
        uv pip install --python $PY -e ".[dev]"
    fi
fi

echo "== building"
[[ -f build/Makefile ]] || cmake -S . -B build >/dev/null
cmake --build build -j"$(nproc)" | grep -vE '^\[|Built target|Consolidate|Entering|Leaving' || true

[[ $setup_only == 1 ]] && { echo "== environment ready"; exit 0; }

case $check in
    quick) echo "== checking before the launch"; $PY preflight.py ;;
    full) echo "== running every test"; $PY preflight.py --full ;;
esac

if [[ $scenario != track ]]; then
    echo "== simulating: $scenario ${sim_args[*]:-}"
    build/simulate "$scenario" "${sim_args[@]}"
    [[ $animate == 1 ]] || exit 0
fi

if [[ -z ${DISPLAY:-}${WAYLAND_DISPLAY:-} && " ${view_args[*]:-} " != *" --save"* ]]; then
    echo "no display found; writing a video to results/ instead"
    view_args+=(--save)
fi
$PY -m sim.view "$scenario" "${view_args[@]}"
