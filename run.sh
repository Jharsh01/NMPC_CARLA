#!/usr/bin/env bash
# Start everything: set up the Python environment if needed, run the tests,
# then run pure pursuit and the NMPC on the scenarios and animate them together.
#
#   ./run.sh                    all scenarios, one window after another
#   ./run.sh late_wet           one scenario
#   ./run.sh --save             all scenarios written to results/*.gif
#   ./run.sh --skip-tests --speed 0.5 nominal
#   ./run.sh --setup-only       only create/update the environment
set -euo pipefail
cd "$(dirname "$0")"

# a sourced ROS 2 environment puts pytest plugins on PYTHONPATH that break collection
unset PYTHONPATH

scenario=all
speed=1.0
run_tests=1
save=""
setup_only=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-tests) run_tests=0 ;;
        --save) save=results ;;
        --speed) speed="$2"; shift ;;
        --setup-only) setup_only=1 ;;
        -h|--help) sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        -*) echo "unknown option: $1" >&2; exit 1 ;;
        *) scenario="$1" ;;
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

if ! $PY -c "import overtake_nmpc, casadi, matplotlib, pytest" 2>/dev/null; then
    echo "== installing the package and dependencies"
    if $PY -m pip --version >/dev/null 2>&1; then
        $PY -m pip install -e ".[dev]"
    else
        uv pip install --python $PY -e ".[dev]"
    fi
fi

[[ $setup_only == 1 ]] && { echo "== environment ready"; exit 0; }

if [[ $run_tests == 1 ]]; then
    echo "== running tests"
    $PY -m pytest -q
fi

echo "== running scenario: $scenario"
args=("$scenario" --speed "$speed")
[[ -n $save ]] && args+=(--save "$save")
if [[ -z $save && -z ${DISPLAY:-}${WAYLAND_DISPLAY:-} ]]; then
    echo "no display found; writing GIFs to results/ instead"
    args+=(--save results)
fi
$PY scripts/watch_overtake.py "${args[@]}"
