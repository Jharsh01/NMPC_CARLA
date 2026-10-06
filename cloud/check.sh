#!/usr/bin/env bash
# Check that this machine can run CARLA and Bench2Drive with this repository's controllers.
#
#   cloud/check.sh              the environments, the GPU and a start of the CARLA server (a minute or two)
#   cloud/check.sh --route      also one Bench2Drive route with its sample agent (several minutes)
#   cloud/check.sh --no-carla   only what needs no CARLA: the environments and the controllers
#
# Every check is run; the exit status is 1 if one of them failed.
set -uo pipefail
cd "$(dirname "$0")/.."

carla=1
route=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-carla) carla=0 ;;
        --route) route=1 ;;
        -h|--help) sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option: $1 (see --help)" >&2; exit 2 ;;
    esac
    shift
done

# shellcheck source=cloud/env.sh
source cloud/env.sh

failed=0
ok() { echo "   ok    $*"; }
fail() { echo "   FAIL  $*"; failed=1; }
hint() { echo "         $*"; }

CHECK_PORT=28000  # the server also uses the two ports above it
CHECK_ROUTE=24906  # Town03, a map of the base CARLA package
logs=results/cloud
mkdir -p $logs

# ---- this repository

echo "== this repository (.venv)"
if .venv/bin/python preflight.py >$logs/preflight.log 2>&1; then
    ok "quick tests: $(tail -n 1 $logs/preflight.log)"
else
    fail "quick tests (python preflight.py):"
    tail -n 15 $logs/preflight.log | sed 's/^/         /'
fi

# ---- the Python 3.8 environment

echo "== Bench2Drive environment (.venv-b2d)"
if $B2D_PYTHON - <<'EOF'
import sys
from importlib.metadata import version

import casadi
import numpy

import carla  # noqa: F401
import overtake_core

c = overtake_core.Circuit()
controller = overtake_core.TrackNMPC(c, overtake_core.VehicleParams())
controller(0.0, c.initial_state())
assert controller.failures == 0, "the NMPC solve did not converge"
print("   ok    Python %d.%d, carla client %s, casadi %s, numpy %s; one NMPC solve in %.0f ms" % (
    sys.version_info[0], sys.version_info[1], version("carla"), casadi.__version__, numpy.__version__,
    1e3 * controller.solve_times[0]))
EOF
then :; else
    fail "CARLA client, CasADi or the controllers (overtake_core) in .venv-b2d; run cloud/setup.sh"
fi

if [[ $carla == 0 ]]; then
    [[ $failed == 0 ]] && echo "== all checks passed (CARLA not checked)"
    exit $failed
fi

if [[ ! -f $CARLA_ROOT/CarlaUE4.sh ]]; then
    fail "CARLA is not installed in $CARLA_ROOT; run cloud/setup.sh"
    exit 1
fi

if $B2D_PYTHON -c "import leaderboard.leaderboard_evaluator; from agents.navigation.basic_agent import BasicAgent" 2>$logs/import.log; then
    ok "the Bench2Drive evaluator imports"
else
    fail "the Bench2Drive evaluator does not import:"
    tail -n 5 $logs/import.log | sed 's/^/         /'
fi

# ---- GPU and Vulkan

echo "== GPU"
if gpu=$(nvidia-smi --query-gpu=name,memory.total,driver_version --format=csv,noheader 2>/dev/null); then
    ok "$gpu"
else
    fail "nvidia-smi does not run: no NVIDIA GPU or no driver"
fi
icd=$(ls /usr/share/vulkan/icd.d/nvidia_icd*.json /etc/vulkan/icd.d/nvidia_icd*.json 2>/dev/null | head -n 1)
if [[ -n $icd && -f $icd ]]; then
    ok "Vulkan driver description: $icd"
else
    # reported, not failed: the start of the server below is the real test
    echo "   note  no nvidia_icd.json in /usr/share/vulkan/icd.d or /etc/vulkan/icd.d"
    hint "If CARLA does not start, the NVIDIA driver is probably installed without its"
    hint "graphics libraries (a compute-only install). Install the libnvidia-gl package"
    hint "of the driver's version, or use an image with the full driver."
fi
if command -v vulkaninfo >/dev/null; then
    devices=$(vulkaninfo 2>/dev/null | grep -i 'deviceName' | sort -u | sed 's/.*= *//' | paste -sd, -)
    if [[ $devices == *NVIDIA* || $devices == *GeForce* || $devices == *Tesla* || $devices == *RTX* ]]; then
        ok "Vulkan sees: $devices"
    else
        echo "   note  Vulkan sees no NVIDIA device (${devices:-nothing}); CARLA renders with Vulkan"
    fi
fi
if [[ $EUID -eq 0 ]]; then
    fail "running as root: CARLA refuses to start as root; use an ordinary user"
fi

# ---- a start of the server

echo "== CARLA server"
log=$logs/carla_check.log
setsid "$CARLA_ROOT/CarlaUE4.sh" -RenderOffScreen -nosound -carla-rpc-port=$CHECK_PORT >"$log" 2>&1 &
server=$!
trap 'kill -9 -- -$server 2>/dev/null' EXIT
if $B2D_PYTHON - $CHECK_PORT $server <<'EOF'
import os
import sys
import time

import carla

port, server = int(sys.argv[1]), int(sys.argv[2])
deadline = time.time() + 240.0
while True:
    try:
        os.kill(server, 0)
    except OSError:
        print("   FAIL  the server stopped by itself")
        sys.exit(1)
    try:
        client = carla.Client("localhost", port)
        client.set_timeout(10.0)
        version = client.get_server_version()
        maps = sorted(set(name.split("/")[-1] for name in client.get_available_maps()))
        break
    except RuntimeError as error:
        if time.time() > deadline:
            print("   FAIL  no answer from the server after 4 minutes: %s" % error)
            sys.exit(1)
        time.sleep(5.0)
print("   ok    server %s answers" % version)
print("   ok    maps: %s" % " ".join(maps))
missing = [town for town in ("Town06", "Town07", "Town11", "Town12", "Town13", "Town15") if town not in maps]
if missing:
    print("   note  additional maps not installed: %s (cloud/setup.sh installs them)" % " ".join(missing))
EOF
then :; else
    failed=1
    hint "last lines of $log:"
    tail -n 15 "$log" | sed 's/^/         | /'
fi
kill -9 -- -$server 2>/dev/null
trap - EXIT
wait $server 2>/dev/null

# ---- one Bench2Drive route

if [[ $route == 1 && $failed == 0 ]]; then
    echo "== Bench2Drive route $CHECK_ROUTE with the sample agent"
    cloud/b2d.sh --routes $CHECK_ROUTE --name check >$logs/route_check.log 2>&1
    if $B2D_PYTHON - results/b2d/check/results.json <<'EOF'
import json
import sys

records = json.load(open(sys.argv[1]))["_checkpoint"]["records"]
assert records, "no route record"
r = records[0]
# the sample agent only follows the lane, so it is expected to stop behind the accident
print("   ok    %s: %s, route completed %.0f %%, driving score %.1f" % (
    r["route_id"], r["status"], r["scores"]["score_route"], r["scores"]["score_composed"]))
EOF
    then :; else
        fail "the evaluator wrote no route record; see $logs/route_check.log:"
        tail -n 15 $logs/route_check.log | sed 's/^/         | /'
    fi
elif [[ $route == 1 ]]; then
    echo "== Bench2Drive route skipped: fix the failures above first"
fi

[[ $failed == 0 ]] && echo "== all checks passed"
exit $failed
