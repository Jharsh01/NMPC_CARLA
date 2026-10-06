#!/usr/bin/env bash
# Run Bench2Drive routes with an agent. The evaluator starts and stops the CARLA
# server itself.
#
#   cloud/b2d.sh --routes 24906                    one route, Bench2Drive's sample agent
#   cloud/b2d.sh --routes 24906,25169 --agent FILE these routes with your agent
#   cloud/b2d.sh --agent FILE                      every route of the routes file
#
#   --routes IDS         route ids separated by commas, or a range FIRST-LAST in file order
#   --routes-file FILE   default: Bench2Drive's leaderboard/data/bench2drive_0.0.4_val.xml (220 routes)
#   --agent FILE         Python file with the agent; default: Bench2Drive's npc_agent.py, which only follows the lane
#   --agent-config TEXT  passed to the agent's setup(), followed by "+" and the name of the run's save folder
#   --name NAME          results go to results/b2d/NAME/ (default: the date and time)
#   --resume             continue the run NAME from its results file
# Other options are passed to the evaluator (leaderboard_evaluator.py --help),
# for example --repetitions 3 or --port 30000.
#
# Results in results/b2d/NAME/: results.json (a record per route), live.txt, and
# save/, the folder the agent is given in SAVE_PATH.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=cloud/env.sh
source "$here/env.sh"

routes=""
routes_file=$LEADERBOARD_ROOT/data/bench2drive_0.0.4_val.xml
agent=$LEADERBOARD_ROOT/leaderboard/autoagents/npc_agent.py
agent_config=none
name=$(date +%m%d-%H%M%S)
resume=0
extra=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --routes) routes="$2"; shift ;;
        --routes-file) routes_file="$2"; shift ;;
        --agent) agent="$2"; shift ;;
        --agent-config) agent_config="$2"; shift ;;
        --name) name="$2"; shift ;;
        --resume) resume=1 ;;
        -h|--help) sed -n '2,19p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) extra+=("$1") ;;
    esac
    shift
done

[[ -f $CARLA_ROOT/CarlaUE4.sh ]] || { echo "CARLA is not installed in $CARLA_ROOT; run cloud/setup.sh" >&2; exit 1; }
[[ -x $B2D_PYTHON ]] || { echo ".venv-b2d is missing; run cloud/setup.sh" >&2; exit 1; }
# the evaluator runs in Bench2Drive's folder, so paths given here are made absolute first
routes_file=$(realpath "$routes_file")
agent=$(realpath "$agent")
out=$OV_REPO/results/b2d/$name
mkdir -p "$out/save"
export SAVE_PATH=$out/save
export PYTHONUNBUFFERED=1  # so the evaluator's progress shows when the output is piped to a file or tee

args=(--routes "$routes_file" --agent "$agent" --agent-config "$agent_config"
      --checkpoint "$out/results.json" --debug-checkpoint "$out/live.txt"
      --port 30000 --traffic-manager-port 50000)
[[ -n $routes ]] && args+=(--routes-subset "$routes")
[[ $resume == 1 ]] && args+=(--resume True)

cd "$B2D_ROOT"
status=0
$B2D_PYTHON leaderboard/leaderboard/leaderboard_evaluator.py "${args[@]}" "${extra[@]}" || status=$?

echo
echo "== results: $out/results.json"
$B2D_PYTHON - "$out/results.json" <<'EOF' || true
import json
import sys

try:
    records = json.load(open(sys.argv[1]))["_checkpoint"]["records"]
except (OSError, KeyError, ValueError):
    sys.exit("   no results file was written")
for r in records:
    infractions = ", ".join(k for k, v in r["infractions"].items() if v and k != "min_speed_infractions")
    print("   %-24s %-34s completed %5.1f %%  score %5.1f  %s" % (
        r["route_id"], r["status"], r["scores"]["score_route"], r["scores"]["score_composed"], infractions))
EOF
exit $status
