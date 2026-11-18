#!/usr/bin/env bash
# Host entry: restore from persistent evidence, no Gazebo/PX4/planner startup.
set -eo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
archive="${1:-$(dirname "$root")/task-3-evidence/stage-3/runs.tar.gz}"
test -f "$archive"
stage="/tmp/racer-replay-$(date +%Y%m%d-%H%M%S)"
context=colima-racer-gvp
container=annual-px4-research
docker --context "$context" exec "$container" test ! -e "$stage"
docker --context "$context" cp "$root/." "$container:$stage"
docker --context "$context" cp "$archive" "$container:$stage-records.tar.gz"
docker --context "$context" exec "$container" bash -c '
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$1:$1/src:$1/src/annual_planner/src:$PYTHONPATH"
cd "$1"
python3 tools/flight/replay_confirmed.py --run "$1-records.tar.gz" --output "$1-output" --seconds 120
' _ "$stage"
