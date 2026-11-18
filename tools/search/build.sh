#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
workspace="${1:?Supply a new or empty build directory outside the source tree}"
shift
source /opt/ros/jazzy/setup.bash
python3 "$root/tools/search/build_native.py" "$workspace" "$@"
cd "$workspace"
workspace="$PWD"
colcon --log-base "$workspace/log" build --base-paths "$workspace/src" --build-base "$workspace/build" --install-base "$workspace/install" --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
