#!/usr/bin/env bash
# Run the Depot 07 integration on the team's WSL/Jazzy machine.
set -eo pipefail
repo="$(cd "$(dirname "$0")/../.." && pwd)"
assets="$(realpath "${1:?Usage: run_depot07.sh ASSET_DIR NATIVE_WS OUTPUT_DIR}")"
native="$(realpath "${2:?Supply the native RACER workspace}")"
out="$(realpath -m "${3:?Supply a new output directory outside the repository}")"
case "$out/" in
    "$repo/"*|"$assets/"*) echo "Output must be outside the repository and asset pack." >&2; exit 1 ;;
esac

# The existing runner uses fixed local GUI and DDS ports.
exec 3>/tmp/aspire-sitl.lock
if ! flock -n 3 || ps -eo comm= | grep -E '^(Xvfb|gz sim|gz-sim|px4|MicroXRCEAgent|exploration_nod)$' >/dev/null; then
    echo "Another SITL or Xvfb session is running." >&2
    exit 1
fi
backing_free() { df -B1 --output=avail /mnt/i | tail -1; }
if (( $(backing_free) < 6400000000 )); then
    echo "Depot 07 needs at least 6.4 GB free on the WSL backing drive (/mnt/i)." >&2
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source /opt/aspire/build/ros-ws/install/setup.bash
source "$native/install/local_setup.bash"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$repo:$repo/src:$repo/src/annual_planner/src:/opt/aspire/venvs/analysis/lib/python3.12/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="/opt/aspire/build/agent-install/lib:${LD_LIBRARY_PATH:-}"
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/aspire/engineering/dds-local.xml
export GZ_IP=127.0.0.1 LP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export RACER_CPU_PROFILE=render3-perception2-px4reserved RACER_EXECUTION_SAFETY=1
export RACER_GZ_SHUTDOWN_GUARD=/opt/aspire/build/gazebo-guard-x86/libgz_sensor_stop_guard.so
export RACER_PX4_LOGGER_TOPICS=/opt/aspire/engineering/logger-topics.txt
export RACER_REFERENCE_NATIVE_PREFIX="$native/install/racer_core"
export ASPIRE_SAFETY_MAP_TIMING=1 ASPIRE_NATIVE_MAP_DIRECT_DDS=1
export ASPIRE_KNOWN_TRANSIT=1 ASPIRE_FRONTIER_STEPS=1 ASPIRE_NATIVE_COLLISION_STOP=1
export YOLO_CONFIG_DIR="$out/yolo-settings"
ulimit -c 0

(cd "$assets" && sha256sum -c SHA256SUMS --quiet)
mkdir -p "$(dirname "$out")"
mkdir "$out"
cd "$repo"
python3 -B -m tools.scenarios.depot07 --assets "$assets" --output "$out/scene"

printf 'RViz during the run: http://127.0.0.1:18680/vnc.html?autoconnect=true&resize=scale\n'
printf 'Launch log: %s\n' "$out/launch.log"
timeout -s INT -k 45s 770s python3 -B -m tools.flight.run \
    --prepared "$out/scene" --output "$out/run" \
    --external /planner/external --px4-build /opt/aspire/build/px4-sitl \
    --confirmation-mode registry --detector yolo11n-seg \
    --segmentation-weights /opt/aspire/assets/yolo11n-seg.pt \
    --detector-python-path /opt/aspire/venvs/seg/lib/python3.12/site-packages \
    --stop-and-plan --recover-no-path --bearing-bias --rviz --sim-real-time-factor 0.42 \
    --budget-clock sim --seconds 180 --wall-watchdog-seconds 700 \
    > "$out/launch.log" 2>&1 &
run_pid=$!
trap 'kill -INT "$run_pid" 2>/dev/null || true' INT TERM
while kill -0 "$run_pid" 2>/dev/null; do
    if (( $(backing_free) < 4200000000 )); then
        echo "Stopping SITL before the WSL backing drive fills." >&2
        kill -INT "$run_pid"
        break
    fi
    sleep 4 || :
done
status=0
wait "$run_pid" || status=$?
printf '%s\n' "$status" > "$out/exit-code.txt"
exit "$status"
