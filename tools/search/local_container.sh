#!/usr/bin/env bash
# Existing local Colima only. Source is copied, never mounted over original work.
set -eo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
context=colima-racer-gvp
container=annual-px4-research
stage="${SEARCH_STAGE:-/tmp/person-search-$(date +%Y%m%d-%H%M%S)}"
docker --context "$context" inspect "$container" >/dev/null
docker --context "$context" exec "$container" test ! -e "$stage"
docker --context "$context" cp "$root/." "$container:$stage"
printf 'Source staged at %s in local container %s\n' "$stage" "$container"
printf 'Build (outside source): bash %s/tools/search/build.sh %s-native\n' "$stage" "$stage"
