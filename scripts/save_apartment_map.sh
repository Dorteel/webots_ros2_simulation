#!/usr/bin/env bash
# A unique directory prevents map_saver_cli from overwriting any existing map.
set -euo pipefail
project=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
map_dir=$(mktemp -d "$project/maps/apartment_$(date +%Y%m%d_%H%M%S)_XXXXXX")
ros2 run nav2_map_server map_saver_cli -f "$map_dir/map" \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=15.0
printf 'Saved map: %s/map.yaml\n' "$map_dir"
