#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT
# Keep the hardware adapter out of host builds. Everything else is the same
# implementation linked into a PROS library/consumer.
core_sources=(
  src/autotune/self_tuning_auton.cpp
  src/autotune/session_control.cpp
  src/autotune/battery_monitor.cpp
  src/autotune/telemetry.cpp
  src/autotune/telemetry_output.cpp
  src/autotune/tuning_return.cpp
  src/autotune/tuning_config.cpp
  src/autotune/gain_search.cpp
  src/autotune/motion_tracking.cpp
  src/autotune/tuning_motion.cpp
  src/autotune/output.cpp
)
for source in test/*_test.cpp; do
  executable="$build_dir/$(basename "$source" .cpp)"
  "${CXX:-c++}" -std="${CXXSTD:-c++17}" -Wall -Wextra -Werror -pedantic -pthread \
    -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer \
    -g -I include "${core_sources[@]}" "$source" -o "$executable"
  "$executable"
done
python3 -m unittest discover -s test -p '*_test.py'
