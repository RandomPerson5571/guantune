#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT
# Motion execution is compiled against real PROS/LemLib in the dependency build.
# Native tests exercise policy, position analysis, recording and returned results.
core_sources=(
  src/autotune/core/tuning_config.cpp
  src/autotune/core/gain_search.cpp
  src/autotune/motion/motion_tracking.cpp
  src/autotune/telemetry/telemetry.cpp
)
for source in test/host/*_test.cpp; do
  executable="$build_dir/$(basename "$source" .cpp)"
  "${CXX:-c++}" -std="${CXXSTD:-c++17}" -Wall -Wextra -Werror -pedantic -pthread \
    -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer \
    -g -I include "${core_sources[@]}" "$source" -o "$executable"
  "$executable"
done
python3 -m unittest discover -s test/packaging -p '*_test.py'
