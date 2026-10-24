#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d)
trap 'rm -rf "$build_dir"' EXIT
for source in test/*_test.cpp; do
  executable="$build_dir/$(basename "$source" .cpp)"
  "${CXX:-c++}" -std="${CXXSTD:-c++17}" -Wall -Wextra -Werror -pedantic \
    -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer \
    -g -I include src/autotune/self_tuning_auton.cpp "$source" -o "$executable"
  "$executable"
done
python3 -m unittest discover -s test -p '*_test.py'
