#!/usr/bin/env bash
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="$(mktemp -d -t hl-fma-manual-tests-XXXXXX)"
trap 'rm -rf -- "$build_dir"' EXIT
"${CXX:-g++}" -std=c++11 -Wall -Wextra -Werror -pedantic -O1 -g \
  -DHL_MANUAL_TEST -fsanitize=address,undefined -fno-omit-frame-pointer \
  "$project_dir/firmware/safe_manual/control_core.cpp" \
  "$project_dir/tests/manual_control_core_test.cpp" -o "$build_dir/core_tests"
ASAN_OPTIONS=detect_leaks=1 "$build_dir/core_tests" "$@"
