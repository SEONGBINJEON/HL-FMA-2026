#!/usr/bin/env bash
# Compile + run the candidate target-steering scenario tests (no hardware).
# Use through tools/dev_run_log.py to keep the execution record:
#   python3 tools/dev_run_log.py --name c1_steer_target_cpp_compile \
#     --source firmware/steer_target/steer_target_core.h --source firmware/steer_target/steer_target_core.cpp \
#     --source firmware/steer_target/tests/steer_target_test.cpp -- bash firmware/steer_target/tests/run.sh
set -euo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
build_dir="$(mktemp -d -t hl-fma-steer-target-XXXXXX)"
trap 'rm -rf -- "$build_dir"' EXIT
"${CXX:-g++}" -std=c++11 -Wall -Wextra -Werror -pedantic -O1 -g \
  -fsanitize=address,undefined -fno-omit-frame-pointer \
  "$project_dir/firmware/steer_target/tests/steer_target_test.cpp" \
  "$project_dir/firmware/steer_target/steer_target_core.cpp" -o "$build_dir/steer_target_test"
ASAN_OPTIONS=detect_leaks=1 "$build_dir/steer_target_test" "$@"
