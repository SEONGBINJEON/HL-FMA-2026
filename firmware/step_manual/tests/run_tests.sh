#!/usr/bin/env bash
set -euo pipefail
step_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
step_build="$(mktemp -d -t hl-step-manual-test-XXXXXX)"
trap 'rm -rf -- "$step_build"' EXIT
"${CXX:-g++}" -std=c++11 -Wall -Wextra -Werror -pedantic -O1 -g \
 -fsanitize=address,undefined -fno-omit-frame-pointer \
 "$step_dir/control_core.cpp" "$step_dir/tests/step_gate_test.cpp" \
 -o "$step_build/step_test"
ASAN_OPTIONS=detect_leaks=1 "$step_build/step_test"
