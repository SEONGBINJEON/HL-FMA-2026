#!/usr/bin/env bash
set -euo pipefail
pulse_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
pulse_build="$(mktemp -d -t hl-steer-pulse-test-XXXXXX)"
trap 'rm -rf -- "$pulse_build"' EXIT
"${CXX:-g++}" -std=c++11 -Wall -Wextra -Werror -pedantic -O1 -g \
 -fsanitize=address,undefined -fno-omit-frame-pointer \
 "$pulse_dir/control_core.cpp" "$pulse_dir/tests/pulse_gate_test.cpp" \
 -o "$pulse_build/pulse_test"
ASAN_OPTIONS=detect_leaks=1 "$pulse_build/pulse_test"
