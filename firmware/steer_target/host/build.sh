#!/usr/bin/env bash
# Build libsteer_host.so (C1.1 target core + manual type150 core model) for ctypes.
# Output: firmware/steer_target/host/build/libsteer_host.so + build_info.json (source hashes).
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd -- "$here/../../.." && pwd)"
out="$here/build"; mkdir -p "$out"
srcs=("$here/steer_host_cabi.cpp" "$here/../steer_target_core.cpp" "$root/firmware/joystick_manual/control_core.cpp")
"${CXX:-g++}" -std=c++11 -O2 -Wall -Wextra -Werror -fPIC -shared "${srcs[@]}" -o "$out/libsteer_host.so"
python3 - "$out" "$root" "${srcs[@]}" "$here/../steer_target_core.h" "$root/firmware/joystick_manual/control_core.h" "$root/firmware/joystick_manual/step_gate.h" <<'PY'
import hashlib, json, sys, datetime, pathlib
out, root, *files = sys.argv[1:]
h = lambda p: hashlib.sha256(open(p, "rb").read()).hexdigest()
info = {"built_kst": datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=9))).isoformat(timespec="seconds"),
        "library": "libsteer_host.so", "library_sha256": h(pathlib.Path(out) / "libsteer_host.so"),
        "sources_sha256": {str(pathlib.Path(f).resolve().relative_to(root)): h(f) for f in files},
        "note": "host-side model; not the uploaded firmware image"}
(pathlib.Path(out) / "build_info.json").write_text(json.dumps(info, indent=2) + "\n")
print(json.dumps(info, indent=1))
PY
