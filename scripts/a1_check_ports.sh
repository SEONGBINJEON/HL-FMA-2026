#!/usr/bin/env bash
# Read-only pre-measurement check: which serial/gamepad nodes exist, who holds them,
# and whether an old controller process is still running. Opens NO port, moves nothing.
set -u
echo "KST: $(TZ=Asia/Seoul date '+%F %T')"
echo "== /dev/serial/by-id"; ls -l /dev/serial/by-id 2>/dev/null || echo "(none)"
echo "== expected Mega by-id (type150 upload record)"
MEGA=/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_95431353338351A01190-if00
if [ -e "$MEGA" ]; then echo "present -> $(readlink -f "$MEGA")"; else echo "MISSING: $MEGA"; fi
echo "== gamepad nodes"; ls -l /dev/input/js* 2>/dev/null || echo "(none)"
echo "== processes that may hold serial/gamepad"
pgrep -af "joystick_desktop|joystick_control|step_manual_control|manual_control|arduino_monitor|sensor_monitor|ultrasonic|gnss_record|rtk_record|imu_record|lidar_record|avrdude|arduino-cli" | grep -v -E "pgrep|a1_check_ports|dev_run_log|shell-snapshots" || echo "(none)"
echo "== holders of tty nodes (fuser)"
for n in /dev/ttyACM* /dev/ttyUSB*; do [ -e "$n" ] && { printf '%s: ' "$n"; fuser -v "$n" 2>&1 | tr '\n' ' ' ; echo; }; done
echo "== dialout membership: $(id -nG | tr ' ' '\n' | grep -c '^dialout$') (1 = yes)"
echo "== type150 hex vs build_provenance"
python3 - <<'PY'
import hashlib, json, pathlib
root = pathlib.Path(__file__).resolve().parents[1] if '__file__' in globals() else pathlib.Path.cwd()
root = pathlib.Path("/home/jeonseongbin/HL FMA v2")
prov = json.load(open(root/"firmware/joystick_manual/build_provenance.json"))
hexf = root/"firmware/joystick_manual/build_output/joystick_manual.ino.hex"
print("hex", "MATCH" if hashlib.sha256(hexf.read_bytes()).hexdigest()==prov["hex_sha256"] else "MISMATCH")
for f,h in prov["source_sha256"].items():
    print(f, "MATCH" if hashlib.sha256((root/"firmware/joystick_manual"/f).read_bytes()).hexdigest()==h else "MISMATCH")
PY
echo "== note: this script does not tell whether motor power is ON or which firmware runs on the board."
