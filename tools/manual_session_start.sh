#!/usr/bin/env bash
# Manual driving session: start LiDAR + OAK camera + IMU recorders in the background, then open the ONE
# joystick control window in the foreground. RTK recorder is NOT started here (keep the running one).
# Closing the joystick window stops the sensor recorders too. Usage: tools/manual_session_start.sh NAME
set -u
cd "$(dirname "$0")/.."
NAME="${1:-manual}"; STAMP="$(date +%H%M%S)"; BASE="data/manual_$(date +%Y%m%d)_${NAME}_${STAMP}"; mkdir -p "$BASE"
echo "session folder: $BASE"
setsid nohup python3 tools/lidar_record.py --until-interrupt --context "manual session $NAME $STAMP" > "$BASE/lidar_stdout.log" 2>&1 < /dev/null &
LIDAR=$!
setsid nohup .venv-oak/bin/python3 tools/oak_burst_capture.py --output-dir "$BASE/oak_video" --frames 18000 --every 3 --width 1280 --height 720 --label obs --ae-comp -3 > "$BASE/oak_stdout.log" 2>&1 < /dev/null &
OAK=$!
setsid nohup .venv-rtk/bin/python3 tools/imu_record.py --until-interrupt --output-dir "$BASE/imu" > "$BASE/imu_stdout.log" 2>&1 < /dev/null &
IMU=$!
sleep 8
echo "--- recorder check ---"
grep -o "OUTPUT=.*" "$BASE/lidar_stdout.log" | head -1 || echo "LIDAR: no OUTPUT line yet (see $BASE/lidar_stdout.log)"
echo "OAK frames: $(ls "$BASE/oak_video" 2>/dev/null | wc -l)"; echo "IMU rows: $(wc -l < "$BASE/imu/samples.jsonl" 2>/dev/null || echo 0)"
kill -0 $LIDAR 2>/dev/null && echo "lidar running" || echo "LIDAR NOT RUNNING"
kill -0 $OAK 2>/dev/null && echo "oak running" || echo "OAK NOT RUNNING"
kill -0 $IMU 2>/dev/null && echo "imu running" || echo "IMU NOT RUNNING (optional)"
echo "--- opening joystick window (Mega: this window only). Close it to end the session. ---"
python3 tools/joystick_desktop.py --connect --mapping-confirmed --initial-drive-pwm 50 --output-dir "$BASE/joystick"
echo "--- joystick closed: stopping recorders ---"
kill -INT $LIDAR $OAK $IMU 2>/dev/null; sleep 3; kill -TERM $LIDAR $OAK $IMU 2>/dev/null
echo "done: $BASE"
