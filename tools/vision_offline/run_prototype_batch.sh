#!/usr/bin/env bash
# Frozen prototype run: 16 review samples (config frames: B02,B03,B06,S03,S05,S06; all others applied as-is)
# + 7 held-out clip frames not used in the review. Single-thread OpenCV, nice 10.
set -u
cd "$(dirname "$0")/../.."
P=tools/vision_offline/boundary_candidates.py
F="영상팀_답변 파일2/차에 붙인 영상/주행 영상 1"
O=docs/analysis/claude2_video_2026-09-18/prototype_final
rm -rf "$O"; mkdir -p "$O"
rc=0
for f in V03_BEND_APPROACH_t00-01-27.000 V03_BEND_ENTRY_t00-01-29.000 V03_BEND_PREEXIT_t00-02-02.167 V03_BEND_POSTEXIT_t00-02-03.667 V03_SCURVE_APPROACH_t00-03-19.000 V03_SCURVE_ENTRY_t00-03-33.000 V03_SCURVE_PREEXIT_t00-04-33.167 V03_SCURVE_POSTEXIT_t00-04-34.667; do
  nice -n 10 python3 $P "$F/추가작업_20260918/frames/$f.png" --out "$O" || rc=$?
done
for f in V03_BEND_f02880 V03_BEND_f03090 V03_BEND_f03330 V03_BEND_f03570 V03_SCURVE_f06570 V03_SCURVE_f07290 V03_SCURVE_f07710 V03_SCURVE_f08070; do
  nice -n 10 python3 $P "$F/V03_표본프레임/$f.png" --out "$O" || rc=$?
done
# held-out clip frames (clip start offsets 87 s / 199 s are the TEAM's documented values)
for t in 5.0 12.0 28.0; do
  nice -n 10 python3 $P "$F/추가작업_20260918/clips/V03_BEND_ENTRY_EXIT_v2.mp4" --time $t --clip-start-original-s 87 --out "$O" --tag HELDOUT_BEND_clip_t${t}s || rc=$?
done
for t in 8.0 30.0 52.0 65.0; do
  nice -n 10 python3 $P "$F/추가작업_20260918/clips/V03_SCURVE_ENTRY_EXIT_v2.mp4" --time $t --clip-start-original-s 199 --out "$O" --tag HELDOUT_SCURVE_clip_t${t}s || rc=$?
done
# rejection cases (expected exit 2)
nice -n 10 python3 $P "$F/추가작업_20260918/clips/V03_BEND_ENTRY_EXIT_v2.mp4" --out "$O" --tag REJECT_no_frame; echo "reject-no-frame exit=$?"
nice -n 10 python3 $P "/nonexistent/file.png" --out "$O" --tag REJECT_missing; echo "reject-missing exit=$?"
echo "batch rc=$rc"; exit $rc
