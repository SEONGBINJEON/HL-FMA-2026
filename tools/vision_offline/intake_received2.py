#!/usr/bin/env python3
"""Claude 2 · read-only intake of `영상팀_답변 파일2/` (2026-09-18).

- Hashes every file, compares with the 09-17 / 09-18 inventories (SHA256).
- Classifies: NEW / SAME_AS_PREVIOUS / ZIP_DUPLICATE_OF_EXTRACTED / UNREADABLE.
- Preserves only unique new payloads into data/video_team_received2_20260918/
  (ZIPs whose members are all present as extracted files are NOT copied).
- Opens every new MP4 with OpenCV (1 thread) and decodes first/middle/last frame.
- Never moves/deletes/modifies the source folder.

usage: intake_received2.py [--dry-run]
"""
import argparse, hashlib, json, shutil, sys, zipfile, zlib
from collections import Counter
from datetime import datetime, timezone, timedelta
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "영상팀_답변 파일2"
DST = ROOT / "data/video_team_received2_20260918"
OUT = ROOT / "docs/analysis/claude2_video_2026-09-18/inventory2.json"
PREV = [ROOT / "docs/analysis/video_team_review_2026-09-17/inventory.json",
        ROOT / "docs/analysis/video_team_review_2026-09-18/inventory.json"]
KST = timezone(timedelta(hours=9))


def sha256(p):
    h = hashlib.sha256()
    with p.open("rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def crc32(p):
    c = 0
    with p.open("rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            c = zlib.crc32(b, c)
    return c & 0xFFFFFFFF


def load_prev():
    known = {}
    for inv in PREV:
        if not inv.is_file():
            continue
        d = json.loads(inv.read_text())
        for f in d["files"]:
            s = f.get("sha256")
            if s and s not in known:
                known[s] = f.get("preserved_at") or f.get("path") or f.get("source")
    return known


def probe_video(p):
    import cv2
    cv2.setNumThreads(1)
    c = cv2.VideoCapture(str(p))
    if not c.isOpened():
        return {"reader": "OpenCV", "opened": False}
    n = int(c.get(cv2.CAP_PROP_FRAME_COUNT)); fps = c.get(cv2.CAP_PROP_FPS)
    info = {"reader": "OpenCV", "opened": True, "width": int(c.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(c.get(cv2.CAP_PROP_FRAME_HEIGHT)), "fps": fps, "frames_reported": n,
            "duration_s_from_frames_fps": (n / fps) if fps else None, "decoded_ok": [], "decode_failed": []}
    for idx in [0, n // 2, max(n - 1, 0)]:
        c.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ok, fr = c.read()
        (info["decoded_ok"] if ok else info["decode_failed"]).append(idx)
    c.release()
    return info


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args(argv)
    if not SRC.is_dir():
        print("source folder missing:", SRC, file=sys.stderr); return 2
    known = load_prev()
    files = sorted(p for p in SRC.rglob("*") if p.is_file())
    rows = []
    by_rel_crc = {}
    for p in files:
        rows.append({"source": str(p.relative_to(ROOT)), "rel": str(p.relative_to(SRC)), "size": p.stat().st_size, "sha256": sha256(p)})
    # crc of non-zip files for zip-member matching
    for r in rows:
        p = ROOT / r["source"]
        if p.suffix.lower() != ".zip" and p.name != "zitvRJOZ":
            by_rel_crc[(p.name, r["size"], crc32(p))] = r["source"]
    if not a.dry_run:
        DST.mkdir(parents=True, exist_ok=True)
    for r in rows:
        p = ROOT / r["source"]
        prev = known.get(r["sha256"])
        r["previous_match"] = prev
        is_zip_like = p.suffix.lower() == ".zip" or p.name == "zitvRJOZ"
        if is_zip_like:
            try:
                zf = zipfile.ZipFile(p)
                members = [i for i in zf.infolist() if not i.is_dir()]
                matched = [m.filename for m in members if (Path(m.filename).name, m.file_size, m.CRC) in by_rel_crc]
                r["zip"] = {"readable": True, "members": len(members),
                            "members_matching_extracted_files": len(matched),
                            "all_members_present_as_files": len(matched) == len(members) and len(members) > 0}
            except zipfile.BadZipFile as e:
                r["zip"] = {"readable": False, "error": str(e)}
        if prev:
            r["status"] = "SAME_AS_PREVIOUS"; r["preserved_at"] = prev
        elif is_zip_like and r.get("zip", {}).get("readable") is False:
            r["status"] = "UNREADABLE_ZIP_NOT_PRESERVED"
        elif is_zip_like and r["zip"]["all_members_present_as_files"]:
            r["status"] = "ZIP_DUPLICATE_OF_EXTRACTED_NOT_COPIED"
        else:
            r["status"] = "NEW"
            out = DST / r["rel"]
            if not a.dry_run:
                out.parent.mkdir(parents=True, exist_ok=True)
                if out.exists():
                    if sha256(out) != r["sha256"]:
                        raise SystemExit(f"preserved copy differs: {out}")
                else:
                    shutil.copy2(p, out)
                    if sha256(out) != r["sha256"]:
                        raise SystemExit(f"copy verification failed: {out}")
            r["preserved_at"] = str(out.relative_to(ROOT))
            if p.suffix.lower() in (".mp4", ".mov"):
                r["video"] = probe_video(p)
    report = {"reviewed_kst": datetime.now(KST).isoformat(timespec="seconds"), "operator": "Claude 2",
              "hardware_access": False, "source": str(SRC.relative_to(ROOT)), "preserve_root": str(DST.relative_to(ROOT)),
              "previous_inventories": [str(x.relative_to(ROOT)) for x in PREV if x.is_file()],
              "dry_run": a.dry_run, "files": rows,
              "counts_by_suffix": dict(Counter(Path(r["source"]).suffix.lower() or "(none)" for r in rows)),
              "counts_by_status": dict(Counter(r["status"] for r in rows))}
    if not a.dry_run:
        OUT.parent.mkdir(parents=True, exist_ok=True)
        OUT.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "files"}, ensure_ascii=False, indent=2))
    for r in rows:
        if r["status"] != "SAME_AS_PREVIOUS":
            print(f"{r['status']:40s} {r['size']:>12d} {r['rel']}")
            if "video" in r:
                print("    video:", json.dumps(r["video"]))
            if "zip" in r:
                print("    zip:", json.dumps(r["zip"]))
    bad = [r for r in rows if r.get("video") and (not r["video"].get("opened") or r["video"].get("decode_failed"))]
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
