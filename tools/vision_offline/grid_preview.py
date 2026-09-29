#!/usr/bin/env python3
"""Downscaled preview with an ORIGINAL-pixel grid, for human/AI visual review.

usage: grid_preview.py --out OUT.png [--scale 0.5] [--grid 100] IMAGE
       grid_preview.py --contact OUT.png --cols 5 --thumb-w 288 IMAGE...
Grid labels are original pixel coordinates; nothing is written back to sources.
"""
import argparse, sys
from pathlib import Path
import cv2, numpy as np
cv2.setNumThreads(1)


def grid(img, scale, step):
    h, w = img.shape[:2]
    small = cv2.resize(img, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
    for x in range(0, w, step):
        xs = int(x * scale)
        cv2.line(small, (xs, 0), (xs, small.shape[0]), (0, 255, 255) if x % (step * 5) == 0 else (0, 160, 160), 1)
        cv2.putText(small, str(x), (xs + 2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
    for y in range(0, h, step):
        ys = int(y * scale)
        cv2.line(small, (0, ys), (small.shape[1], ys), (0, 255, 255) if y % (step * 5) == 0 else (0, 160, 160), 1)
        cv2.putText(small, str(y), (2, ys - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)
    return small


def contact(paths, cols, tw):
    tiles = []
    for p in paths:
        im = cv2.imread(str(p))
        if im is None:
            im = np.zeros((tw * 3 // 4, tw, 3), np.uint8)
        h, w = im.shape[:2]
        t = cv2.resize(im, (tw, int(h * tw / w)), interpolation=cv2.INTER_AREA)
        cv2.putText(t, Path(p).name[:40], (3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 3)
        cv2.putText(t, Path(p).name[:40], (3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        tiles.append(t)
    th = max(t.shape[0] for t in tiles)
    tiles = [cv2.copyMakeBorder(t, 0, th - t.shape[0], 0, 0, cv2.BORDER_CONSTANT) for t in tiles]
    rows = [np.hstack(tiles[i:i + cols] + [np.zeros((th, tw, 3), np.uint8)] * (cols - len(tiles[i:i + cols]))) for i in range(0, len(tiles), cols)]
    return np.vstack(rows)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("images", nargs="+")
    ap.add_argument("--out"); ap.add_argument("--contact")
    ap.add_argument("--scale", type=float, default=0.5); ap.add_argument("--grid", type=int, default=100)
    ap.add_argument("--cols", type=int, default=5); ap.add_argument("--thumb-w", type=int, default=288)
    a = ap.parse_args(argv)
    if a.contact:
        cv2.imwrite(a.contact, contact(a.images, a.cols, a.thumb_w)); print("wrote", a.contact); return 0
    img = cv2.imread(a.images[0])
    if img is None:
        print("unreadable", a.images[0], file=sys.stderr); return 2
    cv2.imwrite(a.out, grid(img, a.scale, a.grid)); print("wrote", a.out); return 0


if __name__ == "__main__":
    sys.exit(main())
