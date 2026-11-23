#!/usr/bin/env python3
"""Printable ChArUco board for camera/hand-eye calibration (OpenCV). A4 portrait at 300 dpi, with a 100 mm scale bar.
Print at 100 % (no fit-to-page), glue flat on a rigid board (foam board, glass or MDF). Use a matt print and matt board (no glare).

    piper-calibration-board [--squares-x 6] [--squares-y 8] [--square-mm 30] [--out DIR]

The board geometry is written next to the print as JSON; the calibration scripts read it from there. If the printed square does not
measure what the sheet says (check the scale bar and one square with a ruler), pass the measured value to the calibration, not the nominal one.

Moved from graspwise tools/calibration (ported from piper_pick scripts/make_charuco.py @ demo-reference cfc95d3) into Studio on 2026-11-23.
"""
import argparse
import json
import os
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw

OUT = Path(os.environ.get("PIPER_DATA_DIR") or Path.home() / "data") / "calibration" / "boards"
DICTIONARY = "DICT_5X5_100"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--squares-x", type=int, default=6)
    ap.add_argument("--squares-y", type=int, default=8)
    ap.add_argument("--square-mm", type=float, default=30.0)
    ap.add_argument("--marker-ratio", type=float, default=0.75, help="marker edge / square edge")
    ap.add_argument("--out", type=Path, default=OUT, help="where the sheet and its JSON are written")
    args = ap.parse_args(argv)
    out = args.out
    dpi = 300
    mm = dpi / 25.4
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, DICTIONARY))
    board = cv2.aruco.CharucoBoard((args.squares_x, args.squares_y), args.square_mm / 1000.0, args.square_mm * args.marker_ratio / 1000.0, dictionary)
    width_mm, height_mm = args.squares_x * args.square_mm, args.squares_y * args.square_mm
    if width_mm > 190 or height_mm > 277:
        raise SystemExit(f"board {width_mm:.0f} x {height_mm:.0f} mm does not fit an A4 page with margins")
    px_w, px_h = int(round(width_mm * mm)), int(round(height_mm * mm))
    image = board.generateImage((px_w, px_h), marginSize=0, borderBits=1)
    page = Image.new("L", (int(210 * mm), int(297 * mm)), 255)
    draw = ImageDraw.Draw(page)
    x0, y0 = int((210 - width_mm) / 2 * mm), int(30 * mm)
    page.paste(Image.fromarray(image), (x0, y0))
    draw.rectangle((x0 - 2, y0 - 2, x0 + px_w + 2, y0 + px_h + 2), outline=0, width=1)
    draw.text((x0, y0 + px_h + int(6 * mm)), f"ChArUco {args.squares_x}x{args.squares_y}, square {args.square_mm:.1f} mm, marker {args.square_mm * args.marker_ratio:.1f} mm, {DICTIONARY}\n"
              "Print at 100 %. Measure the scale bar and one square. Glue flat and matt.", fill=0)
    y = int(260 * mm)
    draw.line((int(55 * mm), y, int(155 * mm), y), fill=0, width=4)
    draw.text((int(55 * mm), y + 12), "scale bar: must measure 100 mm, else the print was scaled", fill=0)
    stem = f"charuco_{args.squares_x}x{args.squares_y}_{args.square_mm:.0f}mm"
    out.mkdir(parents=True, exist_ok=True)
    page.save(out / f"{stem}.pdf", resolution=dpi)
    page.save(out / f"{stem}.png")
    spec = {"squares_x": args.squares_x, "squares_y": args.squares_y, "square_m": args.square_mm / 1000.0,
            "marker_m": args.square_mm * args.marker_ratio / 1000.0, "dictionary": DICTIONARY, "opencv": cv2.__version__}
    (out / f"{stem}.json").write_text(json.dumps(spec, indent=1))
    # self-test: the rendered page must be detected completely by the same library the calibration will use
    detector = cv2.aruco.CharucoDetector(board)
    corners, ids, _, _ = detector.detectBoard(np.array(page))
    print(f"self-test: {0 if ids is None else len(ids)} of {(args.squares_x - 1) * (args.squares_y - 1)} inner corners found")
    print("wrote", out / f"{stem}.pdf")


if __name__ == "__main__":
    main()
