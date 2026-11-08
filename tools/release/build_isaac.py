#!/usr/bin/env python3
"""Build the data of the `piper_isaac` wheel from the repo's Isaac files and the EXISTING USD build (no ROS, no Isaac needed).

    python tools/release/build_isaac.py [--usd-dir ~/data/ml/isaac/piper_studio] [--out src/piper_isaac/piper_isaac]

Run `scripts/isaac.sh build-usd` first (it needs the ROS environment once, and Isaac) and `scripts/isaac.sh check` to verify the USD against the URDF.
Writes into the package (git-ignored, built into the wheel by scripts/build_wheels.sh):
  runner/        isaac/world.py, run_server.py, servo_gains.py (the Isaac-side files the client starts in Isaac's own environment)
  data/usd/      the no-camera Piper USD (piper_isaac/piper_isaac.usda with its payloads and textures), carrying the prefix it was built with (piper_)
  data/manifest.json  source commit, USD build time, hashes of every file
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

WS = Path(__file__).resolve().parents[2]
RUNNER = ("world.py", "run_server.py", "servo_gains.py")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--usd-dir", type=Path, default=Path(os.environ.get("PIPER_ISAAC_DATA", Path.home() / "data/ml/isaac/piper_studio")))
    ap.add_argument("--out", type=Path, default=WS / "src/piper_isaac/piper_isaac")
    a = ap.parse_args()
    usd = a.usd_dir.expanduser() / "usd" / "piper_isaac"
    if not (usd / "piper_isaac.usda").exists():
        sys.exit(f"no USD at {usd}: run scripts/isaac.sh build-usd first (needs the ROS environment once, and Isaac)")
    out = a.out.resolve()
    for d in ("runner", "data"):
        shutil.rmtree(out / d, ignore_errors=True)
    (out / "runner").mkdir(parents=True)
    for f in RUNNER:
        shutil.copy2(WS / "isaac" / f, out / "runner" / f)
    shutil.copytree(usd, out / "data" / "usd" / "piper_isaac")

    def run(*args):
        return subprocess.run(["git", "-C", str(WS), *args], capture_output=True, text=True).stdout.strip()
    files = {str(p.relative_to(out)): hashlib.sha256(p.read_bytes()).hexdigest() for d in ("runner", "data") for p in sorted((out / d).rglob("*")) if p.is_file()}
    built = (a.usd_dir.expanduser() / "usd" / "piper_isaac" / "piper_isaac.usda").stat().st_mtime
    (out / "data" / "manifest.json").write_text(json.dumps({"source": {"commit": run("rev-parse", "HEAD"), "describe": run("describe", "--tags", "--always", "--dirty")},
                                                  "usd": {"prefix": "piper_", "camera": False, "built_epoch": built, "from": str(usd)}, "files": files}, indent=1))
    size = sum(p.stat().st_size for p in out.rglob("*") if p.is_file() and "__pycache__" not in p.parts) / 1e6
    print(f"{out}: {len(files)} files, {size:.1f} MB")


if __name__ == "__main__":
    main()
