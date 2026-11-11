#!/usr/bin/env python3
"""Regenerate the cuRobo Piper models (robot only, and with the ZED X Mini on its bracket) from the description, and write them to src/piper_description/config/curobo/.

    scripts/curobo_env.sh run tools/curobo/build_model.py [--cameras none zedxm] [--no-validate]        # in the optional cuRobo environment, on the GPU (through gpu-run)

Steps per camera: cuRobo's builder (structure only) -> fit_spheres.py (spheres, policy.yaml) -> finalize.py (limits, zero pose, ignore list, provenance + fingerprint)
-> sphere_quality.py (mesh coverage) -> validate.py (audit, zero-pose overlap, 100 planned tool poses against every limit). Inputs are the shipped URDF and SRDF of the data dir
(`python tools/release/build_assets.py` first, in the ROS environment, after any description change). Afterwards run build_assets.py again so the wheel carries the new models, and
`python -m pytest src/piper_assets` (the staleness and consistency tests). Intermediate files: ~/data/ml/isaac/piper_studio/curobo/<time>/. Nothing here moves a robot.
"""
import argparse
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
WS = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(WS / "src/piper_assets"))
import piper_assets as pa  # noqa: E402

import finalize  # noqa: E402
import fit_spheres  # noqa: E402
import sphere_quality  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cameras", nargs="+", default=["none", "zedxm"], choices=["none", "zedxm"])
    ap.add_argument("--out", type=Path, default=WS / "src/piper_description/config/curobo")
    ap.add_argument("--work", type=Path, default=Path.home() / "data/ml/isaac/piper_studio/curobo" / time.strftime("%Y%m%d_%H%M%S"))
    ap.add_argument("--no-validate", action="store_true", help="skip the planning validation (it needs the GPU and takes about a minute)")
    a = ap.parse_args()
    a.work.mkdir(parents=True, exist_ok=True)
    a.out.mkdir(parents=True, exist_ok=True)
    data = pa.data_dir()
    policy = HERE / "policy.yaml"
    for cam in a.cameras:
        urdf, srdf = data / f"piper_{cam}_w0.07.urdf", data / f"piper_{cam}.srdf"
        print(f"\n===== {cam}: {urdf.name} =====", flush=True)
        builder, fit, final = a.work / f"piper_{cam}_builder.yml", a.work / f"piper_{cam}_fit.yml", a.work / f"piper_{cam}.yml"
        subprocess.run([sys.executable, "-m", "curobo.examples.getting_started.build_robot_model", "--urdf", str(urdf), "--asset-path", str(data),
                        "--tool-frames", "tcp_link", "--output", str(builder), "--seed", "0"], check=True, cwd=a.work)
        fit_spheres.main(str(urdf), str(builder), str(fit), policy)
        finalize.main(str(fit), str(urdf), str(srdf), str(final), policy)
        sphere_quality.main(str(urdf), str(final), str(a.work / f"quality_{cam}.json"))
        if not a.no_validate:
            import validate
            validate.main(str(final), cam, str(a.work / f"validation_{cam}.json"))
        for f in (final, Path(str(final) + ".provenance.json"), a.work / f"quality_{cam}.json", a.work / f"validation_{cam}.json"):
            if f.exists():
                shutil.copy2(f, a.out / f.name)
    print(f"\nwrote {a.out} (work files: {a.work})\nnext: python tools/release/build_assets.py && python -m pytest src/piper_assets -q")


if __name__ == "__main__":
    main()
