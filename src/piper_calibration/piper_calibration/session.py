"""Captures on disk, the offline solve of a saved session, the result file. A session folder holds `pose-NN.npz/jpg` (+ `plan.json`, `hand_eye.json`)."""
import json
import sys
import time
from pathlib import Path

import numpy as np

from . import handeye
from . import kinematics as kin
from .planning import CalibrationError

def save_capture(directory, index, rgb, depth, K, q_meas, q_cmd, detection=None):
    directory.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(directory / f"pose-{index:02d}.npz", rgb=rgb, depth=depth.astype(np.float16), K=K, q=np.asarray(q_meas), q_cmd=np.asarray(q_cmd))
    import cv2
    preview = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if detection is not None:
        for p in detection["corners"]:
            cv2.circle(preview, (int(round(p[0])), int(round(p[1]))), 3, (0, 255, 0), -1)
    cv2.imwrite(str(directory / f"pose-{index:02d}.jpg"), preview, [cv2.IMWRITE_JPEG_QUALITY, 85])


def solve_session(directory, board_path, camera="zedxm", method=handeye.DEFAULT_METHOD):
    """Detect the board in every saved capture and solve. Offline: needs only the files and the wheels for the kinematics."""
    directory = Path(directory)
    board, spec = handeye.load_board(board_path)
    A, B, used, skipped = [], [], [], []
    for path in sorted(directory.glob("pose-*.npz")):
        data = np.load(path)
        detection = handeye.BoardDetector(board, data["K"]).detect(data["rgb"])
        if detection is None:
            skipped.append(path.name)
            continue
        A.append(kin.link6_pose(data["q"], camera))
        B.append(detection["T_cam_board"])
        used.append({"file": path.name, "q": np.asarray(data["q"]).round(5).tolist(), "n_corners": detection["n_corners"],
                     "reprojection_px": round(detection["reprojection_px"], 3)})
    if len(A) < 6:
        raise CalibrationError(f"{len(A)} usable captures in {directory} ({len(skipped)} without a clean board): need at least 6")
    report = handeye.solve(A, B, nominal=kin.nominal_link6_optical(camera), method=method)
    report.update(captures_used=used, captures_skipped=skipped, board=spec, camera=camera)
    return report


def write_result(directory, report, extra=None):
    import cv2
    versions = {"opencv": cv2.__version__, "numpy": np.__version__, "python": sys.version.split()[0]}
    for name in ("pyzed.sl", "piper_core", "piper_assets", "piper_calibration"):
        try:
            module = __import__(name, fromlist=["x"])
            versions[name] = getattr(module, "__version__", None) or (module.Camera.get_sdk_version() if name == "pyzed.sl" else "?")
        except Exception:
            pass
    result = {"id": "handeye-" + time.strftime("%Y%m%d-%H%M%S"), "created": time.strftime("%Y-%m-%d %H:%M:%S"), "versions": versions, **report, **(extra or {})}
    path = Path(directory) / "hand_eye.json"
    path.write_text(json.dumps(result, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)))
    return path


def print_summary(report):
    spread, d = report["board_in_base"], report["verdict"]
    print(f"\nhand-eye result from {report['n_captures']} captures (method {report['method']}):")
    print("  T_link6_optical translation [m]:", np.array(report["T_link6_optical"])[:3, 3].round(4).tolist())
    if "vs_nominal_deg_mm" in report:
        print(f"  change from the CAD value: {report['vs_nominal_deg_mm'][0]:.2f} deg, {report['vs_nominal_deg_mm'][1]:.1f} mm "
              f"(board position spread with the CAD value: {report['nominal_board_spread_rms_mm']:.1f} mm rms)")
    print(f"  board position spread over the captures: {spread['rms_mm']:.2f} mm rms, {spread['max_mm']:.2f} mm max; orientation {spread['rotation_rms_deg']:.2f} deg rms")
    if report["leave_out"]:
        print(f"  leave-out (held-out poses): {report['leave_out']['rms_mm']:.2f} mm rms")
    print("  methods vs chosen (deg, mm):", {k: [round(x, 2) for x in v["vs_chosen_deg_mm"]] for k, v in report["methods"].items()})
    print("  VERDICT:", "acceptable" if d["acceptable"] else "NOT ACCEPTABLE")
    for issue in d["issues"]:
        print("   -", issue)


