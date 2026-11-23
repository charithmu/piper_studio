"""The command `piper-calibrate`: hand-eye calibration of the wrist ZED X Mini (camera through pyzed, arm through Studio's RealArm, solve with OpenCV).

    piper-calibrate --board BOARD.json --look                  camera only (no arm, no CAN): is the printed board seen cleanly from here?
    piper-calibrate --board BOARD.json --session NAME --dry    look for the board, plan and check the poses; the arm is only READ, nothing moves
    piper-calibrate --board BOARD.json --session NAME          the real run: THE ARM MOVES, you stand at it with the e-stop (`piper-estop`)
    piper-calibrate --offline DIR --board BOARD.json           re-solve saved captures; no arm, no camera

What happens (docs/CALIBRATION.md has the long version):
  1. the arm goes to the ready pose (Studio's start(), then one slow joint move; you approve each);
  2. the board must be visible from there; its position in the base frame is estimated with the description's camera pose, only to AIM the following poses;
  3. N poses are chosen where the board is predicted fully visible at 0.22-0.70 m, seen from different directions and rolls, with every
     fingertip/wrist/camera point above the board plane; the arm visits them slowly (0.25 rad/s peak), one frame each; a pose whose frame does not show
     the board cleanly is skipped;
  4. cv2.calibrateHandEye (five algorithms) + cv2.calibrateRobotWorldHandEye give T_link6_optical, with the board-position spread, a leave-out check and
     the agreement between methods as quality numbers; written to hand_eye.json together with the captures and the library versions.
Nothing is applied by this program: a person reads the verdict and changes Studio's camera pose in a reviewed commit.

Whatever else holds the camera or the CAN bus (on the ZED box the box server) must be stopped first.
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

from . import handeye
from . import kinematics as kin
from .planning import CLEARANCE_PATH_M, READY, CalibrationError, eased, path_clear, plan_poses
from .session import print_summary, save_capture, solve_session, write_result
from .zed import CameraError, ZedCamera

DATA = Path(os.environ.get("PIPER_DATA_DIR") or Path.home() / "data")


def open_zed(resolution):
    return ZedCamera(resolution=resolution)


def connect_real_arm(canbus):
    """Studio's RealArm on the USB-CAN adapter's interface (found by USB, not by name)."""
    from piper_core import RealArm
    from piper_core.canbus import resolve_canbus
    return RealArm.connect(resolve_canbus(canbus))


def ask(question, assume_yes=False):
    if assume_yes:
        return True
    return input(f"{question} [Enter = yes, anything else = stop] ").strip() == ""


def main(argv=None, *, open_camera=None, connect_arm=None):
    """The command. `open_camera(resolution) -> camera` and `connect_arm(canbus) -> RealArm` default to the ZED and Studio's RealArm; tests and other setups pass their own."""
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--board", required=True, help="the board JSON next to the printed sheet (`piper-calibration-board`)")
    ap.add_argument("--session", default="calib-" + time.strftime("%Y%m%d-%H%M%S"))
    ap.add_argument("--root", default=str(DATA / "calibration"), help="where sessions are written ($PIPER_DATA_DIR/calibration, else ~/data/calibration)")
    ap.add_argument("--offline", help="re-solve the captures in this directory; no arm, no camera")
    ap.add_argument("--dry", action="store_true", help="open the camera, look for the board, plan the poses and check the paths; the arm is not touched")
    ap.add_argument("--resolution", default="HD1200", help="ZED resolution: HD1200 (1920x1200, default: small boards need it beyond 0.4 m), HD1080 or SVGA (960x600)")
    ap.add_argument("--look", action="store_true", help="camera only: grab one frame, report whether the board is found, save the annotated image; the arm is not touched")
    ap.add_argument("--poses", type=int, default=14)
    ap.add_argument("--speed", type=int, default=10, help="Studio speed percent for start() and stop()")
    ap.add_argument("--ready", type=float, nargs=6, default=READY)
    ap.add_argument("--step", action="store_true", help="ask before every pose, not only before the sequence")
    ap.add_argument("--can", default=None)
    ap.add_argument("--camera-model", default="zedxm")
    args = ap.parse_args(argv)

    if args.offline:
        report = solve_session(args.offline, args.board, args.camera_model)
        print_summary(report)
        print("wrote", write_result(args.offline, report))
        return 0 if report["verdict"]["acceptable"] else 2

    board, spec = handeye.load_board(args.board)
    out = Path(args.root) / args.session
    out.mkdir(parents=True, exist_ok=True)
    try:
        camera = (open_camera or open_zed)(args.resolution)
    except CameraError as error:
        print(f"cannot open the camera: {error}\nAnother program probably holds it (on the ZED box: the box server, `sudo systemctl stop piper-box`); stop it, then run this again.")
        return 1
    detector = handeye.BoardDetector(board, camera.K)
    if args.look:
        try:
            header, arrays = camera.grab(settle_frames=8)
        finally:
            camera.close()
        found = detector.detect(arrays["rgb"])
        save_capture(out / "look", 0, arrays["rgb"], arrays["depth"], camera.K, np.zeros(6), np.zeros(6), found)
        if found is None:
            markers = detector.count_markers(arrays["rgb"])
            hint = ("no marker at all: board not in view, too far for this resolution, blurred or glare" if markers == 0 else
                    f"{markers} markers seen but not enough clean corners: board too far or blurred, or the printed layout differs from the JSON (squares, markers, dictionary)")
            print(f"board NOT found cleanly (need {handeye.MIN_CORNERS}+ corners, reprojection <= {handeye.MAX_REPROJECTION_PX} px): {hint}. Image: {out / 'look' / 'pose-00.jpg'}")
            return 2
        print(f"board found: {found['n_corners']} corners, reprojection {found['reprojection_px']:.2f} px, "
              f"distance {np.linalg.norm(found['T_cam_board'][:3, 3]):.2f} m from the camera. Image: {out / 'look' / 'pose-00.jpg'}")
        return 0
    ready = np.asarray(args.ready, float)
    arm = None
    try:
        from piper_core.backend import Trajectory
        try:
            arm = (connect_arm or connect_real_arm)(args.can)               # reads the firmware; no motion, no motors
        except Exception as error:
            print(f"cannot connect the arm: {error}\nAnother program probably holds the CAN bus (on the ZED box: the box server, `sudo systemctl stop piper-box`); stop it, then run this again.")
            return 1
        arm.speed_percent = args.speed
        current = None

        def go(goal, label):
            nonlocal current
            t, rows = eased(current, goal)
            res = arm.execute(Trajectory.from_pairs(list(zip(t, rows.tolist()))))
            if not res.ok:
                arm.hold()
                raise CalibrationError(f"{label}: {res.reason} {res.detail}")
            current = np.asarray(goal, float)

        if not args.dry:
            print("The arm will now go through Studio's start() (motors on, checked move to zero), then one slow move to the ready pose.")
            print("You stand at the arm, the e-stop is in reach (`piper-estop`), and the board lies in front of the arm in view of the camera.")
            if not ask("Start?"):
                return 1
            r = arm.start(args.speed)
            if not r.ok:
                raise CalibrationError(f"start refused: {r.message}")
            current = np.array([arm.zero()[j] for j in arm.zero()])
            if not ask(f"Move to the ready pose {ready.round(2).tolist()}?"):
                return 1
            go(ready, "ready")
            time.sleep(1.0)
        else:
            print("dry run: the arm is read where it stands (not moved); the board must be in view of the camera from here")
        q_ready = np.array(arm.state().q, float)
        header, arrays = camera.grab(settle_frames=8)
        first = detector.detect(arrays["rgb"])
        save_capture(out / "overview", 0, arrays["rgb"], arrays["depth"], camera.K, q_ready, ready, first)
        if first is None:
            raise CalibrationError("the board is not clearly visible from the ready pose: put it flat on the bench in front of the arm, "
                                   f"about 0.3-0.5 m from the base, in view of the camera (look at {out / 'overview' / 'pose-00.jpg'})")
        T_base_board = kin.camera_optical(q_ready, args.camera_model) @ first["T_cam_board"]
        print(f"board seen from the ready pose: {first['n_corners']} corners, reprojection {first['reprojection_px']:.2f} px; "
              f"estimated at {T_base_board[:3, 3].round(3).tolist()} m in the base frame (CAD camera pose, used only to aim)")
        poses, n_candidates = plan_poses(T_base_board, spec, camera.K, camera.size, args.poses, ready, camera=args.camera_model)
        previous, lowest = ready, 9.0
        for q in poses:
            ok, clearance = path_clear(previous, q, T_base_board[2, 3], args.camera_model)
            lowest = min(lowest, clearance)
            if not ok:
                raise CalibrationError(f"a path to a calibration pose passes {clearance:.3f} m above the board plane (needs {CLEARANCE_PATH_M} m)")
            previous = q
        print(f"{len(poses)} poses chosen out of {n_candidates} candidates; lowest checked point along the paths: {lowest:.3f} m above the board plane")
        (out / "plan.json").write_text(json.dumps({"ready": ready.tolist(), "poses": [p.tolist() for p in poses], "board_estimate": T_base_board.tolist(),
                                                   "spec": spec, "candidates": n_candidates}, indent=1))
        if args.dry:
            print("dry run: nothing moved. plan written to", out / "plan.json")
            return 0
        if not ask(f"Visit the {len(poses)} poses (slow, one frame each)? Ctrl-C holds the arm."):
            return 1
        got = 0
        for i, q in enumerate(poses):
            if args.step and not ask(f"Pose {i + 1}/{len(poses)}?"):
                break
            go(q, f"pose {i}")
            time.sleep(1.0)
            before = np.array(arm.state().q, float)
            header, arrays = camera.grab(settle_frames=8)
            after = np.array(arm.state().q, float)
            if np.abs(before - after).max() > 1e-3:
                print(f"pose {i}: the arm was still moving; skipped")
                continue
            detection = detector.detect(arrays["rgb"])
            save_capture(out, i, arrays["rgb"], arrays["depth"], camera.K, after, q, detection)
            if detection is None:
                print(f"pose {i}: board not clean in the image; kept on disk, not used")
                continue
            got += 1
            print(f"pose {i}: {detection['n_corners']} corners, reprojection {detection['reprojection_px']:.2f} px")
        go(ready, "back to ready")
        r = arm.stop(args.speed)
        print("session end (zero, park, e-stop, clear):", "ok" if r.ok else r.message)
    except KeyboardInterrupt:
        print("\ninterrupted: holding the arm")
        if arm is not None:
            arm.hold()
        return 130
    except CalibrationError as error:
        print("STOPPED:", error)
        if arm is not None:
            try:
                arm.hold()
                print("The arm is held where it stands (motors on, nothing moves by itself). End the session with `piper-arm` (stop) or `piper-estop`.")
            except Exception:
                pass
        return 1
    finally:
        camera.close()
        if arm is not None:
            arm.close()
    report = solve_session(out, args.board, args.camera_model)
    print_summary(report)
    print("wrote", write_result(out, report, {"camera_meta": camera.hello()}))
    return 0 if report["verdict"]["acceptable"] else 2


if __name__ == "__main__":
    sys.exit(main())
