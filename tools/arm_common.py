"""Shared helpers for the real-arm tools (tools/arm_*.py). Importing this sends nothing."""
import math
import sys
import time
from pathlib import Path

from pyAgxArm import AgxArmFactory, ArmModel, PiperFW, create_agx_arm_config, resolve_firmware_profile


def deg(x):
    return round(math.degrees(x), 3)


def fields(m):
    """Plain dict/list of an SDK message (robust to SDK version differences)."""
    msg = getattr(m, "msg", m)
    if isinstance(msg, (list, tuple)):
        return list(msg)
    if hasattr(msg, "__dict__"):
        return {k: (fields(v) if hasattr(v, "__dict__") and not isinstance(v, (int, float, str)) else v)
                for k, v in vars(msg).items() if not k.startswith("_")}
    return str(msg)


def wait_for(fn, secs=3.0):
    """Poll fn() until it returns something other than None (or time runs out)."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < secs:
        v = fn()
        if v is not None:
            return v
        time.sleep(0.05)
    return None


def can_interfaces():
    """SocketCAN buses (network interfaces) on this machine as [(name, operstate)] (network type 280 = CAN). Reads sysfs only."""
    found = []
    for d in sorted(Path("/sys/class/net").iterdir()):
        try:
            if (d / "type").read_text().strip() == "280":
                found.append((d.name, (d / "operstate").read_text().strip()))
        except OSError:
            pass
    return found


def resolve_canbus(requested=None):
    """Pick the CAN bus: the requested one, else the only one, else the only one that is up. Exits with a clear message otherwise.

    The "CAN bus" is the name of the Linux CAN network interface the USB-CAN adapter creates (can0, can1, a renamed one...), not something the arm reports.
    """
    ifs = dict(can_interfaces())
    if requested:
        if requested not in ifs:
            sys.exit(f"CAN bus '{requested}' does not exist. Present: {sorted(ifs) or 'none'}. (Adapter plugged in? lsusb shows 1d50:606f.)")
        name = requested
    elif not ifs:
        sys.exit("No CAN bus found: the USB-CAN adapter is not seen. Check the USB plug (lsusb shows 1d50:606f when present).")
    elif len(ifs) == 1:
        name = next(iter(ifs))
    else:
        up = [n for n, st in ifs.items() if st == "up"]
        if len(up) != 1:
            sys.exit(f"Several CAN buses ({', '.join(f'{n} {st}' for n, st in ifs.items())}): choose one with --canbus.")
        name = up[0]
    if ifs[name] != "up":
        sys.exit(f"{name} is {ifs[name]}: bring it up first (asks for your sudo password):  scripts/can_up.sh {name}")
    return name


# Note: pyAgxArm and python-can both call the CAN bus name `channel` in their own APIs; everywhere in our tools it is `canbus`.
def open_arm(canbus="can0", gripper=False):
    """Connect with the right SDK profile for the arm's firmware. Sends ONE frame (the firmware query).

    Opens with the oldest profile (the firmware query is identical on all of them), reads the firmware, closes, and reopens with the
    profile that matches the version, because status/MIT frames decode differently per profile.
    Returns (arm, gripper_driver_or_None, firmware_dict, profile). Caller must arm.disconnect().
    """
    probe = AgxArmFactory.create_arm(create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=PiperFW.DEFAULT, channel=canbus))
    probe.connect()
    try:
        wait_for(lambda: probe.get_fps() or None, 3.0)
        fw = probe.get_firmware(timeout=3.0)
    finally:
        probe.disconnect()
    if not fw:
        raise RuntimeError("no firmware answer (arm silent, another program on the bus, master mode or teach playback?)")
    profile = resolve_firmware_profile(ArmModel.PIPER, fw["software_version"])
    arm = AgxArmFactory.create_arm(create_agx_arm_config(robot=ArmModel.PIPER, firmeware_version=profile, channel=canbus))
    grip = arm.init_effector(arm.OPTIONS.EFFECTOR.AGX_GRIPPER) if gripper else None   # before connect(); sends nothing
    arm.connect()
    wait_for(lambda: arm.get_fps() or None, 3.0)
    return arm, grip, fw, profile


def angles(arm):
    ja = arm.get_joint_angles()
    return list(ja.msg) if ja is not None else None
