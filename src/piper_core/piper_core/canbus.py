"""Which Linux CAN interface is the USB-CAN adapter? Sysfs only: no SDK, no ROS, sends nothing.

A machine with its own CAN controller as well (Jetson `mttcan`) has two buses whose names (can0, can1) can swap from boot to boot, so we choose by the
USB device the interface sits on, never by name. `$PIPER_CANBUS` names one explicitly; `$PIPER_CAN_USB_PORT` pins one adapter by its USB port path.
"""
import os
import sys
from pathlib import Path


def can_info():
    """SocketCAN buses on this machine: {name: {"state": "up"|"down"|..., "usb": bool, "path": sysfs device path or ""}} (network type 280 = CAN). Reads sysfs only.

    `usb` is True when the interface sits on a USB device (the USB-CAN adapter, driver gs_usb). A SoC's own CAN controller (Jetson mttcan) is not USB.
    The names (can0, can1) can swap from boot to boot when a machine has both, so tools choose by `usb`, never by name.
    """
    found = {}
    for d in sorted(Path("/sys/class/net").iterdir()):
        try:
            if (d / "type").read_text().strip() != "280":
                continue
            path = str((d / "device").resolve()) if (d / "device").exists() else ""
            found[d.name] = {"state": (d / "operstate").read_text().strip(), "usb": "/usb" in path, "path": path}
        except OSError:
            pass
    return found


def can_interfaces():
    """SocketCAN buses as [(name, operstate)] (kept for older callers; see can_info)."""
    return [(n, i["state"]) for n, i in can_info().items()]


def usb_canbus(usb_port=None):
    """The name of the CAN interface that is the USB-CAN adapter, or an error text. `usb_port` (or $PIPER_CAN_USB_PORT) pins one adapter by its USB port path
    (e.g. "1-2", the folder under /sys/bus/usb/devices) when several adapters are present. Returns (name, None) or (None, why)."""
    usb_port = usb_port or os.environ.get("PIPER_CAN_USB_PORT")
    info = can_info()
    usb = {n: i for n, i in info.items() if i["usb"]}
    if usb_port:
        usb = {n: i for n, i in usb.items() if f"/{usb_port}/" in i["path"] + "/" or f"/{usb_port}:" in i["path"]}
    if len(usb) == 1:
        return next(iter(usb)), None
    if not usb:
        return None, ("no USB-CAN adapter seen (CAN buses present: " + (", ".join(f"{n} on " + ("usb" if i["usb"] else "the SoC") for n, i in info.items()) or "none")
                      + "). Check the USB plug: lsusb shows 1d50:606f when present.")
    return None, "several USB-CAN adapters (" + ", ".join(f"{n} {i['path'].split('/usb')[-1]}" for n, i in usb.items()) + "): pin one with --canbus NAME or PIPER_CAN_USB_PORT=<usb port path>."


def resolve_canbus(requested=None):
    """Pick the CAN bus: the requested one (--canbus NAME, or $PIPER_CANBUS), else the USB-CAN adapter's interface (never the SoC's own controller, whatever it is
    called today). Exits with a clear message otherwise.

    The "CAN bus" is the name of the Linux CAN network interface the USB-CAN adapter creates (can0, can1, a renamed one...), not something the arm reports.
    """
    info = can_info()
    requested = requested or os.environ.get("PIPER_CANBUS")
    if requested:
        if requested not in info:
            sys.exit(f"CAN bus '{requested}' does not exist. Present: {sorted(info) or 'none'}. (Adapter plugged in? lsusb shows 1d50:606f.)")
        name = requested
    else:
        name, why = usb_canbus()
        if name is None:
            sys.exit(why)
    if info[name]["state"] != "up":
        sys.exit(f"{name} (the USB-CAN adapter) is {info[name]['state']}: bring it up first (sudo password):  piper-can-up")
    return name
