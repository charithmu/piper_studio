"""Bring the USB-CAN adapter's interface up (1 Mbit/s). Finds it by USB, not by name (can0/can1 swap between boots on a box with the SoC's own CAN too).

    piper-can-up              find the USB-CAN adapter, bring it up at 1000000 bit/s (asks for the sudo password; does nothing if it is already up at that rate)
    piper-can-up --dry-run    print what it would run
    piper-can-up --down       take it down
Sends nothing to the arm. It never touches the SoC's own CAN controller.
"""
import argparse
import re
import subprocess
import sys

from piper_tools.arm_common import can_info, usb_canbus


def current_bitrate(name):
    out = subprocess.run(["ip", "-d", "link", "show", name], capture_output=True, text=True).stdout
    m = re.search(r"bitrate (\d+)", out)
    return int(m.group(1)) if m else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bitrate", type=int, default=1000000)
    ap.add_argument("--canbus", "-b", help="interface name to use instead of finding the USB adapter (it is checked to be a USB one)")
    ap.add_argument("--down", action="store_true")
    ap.add_argument("--dry-run", "-n", action="store_true")
    a = ap.parse_args()
    if a.canbus:
        info = can_info()
        if a.canbus not in info or not info[a.canbus]["usb"]:
            sys.exit(f"{a.canbus} is not a USB-CAN interface here: {({n: ('usb' if i['usb'] else 'soc') for n, i in info.items()})}")
        name = a.canbus
    else:
        name, why = usb_canbus()
        if name is None:
            sys.exit(why)
    state = can_info()[name]["state"]
    print(f"USB-CAN adapter = {name} ({state}, {can_info()[name]['path'].split('/usb')[-1] and 'usb' + can_info()[name]['path'].split('/usb')[-1]})")
    if a.down:
        cmds = [["sudo", "ip", "link", "set", name, "down"]]
    elif state == "up" and current_bitrate(name) == a.bitrate:
        print(f"already up at {a.bitrate} bit/s: nothing to do")
        return
    else:
        cmds = ([["sudo", "ip", "link", "set", name, "down"]] if state == "up" else []) + [["sudo", "ip", "link", "set", name, "up", "type", "can", "bitrate", str(a.bitrate)]]
    for c in cmds:
        print("+", " ".join(c))
        if not a.dry_run:
            subprocess.run(c, check=True)
    if not a.dry_run and not a.down:
        print(f"{name}: {can_info()[name]['state']} at {current_bitrate(name)} bit/s")


if __name__ == "__main__":
    main()
