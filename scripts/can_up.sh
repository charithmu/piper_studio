#!/usr/bin/env bash
# Bring the Piper's CAN bus (the USB-CAN adapter's network interface) up (SocketCAN, 1 Mbit/s). Runs `sudo ip link ...` itself: it will ask for your sudo password.
#
#   scripts/can_up.sh                  auto-detect the CAN bus, bring it up at 1000000 bit/s
#   scripts/can_up.sh can1             a specific CAN bus
#   scripts/can_up.sh can0 500000      a specific CAN bus and bit rate
#   scripts/can_up.sh --down [can0]    take it down
#   scripts/can_up.sh --dry-run ...    print what would run, change nothing
#
# Sends nothing to the arm. Safe to re-run: if the CAN bus is already up at the right bit rate it does nothing.
set -euo pipefail

dry=0; down=0; args=()
for a in "$@"; do
  case "$a" in
    --dry-run|-n) dry=1 ;;
    --down) down=1 ;;
    -h|--help) sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) args+=("$a") ;;
  esac
done
canbus="${args[0]:-}"
bitrate="${args[1]:-1000000}"

can_buses() { for d in /sys/class/net/*; do [ "$(cat "$d/type" 2>/dev/null)" = 280 ] && basename "$d"; done; }

if [ -z "$canbus" ]; then
  mapfile -t found < <(can_buses)
  case "${#found[@]}" in
    0) echo "No CAN bus found. Is the USB-CAN adapter plugged in?  lsusb | grep -i -E '1d50|candle|geschwister'"
       echo "If it is listed but there is no CAN bus: sudo modprobe gs_usb"; exit 1 ;;
    1) canbus="${found[0]}" ;;
    *) echo "Several CAN buses: ${found[*]}. Name one: scripts/can_up.sh <canbus>"; exit 1 ;;
  esac
fi
[ -e "/sys/class/net/$canbus" ] || { echo "CAN bus '$canbus' does not exist (present: $(can_buses | tr '\n' ' '))"; exit 1; }

run() { echo "+ $*"; [ "$dry" = 1 ] || "$@"; }

if [ "$down" = 1 ]; then
  run sudo ip link set "$canbus" down
else
  state="$(cat "/sys/class/net/$canbus/operstate")"
  cur="$(ip -details link show "$canbus" | sed -n 's/.*bitrate \([0-9]*\).*/\1/p' | head -1)"
  if [ "$state" = up ] && [ "$cur" = "$bitrate" ]; then
    echo "$canbus is already up at $bitrate bit/s: nothing to do."
  else
    [ "$state" = up ] && run sudo ip link set "$canbus" down    # the bit rate can only change while down
    run sudo ip link set "$canbus" up type can bitrate "$bitrate"
  fi
fi

[ "$dry" = 1 ] || ip -details -brief link show "$canbus"
