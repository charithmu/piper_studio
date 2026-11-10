"""One program on the CAN bus at a time. `RealArm.connect` and the SDK tools take an exclusive lock per bus; a second one fails at once with the holder's pid instead of both transmitting.

    lock = BusLock("can0"); lock.acquire()      # raises BusBusyError if another process holds it
    lock.release()                              # also released when the process ends, however it ends

The lock is an advisory flock on $XDG_RUNTIME_DIR/piper-arm-<bus>.lock (/tmp if unset). It covers our programs (RealArm, piper-arm, the tools/real_arm SDK tools); not the vendor ROS driver node, not `candump`
(listening is harmless), and not the emergency stop `arm_stop.py`, which deliberately skips it so it works while another program drives the arm.
"""

from __future__ import annotations

import fcntl
import os
import sys


class BusBusyError(RuntimeError):
    pass


class BusLock:
    def __init__(self, canbus: str):
        self.canbus = canbus
        self.path = os.path.join(os.environ.get("XDG_RUNTIME_DIR") or "/tmp", f"piper-arm-{canbus}.lock")
        self.fd: int | None = None

    def acquire(self) -> "BusLock":
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            try:
                holder = os.pread(fd, 200, 0).decode(errors="replace").strip() or "another process"
            except OSError:
                holder = "another process"
            os.close(fd)
            raise BusBusyError(f"{self.canbus} is in use by {holder}: only one program may drive the arm at a time (close it first)") from None
        try:
            os.ftruncate(fd, 0)
            os.pwrite(fd, f"pid {os.getpid()} ({os.path.basename(sys.argv[0]) or 'python'})".encode(), 0)
        except BaseException:                           # a failure here must not leave the lock held
            fcntl.flock(fd, fcntl.LOCK_UN)
            os.close(fd)
            raise
        self.fd = fd
        return self

    def release(self) -> None:
        if self.fd is not None:
            try:
                fcntl.flock(self.fd, fcntl.LOCK_UN)
            finally:
                os.close(self.fd)
                self.fd = None
