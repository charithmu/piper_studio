#!/usr/bin/env bash
# Remove ORPHANED Fast DDS shared-memory files (/dev/shm/fastrtps*, sem.fastrtps*) of this user: the ones no running process holds open. Crashed or killed ROS / Isaac processes leave
# them behind (1400 of them were found on 2026-11-11); a stale lock can make a new participant hang at start (an Isaac launch test timed out after 90 s). Files a live process holds are never touched.
#   scripts/clean_shm.sh          report only      scripts/clean_shm.sh --yes   remove the orphaned ones
exec python3 - "${1:-}" <<'PY'
import os, sys
held = set()
for pid in filter(str.isdigit, os.listdir("/proc")):
    for kind in ("fd", "map_files"):
        try:
            for f in os.listdir(f"/proc/{pid}/{kind}"):
                try: t = os.readlink(f"/proc/{pid}/{kind}/{f}")
                except OSError: continue
                if t.startswith("/dev/shm/"): held.add(t.split(" ")[0])
        except (PermissionError, FileNotFoundError): pass
    try:
        for line in open(f"/proc/{pid}/maps"):
            if "/dev/shm/" in line: held.add(line.split()[-1] if "(deleted)" not in line else line.split()[-2])
    except (PermissionError, FileNotFoundError): pass
mine = [f"/dev/shm/{n}" for n in os.listdir("/dev/shm") if n.startswith(("fastrtps", "sem.fastrtps")) and os.stat(f"/dev/shm/{n}").st_uid == os.getuid()]
stale = [f for f in mine if f not in held]
print(f"fastdds shm files of this user: {len(mine)}, held by a running process: {len(mine) - len(stale)}, orphaned: {len(stale)}")
if sys.argv[1] == "--yes":
    n = 0
    for f in stale:
        try: os.remove(f); n += 1
        except FileNotFoundError: pass
    print(f"removed {n}")
elif stale:
    print("run with --yes to remove the orphaned ones")
PY
