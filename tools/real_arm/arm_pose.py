#!/usr/bin/env python3
"""Shim: the code lives in src/piper_tools/piper_tools/arm_pose.py (installed as a command on a box, see src/piper_tools/README.md).
Running or importing this file works as before, from the repo with or without piper_tools installed."""
import pathlib
import sys

try:
    import piper_tools  # noqa: F401
except ImportError:     # not installed in this environment: use the checkout
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "src/piper_tools"))
import piper_tools.arm_pose as _impl

if __name__ == "__main__":
    _impl.main()
else:
    sys.modules[__name__] = _impl    # `import arm_pose` gives the real module
