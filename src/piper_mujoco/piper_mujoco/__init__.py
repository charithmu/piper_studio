"""The Piper arm in MuJoCo without ROS; see README.md."""

from piper_mujoco.sim import PiperSim

try:
    from importlib.metadata import version as _version
    __version__ = _version("piper_mujoco")
except Exception:
    __version__ = "0.0.0+source"

__all__ = ["PiperSim"]
