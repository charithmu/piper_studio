"""The Piper arm in Isaac Sim without ROS; see README.md."""

from piper_isaac.sim import PiperIsaacSim

try:
    from importlib.metadata import version as _version
    __version__ = _version("piper_isaac")
except Exception:
    __version__ = "0.0.0+source"

__all__ = ["PiperIsaacSim"]
