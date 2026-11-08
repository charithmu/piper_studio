"""Make `piper_isaac` importable from the source tree (tests run without installing it); piper_core and piper_assets still come from the environment."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
