"""Interactive Isaac session: python -m piper_isaac --gui --scene table."""

import argparse
import code

from . import PiperIsaacSim


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gui", action="store_true", help="open the full Isaac Sim editor")
    parser.add_argument("--scene", choices=["empty", "table"], default="table")
    parser.add_argument("--render", action="store_true", help="enable video recording in headless mode")
    args = parser.parse_args()
    with PiperIsaacSim(scene=args.scene, gui=args.gui, render=args.render) as sim:
        code.interact(banner="Isaac session ready as `sim`. Try sim.state(), sim.move_to(...), sim.record_start(path). Ctrl-D exits.",
                      local={"sim": sim})


if __name__ == "__main__":
    main()
