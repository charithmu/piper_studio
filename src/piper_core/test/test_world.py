"""piper_core.World: the neutral description of what is around the robot (the seam to downstream projects)."""

import pytest

from piper_core.world import Body, FixedCamera, World


def test_json_round_trip_keeps_everything():
    w = World([Body("t", "box", (1, 1, 0.04), pos=(0.5, 0, -0.02)), Body("m", "mesh", (1.0,), mesh="/x/a.obj", texture="/x/a.png", static=False, mass=0.3, friction=(0.7, 0.005, 0.0001)),
               Body("c", "cylinder", (0.03, 0.1)), Body("s", "sphere", (0.02,))], ground_z=-0.5, cameras=[FixedCamera("top", (0, 0, 1), (1, 0, 0, 0, 1, 0), 60.0, (320, 240))])
    assert World.from_json(w.to_json()) == w


@pytest.mark.parametrize("kw,msg", [
    (dict(name="a", shape="cone", size=(1,)), "shape"), (dict(name="a", shape="box", size=(1, 2)), "size"), (dict(name="a", shape="mesh"), "mesh"),
    (dict(name="a", shape="box", size=(1, 1, 1), mesh="x.obj"), "belong"), (dict(name="a", shape="box", size=(1, 1, 1), static=False, mass=0), "mass"),
    (dict(name="a", shape="box", size=(1, 1, 1), friction=(-1, 0, 0)), "friction"), (dict(name="a", shape="box", size=(1, 1, 1), pos=(0, 0)), "pos"),
])
def test_bad_bodies_are_refused(kw, msg):
    with pytest.raises(ValueError, match=msg):
        Body(**kw)


def test_names_are_unique_and_the_schema_is_checked():
    with pytest.raises(ValueError, match="unique"):
        World([Body("a", "sphere", (1,)), Body("a", "sphere", (1,))])
    with pytest.raises(ValueError, match="piper.world/1"):
        World.from_json('{"schema": "other"}')
