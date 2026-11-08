"""Joint limits, read from the URDF (the one place they are defined)."""

import xml.etree.ElementTree as ET


def limits_from_urdf(urdf: str) -> dict[str, tuple[float, float]]:
    """{joint name: (lower, upper)} for every revolute and prismatic joint with limits (radians, metres)."""
    root = ET.fromstring(urdf)
    return {
        j.get("name"): (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
        for j in root.findall("joint")
        if j.get("type") in ("revolute", "prismatic") and j.find("limit") is not None
    }
