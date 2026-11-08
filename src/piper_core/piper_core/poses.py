"""Named poses (zero, rest, ready, ...) from an expanded SRDF."""

import xml.etree.ElementTree as ET


def named_poses_from_srdf(srdf: str, group: str = "arm") -> dict[str, dict[str, float]]:
    """{pose name: {joint name: value}} for the group's <group_state> entries. `srdf` is the expanded SRDF text (not the xacro)."""
    root = ET.fromstring(srdf)
    return {s.get("name"): {j.get("name"): float(j.get("value")) for j in s.findall("joint")}
            for s in root.findall("group_state") if s.get("group") == group}
