#!/usr/bin/env python3
"""Generate and validate urdf/piper_arm.xacro, the Piper arm (and gripper) as a mountable, prefixed xacro macro.

    python piper_arm_macro.py generate     # rewrite urdf/piper_arm.xacro from the official AgileX model
    python piper_arm_macro.py report       # read-only: where the firmware values (config/firmware_values.yaml) differ from the vendor URDF
    python piper_arm_macro.py check        # read-only: is the committed macro still the official model? (exit 1 if not)

The macro file is committed and is NOT generated at build or launch time. `check` runs in the test suite, so bumping the
`external/agx_arm_ros` submodule fails a test until `generate` is run and the diff is reviewed. See docs/ARCHITECTURE.md (D12).

What the generator does, on the parsed XML (never by text search): every link and joint name, and every reference to one
(`parent`, `child`, `mimic`), gets `${prefix}` in front; the official fixed `world -> base_link` joint becomes a mount joint
from the macro's `parent` link at `xyz`/`rpy`; the gripper part is conditional on `gripper`; mesh paths are left alone and still
point into the official `agx_arm_description` package.
"""
import copy
import datetime
import hashlib
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PKG = Path(__file__).resolve().parents[1]
WS = PKG.parents[1]
SRC = WS / "external/agx_arm_ros/src/agx_arm_description/agx_arm_urdf/piper/urdf"
ARM, GRIPPER = SRC / "piper_description.urdf", SRC / "piper_with_gripper_description.xacro"
OUT = PKG / "urdf/piper_arm.xacro"
SRDF_SRC = WS / "external/agx_arm_ros/src/agx_arm_moveit/config/agx_arm.srdf.xacro"
SRDF_OUT = WS / "src/piper_control/config/moveit/piper_official.srdf.xacro"
LINK_ATTRS = ("base_link", "tip_link", "parent_link", "link1", "link2")   # SRDF attributes that name a link
NAME_TAGS = ("link", "joint", "passive_joint")                           # SRDF elements whose `name` is a link or joint
FIRMWARE = PKG / "config/firmware_values.yaml"
GENERATOR_VERSION = 2
P = "${prefix}"


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git_rev(path: Path) -> str:
    try:
        return subprocess.check_output(["git", "-C", str(path), "rev-parse", "--short", "HEAD"], text=True).strip()
    except Exception:  # noqa: BLE001
        return "unknown"


def firmware_values() -> dict:
    """Overrides from config/firmware_values.yaml (values the arm's firmware reports; they win over the vendor URDF)."""
    import yaml

    return yaml.safe_load(FIRMWARE.read_text())


def apply_firmware(root: ET.Element, values: dict | None = None) -> ET.Element:
    """Apply the firmware overrides to an official (unprefixed) URDF tree, in place: arm joint velocity and effort (SDK range), gripper stroke (fingers are +-stroke/2)."""
    v = values or firmware_values()
    stroke = float(v["gripper_stroke"])
    for j in root.iter("joint"):
        lim, name = j.find("limit"), j.get("name")
        if lim is None:
            continue
        if re.fullmatch(r"joint[1-6]", name):
            lim.set("velocity", str(v["arm_joint_max_velocity"]))
            lim.set("effort", str(v["arm_joint_max_effort"][int(name[-1]) - 1]))
        elif name == "gripper":
            lim.set("upper", str(stroke))
        elif name == "gripper_joint1":
            lim.set("upper", str(stroke / 2))
        elif name == "gripper_joint2":
            lim.set("lower", str(-stroke / 2))
    return root


def scale_gripper_states(srdf: ET.Element, values: dict | None = None) -> ET.Element:
    """The official SRDF's gripper_open/half/close values assume a 0.1 m stroke; scale them to the firmware's stroke."""
    k = float((values or firmware_values())["gripper_stroke"]) / 0.1
    for st in srdf.iter("group_state"):
        if st.get("group") == "gripper":
            for jn in st.findall("joint"):
                jn.set("value", f"{float(jn.get('value')) * k:g}")
    return srdf


def prefixed(root: ET.Element) -> list[ET.Element]:
    """Official <link>/<joint> elements, deep-copied, with every name and reference prefixed. The world link and its joint are dropped."""
    out = []
    for el in root:
        if el.tag not in ("link", "joint"):
            continue
        if el.tag == "link" and el.get("name") == "world":
            continue
        if el.tag == "joint" and el.find("parent").get("link") == "world":
            continue
        el = copy.deepcopy(el)
        el.set("name", P + el.get("name"))
        for tag in ("parent", "child"):
            ref = el.find(tag)
            if ref is not None:
                ref.set("link", P + ref.get("link"))
        mimic = el.find("mimic")
        if mimic is not None:
            mimic.set("joint", P + mimic.get("joint"))
        out.append(el)
    return out


def build() -> str:
    arm = apply_firmware(ET.parse(ARM).getroot())
    grip = apply_firmware(ET.parse(GRIPPER).getroot())
    world_joint = next(j for j in arm.findall("joint") if j.find("parent").get("link") == "world")
    base = world_joint.find("child").get("link")

    macro = ET.Element("xacro:macro", {"name": "piper_arm", "params": "parent prefix:=piper_ xyz:='0 0 0' rpy:='0 0 0' gripper:=true"})
    mount = ET.SubElement(macro, "joint", {"name": P + "mount_joint", "type": "fixed"})
    ET.SubElement(mount, "parent", {"link": "${parent}"})
    ET.SubElement(mount, "child", {"link": P + base})
    ET.SubElement(mount, "origin", {"xyz": "${xyz}", "rpy": "${rpy}"})
    macro.extend(prefixed(arm))

    # The gripper file adds flange_link (+ joint) and the gripper; without a gripper keep an empty flange_link so the
    # tool/mount frames have the same parent either way (what piper.urdf.xacro always did).
    flange_names = {"flange_link", "flange_joint"}
    parts = [e for e in prefixed(grip) if e.get("name")[len(P):] in flange_names]
    rest = [e for e in prefixed(grip) if e.get("name")[len(P):] not in flange_names]
    with_gripper = ET.SubElement(macro, "xacro:if", {"value": "${gripper}"})
    with_gripper.extend(parts + rest)
    without = ET.SubElement(macro, "xacro:unless", {"value": "${gripper}"})
    ET.SubElement(without, "link", {"name": P + "flange_link"})
    fj = ET.SubElement(without, "joint", {"name": P + "flange_joint", "type": "fixed"})
    ET.SubElement(fj, "parent", {"link": P + "link6"})
    ET.SubElement(fj, "child", {"link": P + "flange_link"})
    ET.SubElement(fj, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})

    robot = ET.Element("robot", {"xmlns:xacro": "http://www.ros.org/wiki/xacro"})
    robot.append(macro)
    ET.indent(robot, space="  ")
    body = ET.tostring(robot, encoding="unicode")
    header = f"""<?xml version="1.0"?>
<!--
  GENERATED FILE: do not edit. Regenerate with `python src/piper_description/tools/piper_arm_macro.py generate`; validate with `... check`.

  Piper arm and AgileX gripper as the xacro macro `piper_arm`, derived from the official AgileX model (agx_arm_urdf), which has no
  prefix and a fixed world root. Names get `prefix` (default piper_), the world joint becomes a mount joint from `parent`.

  source:    agx_arm_ros @ {git_rev(WS / "external/agx_arm_ros")}, agx_arm_urdf @ {git_rev(SRC)}
  generated: {datetime.date.today().isoformat()} by tools/piper_arm_macro.py (generator version {GENERATOR_VERSION})
  sha256 piper_description.urdf:            {sha(ARM)}
  sha256 piper_with_gripper_description.xacro: {sha(GRIPPER)}

  firmware overrides (config/firmware_values.yaml): joint velocity and gripper stroke follow the arm's firmware, not the vendor URDF.

  Macro parameters: parent (link to mount on), prefix (default piper_; "" gives the official names), xyz, rpy (mount pose in
  `parent`), gripper (true|false). Meshes stay in the official agx_arm_description package.
-->
"""
    return header + body.replace("<robot ", "<robot ", 1) + "\n"


def prefix_srdf(el: ET.Element, prefix: str) -> ET.Element:
    el = copy.deepcopy(el)
    for e in el.iter():
        for a in LINK_ATTRS:
            if a in e.attrib:
                e.set(a, prefix + e.get(a))
        if e.tag in NAME_TAGS and "name" in e.attrib:
            e.set("name", prefix + e.get("name"))
    return el


def official_srdf(gripper: bool, firmware: bool = False) -> ET.Element:
    root = _official_srdf(gripper)
    return scale_gripper_states(root) if firmware else root


def _official_srdf(gripper: bool) -> ET.Element:
    return expand(f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="x"><xacro:include filename="{SRDF_SRC}"/></robot>',
                  mappings={"effector_type": "agx_gripper" if gripper else "none"})


def build_srdf() -> str:
    macro = ET.Element("xacro:macro", {"name": "piper_official_srdf", "params": "prefix:=piper_ gripper:=true"})
    for gripper, tag in ((True, "xacro:if"), (False, "xacro:unless")):
        block = ET.SubElement(macro, tag, {"value": "${gripper}"})
        block.extend(list(prefix_srdf(official_srdf(gripper, firmware=True), P)))
    robot = ET.Element("robot", {"xmlns:xacro": "http://www.ros.org/wiki/xacro"})
    robot.append(macro)
    ET.indent(robot, space="  ")
    header = f"""<?xml version="1.0"?>
<!--
  GENERATED FILE: do not edit. Regenerate with `python src/piper_description/tools/piper_arm_macro.py generate`; validate with `... check`.

  The official agx_arm_moveit SRDF (groups, passive joints, end effector, official named poses, collision matrix) for the Piper, with
  every link and joint name prefixed by `prefix` (default piper_). Group and end-effector names are unchanged.
  Included by piper.srdf.xacro, which adds Piper Studio's named poses.

  source:    agx_arm_ros @ {git_rev(WS / "external/agx_arm_ros")}
  generated: {datetime.date.today().isoformat()} by tools/piper_arm_macro.py (generator version {GENERATOR_VERSION})
  sha256 agx_arm.srdf.xacro: {sha(SRDF_SRC)}
-->
"""
    return header + ET.tostring(robot, encoding="unicode") + "\n"


def header_hashes(text: str) -> dict[str, str]:
    found = {}
    for line in text.splitlines():
        for key in ("piper_description.urdf", "piper_with_gripper_description.xacro"):
            if line.strip().startswith(f"sha256 {key}"):
                found[key] = line.split()[-1]
    return found


def canonical(urdf: ET.Element, prefix: str) -> dict:
    """Comparable summary of a URDF: links (inertial, meshes) and joints (type, topology, pose, axis, limit, mimic), prefix removed."""
    strip = lambda n: n[len(prefix):] if n.startswith(prefix) else n  # noqa: E731
    dump = lambda e: None if e is None else (e.tag, tuple(sorted(e.attrib.items())), tuple(dump(c) for c in e))  # noqa: E731
    out = {}
    for link in urdf.findall("link"):
        link = copy.deepcopy(link)
        link.set("name", strip(link.get("name")))
        out["link:" + link.get("name")] = dump(link)
    for j in urdf.findall("joint"):
        if j.get("name").endswith("mount_joint") or j.find("parent").get("link") == "world":
            continue
        j = copy.deepcopy(j)
        j.set("name", strip(j.get("name")))
        for tag in ("parent", "child"):
            j.find(tag).set("link", strip(j.find(tag).get("link")))
        m = j.find("mimic")
        if m is not None:
            m.set("joint", strip(m.get("joint")))
        out["joint:" + j.get("name")] = dump(j)
    return out


def expand(xacro_text: str, mappings: dict | None = None) -> ET.Element:
    import tempfile

    import xacro

    with tempfile.NamedTemporaryFile("w", suffix=".xacro", delete=False) as f:
        f.write(xacro_text)
    try:
        return ET.fromstring(xacro.process_file(f.name, mappings=mappings or {}).toxml())
    finally:
        Path(f.name).unlink()


def official(gripper: bool, firmware: bool = True) -> ET.Element:
    root = _official(gripper)
    return apply_firmware(root) if firmware else root


def _official(gripper: bool) -> ET.Element:
    if gripper:
        return expand(f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="x"><xacro:include filename="{GRIPPER}"/></robot>')
    root = expand(f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="x"><xacro:include filename="{ARM}"/></robot>')
    ET.SubElement(root, "link", {"name": "flange_link"})  # the mirror that piper.urdf.xacro always added
    fj = ET.SubElement(root, "joint", {"name": "flange_joint", "type": "fixed"})
    ET.SubElement(fj, "parent", {"link": "link6"})
    ET.SubElement(fj, "child", {"link": "flange_link"})
    ET.SubElement(fj, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    return root


def macro_expansion(prefix: str, gripper: bool) -> ET.Element:
    return expand(f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="x"><link name="world"/>'
                  f'<xacro:include filename="{OUT}"/><xacro:piper_arm parent="world" prefix="{prefix}" gripper="{str(gripper).lower()}"/></robot>')


def check() -> list[str]:
    problems = []
    text = OUT.read_text()
    for key, src in (("piper_description.urdf", ARM), ("piper_with_gripper_description.xacro", GRIPPER)):
        if header_hashes(text).get(key) != sha(src):
            problems.append(f"source changed since generation: {key} (submodule bumped? run `generate` and review the diff)")
    for gripper in (True, False):
        want = canonical(official(gripper), "")
        for prefix in ("piper_", ""):
            got = canonical(macro_expansion(prefix, gripper), prefix)
            for k in sorted(set(want) | set(got)):
                if want.get(k) != got.get(k):
                    problems.append(f"gripper={gripper} prefix={prefix!r}: {k} differs from the official model")
    srdf_text = SRDF_OUT.read_text()
    if re.search(r"sha256 agx_arm.srdf.xacro:\s+(\w+)", srdf_text).group(1) != sha(SRDF_SRC):
        problems.append("source changed since generation: agx_arm.srdf.xacro (run `generate` and review the diff)")
    for gripper in (True, False):
        want = [ET.tostring(e) for e in official_srdf(gripper, firmware=True)]
        for prefix in ("piper_", ""):
            root = expand('<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="x">'
                          f'<xacro:include filename="{SRDF_OUT}"/><xacro:piper_official_srdf prefix="{prefix}" gripper="{str(gripper).lower()}"/></robot>')
            unprefixed = copy.deepcopy(root)
            for e in unprefixed.iter():
                for attr in LINK_ATTRS:
                    if attr in e.attrib:
                        e.set(attr, e.get(attr).removeprefix(prefix))
                if e.tag in NAME_TAGS and "name" in e.attrib:
                    e.set("name", e.get("name").removeprefix(prefix))
            if [ET.tostring(e) for e in unprefixed] != want:
                problems.append(f"SRDF gripper={gripper} prefix={prefix!r}: differs from the official SRDF after removing the prefix")
    return problems


def report() -> list[str]:
    """Every place where the firmware values differ from the vendor URDF (for the user to review)."""
    out = []
    for gripper in (True,):
        a, b = canonical(official(gripper, firmware=False), ""), canonical(official(gripper, firmware=True), "")
        for k in sorted(a):
            if a[k] != b[k]:
                out.append(k)
    return out


def main() -> int:
    cmd = sys.argv[1] if len(sys.argv) > 1 else ""
    if cmd == "report":
        for k in report():
            print("firmware differs from vendor URDF:", k)
        v = firmware_values()
        print(f"overrides: arm joint velocity {v['arm_joint_max_velocity']} rad/s, effort {v['arm_joint_max_effort']} N.m (SDK range), gripper stroke {v['gripper_stroke']} m")
        return 0
    if cmd == "generate":
        OUT.write_text(build())
        SRDF_OUT.write_text(build_srdf())
        print(f"wrote {OUT}\nwrote {SRDF_OUT}")
        return 0
    if cmd == "check":
        problems = check()
        for p in problems:
            print("FAIL:", p)
        print("OK: piper_arm.xacro and piper_official.srdf.xacro match the official files" if not problems else f"{len(problems)} problem(s)")
        return 1 if problems else 0
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main())
