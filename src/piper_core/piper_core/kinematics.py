"""Forward kinematics of the Piper from its URDF string (numpy only, no ROS, no SDK): where every link, the TCP and the camera are for given joint angles.

    from piper_core.kinematics import Kinematics
    kin = Kinematics.from_assets("zedxm")             # the shipped description (piper_assets); or Kinematics(urdf_text)
    kin.pose(q, "tcp_link")                           # 4x4, in base_link
    kin.points(q)                                     # {"link1": xyz, ..., "link6": xyz, "tcp": xyz, "camera_optical": xyz (if the description has a camera)}

q is six radians (joint1..joint6) or a {joint name: radian} dict. Frames are expressed in `base_link` (the arm's base), metres. Joints other than the six
arm joints (the gripper, the camera mount) are fixed in the shipped description, so the result is exact for the gripper width and camera it was built with.
The same URDF is used by MoveIt and every simulator; `tools/real_arm/fk_compare.py` checks it against the firmware's own kinematics (0.09 mm).
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import numpy as np

from piper_core.urdf_tools import _rpy_matrix

ARM_JOINTS = tuple(f"joint{i}" for i in range(1, 7))
CAMERA_OPTICAL = "zed_left_camera_frame_optical"      # the left lens: z forward, x right, y down


def _axis_rotation(axis, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * k + (1 - np.cos(q)) * k @ k


class Kinematics:
    def __init__(self, urdf: str, prefix: str = "", base: str | None = None):
        root = ET.fromstring(urdf)
        self.prefix = prefix
        self._joints = {}                       # child link -> (parent link, joint name, type, xyz, R, axis)
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = np.array([float(v) for v in ((o.get("xyz") if o is not None else None) or "0 0 0").split()])
            rpy = [float(v) for v in ((o.get("rpy") if o is not None else None) or "0 0 0").split()]
            ax = j.find("axis")
            axis = [float(v) for v in ax.get("xyz").split()] if ax is not None else [0.0, 0.0, 1.0]
            self._joints[j.find("child").get("link")] = (j.find("parent").get("link"), j.get("name"), j.get("type"), xyz, _rpy_matrix(*rpy), axis)
        self.links = [l.get("name") for l in root.findall("link")]
        self.base = base or f"{prefix}base_link"
        if self.base not in self.links:
            raise ValueError(f"no link {self.base!r} in this description")
        self.joint_names = tuple(f"{prefix}{j}" for j in ARM_JOINTS)
        for j in self.joint_names:
            if not any(v[1] == j for v in self._joints.values()):
                raise ValueError(f"no joint {j!r} in this description (prefix={prefix!r}?)")

    @classmethod
    def from_assets(cls, camera="zedxm", gripper_width: float | None = 0.07) -> "Kinematics":
        """The description shipped in piper_assets (unprefixed names), with the camera (`"zedxm"`, default) or without (`None`)."""
        import piper_assets
        return cls(piper_assets.urdf_path(gripper_width=gripper_width, camera=camera).read_text())

    def _values(self, q):
        if isinstance(q, dict):
            return {(k if k in self.joint_names else self.prefix + k): float(v) for k, v in q.items()}
        q = list(q)
        if len(q) != 6:
            raise ValueError(f"q needs 6 joint angles, got {len(q)}")
        return dict(zip(self.joint_names, (float(v) for v in q)))

    def _chain(self, link):
        chain = []
        while link in self._joints:
            chain.append(link)
            link = self._joints[link][0]
        return link, chain[::-1]

    def pose(self, q, link: str) -> np.ndarray:
        """4x4 pose of `link` in the base frame."""
        if link not in self.links:
            raise KeyError(f"no link {link!r}")
        vals = self._values(q)
        T_root = {}
        for target in (link, self.base):
            top, chain = self._chain(target)
            T = np.eye(4)
            for child in chain:
                _, name, typ, xyz, R, axis = self._joints[child]
                step = np.eye(4)
                step[:3, :3] = R @ (_axis_rotation(axis, vals[name]) if typ in ("revolute", "continuous") else np.eye(3))
                step[:3, 3] = xyz
                T = T @ step
            T_root[target] = (top, T)
        if T_root[link][0] != T_root[self.base][0]:
            raise ValueError(f"{link!r} and {self.base!r} are not in one tree")
        return np.linalg.inv(T_root[self.base][1]) @ T_root[link][1]

    def link_origins(self, q, links=None) -> dict[str, np.ndarray]:
        """{link: xyz in the base frame} for `links` (default: link1..link6)."""
        names = links or [f"{self.prefix}link{i}" for i in range(1, 7)]
        return {n: self.pose(q, n)[:3, 3] for n in names}

    def points(self, q) -> dict[str, np.ndarray]:
        """Link origins of joints 1..6, "tcp" and (when the description has one) "camera_optical", as xyz in the base frame."""
        out = {f"link{i}": p for i, p in enumerate(self.link_origins(q).values(), 1)}
        if f"{self.prefix}tcp_link" in self.links:
            out["tcp"] = self.pose(q, f"{self.prefix}tcp_link")[:3, 3]
        if f"{self.prefix}{CAMERA_OPTICAL}" in self.links:
            out["camera_optical"] = self.pose(q, f"{self.prefix}{CAMERA_OPTICAL}")[:3, 3]
        return out
