"""Produce the final Piper cuRobo YAML from: builder YAML (structure) + covering-fit spheres + the SRDF + explicit values.
Hand-tuned values: policy.yaml (section model). Limits, acceleration, jerk, the zero pose and the guard margin come from piper_assets' limits.json (one source), not from here.
Every decision is written to <out>.provenance.json, with the fingerprint of the URDF and SRDF the model was built for (piper_assets.urdf_fingerprint); a test fails when the description moves on.
    python -I tools/curobo/finalize.py <fitted.yml> <urdf> <srdf> <out.yml> [policy.yaml]
The SRDF is the one that ships with the URDF (piper_assets/data/piper_<camera>.srdf; with the camera it also disables the camera and bracket against link5, link6, flange_link, gripper_base).
"""
import json, re, sys
from pathlib import Path
import yaml

WS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WS / "src/piper_assets"))
import piper_assets as pa                                    # noqa: E402  (pure Python, no ROS: used from the source tree)
POLICY_FILE = Path(__file__).with_name("policy.yaml")


def srdf_pairs(srdf):
    out = set()
    for m in re.finditer(r'<disable_collisions\s+link1="([^"]*)"\s+link2="([^"]*)"', Path(srdf).read_text()):
        out.add(tuple(sorted(m.groups())))
    return out


def decisions(policy=POLICY_FILE, data=None):
    P = yaml.safe_load(Path(policy).read_text())["model"]; L = json.loads(((Path(data) if data else pa.data_dir()) / "limits.json").read_text())
    return {
        "base_link": "base_link",                                           # the builder wrote the URDF root 'world'
        "limits_source": "piper_assets limits.json: " + L["position"]["source"] + "; velocity " + str(L["velocity"]["value"]) + " rad/s (" + L["velocity"]["source"] + ")",
        "max_acceleration_rad_s2": L["acceleration"]["value"], "max_jerk_rad_s3": L["jerk"]["value"],     # jerk is DERIVED (flag in limits.json), no source
        "operating_scale": P["operating_scale"], "velocity_scale_note": P["velocity_scale_note"],
        "default_joint_position": [L["zero_pose"][j] for j in L["joints"]],  # the guard's zero pose, 0.01 rad inside the limits (exact zero is rejected by the guard)
        "position_limit_clip_rad": L["margin"],                              # cuRobo shrinks every position limit by this = the guard margin, so plans pass execute() unchanged
        "collision_sphere_buffer_m": P["collision_sphere_buffer_m"], "self_collision_buffer_m": P["self_collision_buffer_m"],
        "self_collision_ignore": "SRDF disable_collisions (Adjacent + Never) restricted to links with spheres; the builder's sampled matrix is NOT used",
    }


def main(fit, urdf, srdf, outp, policy=POLICY_FILE):
    D = decisions(policy, Path(urdf).parent)
    d = yaml.safe_load(open(fit)); top = d.get("robot_cfg", d); k = top["kinematics"]
    k["base_link"] = D["base_link"]; n = len(k["cspace"]["joint_names"])
    cs = k["cspace"]; cs["max_acceleration"] = [D["max_acceleration_rad_s2"]] * n; cs["max_jerk"] = [D["max_jerk_rad_s3"]] * n
    cs["velocity_scale"] = [D["operating_scale"] ** 0.5] * n
    for key in ("acceleration_scale", "jerk_scale"): cs[key] = [D["operating_scale"]] * n
    cs["default_joint_position"] = D["default_joint_position"]; cs["position_limit_clip"] = D["position_limit_clip_rad"]
    k["collision_sphere_buffer"] = D["collision_sphere_buffer_m"]
    sph = set(k["collision_spheres"]); k["self_collision_buffer"] = {l: D["self_collision_buffer_m"] for l in sph}
    ign = {}
    for a, b in srdf_pairs(srdf):
        if a in sph and b in sph: ign.setdefault(a, set()).add(b); ign.setdefault(b, set()).add(a)
    # structural entries cuRobo's builder always writes (virtual links without spheres)
    ign.setdefault("world", set()).add("base_link"); ign.setdefault("base_link", set()).add("world")
    ign.setdefault("tcp_link", set()).add("gripper_base"); ign.setdefault("gripper_base", set()).add("tcp_link")
    ign.setdefault("gripper_link", set()).add("gripper_base"); ign.setdefault("gripper_base", set()).add("gripper_link")
    k["self_collision_ignore"] = {a: sorted(v) for a, v in sorted(ign.items())}
    k["urdf_path"] = "resolved_from_installed_piper_assets_at_runtime"        # piper_assets.curobo_config() puts the shipped URDF here; a model file never carries a machine path
    top["load_dynamics"] = False                                              # inertias and efforts are provisional (docs/description/MASSES.md)
    yaml.safe_dump(d, open(outp, "w"), sort_keys=False)
    mf = json.loads((Path(urdf).parent / "manifest.json").read_text())["source"]
    prov = {"description_commit": mf["commit"][:7] + ("+dirty" if mf["dirty"] else ""), "urdf": Path(urdf).name, "srdf": Path(srdf).name,
            "fingerprint": pa.urdf_fingerprint(urdf, srdf), "decisions": D, "policy_file": "tools/curobo/policy.yaml",
            "sphere_counts": {l: len(v) for l, v in k["collision_spheres"].items()}, "ignore_pairs": {a: v for a, v in k["self_collision_ignore"].items()}}
    Path(str(outp) + ".provenance.json").write_text(json.dumps(prov, indent=2)); print("wrote", outp, "| spheres", sum(prov["sphere_counts"].values()), "| description", prov["description_commit"])


if __name__ == "__main__":
    main(*sys.argv[1:6])
