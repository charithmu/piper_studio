"""Static pages for the Piper Studio site: status report, control levels, architecture, camera gallery.

Called from build_demo_page.py. Content that changes with the project lives in the dicts below; everything else is layout.
"""
from __future__ import annotations

import html
from pathlib import Path

CSS = """
:root { --bg:#f7f7f5; --card:#fff; --ink:#1c1e21; --muted:#6b7280; --line:#e5e7eb; --ok:#15803d; --bad:#b91c1c; --warn:#b45309; --accent:#2563eb;
  --vendor:#dbeafe; --vendor-l:#1d4ed8; --upstream:#e5e7eb; --upstream-l:#4b5563; --ours:#dcfce7; --ours-l:#15803d; --plan:#fef3c7; --plan-l:#b45309; }
@media (prefers-color-scheme: dark) { :root { --bg:#0f1115; --card:#181b21; --ink:#e6e8eb; --muted:#9aa3af; --line:#2a2f38; --ok:#4ade80; --bad:#f87171; --warn:#fbbf24; --accent:#60a5fa;
  --vendor:#1e3a8a; --vendor-l:#93c5fd; --upstream:#374151; --upstream-l:#d1d5db; --ours:#14532d; --ours-l:#86efac; --plan:#78350f; --plan-l:#fcd34d; } }
* { box-sizing:border-box; }
body { margin:0; background:var(--bg); color:var(--ink); font:15px/1.55 system-ui,-apple-system,Segoe UI,Roboto,sans-serif; }
main { max-width:1080px; margin:0 auto; padding:16px 16px 64px; }
nav { display:flex; flex-wrap:wrap; gap:6px 18px; padding:10px 0 14px; border-bottom:1px solid var(--line); margin-bottom:18px; font-size:14px; }
nav a { color:var(--accent); text-decoration:none; } nav a.here { font-weight:700; color:var(--ink); }
h1 { font-size:28px; margin:8px 0 4px; } h2 { font-size:20px; margin:34px 0 10px; } h3 { font-size:16px; margin:18px 0 6px; }
.muted { color:var(--muted); font-size:13px; } .lead { font-size:17px; max-width:72ch; }
.card { background:var(--card); border:1px solid var(--line); border-radius:12px; padding:16px; overflow-x:auto; }
table { border-collapse:collapse; width:100%; font-size:14px; } th,td { padding:7px 10px; border-bottom:1px solid var(--line); text-align:left; vertical-align:top; }
th { font-weight:600; } td.c { text-align:center; white-space:nowrap; }
.ok { color:var(--ok); font-weight:600; } .bad { color:var(--bad); font-weight:600; } .warn { color:var(--warn); font-weight:600; }
code, pre { font:13px ui-monospace,SFMono-Regular,Menlo,monospace; } pre { background:var(--bg); border:1px solid var(--line); border-radius:8px; padding:12px; overflow-x:auto; }
ul { padding-left:20px; } li { margin:4px 0; }
.pill { display:inline-block; padding:2px 10px; border-radius:999px; background:var(--card); border:1px solid var(--line); font-size:13px; }
.grid { display:grid; grid-template-columns:repeat(auto-fit,minmax(300px,1fr)); gap:16px; align-items:start; }
figure { margin:0; background:var(--card); border:1px solid var(--line); border-radius:12px; overflow:hidden; }
figure img, figure video { width:100%; display:block; background:#000; } figcaption { padding:10px 14px; font-size:14px; }
svg text { font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif; fill:var(--ink); }
svg .v { fill:var(--vendor); stroke:var(--vendor-l); } svg .u { fill:var(--upstream); stroke:var(--upstream-l); }
svg .o { fill:var(--ours); stroke:var(--ours-l); } svg .p { fill:var(--plan); stroke:var(--plan-l); stroke-dasharray:5 3; }
svg rect { stroke-width:1.5; } svg .band { fill:none; stroke:var(--line); stroke-width:1; }
svg .arr { stroke:var(--muted); stroke-width:1.5; fill:none; marker-end:url(#ah); } svg .lab { font-size:12px; fill:var(--muted); }
.sw { display:inline-block; width:12px; height:12px; border-radius:3px; margin-right:5px; vertical-align:-1px; border:1.5px solid; }
.sw.v { background:var(--vendor); border-color:var(--vendor-l); } .sw.u { background:var(--upstream); border-color:var(--upstream-l); }
.sw.o { background:var(--ours); border-color:var(--ours-l); } .sw.p { background:var(--plan); border-color:var(--plan-l); border-style:dashed; }
"""

PAGES = [("index.html", "Demo"), ("status.html", "Status"), ("control.html", "Control levels"),
         ("architecture.html", "Architecture"), ("camera.html", "Cameras")]


def nav(here: str) -> str:
    return "<nav>" + "".join(f'<a href="{f}"{" class=here" if f == here else ""}>{t}</a>' for f, t in PAGES) + "</nav>"


def page(here: str, title: str, body: str, meta: str) -> str:
    return (f'<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">'
            f"<title>Piper Studio {html.escape(title)}</title><style>{CSS}</style></head><body><main>{nav(here)}"
            f'<h1>{html.escape(title)}</h1><div class="muted">{meta}</div>{body}</main></body></html>')


def mark(s: str) -> str:
    cls = {"✓": "ok", "◐": "warn", "✗": "bad"}.get(s[:1], "muted")
    return f'<td class="c {cls}">{html.escape(s)}</td>'


# --------------------------------------------------------------------------------------------------------------------
# Status
# --------------------------------------------------------------------------------------------------------------------
BACKENDS = ["Mock", "Gazebo", "MuJoCo", "Isaac Sim", "Real arm"]


def status_rows(facts: dict) -> list[tuple]:
    isaac_cam = facts.get("isaac_camera", "✗ not built")
    return [
        ("Launch, controllers, joint state", "✓", "✓", "✓", "✓", "◐ stand-in driver only"),
        ("Direct joint trajectory, streaming, gripper", "✓", "✓", "✓", "✓", "◐ stand-in driver only"),
        ("MoveIt: joint goal, pose goal, straight line, unreachable fails", "✓", "✓", "✓", "✓", "◐ untested on hardware"),
        ("MoveIt Servo: TCP twist / pose tracking", "✓", "✓", "✓", facts.get("isaac_servo", "✓"), "✗ not tried"),
        ("Wrist camera (D435): colour, aligned depth, camera_info", "— n/a", "✓ fixed today", "✓", isaac_cam, "✗ driver not set up"),
        ("Shared scene objects (cubes, cylinder)", "—", "✓", "✓", facts.get("isaac_scene", "✗ not yet"), "—"),
        ("Command guard (NaN / out-of-limit / jump filter)", "—", "—", "—", "—", "✓ tested with stand-in"),
        ("Automated end-to-end test", "✓ 9 + skip", "✓ 10/10", "✓ 10/10", facts.get("isaac_test", "opt-in, GPU"), "◐ plumbing test"),
    ]


def status_page(facts: dict, meta: str) -> str:
    rows = "".join("<tr><td>" + html.escape(r[0]) + "</td>" + "".join(mark(c) for c in r[1:]) + "</tr>" for r in status_rows(facts))
    head = "".join(f"<th>{b}</th>" for b in BACKENDS)
    body = f"""
<p class="lead">Everything below was run in simulation. <b>The real arm has not been moved by this repository</b>; the real-arm path is
exercised only against a stand-in driver. ✓ = passed in the automated test or the recorded demo; ◐ = partly; ✗ = not done.</p>

<h2>What works, per backend</h2>
<div class="card"><table><tr><th>Capability</th>{head}</tr>{rows}</table></div>

<h2>What does not work yet</h2>
<ul>
<li><b>Real arm:</b> never driven. Needs you present. First session is read-only (state, feedback, <code>arm_status</code>), then enable, then slow moves.</li>
<li><b>Isaac speed with cameras:</b> every rendered view costs Isaac about 8&nbsp;ms per frame. With the wrist camera alone it runs at about 0.6 of real time at 90&nbsp;Hz frames; with the observer video as well, about 0.4. Simulated time stays correct (everything runs on <code>/clock</code>), it is just slower on the wall clock. Without cameras it runs at real time.</li>
<li><b>Gazebo camera viewpoint</b> is 4&nbsp;cm ahead of the true D435 optical centre (the housing workaround), so Gazebo's view differs slightly from MuJoCo's and Isaac's, which agree closely. A visibility mask for the housing would remove the offset.</li>
<li><b>Isaac control latency:</b> joint targets travel over ROS topics to Isaac and back (one extra hop). NVIDIA recommends the in-process
<code>isaacsim.ros2.control</code> for this; not tried yet. See the <a href="control.html">control levels</a> page and the architecture notes.</li>
<li><b>Firmware-level modes</b> of the arm (Cartesian PTP/line/arc, per-joint MIT impedance, unsmoothed joint streaming, CPV, limits/payload/protection) are not exposed by our API. Details and the recommendation on the <a href="control.html">control levels</a> page.</li>
<li><b>Fault reporting:</b> the driver publishes <code>feedback/arm_status</code> (errors, motion state); we do not consume it yet.</li>
<li><b>Recording and policy interfaces</b> (LeRobot, VLA adapters), <b>Go2 composition</b>, a real RealSense driver launch: not started. Waiting for the discussion you asked for.</li>
<li><b>Simulator actuator parameters</b> (MuJoCo/Isaac servo gains, URDF effort limits, gripper stroke) are provisional until identified on the real arm.</li>
</ul>

<h2>Bugs found and fixed in this round</h2>
<ul>
<li><b>Gazebo camera showed only background.</b> Two causes, both ours: (1) the sensor frame sits <i>inside</i> the D435 housing mesh, so every ray hit the housing or nothing, fixed by placing the sensor at the lens plane (4&nbsp;cm along its viewing axis); (2) headless Gazebo needs the EGL renderer, so the server now starts as <code>gz sim -s --headless-rendering</code> through <code>piper_bringup/scripts/gz_server.sh</code>. A minimal world proved the renderer itself was fine (100&nbsp;% depth, correct range) before I looked at the robot.</li>
<li><b>Isaac arm sagged under gravity</b> (bare PhysX PD drive, about 9&nbsp;mm at the TCP), so Servo pose tracking settled 9&nbsp;mm short and its test failed. The Isaac arm bodies now feel no gravity, like MuJoCo's gravity compensation and the real arm's firmware; Servo converges to 3&nbsp;mm and the Isaac end-to-end test passes 10/10. This changes Isaac's tracking numbers on the demo page.</li>
<li><b>MuJoCo camera published at 5&nbsp;Hz</b> (the plugin default): set to 30&nbsp;Hz with <code>camera_publish_rate</code>.</li>
<li><b>Orphaned Gazebo servers</b> after a hard kill: the wrapper now has a watchdog that stops the server if the wrapper dies.</li>
<li><b>A test that never ran the camera check</b> on the simulators (the test launch did not request the camera): fixed; it also skips cleanly on backends without a camera.</li>
</ul>

<h2>Try the Gazebo camera on your desktop</h2>
<p>Needs a display. Use any free <code>ROS_DOMAIN_ID</code> (each domain gets its own Gazebo partition, so this does not disturb other sessions).</p>
<pre>cd ~/projects/ros/piper_studio &amp;&amp; git switch feature/servo-camera &amp;&amp; source scripts/env.sh
export ROS_DOMAIN_ID=77
ros2 launch piper_bringup piper.launch.py backend:=gazebo camera:=d435 gui:=true rviz:=true</pre>
<p>In a second terminal (same <code>source scripts/env.sh</code> and domain):</p>
<pre>ros2 run rqt_image_view rqt_image_view /camera/color/image_raw        # also /camera/aligned_depth_to_color/image_raw
ros2 run piper_py piper named ready                                  # camera looks down at the three objects
ros2 run piper_py piper joints 0 1.2 -1.0 0 0.6 0                    # move and watch the view change
ros2 run piper_py piper demo --backend gazebo --out /tmp/demo.json   # the full scripted sequence</pre>
<p class="muted"><code>gui:=true</code> opens the Gazebo window as a client of the headless server; I have not tried that combination myself (no display here),
so if the window stays empty, tell me what the terminal prints. The camera topics work without it. The MuJoCo equivalent is <code>backend:=mujoco camera:=d435 gui:=true</code>.</p>

<h2>Open decisions for you</h2>
<ol>
<li>Which real-arm streaming path to use for MoveIt/Servo/policies: firmware-smoothed <code>move_j</code> (current, safest), unsmoothed <code>move_js</code> (fast_mode), CPV, or MIT. Decide on the real arm; see the control levels page.</li>
<li>Whether to try the native in-process Isaac controller manager (removes the topic hop, needs system ROS sourced inside Isaac's process).</li>
<li>Recording / policy interface report: do you want it next, before the real-arm session?</li>
</ol>
"""
    return page("status.html", "Status", body, meta)


# --------------------------------------------------------------------------------------------------------------------
# Control levels
# --------------------------------------------------------------------------------------------------------------------
LEVELS = [
    ("Supervision & safety", "enable / disable, reset, e-stop, home, drag (teach) mode, limits, payload, crash protection, installation pose, firmware info",
     "all of it (<code>enable</code>, <code>electronic_emergency_stop</code>, <code>set_*_limits</code>, <code>set_payload</code>, <code>set_crash_protection_rating</code>, leader/follower, …)",
     "services <code>enable_agx_arm</code>, <code>control_enable</code>, <code>emergency_stop</code>, <code>move_home</code>, <code>exit_teach_mode</code>; parameters <code>auto_enable</code>, <code>speed_percent</code>, <code>tcp_offset</code>",
     "<span class=ok>partly</span>: <code>piper activate/deactivate</code> (checks feedback is fresh and matches before enabling controllers), <code>auto_enable:=false</code> default, command guard",
     "Ours is a safety wrapper the vendor lacks (the driver maps ROS's idle NaN to 0.0). <b>Gap:</b> e-stop, home, limits/payload read-out and <code>arm_status</code> faults are not reachable from <code>piper_py</code>. Add read-only first; writes need your approval."),
    ("Joint torque / impedance (MIT)", "per-joint position + velocity + torque with kp/kd, the lowest level the arm offers",
     "<code>move_mit</code> (per motor), V1.8.8 changes its parameters", "topic <code>control/move_mit</code> (<code>MoveMITMsg</code>)",
     "<span class=bad>not exposed</span>; our ros2_control joints have a position command interface only",
     "<b>Gap by choice.</b> Needed later for compliance, teleoperation and learned low-level policies. Needs an effort/MIT command interface and gain handling; do it with the real arm."),
    ("Joint streaming, unsmoothed (JS)", "follower mode: target is chased as fast as possible, no planning or smoothing",
     "<code>move_js</code> (docs: “extremely high risk”; older firmware needs <code>reset()</code> to leave it)", "topic <code>control/move_js</code>; parameter <code>fast_mode:=true</code> sends <i>all</i> <code>control/joint_states</code> through it",
     "<span class=bad>not used</span> (<code>fast_mode</code> is off)",
     "<b>Option, not a gap.</b> It is the vendor's answer for high-rate streaming (Servo, policies). Candidate for the real arm, with the stiffness caveat the docs give."),
    ("Joint position-velocity (firmware-smoothed)", "firmware plans the motion to each new target within a speed limit",
     "<code>move_j</code> + <code>set_speed_percent</code>", "topics <code>control/joint_states</code>, <code>control/move_j</code>; parameter <code>speed_percent</code>",
     "<span class=ok>used</span>: every ros2_control command (trajectories, streaming, MoveIt, Servo) reaches the arm as <code>control/joint_states</code> → <code>move_j</code>, through the command guard",
     "Matches the vendor's own recommendation as the default. <b>Check on the arm:</b> <code>speed_percent</code> is not set by us (driver default 0 = leave firmware default), and how the firmware smoothing behaves under 50–200&nbsp;Hz targets."),
    ("Joint position, per-joint gains (CPV)", "per-joint position / velocity command with tunable acc, dec, max speed, kp, ki",
     "<code>move_cpv_pos</code>, <code>move_cpv_vel</code>, getters/setters for each gain", "topic <code>control/move_cpv</code> (position only; velocity not exposed)",
     "<span class=bad>not exposed</span>", "<b>Gap.</b> Velocity mode would suit Servo-style jogging. Evaluate against <code>move_j</code> on the arm before building."),
    ("Cartesian, in firmware", "point-to-point, straight line and arc to a flange pose; the arm's own IK",
     "<code>move_p</code>, <code>move_l</code>, <code>move_c</code>, <code>set_tcp_offset</code>; V1.8.8 adds IK joint feedback", "topics <code>control/move_p</code>, <code>move_l</code>, <code>move_c</code>",
     "<span class=bad>not exposed</span>; TCP motion is done in software instead (next two rows)",
     "<b>Not redundant, not needed for parity:</b> firmware Cartesian moves cannot be simulated, are not collision-aware and differ per firmware. Keep as a real-arm-only convenience; expose later behind the same API."),
    ("Cartesian / task, in software: MoveIt", "collision-aware planning to joint or pose goals, straight lines (Pilz LIN), named poses",
     "— (not in the SDK)", "vendor ships an <code>agx_arm_moveit</code> config for its own launch",
     "<span class=ok>used</span>: one MoveIt config for every backend, plans execute through the same controllers; <code>piper_py</code> wraps joint, pose, linear and named goals",
     "<b>Ours is the superset</b> (shared across five backends, Pilz, Servo, scene objects, camera collision exclusions). Reuse the vendor's SRDF ideas, not its launch."),
    ("Cartesian velocity / servoing: MoveIt Servo", "real-time TCP twist and pose tracking with singularity and limit handling",
     "—", "—", "<span class=ok>used</span>: <code>servo_twist</code>, <code>servo_to_pose</code>; tested on every simulator",
     "<b>Our gap-closing work.</b> Output goes through the same streaming controller, so it inherits whatever real-arm path is chosen above."),
    ("Gripper", "width and force",
     "<code>gripper.move(width, force)</code>", "<code>gripper</code> joint inside <code>control/joint_states</code>; parameter <code>gripper_default_effort</code>; <code>feedback/gripper_status</code>",
     "<span class=ok>used</span>: ParallelGripperActionController; <code>piper_py</code> reports GRASPED when it stalls while closing",
     "Force is the vendor default for now; no force argument from our side, no gripper feedback consumed."),
    ("State feedback", "joint positions/velocities/efforts, flange/TCP pose, motor temperature and current, errors",
     "<code>get_joint_angles</code>, <code>get_flange_pose</code>, <code>get_motor_states</code>, <code>get_arm_status</code>, driver states",
     "<code>feedback/joint_states</code> (200 Hz), <code>feedback/tcp_pose</code>, <code>feedback/arm_status</code>, <code>feedback/gripper_status</code>",
     "<span class=warn>joint states only</span>", "<b>Gap.</b> TCP pose comes from our own TF/kinematics (consistent across backends); motor temperature/current and fault flags should be surfaced before any real session."),
]


def control_page(meta: str) -> str:
    rows = "".join(
        f"<tr><td><b>{n}</b><div class='muted'>{d}</div></td><td>{sdk}</td><td>{node}</td><td>{ours}</td><td>{verdict}</td></tr>"
        for n, d, sdk, node, ours, verdict in LEVELS)
    body = f"""
<p class="lead">Industrial arms are usually driven at a few levels: supervision, joint torque, joint streaming, joint motion, Cartesian motion, then task planning.
This page maps those levels onto what the Piper SDK (<code>pyAgxArm</code>), the official ROS node (<code>agx_arm_ctrl</code>) and Piper Studio offer. Everything here was read
from the vendor sources in <code>for_reference/agilex/</code> (26 AgileX repositories cloned for study; commits in <code>clone.log</code>), not from memory.</p>

<h2>Level by level</h2>
<div class="card"><table><tr><th>Level</th><th>Vendor SDK (pyAgxArm)</th><th>Vendor ROS node</th><th>Piper Studio today</th><th>Verdict</th></tr>{rows}</table></div>

<h2>The path a command takes today</h2>
<pre>piper_py / CLI / MoveIt / Servo
   -> ros2_control controllers (same on all backends)
   -> JointStateTopicSystem (real)  --hw/joint_commands-->  command_guard  --control/joint_states-->  agx_arm_ctrl
   -> pyAgxArm  move_j (firmware-smoothed)  ->  CAN  ->  arm firmware</pre>
<p>The guard filters NaN, out-of-limit and large jumps; the vendor node has none of that and turns NaN into 0.0, which is why it exists.</p>

<h2>Vendor behaviours that matter (from the SDK docs and node)</h2>
<ul>
<li>The SDK switches motion mode automatically on each <code>move_*</code> call; <code>set_auto_set_motion_mode_enabled(False)</code> turns that off. Our node does not touch it.</li>
<li><code>move_j</code> targets overwrite each other (no queue): streaming at a fixed rate is safe, but the arm's own smoothing sits between us and the joint.</li>
<li><code>move_js</code> has lower joint stiffness than position mode; below firmware S-V1.8-5, leaving JS mode needs <code>reset()</code> (arm powers off). The node decides this itself (<code>is_switch_seamlessly</code>).</li>
<li>Firmware profiles differ (DEFAULT, V183, V188): V188 changes the MIT message and mode codes and adds IK feedback. The node reads the firmware version and picks the profile.</li>
<li>Per-link Cartesian velocity/acceleration feedback is deprecated in the main controller.</li>
</ul>

<h2>Recommendation</h2>
<ol>
<li>Keep <code>move_j</code> as the default real path; it is the safest and what the vendor ships for ROS.</li>
<li>Before a real session, add <b>read-only</b> access to <code>arm_status</code>, motor states and firmware version to <code>piper_py</code> and show faults in <code>piper state</code>.</li>
<li>On the arm (you present), compare <code>move_j</code>, <code>move_js</code> (<code>fast_mode</code>) and CPV with the step-response tool, then choose the streaming path for Servo and policies.</li>
<li>Expose firmware Cartesian moves and MIT only as separate, clearly named real-arm calls; do not route simulators through them.</li>
<li>Reuse, don't duplicate: no Python client exists in the vendor ROS stack, so <code>piper_py</code> is not redundant; its MoveIt and Servo wrappers sit on upstream MoveIt, not on vendor code.</li>
</ol>
"""
    return page("control.html", "Control levels", body, meta)


# --------------------------------------------------------------------------------------------------------------------
# Architecture
# --------------------------------------------------------------------------------------------------------------------
def _box(x, y, w, h, kind, title, sub="", size=13):
    t = f'<text x="{x + w / 2}" y="{y + (h / 2 - 3 if sub else h / 2 + 5)}" text-anchor="middle" font-size="{size}" font-weight="600">{title}</text>'
    if sub:
        t += f'<text x="{x + w / 2}" y="{y + h / 2 + 13}" text-anchor="middle" font-size="11" class="lab">{sub}</text>'
    return f'<rect class="{kind}" x="{x}" y="{y}" width="{w}" height="{h}" rx="8"/>{t}'


def _arrow(x1, y1, x2, y2):
    return f'<path class="arr" d="M{x1} {y1} L{x2} {y2}"/>'


def architecture_svg() -> str:
    s = ['<svg viewBox="0 0 1000 760" role="img" aria-label="Piper Studio architecture" style="width:100%;height:auto">',
         '<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
         '<path d="M0 0 L10 5 L0 10 z" fill="var(--muted)"/></marker></defs>']
    band = lambda y, h, label: s.append(f'<rect class="band" x="4" y="{y}" width="992" height="{h}" rx="10"/>'  # noqa: E731
                                        f'<text x="14" y="{y + 16}" font-size="11" class="lab">{label}</text>')
    # layer 5: clients
    band(6, 104, "5  Clients")
    for i, (k, t, sub) in enumerate([("o", "piper CLI", "state · move · demo"), ("o", "piper_py.Piper", "Python API"),
                                     ("u", "RViz2", ""), ("p", "VLA / agent adapter", "planned"),
                                     ("p", "Recording (LeRobot)", "planned"), ("p", "Go2 composition", "planned")]):
        s.append(_box(16 + i * 163, 28, 151, 66, k, t, sub))
    # layer 4: motion and task
    band(118, 104, "4  Motion and task (ROS 2 actions / topics)")
    for i, (k, t, sub) in enumerate([("u", "MoveIt 2 move_group", "OMPL · Pilz LIN"), ("u", "MoveIt Servo", "TCP twist / pose"),
                                     ("o", "MoveIt config", "SRDF poses · limits"), ("u", "Joint trajectory", "FollowJointTrajectory"),
                                     ("u", "Position streaming", "50 Hz targets")]):
        s.append(_box(16 + i * 197, 140, 185, 66, k, t, sub))
    # layer 3: ros2_control
    band(230, 92, "3  ros2_control (one configuration for every backend)")
    s.append(_box(16, 250, 470, 60, "o", "controllers.yaml", "joint_state_broadcaster · JTC · position stream · gripper"))
    s.append(_box(500, 250, 230, 60, "u", "controller_manager", "ros2_control 4.x"))
    s.append(_box(744, 250, 240, 60, "o", "command_guard", "real arm only: NaN · limits · jumps"))
    # layer 2: hardware interface per backend
    band(330, 96, "2  Hardware interface: the only per-backend difference")
    cols = [("u", "mock_components", "ideal tracking"), ("u", "JointStateTopicSystem", "→ real arm topics"),
            ("u", "gz_ros2_control", "inside Gazebo"), ("u", "mujoco_ros2_control", "inside MuJoCo"),
            ("u", "JointStateTopicSystem", "→ Isaac topics")]
    xs = [16 + i * 197 for i in range(5)]
    for x, (k, t, sub) in zip(xs, cols):
        s.append(_box(x, 352, 185, 62, k, t, sub))
    # layer 1: plant
    band(434, 150, "1  Plant")
    s.append(_box(xs[0], 456, 185, 112, "u", "(none)", "software only"))
    s.append(_box(xs[1], 456, 185, 54, "v", "agx_arm_ctrl", "vendor ROS node"))
    s.append(_box(xs[1], 514, 185, 54, "v", "pyAgxArm → CAN", "vendor SDK · firmware"))
    s.append(_box(xs[2], 456, 185, 112, "u", "Gazebo Harmonic", "gz-sim 8 · DART · EGL cameras"))
    s.append(_box(xs[3], 456, 185, 112, "u", "MuJoCo 3.12", "generated MJCF + scene"))
    s.append(_box(xs[4], 456, 185, 54, "o", "isaac/run_piper.py", "OmniGraph ROS 2 bridge"))
    s.append(_box(xs[4], 514, 185, 54, "u", "Isaac Sim 6.1 · PhysX", "own process on the GPU"))
    # arrows between layers
    for x in xs:
        s.append(_arrow(x + 92, 414, x + 92, 456))
    for x in (100, 590, 860):
        s.append(_arrow(x, 310, x, 352))
    s.append(_arrow(500, 206, 500, 250))
    s.append(_arrow(500, 110, 500, 140))
    # description band
    band(596, 158, "Robot description and sensors (feeds every layer)")
    s.append(_box(16, 618, 230, 60, "v", "agx_arm_description", "official URDF + meshes · unmodified"))
    s.append(_box(262, 618, 330, 60, "o", "piper_description", "xacro overlay: tcp_link · D435 · ros2_control per backend"))
    s.append(_box(608, 618, 376, 60, "o", "generators", "physics URDF · MJCF + scene · Gazebo world SDF · Isaac USD"))
    s.append(_arrow(246, 648, 262, 648))
    s.append(_arrow(592, 648, 608, 648))
    s.append(_box(16, 690, 230, 52, "u", "realsense2_description", "D435 model"))
    s.append(_box(262, 690, 330, 52, "o", "sim cameras → /camera/*", "Gazebo · MuJoCo · Isaac"))
    s.append(_box(608, 690, 376, 52, "p", "realsense2_camera (real)", "same topics and frames · planned"))
    s.append(_arrow(246, 716, 262, 716))
    s.append(_arrow(592, 716, 608, 716))
    s.append("</svg>")
    return "".join(s)


PROVENANCE = [
    ("Robot geometry, limits, inertials, meshes", "AgileX <code>agx_arm_urdf</code> via the <code>external/agx_arm_ros</code> submodule, unmodified (pinned e4ccec1)", "Never copied or edited; we generate from it"),
    ("Real-arm driver", "AgileX <code>agx_arm_ctrl</code> + <code>pyAgxArm</code> SDK", "Used as shipped; our guard and activation logic sit in front"),
    ("Motion planning, servoing", "MoveIt 2.12.4 (OMPL, Pilz, Servo)", "Our SRDF, limits, Servo tuning, tests"),
    ("Controllers, hardware abstraction", "ros2_control, <code>joint_state_topic_hardware_interface</code>, <code>mock_components</code>", "Our one <code>controllers.yaml</code> and ros2_control xacro"),
    ("Gazebo / MuJoCo plugins", "<code>gz_ros2_control</code>, <code>mujoco_ros2_control</code>", "Our model generators, world, scene objects, camera sensor"),
    ("Isaac Sim integration", "Isaac Sim 6.1 URDF importer and ROS 2 OmniGraph nodes", "Our <code>isaac/</code> runner and conversion (the vendor's <code>piper_isaac_sim</code> is only a reference: no license)"),
    ("Camera model and mount", "<code>realsense2_description</code> D435; AgileX mount pose from <code>piper_isaac_sim</code>", "Our placeholder bracket, simulated sensors, the Gazebo lens-plane fix"),
    ("Python API, CLI, demo, step-response study", "—", "Entirely ours (<code>piper_py</code>)"),
    ("Real-arm safety defaults", "—", "Ours: no auto-enable, controllers start inactive, command guard, activation checks"),
]


def architecture_page(meta: str) -> str:
    prov = "".join(f"<tr><td>{a}</td><td>{b}</td><td>{c}</td></tr>" for a, b, c in PROVENANCE)
    body = f"""
<p class="lead">One description, one set of controllers, one API. The only thing that changes between the mock, the three simulators and the real arm is the hardware interface
and what sits behind it.</p>
<p><span class="sw v"></span>vendor (AgileX), unmodified &nbsp; <span class="sw u"></span>upstream open source &nbsp; <span class="sw o"></span>written here &nbsp; <span class="sw p"></span>planned, not built</p>
<div class="card">{architecture_svg()}</div>

<h2>Repositories and packages</h2>
<div class="card"><table>
<tr><th>Where</th><th>What</th></tr>
<tr><td><code>piper_studio</code> (this repo, branch <code>main</code> + feature branches)</td><td>everything below; the legacy work is kept at tag <code>legacy-final-20261008</code></td></tr>
<tr><td><code>external/agx_arm_ros</code> (submodule)</td><td>AgileX's official stack: <code>agx_arm_ctrl</code>, <code>agx_arm_description</code>, <code>agx_arm_moveit</code>, <code>agx_arm_msgs</code></td></tr>
<tr><td><code>src/piper_description</code></td><td>xacro overlay and the generators (physics URDF, MJCF, Gazebo world, scene objects), model audit against the official URDF</td></tr>
<tr><td><code>src/piper_bringup</code></td><td>the one launch file, controllers, MoveIt and Servo config, command guard, Gazebo server wrapper</td></tr>
<tr><td><code>src/piper_py</code></td><td>Python API, <code>piper</code> CLI, demo recorder, step-response tool, stand-in driver, end-to-end tests</td></tr>
<tr><td><code>isaac/</code></td><td>Isaac runner and USD conversion, run in Isaac's own environment (nothing installed into it)</td></tr>
<tr><td><code>for_reference/agilex/</code> (git-ignored)</td><td>26 AgileX repositories cloned for study: SDKs, simulators, kinematics, gravity compensation, VLA training repos</td></tr>
</table></div>

<h2>Where each part comes from</h2>
<div class="card"><table><tr><th>Part</th><th>Source</th><th>Our contribution</th></tr>{prov}</table></div>

<h2>Design rules the structure enforces</h2>
<ul>
<li>One robot description; simulator models are generated from it, never hand-edited (<code>test_description.py</code> guards this).</li>
<li>One controller configuration for every backend; backends differ only in the hardware plugin.</li>
<li>One command owner per resource; switching controllers is explicit.</li>
<li>Success means the controller or MoveIt reported the goal executed, not just planned.</li>
<li>Real arm: motors are not enabled by launch, command controllers start inactive, every command passes the guard.</li>
</ul>

<h2>How the simulators differ (and why plots differ)</h2>
<ul>
<li><b>Gazebo</b>: a velocity P-loop around the position target, effectively first order and independent of link inertia: the closest to the ideal mock.</li>
<li><b>MuJoCo</b>: a PD servo with inertia; Isaac uses a PhysX drive. These two are physically alike and differ from Gazebo's actuator model, not from the geometry.</li>
<li><b>Isaac</b>: also pays one topic hop each way because ros2_control runs outside Isaac. The in-process alternative is the open experiment noted on the status page.</li>
</ul>
"""
    return page("architecture.html", "Architecture", body, meta)


# --------------------------------------------------------------------------------------------------------------------
# Cameras
# --------------------------------------------------------------------------------------------------------------------
def camera_page(site: Path, meta: str) -> str:
    def fig(media, title, cap):
        f = site / media
        if not f.exists():
            return ""
        if media.endswith(".mp4"):
            poster = media[:-4] + ".png"
            ptag = f' poster="{poster}"' if (site / poster).exists() else ""
            return f'<figure><video controls preload="metadata"{ptag} src="{media}"></video><figcaption><b>{title}</b><br>{cap}</figcaption></figure>'
        return f'<figure><img src="{media}" alt="{title}"><figcaption><b>{title}</b><br>{cap}</figcaption></figure>'

    stills = "".join([
        fig("gazebo_wrist.png", "Gazebo: wrist camera, ready pose", "Colour (top) and depth (bottom, black = no return)."),
        fig("mujoco_wrist.png", "MuJoCo: wrist camera, ready pose", "Same pose, same scene objects."),
        fig("isaac_wrist.png", "Isaac Sim: wrist camera, ready pose", "Same pose, same scene objects."),
    ])
    vids = "".join([
        fig("gazebo_overview.mp4", "Gazebo: observer camera during the demo", "Fixed camera rendered inside Gazebo (EGL, headless)."),
        fig("gazebo_wrist.mp4", "Gazebo: wrist camera during the demo", "What the D435 sees; published on /camera/color/image_raw."),
        fig("mujoco_wrist.mp4", "MuJoCo: wrist camera during the demo", "Same topics and frames."),
        fig("mujoco.mp4", "MuJoCo: observer view (replay)", "Replay of the recorded run through the generated model."),
        fig("isaac.mp4", "Isaac Sim: observer view (live render)", "Rendered by Isaac while ROS drives it."),
    ])
    body = f"""
<p class="lead">The simulated Intel RealSense D435 sits on link 6 at AgileX's mount pose, publishes colour, aligned depth and camera_info on the topics and frames the real
<code>realsense2_camera</code> driver uses (<code>/camera/color/image_raw</code>, <code>/camera/aligned_depth_to_color/image_raw</code>, frame <code>camera_color_optical_frame</code>), and sees the same three objects in every simulator.</p>
<h2>Same pose, each simulator</h2><div class="grid">{stills or "<p class='muted'>no stills yet</p>"}</div>
<h2>Videos</h2><div class="grid">{vids or "<p class='muted'>no videos yet</p>"}</div>
<h2>Notes</h2>
<ul>
<li>Differences in shading and floor texture are cosmetic (each renderer has its own defaults); the object positions and the view geometry agree.</li>
<li>The bracket is a placeholder block: the real one is not designed yet and AgileX's mesh is not redistributable (no licence).</li>
<li>Gazebo needs <code>--headless-rendering</code> without a display, and the sensor must sit in front of the D435 housing mesh; both are handled in the repo.</li>
</ul>"""
    return page("camera.html", "Cameras", body, meta)
