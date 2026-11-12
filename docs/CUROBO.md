# cuRobo model of the Piper: Piper Studio owns it

Written 2026-11-11 (0.10.0). Decision (user): everything **hand-tuned** about the cuRobo side, and **how to regenerate it**, lives here, next to the description it is derived from.
Downstream planners (`piper_pick`, the cuRobo study, anything else) import the finished model and keep only planning code.

## Who owns what
| Piper Studio (this repo) | Downstream planner repos |
|---|---|
| the sphere fit policy and every tuned number (`tools/curobo/policy.yaml`: overhangs, per-link rules, operating scale, buffers) | planning code: `PiperPlanner`-style wrappers, scenes, grasp selection, retries, payload handling |
| the generation pipeline (`tools/curobo/`: fit, finalize, quality, validate, `build_model.py`) | their own cuRobo/torch environment (`piper_pick/.venv`, the study's) |
| the generated models + provenance + validation + quality, shipped in `piper_assets` | their own tuning on top (they change the returned dict, never the shipped file) |
| staleness and consistency tests (CPU), the optional cuRobo environment, the model viewer | evidence about *their* use (pick results, sweeps) |

Studio never imports torch or cuRobo for normal work, and the wheels do not depend on them.

## Using the model (downstream)
```python
import piper_assets as pa
top = pa.curobo_config(camera="zedxm")        # dict for cuRobo's RobotCfg.create(); camera=None: the robot only. urdf_path is set to the shipped URDF.
pa.curobo_model_is_current("zedxm")           # False: the description moved on; the model must be regenerated here before you trust it
pa.curobo_provenance("zedxm")                 # description commit, fingerprint, every decision, sphere counts, validation and quality results
pa.curobo_planner_defaults()                  # algorithm, interpolation step, collision cache, warmup: starting values
```
Change what you need on the returned dict (`load_dynamics`, `position_limit_clip`, payload spheres, ...); never edit the shipped file. **Use the model of the arm that is physically there**
(the camera is on the real arm since 2026-11-11, so `camera="zedxm"` is the arm that is there; run on it: CHECKLIST R3.10). The model with the camera covers the bracket only about 70 % (numbers below).

## The arm on the Go2 (setup `go2`, nominal mount)
The arm model is unchanged; only its surroundings change. `pa.curobo_config(setup="go2")` is the same arm (with the camera) and `pa.curobo_world("go2")` the world: **one box** around the whole dog at its stand pose (0.71 x 0.33 x 0.42 m) plus a small box for the mount plate (the numbers the URDF places it with) and any `payload_boxes` listed in `config/go2/go2_setup.yaml` (the ZED Box Mini, a battery, ...; none given yet), all in the arm's base frame, which is the frame cuRobo plans in.
The world is otherwise empty (no floor). Today the plate lies inside the dog's box (its top is 1 mm under the box top), so it changes nothing yet; it matters once the plate is measured or sticks out. The box is **generated**, not written: `tools/go2_cover/build_dog_cover.py` puts the Go2's meshes and collision shapes at the stand pose of `config/go2/go2_setup.yaml`
(go2_studio's `STAND`) and takes their bounding box; `dog_cover_report.json` records the Go2 description hashes and that every point is inside, and a test regenerates it and compares. From the arm it looks like standing on a block.
A box fitted to the legs and body was tried and dropped (2026-11-12): boxes cannot hug round legs, the gap stayed 17-47 mm with 80 boxes per link. Needs the Go2 only standing: the legs are not modelled as moving.
**Known, from the nominal mount:** the box top is the dog's highest point (the shoulder hump, z = 0.0893 in the Go2 `base` frame), the arm's base spheres start 1.96 mm above it and the model's sphere buffer is 2.0 mm, so a planner applying the buffer sees `base_link` in contact with the dog.
A plate of 31 mm or more (the estimate is 30 mm) clears it; `test_go2_cover.py` keeps this as a strict xfail until the mount is measured. CPU checks (no cuRobo): the arm is clear of the box at `zero` and `ready`; 33 of 150 random poses inside the limits would hit it.
The viewer (`/piper-studio/curobo/`) shows it as the model `piper_go2` (default). **One cuRobo planning in this world (GPU, 2026-11-12, `tools/curobo/plan_on_go2.py`):** with the nominal 30 mm plate it FAILS: the start is 1.96 mm from the dog's box, inside the model's 2.0 mm sphere buffer, and `ready` is not planned
(`config/go2/planning_check_nominal_plate30.json`). With the plate 6 mm thicker (36 mm, the dog and plate boxes lowered in the arm frame) it plans 31 of 31 goals (`ready` + 30 random joint poses no closer to the dog than the arm's own base):
goal error at most 2.1 mm, no limit violated, every trajectory clear of the world with the buffer (minimum 3.0 mm, the base), mean solve 67 ms after the first call (`planning_check_plate36.json`). The 100-pose validation of the arm model itself (`validation_zedxm.json`) was made on a table and is unchanged.

## Regenerating (only when the description or `policy.yaml` changes; optional environment)
Normal users never need this: running, controlling and simulating the arm, the wheels and reading the model need no cuRobo.
```bash
scripts/curobo_env.sh setup                      # once: .venv-curobo (cuRobo at env/curobo.rev in third_party/curobo + torch, ~7 GB; versions: env/constraints-curobo.txt)
python tools/release/build_assets.py             # ROS env: fresh URDFs/SRDFs (after any description change)
scripts/curobo_env.sh run tools/curobo/build_model.py        # GPU (gpu-run): builder, fit, finalize, quality, validate for both arms -> src/piper_description/config/curobo/
python tools/release/build_assets.py && python -m pytest src/piper_assets -q     # wheel data with the new models; staleness + consistency tests
```
Each model records the fingerprint of the URDF (joints, collision meshes by content) and SRDF pairs it was built for (`piper_assets.urdf_fingerprint`); `test_curobo_model.py` fails when the
description changed without a regeneration, when a link pair overlaps at `zero` or `ready` (CPU forward kinematics, no GPU), when limits, jerk or the zero pose differ from `limits.json`,
or when the 100-pose planning validation was not run or was poor.

## The viewer
`scripts/curobo_viewer.sh start` -> http://localhost:8782/ (tailnet: `https://atlas.buri-fence.ts.net:8700/piper-studio/curobo/`). Spheres per link, live self-collision highlight, sliders, the named poses `zero` and `ready`,
the URDF meshes, and whether the model is current. It reads the shipped models only (no cuRobo); `--extra DIR` views an experiment's yml.

## What the ZED X Mini bracket costs (the one tuned decision)
The bracket is a thin clamp 15 mm from the fingers: the default 12 mm sphere overhang put its spheres into the finger spheres at zero (0 of 100 plans). Its own overhang is 6 mm
(`policy.yaml`), which clears the fingers but leaves it covered only about 71 % with up to 10.3 mm of it outside the spheres (`quality_zedxm.json`; the camera body: 95 %, 3.6 mm): keep 2 cm more clearance near the wrist. Planning
speed is about the same as the robot only (validation 2026-11-11: 92 of 100 tool poses planned for both models, mean solve 73 and 84 ms, goal error under 5 mm, no limit exceeded). The camera pose is nominal (bracket v3, not hand-eye calibrated); nothing here has run on the real arm.

## History
Until 2026-11-11 the model and its scripts lived in `~/projects/ml_tests/experiments/curobo_piper_study` (a study that found how to do this) and were copied into `piper_pick` and the bake-off; they drifted
and the camera model did not exist. The study is now a client of `piper_assets`; its scripts 08-11 and 16 became `tools/curobo/` here (the policy as a yml, limits from `limits.json`).
