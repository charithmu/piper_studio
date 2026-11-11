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
(the camera has not been on the real arm yet: `camera=None` until CHECKLIST R3). The model with the camera covers the bracket only about 70 % (numbers below).

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
