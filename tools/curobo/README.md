# tools/curobo: the cuRobo sphere model of the Piper (optional, only to regenerate it)

Everything hand-tuned is in `policy.yaml`; the pipeline is `build_model.py` (builder -> `fit_spheres.py` -> `finalize.py` -> `sphere_quality.py` -> `validate.py`).
Needs the optional environment (`scripts/curobo_env.sh setup`) and the GPU. Ownership, usage by downstream planners, steps and the staleness tests: `docs/CUROBO.md`.
