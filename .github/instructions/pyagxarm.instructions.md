---
description: "Use when editing the pyAgxArm SDK, its tests, stubs, or SDK documentation. Covers standalone packaging, COLCON_IGNORE, firmware-aware APIs, and targeted pytest validation."
applyTo: "src/pyAgxArm/**"
---

# pyAgxArm Instructions

## Environment Setup — Required Before Any Command

Always activate the workspace venv before running SDK tests or tooling:

```bash
source /opt/ros/jazzy/setup.bash
source <workspace_root>/.venv/bin/activate
```

ROS distro is **jazzy**. The workspace venv is `.venv/` at the workspace root.

- `src/pyAgxArm` is a standalone Python SDK that remains in this workspace for coordinated development but is intentionally excluded from `colcon build` by `COLCON_IGNORE`.
- Use the workspace `.venv` for SDK installs, tests, and tooling. Prefer editable installs or direct local test runs over changing the colcon strategy.
- Preserve cross-platform CAN interface behavior and docs: Linux `socketcan`, Windows `agx_cando`, and macOS `slcan`.
- Preserve firmware-aware behavior for Piper and Nero variants. If a public API changes, update the relevant tests and any affected docs or stubs in the same change.
- Keep public typing support intact. `pyproject.toml` declares `py.typed` and ships `*.pyi`; update them when signatures or exported behavior changes.
- Validate SDK changes with targeted `pytest` runs in `src/pyAgxArm/tests` before considering broader checks.