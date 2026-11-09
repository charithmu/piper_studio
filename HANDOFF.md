# Handoff (2026-11-10)

v0.5.0 includes the existing Isaac stepping/recording optimizations, full GUI entry points and four versioned wheels. Validation and limitations: docs/VERSIONS.md.

All test simulators were stopped; gpu-run reservation released. GUI runs use a local display and private unix sockets, with no new TCP service or gateway route. Registry source lists the interactive GUI and benchmark jobs; no hosting or platform files were changed.

Launch: `scripts/isaac.sh run --gui` (ROS) or, with the four wheels installed, `python -m piper_isaac --gui --scene table`. Ctrl-D closes the interactive client.
