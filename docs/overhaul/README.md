# Piper Studio overhaul proposal

Date: 2026-10-08. Status: proposed architecture for user review; implementation has not started.

The objective is a reproducible workspace for the standard Piper and AgileX gripper, with Gazebo, MuJoCo, Isaac, and hardware profiles; multiple control levels; optional MoveIt; reproducible data collection; and adapters that permit later camera and robot-platform integration.

For agent replacement, read the self-contained [session takeover document](../../TAKEOVER.md), which records the initial instruction, evolving scope, actual work, approvals, and next steps.

The existing implementation is preserved on `archive/legacy-work-20261008`. Historical plans and the new proposal are on `planning/overhaul-20261008`. Archive status is not a claim that the legacy stack builds or operates correctly.

- [Preservation record](PRESERVATION.md): commits, additional branch tips, backup, and the unpublished description commit.
- [Audit](AUDIT.md): observed defects, provenance, and validation limits.
- [Architecture](ARCHITECTURE.md): proposed layers, contracts, command ownership, models, and optional profiles.
- [Roadmap](ROADMAP.md): bounded implementation jobs and acceptance criteria.
- [Upstream candidates](UPSTREAMS.md): verified official revisions and selective reuse of community work.
- [Preserved branch data](preservation.json): machine-readable inventory of package archive refs.
- [Official candidate revisions](official-candidates.json): research inputs, not an approved compatibility lock.

`PLAN.md`, `ImplementationPlan.md`, `DESIGN.md`, `TODO.md`, and the package graph retain historical proposals. They have not been rewritten to imply that this new architecture is implemented. Once the design is agreed, the active README, DESIGN, TODO, and agent instructions should point to one current roadmap.

## Decisions proposed

1. Preserve legacy code and histories before updating vendor dependencies; implement the overhaul on a separate integration branch.
2. Use the latest official SDK, ROS stack, and descriptions as compatibility candidates, qualified together and then pinned to exact revisions.
3. Keep robot-specific transport and firmware logic in adapters and the official SDK; expose capability-based control and observation interfaces above them.
4. Make MoveIt optional and maintain direct joint, trajectory, Cartesian-servo, task, and policy entry points.
5. Share semantic model and controller contracts across backends; validate dynamics separately.
6. Prefer workspace-owned packages in the superrepo when their releases are coupled; retain external sources as pinned dependencies. Decide the history-preserving consolidation method before moving repositories.

Publishing the proposal does not approve repository consolidation, dependency installation, firmware changes, or hardware motion.
