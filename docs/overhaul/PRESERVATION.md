# Preservation record

Date: 2026-10-08. Publishing status: pending exact-destination authorization after automatic approval review rejected the batch push. All commits and archive refs below currently exist locally.

## Superrepo branches and separation

- `archive/legacy-work-20261008` at `f6aa65b`: existing implementation, package pointers, camera rename, user deletions, and agent configuration. Two integration-completion changes make source retrieval explicit: register the existing MuJoCo repository and use the accessible camera fork URL.
- `planning/overhaul-20261008`: starts from the legacy archive; commit `2f5f491` saves historical plans/graphs and session handoffs separately. Subsequent commits preserve the orphaned model work and add the new overhaul proposal.
- Existing main/master/ros2 branches remain unchanged. No history rewrite, reset, force push, repository deletion, or GitHub repository rename was performed.

## Package archive refs

Each user-owned package has `archive/legacy-work-20261008`. Where the existing local main/ros2 branch was ahead of the checkout, its tip is also preserved as `archive/legacy-branch-tip-20261008`. The snapshot retains the actual pre-overhaul checkout; later branch tips are not silently merged into it.

| Package | Snapshot SHA | Additional old branch tip |
|---|---|---|
| `src/pyAgxArm` | `cd6f878` | — |
| `src/agx_arm_ros` | `125ebe2` | `6d7ec47` |
| `src/agx_arm_gzsim` | `735e076` | `0ff5333` |
| `src/agx_arm_motion` | `235fdd6` | — |
| `src/agx_arm_detect` | `20678a3` | `20f0e7d` |
| `src/agx_arm_wristcam` | `3a4d689` | `fac10b0` |
| `src/agx_arm_graspgen` | `4f3b99b` | `4b29e09` |
| `src/agx_arm_manipulation` | `7b814f4` | `16dc98b` |
| `src/agx_arm_workspace` | `2164dd2` | — |
| `src/agx_arm_mjsim` | `95f6d89` | — |

New child commits: motion `235fdd6`, detection `20678a3`, wrist camera `3a4d689`. The other snapshot refs preserve existing commits. The camera source remains at `charithmu/agx_arm_eyes`; its ROS package/path is `agx_arm_wristcam`.

The official nested description stays at `3080af4`, and MoveIt calibration stays at `3f9d48e`. These upstream repositories are not push targets.

## Unpublished legacy description work

The later ROS tip `6d7ec47` refers to description commit `f56478761ebbb4e038270fec1e3f6f760f83a131`, whose parent is `3080af4`. It adds the two top-level TCP xacros. It had no local ref and is not assumed reachable from an official remote.

It is preserved on local branch `archive/legacy-tcp-description-20261008`, in [the 1,272-byte bundle](legacy/agx_arm_urdf-tcp.bundle), and as [a readable patch](legacy/agx_arm_urdf-tcp.patch). The bundle preserves the exact commit and requires parent `3080af4` to be present; the patch is a review/reapplication alternative.

A fresh legacy snapshot uses `3080af4` and does not require this special restore. To inspect the later ROS branch together with its exact old nested description, first initialize the original description checkout, then fetch the bundled ref from the superrepo root:

```bash
git -C src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf fetch "$PWD/docs/overhaul/legacy/agx_arm_urdf-tcp.bundle" refs/heads/archive/legacy-tcp-description-20261008:refs/heads/archive/legacy-tcp-description-20261008
```

Select that description branch only in an isolated clean checkout when inspecting the later ROS work. The new overhaul must evaluate the patch against the current official flange/TCP design; it is not automatically retained.

## Backup and static validation

Backup: `/tmp/piper-studio-preservation-20261008T153911Z`, 247,031,510 bytes before the supplemental orphan-description bundle. Contains 13 verified history bundles, saved indexes/config/ref metadata, staged/unstaged binary patches, inventory, and 804 SHA256-verified non-ignored working files. Build/install/venv runtime products are excluded. `/tmp` is temporary storage; publishing the archive refs and portable model bundle provides durable Git preservation.

Static checks: 25 Python files parsed, five package XML files parsed, launcher shell syntax passed, and 168 candidate source/document files scanned for selected secret patterns with no matches. The existing overlay emitted missing-path warnings when sourced. No builds, launches, GPU jobs, installs, or hardware commands were performed.

## Publication procedure

1. Obtain authorization for the exact existing user-owned destinations listed in `comms/NEEDS_USER.md`.
2. Push only the new package archive refs; verify each advertised remote SHA.
3. Verify every superrepo gitlink corresponds to a remotely reachable source commit, accounting separately for the bundled legacy experiment.
4. Push parent archive/planning refs, verify remote SHAs, and validate checkout/submodule metadata in an isolated directory.
5. Record the result in communication/evidence files and report exact branch links.

Automatic approval review rejected the first batch push because explicit authorization for those exact destinations and their trusted ownership was required. No push was executed by that rejected command. Git commits and design work continued locally.
