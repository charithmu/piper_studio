# Preserved TCP-description experiment

The saved ROS branch tip `6d7ec47` references nested description commit `f56478761ebbb4e038270fec1e3f6f760f83a131`. This commit had no branch/ref in the description checkout and must not be assumed reachable on the official remote.

- Parent: `3080af4b579238c850b709c411abdc88fc930c82`.
- Local preserved ref: `archive/legacy-tcp-description-20261008`.
- [Git bundle](agx_arm_urdf-tcp.bundle): exact commit and added file objects, 1,272 bytes; requires the parent history.
- [Patch](agx_arm_urdf-tcp.patch): readable/reapplicable version of the two added TCP xacros.
- Bundle SHA256: `5bb8297a657dc0b099391087b137067b95ec5beec62bfee718286aba5b6334d4`.

Restore the ref only in an initialized description repository containing the parent. From the superrepo root:

```bash
git -C src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf fetch "$PWD/docs/overhaul/legacy/agx_arm_urdf-tcp.bundle" refs/heads/archive/legacy-tcp-description-20261008:refs/heads/archive/legacy-tcp-description-20261008
```

This preserves an earlier experiment. The overhaul should compare it against current official flange/TCP composition before retaining any patch.
