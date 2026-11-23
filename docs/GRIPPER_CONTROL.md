# Gripper control: what the firmware and the SDK give us, and the plan for a distance AND force target

Status 2026-11-23: findings from the vendor SDK source, the earlier measurements (`docs/real_arm/REPORT.md` section 4, raw files `<data dir>/gripper/*.json`, 2026-11-10) and the vendor demos. Nothing new was run on the arm for this note. Inferences are marked.

## 1. What the interface is (SDK `pyAgxArm` d90e234, `agx_gripper`)
| | Command `0x159` (we send) | Feedback `0x2A8` (arm sends, 200 Hz) |
|---|---|---|
| width | int32, 1 um (`move_gripper_m` rounds to 1e-6 m) | int32, 1 um |
| force | int16, 1 mN: **a limit on the move** (vendor ROS node clips it to 0.5-3 N) | int16, 1 mN: **measured**, negative while gripping |
| other | enable / clear-error / width-or-angle mode, set-zero | status byte: voltage low, motor and driver over-temperature, over-current, sensor, driver error, enabled, zeroed; mode |
There is **no speed field**: the firmware closes at its own speed (0.28 s for the stroke, nearly independent of force). The only things the firmware takes are a position target and a force limit. Any slower close, and any stop on a condition, is the host re-sending targets.
Also `0x47D/0x47E` (range configuration: `max_range_config` 0 / 70 / 100, `teaching_range_per` 100-200 %, `teaching_friction` 1-10): a lead for the width-map problem (70 mm read, about 90 mm real), to try in the caliper session. The old vendor ROS node reads the gripper's "effort" as `grippers_effort / 1000` into the joint effort field: the same family as the command's force; the unit label (N or N.m at the shaft) is the vendor's, not checked against a gauge.

## 2. Is the force feedback a measurement or the echo of the command? (inference from the saved data)
**A measurement, almost certainly of motor current (a torque estimate), not the command and not back-EMF.**
- Empty, at rest: open +0.18 N, closed -0.08 N, noise about +-0.2 N. An echo of a 0.5-3 N command would read the command.
- While closing on the box the peak is **-3.4 / -5.7 / -6.7 N** at 1 / 2 / 3 N commanded (more than the command, more than the 2.09 N plateau): the sensor does not saturate at 2.09, so **2.09 N is the regulated holding force, not a sensor limit**.
- The steady value is not the command either: -1.04 at 1 N (+4 %), -2.09 at 2 N, -2.09 at 3 N.
- Back-EMF is proportional to speed and is zero at a stall; the reading is -2.09 N at a stall. A current-based estimate fits (the driver is a FOC driver, its status bits say so). **Not proven** and not calibrated: whether "N" is newtons at the fingertips needs a force gauge between the fingers (a small compression load cell or calibrated FSR, 0-10 N), one session.

## 3. A finding that changes how a grasp is carried (from `object_box.json`, three repeats per force)
**The holding force decays.** 3 s after the stall the reading is **-0.26 to -0.51 N at every commanded force (1, 2, 3 N)**, while the width grows 0.5-1.1 mm (fingers and box spring back as the force falls). In the slip test the force dips from -2.08 to -0.47 N within a 2 s arm move. So the firmware appears to reduce the stall current with time (stall protection, inference): after about 1-3 s the grip is a quarter of what was commanded, whatever was commanded. Open questions: the time course (we only have the endpoint), whether re-sending the command resets it (probably), the limit at 0.1-0.5 N.

## 4. What the bus allows (computed from the documented rates, not measured on this arm)
1 Mbit/s CAN, 8-byte standard frame = 111 bits, up to about 135 with stuffing. Feedback: 0x2A1 + 3 + 3 + 1 frames at 200 Hz (1600/s), motors 0x251-6 at 100-200 Hz (600-1200/s), drivers 0x261-6 at 20-40 Hz (120-240/s) = 2300-3000 frames/s = **26-41 % of the bus**. Our commands (three joint frames + mode + gripper at up to 200 Hz) add about 11-14 %. One gripper feedback stream is 2 % of the bus, one gripper command stream at 200 Hz another 2 %: **the bus is not the limit.** The limits:
- the gripper feedback is a new sample every **5 ms**; at 250 mm/s the fingers move 1.25 mm per sample;
- `RealArm.set_gripper` polls at 50 Hz (20 ms: 5 mm of travel at full speed): a host stop on a distance overshoots by 5 mm or more, so **a distance target must be the firmware's position target, never a host-side stop**;
- the USB-CAN adapter and Python parse thread add latency we have not measured (command to first motion; command rate the firmware accepts: unknown, the vendor demos send about 10-50 Hz).

## 5. Design (what we build)
`set_gripper(width, effort, timeout, speed=None, force=None)` in `RealArm`, `PiperSim`, `PiperIsaacSim` (one implementation, `piper_core.gripper.run_gripper_move`):
- **width** is sent to the firmware as the position target (it stops exactly there, 0.14 mm repeatable). **effort** is the force limit sent with it (the firmware regulates to it: the measured plateau tracks it up to 2.09 N).
- **speed** (m/s, optional): the host ramps the position target. Needed to keep the impact peak down (the peak at full speed is 2-3 times the steady force); to be measured at 25 / 50 / 100 / 250 mm/s.
- **force** (N, optional, the measured force at which to stop): contact is detected from the feedback (debounced: at least 3 consecutive samples above the target, 15 ms at 200 Hz, and above the 0.3 N noise floor), then the ramp stops and the position target is set to the measured width; the effort stays as the limit. The impact spike makes a force trigger fire at first contact, so the force target is only meaningful together with a slow `speed`.
- **First one wins** (the owner's rule), reported as `reason`: `width` (distance reached), `force` (force reached), `stalled` (the fingers stopped and neither target was reached: blocked physically), `timeout`, `no_motion`. The result carries both measured numbers and two flags, `width_ok` and `force_ok`, so a caller that needs **both** checks them afterwards. Requiring both as a control law is not possible: only one of them can be driven, the other follows from the object; "both" is a verdict, not a mode.
- **Hold**: because of section 3, a held grasp re-sends the command (target = the stalled width, effort = the force target) at a low rate while the arm moves, if re-sending is shown to restore the force.
- The simulators get the same arguments through the same function (done for speed and effort in 0.12); the impact peak and the decay are modelled only after they are measured (`actuation.yaml`).

## 6. Hardware session (the owner at the arm; no agent moves the arm). `piper-gripper-probe`, to be written, each step on its own
1. Echo and rates: send a command, read `get_gripper_ctrl_states()` (what the firmware accepted: is 3 N stored as 3 N, a 0.2 N as 0.2 N?), print `get_gripper_status().hz`; log raw feedback timestamps for 5 s (is it 200 Hz, how steady).
2. Latency: command a step from open, log time to first width change at the full feedback rate.
3. Command rate: ramp to 0 at 10 / 20 / 50 / 100 / 200 Hz; tracking and any refusal.
4. Force: the box and a soft object (foam, a sponge), efforts 0.2 / 0.5 / 1 / 2 / 3 N, the full 200 Hz force trace for 10 s: steady value, peak, **the decay curve**, and does re-sending restore it.
5. Speed vs impact: ramp 25 / 50 / 100 / 250 mm/s onto the box, peak force.
6. Gauge: the load cell between the fingers against the reading (is "N" newtons).
Each step reports numbers into `docs/real_arm/REPORT.md`. Only after that does the host-side force stop get built on the real arm; the simulator and fake-SDK parts can be built and tested first.
