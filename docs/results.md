# Results log

Measured results, kept for good (IMPLEMENTATION_PLAN.md is deleted when its
stages are done -- its Status numbers are copied here first).

Every entry: date, the number, before -> after, **where** it was measured, and
the commit. "Where" is one of:

- **unit** -- a test script, no robot
- **Houdini** -- the scene / PDG, no robot
- **SimMachine** -- Fairino's controller simulator (FR20-V1-001 V6.0 image), no arm moving
- **real FR20** -- the physical arm in the lab

Only **real FR20** rows describe something the arm has actually done.

## Real FR20

| Date | Result | Before -> after | Commit |
|---|---|---|---|
| 2026-09-25 | First clip streamed on the real arm: `fr20_test_new.csv` (323 frames, 13.4 s), ServoJ at 125 Hz, 3 runs, 0 controller errors | -- | bdfa47c (retime), 6c4b335 (play_ui) |
| 2026-09-25 | At speed 1.0 the clip played at its designed length, 13.4 s, not slowed (peak J5 acceleration 148.8 of 150 deg/s^2) | the same curve's earlier export needed x8.54 | bdfa47c |
| 2026-09-25 | Tracking after removing the controller's ~114 ms lag: max 0.80 / 0.76 deg, RMS 0.29 / 0.26 deg (speed 1.0); two runs agree within 0.23 deg, end pose 0.035 deg | raw, un-aligned error ~7 deg (all lag) | -- |
| 2026-09-25 | Speed 0.3 run: 26.5 s (x1.97, since speed scales the acceleration limit, not the time), max 0.43 deg, RMS 0.11 deg | -- | -- |
| 2026-09-25 | Room probed with the arm (`probe_ui.py`): our URDF TCP vs the controller's TCP ~1.1 mm on every point | SimMachine: 0.004 mm | c92509d, 9d73d98 |
| 2026-09-25 | Measured room: floor, two walls (arm mounted turned 11.28 deg to the room), ceiling 2.16 m by tape; the rest from a RoomPlan scan aligned to the probed walls (agree to 0.47 deg) | an estimated room | 9d73d98, 1f9c43c |
| 2026-09-25 | Acceleration probe (`accel_ui.py`), every joint to 900 deg/s^2 with no controller error; tracking 150 -> 900: J1 0.20 -> 0.38, J2 0.24 -> 0.49, J3 0.26 -> 0.63, J4 0.30 -> 0.69, J5 0.29 -> 0.67, J6 0.38 -> 1.34 deg. J1-J3 visibly shake (arm + plywood base) | planned at 150 on every joint | 074bdd0, 0087660 |
| 2026-09-25 | Planning limits set from it: J1-J3 300, J4-J6 600 deg/s^2 (J1-J3 kept low until the base is bolted to spec) | 150 all joints | 7435161 |
| 2026-09-25 | Controller interference zone (cube, "work area") stops the arm when the TCP leaves it -- tested with PTP in auto mode (jog ignores it) | -- | (WebApp setting, no code) |

Not yet run on the real arm: any dance clip, `--goto-home`, a clip refused by
the cell check. Dance clips so far: dry-run and SimMachine only.

## SimMachine

| Date | Result | Commit |
|---|---|---|
| 2026-09-24 | FR20 kinematics: our FK vs `GetForwardKin` 0.016 mm / 0.0006 deg over 23 poses; the controller's IK is among our branches 18/18 | 4c64d36 |
| 2026-09-24 | ServoJ player: 240-frame curve, 2666 sends at 125.04 Hz, 0 skips, max lateness 0.55 ms, send p50/p95/max 1.2/3.4/8.3 ms; aligned tracking max 0.072 deg (RMS 0.018), lag ~40 ms | 4c64d36 |
| 2026-09-24 | Probe tip vs controller TCP 0.004 mm | -- |
| 2026-09-25 | d01 dance clip played at speed 1.0, no controller error | -- |

## Houdini / unit

| Date | Result | Before -> after | Commit |
|---|---|---|---|
| 2026-09-24 | Closed-form IK: 10,000 random in-limit poses, every branch reproduces its target (worst 4e-12 m, 8e-11 deg); drawn curve 240/240 frames <= 0.005 mm, 4 branch changes, all forced by a limit | FBIK (iterative) | 4c64d36 |
| 2026-09-24 | Retime with acceleration (TOPP): 433 frames, dry-runs at time scale 1.0, 18.0 s | x8.54 slower than designed | a6a5ae4 |
| 2026-09-25 | Retime fits the keyed, cooked frames: the user's curve passes Pre-Flight at 323 frames; 216 at the measured limits | 1.04 -> 1.18 with a uniform stretch (the failed first fix) | bdfa47c, 7435161 |
| 2026-09-24 | Capability atlas PDG bake (10 cm, with the room): 136 s of work in 38 s over 8 items; 45 % of the reachable space clear of the lab, tool down | -- | f921932, 0617eb2 |
| 2026-09-25 | Primitive factory (50 variants) against the measured room at the measured limits: 24/50 ok (14 unreachable, 12 cell) | 20/50 (14 unreachable, 16 cell) | 7435161 |
| 2026-09-25 | Dance factory: 48/48 ok; d32 37.6 -> 25.8 s at the new limits | -- | 7435161 |
| 2026-09-25 | Export round trip through the asset exact (0.00000 deg), with J4 shifted by a whole turn into its limits | J4 233 deg "over its limit" | 47e3286, b9b730b |
| 2026-09-25 | Clip review in PDG: 98 clips rendered, tiled, 266 s | -- | 9a73869, 54ef62d |

## Failures and what they taught

- **Labels drift at new limits** (2026-09-25, Houdini). The measured Laban
  labels were calibrated at 150 deg/s^2. At 300/600 the same phrases measure
  differently: d32 generated float-slash-press-float, measured
  flick-wring-punch-slash; in the review 76/127 bars measure as intended. Found by the
  factory's own label-vs-intent check and in the review videos. Open:
  recalibrate. Picture: `geo/review/page_dance_*.mp4`.
- **26 of 50 primitives refused** (2026-09-25, Houdini): 14 unreachable, 12
  hit the room (mostly the upper arm near the floor, one into the left
  partition). Shown framed red in the review, the unreachable part of the
  path drawn red. The system refusing clips is the point: the check runs
  before anything reaches the robot.
- **IK success criterion changed** (2026-09-24, unit). "No joint step above
  the velocity limit" failed on the test curve -- but the curve itself needs
  8-14 deg/frame on J4/J6 near the wrist singularity. Rewritten as "every
  branch change is forced by a joint limit": a property of the solver, not of
  the curve.
- **Retime still 1.04x slow** (2026-09-25, Houdini). The fit measured
  un-eased frames on a different grid than the player; a uniform stretch made
  it worse (1.18x). Fixed by fitting on the keyed, eased, cooked frames.
- **Floor fitted 6 deg tilted** (2026-09-25, real FR20 probe). Three nearly
  colinear points gave a plane +17 cm off. Fixed with a spread warning and
  "wall" / "level" fits that use what is known (walls vertical, floor level).
- **MoveJ error 154 + FK mismatch of 11.28 deg** (2026-09-25, real FR20). A
  workpiece frame was left active on the controller after setting the safe
  zone; `GetForwardKin` answers in that frame. The player now refuses to move
  and says which frame to reset. ccbf75f
- **Runs labelled "sim" were real** (2026-09-25). play_ui's Target was
  SimMachine while the IP pointed at the real arm, so no confirmation was
  asked and files were named `_sim_`. Noticed from the controller model in the
  report.

## Media index

Real-robot video: none recorded yet. Name files
`YYYYMMDD_<clip>_speed<s>.mp4` (e.g. `20260925_d17_speed0.6.mp4`) with a
matching Houdini viewport capture from the same angle, and list them here.

| File | Where | What |
|---|---|---|
| docs/images/portfolio/review_contact_sheet_1600.jpg | Houdini (OpenGL, clean) | the review contact sheet, 1600 px: 98 clips, rejected framed red |
| geo/review/contact_sheet.png | Houdini | every clip, one frame, rejected framed red |
| geo/review/page_*.mp4, overview.mp4 | Houdini | clip review videos |

Still to make (1600 px, clean -- the Karma previews are noisy): atlas, cell,
dance sheet.

## Credits

To fill in: whose lab, who operated, collaborators on the installation, who
owns the FR20.
