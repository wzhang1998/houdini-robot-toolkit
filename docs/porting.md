# Porting the show to another arm (UF850 / xArm)

The steps to put a UFACTORY 850 (or an xArm 6/7) under the **full** show
— library, build, stream, tracking, Isaac — in the order they depend on each
other, each with how far the code already supports it. The contracts these
steps fill in are in [architecture/contracts.md](architecture/contracts.md);
the FR20-specific code each step works around is listed in
[architecture/coupling.md](architecture/coupling.md).

Legend: ✅ works today · 🟡 partly (says what is missing) · ❌ nothing yet

| # | Step | UF850 today | xArm today |
|---|---|---|---|
| 0 | [Will the show fit the arm?](#0-will-the-show-fit-the-arm) | ❌ not checked — see below, probably not as is | ❌ |
| 1 | [Profile](#1-profile) | 🟡 `profiles/uf850.json` (HDA-complete; not show-complete) | ❌ |
| 2 | [URDF and meshes](#2-urdf-and-meshes) | ❌ FBX only | ❌ |
| 3 | [IK solver](#3-ik-solver) | 🟡 FBIK in Houdini; ❌ in Python | ❌ |
| 4 | [Backend](#4-backend) | 🟡 the contract and its tests; ❌ the code | ❌ |
| 5 | [Limits and acceleration, measured](#5-limits-and-acceleration-measured-on-the-arm) | 🟡 datasheet / series figures | ❌ |
| 6 | [Tool, cable, collision model](#6-tool-cable-collision-model) | ❌ | ❌ |
| 7 | [The Python side reads the profile](#7-the-python-side-reads-the-profile-not-fr20) | ❌ 111 `"fr20"` code sites | ❌ |
| 8 | [Show config overlay](#8-show-config-per-arm) | ❌ | ❌ |
| 9 | [Library: re-solve, re-check, re-test](#9-library-re-solve-re-check-re-test) | ❌ (needs 2, 3, 7) | ❌ |
| 10 | [Isaac, TouchDesigner, perception](#10-isaac-touchdesigner-perception) | 🟡 perception ✅; Isaac and TD ❌ | same |

Steps 1–6 are about the arm and can start now, in parallel. Step 7 is the
code change everything after it waits for. Run `python scripts/contracts/robot.py`
at any point to see what a profile still lacks.

## 0. Will the show fit the arm?

Before any code: the party show was designed around an arm with **~1.85 m**
reach and a **20 kg** payload. The UF850 reaches **~0.85 m** with **5 kg**; an
xArm 6 ~0.7 m with 5 kg (vendor datasheets — confirm for the exact model).

- The show's hubs are Cartesian: `high.tcp = [-0.95, 0.0, 1.45]` is ~1.7 m
  from the base — out of a UF850's reach whatever its base height.
  `greet`, `low` and the scan area are similar. The **arm-neutral** half of
  the show config (contracts.md §3.1) is neutral in format, not in scale: the
  hubs, zones, stage, canvas and scan area must be redrawn (or the base raised
  and moved closer) for a smaller arm.
- The tool is a **1 m UV bar** (`tool.led_span_m`); its mass is not recorded
  (`tool.mass_kg: null`). On a 5 kg arm, weigh it and check the moment at the
  flange against the vendor's payload chart before anything else.
- The UF850's joint ranges differ (J1 ±360, J3 −241.9..3.4, J5 ±123.9) —
  the FR20's `range.j1_deg` and cable range are not transferable.

**Decide first:** the same show scaled down for a desk-size arm, or a different
show on the same toolchain. The rest of this page holds either way.

## 1. Profile

`profiles/uf850.json` exists and drives the HDA (FBX rig, three frames,
limits, FBIK quirks, UFACTORY CSV writer). For the show it lacks:

| Key | Status | Do |
|---|---|---|
| `robot.home_deg` | ❌ | choose a home clear of singularities (step 3), inside the limits; `_check_profile` verifies |
| `robot.max_velocity_deg_s` per joint | 🟡 one number (180) | per joint from the datasheet, then measured (step 5) |
| `robot.max_acceleration_deg_s2` | 🟡 1146, the series figure | measure (step 5) |
| `rig.urdf` | ❌ | step 2 |
| `rig.ik_solver` | 🟡 `fbik` | step 3 |
| `rig.ik_reference_deg` | ❌ | the IK's tie-breaker with no previous frame (≈ home) |
| `tool` | ❌ | step 6 |

For an xArm: copy `uf850.json`, change `id`, `label`, limits and envelope from
the datasheet, and either find an FBX rig or go URDF-only (the FR20 route:
`"fbx": null`, `"urdf": …`, axes/sign/offset measured by `urdf_rig.py`).
**Status: 🟡** — the profile format and loader are done; the values are work.

## 2. URDF and meshes

The Python side (collision capsules, TCP, IK, cuRobo, Isaac) needs a URDF with
**STL** link meshes (`collision._stl_vertices` reads STL only).

1. Source: UFACTORY's ROS 2 description package (`xarm_ros2`,
   `xarm_description`: xacro for `uf850` and `xarm6/7`). Expand the xacro to a
   plain URDF for the exact model and firmware; copy it and its meshes to
   `assets/ufactory_description/`, as `assets/fairino_description/` was —
   unmodified, and record the source and its licence in the README layout table.
2. `python scripts/urdf_rig.py` builds the rest skeleton from any URDF (each
   joint classified as twist or hinge, a skewed joint raises) — **tested on the
   FR20 only**; add the new arm to its tests.
3. **URDF zero = controller zero?** Measure it, as was done for the FR20
   (23 poses in SimMachine, `GetForwardKin` vs URDF FK to 0.016 mm): compare the
   vendor's FK (xArm SDK `get_forward_kinematics`, on UFactory Studio's
   simulation) against `ur_ik.pose_of` over ~20 poses. Any offset goes into
   `rig.offset_deg` / `sign`, never into the clips.
4. `rig.flange_offset_m`: measure from the last link's mesh
   (`urdf_rig.flange_offset_from_mesh`), as for the FR20.
5. The FBX rig the HDA uses today and the URDF must agree; or switch the HDA to
   the URDF path (`rig.fbx: null`), which `FR20_rig.hiplc` shows works.

**Status: ❌** — no UFACTORY URDF in the repo; the tools that consume one are generic.

## 3. IK solver

Every Python generator and check solves with `ur_ik` (closed form, up to 8
branches, µm accuracy). It accepts only UR-type arms — J2 ∥ J3 ∥ J4 — and the
UF850 / xArm put a **roll** at J4 (the UF850 profile's own `rotation_axis` is
`y z z y z y`: J4 along the forearm). Expect `ur_ik.analyse` to raise
`NotURType("… J2 parallel to J4 fails")` on their URDFs — confirm once step 2
lands. FBIK in Houdini works but is mm-level and does not enforce J4's limits.

Options, in the order I would try them:

| Option | Branches | Accuracy | Work | Notes |
|---|---|---|---|---|
| **Numerical IK on the URDF**, multi-seed (the seeds enumerate shoulder/elbow/wrist configurations), polished with `ur_ik._polish` | as many as the seeds find | µm (`_polish` is chain-generic: damped Newton on `pose_of`) | small–medium | same interface as `ur_ik` (`solve`, `within_limits`, `nearest`), so `gestures.Rig` and `choreo` take it without change |
| Closed form for the arm's own geometry | all | exact | medium | only if its wrist axes meet (check on the URDF, as `ur_ik.analyse` checks UR-ness) |
| IKFast (OpenRAVE-generated) | all | exact | medium, tooling-heavy | |
| The controller's IK (xArm SDK `get_inverse_kinematics`) | one | exact for the controller | small | online only, one branch, per-call latency: an authority check, not a generator |

Whatever is chosen: it is selected by `rig.ik_solver` and implements the
`ur_ik` functions the generators call (`solve`, `within_limits`, `nearest`,
`pose_of`, `_polish`, `_pose_error`, `orthonormalize`), and it is validated
against the vendor's IK on the simulator the way `ur_ik` was against
`GetInverseKin`. Also record the arm's singularities (profile v2
`kinematics.singular`): `show.wrist_singular_report` and `show_stream.recover`
assume the FR20's J5 = 0.

**Status: 🟡** — Houdini (FBIK) yes; Python no.

## 4. Backend

A `RobotBackend` (`scripts/contracts/robot.py`) for the UFACTORY controller,
opened through a `BackendFactory` (a sender connection and a feedback one).
Sketch of the mapping onto the xArm Python SDK (`xarm-python-sdk`, `XArmAPI` —
names from the SDK's documentation; confirm against the installed version):

| RobotBackend | xArm SDK | To verify |
|---|---|---|
| `model()` | the device type / version the SDK reports | must contain `UF850` (the profile id) |
| `joints()` | `angles` (degrees with `is_radian=False`) | how fresh the reported angles are (report socket rate) |
| `error_code()` | `error_code`, `warn_code` → `[0, main, sub]` | which codes are a singular-pose stop (→ `FaultPolicy`) |
| `reset_errors()` | `clean_error()`, `clean_warn()`, `motion_enable(True)`, `set_state(0)` | |
| `prepare()` | `set_mode(0)`, `set_state(0)` | |
| `move_to(q, vel_pct)` | `set_servo_angle(angle=q, speed=…, wait=True)` | speed in deg/s, not % |
| `servo_start()` | `set_mode(1)`, `set_state(0)` | |
| `servo_j(q, cmd_t, id)` | `set_servo_angle_j(q)` | **no `cmd_t`**: the controller moves to each point at once — pacing is entirely the sender's clock; the FR20's `playback_ppm` / lag-correction logic may not apply. Rate the controller wants (100–250 Hz?) |
| `servo_end()` | `set_mode(0)`, `set_state(0)` | |
| `stop()` | `set_state(4)` | callable from a second `XArmAPI` instance while the first is mid-send? |
| feedback connection | a second `XArmAPI`, or the SDK's report thread | whether two instances may connect |

Then: `python scripts/contracts/conformance.py --robot mymod:factory --profile uf850`
**against UFactory Studio's simulation, never the arm** (the suite moves it,
and with `--allow-fault-tests` sends a jump and an out-of-limit target), until
every check passes — including the 3 s run of `show_stream.stream` on it.
`td-robot-twin` (PLAYBACK_FINDINGS.md, cited in `fairino_player.py`) has the
lessons from streaming this arm before.

The rest of the driver path is FR20-specific and needs the backend threaded
through ([migration step 3](architecture/migration.md)): `show_stream.main` and `track_mode` construct
`P.Controller(ip)` and `Link(ip)` themselves, check `SIM_NET`, and
`fairino_player.py` is the only player CLI.

**Status: 🟡** — the interface, `SimBackend` and the conformance suite exist; no UFACTORY code.

## 5. Limits and acceleration, measured on the arm

The FR20's acceleration came from `accel_ui.py` / `accel_probe.py` on the arm
(clean to 900 deg/s², planned at 300 / 600 because of the base mounting). Do
the same for the new arm: per joint, rising steps, tracking error and the
controller's faults, the base mounted as the vendor specifies.

- `accel_probe.py` uses `fairino_player.Controller` and `play` directly:
  ❌ until it takes a `RobotBackend` (small change, after step 4).
- Cross-check `robot.limits_deg` against the controller's **soft** limits (the
  FR20's comment says it still needs this too).
- Payload: set the tool's mass in the controller and in the profile (step 6).
- Write the result into the profile with `_comment_acceleration` saying
  "MEASURED <date>" — `scripts/contracts/robot.py` reports a figure without
  that as `partial`.

**Status: 🟡** — procedure and tools exist, bound to the FR20 driver.

## 6. Tool, cable, collision model

- Mount the tool URDF (`assets/tools/uv_bar.urdf`: root = mounting face, `tcp`
  link, box collision) on the new flange: the frame convention is the tool's,
  so it carries over; the flange offset is the arm's (step 2).
- `tool.cable_j6_deg`: the cable's J6 range on *this* arm's flange.
- `collision.load_model(profile)` fits capsules to the URDF's STL meshes and
  adds the tool's boxes — generic, ✅ once steps 2 and 7 are done.
- cuRobo (`curobo_bridge.py`) writes its robot config from the profile + URDF,
  but `up` loads `"fr20"` by name (coupling.md §A1).
- Prefer moving `tool` out of the arm profile (contracts.md Gap R3) before
  copying it into a second one.

**Status: ❌** for the new arm; the machinery is generic.

## 7. The Python side reads the profile, not "fr20"

The one code change everything after it waits for: thread a profile id from
the show (a new `robot` key in `shows/<name>.json`, default `"fr20"` so nothing
changes) through `show.build`, the generators, the checks and the tracking
layer. In order of payoff:

1. `capability.load_fr20(root)` → `load_rig(root, profile_id)`; `gestures.Rig(profile_id)` (10 users).
2. `collision.load_model("fr20")` → the caller's profile (38 sites); `RP.load("fr20")` (25); `M._tcp_path("fr20", …)` (11).
3. Remove the `robot="fr20"` defaults (12) once every caller passes one.
4. Clips: write the profile actually used into `clip["robot"]`; refuse to stream a compiled show whose robot is not the backend's (contracts.md Gap M1).

`python scripts/contracts/coupling_audit.py --sites` lists every site and is
the checklist. Each sub-step is behaviour-preserving for the FR20 and is
checked by rebuilding `party` and diffing against the tracked compiled show
(`scripts/rebuild_library.py` on the `review-loop` branch does exactly this).

**Status: ❌**

## 8. Show config per arm

The show config mixes the show's intent with FR20 joint values (contracts.md
§3.1): `hubs.rest.q`, every hub's `near` IK seed, `range.j1_deg`, and
`gestures.REACH_M` / `SHOULDER` in code. Proposal: keep `shows/party.json`
arm-neutral and put those in an overlay `shows/party.<profile>.json` the build
merges; for a small arm, also the redrawn hubs, zones, stage and scan area
(step 0). `envs/*.usda`: where the new base stands and the controller's work
area.

**Status: ❌**

## 9. Library: re-solve, re-check, re-test

Never retarget the FR20's joint angles ([ADR 005](adr/005-clips-are-solved-per-arm.md)).
Rebuild from the arm-neutral sources:

1. `show.py build shows/party.json` with the new profile → a compiled show for
   the new arm: every clip generated, measured (`motion_clip.measure`),
   collision-checked, range-checked; dropped clips and warnings in `info`.
2. `show.py report` — variety, energy spread, what the library misses at each
   hub; `scripts/review_render.py` (the `review-loop` branch) for the eye.
3. `show.py dry-run` — continuity, trigger latency.
4. Isaac (`isaac/run_show.py`, `record_library.py`) — needs step 10.
5. The vendor simulator: `show_stream` with the new backend, `rehearse.py`
   (every motion once).
6. The arm: `--speed 0.3`, every motion once (`--rehearse`), then a long run;
   the stream report's tracking, lag and faults.

**Status: ❌** — every tool exists for the FR20; each needs step 7.

## 10. Isaac, TouchDesigner, perception

- **Perception: ✅ unchanged.** `SceneState` is in the robot base frame; if the
  new arm's base is elsewhere, re-solve the extrinsic (people_track Calibrate)
  and nothing downstream changes.
- **Isaac: ❌** `isaac_stage.py` hard-codes the FR20 URDF path, `/World/fr20`
  and `fr20_usd`; the scripts name the articulation `"fr20"` (6 sites). Take
  the URDF from the profile.
- **TouchDesigner: 🟡** OSC out (`/robot/joints`, `/robot/strip_a/_b`,
  `/robot/facing`, `/robot/paper`) is computed by `show.strip_ends`,
  `facing`, … with `robot="fr20"` defaults — correct once step 7 passes the
  profile. TD's own scene (a robot model, if any) is outside this repo.
- **Directors: ✅ unchanged** — they name clips and triggers, never joints;
  the library index changes, the contract does not.
