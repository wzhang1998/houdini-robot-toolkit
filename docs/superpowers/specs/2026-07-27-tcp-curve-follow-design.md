# TCP Curve Following for the UF850 IK Rig — Design

**Date:** 2026-07-27
**Scene:** `ufactory_850_rig_v2_22.hiplc` — `/obj/geo1`
**Houdini:** 22.0.368
**Scene units:** meters (1.0 unit = 1 m)

## Goal

Drive the UF850's tool centre point (TCP) along an arbitrary curve, replacing the
current static hand-posed IK goal. The tool must move at constant speed, support
two selectable orientation behaviours, and report when the requested path leaves
the arm's reachable envelope.

## Current State

The IK branch in `/obj/geo1` works like this:

```
configurejoints1 ──┬─→ blast1 ─→ add1 ─→ rigpose_ik ─→ fullbodyik1 ─→ FK_IK ─→ bonedeform2
                   └──────────────────────────────────→ fullbodyik1 (input 0)
```

- `blast1` keeps only point 5 (`joint_6`), producing the single IK goal point.
- `rigpose_ik` (a `kinefx::rigpose`) sets that goal via parameters — one
  transformation, `group0 = @name=joint_6`, static at
  `t = (-0.291, -0.349, 0.464)`, `rz = 72.5°`. Distance from origin: 0.650 m.
- `fullbodyik1` solves with `root = @name=joint_1`, `pinroot = 1`, solver 1,
  30 iterations, damping 0.5.
- `configurejoints1` defines the 6-DOF constraint set — one rotation axis enabled
  per joint (`joint_1` is `ry` at ±360°, with `rx`/`rz` pinned to 0, and so on),
  published as the `fbik_jointconfig` point dict.

### Goal point attribute signature (verified, not assumed)

The geometry entering `fullbodyik1` input 1 is **1 point, 0 prims**, carrying:

| Attribute | Class | Type | Meaning |
|---|---|---|---|
| `P` | point | float3 | goal position, world space |
| `transform` | point | **float9 (matrix3)** | goal rotation, world space |
| `rest_transform` | point | float9 | rest rotation |
| `localtransform` | point | float16 | parent-relative 4×4 |
| `name` | point | string | `joint_6` |
| `fbik_jointconfig` | point | dict | joint limits consumed by FBIK |
| `parent_idx`, `path`, `fbx_node_type` | point | — | KineFX bookkeeping |

**Position and rotation are separate attributes.** `transform` is a matrix3, not
a 4×4. This is load-bearing: it lets position be computed once and shared, with
only the rotation branching between orientation modes.

> Note: `context/kinefx-patterns.md` in the houdini-agent toolkit documents
> `transform` as a 4×4. That is wrong for this rig and should be corrected.

## Decisions

| Question | Decision | Rationale |
|---|---|---|
| Orientation | **Selectable** tangent-aligned or aim-at-target | Both are needed depending on the operation |
| Speed | **Arc-length constant speed + keyframable progress** | Real toolpaths need steady TCP speed; keyframes allow dwells and easing |
| Deliverable | **Previz now, real arm later** | Build export-ready, but do not build export |
| Curve source | **Wired node input** | Any curve works; no hardcoded path |
| Unreachable poses | **Measure the residual** | FBIK fails silently against joint limits |
| Packaging | **Inline nodes in `/obj/geo1`**, not an HDA | YAGNI; promote later if it needs reuse |

### Rejected alternatives

**Expression-driven `rigpose_ik` parms.** Would require decomposing a rotation
matrix into Euler angles across six independent parameter expressions, each
re-evaluating geometry every cook. Rotation-order bugs are near-certain and the
orientation-mode switch would have to be duplicated six times.

**KineFX rig constraint network (`constraintpath`).** The textbook rigging
answer and the right choice if this grows into a multi-target rig with stacked
constraints. For a single goal point it is disproportionate machinery, and
constraint networks are harder to debug when FBIK stalls.

## Architecture

```
CURVE_IN ─→ resample_arclen ─→ orient_along_curve
 (null)     Even Length Segs    parallel-transport frames
                                        │
configurejoints1 → blast1 → add1 ───────┼─→ sample_curve_pos ──┬─→ orient_tangent ─┐
                             │          │      (writes @P)     └─→ orient_aim ─────┤
                             │                                                     ↓
                             │                                            ORIENT_MODE (switch)
                             │                                                     ↓
                             └─→ rigpose_ik ──────→ GOAL_MODE (switch) ←───────────┘
                                                          ↓
                                                    fullbodyik1 (input 1)
                                                          ↓
                                                   measure_residual ──→ residual_guide
                                                          ↓              (display-only)
                                                       FK_IK → bonedeform2
```

Two new switches, matching the existing `FK_IK` idiom already used in this scene:

- **`GOAL_MODE`** — `0 = manual` (existing `rigpose_ik`, untouched), `1 = curve`.
- **`ORIENT_MODE`** — `0 = tangent`, `1 = aim`.

`rigpose_ik` is preserved and continues to work as it does today. The only
rewiring of existing nodes: `fullbodyik1` input 1 moves from `rigpose_ik` to
`GOAL_MODE`, and `FK_IK` input 1 moves from `fullbodyik1` to `measure_residual`.

## Components

Each node has one job and is named for what it produces.

### `CURVE_IN` — `null`
Wiring point for the path curve. Any curve topology works. Isolating it as a null
means the curve source can change without touching downstream nodes.

### `resample_arclen` — `resample`
**Even Length Segments** on. This is what converts uniform parametric `u` into
uniform arc length, which is what makes constant `progress` velocity equal
constant TCP velocity. Without it, TCP speed varies with the curve's authored
point spacing.

### `orient_along_curve` — `orientalongcurve`
Produces stable per-point orientation frames with explicit up-vector and twist
control. Chosen over `polyframe` or a hand-rolled cross product because it uses
parallel transport, which prevents the 180° tool flip that occurs when the curve
tangent swings near the up axis. That flip is the classic failure mode of naive
`maketransform(tangent, up)` and is unacceptable on a real toolpath.

### `sample_curve_pos` — `attribwrangle`
Single `primuv()` lookup at `progress` against the resampled curve (input 1).
Writes `@P` on the goal point. Runs over points, one point.

### `orient_tangent` — `attribwrangle`
Samples the frame from `orient_along_curve` at the same `u` and writes
`3@transform`. Tool axis follows the curve direction.

### `orient_aim` — `attribwrangle`
Computes `normalize(aim_target - @P)` as the tool axis, with `up_vector`
resolving roll, and writes `3@transform`. Tool points at a fixed world position.

### `measure_residual` — `attribwrangle`
Runs over detail. Input 0 is the solved skeleton from `fullbodyik1`; input 1 is
the requested goal from `GOAL_MODE`. Finds the solved `joint_6` world position,
compares it to the requested goal position, and writes:

- `f@tcp_residual` — distance in metres between requested and achieved TCP
- `i@tcp_offtarget` — `1` when `tcp_residual > residual_tolerance`

The flag is named `tcp_offtarget`, not `tcp_unreachable`, because a residual has
three possible causes and the node cannot distinguish them:

1. **Genuinely out of reach** — target beyond the arm's envelope.
2. **Solver non-convergence** — `iterations` too low for the requested move.
3. **Position/orientation conflict** — the 6-DOF constraint set cannot satisfy
   the requested TCP position *and* rotation simultaneously, so FBIK trades one
   against the other.

Cause 3 is the common one on this rig (see Measured Baseline below) and is not a
fault — it is the arm's kinematics. The residual tells you *that* the TCP is off
target, and by how much; diagnosing *why* is a separate step.

**Pass-through only.** It adds detail attributes and modifies no point data, so
it has zero effect on the deform stream feeding `bonedeform2`. This is why it can
sit inline rather than on a side branch.

Rationale: `configurejoints1`'s joint limits mean FBIK does not error on an
unreachable target — the solver simply stops tracking and the arm stalls at its
limit. Without an explicit residual measurement, an out-of-envelope curve is
invisible until export.

### `residual_guide` — `attribwrangle` (display-only side branch)
Reads the `tcp_offtarget` detail attribute and writes `@Cd` — red when set,
green otherwise. A wrangle rather than a `color` SOP because the colour is
conditional on a detail attribute, which the `color` SOP cannot express cleanly.
Not wired into the deform path; toggled by display flag when diagnosing.

## Controls

Promoted onto a `TCP_PATH_CTRL` null so there is a single place to animate.

| Parm | Type | Default | Notes |
|---|---|---|---|
Every parm is conditionally hidden so only the ones the active mode consumes are
visible. The `Visible when` column is the rule; verified across all four mode
combinations.

| Parm | Type | Default | Visible when | Notes |
|---|---|---|---|---|
| `goal_mode` | menu | curve | always | Manual (`rigpose_ik`) or Curve |
| `residual_tolerance` | float | `0.012` | always | Residual is measured in both goal modes |
| `progress` | float 0–1 | `fit($FF, $RFSTART, $RFEND, 0, 1)` | curve | Delete the expression to keyframe freely |
| `orient_mode` | menu | tangent | curve | Tangent / Aim At Target / Fixed Direction |
| `resample_length` | float | `0.05` | curve | Final even-length TCP spacing |
| `twist` | float | `0.0` | tangent | Roll about the tool axis → `orientalongcurve.roll` |
| `tool_yaw` | float −180..180 | `0.0` | tangent | Lead/lag off the tangent → `orientalongcurve.yaw` |
| `tool_pitch` | float −180..180 | `0.0` | tangent | Tilt off the tangent → `orientalongcurve.pitch` |
| `aim_target` | vector3 | `(0, 0, 0)` | aim | World position the tool points at |
| `up_vector` | vector3 | `(0, 1, 0)` | aim | Roll reference. **Only `orient_aim` reads this** — tangent mode uses `orientalongcurve`'s Curve Normal up type instead |
| `fixed_dir` | vector3 | `(0, -1, 0)` | fixed | World direction the tool axis points |
| `fixed_roll` | float −180..180 | `0.0` | fixed | Spin about that direction |

Conditionals use `HideWhen` with OR'd brace groups, e.g. `twist` is
`{ goal_mode == 0 } { orient_mode != 0 }`.

**Curve conditioning is deliberately not promoted.** `decimate_curve.length`
(0.03) and `fit_spline.tol` (0.005) are set directly on their nodes rather than
driven from `TCP_PATH_CTRL`. Good values depend entirely on how a given curve was
authored -- a clean CAD path wants a tolerance near zero, a freehand stroke wants
one larger than its tremor -- so promoting them implies a rig-wide setting that
does not exist. Tune them per curve on the nodes.

**Verification gotcha:** `hou.Parm.isHidden()` caches conditional state. Setting
the driving parm and reading `isHidden()` in the same script returns the previous
value. Call `node.updateParmStates()` in between, or verify across separate
round-trips.

Constant speed comes from `resample_arclen`; the keyframable `progress` sits on
top for dwells, easing, and reversals.

## Measured Baseline (2026-07-27)

Measured against the existing static `rigpose_ik` goal, before any changes.

- Requested TCP: `(0.4413, 0.6058, 0.4644)`
- Achieved TCP: `(0.4343, 0.5951, 0.4541)`
- Residual at shipped settings (`iterations=30`, `damping=0.5`): **16.8 mm**

Convergence sweep:

| iterations | damping | residual |
|---|---|---|
| 30 (current) | 0.5 | 16.8 mm |
| 200 | 0.5 | 9.9 mm |
| 500 | 0.1 | 9.9 mm |
| 2000 | 0.0 | 9.9 mm |

**Interpretation.** The residual converges to a hard floor of ~9.9 mm that no
iteration count or damping value improves. Roughly 7 mm is recoverable
non-convergence; the remaining ~9.9 mm is a position/orientation conflict against
the one-rotation-axis-per-joint constraint set in `configurejoints1`. The arm is
*not* out of reach — it is visibly folded well inside its envelope.

**Consequences:**

1. `residual_tolerance` defaults to **12 mm**, not 1 mm. A 1 mm tolerance would
   flag every frame, since the rig cannot achieve 1 mm with a fully constrained
   orientation goal.
2. `fullbodyik1.iterations` should be raised from 30 to ~200. This is a free
   ~7 mm accuracy gain. *Not changed by this work* — it alters existing rig
   behaviour and is the user's call.
3. `fullbodyik1` exposes `target_config_attribute = fbik_targetconfig`, a
   per-target dict controlling translation-vs-rotation weighting. Lowering the
   rotation weight would let position track more tightly at the cost of tool
   orientation. This is the correct lever if the ~9.9 mm floor matters for the
   real arm. See Out of Scope.

## Critical Convention: The Tool Axis Is +Y, Not +Z

On this rig a joint's **+Y axis runs down the bone toward its child**, so +Y is
the tool/flange axis. Verified empirically: the Y row of `joint_6`'s solved
`transform` equals the normalised `joint_5 → joint_6` direction to `dot == 1.000`.

The `transform` matrix3 stores its basis vectors as **rows**, confirmed by the
same measurement.

Both orientation branches must therefore drive **Y**. The first implementation
drove Z, which aimed an arbitrary perpendicular axis at the target. It produced
plausible-looking but badly degraded solves and led to an incorrect conclusion
(see the superseded finding below). Any future orientation work on this rig must
start from this convention.

### Tangent mode axis mapping

`orient_along_curve` emits three axis attributes. Measured against the raw curve
direction: `cz` dots 0.988–1.000 (**the tangent**), `cy` ~0.0–0.14 (up), `cx`
~0.0–0.08 (binormal). `orient_tangent` therefore maps:

| Robot axis | Source |
|---|---|
| **+Y** (tool axis) | `cz` — the curve tangent |
| +X | `cy` (up), orthogonalised against Y |
| +Z | cross product of the two |

Deflection off the tangent is controlled by three parms on `TCP_PATH_CTRL`, each
verified to rotate the tool axis by exactly its value: `twist` (roll about the
tool axis), `tool_yaw` (lead/lag), `tool_pitch` (tilt). Yaw and pitch deflect in
planes 86° apart, so they are independently useful.

## Superseded Finding: "Orientation Is The Binding Constraint" — WRONG

Measured on the built network against a 0.12 m radius horizontal test circle
centred at `(0.3, 0.5, 0.3)` — entirely inside the reach envelope.

Residual sampled at `progress` = 0.0, 0.2, 0.4, 0.6, 0.8:

| Orientation mode | iterations | residual (m) |
|---|---|---|
| **Position only** (no `transform` written) | 30 | 0.0098, 0.0100, 0.0092, 0.0096, 0.0090 |
| **Position only** | 300 | 0.0094, 0.0095, 0.0092, 0.0090, 0.0091 |
| Tangent | 30 | 0.496, 0.510, 0.176, 0.005, 0.006 |
| Tangent | 300 | 0.496, 0.509, 0.176, 0.006, 0.007 |
| Aim at origin | 30 | 0.320, 0.334, 0.052, 0.082, 0.213 |

**Position is reachable across the entire curve** at a flat ~9 mm. Every large
residual comes from the orientation demand, and a 10× iteration increase changes
nothing — the constraint is kinematic, not numerical.

The rig cannot hold an arbitrary tool orientation at an arbitrary position. With
one rotation axis per joint, demanding the flange axis track a curve tangent
requires wrist configurations the limits forbid, so FBIK sacrifices position.

**All of the above was an artefact of driving the wrong axis.** The measurements
were accurate; the interpretation was not. Aiming an arbitrary perpendicular axis
at a target is a far harder and essentially meaningless constraint, and that —
not the joint limits — produced the half-metre residuals.

### Corrected measurement (tool axis = +Y)

Same curve, same solver settings, `progress` = 0.0, 0.2, 0.4, 0.6, 0.8:

| Orientation mode | residual (m) | max |
|---|---|---|
| Position only (baseline) | 0.0115, 0.0106, 0.0092, 0.0090, 0.0100 | 12 mm |
| **Tangent** | 0.0090, 0.0395, 0.0204, 0.0076, 0.0095 | **40 mm** |
| **Aim at (0.9, 0, 0)** | 0.0162, 0.0086, 0.0032, 0.0210, 0.1038 | **104 mm** |

Orientation costs roughly 0–30 mm over the position-only baseline. Both modes are
usable as built. Aim mode points the tool within **1.45°** of the target.

`fbik_targetconfig` weighting is therefore **optional**, not required — worth
adding only if the residual needs to go below the ~10 mm solver floor.

FBIK itself is deterministic — verified identical (0.161 m) reaching the same
`progress` directly, ascending, and descending, and repeatable across cooks. The
solve is not path-dependent, so scrubbing and playback agree.

## Out of Scope

- **Joint-angle export to the physical arm.** Slots in after `fullbodyik1`
  without restructuring, because the goal is a point attribute rather than a set
  of parameter expressions. Likely routes through the existing
  `boning::robot_anim_by_csv::1.0` HDA.
- **Full-curve reachability scan.** Reporting *which stretches* of the curve are
  unreachable requires solving FBIK at N sample points — a for-loop with a solve
  inside, which is expensive. The per-frame residual covers the previz need.
- **Per-target translation/rotation weighting** via `fbik_targetconfig`. This is
  the lever that would reduce the ~9.9 mm orientation-conflict floor, but it
  trades tool orientation accuracy for position accuracy — a decision that
  depends on the actual operation and should be made deliberately.
- **Raising `fullbodyik1.iterations`.** Recommended (~7 mm free accuracy) but it
  changes existing rig behaviour, so it stays the user's call.
- **Collision checking** against obstacles or the arm against itself.
- **Singularity detection.** Wrist singularities produce valid-but-erratic joint
  motion that a position residual will not catch.

## Verification

1. Goal geometry stays **1 point, 0 prims**, `name` still `joint_6`.
2. `3@transform` is **orthonormal** — a non-orthonormal rotation matrix is the
   fast path to NaN output from `fullbodyik1`
   (`context/kinefx-patterns.md` gotcha 6).
3. `fullbodyik1` cooks without error at `progress` = 0.0, 0.25, 0.5, 0.75, 1.0.
4. `tcp_residual` is near zero for a curve known to be inside the envelope, and
   rises for one deliberately placed outside it.
5. Both `ORIENT_MODE` values produce distinct, non-flipping tool orientations
   across the full progress range.
6. `GOAL_MODE = 0` reproduces the current static pose exactly — proving the
   existing rig is untouched.
7. Viewport screenshots at 4+ frames confirming the arm visibly tracks the curve.

## Risks

| Risk | Mitigation |
|---|---|
| Curve leaves reachable envelope | `tcp_residual` / `tcp_offtarget` flags it explicitly |
| Tolerance set below the achievable floor → constant false flags | Default 12 mm, derived from the measured baseline rather than guessed |
| Tool flips mid-path | `orientalongcurve` parallel transport; verification step 5 |
| Non-orthonormal rotation → NaN | Verification step 2 |
| `resample_length` too coarse → jerky motion | Exposed as a parm; 5 cm default, tune against curve scale |
| Aim mode near-parallel to `up_vector` | Degenerate roll; document and expose `up_vector` |
| Existing rig regression | `GOAL_MODE = 0` path preserved; verification step 6 |

## Notes

- `$HIP` (`D:/WZ_Working/01_projects/uf850_houdini_rig`) is **not a git
  repository**, so this spec is unversioned. Consider `git init`.
- `rigdoctor2` is currently **bypassed** while feeding the rest-skeleton input of
  both `bonedeform2` and `capturepackedgeo1`, whereas the animated branch runs
  through `rigdoctor3` + `clear_eps` + `set_transform_rest`. Unrelated to this
  work, but it is the first place to look if the deform shows a subtle offset.
