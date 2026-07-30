# houdini-robot-toolkit

Houdini toolset for animating a 6-axis robot arm — FK and IK, motion
analysis, and CSV export to a real controller.

Model-agnostic by design: the kinematic specification for a given arm lives
in `profiles/`, not in the assets. UF850 is the first profile.

## Layout

| Path | Contents |
|---|---|
| `otls/` | Digital assets (runtime binaries) |
| `profiles/` | Per-robot spec JSON — joint limits, axes, sign conventions, output format |
| `scripts/` | Python and VEX extracted from the binaries, in diffable form |
| `scenes/` | `.hiplc` scene files |
| `assets/fbx/` | Source geometry |
| `tests/csv/` | Reference fixtures for export/import validation |
| `docs/` | Design notes |
| `geo/` | IK solve cache — gitignored, regenerate with **Clear and Recache** |

## The asset

`wenyi::robot_arm::1.0` (`otls/sop_wenyi.robot_arm.1.0.hdalc`) wraps the whole
tool: 46 nodes, 78 parameters, six tabs following the workflow.

| Input | |
|---|---|
| 0 | Rest skeleton |
| 1 | Goal curve |
| 2 | Goal point — overrides the built-in target when connected |
| 3 | Collision — **reserved, unused** |

Tabs: **Setup · 1 Motion · 2 Solve · 3 Analyze · 4 Output · Advanced**.

An internal `TCP_PATH_CTRL` null sits beside the VEX nodes so every
`ch("../TCP_PATH_CTRL/...")` reference inside the wrangles keeps resolving
untouched; its parameters are channel-referenced to the asset's. Input
parameters flow *down* (asset is master); status strings flow *up* (the inner
node writes, the asset mirrors).

The solve cache name is a **raw string with backtick expressions**, not a parm
expression — `setExpression` is flattened to a literal when the definition is
saved, which made every new instance inherit one stale path and overwrite its
neighbour's cache.

## Two selectors drive everything

**Goal Mode** — what the IK solver aims at:

| Mode | Position from | Notes |
|---|---|---|
| Manual Rig Pose | Manual TCP Goal parms | direct translate/rotate on the TCP |
| Curve | `CURVE_IN` sampled at Progress | |
| Point Transform | `POINT_IN`, else the built-in target | external input overrides the built-in |

Position and orientation are **independent**. All three orient modes work with
either goal source:

| Orient Mode | Curve goal | Point goal |
|---|---|---|
| Follow | curve **tangent** | the point's own orient (`transform` → `orient` → `N`+`up`) |
| Aim At Target | point the tool at Aim Target | same |
| Fixed Direction | Fixed Tool Direction + Roll | same |

`Point Roll` adds spin about the tool axis on top of the point's orientation.

**Pose Source** — what actually drives the deformed robot *and* the CSV
exporter: FK (manual joints) / IK (solved) / Imported CSV / Baked IK→FK.

Manual FK is typed into `Joint_controller`'s `j1`–`j6` in the **robot frame**,
clamped to the profile's limits. The profile's sign is applied on the way to
the Rig Pose, so what you type is what the CSV exports.

`COLLISION_IN` is a reserved, unconnected wiring point. Collision avoidance is
not implemented.

## `$HIP` is not the project root

Scenes live in `scenes/`, so `$HIP` resolves to that folder. Project-relative
paths inside a scene use `$HIP/../`:

```
$HIP/../assets/fbx/uf850_fk_01.fbx
$HIP/../geo
$HIP/../tests/csv/test01_houdini.csv
```

`$JOB` would read better but needs environment setup to be reliable. `$HIP/..`
resolves correctly on a fresh clone with no configuration.

## scripts/ vs otls/

`otls/` holds what Houdini runs. `scripts/` holds the same code as text so it
can be reviewed and diffed — a change inside a `.hdalc` is otherwise invisible
in a commit. Treat `scripts/` as the readable copy, and keep it in sync when
the asset changes.

## Conventions

- All assets use the **`wenyi::`** namespace. `sop_vvox.robot_anim_by_csv.1.0.hdalc`
  (type `boning::robot_anim_by_csv::1.0`) is legacy and superseded by
  `wenyi::robot_anim_csv_io::1.0`.
- Houdini incremental saves (`backup/`, `otls/backup/`) are gitignored.

## Reading the analysis colours

**3 Analyze → Colour By** picks the metric; **Colour Scale** decides what red
means:

- **Profile Limits** (default) — red at a fixed value, so a colour means the
  same thing on every clip and takes are comparable. `vel_max` reds out at the
  profile's 180 °/s, `residual` at Red At Residual, and so on.
- **Percentile (5–95)** — spans this clip only. Used automatically for
  `flip_ratio` and `tcp_speed`, which have no absolute reference.

**Legend** states the scale in words, e.g.
`vel_max  blue 0 → red 180 deg/s  (profile limits)  actual range 0 .. 4264`.
Outliers pin to the ends of the ramp rather than being clipped out of the data.

Cache controls live on this tab, not on Solve: `cache_solve` reads
`POSE_SOURCE` and only the analysis chain consumes it — export and pre-flight
both read the live solve.

## Pre-flight gate

**4 Output → Run Pre-Flight Check** validates the clip and writes a report.
**Gate Export On Pre-Flight** is on by default and refuses to write a CSV that
fails. Checks run through the same `_collect()` the exporter uses, so the gate
validates exactly what ships.

| Check | Blocks? |
|---|---|
| Joint limits (on the unwrapped values that ship) | **FAIL** |
| Angle continuity — any step over 180° | **FAIL** |
| Unwrap enabled | **FAIL** |
| Joint velocity vs profile max | **FAIL** |
| Wrist branch resolved | warn |
| Frame range vs playbar | warn |
| Tracking residual vs tolerance | warn |
| Solve cache vs live solve | warn |

The cache check matters: the Analyze tab reads the cache while pre-flight and
export read live, so a stale cache means the numbers on screen describe a
different clip from the one about to ship.

**Quiet (no popup dialogs)** in Advanced suppresses modal dialogs.
`hou.ui.displayMessage` blocks Houdini's main thread until dismissed, which
deadlocks scripted and bridge-driven runs.

## Known issues

- **No orientation mode currently passes pre-flight on the drawn curve.**
  Measured on the same clip: Tangent hits 3307 °/s, Aim puts J6 at 627.6° and
  misses by 634 mm, Fixed puts J4 at −422.1° (62.1° past its limit) at frame
  225. The curve is more aggressive than the arm can follow as planned —
  slow it down, constrain J4 through Roll Freedom, or redraw.

- **Why J4 unwraps past its limit.** Extracted angles can only live in
  (−180, 180], but a controller needs continuous values: 179° followed by
  −179° reads as a −358° command and spins the joint backwards at speed. So
  the exporter unwraps, adding ±360° to preserve continuity — and that
  accumulates. J4 is a wrist roll joint, it is the one FBIK ignores limits on,
  and the flip resolver commits to whichever branch is nearest, so it can wind
  steadily in one direction until it runs past the ±360° the joint has.

- FBIK enforces joint limits on J1/J2/J3/J5 but ignores them on J4. Rotation
  weights *do* bind on J4, so use J4 Roll Freedom to constrain it. Unexplained.
- J3 carries a +90° offset between the Configure Joints frame and the frame
  the analysis and CSV export report in.
- ~~Joint limits duplicated across three files~~ — resolved; `profiles/uf850.json`
  is now the single source and `scripts/robot_profile.py` the only place the
  frame conversion lives.
