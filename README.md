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

## Two selectors drive everything

**Goal Mode** — what the IK solver aims at:

| Mode | Source | Notes |
|---|---|---|
| Manual Rig Pose | `rigpose_ik` | pose the TCP joint directly |
| Curve | `CURVE_IN` → orient modes | tangent / aim / fixed orientation |
| Point Transform | `POINT_IN` | reads `transform`, else `orient`, else `N`+`up`, else the manual Fixed direction |

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

## Known issues

- **J4 exports past its limit on the current clip.** Wrapped, J4 stays inside
  ±180°, but continuity unwrapping accumulates it to −422° across 50 frames —
  beyond the ±360° the arm can reach. The exporter reports this as
  `50 limit warnings`; do not send such a clip to hardware. The pre-flight
  gate will make this a hard stop.

- FBIK enforces joint limits on J1/J2/J3/J5 but ignores them on J4. Rotation
  weights *do* bind on J4, so use J4 Roll Freedom to constrain it. Unexplained.
- J3 carries a +90° offset between the Configure Joints frame and the frame
  the analysis and CSV export report in.
- ~~Joint limits duplicated across three files~~ — resolved; `profiles/uf850.json`
  is now the single source and `scripts/robot_profile.py` the only place the
  frame conversion lives.
