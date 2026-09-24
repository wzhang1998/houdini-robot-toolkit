# houdini-robot-toolkit

Houdini toolset for animating a 6-axis robot arm — FK and IK, motion
analysis, and CSV export to a real controller.

Model-agnostic by design: the kinematic specification for a given arm lives
in `profiles/`, not in the assets. Two profiles so far: **UF850** (FBX rig) and
**Fairino FR20** (built from the vendor URDF). The asset reads the profile when
it cooks, so a *locked* instance follows whichever robot its Robot Profile
names — see [Profiles drive the asset](#profiles-drive-the-asset).

## Layout

| Path | Contents |
|---|---|
| `otls/` | Digital assets (runtime binaries) |
| `profiles/` | Per-robot spec JSON — joint limits, axes, sign conventions, output format |
| `scripts/` | Python and VEX extracted from the binaries, in diffable form |
| `scenes/` | `.hiplc` scene files |
| `assets/fbx/` | Source geometry |
| `assets/fairino_description/` | FR20 URDF and link STLs, copied unmodified from FAIR-INNOVATION/frcobot_ros2 (`fairino_description/`), which declares no license; the asset reads `urdf/fairino20_v6.urdf` and `meshes/fairino20_v6/*.STL` |
| `tests/csv/` | Reference fixtures for export/import validation |
| `docs/` | Design notes |
| `geo/` | IK solve cache — gitignored, regenerate with **Clear and Recache** |

## The asset

`wenyi::robot_arm::1.0` (`otls/sop_wenyi.robot_arm.1.0.hdalc`) wraps the whole
tool: 110 internal nodes, six tabs following the workflow.

| Input | |
|---|---|
| 0 | Rest skeleton — optional override of the profile's own (FBX or URDF) skeleton; the mesh still follows the profile |
| 1 | Goal curve |
| 2 | Goal point — overrides the built-in target when connected |
| 3 | **Tool geometry** — display only |
| 4 | Collision — **reserved, unused** |

| Output | |
|---|---|
| 0 | Display |
| 1 | **Tool Tip** — one point: `P`, `transform`, `orient` |
| 2 | Analysis |
| 3 | Posed Skeleton — the KineFX skeleton the active Pose Source produces |

Outputs 2 and 3 existed inside the asset and were labelled in its dialog
script, but the definition allowed only two outputs, so neither was reachable.
It now allows four.

## Tools

**Setup → Tool** sets the tool frame relative to the flange face, tool axis
+Y. The face is `joint_6` plus the profile's `flange_offset_m` along +Y — zero
on UF850, where `joint_6` is the face; 0.12 m on FR20, whose last joint sits
behind it. The IK goal is moved *back* by that frame so the **tip** lands on the
goal, and the solver keeps targeting `joint_6` exactly as before — no tool
means a bit-identical no-op.

**Tool Frame** decides where the TCP comes from:

- **From Geometry** (default) — read out of the mesh on input 3, in order:
  a point group called `tcp`, else a point named `tcp`, else the centroid of
  the points furthest along +Y. Wire a tool and the arm reaches with it.
- **Manual** — the numeric Tool Offset / Rotate

The switch lives in one place: `TCP_PATH_CTRL/tool_offset` takes
`tool_tcp_probe`'s `tcp_p` under From Geometry with input 3 wired, else the
typed offset, and the goal offset, `TOOL_TIP` and `joint_angles` all read it.
Rotation stays manual — the probe finds a point, not a frame. Until this was
wired, be5f879's probe and status shipped but the solve still read only the
typed offset, so a wired tool moved nothing. Checked on FR20 with an off-axis
tip at (0.03, 0.15, -0.02) m: the solved tip sits 154.3 mm from the flange
face (the tool's length) and lands on the goal to within the solver's own
residual.

Tool Status names the source and says outright when a tip was *guessed* from
extent rather than declared, so add a `tcp` point when it matters.

The centroid, not a single furthest vertex: on a tube the far points form a
ring, so picking one put the tip a tool-radius off axis.

Output 1 is the **achieved** tip, not the goal — the two differ by the tracking
residual.

The **analysis follows the tip too**: the achieved path, TCP speed and the
whole Analyze tab measure the working point, not the flange. On a 150 mm tool
that is not cosmetic — peak TCP speed read 0.61 m/s at the flange against
1.22 m/s at the tip, so the flange reading understated it by half.

> Appending a rigid `tool_tip` joint and retargeting FBIK at it does **not**
> work. The joint stays rigid correctly, but FBIK will not solve to a goal on
> an appended joint: zero weights, zero limits and untouched config all left
> every joint at 0.0 with an 886 mm residual.

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

**Aim Target Source** is either Manual XYZ or **From Object**. The object may be
either:

- **SOP geometry** — aims at point 0 (an Add or Transform SOP works)
- **An OBJ node** — aims at its origin (a null you drag in the viewport)

Aim Object Status states which was resolved and how, so a silent zero is not
mistaken for a working target. Relative paths are resolved against the asset,
which is where you type them.

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

`joint_angles_sop.py` had drifted: the readable copy carried the "analysis
follows the tool tip" block (d916492) but the asset never did — that commit
changed a scene instance, not the definition. The asset now runs the copy.

## Profiles drive the asset

Until now three things inside `wenyi::robot_arm` were written for UF850 only,
although limits and presets already came from the profile: the axis each joint
turns about (J5 pinned to z) and literal J2/J4/J6 limits in `configurejoints1`;
the FK axis per joint and J3's sign as a literal `-1` in `rigpose_fk`; and
UF850's limits baked into the `fk_j1..6` slider templates as strict ranges.

They are now **expressions** on the internal nodes, evaluated against the
current profile by the asset's PythonModule (`scripts/hda/robot_arm_module.py`).
Not a callback that writes them: a locked instance forbids writes to internal
parameters, so a callback would only ever have worked on an unlocked copy.

What a callback still does — `on_profile_changed()`, run by the Robot Profile
menu and on creation — touches only promoted parameters: the `invert_jN`
toggles from the profile's sign, Robot Mesh on when the profile has a body
(FBX or URDF) and off when it has neither, and the Configuration presets via
`cfg_reset`. The menu lists every
`profiles/*.json`.

The FK sliders are now plain ±360 °; `Joint_controller` clamps each one to the
profile's limit.

**Solve cache path.** `cache_solve`'s basename was the literal
`ik_solve_uf850_robot_arm`, not a reference to `cache_name`, so every instance
in every scene shared one folder — and Recache deletes the matching files
before writing. It now follows `cache_name` (`ik_solve_<profile>_<node>`). An
instance named `robot_arm` on UF850 resolves to the same folder as before;
any other name gets its own and needs one recache.

**Checked against a baseline.** Before any of this, a fresh UF850 instance was
recorded over four FK poses and three IK goals (skeleton P/transform, tool tip,
residuals, joint config). After each change a fresh locked instance matched it
except for the cache path and J2's range, which moved from the literal ±132 to
the profile's ±131.9.

## Adding a robot from a URDF

`scripts/urdf_rig.py` builds the rest skeleton and places the link meshes from
a URDF. KineFX wants +Y down each bone; a URDF turns every joint about its own
local z, Z-up. So each joint frame is rebuilt from where the joints are and
which way each turns, and classified: `y` where the axis runs along the bone
(a twist), `z` where it is perpendicular (a hinge). A skewed joint raises
rather than producing a wrong rig.

`python scripts/urdf_rig.py` checks, for FR20: frames orthonormal and
right-handed, +Y down every bone at rest and posed, +25° in reads +25° out on
every joint, upper arm + forearm + J5→J6 = the datasheet's 1854 mm, and that
`profiles/fr20.json` states the same axes, signs, limits and flange offset the
geometry does.

The asset does this itself. When a profile names `rig.urdf` and no `rig.fbx`:

| Node | |
|---|---|
| `urdf_skeleton` | Python SOP — the rest skeleton, into `SKEL_SOURCE` (input 2) |
| `urdf_links` | Python SOP — one rigid STL per link at the URDF zero, prim `name` = the joint that moves it (`base` for the static link). STLs are reversed on load: their CCW winding points every face inward in Houdini |
| `urdf_normals` | vertex N, cusp 45° — CAD tessellation smears under point normals |
| `urdf_drive` | Transform Pieces — each link from `to_fk_ik` (rest) to `POSE_SOURCE` (posed) |
| `ROBOT_MESH_SOURCE` | switch — FBX skin (`out_robot`) or URDF links (`urdf_robot`), into Robot Mesh |

Both switches are Python expressions (`skel_source`, `mesh_source` in the
module), so a locked instance follows the profile. Mesh paths are
`package://` URIs, resolved the ROS way: the URDF sits at
`<package_root>/<package>/urdf/`. Nothing is wired outside — dropping the
asset and picking `fr20` is the whole setup. Input 0 still overrides the
skeleton when wired, but the links stay placed for the URDF's own rest, so
only wire one that matches it.

`scenes/FR20_rig.hiplc` is that: one locked `robot_arm`, profile `fr20`, no
inputs. On a fresh locked instance, FK matches URDF forward kinematics to
4 µm, every link-mesh vertex matches its STL placed by URDF FK to 5 µm, and
IK → extracted angles → URDF FK lands on the solved tool tip to 2 µm.

## Conventions

- All assets use the **`wenyi::`** namespace. `sop_vvox.robot_anim_by_csv.1.0.hdalc`
  (type `boning::robot_anim_by_csv::1.0`) is legacy and superseded by
  `wenyi::robot_anim_csv_io::1.0`.
- Houdini incremental saves (`backup/`, `otls/backup/`) are gitignored.

## Reading the analysis colours

**3 Analyze** is three collapsible groups in workflow order — **Cache** (recache
first, nothing below is valid until you do), **Colour** (read the result),
**Retime** (act on it). Cache location lives under Advanced → Cache Location,
since it is set once rather than used daily.

**Colour By** picks the metric; **Colour Scale** decides what red means:

- **Profile Limits** (default) — red at a fixed value, so a colour means the
  same thing on every clip and takes are comparable. `vel_max` reds out at the
  profile's 180 °/s, `residual` at Red At Residual, and so on.
- **Percentile (5–95)** — spans this clip only. Used automatically for
  `flip_ratio` and `tcp_speed`, which have no absolute reference.

Only the threshold belonging to the current metric is shown — pick velocity and
you see Red At Velocity, not four fields of which three are irrelevant.

### Colour is reserved for data

The metric ramp owns **blue → cyan → green → yellow → red**, and the bands own
green / amber / red. Nothing else is drawn in those colours, so a colour on
screen always means a measurement:

| | |
|---|---|
| Achieved path | the metric ramp — **the only thing that carries meaning** |
| Planned curve | white — a reference, not a measurement |
| TCP marker / Aim target | magenta / violet — controls |
| Residual ties | light neutral |
| Problem markers | hot pink — deliberately loud |
| Tool | steel grey |

The planned curve used to be cyan, which sat **0.12** from the ramp's cyan
stop — on a well-tracking clip both curves rendered nearly the same colour.
It is now 0.86 away.

**Legend** states the scale in words, e.g.
`vel_max  blue 0 → red 180 deg/s  (profile limits)  actual range 0 .. 4264`.
Outliers pin to the ends of the ramp rather than being clipped out of the data.

**Pass / Warn / Fail** replaces the ramp with three flat colours against the
profile limits — green fine, amber approaching, red over. A ramp shows
relative severity but never states where the line is; banding answers "where
does this clip break?" directly. **Warn At** sets the amber boundary as a
fraction of the limit.

**Problem Frames** markers sit on the path at the frames the last pre-flight
rejected. They come from pre-flight's own numbers rather than being recomputed,
because the analysis chain works on *wrapped* angles and cannot see unwrap
accumulation — J4 reaching −422° is invisible to `limit_margin`, since wrapped
it never leaves ±180. Consequence: markers are empty until **Run Pre-Flight
Check** has been pressed.

Cache controls live on this tab, not on Solve: `cache_solve` reads
`POSE_SOURCE` and only the analysis chain consumes it — export and pre-flight
both read the live solve. If the analysis looks frozen (identical values on
every frame, zero velocity), the cache is stale — recache.

Cache Directory, Cache Name and Cache Version are asset parameters, not
buried in the internal filecache. That is not cosmetic: **a locked asset
instance can read its internal parameters but not write them**, so anything a
callback has to change must be promoted. Cache Name defaults to
`ik_solve_`chs("robot_profile")`_`opname(".")`` — backticks evaluate at cook
time and `opname(".")` is the instance, so two arms in one scene never share a
cache directory.

## Pre-flight gate

**4 Output** is grouped as **Export CSV · Pre-Flight · Import CSV · Bake IK to
FK**.

**Run Pre-Flight Check** validates the clip and writes a report.
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

- **FBIK cannot use a range that crosses ±180° without being a full turn.**
  FR20's J2 and J4 ([−265, 85]) jammed the solver — J2 sat at 5° on every
  frame of the drawn curve, the tip missing by 200–800 mm. `fbik_range()` in
  the module now keeps such a range's part inside ±179° (−180 itself still
  failed a frame); full-turn ranges like UF850's ±360° pass unchanged. It
  also trims UF850's Shoulder *back* [90, 270] and Elbow *down* presets,
  which cross the same edge — not measured on UF850.
- **FR20 still misses a few frames of the drawn curve at the wrist flip.**
  After the range fix, 6 of 40 sampled frames miss by 190–520 mm, all where
  J5 passes through 0 while the solution switches wrist sign. Each frame
  solves from the URDF zero — itself fully stretched and singular. Pinning
  the Wrist preset to either sign made every frame miss (a range starting at
  0 puts the rest pose on its edge).
- **Progress** now defaults to `fit($FF, $RFSTART, $RFEND, 0, 1)`, what Reset
  Progress writes; it used to default to a flat 0, so a new instance on curve
  mode never moved unless someone had pressed Reset or Retime.
- FBIK enforces joint limits on J1/J2/J3/J5 but ignores them on J4. Rotation
  weights *do* bind on J4, so use J4 Roll Freedom to constrain it. Unexplained.
  Reproduced on FR20: an IK solve returned J4 = +95.67° against a configured
  [−265, 85]. FR20's other joints are not yet probed.
- FR20's URDF zero is assumed to equal the controller's zero (Fairino's own
  ROS 2 driver passes positions straight through). Not yet confirmed against
  SimMachine or hardware.
- ~~Recache left the old solve on screen~~ — resolved. On a **locked**
  instance, after `cache_solve` wrote, its load side kept serving the solve it
  had loaded before (same filenames, nothing dirtied): 222 mm off the new files
  on UF850, 647 mm on FR20, robot and debug path both. Unlocked instances were
  unaffected, which is why an unlocked UF850 scene never showed it. Recache
  now presses `cache_solve`'s Reload after writing; all four cases read the
  new files. The internal `TCP_PATH_CTRL/recache_btn` still carries an older
  copy of the script and is not exposed in the asset's UI.
- J3 carries a +90° offset between the Configure Joints frame and the frame
  the analysis and CSV export report in.
- ~~Joint limits duplicated across three files~~ — resolved; `profiles/uf850.json`
  is now the single source and `scripts/robot_profile.py` the only place the
  frame conversion lives.
