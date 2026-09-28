# Real-robot test checklist (FR20) — 2026-09-25 version

Goals:
1. Confirm the whole chain is reliable on the real robot: Houdini → Pre-Flight → CSV → player → FR20.
2. Replace three assumptions in the software with measured data: the room, the acceleration limit, and whether the URDF and controller kinematics match.
3. Record the controller's current safety settings, to prepare for the next step, "controller safety settings as the runtime line of defense" (see `docs/standard_tools_eval.md` section 3).

**Before every step that moves, confirm three things: the workspace is clear, hand on the E-stop, and you know the WebUI global speed.**
In the table, 🟢 does not move, 🟡 small moves, 🔴 large moves.
All commands run from the repo root; replace `<IP>` with the real robot's IP.

**Film video (portfolio material)**: every time you run something new on the real robot **for the first time** (the first dance clip, HOME, wiggle, a clip refused by the collision check), set up the phone before you start:
- Landscape, 10-30 s, fixed camera position, the whole arm and base in frame, ideally with the screen or Houdini visible too;
- Then record a clip of the Houdini viewport from the same angle, so it can be edited side by side later;
- Name the file with date, clip, and speed, e.g. `20260925_d17_speed0.6.mp4`, and log it in the Media index of `docs/results.md`.

---

## 0. Preparation (does not move)

1. Run the self-tests; each one should print OK at the end:

```bash
uv run scripts/fairino_player.py --self-test
```

```bash
uv run scripts/collision.py
```

2. **Change the playback settings.** Your local `playback.toml` currently has `speed = 1.0` and `acc_limit = 300` — that's for SimMachine; do not use it on the real robot. Open `uv run scripts/play_ui.py` and change it in the UI to:

| Item | Hardware value | Reason |
|---|---|---|
| Target | Hardware | the UI shows a red warning |
| Controller IP | real robot IP | |
| Speed (0-1) | **0.3** | slow first; on hardware the UI asks you before every move |
| Acc cap | **0** | 0 = use the profile's measured values (J1–J3 300, J4–J6 600), matching Houdini's Retime and Pre-Flight |
| MoveJ % | **10** | for going to start and returning to HOME |
| Wiggle | J1, 3°, 4 s, 1 time | J6 turning 5° isn't visible |

3. **Photograph the WebUI safety settings for the record** (do not change them, just record):
   - Soft limits
   - Collision detection level and collision strategy
   - Speed limits
   - Safety walls, interference zones
   - Singularity protection
   - If there is a "safety parameter checksum," record that too

   Later the player will read these settings back to check them; if they don't match, it refuses to play.

## 1. Connection check 🟢

In play_ui press **1 Check**, or:

```bash
uv run scripts/fairino_player.py --check --hardware --ip <IP>
```

Check four things: model, error code is 0, current joint angles, **the FK vs URDF difference** (the difference between the controller's TCP and the TCP computed from our URDF).
On SimMachine this difference is 0.004 mm, but SimMachine is actually modeled on an FR5. **The real-robot number is the first real validation of the FR20 URDF.** If the difference is over 1 mm, stop and send me the output.

## 2. Return to HOME 🔴 (MoveJ)

HOME = `[0, -90, 90, -90, -90, 0]`: upper arm up, forearm forward, tool down, TCP about 0.85 m in front of the base and about 1.1 m high.
**Do not return to all-zero**: at all-zero the FR20 lies flat, with the TCP only about 8 cm from the base plate.
The player's path check is paused for this, so use MoveJ % 10 and watch closely.

In play_ui press **6 Go HOME**. Once it's in position, press **1 Check** again and record the FK difference at the HOME position.

**FK cross-check (recommended)**: use drag-teach to put the arm into 3 clearly different poses, press Check once per pose, and record the FK difference.
- 3 poses: reaching far out, close to the base, wrist rotated to a large angle.
- If the difference changes with pose, it means the URDF link lengths don't match the controller.

## 3. Wiggle 🟡

In play_ui press **4 Wiggle** (J1 3°), or:

```bash
uv run scripts/fairino_player.py --hardware --ip <IP> --wiggle 1 3 4 1
```

The whole arm will sway gently side to side. The log will show `wiggle: J1 moved 3.00 deg`. This step checks whether the ServoJ streaming path works: earlier, J6 at 5° wasn't visible moving on the real robot.

## 4. Measure the room 🟡 (drag-teach)

The room (`envs/volvox_lab.json` then; now `envs/volvox_lab.usda`) was estimated from photos, and the orientation is also assumed: it assumes the arm's straight-ahead direction (the direction the arm extends when J1=0) faces the TV wall. The Cell check in Houdini's Pre-Flight and the room shown by Show Cell are both based on this estimate.

1. Open drag-teach in the WebUI.
2. Open the point-probing window (read-only, will not move the arm):

```bash
uv run scripts/probe_ui.py
```

   Select the object (common ones are in the dropdown), touch the tool tip to the point, and press **Record point** or Enter.
   - The table lists each point along with "URDF vs controller mm": how much the TCP computed from our URDF differs from the controller's TCP. This is the FK cross-check, done incidentally at every point.
   - The line below shows how many points each object already has and how many more are needed.
   - The command-line version is still there too: `uv run scripts/probe_env.py --ip <IP> --tool-len 0.0`; type the name and press Enter, `u` to undo, `q` to quit.

| Name | Points |
|---|---|
| `floor:level` | 3 floor points, spread out; keep the flange face as level as possible (if tilted, the edge touches first and the reading is high) |
| `wall_tv:wall` | TV wall, 2–3 points spread out horizontally along the wall (fit as a vertical wall) |
| `partition_left:wall` | Left-side partition, same as above |
| `control_cart:box` | 4 corners of the red cart's tabletop |
| `operator:cylinder` | Floor at the operator's standing spot, 4–5 points around a circle |
| `stage:point` | A few corners of the performance area, reference only |

If a tool is attached, fill in the tool length (in meters) with `--tool-len`. For a more accurate result, first measure the TCP using the controller's tool calibration (4-point or 6-point), then fill in that length.

3. In the window, press **Preview fit** first to see what will change, then press **Write env** to write it (the old file is saved as `.bak`, and it lists how much each object moved). Command-line version:

```bash
uv run scripts/env_from_points.py envs/volvox_lab_points.json
```

4. In Houdini, open **Display > Cell** on robot_arm and check whether the room matches reality.
   - Zones and tall walls now have solid outlines, visible without selecting the node.
   - If the orientation is flipped (the TV wall ends up behind the arm), tell me and I'll fix the coordinate frame.

## 5. Measure the acceleration limit 🟡

All planning currently uses 150 deg/s², which is the manual's value for a 20–25 kg extended payload. Unloaded, it can very likely go higher. This number determines how "fast" a move can be:
- At 150, a large move cannot be fast; a "sudden" motion can only be a small 7–10° jab;
- At 450, the same punch segment raises TCP speed from 0.6 m/s to 1.3 m/s.

**First confirm the base is fixed** (Fairino manual: 6x M10 bolts, grade ≥ 8.8, torque ≥ 45 N·m, mounted on a rigid, resonance-free base; for the FR20, fixing it directly to the floor is recommended). Testing the large joints transmits reaction force into the base plate.

Test with the window (moves only one joint; it checks joint limits and the room before starting; on hardware it asks you before every level):

```bash
uv run scripts/accel_ui.py
```

First test J6, J5, J4 (3°, the wrist is mostly pose-independent), then test J3, J2, J1 at HOME (2°, results only apply to poses close to the test pose). Press **1 Check** first to see the pre-check, then press **2 Run levels**. Results are stored in `tests/accel/`. Command-line version:

```bash
uv run scripts/accel_probe.py --hardware --ip <IP> --joint 6 --amp 3 --report accel_j6.json
```

Record each joint's "clean up to" value. If there's shaking, unusual noise, or an error, stop — that level doesn't count. **Don't put these numbers into the profile yet — send them to me.**

## 6. Play clips 🔴

Play each clip at 0.3 → 0.6 → 1.0 speed, with Record and Report both checked.

**Procedure for each clip**:
1. Select the CSV in play_ui.
2. **2 Dry run**: `time_scale` should be 1.0; greater than 1 means the player will slow down.
3. **3 Go to start**.
4. **5 Play**.
5. After playback, press **6 Go HOME**.

| Order | Clip | Content |
|---|---|---|
| a | `tests/csv/fr20_test.csv` | Your curve, 18 s, previously run on the real robot at 0.3 |
| b | `tests/csv/dance_d01_punch-punch.csv` | Single move: punch, 14 s |
| c | `tests/csv/dance_d17_punch-float-punch.csv` | Contrast passage ABA, 16 s |
| d | `tests/csv/dance_d32_float-slash-press-float.csv` | Four-bar mix, 38 s |
| e | Curve from the current Houdini scene, newly exported | Verify today's Retime fix |

I dry-ran all four CSVs today; at 1.0, `time_scale` is 1.0 for all of them.

**How to do e**: In Houdini, press Retime → Pre-Flight (Robot playback should be OK) → Export, in that order. Then select this new CSV in play_ui and play it following the procedure above.

**Check three things in the report**:
- `tracking_after_lag_max_deg`: tracking error, smaller is better;
- `controller_error_after`: should be 0;
- whether `skipped` shows up.

## 6b. Quick-test dance clips (stage set) 🔴

`tests/csv/stage/` has 44 dance clips (generated by `scripts/stage_set.py`):
- All rotated −60° around the base as a whole, facing the center of the work area, all within ±90° of that direction;
- Each one has been rechecked against the room, all within the work area;
- **The start and end point for all of them is the same stage home** `[−60, −90, 90, −90, −90, 0]`, so switching between clips needs no movement.

Preview: `geo/review/overview_stage_*.mp4`, `contact_sheet_stage.png`.

**First time (only once)**:
1. In play_ui select `tests/csv/stage/_stage_home.csv`, **3 Go to start**, MoveJ 10%.
2. If coming from another pose (e.g. the inside_wall clip's pose), going straight there would flip the elbow over while vertical, leaving only 6 cm to the ceiling. The player now routes around this automatically: lower the upper arm first, then flip the elbow, then rotate over. The window will list the waypoints; confirm before it executes. The first time, watch closely how it moves, with your hand on the E-stop.
3. If it shows `REFUSED`: it means no safe route was found; jog it manually in the WebApp to a position close to stage home first.

**For every clip after that**: select CSV → **2 Dry run** → **5 Play** (the start point is the current position, no large move). Speed 0.3 → 0.6 → 1.0. Film video before starting, per the requirements in `docs/results.md`.

**All moves check the room**: before Go to start / Go HOME, the player checks the whole MoveJ path against `playback.toml`'s `robot.env` (default `envs/volvox_lab.usda`), keeping 0.30 m clearance from the ceiling and 0.10 m from other obstacles; if that's not enough it routes around, and if it can't route around it refuses.

## 6c. The show as one stream (show_stream.py) 🔴

The whole show streamed from the state machine. The arm idles through the
library, TouchDesigner can trigger sequences over OSC, and there is no CSV
per run. On SimMachine (2026-09-27): 3 min, 22,634 ServoJ sends, 0 skips,
no controller error, and tracking 0.54 deg max after a 40 ms lag. OSC
triggers and `/robot/stop` were tested too.

**Before**: `uv run scripts/show.py build shows/party.json` if the show
changed (it writes `shows/party.compiled.json`). Film it (see the top of
this file). Hand on the E-stop.

**With the window** (`uv run scripts/show_ui.py`), the easiest way:
1. Target **Real FR20** (the window turns red), the controller's IP.
2. Tick the checklist: area clear, hand on the E-stop, no alarm.
3. **Move to start**, at Move speed 10 % (3..30 %): a dialog shows where
   the arm is, the route (checked against the room) and the speed; Yes
   moves it, nothing else. STOP stops it mid-way (StopMotion straight to
   the controller).
4. Show speed 0.3 (the window allows up to 0.6), Minutes 3, tick the
   checklist again, **Start**. Two dialogs: the (now tiny) move to the
   start hub, then the plan. Trigger greet / scan, pause / resume from the
   window; STOP at the end.
5. Again at 0.6 for 5 min. TouchDesigner can join: it sends to the same
   port, and "Status also to" sends it the state.
Tried end to end on SimMachine through the same `--hardware` path
(2026-09-28): the move to the start hub at 8 %, the show at 0.3, STOP
during the show and during the move (the arm stopped at once, 115 deg
short of the hub).

**Or on the command line, in order** (each asks for `yes` on hardware; the
first move to the start hub goes through the checked route):
1. `uv run scripts/show_stream.py shows/party.json --hardware --ip <IP> --speed 0.3 --minutes 3`
2. The same at `--speed 0.6 --minutes 5`.
3. With TouchDesigner: add `--osc`.
   - Send `/robot/trigger greet`, then `/robot/trigger scan`.
   - Send `/robot/pause` and `/robot/resume`.
   - Last, send `/robot/stop`.

**Stopping**: Ctrl+C or OSC `/robot/stop` sends StopMotion at once (a
software stop). At the end of `--minutes` the running clip finishes at a
hub, at rest. A controller error or a joint step past the limits stops
the stream by itself (FAULT). None of this replaces the E-stop.

**The report** (printed, and `logs/stream/stream_<time>.json` with the
commanded and actual joints as CSV; the joints are written first, so a
stopped run keeps them):
- `pacing` should be `controller clock`: points are scheduled at the
  controller's ServoJ playback rate, `clock_ppm`, kept per controller IP in
  playback.toml `[controller_clock_ppm]`. The real FR20 plays ServoJ ~950
  ppm slower than the PC sends them (its own clock is only -126 ppm of
  that, 2026-09-28): paced by the PC's clock the lag grew ~1 ms per s.
  Start with `"<ip>" = -950.0`; after each run the report's
  `clock_ppm_suggested` (and a printed line) says what to set it to --
  `lag_ms_by_window` should then stay flat;
- `ended` should be `at a hub`;
- `controller_error` should be null;
- `skipped` should be 0 (with the queue pacing, a slow send no longer skips);
- look at `tracking_after_lag_max_deg`;
- `lag_ms_by_window` (one value per 3 min): if it grows run after run, the
  controller's clock drifts against this PC's (SimMachine: +28 ms in 30
  min). Note it for the long show.

Send me the JSON of each run; they go into `docs/results.md`.

## 7. Review the real-robot trajectory (does not move, Houdini)

The `*_actual_*.csv` produced by Record can be played back directly: select it in robot_arm's **Output > Import CSV** and press Import. Even a locked instance can import now; the file is read live.

What to look at:
- Where the real-robot trajectory differs from the designed curve;
- Which section lags the most.

## 8. OAK-D (if you brought it) 🟢

1. First tell me the camera model, and whether `pip install depthai` works.
2. I'll write the capture script with depthai plus MediaPipe; we won't build our own pose estimation.
3. Camera-to-arm-base calibration uses OpenCV ChArUco hand-eye calibration.
4. Data format follows `tests/keypoints/synthetic_float_punch_float.json`: 3D points for shoulder, elbow, wrist per frame, in meters.

---

## Do not do today

- **Do not test on the real robot whether "soft limits or interference zones can trigger during ServoJ playback."** Test that on SimMachine first.
- Do not change the WebUI safety settings — only photograph them for the record.
- Do not use `acc_limit = 300` or `speed = 1.0` for the first pass.

## Send me

| File or record | Purpose |
|---|---|
| Step 0's WebUI safety settings photos | Controller safety settings as the runtime line of defense |
| Steps 1–2 Check's FK difference (HOME + 3 poses) | URDF vs controller kinematics comparison |
| `envs/volvox_lab_points.json` | Measured room points |
| `accel_*.json` | New acceleration limits |
| `*_actual_*.csv` and report JSON | Tracking error, playback performance |
| Any error codes and what you were doing at the time | Troubleshooting |
