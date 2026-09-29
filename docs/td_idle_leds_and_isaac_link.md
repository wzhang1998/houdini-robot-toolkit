# Idle LED animations, their Isaac preview, TouchDesigner <-> Isaac

Research, 2026-09-29 (second priority after the cable); built the same day -- see What was built.
The user wants TouchDesigner to play LED animations designed per clip while
the arm is not scanning, to preview them in Isaac Sim, the scan preview to
show TD's banana by default, and to know whether TD can drive Isaac's
run_show.

## What was built (2026-09-29, later that day)

The research below is kept as it was; what runs now differs from it:

- **Where the idle LEDs play**: not greet only. The show sends
  `/robot/paper` (show.paper_light: how much the strip lights the paper,
  a fraction of the scan's light square on at 6 cm) and TD's `idle_leds`
  plays in idle clips and moves (States `IDLE MOVE`, Hubs `*`) while it is
  under Paper Light Limit 0.001 -- greet, high and low always (at most
  0.0008 over party), rest (up to 0.024: it faces the paper) and moves
  towards it stay dark -- and while TD is IDLE or FADING (TD States; the
  minute after a scan: the paper gate keeps its image clean). Unknown
  paper light (-1) is dark.
- **The show's status** also carries `/robot/family`, `/action`,
  `/clip_t`, `/clip_len` (wall s), `/beat`, `/bpm_now` (the tempo played),
  `/facing` (the LEDs to the guests, 0..1).
- **TD** (TD-ROBOT-UVSCAN `td-modules/idle_leds`): four layers in a 60 x 1
  float TOP -- a breath over the clip, a comet with a trail driven by the
  arm's speed, a pulse on the beat, dimmed as the LEDs turn from the
  guests -- weighted per family by the `looks` table (then `move`, the
  Laban action, `default`), Contrast (a power curve), smoothed, then faded
  at the clip's ends, gated, Brightness (0-100 % of Master) on the CHOP
  side so a closed gate is exactly dark. pixel_scan takes the brighter of
  it and the scan; Enable LEDs stays the last gate. Its parameters: the
  TD repo's README.
- **Isaac, live** (not the shared pattern module proposed below: the
  patterns are designed in TD): `run_show.py --td-live` sends TD the
  status on 9002 and draws what TD sends back -- the strip's 60 LEDs
  (Art-Net, 127.0.0.1:6455; artnet.py, isaac/led_viz.py) and the canvas
  preview (canvas_link.py on 6457, isaac/canvas_viz.py). TD: pixel_scan >
  Output > Preview Art-Net IP 127.0.0.1.
- **Isaac, in a video**: `run_show.py --demo --td-live` records the run
  with TD's LEDs and canvas (td_capture.py); `replay_render.py` renders it
  at leisure. Without TD: `--demo` simulates the paper (canvas_model.py:
  pixel_scan and canvas_sim.glsl in numpy, the banana by default).
- **TD's pixel_scan Source**: Text / Pattern (the banana for now) /
  Capture (the Capture Image File).

## 1. What TD needs to animate per clip

- Sent now (show.py `Runner.status` -> `OscBridge.send`): `/robot/state`,
  `/robot/clip`, `/robot/hub`, `/robot/progress` (0..1 of the segment),
  `/robot/time_left`, `/robot/clip_energy`, `/robot/energy_now`,
  `/robot/sequence`, `/robot/next`, `/robot/joints`.
- Not sent: the clip's family, Laban action, tempo, time into the clip,
  length, beat phase. All 70 idle clips carry `action`, `bpm`, `energy`,
  `intent`; only the 58 gesture clips carry `family` (the 12 `rest_*`
  choreo clips have none) -- so TD's lookup goes family -> action -> a
  default.
- `labels.bpm` is the tempo a clip was *made* at, not played at (gesture
  clips are slowed: `params.slowed`, `tempo_factor`; `show_stream --speed`
  scales the clock; 0.3 on the real arm). A beat phase TD derives from bpm
  drifts: the player should send `/robot/beat` (phase 0..1) and
  `/robot/bpm_now`, the build keep `bpm_played`.
- Proposed additions: `/robot/family`, `/robot/action`, `/robot/clip_t`,
  `/robot/clip_len`, `/robot/beat`, `/robot/bpm_now`, and their names in
  TD's `robot_link/osc_contract.py` (TEXT / NUMBERS).
- TD side, in the project's plain style: a module `td-modules/idle_leds/`
  (a .tox and a tested .py): a Table DAT family/action -> pattern (chase,
  breathe, sparkle, wipe) and its parameters, energy scaling speed and
  depth, beat driving the phase; 60 samples 0..1, intensity only (the
  strip's R, G, B are the same 395 nm dies). Clips start and end at rest,
  so each clip fades itself in and out (~0.5 s); a dim pattern for MOVE.
  One Switch CHOP before the one DMX Out: SCANNING -> `pixel_scan`
  (always wins), else when allowed -> `idle_leds`, else zeros; Master and
  Enable LEDs stay the last gates.
- **UV and the paper.** docs/uv_scan_exposure.md says to light the strip
  only over the canvas; idle animations break that. Idle clips keep only
  `margins.idle_canvas_m` (0.15 m) from the paper, and a still strip darkens
  the paper within seconds. Limits proposed: idle LEDs only in TD's IDLE (maybe
  PAUSED), never in CAPTURE, SCANNING, FADING, FAULT, OFFLINE, STOP; a cap
  of ~10-15 % of Master; no strobe; a build label per clip (the strip's
  distance to the paper and whether it faces it, from `gestures.Rig().tool`)
  so clips near or facing the paper stay dark. The hood also cuts glare to
  the audience.

**Decided (the user, 2026-09-29): idle LEDs only in greet** -- the arm
turned to the guests lights up when it attends to them; every other clip
stays dark. Measured over party and party_bigwipe (the strip's ends and
middle against the paper's face, the LEDs' direction against its normal):

| hub | idle clips | nearest the paper | facing the paper |
|---|---|---|---|
| greet | 45 | 1.41 m | never |
| high | 36 | 0.76 m | never |
| low | 37 | 1.25 m | never |
| rest | 24 | 0.25 m | up to cos 0.24 |

So greet keeps the paper clean. It points the UV at the audience instead:
keep the brightness cap and no strobe, and the hood. TD's gate: `idle_leds`
on only while `/robot/hub` is greet and the state is IDLE (a move to or
from greet stays dark).

## 2. Previewing them in Isaac

- Recommended: **one pattern module both use** --
  `td-modules/idle_leds/led_patterns.py`, pure Python,
  `levels(labels, clip_t, clip_len, beat, energy) -> 60 floats`; TD runs it
  in a Script CHOP, Isaac imports it. The demo videos are recorded with
  `--no-osc`, headless, so they need the LEDs without TD; the module is
  testable. Cost ~1 day; the patterns are code, not LFO / Pattern CHOP
  networks.
- A cheap extra for checking TD against Isaac: a second DMX Out to
  127.0.0.1:6454 and ~40 lines of ArtDmx parsing in Isaac (needs TD
  running; no offline video).
- Drawing: `isaac/scan_viz.py` draws the lit strip as one violet curve.
  Per-LED colour: one BasisCurves of 60 two-point curves with `displayColor`
  at uniform interpolation (cheap, not glowing), or 60 small cubes with a
  per-frame `emissiveColor` (glows; 60 writes a frame).

## 3. The banana in the scan preview

- `C:\Program Files\Derivative\TouchDesigner\Samples\Map\Banana.tif`
  (1280 x 720 RGBA; versioned copies beside it) -- what TD's pixel_scan
  loads when its File is empty.
- In `ScanViz`: bind a UsdPreviewSurface + UsdUVTexture to the exposed
  quad and give it `primvars:st` -- top to bottom (0,1) (1,1) (1,1-u)
  (0,1-u) -- so the image appears row by row. Bake it once (PIL):
  alpha -> paper white, luminance -> violet, 60 px across (the strip's
  resolution). Its fit, flip and LED 0 end must follow pixel_scan's.
  Later: TD saves its frozen Cache TOP at the trigger and Isaac shows that.

## 4. Can TD drive Isaac's run_show?

- `run_show.py` makes `show.OscBridge(runner, 9000, "127.0.0.1", 9001)`
  from party.json's `osc` block: it hears TD's commands on 9000, but its
  status goes only to 9001, and TD listens on 9002 -- TD stays OFFLINE and
  ignores Trigger/Play there. So today: **no**.
- `/robot/stop` is mapped only when the runner has `stop()` (show_stream's
  streaming backend); Isaac's Runner has none.
- Clashes: only one process can hold 9000 (show_stream --osc, show.py osc
  or Isaac); a second one runs without OSC ("OSC off"). show_ui binds
  127.0.0.1:9001. Isaac with `--no-osc` is safe beside the live show.
- Change: `--osc-out HOST:PORT` on run_show (as show_stream has) passed as
  `also=`, and `/robot/stop` -> hold the runner. Then, with show_stream and
  show_ui closed:
  `C:/isaacsim6/python.bat scripts/isaac/run_show.py shows/party.json --osc-out 127.0.0.1:9002`
  Clear TD's Controller IP while previewing (its STOP also calls
  StopMotion on the real controller). Isaac's status runs on sim time; if
  it is slower than real time, TD's wall-clock timers (ack 3 s, watchdog)
  can misfire.

## Plan (as proposed; see What was built above for what ran)

1. Isaac <-> TD: `--osc-out`, `/robot/stop`. Done when TD leaves OFFLINE,
   its Trigger scans in Isaac, STOP holds it.
2. Clip data over OSC (family, action, clip_t, clip_len, beat, bpm_now;
   `bpm_played` and the strip-to-paper labels in the build; osc_contract).
   Done when the self-tests pass and the beat is right at --speed 0.3.
3. `led_patterns.py` and TD's `idle_leds` (patterns, table, the gating
   Switch, the cap). Done when dark in every non-IDLE state, SCAN wins.
4. Isaac's strip as 60 LEDs from `led_patterns`. Done when the no-OSC demo
   shows the patterns TD shows for the same clip.
5. The banana on the exposed quad. Done when a mid-scan snapshot shows the
   banana's top rows the way pixel_scan lays them.

Open: idle LEDs near the paper at all, or only in clips proven safe? The
pattern module in the TD repo? Code-defined patterns, or live TD design?
pixel_scan's fit for a non-square image?
