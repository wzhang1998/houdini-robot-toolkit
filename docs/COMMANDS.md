# Commands

The ones used day to day, from the repo root. `uv run` runs a script in the
project's `.venv`; Isaac Sim brings its own Python (`C:/isaacsim6/python.bat`).
SimMachine is `192.168.116.128`; the real arm's IP is in `playback.toml`.

## The show

```bash
uv run scripts/show.py build shows/party.json            # after changing clips or the config: every motion, checked (~2.5 min)
uv run scripts/show.py build shows/party.json --scan-only
uv run scripts/show.py dry-run shows/party.json --minutes 10 --triggers scan,low_wipe_rows
uv run scripts/show.py report shows/party.json            # the library as numbers
```

```bash
uv run scripts/show_ui.py                                 # THE window: target (SimMachine / arm), triggers, big wipes, STOP
uv run scripts/scan_test_ui.py                            # the scan step by step: line the strip up, try exposures
uv run scripts/rehearse.py shows/party_bigwipe.json --speed 0.3   # the rehearsal's steps and length (nothing moves)
uv run scripts/show.py build shows/party_bigwipe.json --scan-only --gap 0.035   # a scan 3.5 cm from the paper, to try
```

Scan distances to try: `build --scan-only --gap M` writes the scan alone at
another gap (the shade's rim to the paper) to `shows/scans/<show>_gap<mm>/`
(in git, a few seconds each, checked as the show's). scan_test_ui's
Distance picks one (show_stream `--scan-test --scan-from DIR`, refused when
built from other inputs than the show's now). The one chosen goes into the
show's config (`scan.led_gap_m`), then a full build.

Nearer than the show's margin to the paper (`margins.scan_canvas_m`) allows:
`--scan-margin M` gives that variant its own margin (show_stream's move to
the start hub takes the variant's). The gap is the bar's front to the paper:
the shade's rim is the CAD's front, where a tape measures from. The bar's
boxes are split narrow along its length so their capsules reach only 0.93 mm
past the rim (cad_tool_mesh's self-test: every CAD point inside, the front
within 1 mm); each build records the collision model's clearance too, and
the Distance list shows both ("front 0.4 cm (collision model 0.3 cm)").
The party's own scan is at 2 mm, its margin 0.8 mm (2026-10-01).

```bash
uv run scripts/show.py build shows/party.json --scan-only --gap 0.004                        # front 4 mm
uv run scripts/show.py build shows/party.json --scan-only --gap 0.002 --speed 0.5 --accel 1.3 --area-height 0.8
```

The wall (the canvas's centre and normal in the show configs) was measured by
laying the bar flat on it by hand and reading the joints (2026-10-01): the
bar's facing is the wall's normal; then a tape at the scan's start set its
distance (the pressed bar had sat 1.7 mm into it).

The scan plays at its built speed times the scan speed (show_ui / scan_test_ui's
"Scan speed", `--scan-speed`), whatever the show's speed: the arm's 0.3 slows
every other move, not the exposure (the party's scan: 2 mm from the paper).

The pigment fades in 10-20 s, so the party's scan is several passes:
`scan.passes` [0.25, 0.21, 0.18, 0.15] m/s runs down, up, down, up the
paper, each slower, the LEDs lit both ways (~26 s). Every pass has the
fastest's ramp, gentler the slower, so all turn at rest at the same two ends
(`show.scan_passes_profile`); `scan.accel_mps2` is that ramp's peak. The OSC
`/robot/scan/speed` is signed along u (negative on a pass back up), and
`/robot/scan` is the progress over all the passes. A `--speed` variant is
one pass at that speed.

Faster than the show's scan (the scan speed only slows a build down):
`--speed V --accel A` builds the variant at V m/s to `..._gap<mm>_v<cm/s>`,
its area lowered by its longer ramps so its top stays the show's (the
controller's Z 1600 mm cap). At the show's safety (0.5 of each joint's
limits) 0.4 m/s with 1.2 m/s2 is the most for the 1 m tall picture (the area
6.5 cm lower, J3 at ~34 % of its speed, ~44 % of its acceleration); 0.5 m/s
wants it 0.8 m tall (`--area-height 0.8`: J3 at ~40 % / ~48 %), else the
forearm comes by the floor at the bottom; 1.0 m/s would ask J3 for ~95 % of
its speed.

```bash
uv run scripts/show.py build shows/party.json --scan-only --gap 0.002 --speed 0.4 --accel 1.2
```


Rehearse (show_ui's button): every motion of the show once -- every idle
clip, every move between hubs, the scan -- then it ends at the start hub.
Walk the real arm through it slowly before the show runs on its own; after a
stop, "from step" goes on where it was. The window shows the run's time so
far and left (a show: to its Minutes; a rehearsal: to its end).

show_ui starts `show_stream.py` itself. By hand, SimMachine:

```bash
uv run scripts/show_stream.py shows/party.json --sim --ip 192.168.116.128 --osc
```

The real arm goes through show_ui (it asks first, speed 0.3). Keep a hand on
the E-stop.

## TouchDesigner (TD-ROBOT-UVSCAN)

`robot_uvscan_party.toe`. robot_link hears the show on 9002 and sends
commands to 9000. `python -m unittest discover -s tests` in that repo tests
the modules' logic.

## Isaac Sim

```bash
C:/isaacsim6/python.bat scripts/isaac/run_show.py shows/party.json                       # the show in the lab room, a window
C:/isaacsim6/python.bat scripts/isaac/run_show.py shows/party.json --td-live             # + TD live: its LEDs, canvas, ceiling drawn
C:/isaacsim6/python.bat scripts/isaac/run_show.py shows/party_bigwipe.json --demo        # the 5 min demo (paper simulated)
C:/isaacsim6/python.bat scripts/isaac/run_show.py shows/party_bigwipe.json --demo --td-live
C:/isaacsim6/python.bat scripts/isaac/replay_render.py geo/isaac/td_capture_<stamp> --camera audience
C:/isaacsim6/python.bat scripts/isaac/replay_render.py geo/isaac/td_capture_<stamp> --camera interact --size 3840x2160   # 4K
C:/isaacsim6/python.bat scripts/isaac/record_library.py --headless                       # the library review grid (v9)
```

`--guides` draws the safety zones (hidden by default). `--camera room |
audience | side | 'ex ey ez tx ty tz [focal]'`. Close show_stream,
scan_test and show_ui before `--td-live`. `--td-live` also draws the
tracked guests (TD tracking_test > Send to Isaac, /track/people on 9010) and
records them; replay_render draws them again. TD's tracking_test only runs
with its cook flag on (it is off so as not to take the Femto downstairs):
Source Sim plays a recording without the camera.

## The tracking test (apart from the party show)

Offline, no Isaac, no TD:

```bash
python scripts/track_eval.py                              # every scenario through the tracking layer, checked (22)
python scripts/track_eval.py mc_wave_call mc_crowd        # some
python scripts/track_eval.py rec:geo/tracking/femto_rec_<stamp>.csv       # a Femto recording from TD
python scripts/track_runner.py                            # the live loop (clips, gaze, interactive mode) on the mocap scenes
python scripts/track_runner.py mc_crowd --engage          # one, with a report
python scripts/mocap_scenes.py --write-all geo/tracking   # the simulated Femto recordings for TD
```

TD (`/project1/tracking_test`, the people_track module): Track > **Active**
on. Source **Sim** plays a recording (Recording: a `femto_sim_*.csv` or
your `femto_rec_*.csv`), **Femto** is the camera. It sends to Isaac (9010)
and SimMachine (9011). Record writes `femto_rec_<stamp>.csv`; Calibrate
solves the camera's placement (below). Active off: it does nothing.

Isaac, the people from TD:

```bash
C:/isaacsim6/python.bat scripts/isaac/run_tracking.py --live --osc-in 9010 --engage
C:/isaacsim6/python.bat scripts/isaac/run_tracking.py --live --osc-in 9010 --engage --td   # + TD's LEDs (pixel_scan > LEDs to Isaac on)
python scripts/track_replay.py geo/tracking/femto_rec_20260930-150731.csv --crouch 13-18   # a real recording to 9010, no TD
C:/isaacsim6/python.bat scripts/isaac/run_tracking.py --live --osc-in 9010 --engage --headless --minutes 1
C:/isaacsim6/python.bat scripts/isaac/run_tracking.py --live --engage                    # no TD: drag the head onto the ring
C:/isaacsim6/python.bat scripts/isaac/run_tracking.py --live --people 3                  # no TD: a crowd, whom it looks at
```

The interactive mode on its own (SimMachine or the real arm): the start
pose, the v9 library's idle clips at random (facing the guests less than
the show), somebody on the spot -> the arm comes to greet and turns to
them; End -> back to the start pose. TD people_track Active on (Send to
SimMachine / Robot). The LEDs play in TD as in the show (the window's LEDs
box, on by default: `--osc --osc-out 127.0.0.1:9002`); TD's Pause ends the
mode, its STOP stops.

```bash
python scripts/track_ui.py                                # the window: target, Start, End, STOP (real arm: red, every move confirmed)
python scripts/track_mode.py --sim --ip 192.168.116.128 --osc --osc-out 127.0.0.1:9002   # without the window
```

SimMachine, the older greet-only test (SimMachine only):

```bash
python scripts/track_test.py --sim --ip 192.168.116.128 --minutes 2 --engage
```

### Calibrating the Femto (once it is mounted)

1. TD: tracking_test > Track: Active on, Source Femto. Somebody is seen
   (the view shows them).
2. Calibrate: stand on floor mark 1 (the spot), Mark 1, Capture (1 s,
   stand still). Marks 2-4 the same: `left` and `right` 0.6 m either side
   along the zone, `glass` 0.35 m nearer the glass. The marks' robot
   coordinates are in the `marks` table (tape them on the floor from the
   robot's base).
3. Solve: writes `femto_extrinsic.json` (rms in the Result field: under
   ~3 cm is good) and uses it. `rec:` scenarios offline use it when it
   sits beside the recording.

Try it without the camera: Source Sim, Recording
`femto_sim_calibration.csv` (someone on each mark 4 s in turn: Capture at
~1.5, 5.5, 9.5, 13.5 s), Solve -- it comes back within ~2.5 cm of the sim's
own placement.

## Self-tests

```bash
python scripts/tracking.py && python scripts/engage.py && python scripts/track_sim.py
python scripts/femto_format.py && python scripts/mocap_cmu.py && python scripts/mocap_scenes.py
python scripts/track_osc.py && python scripts/track_runner.py && python scripts/track_test.py --self-test
python scripts/track_mode.py --self-test && python scripts/track_ui.py --self-test
uv run scripts/show_stream.py --self-test
uv run scripts/rehearse.py --self-test && uv run scripts/show_ui.py --self-test
```

The tracking scripts need numpy and ruckig: the system Python has them
(`uv run` lacks numpy; femto_format does without it).

## Ports

| Port | |
|---|---|
| 9000 | the show player's OSC in (TD's commands) |
| 9001 | show_ui |
| 9002 | TD's status in (from the show) |
| 9010 / 9011 | the tracking test's people: Isaac / SimMachine |
| 6455 | Isaac's LEDs (Art-Net from TD) |
| 6457 | Isaac's canvas preview (from TD) |
| 7000 | TD's agent bridge |
