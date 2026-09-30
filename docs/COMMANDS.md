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
```

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
C:/isaacsim6/python.bat scripts/isaac/run_show.py shows/party.json --td-live             # + TD live: its LEDs and canvas drawn
C:/isaacsim6/python.bat scripts/isaac/run_show.py shows/party_bigwipe.json --demo        # the 5 min demo (paper simulated)
C:/isaacsim6/python.bat scripts/isaac/run_show.py shows/party_bigwipe.json --demo --td-live
C:/isaacsim6/python.bat scripts/isaac/replay_render.py geo/isaac/td_capture_<stamp> --camera audience
C:/isaacsim6/python.bat scripts/isaac/record_library.py --headless                       # the library review grid (v9)
```

`--guides` draws the safety zones (hidden by default). `--camera room |
audience | side | 'ex ey ez tx ty tz [focal]'`. Close show_stream,
scan_test and show_ui before `--td-live`.

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
C:/isaacsim6/python.bat scripts/isaac/run_tracking.py --live --osc-in 9010 --engage --headless --minutes 1
C:/isaacsim6/python.bat scripts/isaac/run_tracking.py --live --engage                    # no TD: drag the head onto the ring
C:/isaacsim6/python.bat scripts/isaac/run_tracking.py --live --people 3                  # no TD: a crowd, whom it looks at
```

The interactive mode on its own (SimMachine or the real arm): the start
pose, the v9 library's idle clips at random (facing the guests less than
the show), somebody on the spot -> the arm comes to greet and turns to
them; End -> back to the start pose. TD people_track Active on.

```bash
python scripts/track_ui.py                                # the window: target, Start, End, STOP (real arm: red, every move confirmed)
python scripts/track_mode.py --sim --ip 192.168.116.128   # without the window
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
