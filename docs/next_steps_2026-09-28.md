# Next steps after the strip day (2026-09-28)

The end-of-day report: where the show stands, seven questions the user asked,
the research behind the answers, and the decisions taken. Sources are linked;
our own estimates are marked. The UV exposure research has its own page:
[uv_scan_exposure.md](uv_scan_exposure.md).

## Where the show stands

- **Library v7** (9b6364e): 70 idle clips (rest 12 dances, greet 22, low 18,
  high 18 gestures), 28 families -- window and spin_sweep new, twirl swinging
  back and forth; the LED cable's J6 range (+-150 deg), a 0.30 m ceiling margin
  (a sprinkler), the canvas at 1.30 m behind its frame's rails.
- **The scan**: one pass, left to right as seen from the robot's side, at an
  even 0.2 m/s across the opening (the ramps over the rails), the LED face
  6 cm from the paper; in by backing off, sliding inward and laying the strip
  level with J6, out by backing off and turning the arm round upright.
- **Checked**: dry run 30 min (31 scans, 60 of 70 clips), SimMachine 3 min
  (0 skipped, 0.53 deg), Isaac 3 min (4 scans, no contacts; J2 peaked at
  3.5 deg of tracking -- watch it on the real arm).
- **Real arm** (the day's runs, paced at -950 ppm): the lag held at 104-128 ms
  over 10 min at 0.5 and 6 min at 1.0, path RMS 0.03-0.22 deg.

## 1. Before Wednesday (the LED strip arrives)

1. Measure the real frame: its centre, height, facing and how far the rails
   stand in front of the canvas; set them on the Houdini show's Zones page,
   rebuild. The scan is the motion most sensitive to it.
2. Real arm, no LED (a strip-sized stand-in): the library at 0.5 then 1.0;
   the scan with the window's Scan speed 0.1-0.25.
3. The controller's safety (question 5).
4. LED hardware: a 5 V supply of 4 A or more, a level shifter, the cable along
   the arm (J6 +-150). A photochromic swatch for the bench test.
5. Who drives the LEDs: the show sends the scan's progress over OSC, TD maps it
   to the image's column and brightness, compensating the fade across a slow
   scan.
6. Cleanup (question 7).

## 2. OAK-D and TouchDesigner

- TD has OAK operators: the OAK Device CHOP runs a DepthAI pipeline from a
  Python callback; OAK Select CHOP / TOP read its data and images
  ([OAK Device CHOP](https://docs.derivative.ca/OAK_Device_CHOP),
  [OAK-D in TD](https://derivative.ca/UserGuide/OAK-D)). Reported: TD freezes
  when hand tracking starts in the bundled examples (a DepthAI SDK bug)
  ([forum](https://forum.derivative.ca/t/td-crashing-when-using-handtracking-in-oakexamples-toe/598300));
  DepthAI v3 (Sept 2025) is not yet fully in TD
  ([forum](https://forum.derivative.ca/t/oak4-d-integration/729091)).
- Models: geaxgx's [hand tracker](https://github.com/geaxgx/depthai_hand_tracker)
  (Edge mode, palm XYZ), [BlazePose](https://github.com/geaxgx/depthai_blazepose),
  spatial detection networks (XYZ in mm). Latency: camera 7.5-33 ms over USB3,
  a detector ~80-105 ms end to end with queue size 1
  ([Luxonis](https://docs.luxonis.com/software/depthai/optimizing/)).
- **Recommendation**: DepthAI in its own Python process (a crash does not take
  TD or the show down; testable headless), USB, ~30 fps, queue 1. It sends
  `/track/target x y z conf t id` at 30 Hz (robot-base metres, after a one-time
  camera-to-robot calibration) and `/track/lost` to both TD and the show; TD
  shows and asks (`/robot/track/enable`), **the show decides**. Track the head
  first (face detection holds at 2-4 m); the hand as a gesture on top.

## 3. A tracking zone facing the audience

Feasible, in three steps, least risk first:

- **a. Presence steers the choreography**: people counted, their distance and
  activity pick the energy and the hub. No new motion.
- **b. Gaze on top of the clips**: a small, bounded joint offset turns the
  strip towards the person while a clip plays. No early exit needed.
- **c. TRACK**: the flange follows a head or hand inside a small box
  (~40 x 30 x 15 cm) on the wall's side, never towards the audience; each tick
  a One Euro filter ([1 Euro](https://gery.casiez.net/1euro/)), IK, and a
  Ruckig trajectory to the moving target ([Ruckig](https://github.com/pantor/ruckig));
  a behaviour layer: pick the most interesting person, look before moving, get
  bored, hold then search when the target is lost, back to the clips.

Precedents: Madeline Gannon's [Mimus](https://atonaton.com/mimus) (an ABB IRB
6700, ceiling depth sensors, no pre-planned moves, "most interesting person",
boredom), [Manus](https://atonaton.com/manus) (ten IRB 1200 sharing a
perception, look before move, never static), [Quipt](https://atonaton.com/quipt)
(follow, mirror, avoid); all behind glass or a barrier.

The latency end to end is ~200-270 ms (ours: camera and network 60-110, robot
110-130): looking at a person reads well, reaching for one does not. Safety:
the zone behind the barrier, sized by speed and separation (ISO/TS 15066, now
in ISO 10218:2025; people approach at 1.6-2 m/s).

## 4. Early exit

Needed for 3c, not for 3a or 3b. Today a trigger waits for the running clip
to end (dry run: 2.9 s mean, 9.3 s at most). Handing over to TRACK needs a
clip left mid-way: a jerk-limited stop (transitions.py has `exit_from` and
`ramp_stop`, not yet in the Runner), then a checked move to the tracking
start.

## 5. The controller's last guard

- The **interference zone** (WebApp: Application > Tool App > Interference
  Area) is not a safety function: its motion setting may be "continue" (a
  warning), a cube zone checks the TCP only, and the docs do not say it
  applies to SDK MoveJ / ServoJ
  ([3.7.8 manual](https://fairino-doc-en.readthedocs.io/3.7.8/CobotsManual/application.html)).
- Guards below the PC, strongest first:
  1. the hardware e-stop, and a light curtain / area scanner on the
     protective-stop input (dual channel, manual reset) -- they stop MoveJ and
     ServoJ alike ([safety](https://fairino-doc-en.readthedocs.io/3.7.8/CobotsManual/safety.html),
     [installation](https://fairino-doc-en.readthedocs.io/3.7.8/CobotsManual/installation.html));
  2. joint soft limits narrowed to the show (`SetLimitPositive/Negative`);
  3. safety planes (ceiling, walls, canvas) -- test on SimMachine whether they
     stop SDK motion;
  4. collision detection (`SetAnticollision`, level and `SetCollisionStrategy`)
     with the payload right ([SDK](https://fairino-doc-en.readthedocs.io/3.7.8/SDKManual/PythonRobotSecuritySettings.html));
  5. a low `SetSpeed` for the start move; firmware 3.9.4+ applies the safe
     speed to servo motion too ([versions](https://manual.fairino.support/latest/CobotsManual/version_intro.html)).
- Ours to add: the start move refused when the arm is outside an allowed
  start region (today it plans from wherever the arm is).
- Not to rely on: the interference zone, PC-side stops, SimMachine for safety
  I/O, collision detection as protection for people.

## 6. Planning: the day's lessons, and the tools

Lessons: a scan wants its entry, exit and process path planned together; a
detour search needs a cost (joint travel, an upright arm, clearance) -- "lying
on the floor" came from ranking by travel alone; analyses keep one time base;
the capsule model is standard (cuRobo and VAMP use spheres).

Tools (as of Sept 2026):

| Tool | Would replace | Windows |
|---|---|---|
| [Tesseract](https://github.com/tesseract-robotics/tesseract_python) (pip: Descartes, TrajOpt, OMPL) | the scan's IK per sample (Descartes samples the tool-axis freedom and the cable window), the approaches (TrajOpt) | native |
| [cuRoboV2](https://github.com/NVlabs/curobo) (Apache 2.0 since April 2026) | safe_move's via-pose grid, batch checks of clips | WSL2 |
| [Isaac Sim 6 cuMotion](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/cumotion/index.html) (experimental) | simulation checks, RMPflow | native |
| MoveIt 2 + [Pilz](https://moveit.picknik.ai/main/doc/how_to_guides/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html) | LIN approaches, blending | poor |
| [VAMP](https://github.com/KavrakiLab/vamp), Drake | free-space moves | no / WSL |

Recommendation: a `plan_freespace()` seam in front of safe_move; cuRobo
compared with safe_move on logged cases; Tesseract / Descartes tried on the
scan; our IK, Ruckig and TOPP kept. The industry's flow: process path ->
IK and redundancy -> LIN approach / retreat -> free-space planning between
taught poses -> timing (constant tool speed on the process, jerk-limited
elsewhere) -> digital-twin check -> controller program or stream.

## 7. Cleanup

The review of the day's commits found patches over bugs and duplication;
the plan is Stage 9 of [IMPLEMENTATION_PLAN.md](../IMPLEMENTATION_PLAN.md):
dead code and stale text; the pacing number named `playback_ppm`
(controller_clock's `--write` had put the clock's -126 where the playback's
-950 belongs); a compiled show that knows its inputs (the one in git was
stale); one canvas / fixture builder; one source for margins; the failing
self-tests (the strip in the collision model: choreo's wring, gestures from a
synthetic high hub, clip_library's fixtures); module splits last.

## Decisions (the user, 2026-09-28)

- Pushed (9b6364e). The strip stays 1 m long, 6 cm from the paper.
- Cleanup first, in Stage 9's order; module splits after Wednesday's LED test.
- mop on greet (it works there 8 of 10 draws; rest plays dances, not
  gestures).
