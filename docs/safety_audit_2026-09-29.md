# Safety audit of the party show (v9, no tracking) -- 2026-09-29

Three read-only passes (build-time checks and the collision model; the
stream to the arm; the command surface and the Runner), each finding then
checked against the code. The motions themselves hold up: every segment is
collision-checked at build, a 125 Hz re-check of both compiled graphs
(party, party_bigwipe) found no violation, joins are held to 0.05 deg, and
limits (J6 cable range included) are enforced. The gaps are around them:
timing, stopping, the start move, and a few checks that look at the wrong
point. Nothing in the party code has been changed yet (below: what the
tracking mode already does differently).

## Status (2026-09-30)

Fixed, each with a test, and run on SimMachine (1 min of the party show:
at a hub, 0 slips, 1.2 ms late at most; stdin closed mid-run: stopped):
1 (one tick a point, the clock slips; the Guard checks one tick), 2 (any
exception takes the fault path; --stdin-control, which show_ui and
scan_test_ui pass: `stop` and the window's end stop, during the MoveJ
too; STOP between and during MoveJ waypoints, the robot enabled once;
the feedback connection times out and a stale feedback is a fault; a
hung send is caught by a watchdog on the feedback thread that sends
StopMotion within 0.5 s -- the 125 Hz sender itself keeps a blocking
socket: a socket timeout there cost 20-30 ms stalls every ~35 calls on
Windows, 198 slips in 44 s), 3 (playback.toml's IP only for its own
target; --sim only on 192.168.116.x, in show_stream, show_ui, track_*),
4 (the start move in the show's room with the paper at the scan's margin:
from a stopped scan it now detours -- unchecked it passed 2.0-2.7 cm
from the paper), and 7 (OSC commands only from this PC and the status's
hosts; one datagram at a time).

Then (2026-09-30): 8 (the build refuses a graph with gaps -- Graph.gaps:
every idle hub to every other, to and from the scan's hub, each
sequence's hub with clips; the Runner refuses a trigger with no route,
logged, never a crash or a jump), 6 (a scan asked while one runs or waits
is ignored; the run's end is latched -- Runner.end(): resume and triggers
after it do nothing; a trigger during an ordinary pause still plays, as
TD's Play relies on), 9 (the inputs hashed before the build reads them;
the robot's URDF and meshes hashed too -- a graph built before is not
held to an input it did not record). 5 and 10 are reported by the build
(WARNING lines, info["warnings"]), not refused: the v9 library plays on
the real arm; 11 greet clips' strip ends cross operator_slow at 1.8-5.4
x its 0.25 m/s, and the scan's LED point is 2 mm outside controller_zone
-- for the user to weigh (where the operator stands; which point the
controller watches). Rebuilt in memory, both shows pass the new checks --
but a rebuild does not reproduce the compiled v9 (up to 27 deg apart), so
v9 was left as it is; find out why before the next build.

## Fix before the next real-arm run

1. **A late tick becomes a jump the Guard lets through.**
   `show_stream.py` stream(): when the loop wakes k ticks late, the Runner
   advances k ticks and one ServoJ point goes out with cmdT = 8 ms -- k
   ticks of motion in one point (k x the clip's velocity). Guard.check
   scales its allowance by the ticks, so it never fires. Runs already skip
   (SimMachine: ~10 in 2 min; 43 ms late seen). At speed 0.3 a 40 ms stall
   during a fast part can ask several times the joint's speed for 8 ms.
   Fix: never advance more than one tick per point (the show clock slips;
   the lag correction takes it back at rests), and check each point
   against a one-tick step.
2. **A stop is not guaranteed on every path.**
   - Only StreamFault and Ctrl+C reach the fault path; any other exception
     (an XML-RPC socket error from servo_j, `/robot/trigger "scan abc"`
     under --scan-test, a missing route) skips StopMotion and the report.
     Fix: `except Exception` into the same path.
   - The window dying (show_ui killed) leaves show_stream streaming to
     --minutes: it never reads stdin while streaming. Fix: a stdin reader;
     its end = stop.
   - STOP during the MoveJ to the start hub: the next waypoint's `goto`
     re-enables and moves again. Fix: a stop flag the MoveJ loop checks
     between waypoints.
   - `fairino_player.Controller` has no XML-RPC timeout: a hung call blocks
     the loop (and the stop check). Fix: a transport timeout (~0.2 s) and
     a fault when the error poll is stale.
3. **`show_stream --sim` without --ip can drive the real arm.**
   `_toml_ip()` takes playback.toml's ip whatever its target; `--sim`
   never asks and plays at 1.0. (show_ui is safe: it reads the IP by
   target.) Fix: honour the target; refuse --sim outside 192.168.116.x.
4. **The move to the start hub ignores the paper.** It is checked against
   `envs/volvox_lab.usda` alone; the canvas lives in party.json. After a
   scan stopped mid-pass the MoveJ back could sweep through the paper.
   Fix: check it in `show_env(env, cfg, margins.scan_canvas_m)` --
   `fairino_player.move_checked(..., env=)` now takes that room (default
   unchanged).

## Next

5. **Slow zones are checked at the tool point only** (collision.py): in
   `greet_08_stretch` the LED strip's tips cross `operator_slow` at
   1.36 m/s (its limit 0.25 m/s) while the flange stays under. Fix: the
   strip's ends and the link capsules' ends too.
6. **The Runner's pause is soft.** A trigger ends a pause; `/robot/resume`
   (TD or show_ui) cancels the end of a run, which then goes on past
   --minutes; a second `scan` during the scan runs a second pass over the
   paper. Fix: latch the end; keep triggers pending while paused; ignore
   `scan` while one runs.
7. **OSC on 0.0.0.0, a thread a datagram, an unbounded queue**
   (show.py OscBridge): anyone on the LAN can trigger, resume or stop; a
   flood competes with the stream thread (feeding 1). Fix: 127.0.0.1 (TD
   is on the same PC) or an allowlist; BlockingOSCUDPServer; a cap per tick.
8. **The build does not require every hub to reach every other.** A missing
   route crashes the scan trigger (`None + [...]`) or queues a clip at
   another hub (the Guard would fault on the jump). All 12 moves exist
   today. Fix: fail the build without them.
9. **A graph can pass as fresh after an edit during its build** (inputs are
   hashed at the end; the Houdini show scene saves party.json). Fix: hash
   the bytes that were loaded, before building; add the URDF/meshes and a
   code version.
10. **Work zones are checked at the flange**, not the LED TCP: in bigwipe's
    `greet_wipe_cols` the LED TCP is 3.4 cm outside `controller_zone`.
    Matters if the controller watches its zone at the LED TCP.

## Worth knowing

- The guests are protected by the glass wall (partition_left, 0.05 m
  margin); keep-out zones get no margin and the audience box is not an
  obstacle.
- The scan's modelled paper clearance is ~5 cm: the real paper must match
  `canvas.center/normal` to that; nothing refuses a canvas marked
  `placeholder`.
- Nothing checks that the arm follows the commands (a controller that
  holds while returning 0 would get a backlog); the stream starts once the
  MoveJ is within 2 deg; ServoMoveEnd goes out while the arm is still
  behind; a missing env file leaves a MoveJ unchecked with a note only;
  stop-call errors are swallowed.

## The interactive mode (track_mode.py) already

- steps one tick per point: a late stream slips, never jumps;
- checks its start and return MoveJs against the room with the paper;
- stops when track_ui (its stdin) goes away; takes `end` / `stop` there;
- needs `--ip` and refuses SimMachine outside 192.168.116.x;
- latches its end (resume and triggers do nothing); plays no scan and no
  showpiece.
