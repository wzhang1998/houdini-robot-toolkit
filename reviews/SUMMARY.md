# Clip review summary

Reviewed from static sheets (`scripts/review_render.py`) against `docs/review_rubric.md`; per clip: `reviews/<id>.json`, the sheet's numbers in `reviews/sheets/<id>.labels.json` (the PNGs are not in git: `scripts/review_render.py` renders them again).

| hub | keep | fix | drop |
|---|---|---|---|
| greet | 16 | 5 | 1 |
| high | 12 | 5 | 1 |
| low | 12 | 4 | 2 |
| rest | 6 | 2 | 4 |
| **all** | 46 | 16 | 8 |

Expressiveness (1 inert .. 5 alive, rubric 4):

| hub | mean | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| greet | 3.1 | 0 | 3 | 14 | 4 | 1 |
| high | 3.0 | 1 | 3 | 10 | 3 | 1 |
| low | 3.1 | 0 | 3 | 11 | 4 | 0 |
| rest | 2.5 | 2 | 4 | 4 | 2 | 0 |
| **all** | 3.0 | 3 | 13 | 39 | 13 | 2 |

Most often missing: phrasing 28, body_involvement 23, follow_through 16, dynamics 16, anticipation 13, overshoot 11, timing_variety 7

Verdicts the expressiveness pass changed: greet_21_figure8 keep -> fix

## Across the library

Source: `geo/rebuild/party` (`scripts/rebuild_library.py shows/party.json`: the same 85 segments as the tracked `shows/party.compiled.json`, sample for sample). 70 idle clips reviewed; the 8 hub-to-hub moves and the scan are not clips and were not reviewed. Four reviewers (one per hub) worked to the same rubric, followed by a consistency pass: the same family judged across hubs, and every "too_hard" note checked against the rubric's lines. Expressiveness (1-5) was scored in a second pass by two reviewers with shared anchors.

**Every judgement here comes from static sheets.** No confidence is above "high", and "high" only means the numbers and the picture agree. What a still cannot show is listed under each clip's live check.

### The whole library

- **Hub geometry, not the family code, causes most of the fixes.** low: the strip's lower end is pinned near the floor (`STRIP_FLOOR_M`), so downward families turn the strip instead of moving. high: the tool tip is 1.56 m from the shoulder (`REACH_M` 1.6) and 0.15 m under the controller's 1.6 m cap, with J5 at 26 deg, so vertical families flatten and `wrist_safe` damps the rolls. greet: the zone is 0.5 m deep, which clamps a reach. The suggested `hubs.<hub>.tcp` / `zones.<hub>` changes redraw every clip at that hub. Apply one at a time, rebuild, and run this review again. Only the high hub's new point was checked to solve (gestures.hub_pose, clear of the room); none was built.
- **salute fails at both hubs that draw it** (greet, high): an upright 1 m strip under `STRIP_TOP_M` leaves no room to raise it, and what remains duplicates the twirl. Both are drops; remove 'salute' from `hubs.greet.families` and `hubs.high.families`.
- **Where the tool has no room, the strip carries the motion**: the J6 turn of the strip becomes the biggest move (strip tips 2-4x the TCP's speed). This works for twirl and is the family's point there. Elsewhere it reads as a windmill (low nod and shy; the strip's turn into level in wipe, window, spin_sweep and mop at greet).
- **The pops' snap is a speed bell, not a dead stop**, at all three hubs: the family description asks for a stop. Check its timing live before changing anything.
- **No clip crosses the wrist (J5) or elbow (J3) singularity, and none reaches 90 % of a joint's acceleration.** The 8 hub-to-hub *moves* do pass the wrist singularity (build warnings, J5 0.02-0.35 deg). That matters on the arm (main code 10) but is outside this review.
- Safety items seen in passing, for the user and not judged here: the strip's ends move faster than `operator_slow` allows in greet_01 and greet_09-20 (build warnings); greet_13_pop's tool point is 5 mm outside the controller zone; the strip rides 0.1-10 mm from the forearm at the rest hub (self clearance).

### Expressiveness (rubric 4)

Scored 1-5 against what each action or family calls for, from the sheets' expression measures and the views. Two reviewers (rest + greet, low + high) used the same anchors: rest_04_glide 1, rest_07_dab 4, greet_14_twirl 4, greet_11_dive 5.

- **The rest phrases are mostly one shape**: a sink and a return, two speed bells, no breath at the bottom, and the wrist leading or level with the arm rather than trailing it (lag -0.10..0.07 s). That scores 3 at best. Only rest_10 (two bars of different character, with breaths) and rest_07 (wind-ups, syncopation) go above it. Knobs: free flow for float bars (`random_spec`), the follow-through part of choreo's `principles`.
- **At greet, the set-up is louder than the gesture**: in wipe, window, lissajous, mop and spin_sweep the fastest move is turning the strip level (or the way to the floor, or the unwind), not the figure, so the loudest moment is on the way in or out; rise's snap home is louder than the rise.
- **Gestures with a sudden character rarely overshoot**: peek, pop and sweep end on symmetric speed bells, and pop's snap is soft (dynamic range ~1.9 where it should stop dead). The gestures drawn at low intensity (reach 0.40, figure8 0.42, sweep 0.24) are the plainest: `library.intensity`'s floor is the knob.
- **Body involvement is what the strip families and the calm wrist gestures lack** (twirl, salute, tilt): fine for a twirl, and why salute reads as a weaker twirl.
- **low: body involvement is what lacks most.** Nod, shy, dive, tilt and rise are carried by the strip rolling (strip_ratio 2.4-4.1) while the body near the floor barely moves; the clips with room (look, peek, bounce, search) are the lively ones. The existing `hubs.low.tcp` z suggestion would help expressiveness as much as readability.
- **high: the 1.6 m cap and the edge of reach shrink tilt, trace, peek and search to the wrist alone** (1-3). Whole-arm, downward clips are the best at either hub: dive 5, sweep 4, rise 4.
- **Inverted climax**: reach, stretch, trace and pop are loudest on the way home, and the pops are symmetric bells (3 at every hub). Knobs: a gesture's `intensity`, the `TEMPO` constant (a return slower than the strike).
- **The strip families at low and high** (twirl, salute) are two alike swings at low intensity (k 0.27 / 0.39): 3, where greet_14_twirl's four swings of different speeds score 4. A higher `intensity` or room for more swings (`SPIN_W_DEG_S`, the length) would bring them up.
- **The score feeds the verdict little here**: every clip scoring 1-2 was already a fix or a drop for readability, except greet_21_figure8, which the pass moved from keep to fix (a fixed-orientation figure at intensity 0.42).

### rest (choreo phrases, 12 clips)

- **No strong actions.** `library.actions` is `[]` (all eight allowed), yet the rest hub has only float, glide, dab and flick. The build dropped 18 of 30 rest tries for "tool_strip within margin of canvas" (130-150 mm against `margins.idle_canvas_m` 0.15); the drop reasons don't name the action, so it is a guess that these were the whole-arm strong phrases. `library.actions` (it feeds only the choreo hub) is the knob to try: e.g. press with the light actions, then look at the drops. Loosening the canvas margin is a safety decision, not a looks one.
- **Most phrases are the same "sink and come back"**: 7 of 12 sink to 0.6-0.8 m and return home. The "high" level barely rises from this pose (0.04-0.15 m), so those clips nearly stand still. Knobs: the choreo spec's `levels` / `size` for light bars at this hub; `library.bars` [1, 2] -> [2] for more contrast within a phrase.
- **Float reads as float only with free flow**: flow +0.17..+0.91 measures float; the bound ones (-0.64, -0.83) measure glide or wring. `random_spec`: flow >= 0.1 for float bars.
- **Two-move glides start dead**: a 3-4 s "look" with the TCP still before the move. One-move glides don't have it: `bars[].moves` 1 for glide at this hub.
- **The strip rides ~7.5 mm from the forearm at the hub pose** (self clearance < 10 mm in every clip; 0.1 mm in rest_08). Not visible from the audience, but check it on the arm. The LEDs face the floor throughout (expected at this hub); all clips stay at 0.6-1.2 m, below a guest's eye height.

### greet (gestures and paths, 22 clips)

- **The strip's turn in and out is the hardest move** in the strip-turning clips (wipe, window, spin_sweep, a lissajous with strip_roll 90, mop): J6 at 76-82 % of its speed, the strip tips at 1.2-1.4 m/s while the TCP stands still. It frames a slower figure. The hub's strip lies diagonal, and these families want it level.
- **The audience is addressed well**: every head and figure clip faces the audience at 0.83-1.00 (mop 0.17, by design).
- **Height travel is flat**: search, sweep, peek, rise's gather and the wipe strokes move only 0.12-0.2 m vertically. The 1.28 m hub sits under the controller's 1.6 m cap and the strip's top limit (`STRIP_TOP_M`), which also removes salute's raise. Only dive and mop go far down.
- **Look-alikes**: twirl, salute and spin_sweep read the same from the audience (a face-on strip fan round the wrist); wipe and window use the same pane and level blade.
- Every greet clip starts 53-59 mm from the base plate (upper arm) at the hub pose itself: a hub-level fact, not a per-clip one.

### low (gestures, 18 clips)

- **The strip is pinned by the floor**: at the hub its lower end is ~0.15 m over the floor; in every downward move (nod, shy, dive, bounce, peek, tilt) that end stays at 0.10-0.18 m (`STRIP_FLOOR_M` 0.2 plus the checks) while the tool sinks, so the generator turns the strip instead: J6 ranges of 63-108 deg, the strip tips at 2-4x the tool's speed. Most "the strip windmills" readings here come from this. The knob: `hubs.low.tcp` z 0.55 -> ~0.70 m with `zones.low` centre z 0.62 -> ~0.77 (redraws every low clip; that the hub still solves there is unverified).
- **Families that go down suffer here** (shy, dive, scoop, nod: little room below); those that go up or across read well (wave, search, twirl, wipe, pop, rise).
- **The tool lives under the guests' eyes** (0.45-0.8 m; only reach and trace pass 1 m); the 1 m strip, reaching ~1.0-1.5 m, keeps the clips visible.
- **Look-alikes**: wave, search and sweep are all side-to-side swings at 0.5-0.7 m with the strip fanning.
- Twitch flags here are tiny (0.02-1.3 deg reversals): noted, not held against a clip; isolated accel spikes are live checks.

### high (gestures, 18 clips)

- **Little room to move**: the hub's tool tip is 1.56 m from the shoulder (`REACH_M` 1.6) and 0.15 m under the controller's 1.6 m TCP cap, which flattens the vertical families (stretch spans 0.06 m in z, rise 0.09 m over the hub, search 0.04 m, wave's "hand up" 0.12 m) and turns salute upside down (it sinks to stand the strip upright).
- **The wrist sits near its singularity**: the hub's J5 is 26 deg (greet's is 59-87), so `wrist_safe` pulls rolls and gazes back (tilt keeps 13 deg of the 34 drawn); look, trace and peek go down to 12.9-13.9 deg.
- One hub-level suggestion is behind most fixes here: `hubs.high.tcp` [-0.95, 0, 1.45] -> [-0.85, 0.15, 1.35], with `zones.high` moved the same way. Its hub pose was checked to solve and clear the room (the tip 1.43 m from the shoulder, 0.25 m under the cap, tilt keeping ~73 % of its roll); whether all 18 clips still draw there needs a rebuild. Each fix also gives a per-family fallback (remove the family from `hubs.high.families`).
- Reads well here: sweep, dive, twirl, bounce, wave.

## Fix (16)

### high_04_tilt  (high confidence; expression 1; twitch)

- expression 1 (lacks dynamics, body_involvement, phrasing): Near stillness: a 13 degree roll is nothing an audience can read as a head tilt.
- reads as: a slight strip roll: Short form (3.6 s, one side only): the TCP still (extent 0.05 x 0.04 x 0.03 m, meant small) but the strip turns only 13 deg (J6 range 12.6, J4 3; strip tips 0.30 m/s vs TCP 0.18): the strip ghosts in the audience-side view nearly overlap. The family draws a roll of 25 + 20 k (34 deg here); re-drawing tilt keys at this hub and passing them through gestures.wrist_safe keeps 0-65 % of it, because the hub's wrist is near its singularity. greet_04_tilt rolls 44 deg and reads; this one is a twitch of the strip from 2 m.
- twitch at t ~1.85-2.0 s (between ghosts 6-7): a J6 accel spike to ~50 % over a flat ~10 % neighbourhood, at the strip-tip speed bump of 0.12 m/s; no twitch counted (labels.flags)
- **change** `shows/party.json hubs.high.tcp (and zones.high centre with it)`: [-0.95, 0.0, 1.45] -> [-0.85, 0.15, 1.35] (metres; zones.high centre [-0.95, 0.0, 1.36] -> [-0.85, 0.15, 1.26]); every high clip is drawn again (the hub tool tip is 1.56 m from the shoulder against REACH_M 1.6 and 0.15 m under the controller's 1.6 m TCP cap, and its solved wrist sits near the singularity so wrist_safe turns rolls and gazes back. At the new point (checked with gestures.hub_pose and hub_clear: clear of the room) the tip is 1.43 m from the shoulder, 0.25 m under the cap, and wrist_safe keeps ~73 % of a tilt's drawn roll instead of ~35 %. The audience is 0.15 m further away.; scope library)
- **change** `shows/party.json hubs.high.families`: if the hub cannot move: remove 'tilt' (wrist_safe takes most of the roll at this hub; high_00_look already rolls the strip here; scope this family)
- live check: whether a 13 deg roll of the 1 m strip is visible at all in perspective
- sheet: `reviews/sheets/high_04_tilt.png`

### low_04_tilt  (high confidence; expression 3; too_hard, twitch)

- expression 3 (lacks body_involvement): Settles and uneven timing as greet_04_tilt, with a lot more light and shade (too hard for the calm one, see awkward); nothing below the wrist joins in.
- reads as: tilt: the tool holds and the strip rolls side to side, but swung hard: TCP nearly still (extent 0.19 x 0.06 x 0.07 m) while the strip rolls ~108 deg end to end (J6), strip tip 1.39 m/s vs TCP 0.39 at 2.2 s and 7.5 s, holds between: the head-tilt shape is there. But 'the calm one' is drawn at k 0.88 and bpm 133: J4 82 %, J6 82 % accel, J6 speed 77 %. greet_04_tilt (k 0.84) peaks at 65 %.
- too_hard at t 1.8-2.6 s and 7.0-7.8 s, ghosts 3-4 and 9-10: J4 82 %, J6 82 % accel (labels peak_acc_pct) in the calm family
- twitch at t 1.4-1.6 s, ghost 3: J6 accel spikes ~70 % on a 25 % speed bump before the main roll (accel timeline)
- **change** `shows/party.json library.intensity`: [0.2, 1.0] -> [0.2, 0.85] (tilt rolls (25 + 20 k) deg each way at the lively tempo; at k 0.88 the calm family's wrist reaches 82 % accel; every gesture's top intensity drops; scope library)
- **change** `shows/party.json hubs.low.tcp (z, metres) and zones.low center z`: hubs.low.tcp z 0.55 -> ~0.70 (zones.low center z 0.62 -> ~0.77); every low clip is redrawn (part of the 108 deg J6 range is the strip being rolled off the floor (lower end 0.10-0.11 m at 1.0-2.0 s); more height lets the tilt roll only as drawn; scope library)
- live check: whether the strip's swing looks violent at full size; whether the strip's roll on each dip reads as a deliberate head tilt or as the prop flailing (Houdini review video)
- sheet: `reviews/sheets/low_04_tilt.png`

### rest_03_float  (high confidence; expression 2; crowding, mechanical)

- expression 2 (lacks follow_through): A small, bound rise that stops in the middle: the float's drift and follow-through are missing from the body, left to the strip.
- reads as: a small lift 1.05 -> 1.20 m with a strip sway: a short glide, not a wandering float: Intent float, measured glide (0/1 agree; space +0.70 direct, flow -0.81 bound). One-move accent rising to the 'high' level, but from the rest pose it only rises 0.15 m (extent 0.02 x 0.08 x 0.15 m). The speed has a notch at ~1.4 s between two humps (TCP to ~0.01 m/s while the strip tip keeps 0.12 m/s): a stop-and-go in a sustained bar.
- mechanical at t ~1.4 s, between ghosts 3 and 4: TCP speed drops to ~0 between two humps inside the float bar (speed timeline)
- crowding at t 4.88 s: self clearance 7.7 mm forearm_link / tool_strip_bottom_b (hub pose level, not visible)
- **change** `choreo random_spec: flow for float bars`: draw flow from [0.1, 1.0] (free) instead of [-1, 1] when a bar is float (the floats with free flow read float (rest_00 +0.17, rest_09 +0.25, rest_11 +0.91); the bound ones misread: rest_03 flow -0.64 measured glide, rest_10 bar 1 flow -0.83 measured wring; scope this family)
- **change** `choreo phrase spec levels (float bars at the rest hub)`: draw 'low' (or levels False) for single-bar floats started at rest instead of 'high' ('high' from the rest pose rises only 0.15 m here (0.04 m in rest_04, rest_08), so the carriage that should make a float travel is missing; 'low' floats (rest_09, rest_11) travel 0.3-0.4 m and read float; scope this family)
- live check: whether the 1.4 s notch is visible on the arm or smoothed by the strip's follow-through; what the LEDs show while the strip faces down at this hub (idle_leds cues)
- sheet: `reviews/sheets/rest_03_float.png`

### greet_03_reach  (medium confidence; expression 2; mechanical)

- expression 2 (lacks phrasing, body_involvement, anticipation): The first offer is well shaped (lean back, out, settle) but the clip is mostly held and the offers are carried by the wrist, so it reads as waiting rather than offering.
- reads as: a small lean and wrist turn, weakly a reach: Reach drawn at size 0.6 (the third of SIZES) with intensity 0.40: the offer is 0.14 x 0.10 x 0.19 m (reach_m 0.22) and the motion is wrist-led (J6 has 41 % of the joint share); 60 % of the 9.1 s is still (still_fraction 0.60: holds t 2.2-5.2 s and 5.6-7.2 s). From 2 m the 'offer towards the audience' is a few centimetres; low_03_reach (size 0.8) moves 0.57 m.
- mechanical at t 2.2-5.2 s and 5.6-7.2 s (ghosts 3-8): two long holds; still_fraction 0.60 (geo clip labels.measured.raw); only J6 visibly moving in the two offers (J6 59 % speed vs J1/J2 13 %)
- **change** `shows/party.json zones.greet`: size [1.3, 0.5, 0.7] -> [1.3, 0.7, 0.7], or its centre moved ~0.1 m towards the audience (inside the stage line) (the hub's tool tip stands near the zone's middle, leaving ~0.2 m of the 0.5 m depth towards the audience: the reach out is clamped and redrawn small; scope this family)
- **change** `shows/party.json hubs.greet.families`: if the zone cannot grow: remove 'reach' (trace and dive already carry the body towards the audience at this hub) (a reach that cannot travel out reads as a wrist turn; scope this family)
- **change** `gestures.make intensity (reach at the greet hub)`: draw reach at intensity >= 0.6 (here 0.40) (expression 2, still 60 %: a calmer draw takes the longer form with longer holds; a livelier one shortens the holds and enlarges the offer; scope this clip)
- live check: whether the two offers read as offering anything from the audience zone, in perspective; whether the holds feel alive (breathing) or dead
- sheet: `reviews/sheets/greet_03_reach.png`

### greet_15_wipe  (medium confidence; expression 2; mechanical, twitch)

- expression 2 (lacks dynamics, overshoot, follow_through): One dynamic for 8 s of strokes, framed by louder set-up turns: the blade neither presses nor trails, so it reads as mechanical cleaning.
- reads as: a small level blade stepping down a pane (a weak wipe): Level blade on a pane facing the audience, 3 strokes top to bottom, but stroke_m 0.18 (the floor of the stroke length; the level blade's strokes are drawn at 0.7 of the upright ones, then size 0.8): TCP extent 0.21 x 0.15 x 0.16 m, strokes at ~0.3 m/s. The strokes and the lifts between have the same speed bumps, so the 'pressed on, lifted off' rhythm reads flat. The most visible motion is the strip turning to level at 0.2-1.4 s and back at 10.1-11.2 s (J6 81 % speed, strip tips 1.36 m/s). greet_18_window does the same pane, larger and better.
- mechanical at t 1.4-9.5 s (ghosts 2-9): strokes and lifts equal ~0.3 m/s bumps (timeline); stroke_m 0.18, step_m 0.124 (labels.params)
- twitch at t ~4.3 s and ~7.3 s (stroke starts): twitches J1 1, J5 1; J3/J4 accel spikes to ~70 % at the press-on of strokes 2 and 3
- **change** `gestures.make wipe stroke length`: the stroke's floor in pace() 0.18 m -> 0.25 m and the level blade's factor 0.7 -> 0.85 (every wipe changes) (a 0.18 m stroke is under what reads as wiping from 2 m; the level blade gets the shortest strokes; scope this family)
- **change** `shows/party.json hubs.greet.families`: if the strokes cannot grow at this hub: remove 'wipe' (greet_18_window covers the pane face on) (the same pane and blade, smaller; scope this family)
- live check: the LED strip's end exceeds the operator_slow zone's speed limit (build warning): a safety review item, check on the arm; whether the press-on / lift-off reads at full size; the strip's light against the imagined pane
- sheet: `reviews/sheets/greet_15_wipe.png`

### greet_17_spin_sweep  (medium confidence; expression 4; too_hard, twitch)

- expression 4 (lacks phrasing): Body and strip work together, with a wind-up, a growing crescendo and a brake; the phrase's loudest moment is the unwind on the way out, which spoils the ending.
- reads as: spin_sweep: TCP on an arc across 0.38 m (arc_m 0.395) with three strip swings along it (spin 94 deg), face on: reads. But after the brake the end is the hardest part: a J6 blip at 7.7-7.9 s (accel spike to ~80 % over ~30 % neighbours, strip tips 0.5 m/s in a hold) and the unwind at 8.4-9.5 s at J6 80 % speed / 82 % accel, strip tips 1.26 m/s, quicker than any swing, in a calm draw (k 0.33) and a sustained (press) bar.
- too_hard at t 8.4-9.5 s (ghosts 9-10, the unwind): J6 82 % accel in a press bar (labels.bars: 7.55-11.2 s press); strip tips 1.26 m/s
- twitch at t 7.7-7.9 s (ghost 9, the brake): isolated J6 accel spike to ~80 % at a 0.1 m/s TCP move
- **change** `gestures.make spin_sweep unwind`: the unwind key 1.0 beat -> 2.0 beats (every spin_sweep changes) (the unwind turns the strip back over the whole spin in one beat: the clip's hardest motion comes in its calm ending; scope this family)
- live check: the LED strip's end exceeds the operator_slow zone's speed limit (build warning): a safety review item, check on the arm; whether the brake at 7.7 s reads as a deliberate stop or a hitch
- sheet: `reviews/sheets/greet_17_spin_sweep.png`

### greet_19_mop  (medium confidence; expression 3; too_hard, twitch)

- expression 3 (lacks phrasing, anticipation): Body and strip work together in the strokes and the wring is a good accent, but the light and shade is inverted: the travel to and from the floor is louder than the mopping.
- reads as: mop: From the 1.28 m hub the TCP drops 0.7 m to z 0.57 for two wide strokes (stroke_m 0.64) with the head swivelling (swivel 42 deg): the mop itself reads. The way down (0.2-1.5 s) and back up (9.3-10.3 s) are the most violent motion at the hub: TCP 1.1-1.2 m/s, strip tips 2.5 m/s (twice any other greet clip), J1 79 %, J6 82 % accel, the last in a sustained (press) bar. MOP_HUB_TOP_M 1.35 lets the 1.28 m greet hub in, and the travel takes the 2 beats it is given.
- too_hard at t 0.2-1.5 s (ghosts 1-2) and 9.3-10.3 s (ghosts 9-10): J6 82 %, J1 79 % accel; strip tips 2.51 m/s, TCP 1.18 m/s (labels.tcp); the end is in a press bar
- twitch at t ~8-9 s (the wring): J6 twitch 1
- **change** `gestures.make mop way down / up`: the way down 2.0 beats -> 3.0 beats and the lift back as long (every mop changes) (the 0.7 m drop and lift in 2 beats drive the strip's end to 2.5 m/s, harder than anything else at the hub; scope this family)
- **change** `gestures.py MOP_HUB_TOP_M`: 1.35 -> 1.2 (the mop is then refused from the greet hub; it is already refused at low and high, so the family leaves the show) (only if the slower travel does not fit the clip: from 1.28 m the way down and back is most of the clip, as the constant's own comment warns; scope this family)
- live check: the LED strip's end exceeds the operator_slow zone's speed limit (build warning): a safety review item, check on the arm; the speed impression of the 2.5 m/s strip end swooping to the floor near guests; whether the low strokes are hidden by the paper or the room from the audience
- sheet: `reviews/sheets/greet_19_mop.png`

### greet_21_figure8  (medium confidence; expression 2; twitch)

- expression 2 (lacks follow_through, phrasing, body_involvement): A small eight traced like a plotter: the fixed orientation leaves the wrist and strip out and nothing breathes; it reads but it is the flattest clip at the hub.
- reads as: figure8: A lying eight in the audience plane, drawn twice, orient fixed (the strip stays upright and translates with it: J6 range 4 deg, strip tips = TCP 0.58 m/s). Legible in the audience-side view but small: 0.22 x 0.12 x 0.15 m at intensity 0.42.
- twitch at t ~2.8-3.1 s (ghosts 6-7, the crossing): J3 twitch 1; jagged J3 accel spikes to 81 % over ~30 % neighbours
- **change** `paths.make orient (figure8 at the greet hub)`: draw orient 'look' or 'tangent' instead of 'fixed' (expression 2: with 'fixed' the strip translates rigidly (strip_ratio 1.0, lag 0); in look / tangent the tool leans or looks and rolls with the figure's rhythm, so the strip trails the eight; scope this clip)
- **change** `paths.make intensity (figure8 at the greet hub)`: draw at intensity >= 0.6 (here 0.42) (extent (0.2-0.6 m) and speed (v ~ k^1.5) grow with intensity: a bigger eight with stronger surges than 0.22 m; scope this clip)
- live check: whether the J3 spikes at the crossing (t ~3 s) show as a hitch at the strip's ends; whether a 0.22 m eight reads from the audience zone
- sheet: `reviews/sheets/greet_21_figure8.png`

### high_03_reach  (medium confidence; expression 2; mechanical)

- expression 2 (lacks phrasing, anticipation): As greet_03_reach: the first offer is shaped, then the clip waits more than half its length and the retreat outshouts the offer.
- reads as: a lean out and down, weakly a reach: A tiny lean back (t 0.3-0.7 s, z to 1.47), then out towards the audience AND down (t 0.8-1.8 s: 0.19 m out, z 1.47 -> 1.28), a long hold (2.3-6.3 s) broken only by a second offer of a few cm (t 4.3 s, TCP 0.15 m/s), then home - the hardest move of the clip (t 6.5-7.0 s, J3 78 % accel). still_fraction 0.53. 'Out and up' is not possible from this hub: the tool tip is 1.56 m from the shoulder (REACH_M 1.6) and 0.15 m under the TCP cap, so the offer goes down; the 'offer again a little further' is not visible.
- mechanical at t 2.3-6.3 s (ghosts 4-9): a 4 s hold with one few-cm bump; still_fraction 0.53 (geo clip labels.measured.raw); the return home is faster than the offer (0.55 vs 0.46 m/s)
- **change** `shows/party.json hubs.high.tcp (and zones.high centre with it)`: [-0.95, 0.0, 1.45] -> [-0.85, 0.15, 1.35] (metres; zones.high centre [-0.95, 0.0, 1.36] -> [-0.85, 0.15, 1.26]); every high clip is drawn again (the hub tool tip is 1.56 m from the shoulder against REACH_M 1.6 and 0.15 m under the controller's 1.6 m TCP cap, and its solved wrist sits near the singularity so wrist_safe turns rolls and gazes back. At the new point (checked with gestures.hub_pose and hub_clear: clear of the room) the tip is 1.43 m from the shoulder, 0.25 m under the cap, and wrist_safe keeps ~73 % of a tilt's drawn roll instead of ~35 %. The audience is 0.15 m further away.; scope library)
- **change** `shows/party.json hubs.high.families`: if the hub cannot move: remove 'reach' (from the hub's edge of reach the offer can only go down, and dive / bounce already carry the body down at this hub; scope this family)
- live check: whether the offer reads as offering anything from the audience zone, in perspective; whether the long hold feels alive (breathing) or dead
- sheet: `reviews/sheets/high_03_reach.png`

### high_05_trace  (medium confidence; expression 2; mechanical)

- expression 2 (lacks follow_through, dynamics): greet_05_trace's loop shrunk to 0.14 m and 0.28 m/s: one soft level with nothing trailing, it barely registers.
- reads as: a small dip and back: Drawn at size 0.45 (the last of SIZES; slowed 1.29): one loop of extent 0.02 x 0.13 x 0.14 m drawn in the plane through the hub and the audience (a small triangle in the FRONT view), a still at 2.5-3.1 s, a separate return at 3.2-3.8 s. Not a 'loose, flowing path through its space': greet_05_trace draws 0.43 m. The draw shrank to its last size at this hub (edge of reach, TCP cap above).
- mechanical at t 2.5-3.1 s (ghosts 7-8), then 3.2-3.8 s: the loop stops, then a separate return (two TCP speed bumps 0.28 and 0.21 m/s with a dead stop between), not one flowing path
- **change** `shows/party.json hubs.high.tcp (and zones.high centre with it)`: [-0.95, 0.0, 1.45] -> [-0.85, 0.15, 1.35] (metres; zones.high centre [-0.95, 0.0, 1.36] -> [-0.85, 0.15, 1.26]); every high clip is drawn again (the hub tool tip is 1.56 m from the shoulder against REACH_M 1.6 and 0.15 m under the controller's 1.6 m TCP cap, and its solved wrist sits near the singularity so wrist_safe turns rolls and gazes back. At the new point (checked with gestures.hub_pose and hub_clear: clear of the room) the tip is 1.43 m from the shoulder, 0.25 m under the cap, and wrist_safe keeps ~73 % of a tilt's drawn roll instead of ~35 %. The audience is 0.15 m further away.; scope library)
- **change** `shows/party.json hubs.high.families`: if a redraw is still at size 0.45: remove 'trace' (a 14 cm trace reads as a twitch; high_13_sweep and high_12_dive carry the body through space at this hub; scope this family)
- live check: the wrist comes within ~13 deg of its singularity (min /J5/ 13.9 at t 2.7 s deg; the hub itself is at 26): whether the FR20 raises its singular-pose fault or the wrist visibly speeds up there
- sheet: `reviews/sheets/high_05_trace.png`

### high_08_stretch  (medium confidence; expression 3; twitch)

- expression 3 (lacks phrasing, body_involvement): The slow stretch with the wrist rolling has the yawn's feel and a wind-up, but it is small and the busiest part is the settle at the end, as low_08_stretch.
- reads as: a wrist roll and a small lean: No room above: z 1.40-1.46 (0.06 m), so the family's 'out and across' fallback: a 0.19 m lean (extent 0.11 x 0.19 x 0.06 m) with the wrist rolling widely (J4 range 47 deg, a fan of strip ghosts in the audience-side view). The 'big slow reach' is not big: the hub stands 1.56 m from the shoulder (REACH_M 1.6) and 0.15 m under the controller's 1.6 m TCP cap. The settle (t 4.6-4.9 s) is a quick wrist snap, not a settle.
- twitch at t 4.6-4.9 s (ghosts 8-9, the settle): twitches J2 1, J3 1 (labels.flags, under 2 deg); J6 accel spike to ~79 % over ~30 % neighbours, in a slow family (just under the 80 % line)
- **change** `shows/party.json hubs.high.tcp (and zones.high centre with it)`: [-0.95, 0.0, 1.45] -> [-0.85, 0.15, 1.35] (metres; zones.high centre [-0.95, 0.0, 1.36] -> [-0.85, 0.15, 1.26]); every high clip is drawn again (the hub tool tip is 1.56 m from the shoulder against REACH_M 1.6 and 0.15 m under the controller's 1.6 m TCP cap, and its solved wrist sits near the singularity so wrist_safe turns rolls and gazes back. At the new point (checked with gestures.hub_pose and hub_clear: clear of the room) the tip is 1.43 m from the shoulder, 0.25 m under the cap, and wrist_safe keeps ~73 % of a tilt's drawn roll instead of ~35 %. The audience is 0.15 m further away.; scope library)
- **change** `shows/party.json hubs.high.families`: if the hub cannot move: remove 'stretch' (a vertical family flattened by the TCP cap and the reach at this hub; high_11_rise and high_12_dive carry the vertical; scope this family)
- live check: whether the wrist snap at ~4.8 s reads as a jolt at the strip's ends; timing feel of the 'yawn'
- sheet: `reviews/sheets/high_08_stretch.png`

### high_16_wipe  (medium confidence; expression 2; mechanical, twitch)

- expression 2 (lacks dynamics, follow_through, overshoot): As greet_15_wipe: 7 s of one dynamic framed by louder set-up turns; pressing on and lifting off look alike and the blade does not trail.
- reads as: a level bar bobbing: Level blade, strokes top to bottom on a pane turned ~45 deg from the audience (pane_normal [0.70, -0.71, 0]), 3 strokes, stroke_m 0.219 drawn at size 0.6 (SIZES' third step): the TCP moves 0.14 x 0.13 x 0.14 m in all. From the audience the strip lies level and steps 13 cm up and down: a squeegee is not legible. Same pattern as greet_15_wipe, smaller.
- mechanical at t 1.4-9.0 s (ghosts 2-8): strokes and lifts as equal ~0.2-0.3 m/s bumps (timeline); stroke_m 0.219, step_m 0.111, size 0.6 (labels.params)
- twitch at t ~1.3, ~3.5, ~4.5 and ~6.5 s (press-on / lift-off between strokes): twitches J1 2, J5 2 (labels.flags) at 3.5 and 6.5 s; J3 accel spikes to ~75-81 % (labels: J3 81 %) standing out of ~30 % neighbours
- **change** `shows/party.json hubs.high.tcp (and zones.high centre with it)`: [-0.95, 0.0, 1.45] -> [-0.85, 0.15, 1.35] (metres; zones.high centre [-0.95, 0.0, 1.36] -> [-0.85, 0.15, 1.26]); every high clip is drawn again (the hub tool tip is 1.56 m from the shoulder against REACH_M 1.6 and 0.15 m under the controller's 1.6 m TCP cap, and its solved wrist sits near the singularity so wrist_safe turns rolls and gazes back. At the new point (checked with gestures.hub_pose and hub_clear: clear of the room) the tip is 1.43 m from the shoulder, 0.25 m under the cap, and wrist_safe keeps ~73 % of a tilt's drawn roll instead of ~35 %. The audience is 0.15 m further away.; scope library)
- **change** `gestures.make wipe stroke length`: the stroke's floor in pace() 0.18 m -> 0.25 m and the level blade's factor 0.7 -> 0.85 (every wipe changes) (as greet_15_wipe: the level blade gets the shortest strokes; here shrunk again by size 0.6; scope this family)
- **change** `shows/party.json hubs.high.families`: if the strokes cannot grow at this hub: remove 'wipe' (a 14 cm wipe on a turned pane does not read as wiping from the audience; scope this family)
- live check: whether the press-on / lift-off reads at full size; the strip's light against the imagined pane
- sheet: `reviews/sheets/high_16_wipe.png`

### low_02_nod  (medium confidence; expression 2; strip_orientation, twitch)

- expression 2 (lacks body_involvement, phrasing): One dip in which the strip windmills while the body barely moves: the gaze leading is nice, but there is no dying-away nod-nod and the prop carries it.
- reads as: a small dip with the strip rolling ~60 deg each way: more a head tilt than a nod: The TCP dips 0.13 m (0.59 -> 0.46 m at 1.5-2.2 s, back by 3.0 s): a nod is meant small. But on each dip the strip windmills (J6 range 63 deg, joint share J6 0.54; strip tip 1.25 m/s vs TCP 0.30; AUDIENCE SIDE ghosts 4-7 go from upright to near level). The strip's lower end stays 0.12-0.18 m over the floor through the dip while the TCP drops 0.13 m: the roll is the strip being turned off the floor (strip_safe), not the gaze. The only clearly 'nod' cue is the TCP dip; the second, smaller nod (3.4-3.6 s) is a 0.13 m/s blip.
- strip_orientation at t 1.5-3.0 s, ghosts 4-7: J6 range 63 deg, strip tip 1.25 m/s vs TCP 0.30 m/s; strip lower end held at 0.12-0.18 m (show.strip_ends) during the dip
- twitch at t 3.4-3.6 s, ghost 8: isolated J5 accel spike 62 % (J4 ~50 %) against ~10 % speed, on a 0.13 m/s blip
- **change** `shows/party.json hubs.low.tcp (z, metres) and zones.low center z`: hubs.low.tcp z 0.55 -> ~0.70 (zones.low center z 0.62 -> ~0.77); every low clip is redrawn (at TCP 0.55 m the strip's lower end sits 0.15 m over the floor, so every dip at this hub is paid for by rolling the strip; 15 cm more under it lets the nod dip without the roll (also nod, shy, dive, bounce, scoop here); scope library)
- **change** `shows/party.json hubs.low.families`: remove 'nod' if hubs.low.tcp stays at 0.55 m (greet_02_nod reads as a nod; from this hub a dip always rolls the strip; scope this family)
- live check: whether the strip's roll on each dip reads as a deliberate head tilt or as the prop flailing (Houdini review video)
- sheet: `reviews/sheets/low_02_nod.png`

### low_08_stretch  (medium confidence; expression 3; too_hard)

- expression 3 (lacks phrasing): It has the dip before and the settle after, but the drop is louder than the stretch and the top is hardly held: the arc of a yawn is inverted.
- reads as: a quick lift and a quicker drop: a pop-up more than a yawn: Short form (4.0 s) at k 0.21: rises 0.55 -> 0.78 m (0.5-1.5 s, 0.55 m/s), a 0.4 s hold, then the settle 2.0-2.7 s is the fastest move of the clip (0.70 m/s; J2 80 % accel at 2.5 s). Nothing goes out or across (extent 0.04 x 0.07 x 0.26 m), the wrist roll is small (J6 range 24). A stretch is 'a big slow reach up and out ... then settles': here the settle is a drop.
- too_hard at t 2.1-2.7 s, ghosts 7-8 (the settle): J2 80 % accel, TCP 0.70 m/s, the peak of the clip, in a calm (k 0.21) stretch
- **change** `gestures.py clip_length (the always-long families tuple: wipe, broom, salute, twirl, spin_sweep)`: add 'stretch' so it is always drawn in the long form (the short form gives the settle 1.5 beats instead of 2 and the yawn 2 beats instead of 3; at 116 bpm the settle becomes the quickest, hardest move; scope this family)
- live check: whether the rise reads as a yawn at full speed; the settle's weight on the arm (J2 at 80 %)
- sheet: `reviews/sheets/low_08_stretch.png`

### rest_08_flick  (medium confidence; expression 2; crowding, twitch)

- expression 2 (lacks anticipation): The flick's ingredients are there (a release, the strip trailing, a settle) but at 3 cm it is a tremor, and it starts with no counter-move.
- reads as: a barely visible strip tremor: the flick's curl is a few degrees: 1/1 agree (flick, time +0.99, space -1.00), but tiny: TCP extent 0.02 x 0.015 x 0.035 m, peak 0.19 m/s, strip tip 0.28 m/s, J4 range 1.1 deg, J6 6.5 deg, show energy 0.01. The 'high' level gave 0.035 m of rise. A flick should be a small but visible curling flourish; here ghosts 1-10 overlap in every view. J2 accel 48 % with J2 speed 6 % at 0.3-0.5 s (jagged accel curves on all joints 0.2-1.0 s).
- crowding at t 1.125 s, ghost 5: self clearance 0.1 mm forearm_link / tool_strip_bottom_b: the strip touching the forearm at the wrist (not visible from the audience)
- twitch at t 0.25-0.5 s, ghosts 2-3: J2 accel spikes 25 % and 48 % against a 6 % speed (accel timeline); twitches flag 0, so possibly interpolation
- **change** `choreo phrase spec size (flick accents)`: size >= 1.2 for one-move flick bars (here 0.80) (at 0.8 the flick's wrist curl is a few degrees (strip tip 0.28 m/s, energy 0.01); the dab at 1.08 (rest_07) reaches 1.09 m/s at the strip tip and reads; scope this family)
- **change** `choreo phrase spec levels (flick at the rest hub)`: draw 'low' (or levels False) instead of 'high' for flicks from rest ('high' gives only 0.035 m of rise from this pose, so the flick has no carriage under it; scope this family)
- live check: whether the flick is visible at all at full size; the J2 accel spikes at 0.3-0.5 s on the arm (jitter or interpolation); the 0.1 mm strip-to-forearm gap on the real arm (strip mount, cable)
- sheet: `reviews/sheets/rest_08_flick.png`

### low_17_scoop  (low confidence; expression 3; twitch)

- expression 3 (lacks anticipation, body_involvement): The toss is placed as the climax and the blade levels through the scoop, but without the dig there is no gathering down before it: half a scoop.
- reads as: a strip turn and a lift with a toss: the dig is missing: The TCP only goes up: 0.55 -> 0.66 m through 'takes it up' and 'tips it down, digs in' (0-2.2 s), scoops 0.66 -> 0.78 m, the toss at 5.6-6.2 s (0.68 m/s), settles. The dig's lower end is clamped: end_z 0.2 = STRIP_FLOOR_M (the hub's tool tip at 0.55 m less the 0.35-0.65 m depth is under it), so the strip's lower end only reaches 0.15-0.24 m and the tool never dips. dig_deg 29.1 (the steep design set). The only scoop in the library (greet and high made none).
- twitch at t 4.0-4.2 s, ghost 6: isolated J5 accel spike ~65 % (neighbours < 30 %); twitches J2 1, J4 1 (reversals under 0.6 deg)
- **change** `shows/party.json hubs.low.tcp (z, metres) and zones.low center z`: hubs.low.tcp z 0.55 -> ~0.70 (zones.low center z 0.62 -> ~0.77); every low clip is redrawn (the dig is floor-bound: from 0.70 m the strip's lower end can go 0.5 m under the tool tip before STRIP_FLOOR_M, so the tool tips down and digs instead of only lifting; scope library)
- live check: whether the strip reads as a shovel in profile in 3D; the toss at 5.8 s
- sheet: `reviews/sheets/low_17_scoop.png`

## Drop (8)

### rest_04_glide  (high confidence; expression 1; mechanical)

- expression 1 (lacks dynamics, phrasing, body_involvement): Near stillness: nothing to express.
- reads as: near-stillness: the wrist turns the strip a little, the TCP moves 4 cm in 9.7 s: Labelled glide 1/1, but there is nothing to see: TCP extent 0.02 x 0.04 x 0.04 m, peak 0.053 m/s, z 1.09-1.13 m, show energy 0.00, wrist share 0.80. The 'high' level gave no rise from rest, size 0.475 and tempo 'rit' kept both travel_near moves tiny. A glide should be a smooth straight travel; this reads as idling. rest_06_glide covers glide at this hub.
- mechanical at whole clip, ghosts 1-10 overlap: TCP extent 0.04 m, peak 0.053 m/s; ghosts nearly coincide in all three views
- **change** `choreo phrase spec levels / size (glide bars at the rest hub)`: for light bars from rest draw level 'low' (not 'high') and size >= 0.8 ('high' gives 0.035 m of rise from this pose and size 0.475 keeps the wrist-led travels at 2-4 cm; the 'low' glide rest_06 at size 0.6 sinks 0.39 m and reads; scope this family)
- replace with: rest_06_glide
- live check: whether the small strip tilt reads at all as light on the paper (idle_leds)
- sheet: `reviews/sheets/rest_04_glide.png`

### greet_16_salute  (medium confidence; expression 3; crowding, twitch)

- expression 3 (lacks body_involvement, anticipation): The strip swings with braking and some timing variety, but the salute's character (raise, hold, bow) needs the body and it never moves: a weaker twirl.
- reads as: a second twirl (strip swung face on round a still wrist): The TCP hardly moves (extent 0.07 x 0.07 x 0.06 m, z 1.23-1.29): the sabre 'raised' and the 'bow' are only the strip turning. The rise (gestures.make: up to 0.3 m) is clamped because an upright 1 m strip from a 1.28 m tool tip already has its top at ~1.8 m (STRIP_TOP_M 1.84). What remains is four face-on strip swings (J6 range 101 deg, strip tips 1.25 m/s vs TCP 0.26): from the audience it looks like greet_14_twirl, which does it better.
- twitch at t ~1.9 s (ghost 3, the held salute): J4 twitch 1; J5 accel spike to ~70 % while the other joints are still, in the hold
- crowding at t 3.83 s (ghost 4): the strip's top 57 mm from partition_left (labels.room)
- **change** `shows/party.json hubs.greet.families`: remove 'salute' (from this hub's height the upright strip leaves no room to raise it, so the salute loses its raise and bow and duplicates the twirl; scope this family)
- replace with: greet_14_twirl
- live check: the LED strip's end exceeds the operator_slow zone's speed limit (build warning): a safety review item, check on the arm; whether the upright hold at ~2 s reads as a salute at all in perspective
- sheet: `reviews/sheets/greet_16_salute.png`

### high_17_salute  (medium confidence; expression 3; crowding, too_hard)

- expression 3 (lacks body_involvement, timing_variety): Holds and strip swings with braking give it some shape, as greet_16_salute, but the raise sinks, the two swings are alike and the body sits out the flourish: a weaker twirl.
- reads as: a dip with the strip stood upright, then strip swings: To stand the 1 m strip upright under STRIP_TOP_M 1.84 the TCP has to go DOWN 0.28 m (z 1.45 -> 1.17, t 0.2-2.8 s, ghosts 2-4): the 'raise' reads as a sink. The held sabre (2.8-3.7 s) is there, the 'cut' flourish is two strip swings (t 3.7-5.0 and 5.3-6.4 s, J6 range 119 deg, 82 % speed, strip tips 1.35 m/s vs TCP 0.05) - from the audience the same face-on fan as high_15_twirl, which does it bigger (140 deg) - and the last move (t 7.4-8.3 s) goes UP back to the hub, the opposite of a bow. From this hub the family's shape is inverted by the strip's top limit.
- too_hard at t ~0.6 s (ghost 1-2, the wind-up): J4 80.8 % accel (labels) in a press (sustained) bar: over the 80 % line
- crowding at t 5.42 s (ghost 8): self clearance 9.9 mm, forearm_link to tool_strip_bottom_b (labels.room): the strip's end passing the forearm in the flourish, a note
- **change** `shows/party.json hubs.high.families`: remove 'salute' (from a 1.45 m tool tip an upright 1 m strip fits under STRIP_TOP_M only by sinking, so the raise and the bow invert; the flourish duplicates high_15_twirl (as greet_16_salute at the greet hub); scope this family)
- replace with: high_15_twirl
- live check: whether the upright hold at ~3 s reads as a salute at all in perspective; the strip's end passing ~1 cm from the forearm at ~5.4 s, seen live
- sheet: `reviews/sheets/high_17_salute.png`

### low_07_shy  (medium confidence; expression 2; strip_orientation)

- expression 2 (lacks body_involvement, phrasing): The slower return is the hesitation greet_07_shy lacks, but the retreat is the prop turning and half the clip is held: little to read as shyness.
- reads as: a 0.1 m sink with the strip rolling, a long hold, back: the shrinking away does not read: Drawn at size 0.6 (SIZES' fallback). The 'shrink back and down' is 0.56 -> 0.46 m (1.8-2.2 s), extent 0.14 x 0.06 x 0.10 m; the visible event is the strip rolling ~70 deg (J6, strip tip 1.04 m/s vs TCP 0.35) because its lower end is already at 0.11-0.15 m over the floor. Then a 2.6 s hold at 0.46 m (2.2-4.8 s) and a slow return. LEDs face the audience 0.99: the gaze never visibly drops away. From this hub there is no room down, which is the family's essence; greet_07_shy shrinks 0.28 m and drops its gaze (facing 0.85).
- strip_orientation at t 1.6-2.6 s, ghosts 3-4: J6 range 70 deg, strip tip 1.04 m/s vs TCP 0.35 m/s; strip lower end 0.11-0.13 m (floor) while the TCP sinks
- **change** `shows/party.json hubs.low.families`: remove 'shy' (shy shrinks back and down; at TCP 0.55 m with the strip's lower end 0.15 m over the floor it can only sink 0.1 m and roll the strip (size already cut to 0.6); greet_07_shy covers the family; scope this family)
- replace with: greet_07_shy
- live check: whether the gaze turning away (30-37 deg down) is visible at all from the audience
- sheet: `reviews/sheets/low_07_shy.png`

### low_12_dive  (medium confidence; expression 2; strip_orientation, twitch)

- expression 2 (lacks anticipation, body_involvement, phrasing): A floor-bound dip: no wind-up, the descent carried by the strip and the burst on the way up, so none of greet_11_dive's swoop survives here.
- reads as: a slow sink with the strip rolling, then a quick rise: the dip reads backwards: The swoop down is 0.55 -> 0.41 m over 0.6-2.2 s (TCP 0.2 m/s, strip tip 1.0 m/s: mostly a strip roll), then the rise 'back up past the hub' 2.3-3.0 s is the fastest move (0.66 m/s, J2 77 %, J6 76 % accel). A bird's dip should be the fast part. Depth is floor-bound: the strip's lower end stays 0.12-0.15 m over the floor while the TCP sinks. greet_11_dive dives 0.77 m and reads.
- strip_orientation at t 0.6-1.6 s, ghosts 2-4: J6 range 64 deg, strip tip 1.0 m/s vs TCP 0.2 m/s on the descent; strip lower end 0.12 m
- twitch at t 3.2-3.5 s, ghost 7: twitches J4 1 (1.3 deg reversal) on a 0.15 m/s blip after the rise
- **change** `shows/party.json hubs.low.families`: remove 'dive' (a dive needs height to swoop from; at TCP 0.55 m the descent is 0.14 m and floor-bound, so the rise is the accent; greet_11_dive covers the family; scope this family)
- replace with: greet_11_dive
- sheet: `reviews/sheets/low_12_dive.png`

### rest_01_glide  (medium confidence; expression 2; mechanical)

- expression 2 (lacks phrasing): The glide itself is smooth and even as it should be, but a third of the clip is a dead wait before it: nothing to watch, then one move.
- reads as: a 3.2 s near-still 'look', then a straight glide sink 1.10 -> 0.74 m and home: 1/1 agree (glide, space +0.75). But 0.64-3.83 s is a 'look' segment whose TCP stays ~0 m/s (timeline flat, strip tip < 0.01 m/s): a third of the 10.25 s clip reads as stillness. The moving part (3.83-6.38 s, peak 0.26 m/s, then home 6.38-9.57 s) is the same straight vertical sink as rest_06_glide, which does it without the dead start.
- mechanical at t 0.64-3.83 s, ghosts 2-4: TCP speed ~0 for 3.2 s inside the glide bar (segment 'look'), show energy 0.17
- **change** `choreo phrase spec bars[].moves (glide at the rest hub)`: moves: 1 for glide bars drawn at rest (a travel accent, as rest_06_glide) (with two moves the glide MENU draws a 'look' whose TCP stays still from this pose (0.64-3.83 s here, 0.8-4.8 s in rest_02) before the same sink; scope this family)
- replace with: rest_06_glide
- live check: whether the 'look' turns the strip visibly enough to read as a gaze rather than a pause; what the LEDs show while the strip faces down at this hub (idle_leds cues)
- sheet: `reviews/sheets/rest_01_glide.png`

### rest_02_glide  (medium confidence; expression 2; mechanical)

- expression 2 (lacks phrasing): A clean glide buried in 12 s of near stillness: mostly held.
- reads as: a slow strip turn and holds (~5.6 s still), then a straight glide sink 1.10 -> 0.77 m and home: 1/1 agree (glide, space +0.77, flow -0.75). 12.0 s long, but the TCP moves only 4.8-7.2 s (peak 0.33 m/s) and on the way home 8.0-11.2 s; 0.8-4.8 s is a look + hold with strip tip < 0.03 m/s, and 7.2-8.0 another hold. Show energy 0.09. Same straight sink as rest_06_glide (which also sinks deeper, to 0.70 m).
- mechanical at t 0.8-4.8 s and 7.2-8.0 s, ghosts 2-5: TCP speed ~0.01 m/s for 4 s then a hold; show energy 0.09 over 12 s
- **change** `choreo phrase spec bars[].moves (glide at the rest hub)`: moves: 1 for glide bars drawn at rest (the two-move glide draws a look + holds that read as 5.6 s of stillness from this pose; the one-move glide (rest_06) is the same sink without it; scope this family)
- replace with: rest_06_glide
- live check: whether the slow strip turn at 0.8-4.0 s reads as a gaze from the audience; what the LEDs show while the strip faces down at this hub (idle_leds cues)
- sheet: `reviews/sheets/rest_02_glide.png`

### rest_05_dab  (medium confidence; expression 1; crowding)

- expression 1 (lacks anticipation, overshoot, dynamics): A barely visible tick with neither wind-up nor snap: rest_07_dab does the same action alive.
- reads as: a single tiny wrist tick (3 cm), then home: 1/1 agree (dab, time +0.59, space +0.62). One-move accent at size 0.70: TCP extent 0.003 x 0.033 x 0.009 m, peak 0.10 m/s, strip tip 0.30 m/s, J4/J6 ranges 7/8 deg, show energy 0.03. A dab is meant small, but this is the weaker of the two dabs at the hub: rest_07_dab (size 1.08) shows two clear taps with the strip tip at 1.09 m/s.
- crowding at t 1.375 s, ghost 6: self clearance 0.8 mm forearm_link / tool_strip_bottom_b (the strip all but touching the forearm at the wrist); not visible from 2 m
- **change** `choreo phrase spec size (dab accents)`: size >= 1.0 for one-move dab accents (here 0.70) (rest_07_dab at size 1.08 reads as a tap (strip tip 1.09 m/s); at 0.70 the dab is a 3 cm tick with energy 0.03; scope this family)
- replace with: rest_07_dab
- live check: the 0.8 mm strip-to-forearm gap on the real arm (cable, strip mount); whether the tick is visible at all at full size
- sheet: `reviews/sheets/rest_05_dab.png`

## Keep (46)

| clip | expr. | reads as | notes | live check |
|---|---|---|---|---|
| greet_00_look | 3 | look | Short glance form (4.6 s): the strip swings between three or four gaze points (J6 range 42 deg, strip tips 1.16 m/s vs TCP 0.53), holds between, a small lean (TCP extent 0.10 x 0.10 x 0.30 m). Reads as the arm looking round the audience.; twitch | whether the J4/J6 reversal at ~1.5 s reads as a hitch at the strip tips or as a settle; timing feel of the holds between glances (accel tempo) |
| greet_01_wave | 3 | wave | The hand up to 1.45 m and four sideways swings of 0.35 m (ghosts 4-7) with the tool kept on one person and the strip rolling with them (J6 range 60 deg); speeds rise and fall evenly with a last smaller swing. A clear wave.; crowding | the LED strip's end exceeds the operator_slow zone's speed limit (build warning): a safety review item, check on the arm; how close the strip's top looks to the left partition at the far end of the swing (t 3.3 s) from where guests stand |
| greet_02_nod | 3 | nod | The gaze drops and the TCP dips 0.2 m (z 1.11-1.33) twice around t 1.8-3.0 s, then a strip roll at ~4.1 s (J6 only) and a small return. The middle dips are the biggest rather than 'smaller each time', and the J6 roll at 4.1 s reads as a head tilt more than a nod; still reads as nodding.; twitch | whether the bottom of the nod (t ~2.7 s) turns smoothly or hitches; whether the late strip roll (t ~4.1 s) reads as part of the nod |
| greet_04_tilt | 3 | tilt | The TCP holds still (extent 0.07 x 0.09 x 0.04 m, meant small) while the strip rolls to one side and the other (J6 range 44 deg, strip tips 0.74 m/s vs TCP 0.22): a head tilt carried by the strip, with holds. The calm one, as described.; crowding | whether a 44 deg strip roll with a still TCP reads as a tilt at full size or as nothing much |
| greet_05_trace | 3 | trace | One loose loop down to z 0.90 and back (extent 0.20 x 0.12 x 0.43 m), the gaze drifting with it, then a still at t 3.3-3.7 s and a separate quick return (J1 49 % speed). The loop has corners at ghosts 2, 5 and 8 (TCP speed dips to 0.15 m/s at 1.9 s) so it reads more as a triangle than a flowing path, and the return is a second, separate gesture; still a trace. | whether the corner at ghost 5 (t 1.9 s) reads as a pause or a flowing turn; whether the still at 3.3-3.7 s splits the clip in two |
| greet_06_peek | 4 | peek | Ducks (z to 1.15, only 0.13 m under the hub), still, pops up (burst at 1.8-2.3 s, J3 81 % accel), a small wrist cock (ghosts 7-8) and back. The duck is shallow (extent 0.10 x 0.10 x 0.20 m; low_06_peek travels 0.34 m) but the pop-and-cock sequence is there. | whether a 0.13 m duck reads as hiding from 2 m; timing of the pop (burst accent) |
| greet_07_shy | 3 | shy | The gaze drops away first (strip turn at 1.8 s, TCP still), then the TCP shrinks back and down towards the robot (to z 1.01, ghosts 5-7), then returns. Intensity 0.94: the return at 4.6-5.5 s is as quick as the retreat (TCP 0.6 m/s both), so the 'slowly looks back' is not slow; reads as shy, a confident one. | whether the quick return undoes the shyness (timing feel) |
| greet_08_bounce | 3 | bounce | Eight even bobs on the beat (TCP 0.3-0.35 m/s each, accel tempo), the gaze fixed, then a strip roll at 5.1-5.8 s and home. Drawn at size 0.45 (the last of SIZES): bobs of ~0.09 m (extent 0.12 x 0.11 x 0.09 m), small but a head bob is meant small and the 1 m strip amplifies it.; crowding; too_hard | whether the tool bracket visibly grazes the forearm at t 5.1 s (self clearance 1 mm on the sheet's mesh); whether the bobs sit on the beat (accel tempo) |
| greet_09_search | 4 | search | Scans side to side across 0.33 m with quick looks and holds, ending on one person. 'Travelling and bending low and high' is missing: z 1.20-1.32 (0.12 m) only.; twitch; too_hard | the LED strip's end exceeds the operator_slow zone's speed limit (build warning): a safety review item, check on the arm; whether the J5 hitch at ~2.5 s shows at the strip's ends |
| greet_10_rise | 3 | rise | Gathers low (z 1.16, only 0.12 m under the hub), then rises to 1.52 looking up and out (ghosts 5-6), holds, and returns. The return at 3.8-4.6 s is the burst accent: the hardest move of the clip (J6 79 % speed / 84 % accel, strip tips 1.54 m/s), so the 'settles back' snaps rather than settles.; too_hard | the LED strip's end exceeds the operator_slow zone's speed limit (build warning): a safety review item, check on the arm; whether the burst return reads as intended or as a whip of the strip |
| greet_11_dive | 5 | dive | A small lift, then a swoop down to z 0.72 in front and back up past the hub to 1.49 (extent 0.77 m vertical), the gaze leading. The biggest body motion at the hub; reads like a bird dipping. | the LED strip's end exceeds the operator_slow zone's speed limit (build warning): a safety review item, check on the arm; the 1.1 m/s TCP / 1.67 m/s strip tip swoop: speed impression at full size |
| greet_12_sweep | 3 | sweep | A swing-shaped arc across 0.51 m (a shallow smile in the audience-side view) and back, the gaze leading. 'Through heights' is weak: z 1.22-1.34 (0.12 m) at intensity 0.24. | the LED strip's end exceeds the operator_slow zone's speed limit (build warning): a safety review item, check on the arm |
| greet_13_pop | 3 | pop | A flick down (0.4 s), a snap up 0.36 m to z 1.57 (pose 'up'), a freeze 1.8-2.9 s, back. The snap is a 1.2 s symmetric speed bell (slowed 1.35) rather than a stop dead; still reads as an accent with a freeze.; too_hard | the LED strip's end exceeds the operator_slow zone's speed limit (build warning): a safety review item, check on the arm; the tool point is 5 mm outside the controller zone at the top of the pop (build warning): the controller may stop the arm, check on the arm; whether the snap reads as stopping dead (timing feel) |
| greet_14_twirl | 4 | twirl | The strip swings like a baton face on to the audience: J6 range 140 deg, strip tips 1.14 m/s vs TCP 0.27 (the strip carries it), two swings each way with braking, settles face on. Spin 120 deg is TWIRL_MIN_DEG, the least the family allows (room 140/130). | the LED strip's end exceeds the operator_slow zone's speed limit (build warning): a safety review item, check on the arm; the strip's light while it swings |
| greet_18_window | 3 | window | Level blade on a pane facing the audience, one lying figure eight 0.55 m wide (width_m 0.549) drawn at t 3.6-8.5 s, the blade tipping with it: visible as an eight in the audience-side view. The strip's turn to level (0-1.3 s) and back (10.5-11.6 s) are the fastest moves (J6 78 % speed, strip tips 1.24 m/s); one eight in 11.8 s is a lot of setup for one stroke. | the LED strip's end exceeds the operator_slow zone's speed limit (build warning): a safety review item, check on the arm; whether the turn-in / turn-out strip spins overshadow the single eight (timing feel) |
| greet_20_lissajous | 3 | lissajous | A 2:1 Lissajous in the audience plane (extent 0.32 x 0.14 x 0.35 m), look orient, strip turned 90 deg to level: the knot is legible in the audience-side view. The strip's turn to level (0-1.2 s) and back (9.0-10.2 s) are the fastest moves (J6 76 % speed, strip tips 1.2 m/s while the TCP is near still).; twitch | the LED strip's end exceeds the operator_slow zone's speed limit (build warning): a safety review item, check on the arm; whether the jolt at ~8.3 s is visible on the arm (the controller's smoothing may hide it) |
| high_00_look | 3 | look | Long form (9.3 s, k 0.32): the TCP almost still (extent 0.04 x 0.11 x 0.10 m) while the strip carries the gaze between four or five points (J4 range 66 deg, J6 54; strip tips 0.62 m/s vs TCP 0.33), quick glances at 0.4 and 2.2 s, slower ones at 4.0, 5.0 and 6.1 s, a slow drift to end (8.5 s). Reads as the arm looking round the audience; the lean and bob towards each point are small because the tool tip has little room at this hub.; twitch | whether the J4/J6 spike at ~5.05 s reads as a hitch at the strip's ends; the wrist comes within ~13 deg of its singularity (min /J5/ 12.9 at t 4.2 s deg; the hub itself is at 26): whether the FR20 raises its singular-pose fault or the wrist visibly speeds up there |
| high_01_wave | 3 | wave | Six even sideways swings of 0.30 m (x extent 0.299 m, TCP peaks 0.85-0.9 m/s, t 2.0-6.2 s, accel tempo) with the tool kept on one person and the strip rolling with them (J6 range 33 deg). The 'hand goes up' is only 0.12 m (z 1.43-1.55): the controller's 1.6 m TCP cap is just above the hub. Still reads as a clear wave; greet_01_wave is the bigger one (0.35 m, 60 deg roll).; twitch | whether the small 'hand up' (0.12 m) still reads as a raised hand in perspective; timing feel of the accelerando swings |
| high_02_nod | 4 | nod | Calm (k 0.22, 10.8 s): the gaze drops first by the wrist (t 0.3-1.0 s, J5 24 % speed, TCP 0.05 m/s), then four dips of the TCP (t 2.0-7.1 s, z 1.33-1.46, extent 0.05 x 0.08 x 0.13 m: meant small), the later two smaller (TCP 0.25 -> 0.13-0.15 m/s), and a slow float drift to finish (8.3-10.5 s). Every joint under 30 % accel. Reads as a slow nod; long for one. | whether 10.8 s of nodding holds attention or drags (timing feel) |
| high_06_peek | 3 | peek | All the parts are there, small: ducks and creeps in (t 0.2-1.8 s, z to 1.36, 0.11 m towards the audience), still, pops up (burst t 2.2-2.8 s, J3 61 % accel), cocks the wrist (t 2.9-3.4 s, J6 34 % speed, strip tips 0.53 m/s with the TCP still), pauses, darts back (3.6 s), home (4.1-5.3 s). The duck is shallow (extent 0.005 x 0.11 x 0.12 m at k 0.24; low_06_peek ducks 0.34 m); the strip's wrist cock is what reads. | whether the 0.1 m duck reads as ducking in perspective, or only the wrist cock does; the wrist comes within ~13 deg of its singularity (min /J5/ 13.5 at t 4.05 s deg; the hub itself is at 26): whether the FR20 raises its singular-pose fault or the wrist visibly speeds up there |
| high_07_shy | 3 | shy | The gaze drops away first (t 0.2-1.0 s, a busy wrist move: J4 73 %, J6 63 % accel, several peaks), then the TCP shrinks back 0.31 m towards the robot and 0.26 m down (t 1.6-2.8 s, z to 1.19, ghosts 4-8), holds, a small look at 4.3-4.6 s, and comes back (t 5.6-7.0 s). At k 0.91 the look back is as quick as the retreat (0.50 vs 0.56 m/s): not 'slowly' (as greet_07_shy). Reads as shy.; crowding; twitch | whether the wrist folded 3 mm from the forearm at ~3.8 s looks like a pinch on the arm; whether the quick look back still reads as shy (timing feel) |
| high_09_bounce | 3 | bounce | Bobs on the beat with the groove sinking (z 1.45 -> 1.26, extent 0.08 x 0.03 x 0.19 m), the gaze and the strip fixed on one person (J6 range 4.5 deg: the strip translates), five bobs t 0.8-5.3 s with an accelerando (TCP peaks 0.27-0.43 m/s). Reads as a head bob to music.; twitch | whether the bobs sit on the beat (timing feel) |
| high_10_search | 3 | search | Short form (5.2 s): the TCP scans 0.21 m sideways (x extent 0.211 m, out to ghost 3 and back) with quick moves and holds, the gaze turning a little (J4 14, J6 9 deg), ending towards one person. 'Travelling and bending low and high' is missing: z 1.44-1.48 (0.04 m) - the TCP cap is just above the hub. Weaker than greet_09_search (0.33 m) but reads.; twitch | whether a 0.21 m scan with a 9 deg gaze reads as searching in perspective |
| high_11_rise | 4 | rise | Gathers low with the gaze on the floor (t 0.2-1.5 s, z to 1.24, ghosts 3-4), still (1.5-2.0 s), rises with the burst to z 1.535 looking up and out (t 2.0-3.5 s, J4 58 % speed; ghosts 5-8), holds, a small settle (4.5 s), home (5.0-6.2 s). The rise is 0.29 m but only 0.09 m above the hub (the 1.6 m TCP cap): the 'high' is made by the wrist looking up (J4 range 77 deg). Like greet_10_rise, reads. | whether 'rises high' reads when the tool ends barely above where it started (perspective) |
| high_12_dive | 5 | dive | A small lift (t 0.3-0.8 s, z 1.54), a swoop down in front to z 1.005 (t 1.5-3.4 s, ghosts 4-6: 0.53 m), back up past the hub (burst t 3.6-4.5 s, TCP 0.92 m/s, z 1.54 at 4.8 s), a settle and home. Gaze leading (J4 range 93, J6 70 deg). The biggest body motion at this hub; reads like a bird dipping.; too_hard | whether the burst back up reads as a swoop or a jerk (timing feel) |
| high_13_sweep | 4 | sweep | A big arc across 0.65 m (x -1.2 to -0.55) and through heights (z 1.24-1.53): out to one end fast (t 0.3-1.5 s, TCP 0.82 m/s, J2 75 % accel), hold, the arc across with the gaze leading (t 2.4-5.4 s, a rainbow in the audience-side view, ghosts 4-8), hold, back (6.5-7.7 s). The clearest whole-arm clip at the hub.; crowding; twitch | whether the fast first swing out (J2 75 %) looks like a lurch in perspective |
| high_14_pop | 3 | pop | A flick up (t 0.2-0.55 s), the snap down 0.6 m to z 0.91 (pose 'down', t 0.6-1.8 s), a small rebound (1.9-2.1 s), a freeze (2.2-2.9 s), back up (2.9-4.1 s, 0.87 m/s). The snap is a 1.2 s symmetric speed bell (slowed 1.22) rather than a 'stop dead', and the way back is as quick as the snap (as greet_13_pop); still reads as an accent with a freeze.; too_hard | whether the snap down reads as a snap or a fall (timing feel); whether the freeze reads as a held pose |
| high_15_twirl | 3 | twirl | The hub's own aim leaves 20 deg one way (room_deg [20, 130]), so it drops 0.2 m to a level aim first (t 0.2-1.0 s), then two big swings of the strip (t 2.1-4.1 and 4.2-6.4 s), braking past the mark (J6 accel bursts ~60 % at 3.9 and 6.4 s), and home. J6 range 140 deg, strip tips 1.09 m/s vs TCP ~0.05 in the swings: the strip carries it, a full face-on fan in the audience-side view. Spin 120 deg is TWIRL_MIN_DEG, as greet_14_twirl. | the strip's light while it swings; the strip's ends at ~1.1 m/s in front of guests (perspective, speed impression) |
| low_00_look | 4 | look: a quick glance up, a slow drift across, a last glance, the gaze leading | Glance up 0.55 -> 0.81 m at 0.2-1.0 s (TCP 0.69 m/s), slow drift 1.9-3.4 s (0.17 m/s), last glance 3.8-4.6 s; LED arrows swing between points in the audience (facing 0.97). Wrist share 0.77: the wrist carries it. Short form, 4.8 s: two to three glances, few holds.; twitch | whether the holds between glances read as looking (breathing) or as pauses; the after-bump at 1.0-1.2 s on the arm |
| low_01_wave | 3 | wave: the hand goes up and swings side to side, the strip rolling with the swing | Rises 0.55 -> 0.73 m, then swings 0.35 m across (AUDIENCE SIDE ghosts 4-8) with the strip fanning (J6 range 55, strip tip 1.32 m/s vs TCP 0.75), aim kept on one person (facing 0.94). Same shape as greet_01_wave (0.35 m across), 0.6 m lower. | whether a wave at knee-to-waist height reads as greeting from 2 m |
| low_03_reach | 3 | reach: a lean back, out and up towards the audience, holds, two small offers, home | Lean-back anticipation 0.4-1.0 s (strip-led), the offer 1.0-3.6 s rising 0.5 -> 1.06 m (TCP 0.5 m/s), hold, 'here' offers at 3.8 s and 5.1 s (0.2-0.3 m/s), then home 5.9-6.9 s is the fastest move (0.91 m/s): the retreat is quicker than the offer. J4 range 149 deg, J4 speed 83 % at 1.9 s: a big wrist turn inside the offer, but the strip stays laid diagonally in the views.; too_hard | whether the offer reads towards the audience in 3D (the outward part is in depth); the J4 149 deg wrist turn at 1.2-2.6 s on the arm |
| low_05_trace | 3 | trace: one loose flowing loop up through the space and back, the gaze drifting | A loop 0.55 -> 1.07 m and down (FRONT and AUDIENCE SIDE, ghosts 3-9), curved throughout (slash), wrist share 0.68, facing 0.97. A speed notch to 0.12 m/s at 2.1 s (ghost 5, top of the loop) splits the flow into two strokes; the way home (3.8-4.8 s, 0.60 m/s) is the quickest part.; twitch | whether the notch at 2.1 s reads as a hitch in the flow |
| low_06_peek | 4 | peek: ducks low, still, pops up, cocks the strip, darts back | Ducks 0.55 -> 0.41 m (0.3-1.9 s), still, pops up to 0.75 m (2.3-3.0 s, 0.64 m/s), cocks the wrist (J6 only, 3.3-3.9 s, strip tip 0.6 m/s), holds, darts back down to 0.56 m (4.8-5.5 s), home. Measured punch in every bar: the sudden parts read. | whether the 'still' before the pop reads as suspense or as a stop; the creep forward in 3D |
| low_09_bounce | 4 | bounce: bobs on the beat, a groove step, quicker bobs (accel) | Size 0.6, k 0.98: four bobs (1.3-5.0 s) carried mostly by the strip swaying (strip tip 0.8 m/s vs TCP 0.25-0.3), a step aside at 5.5 s, four tighter bobs 6.9-9.0 s (TCP 0.35-0.4 m/s). Extent 0.20 x 0.07 x 0.19 m; facing 0.99. Each dip rolls the strip (lower end 0.10-0.16 m).; twitch | whether the bobs sit on the beat; whether the strip's roll on each dip reads as a deliberate head tilt or as the prop flailing (Houdini review video) |
| low_10_search | 4 | search: scans across the audience side to side, quick looks and holds, ends on one person | Travels 0.39 m across (AUDIENCE SIDE ghosts 1-8) with the strip fanning (J4 106, J6 97 deg), holds between looks, ends turned towards one person (ghosts 9-10); facing 0.94. 'Bending low and high' is small: z 0.50-0.67.; twitch | whether the small blips at 3.1, 4.8 and 7.1 s read as hesitations or hitches |
| low_11_rise | 3 | rise: sinks with the gaze down, still, rises looking up and out, holds, settles | Gathers 0.55 -> 0.45 m (0.5-1.6 s), still to 2.4 s, rises to 0.72 m (2.4-5.0 s) with the arrow turning up (ghost 6), holds 5-7 s, settles. Modest at k 0.33: 0.27 m of height (extent 0.01 x 0.10 x 0.28 m), TCP peak 0.21 m/s; the strip swings more than the tool (tip 0.56 m/s).; twitch; twitch | the wrist spike mid-rise at 3.45 s on the arm; whether 'looking up over the audience' reads from a tool at 0.7 m |
| low_13_sweep | 3 | sweep: a low arc across ~0.5 m and back, the gaze leading | Size 0.6: an arc 0.51 m across (AUDIENCE SIDE ghosts 3-8), a shallow rainbow (z 0.48-0.62, 0.14 m), the strip turning with it (J6 104 deg, strip tip 1.29 m/s vs TCP 0.48). 'Through heights' is weak, as greet_12_sweep (0.12 m).; crowding; twitch | the 4 mm strip-to-forearm gap on the real arm (strip mount, cable) |
| low_14_pop | 3 | pop (up): a strip flick, a rise to a pose, a freeze, back | Flick the other way 0.3-0.9 s (J6, strip tip 0.65 m/s), up 0.50 -> 0.88 m (1.0-1.85 s, 0.79 m/s), a small after-bump at 2.0 s, freeze 2.2-2.9 s, back. At bpm 77 / k 0.45 the snap is a 0.85 s symmetric speed bell, as long as the return: an accent with a freeze rather than a snap that stops dead (as greet_13_pop). | whether the stop reads as 'dead' at this tempo |
| low_15_twirl | 3 | twirl: the strip swings back and forth face on like a baton, the tool tip still | Two swings at 2.8-5.0 s and 5.3-7.7 s (J6 range 139 deg, strip tip 1.06 m/s vs TCP 0.16 m/s), braking past the mark (bumps at 5.0 s and 7.7 s), face on (facing 0.98). spin_deg 120 (= TWIRL_MIN_DEG). Wind-up 0-2.8 s and settle 8.8-11.6 s are quiet (strip tip <= 0.17 m/s): ~5.5 s of the 11.6 s. | whether the 2.8 s wind-up and settle feel alive; the strip's light while spinning |
| low_16_wipe | 3 | wipe: strokes side to side on a leaning pane, stepping down, a shake | Pane tilted back (normal 0.51, -0.64, 0.57); strokes across ~0.45 m at 0.55-1.01 m, stepping down (FRONT ghosts 2-10 zigzag), lifted between (0.3 m/s bumps), a strip shake 9.0-10.0 s (TCP still), home. Every bar measures punch: even strokes on the beat, as a routine should be. The blade lies ~45 deg throughout (the leaning pane).; too_hard; twitch | whether the even strokes read as wiping or as mechanical; the strip's light on the strokes |
| rest_00_float-float | 3 | float-float: a slow sink 1.11 -> 0.64 m and a rise back, strip fanning | 2/2 bars agree (float, space -0.99, time -1.00, flow +0.48). Two smooth speed humps 0.72-3.61 s and 3.61-6.51 s (peak 0.34 m/s), accel < 9 %. The TCP path itself is near-vertical (extent 0.08 x 0.13 x 0.47 m); the indirectness is mostly the strip fanning ~30 deg in TOP (J4 range 25, J6 16). Same sink-and-return shape as rest_06/09/10/11.; crowding | whether the float feels weightless/wandering or reads as a plain sink-and-rise; what the LEDs show while the strip faces down at this hub (idle_leds cues) |
| rest_06_glide | 3 | glide: a smooth, straight, even sink 1.09 -> 0.70 m and an equally smooth return | 1/1 agree (glide, time -1.00, space +0.76). One-move accent: one bell of speed 0.33-3.59 s (peak 0.24 m/s), accel <= 6 %, J4 range 40 with J6 still (strip stays parallel), ghosts 2-6 in a straight vertical line. The best glide at the hub. The return home (3.59-7.24 s) is as long as the bar, so the clip reads as sink-and-back (shared by most rest clips).; crowding | whether the even speed reads as a glide or as slow and dull at full size; what the LEDs show while the strip faces down at this hub (idle_leds cues) |
| rest_07_dab | 4 | dab: two quick, precise wrist taps (the second larger), syncopated | 1/1 agree (dab, time +0.95, space +0.59). Taps at ~0.9 s and ~2.3 s, TCP peak 0.40 m/s, strip tip 1.09 m/s, extent 0.06 x 0.01 x 0.09 m (small as a dab should be). Accel peaks J4 60 % / J5 49 % in a sudden bar (under 80 %). The flagged J4 twitch (1) is the designed anticipation: J4 goes back ~0.9 deg at 1.83-2.08 s (the hold before the second tap) and then strikes; the accel curve shows no isolated spike.; twitch | whether the 0.9 deg wind-up at 1.9 s reads as anticipation or as a hiccup on the arm; whether the syncopated rhythm sits on the beat; what the LEDs show while the strip faces down at this hub (idle_leds cues) |
| rest_09_float | 3 | float: a slow, slightly curving sink 1.09 -> 0.80 m with the strip fanning, then home | 1/1 agree (float, time -0.99, space -1.00, flow +0.63). One smooth bell 0.36-3.21 s (peak 0.21 m/s), accel <= 6 %. The path bows ~5 cm in AUDIENCE SIDE (ghosts 3-6) and the strip fans in TOP (J4 range 26): indirect, if mildly. Very close to rest_11_float and rest_06_glide in shape (sink to 0.7-0.8 m and back). | whether it reads as weightless/wandering rather than as the same sink as rest_06_glide; what the LEDs show while the strip faces down at this hub (idle_leds cues) |
| rest_10_float-float | 4 | float-float: a quicker sink 1.11 -> 0.61 m, then a wandering rise with the strip swinging | Bar 1 intent float measured wring (1/2 agree; whole clip float). Bar 1 is the fastest float at the hub (TCP 0.46 m/s, J3 speed 24 %), with bound flow -0.83; bar 2 wanders (the two strip tips' speeds cross, J6 range 24). Largest extent at the hub (0.05 x 0.17 x 0.50 m). More varied than rest_00 (same intent) so both are worth keeping. | whether bar 1 reads heavy (wring) on the arm or still light; what the LEDs show while the strip faces down at this hub (idle_leds cues) |
| rest_11_float | 3 | float: a slow sink 1.10 -> 0.70 m with the strip fanning, then home | 1/1 agree (float, time -1.00, space -0.58, flow +0.75). One smooth bell 0.42-3.80 s (peak 0.23 m/s), accel <= 6 %. The path is nearly straight (slight wiggle at ghosts 3-4); the indirectness is in the strip fanning (TOP, J4 range 24, J6 11). Shape is close to rest_09_float and rest_06_glide.; crowding | whether it reads as a float or as the same sink as rest_06_glide; what the LEDs show while the strip faces down at this hub (idle_leds cues) |
