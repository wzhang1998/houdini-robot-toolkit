# Review rubric: is a clip worth showing?

How to judge a clip of the show's library **for how it looks**, from a review
sheet (`scripts/review_render.py`: three orthographic views with ghost poses,
the TCP path coloured bar by bar, the timeline, the numbers) and its
`.labels.json`. The safety checks (room, limits, joins, the player) already
ran at build time; a clip in the library passed them. This rubric is about
whether it is worth an audience's attention, and what to change in its
**generator** when it is not.

Each clip is judged on readability, awkwardness, the audience's view and
expressiveness (scored 1-5), then given a verdict. The skill
`.claude/skills/review-clips` applies it and writes
`reviews/<clip_id>.json` and `reviews/SUMMARY.md`.

## What a reviewer looks at

| where | what it shows |
|---|---|
| FRONT, AUDIENCE SIDE, TOP | ghost poses 1..N (pale early, dark late, numbered at the tool), the LED strip (magenta, an arrow where its LEDs face), the TCP path coloured by bar, the room as wireframes (keep-out red, slow orange, work zones dashed, the paper gold, the audience zone and the hub's zone dotted blue) |
| timeline, top | TCP and strip-tip speed (m/s) over the bars, each bar labelled `intent -> measured` (red when they differ); ghost times marked |
| timeline, middle / bottom | each joint's speed and acceleration as % of its limit (dashed line: 100 %) |
| timeline marks | **A** a big move's wind-up, **O** its overshoot (section 4) |
| number panel | generator, hub, family, bpm; intent / measured per bar; peak speed / accel (/ jerk) % per joint; room and self clearance; flags: J5 and J3 zero crossings, J4 / J6 ranges, twitches per joint, wrist share, how much the LEDs face the audience; TCP height and extent; the expression measures (section 4) |

Laban colours (as render_clip_review.py): punch red, slash orange, press dark
red, wring purple, dab yellow, flick cyan, glide green, float light blue; grey
outside a bar (lead-in, travel home). A gesture or path clip has no Laban
intent: its path is coloured by the **measured** action of its own 4-beat bars,
and its intent is its family.

## 1. Readability (可读性): does it read as what it is meant to be?

**Choreo phrases (`rest_*`, `choreo.py`)**: does each bar look like its Laban
action?

| action | weight / time / space | should look like |
|---|---|---|
| punch | strong, sudden, direct | the whole arm drives to a point and stops; straight path, sharp speed peak |
| slash | strong, sudden, indirect | a whole-arm swing on a curve, whip at the wrist |
| press | strong, sustained, direct | slow, heavy, straight; a steady speed plateau |
| wring | strong, sustained, indirect | slow and twisting (J4 against J6), a curling path |
| dab | light, sudden, direct | a small, quick, precise tap by the wrist |
| flick | light, sudden, indirect | a small quick flourish, curling, released |
| glide | light, sustained, direct | smooth, even and straight, wrist-led |
| float | light, sustained, indirect | slow, wandering, weightless |

- Count `bars_agree`. One disagreeing bar out of three is a note; a clip whose
  **main** action (its most frequent intended one) measures as something else
  is a fix. Some confusions are known (motion_labels.py: punch -> press,
  slash <-> wring, flick -> float); still note them, but weigh what the sheet
  shows over the label.
- A phrase of different actions should show the **contrast**: the speed
  timeline should change character from bar to bar. Flat timelines across a
  "punch-float" are a fix.

**Gestures and paths (`greet_*`, `low_*`, `high_*`; gestures.py, paths.py)**:
does it look like its family (the families are described at the top of
`scripts/gestures.py` and `scripts/paths.py`)?

- A look should be read in the gaze (the LED arrow swinging between points in
  the audience), a wave in a sideways swing with the tool kept on one person,
  a nod in a dip and back, a twirl in the strip swinging (J6 range large,
  strip tips much faster than the TCP), a wipe or window in strokes against a
  pane, a sweep in a big arc through heights, a figure (lissajous, figure8,
  spiral, helix, rose) in a recognisable shape **in the view facing its
  plane**.
- A family whose sheet could be any other family (a "wave" that is a small
  nod; a "sweep" with 10 cm of extent; a "spiral" that reads as a blob from
  every view) is a fix; when its parameters can't make it read, drop it.
- Size matters for a room of guests: extent under ~0.15 m reads as a
  twitch from 2 m away unless it is meant to (dab, flick, pop, nod).

## 2. Awkwardness (别扭之处)

| problem | how it shows on the sheet | usually |
|---|---|---|
| wrist flip | `J5 crosses 0` > 0, or `min abs J5` under ~10 deg; consecutive ghosts with the wrist turned over; J4 / J6 jumping by ~180 deg | fix (it also stops the real FR20: the singular-pose fault, COMMANDS.md); drop if it is the family's essence from that hub |
| elbow flip | `J3 crosses 0` > 0; the elbow ghosts on both sides of the upper arm | fix or drop |
| twitch / jitter | `twitches` > 0 on a joint; spikes in the accel % timeline that stand out from their neighbours; a speed curve with a notch in a "sustained" bar. (A fine, even sawtooth on the accel curve is the player's interpolation of 24 fps samples, not motion: a spike counts when it stands well above its neighbours.) | fix |
| too hard | any joint over ~80 % of its acceleration limit **in a light or sustained bar**, or over 90 % anywhere | fix (it will also look violent on the arm) |
| strip orientation | the LED arrow pointing at the floor, the ceiling or a wall for most of the clip; `LEDs face the audience` low for a gesture whose family is about the audience (look, wave, greet hub clips); the strip edge-on to the audience in a strip family (twirl, salute) | fix |
| mechanical | equal-length moves on a constant beat with hard stops; only one joint moving (`wrist share` ~0 or ~1 for a whole-arm action); a path made of straight segments in a float or glide; nothing settling (no holds, no overshoot) | fix |
| crowding | room clearance under ~60 mm (built clips keep the margin, but closeness reads as risk to people watching); the arm folding against itself (self clearance under ~10 mm) | note; fix when it is visible from the audience side |

## 3. The audience's view (观众视角)

Judge from the AUDIENCE SIDE view first (it looks from the audience zone
towards the base); the TOP view puts the audience at the bottom.

- Is the motion **seen**: the important part happens in front of the arm
  from the audience, not behind the column or the upper arm?
- Is the path **legible as a shape** from there: a figure drawn in a plane
  edge-on to the audience is a line to them.
- Does the arm **address** people when the family is about them: the gaze
  (LED arrow) turns towards the audience zone, `LEDs face the audience` is not
  near 0.
- Height: guests' eyes are ~1.5 m. A clip that lives entirely under 0.6 m or
  over 1.8 m is hard to watch for long.
- Clips at the same hub should not all look the same from there (note it
  across the summary, not per clip).

## 4. Expressiveness (表现力): does it look alive?

Readability asks whether the clip says what it claims; expressiveness asks
whether it says it **like a performer** or like a machine moving between
points. Score it 1-5, against what its action or family calls for, not
against one ideal: a bounce *should* keep an even beat, a float *should* have
little light and shade, a dab *should* be small.

The sheet's number panel has two `expression:` lines (also
`labels.expression`), and the timeline marks each big move's wind-up **A** (at
its start) and overshoot **O** (at its end). They are hints measured on the
TCP's path, not verdicts:

| measure | what it hints at | flat when |
|---|---|---|
| `lag_s` | follow-through: the wrist trailing the arm (successive flow) | ~0 in a slash, flick, float, wave (everything at once) |
| `anticipated` / `big_moves` | a wind-up against a big move just before it | 0 for sudden actions and families that strike (punch, slash, dab, flick, pop, reach, dive) |
| `overshot` / `big_moves` | passing the mark and settling back | 0 for sudden strikes and swings |
| `accents_per_min`, `holds` | phrasing: surges, dips, breaths | no holds and few accents in a long clip; or mostly holds (`still_share` over ~0.6) |
| `dynamic_range` | light and shade: the peak speed over the mean while moving | under ~1.8 for anything but sustained actions |
| `onset_cv` | timing variety between big moves | under ~0.1 over 3+ moves: metronomic (right for bounce, nod, wave swings) |
| `bar_contrast` | the change of effort between bars of a phrase (or a gesture's 4-beat bars) | under ~0.5 in a phrase of different actions |
| `strip_ratio` | how much the LED strip carries the motion (its tips' path over the TCP's) | over ~3 outside the strip families: the prop flails while the body stands |
| `joints_used` | the whole body taking part (joints with >10 % of the travel) | 1-2 for a whole-arm action or family |

The detector sees a wind-up or overshoot only as a separate short counter-move
of the TCP. A wind-up done in the wrist alone, or a lean that curves rather
than reverses, is not counted: look at the sheet before calling it missing.

| score | reads as |
|---|---|
| 5 | alive: clear phrasing, light and shade, the anticipation / overshoot / follow-through its character calls for, the body and the strip working together, something unexpected in the timing |
| 4 | expressive with one thing missing (e.g. no wind-up on a strike, or even timing) |
| 3 | fine but plain: it reads, evenly; good filler, nothing a guest remembers |
| 2 | flat: metronomic, one dynamic throughout, or the body still while only the strip moves (outside the strip families), or a long clip mostly held |
| 1 | inert or mechanical: near stillness, or point-to-point moves with hard stops |

Write the score with its evidence (the measures and what the sheet shows) and
what it `lacks` (anticipation, overshoot, follow_through, phrasing, dynamics,
contrast, timing_variety, body_involvement). Expressiveness feeds the verdict
but does not override readability or the awkwardness above:

- a clip that reads but scores 1-2 is a **fix** when a parameter would liven
  it (`intensity`, choreo `principles`, `tempo`, `syncopate`, `flow`, a
  gesture's form), otherwise keep with a note;
- of two clips that duplicate each other, drop the less expressive;
- timing feel itself (whether a wind-up *reads*) stays a live check: the score
  is about what the motion contains, not how it lands on an audience.

## 5. The verdict

- **keep**: reads as its family / actions; nothing awkward a guest would
  notice. Notes are allowed.
- **fix**: worth having, but one or more problems above; each comes with a
  suggestion of a generator parameter to change.
- **drop**: does not read as what it claims and no parameter of its
  generator is likely to make it; or it duplicates a better clip at the same
  hub; or its problem is the family's essence from that hub (e.g. a twirl
  that can only swing 120 deg there). Say which clip replaces it, or that the
  hub's `families` should lose it.

When in doubt between keep and fix, keep and note it. When the sheet cannot
show it (timing feel, the strip's light, the shadow on the paper), say so in
`needs_live_check` instead of guessing.

## 6. Suggestions are generator parameters, never joint angles

A suggestion names **where** (a file and key, or a generator's argument) and
**what** to change. Never write joint angles, never a pose in degrees, never
"move J5 to ...", never a hub's `q` or `near` (those are joint angles too).
The clips are drawn again from these; a hand-tuned pose is lost on the next
build and was never checked.

Allowed targets:

| target | parameters |
|---|---|
| `shows/party.json` `library` | `bpm` [lo, hi], `intensity` [lo, hi], `duration_s`, `bars`, `actions` (the Laban actions a phrase may use), `seed`, `tries` |
| `shows/party.json` `hubs.<hub>` | `families` (drop or add a family at that hub), `clips`, `generator`; `tcp` / `look` (where the tool stands and what it looks at, metres in the base frame) |
| `shows/party.json` | `zones.*` (the audience, greet, idle, low, high zones: centre, size, yaw), `range.speed`, `range.tcp_z`, `select.*` |
| `choreo.py` phrase spec (`random_spec` / `make_clip`) | `bars[].action`, `bars[].moves`, `bpm`, `flow`, `tempo` (steady / accel / rit / freeze), `syncopate`, `principles` (0..1: anticipation, overshoot, follow-through), `levels`, `size`, `tcp_z`, `max_s` |
| `gestures.make` | `family`, `bpm`, `intensity`, `safety`; the module's constants (`GAZE_CONE_DEG`, `TEMPO`, `SIZES`, `MAX_STRETCH`, `SPIN_W_DEG_S`, `WINDOW_M`, `MOP_M`, `SPIN_SWEEP_M`, `STRIP_ROLLS`, `WRIST_KEY_MIN`, `TWIRL_MIN_DEG`, ...) |
| `paths.make` | `family`, `bpm`, `intensity`, `orient` (look / tangent / fixed), `zone`; constants `EXTENT_M`, `V_MPS`, `PLANE_WEIGHTS`, `COUNTS`, `WRIST_SOFTEN`, `MAX_SLOW` |

Prefer the narrowest target: this clip's family at this hub (`hubs.<hub>.families`,
or the family's own constant) over the whole library's ranges. A suggestion
for a module constant changes every clip of that family: say so.

## 7. What a static sheet cannot judge

The sheet is a few frozen poses and some curves. Mark these as
`needs_live_check` (Houdini's review video, Isaac Sim, or the arm) rather than
deciding them from the image:

- timing feel: whether an anticipation or overshoot reads, whether a hold feels
  alive or dead, whether the rhythm sits on the beat;
- the strip's light: what the LEDs show while moving (TouchDesigner's
  idle_leds cues), glare at the audience;
- what a guest actually sees in 3D from where they stand (the sheet is
  orthographic, not their perspective, and does not show occlusion by the
  paper or the room);
- vibration, sag and the controller's own smoothing on the real FR20 (the sheet
  is the plan, not the arm);
- noise and the speed impression at full size.

## reviews/<clip_id>.json

```json
{
  "id": "greet_03_wave",
  "verdict": "keep | fix | drop",
  "confidence": "high | medium | low",
  "readability": {"reads_as": "wave", "matches_intent": true, "note": "..."},
  "awkward": [{"kind": "wrist_flip | elbow_flip | twitch | too_hard | strip_orientation | mechanical | crowding",
               "where": "t 3.1-3.8 s, ghosts 4-5", "evidence": "J5 crosses 0 once (labels.flags)"}],
  "audience": {"seen": true, "faces_audience": 0.42, "note": "..."},
  "expression": {"score": 3, "lacks": ["anticipation", "timing_variety"],
                 "evidence": ["4 big moves, A 0 O 0; onset cv 0.06"], "note": "..."},
  "suggestions": [{"target": "shows/party.json hubs.greet.families", "change": "...", "why": "...",
                   "scope": "this clip | this family | library"}],
  "needs_live_check": ["..."],
  "evidence": {"image": "reviews/sheets/greet_03_wave.png", "labels": "reviews/sheets/greet_03_wave.labels.json"},
  "reviewed_from": "static sheet"
}
```
