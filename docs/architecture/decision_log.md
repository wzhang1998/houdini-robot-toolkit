# Decision log (`motionlab.decision/1`)

Every time the Runner needs the next idle clip and a Director is plugged in
(`contracts/director.py`, `DirectorSelector`), one JSON line records what the
Director was shown, what it answered, how long it took, whether the answer was
used or the show fell back to `show.Selector`, and what actually played. The
log is what makes a Director **replaceable and comparable**: two Directors are
compared on their logs, never on impressions.

Writer and reader: `scripts/contracts/decision_log.py`. Replay and comparison:
`scripts/contracts/director_replay.py`. The show itself never reads a log.

## File

JSON Lines, one object a line, UTF-8, keys sorted. A run is:

```
{"kind": "run_start", ...}       once
{"kind": "decision", ...}        once a pick
{"kind": "run_end", ...}         once, when the run ends cleanly (absent after a crash)
```

Default place: `logs/decisions/<tag>_<director>.jsonl` (`logs/` is gitignored).

Every line carries `"schema": "motionlab.decision/1"` and the run's `run_id`.

## `run_start`

```jsonc
{
  "schema": "motionlab.decision/1", "kind": "run_start", "run_id": "3f9c0a1b2d4e",
  "t_wall": 1793700000.12,
  "library": {                                    // LibraryIndex, in full: the decisions refer to it by digest
    "digest": "a1b2c3d4e5f60718",
    "hubs": ["greet", "high", "low", "rest"],
    "triggers": ["scan", "calm", "greet"],
    "clips": [{"id": "rest_00_float-float", "hub": "rest", "family": "float", "action": "float",
               "energy": 0.348, "duration_s": 8.71, "tags": ["float", "float"]}, ...]
  },
  "meta": {
    "show": "shows/party.json", "seed": 1, "minutes": 10, "dt": 0.05, "budget_s": 0.05,
    "director": {"name": "example_attentive", "version": "0"},
    "perception": {"source_id": "femto_recording:femto_rec_….csv", "kind": "recording",
                   "rate_hz": 30.0, "extrinsic": "femto_extrinsic.json@e7a2e8f6344f", "latency_s": 0.09}
  }
}
```

## `decision`

| Field | Type | Meaning |
|---|---|---|
| `seq` | int | the pick's number in the run (1 = the Runner's first pick, made before it is attached: no `state`, no `pending`) |
| `t_wall` | float | wall clock (s since epoch) when the Director was asked |
| `show_clock` | float | show seconds (`Runner.clock`) — the replay's timeline |
| `director` | `{name, version}` | who was asked |
| `input` | object | the **input snapshot** (`DirectorInput.to_dict()`, `motionlab.director_input/1`): `scene` (SceneSummary), `library` (digest + triggers only), `show` (ShowState), `candidates` |
| `input_digest` | str | sha256[:16] of `input` — equal digests are the same question |
| `output.intent` | object \| null | the parsed answer (`motionlab.intent/1`) — also when it was refused as invalid (it is logged, never carried out); null if none (timeout, error, unparsable) |
| `output.raw` | str \| null | the raw answer when it came as a dict (≤ 500 chars) — what a model actually said |
| `validation` | `{ok, errors[]}` | `validate_intent`'s verdict; errors name the field |
| `fallback` | `{used, reason}` | `reason` ∈ `timeout`, `error`, `invalid`, `busy`, `defer`, or null; `used` ⇔ `reason` not null |
| `latency_ms` | float | from submit to answer (or to the budget, on a timeout) |
| `budget_ms` | float | the budget it had |
| `applied` | object | what the show did: `source` (`director` \| `fallback`), `clip` (always — every pick plays a clip), `clip_by` (`director` \| `selector`), `trigger` (a name the Director triggered, or null) |

Example (a `trigger` intent: the trigger is the Director's, this clip the Selector's):

```jsonc
{
  "schema": "motionlab.decision/1", "kind": "decision", "run_id": "3f9c0a1b2d4e", "seq": 4,
  "t_wall": 1793700003.81, "show_clock": 41.25,
  "director": {"name": "example_attentive", "version": "0"},
  "input": {
    "schema": "motionlab.director_input/1", "seq": 4,
    "scene": {"n_people": 1, "n_predicted": 0, "hands_up": 1, "ids": [3], "age_s": 0.04, "stale": false,
              "nearest": {"id": 3, "dist_m": 2.01, "bearing_deg": -92.3, "height_m": 1.6}},
    "library": {"digest": "a1b2c3d4e5f60718", "triggers": ["scan", "calm", "greet"]},
    "show": {"clock": 41.25, "hub": "high", "state": "IDLE", "sequence": null, "mood": null,
             "operator_mood": null, "operator_energy": null, "energy_target": 0.214,
             "recent": ["high_03_tilt", "high_12_dive"], "recent_picks": ["high_03_tilt", "high_12_dive"],
             "pending": []},
    "candidates": ["high_00_look", "high_01_wave", "..."]
  },
  "input_digest": "9e1d…",
  "output": {"intent": {"schema": "motionlab.intent/1", "kind": "trigger", "trigger": "greet",
                        "reason": "a hand up"}, "raw": null},
  "validation": {"ok": true, "errors": []},
  "fallback": {"used": false, "reason": null},
  "latency_ms": 0.21, "budget_ms": 50.0,
  "applied": {"source": "director", "clip": "high_07_peek", "clip_by": "selector", "trigger": "greet"}
}
```

`decision_log.validate_record(rec)` checks the required fields and the
invariants (`used` ⇔ `reason`; a fallback is never `applied.source = director`;
every decision has a clip). The conformance and replay self-tests validate
every line they write.

**Privacy.** The input holds a `SceneSummary` — counts, one nearest distance and
bearing, the tracker's numeric ids — never images, skeletons or anything that
identifies a person beyond a session-local id. Keep it that way: a Director
that needs more must get a new summary field (a contract change), not raw data
in the log.

## `run_end`

```jsonc
{"schema": "motionlab.decision/1", "kind": "run_end", "run_id": "…", "t_wall": …,
 "summary": {"history": [[0.0, "rest_04_…"], …],       // Runner.history: (show clock, segment) — moves and scans too
             "counts": {"director": 3, "defer": 9, "timeout": 0, "error": 0, "invalid": 0, "busy": 0},
             "metrics": { … see below … }}}
```

## Metrics (`decision_log.metrics`)

| Metric | Reads as |
|---|---|
| `decisions` | picks in the run |
| `director_applied` | share of picks where the Director's answer was used |
| `fallback_by_reason` | `{none, defer, timeout, error, invalid, busy}` counts |
| `latency_ms_p50 / p95 / max` | the Director's speed |
| `distinct_clips`, `repeats_within_no_repeat`, `same_family_twice` | variety (the baseline's own rules, measured on anyone) |
| `energy_abs_error_mean` | mean \|clip energy − the energy the show wanted\| (the arc or the operator) |
| `triggers_issued` | triggers a Director asked for |
| `decisions_with_people`, `director_applied_with_people` | how often there was someone, and how often the Director acted then |

These are deliberately the measurable ones. Whether a choice *felt* right to
an audience is not in the log; it needs a human looking at the same run
(Isaac render or the arm), with the log as the index.

## Replaying and comparing two Directors

The room comes from a Femto recording through the same path `track_replay.py`
sends it to Isaac or `track_mode` — `track_replay.frames` (→ `femto_format.read`,
`messages`, the extrinsic) — wrapped as a `PerceptionSource`
(`RecordedPerception`, `--latency` added, looped). The show runs on a dry clock
(`show.Runner.step`, as `show.py dry-run`): no arm, no Isaac, no TD.

### Closed loop: each Director drives its own run

```bash
uv run scripts/contracts/director_replay.py shows/party.json \
    --recording geo/tracking/femto_rec_20260930-150731.csv \
    --a baseline --b attentive --minutes 10 --seed 1 --trigger-every 90 --out logs/decisions
```

Both runs get the **same** show, seed, recording, latency, triggers and budget.
What differs is only the Director, so every difference in the metrics is its
doing — but the runs diverge (a different clip leads to a different hub, so to
different questions), which is the point: it measures what the Director's
choices lead to. Output: `logs/decisions/a_<name>.jsonl`, `b_<name>.jsonl` and a
side-by-side table.

Without a recording, `--synthetic` uses a generated walk-by (one person
crossing, a hand raised 10 s in every 20, a second person from 3 s); with
neither, the room is empty and only `show` drives the choice.

### Open loop: B answers A's questions

```bash
uv run scripts/contracts/director_replay.py --open-loop logs/decisions/a_baseline_selector.jsonl --b attentive
```

Each logged `input` is rebuilt (`DirectorInput.from_dict` with the library from
`run_start`) and put to B through the same guard. Reports how often B's answer
would have been used, why not, and how often it names the clip A played.
Cheap and deterministic, but it cannot say what B's choices would have led to.

### Live, alongside the real show (later)

`track_replay.py` sends the recording as OSC (`/track/people`, `/track/hands`)
to `track_mode` or Isaac's `run_tracking`. When a Director runs inside the
stream ([migration step 6](migration.md)), the same `DecisionLog` is written next to the
stream's own log (`logs/`), and the recording + the decision log replay
offline with `--open-loop` — the room, the Director's view and its answers
all in the log, the actual arm motion in the stream's CSV.

### A Director of your own

Any object with `name`, `version` and `decide(DirectorInput) -> Intent`
(or a dict that parses as one):

```bash
uv run scripts/contracts/director_replay.py shows/party.json --synthetic --a baseline --b mypkg.mydirector:MyDirector
uv run scripts/contracts/conformance.py --director mypkg.mydirector:make     # make(graph, seed) -> Director
```

`conformance.py --director` asks it 120 generated questions (every hub, empty /
near / waving / stale rooms, recent picks, moods, energies) and requires: every
answer a valid Intent, the input never changed, every answer serialisable, p95
latency within the budget, the same answers for the same seed (or
`deterministic = False`), and 5 minutes of the show with no `error` or `invalid`.
