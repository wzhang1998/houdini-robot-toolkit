---
name: review-clips
description: Review motion clips of the FR20 show library for looks, without Houdini. Renders review sheets (three views, ghost poses, TCP path by bar intent, joint peaks, room clearance) with scripts/review_render.py, judges each against docs/review_rubric.md, and writes reviews/<clip_id>.json plus reviews/SUMMARY.md (fix and drop first). Use when asked to review, audit or judge clips, gestures, phrases or a show's library for aesthetics, readability or awkward motion.
---

# Review clips

Judge clips of the motion library the way a choreographer would, from static
review sheets, and write down what to change **in their generators**. Never
edit a generator, the runtime or a clip; this skill only adds review files.

Read `docs/review_rubric.md` first, whole: it is the standard, the verdicts,
the review JSON's fields and the parameter targets a suggestion may name.

## 1. Get the clips

Which clips: what the user named (ids, a hub, a family, "all"). The sources:

- **The show's library, full clips** (preferred: they carry the generator's
  bars, intent and params). Build it beside the tracked one, no Houdini
  (~3-10 min, `geo/` is not in git):

  ```bash
  uv run scripts/rebuild_library.py shows/party.json
  ```

  It exits 1 if the rebuild differs from `shows/party.compiled.json`: then the
  sheets show the rebuild, not what the arm plays. Say so in the summary.
  The clips are `geo/rebuild/party/<id>.json` with `manifest.json`.
- **The tracked compiled show** when a rebuild is not possible:
  `shows/party.compiled.json` (idle segments: joints and labels only; a
  choreo phrase's bars are then spread evenly, marked `approx` on the sheet).
- **Fixtures**: `tests/clips/*.json`, `tests/csv/*.csv` (not made for the
  show's room: their room clearance is against the party room anyway).

## 2. Render the sheets

```bash
uv run --with numpy --with matplotlib scripts/review_render.py geo/rebuild/party --out reviews/sheets
uv run --with numpy --with matplotlib scripts/review_render.py geo/rebuild/party --ids greet_01_wave low_15_twirl --out reviews/sheets
```

~15 s a clip. Each clip gets `reviews/sheets/<id>.png` and
`reviews/sheets/<id>.labels.json`. `--capsules` draws the collision capsules
instead of the meshes (when the STLs are missing or for a quick look).
`scripts/review_render.py --self-test` first if the script changed.

## 3. Judge each clip

For every clip, look at the PNG (Read it) **and** read its `.labels.json`;
the numbers settle what the picture suggests (a J5 crossing, a % of a
limit, a clearance). Then, from the rubric:

1. **Readability**: what it reads as. For a choreo phrase, each bar's
   intended Laban action against what the sheet shows and what was measured.
   For a gesture or path, its family (the descriptions at the top of
   `scripts/gestures.py` and `scripts/paths.py`).
2. **Awkwardness**: wrist or elbow flips, twitches, too hard for its
   character, the strip's orientation, mechanical rhythm, crowding; each with
   where (time, ghost numbers) and the evidence (the label's number or what
   the view shows).
3. **The audience's view**: the AUDIENCE SIDE view first, then TOP (the
   audience at the bottom).
4. **Verdict** keep / fix / drop, and a **confidence**: high when the numbers
   and the picture agree, low when only the picture says it.
5. **Suggestions** for fix and drop: generator parameters only (rubric 5),
   the narrowest target that would do it; for a module constant, say every
   clip of the family changes. Never joint angles, never a hub's `q` / `near`.
6. **needs_live_check**: what a still sheet cannot show (rubric 6).

Compare clips of the same family across hubs (`greet_*_wave`, `low_*_wave`,
`high_*_wave`) and clips at one hub against each other: a duplicate or a
weaker version of another clip is a drop with `replace_with`.

Write `reviews/<clip_id>.json` exactly as the rubric's schema, with
`"reviewed_from": "static sheet"` and the sheet and labels paths in
`evidence`. Keep notes short and concrete (time, ghost, number).

Many clips: work hub by hub. With subagents, give each a hub's clips, this
SKILL.md and the rubric; they write the per-clip JSON only. Then one pass
reads all the reviews together for consistency (the same problem judged the
same way across hubs) before the summary.

## 4. Summarise

Cross-clip notes (patterns: a family that fails at every hub, a library-wide
range that causes most fixes, a hub whose clips all look alike) go in a
markdown file, then:

```bash
python .claude/skills/review-clips/summarize.py --notes <notes.md>
```

It checks every review (fields, verdicts, a suggestion for each fix / drop,
**no joint angles in a suggestion**) and refuses to write anything if one fails;
fix those reviews and run it again. It writes `reviews/SUMMARY.md`: counts per
hub, then fix and drop with their suggestions, keep last.

## 5. Report

Tell the user: how many keep / fix / drop, the main patterns, which
judgements rest only on the static sheet (low confidence), and what needs
Houdini (the review video, `render_clip_review.py`), Isaac Sim or the arm to
confirm. Do not change generator or runtime code; a suggestion is for the
user to apply and rebuild.
