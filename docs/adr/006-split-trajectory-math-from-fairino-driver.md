# ADR 006: Split the trajectory math out of fairino_player.py

- Status: Proposed — not done in this change (it would edit runtime code)
- Date: 2026-11-04
- See: [coupling.md §B1](../architecture/coupling.md#b1-the-arm-fairino_player)

## Context

`fairino_player.py` holds two things: the Fairino XML-RPC driver
(`Controller`, `goto`, `play`, `move_checked`, the CLI) and arm-neutral
trajectory math (`load_csv`, `Path` — PCHIP —, `condition`, `limiting`,
`need_profile`, `_peaks`). 23 files import it: 8 for the driver, **15 for the
math only** (the generators, `motion_clip`, `show`, the HDA callbacks).

## Decision (proposed)

Move the math to `scripts/trajectory.py` unchanged; `fairino_player` imports it
and re-exports every moved name, so no importer has to change in the same
commit. Importers move to `trajectory` one at a time afterwards.

## Consequences

- A UFACTORY backend and player reuse the conditioning without importing the
  Fairino driver; generators stop depending on a driver file.
- Behaviour-preserving by construction (same code, re-exported); checked by
  `fairino_player.py --self-test`, `motion_clip.py`, and a rebuild of `party`
  diffed against the tracked compiled show.
- The Houdini HDA modules (`hda/csvio_module.py`, `callbacks/ctrl_retime.py`)
  import it too: their copies inside the `.hdalc` must be updated when they
  switch (README: keep `scripts/` and `otls/` in sync).

## Alternatives considered

- **Leave it.** Costs nothing now; every new backend then forks or imports the
  Fairino file.
- **Move the driver out instead** (`fairino_backend.py`). Same split, but the 8
  driver call sites and the CLI path in `play.py` / `play_ui.py` change.
