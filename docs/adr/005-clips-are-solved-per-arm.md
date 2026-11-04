# ADR 005: Clips are solved per arm; joint angles are never retargeted

- Status: Proposed
- Date: 2026-11-04
- Contracts: [Motion](../architecture/contracts.md#3-motion--what-is-the-shows-what-is-one-arms-solution)

## Context

The library is generated (choreo phrases, gestures, paths, showpieces, the
scan) from parameters and Cartesian hubs, then solved with the FR20's IK,
retimed to its envelope and checked in the room. The compiled show stores the
result: FR20 joint trajectories. A second arm has different link lengths,
joint ranges, wrist geometry and singularities; the same joint angles put its
tool somewhere else entirely.

## Decision

The arm-neutral sources — show config (Cartesian hubs, zones, stage, canvas,
scan, selection), generator parameters, labels of intent — are the library.
Joint trajectories, measured energy and clearances are **build products for one
profile**, rebuilt and re-checked for each arm by the same build. Nothing maps
one arm's joint angles onto another's. FR20 joint values that today sit in the
show config (`rest.q`, `near` seeds, `range.j1_deg`) move to a per-arm overlay.

## Consequences

- One pipeline, one set of checks, for every arm; a clip on the UF850 carries
  the same guarantee as on the FR20.
- A clip may not exist on the other arm (out of reach, through a singularity):
  the build drops it and says so; the library at a hub may thin out, and
  `show.py report` shows what is missing.
- The "same" clip looks different on another arm — the intent (Laban action,
  family) is kept, the exact path is not. For a much smaller arm the Cartesian
  sources themselves must be redrawn (porting.md step 0).
- The build needs the arm's URDF and a Python IK (porting.md steps 2–3); FBIK
  in Houdini is not enough.

## Alternatives considered

- **Joint-space retargeting** (map ranges, scale). Fast, but no guarantee
  about where the tool goes, so every clip would need the full check anyway,
  and most would fail it.
- **Cartesian retargeting of the compiled TCP path** (re-solve the recorded
  path on the new arm). Keeps the path exactly, but loses the generators'
  per-arm choices (reach, the hub's own pose, J6 cable range) and still
  re-checks everything; running the generators is simpler and already exists.
