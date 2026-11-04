# ADR 001: A Director outputs an enum-limited intent, never joint angles

- Status: Proposed
- Date: 2026-11-04
- Contracts: [Director](../architecture/contracts.md#4-director--enum-limited-intent-the-runner-executes)

## Context

We want "what plays next" to be replaceable: today a weighted random pick
(`show.Selector`), tomorrow perhaps a learned policy or a language model that
reads the room. Such a component can be slow, wrong, or confidently
nonsensical, and it may be changed by people who never read the safety audit.

Everything the arm does in a show was **built and checked before the show**:
each clip generated, measured against the joint envelope, collision-checked in
the room with margins, range-checked, and every route between hubs planned and
timed (`show.py build`; `docs/safety_audit_2026-09-29.md`). At show time
nothing is planned, only chosen. That property is why the show can run for
hours unattended, and it is the property a Director must not be able to break.

## Decision

A Director's output is an `Intent` of one of four kinds — `play_clip` (an idle
clip at the current hub, by id), `play_like` (an action / family / energy the
Selector resolves at this hub), `trigger` (a name `/robot/trigger` already
accepts), `defer` — plus a short reason. Nothing else parses
(`Intent.from_dict` refuses unknown kinds, unknown fields, and fields of the
wrong type). There is no way to say
a joint angle, a Cartesian pose, a speed, a duration, or a route.

## Consequences

- The worst a Director can do is choose badly among checked motions. It cannot
  move the arm somewhere unchecked, faster than built, or through an unplanned
  path; the stream's per-tick `Guard` remains as a second line regardless.
- The Director's quality is measurable from a log (`decision_log.md`) and can
  be compared with the baseline offline, because its whole effect is a
  sequence of names.
- Expressiveness is bounded by the library. A Director that wants a motion the
  library lacks must cause it to be *built* (offline: generator parameters, a
  new family), not improvise it. Live, continuous reaction to a person stays
  where it is: the tracking layer's bounded, collision-checked gaze offsets
  (`tracking.py`), which have their own limits and tests.
- Adding a kind (e.g. `go_hub`) is a contract change with a version bump, a
  Runner hook and its own checks — deliberately not cheap.

## Alternatives considered

- **Joint targets or poses from the Director**, checked at runtime. Would need
  collision, envelope and route checks inside the 125 Hz loop for arbitrary
  input; moves the build-time guarantee to runtime and makes every Director a
  safety-critical component.
- **Free-text or open JSON** interpreted by the Runner. Unbounded surface; every
  new phrasing is a new behaviour no test covers.
- **Weights only** (the Director returns a score per clip). Safe, but cannot
  express a trigger, and an opaque vector is hard to log and review. `play_like`
  covers the useful part.
