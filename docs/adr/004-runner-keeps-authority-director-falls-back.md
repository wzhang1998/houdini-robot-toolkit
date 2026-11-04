# ADR 004: The Runner keeps the final say; a Director falls back to show.Selector

- Status: Proposed
- Date: 2026-11-04
- Contracts: [Director](../architecture/contracts.md#42-the-runner-keeps-the-final-say)

## Context

The show must keep playing whatever a Director does: hang on a network call,
raise, return garbage, or say nothing. The existing `show.Selector` has run
the show for hours on the real arm and is the behaviour everyone has seen.

## Decision

- `show.Runner` stays the only thing that changes show state (queue, hub,
  pending triggers, faults). A Director can only *ask*: a clip at this hub, a
  kind of clip, a trigger the Runner then routes or refuses as an operator's.
- Every call goes through `GuardedDirector`: a time budget on a worker thread,
  parsing and validation against the current hub and the library, and on
  `timeout` / `error` / `invalid` / `busy` / `defer` the inherited
  `show.Selector.pick` decides. A Director still running past its budget is
  not called again until it returns (no pile-up).
- `BaselineDirector` is `show.Selector` behind the contract, proved identical
  (same segment history, three seeds, 15 minutes, with triggers and operator
  changes). Plugging it in is a no-op; every other Director is measured against it.
- Every decision, used or not, is logged with its reason (`decision_log.md`).

## Consequences

- Introducing a Director cannot stop the show; the worst case is the show as
  it is today, which the log shows as a fallback rate.
- The call happens inside `Runner.step`, i.e. on the stream's tick thread.
  Synchronous calls are fine on a dry clock, in replay and in Isaac, **not in
  the 125 Hz stream**: there the Director must be asked during the running
  clip and `pick()` must only read a ready answer. Until that is built
  ([migration step 6](../architecture/migration.md)), Directors run offline only.
- Two kinds of valid-but-harmful answer are refused by default
  (`TriggerPolicy`, contracts.md §4.6): a second Director scan in a run (the
  paper is exposed once), a trigger repeated inside 120 s, and a `play_clip`
  within `no_repeat`. Anything else valid but poor is not stopped by the
  guard — only by metrics and review.

## Alternatives considered

- **The Director drives the Runner** (it owns the queue). Every Director then
  re-implements routes, sequences and fault handling, and a failure leaves the
  show with no driver.
- **No fallback; hold at the hub until the Director answers.** A stalled model
  becomes a frozen show in front of an audience.
