# Architecture decision records

One file per decision: the context, the decision, what it costs, and what
was considered instead. Status is **Proposed** until you accept it (edit the
status line and the date); a superseded ADR stays, marked with what replaced it.

| # | Decision | Status |
|---|---|---|
| [001](001-director-outputs-intent-not-joints.md) | A Director outputs an enum-limited intent, never joint angles or poses | Proposed |
| [002](002-robotbackend-mirrors-the-existing-controller.md) | `RobotBackend` is the duck type the stream already calls, not a new API | Proposed |
| [003](003-scenestate-in-the-robot-base-frame.md) | Perception hands over people in the robot base frame; the producer owns the extrinsic | Proposed |
| [004](004-runner-keeps-authority-director-falls-back.md) | The Runner keeps the final say; a late, failed or invalid Director falls back to `show.Selector` | Proposed |
| [005](005-clips-are-solved-per-arm.md) | Clips are solved per arm from arm-neutral sources; joint angles are never retargeted | Proposed |
| [006](006-split-trajectory-math-from-fairino-driver.md) | Split the trajectory math out of `fairino_player.py` | Proposed (not done) |

Template: Context · Decision · Consequences · Alternatives considered.
