# ADR 002: RobotBackend is the duck type the stream already calls

- Status: Proposed
- Date: 2026-11-04
- Contracts: [Robot](../architecture/contracts.md#1-robot--profile--robotbackend)

## Context

`show_stream.stream`, its feedback `Link`, `recover`, `fairino_player.goto` and
`move_checked` call ten methods on a controller object: `model`, `joints`,
`error_code`, `reset_errors`, `prepare`, `move_to`, `servo_start`, `servo_j`,
`servo_end`, `stop` (+ optional `queue_length`, `frames`, `diagnose`). The
stream already takes that object as a parameter, and its self-test drives it
with a fake. The names are Fairino's (`ServoJ`, `MoveJ`).

## Decision

`contracts.robot.RobotBackend` is exactly that set, as a `typing.Protocol`, with
the semantics written down (units, frames, the `[ret, main, sub]` error shape,
"stop must work from the other connection"). `fairino_player.Controller`
satisfies it unchanged — checked by the conformance suite — and a UFACTORY
backend implements the same ten methods.

## Consequences

- No change to the stream, the player or the Controller to adopt it; the 8
  driver call sites keep working, and a new backend plugs into
  `show_stream.stream` the way `SimBackend` does in the conformance run.
- Fairino vocabulary leaks into the interface (`servo_j(q, cmd_t, cmd_id)`).
  A controller without a per-point duration (the xArm's `set_servo_angle_j`)
  ignores `cmd_t`, and its pacing is the sender's clock alone — written in the
  porting notes rather than hidden.
- What is *not* in the interface — the subnet test, timeouts, fault codes,
  `playback_ppm` — goes into `BackendInfo` / `FaultPolicy` and the factory,
  and has to be moved there from `show_stream` / `fairino_player` ([migration step 3](../architecture/migration.md)).

## Alternatives considered

- **A clean, vendor-neutral API** (`stream_begin`, `send(q, t)`, …) with an
  adapter for the Controller. Nicer names, but every caller changes and the
  stream's behaviour, tuned over many hours on the real arm, gets re-plumbed
  for no functional gain.
- **ROS 2 `ros2_control` / a JointTrajectory action.** The right long-term
  boundary if the stack moves to ROS (motion-lab-architecture.md, Stage 4);
  a ROS backend can sit behind this same Protocol when it exists.
