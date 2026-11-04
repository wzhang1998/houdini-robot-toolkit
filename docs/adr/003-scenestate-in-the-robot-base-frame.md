# ADR 003: Perception hands over people in the robot base frame

- Status: Proposed
- Date: 2026-11-04
- Contracts: [Perception](../architecture/contracts.md#2-perception--scenestate-in-the-robot-base-frame)

## Context

The Femto's body tracking reaches the show as `/track/people` and
`/track/hands` with every point already in the robot base frame:
`femto_format.messages` (and TD's people_track) apply the extrinsic, solved on
site from floor marks, before anything is sent. Eight modules consume those
messages and none of them knows which camera it was. The camera's own axis
convention (the CHOP's handedness is undocumented) never leaves the producer.

## Decision

`SceneState` (`motionlab.scene/1`) is that message pair, typed: heads and
higher hands, ids, confidences, measurement time, **robot base frame only**,
plus the id of the extrinsic that produced it. The producer owns the
extrinsic and its calibration; a consumer never transforms camera
coordinates. `femto_format.py` is the reference producer. A Director sees only
`summarize(scene)` — counts and the nearest person — never the skeleton.

## Consequences

- Swapping the Femto for an OAK-D is one new producer (DepthAI → the same OSC)
  and its own calibration; tracking, engage, the Directors and the logs stay.
- A bad calibration is invisible downstream except as people in odd places;
  the extrinsic id in every frame and decision is how it is traced, and the
  tracking layer's bounded, collision-checked offsets are how it is contained.
- The contract carries only heads and one hand. A behaviour needing more
  (gaze direction, a pointing arm) needs a new field or message — a contract
  change, on purpose.

## Alternatives considered

- **Camera-frame points plus a transform in the consumer.** Every consumer
  needs the extrinsic and the handedness; a stale extrinsic becomes many bugs.
- **Full skeletons in the contract.** Ties consumers to a body model (32
  Azure Kinect joints) that an OAK-D pipeline may not produce, and puts
  identifiable data into logs.
