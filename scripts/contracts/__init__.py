"""Contracts between the show and what it runs on: the arm, the camera, and
whoever chooses the next motion. See docs/architecture/contracts.md.

    robot.py                RobotBackend (+ BackendFactory) and the profile's part of the Robot contract
    perception.py           SceneState (robot base frame), PerceptionSource, summarize()
    director.py             Intent (an enum-limited choice), DirectorInput, Director, GuardedDirector, DirectorSelector
    decision_log.py         one JSON line a decision (motionlab.decision/1): write, read, compare
    sim_backend.py          SimBackend: a RobotBackend of pure kinematics, no hardware
    recorded_perception.py  RecordedPerception: a Femto recording (track_replay / femto_format) as a PerceptionSource
    baseline_director.py    BaselineDirector: show.Selector's weighted random choice as a Director
    director_replay.py      two Directors on the same show and the same recording, a decision log each, compared
    conformance.py          the acceptance tests any new backend, source or Director runs

Nothing here is imported by the show, the stream or the player: adding this
package changes no runtime behaviour. Every module is plain Python (no hou,
no Isaac); the tests run as

    python scripts/contracts/conformance.py
"""
