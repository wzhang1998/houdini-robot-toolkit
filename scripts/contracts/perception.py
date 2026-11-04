"""The Perception contract: what any camera gives the show, whatever the
camera. docs/architecture/contracts.md, Perception.

A SceneState is one frame: the people (each one's head) and their hands
(each one's higher wrist), in the ROBOT BASE frame -- the URDF's base_link,
Z up, metres -- with a confidence each and the time it was measured. That
is exactly what /track/people and /track/hands carry today (femto_format.messages
makes them from the Femto's 32 joints; TD's people_track sends them;
track_osc.people_of reads them), given a type, a schema id and the extrinsic
that put it in the base frame. A camera is swapped by writing a new
producer of SceneState -- nothing that consumes one changes.

    scene = SceneState.from_osc(t, people_args, hands_args, source="femto")
    scene.to_osc()            -> [("/track/hands", args), ("/track/people", args)]  (track_replay.osc_frame's)
    validate(scene.to_dict()) -> [errors]
    summarize(scene, now)     -> the Director's view: counts, the nearest person, hands raised -- no skeletons

    python scripts/contracts/perception.py      self-test
"""

import math
import os
import sys
from dataclasses import asdict, dataclass, field
from typing import List, Optional, Protocol, Tuple, runtime_checkable

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

SCHEMA = "motionlab.scene/1"
FRAME = "robot_base"              # the URDF's base_link: Z up, metres (femto_format.to_robot's target)
MIN_CONF = 0.5                    # tracking.MIN_CONF: below it a point is predicted, not seen


@dataclass(frozen=True)
class Point:
    id: int                       # the tracker's body id (> 0); the same id across frames is the same person
    x: float
    y: float
    z: float
    conf: float                   # 0..1 (femto_format.CONF: 0.4 predicted, 0.9 seen)

    @property
    def p(self):
        return (self.x, self.y, self.z)


@dataclass(frozen=True)
class SceneState:
    t: float                                  # when measured, s, on the source's timeline
    people: Tuple[Point, ...] = ()            # heads
    hands: Tuple[Point, ...] = ()             # each body's higher wrist, if seen or predicted
    source: str = ""                          # which producer ("femto_recording:<file>", "osc:9011", ...)
    received: Optional[float] = None          # when the consumer got it, on the consumer's clock
    extrinsic: Optional[str] = None           # which camera -> base transform made it (a file, a digest)
    frame: str = FRAME
    schema: str = SCHEMA

    def to_dict(self):
        d = asdict(self)
        d["people"] = [asdict(p) for p in self.people]
        d["hands"] = [asdict(p) for p in self.hands]
        return d

    @staticmethod
    def from_dict(d):
        return SceneState(float(d["t"]), tuple(Point(**p) for p in d.get("people", ())),
                          tuple(Point(**p) for p in d.get("hands", ())), d.get("source", ""), d.get("received"),
                          d.get("extrinsic"), d.get("frame", FRAME), d.get("schema", SCHEMA))

    @staticmethod
    def from_tuples(t, people, hands, **kw):
        """From femto_format.messages / track_replay.frames' [(id, x, y, z, conf)]."""
        mk = lambda items: tuple(Point(int(i), float(x), float(y), float(z), float(c)) for i, x, y, z, c in items)  # noqa: E731
        return SceneState(float(t), mk(people), mk(hands), **kw)

    @staticmethod
    def from_osc(t, people_args, hands_args=None, **kw):
        """From /track/people (and /track/hands) arguments: t n (id x y z conf) * n."""
        def of(args):
            if not args:
                return []
            n = int(args[1])
            return [tuple(args[2 + 5 * k:7 + 5 * k]) for k in range(n)]
        return SceneState.from_tuples(t, of(people_args), of(hands_args), **kw)

    def to_osc(self):
        """[(address, args)] as TD's people_track sends a frame (track_replay.osc_frame): the hands first."""
        flat = lambda items: [v for p in items for v in (p.id, p.x, p.y, p.z, p.conf)]  # noqa: E731
        return [("/track/hands", [round(self.t, 4), len(self.hands)] + flat(self.hands)),
                ("/track/people", [round(self.t, 4), len(self.people)] + flat(self.people))]


def validate(d) -> List[str]:
    """Errors of a SceneState as a dict ([] when it keeps the contract)."""
    errs = []
    if d.get("schema") != SCHEMA:
        errs.append("schema %r, expected %r" % (d.get("schema"), SCHEMA))
    if d.get("frame") != FRAME:
        errs.append("frame %r: a SceneState is in the robot base frame (%s)" % (d.get("frame"), FRAME))
    t = d.get("t")
    if not isinstance(t, (int, float)) or not math.isfinite(t):
        errs.append("t %r is not a finite time" % (t,))
    for key in ("people", "hands"):
        seen = set()
        for p in d.get(key, ()):
            if not (isinstance(p.get("id"), int) and p["id"] > 0):
                errs.append("%s: id %r must be an int > 0" % (key, p.get("id")))
            if p.get("id") in seen:
                errs.append("%s: id %r twice in one frame" % (key, p.get("id")))
            seen.add(p.get("id"))
            if not all(isinstance(p.get(c), (int, float)) and math.isfinite(p[c]) for c in "xyz"):
                errs.append("%s %r: position not finite" % (key, p.get("id")))
            elif max(abs(p[c]) for c in "xyz") > 50.0:
                errs.append("%s %r: %.1f m from the base -- metres, robot frame?" % (
                    key, p.get("id"), max(abs(p[c]) for c in "xyz")))
            c = p.get("conf")
            if not isinstance(c, (int, float)) or not 0.0 <= c <= 1.0:
                errs.append("%s %r: conf %r not in [0, 1]" % (key, p.get("id"), c))
    pid = {p.get("id") for p in d.get("people", ())}
    stray = [h.get("id") for h in d.get("hands", ()) if h.get("id") not in pid]
    if stray:
        errs.append("hands of nobody in people: %s" % stray)
    return errs


@dataclass
class SourceInfo:
    source_id: str
    kind: str                         # "recording", "live_osc", "sim"
    rate_hz: float = 30.0
    extrinsic: Optional[str] = None
    latency_s: float = 0.0            # measurement to arrival the source adds (a recording replayed: what it is told)
    frame: str = FRAME
    notes: str = ""


@runtime_checkable
class PerceptionSource(Protocol):
    def info(self) -> SourceInfo:
        """What the source is; the frame is always FRAME."""

    def poll(self, now: float) -> Optional[SceneState]:
        """The newest frame that has arrived by `now` (the consumer's clock, s), or None before the
        first. Never blocks: the show calls it from its own loop."""

    def close(self) -> None: ...


# --------------------------------------------------------------------------
# the Director's view of a scene
# --------------------------------------------------------------------------

@dataclass
class SceneSummary:
    """What a Director is shown of the room: a few numbers, never the skeletons (smaller, stable
    under a camera swap, and nothing in a decision log identifies anyone)."""
    n_people: int = 0                         # with a head over MIN_CONF
    n_predicted: int = 0                      # heads under it (hidden, predicted)
    nearest: Optional[dict] = None            # {"id", "dist_m" (floor distance to the base), "bearing_deg", "height_m"}
    hands_up: int = 0                         # people whose higher hand is above their head
    ids: List[int] = field(default_factory=list)
    age_s: Optional[float] = None             # now - the frame's arrival (None: never had one)
    stale: bool = True                        # no frame, or older than STALE_S

    def to_dict(self):
        return asdict(self)


STALE_S = 1.0


def summarize(scene: Optional[SceneState], now: Optional[float] = None) -> SceneSummary:
    if scene is None:
        return SceneSummary()
    seen = [p for p in scene.people if p.conf >= MIN_CONF]
    hands = {h.id: h for h in scene.hands}
    nearest = None
    if seen:
        p = min(seen, key=lambda p: math.hypot(p.x, p.y))
        nearest = {"id": p.id, "dist_m": round(math.hypot(p.x, p.y), 3),
                   "bearing_deg": round(math.degrees(math.atan2(p.y, p.x)), 1), "height_m": round(p.z, 3)}
    up = sum(1 for p in seen if p.id in hands and hands[p.id].z > p.z)
    ref = scene.received if scene.received is not None else scene.t
    age = None if now is None else round(max(0.0, now - ref), 3)
    return SceneSummary(len(seen), len(scene.people) - len(seen), nearest, up, sorted(p.id for p in seen), age,
                        age is not None and age > STALE_S)


class OscPerception:
    """The live door: /track/people + /track/hands on a UDP port (track_osc.TrackIn), as a
    PerceptionSource. Needs python-osc."""

    def __init__(self, port, host="127.0.0.1", extrinsic=None):
        import track_osc
        self.rx, self.port, self.extrinsic = track_osc.TrackIn(port, host), port, extrinsic
        self.last = None

    def info(self):
        return SourceInfo("osc:%d" % self.port, "live_osc", 30.0, self.extrinsic,
                          notes="TD people_track (Femto) or track_replay; the base frame is the sender's job")

    def poll(self, now):
        got = self.rx.poll()
        if got is not None:
            people, hands = got
            # track_osc drops the frame's t; the arrival time stands in for it
            self.last = SceneState.from_tuples(now, people, hands, source="osc:%d" % self.port, received=now,
                                               extrinsic=self.extrinsic)
        return self.last

    def close(self):
        try:
            self.rx.sock.close()
        except Exception:
            pass


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    s = SceneState.from_tuples(1.5, [(4, 0.1, -2.0, 1.6, 0.9), (7, 1.0, -1.0, 1.7, 0.4)],
                               [(4, 0.3, -2.0, 1.9, 0.9)], source="test")
    check("a frame keeps the contract", validate(s.to_dict()) == [], validate(s.to_dict()))
    check("round trip through a dict", SceneState.from_dict(s.to_dict()) == s)
    import track_replay
    want = track_replay.osc_frame(1.5, [(4, 0.1, -2.0, 1.6, 0.9), (7, 1.0, -1.0, 1.7, 0.4)], [(4, 0.3, -2.0, 1.9, 0.9)])
    check("to_osc is what TD's people_track sends (track_replay.osc_frame)", s.to_osc() == want, s.to_osc())
    back = SceneState.from_osc(1.5, want[1][1], want[0][1], source="test")
    check("... and from_osc reads it back", back == s, back)
    import track_osc
    check("... as track_osc.people_of reads it", [(p.id, p.x, p.y, p.z, p.conf) for p in back.people]
          == track_osc.people_of(want[1][1]))
    bad = dict(s.to_dict(), frame="camera")
    bad["people"] = bad["people"] + [dict(bad["people"][0])]
    bad["hands"] = [dict(bad["hands"][0], id=99, conf=1.5)]
    errs = validate(bad)
    check("the camera's frame, a duplicate id, a hand of nobody, a conf over 1: all refused", len(errs) == 4, errs)
    m = summarize(s, now=1.7)
    check("summary: one seen, one predicted, the nearest's distance and bearing, a hand up",
          m.n_people == 1 and m.n_predicted == 1 and m.nearest["id"] == 4 and abs(m.nearest["dist_m"] - 2.002) < 1e-3
          and m.hands_up == 1 and m.age_s == 0.2 and not m.stale, m)
    check("no frame: an empty, stale summary", summarize(None).stale and summarize(None).n_people == 0)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
