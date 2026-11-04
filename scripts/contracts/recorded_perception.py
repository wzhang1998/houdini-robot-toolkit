"""RecordedPerception: a Femto recording as a PerceptionSource -- the
reference implementation of the Perception contract, built on what already
reads one (track_replay.frames -> femto_format.read / messages / to_robot).

The recording is TD people_track's Record (or mocap_scenes' simulation): the
Kinect Azure CHOP's channels, a row a frame. Its extrinsic (femto_extrinsic.json
beside it, {"R": 3x3, "p": 3}: q = R c + p) takes the camera's space to the
robot base frame, as track_replay does. The recording's time 0 is the
consumer's `start`; frame k has arrived once now >= start + t_k + latency_s.

    src = RecordedPerception("geo/tracking/femto_rec_....csv")          # extrinsic beside it
    src = RecordedPerception(path, extrinsic="x.json", latency_s=0.09, loop=True, crouch=(13, 18))
    scene = src.poll(now)

    python scripts/contracts/recorded_perception.py      self-test (writes a synthetic recording)
"""

import bisect
import hashlib
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from contracts import perception as PC  # noqa: E402


def _digest(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()[:12]


class RecordedPerception:
    def __init__(self, recording, extrinsic=None, latency_s=0.0, loop=False, start=0.0, crouch=None):
        import track_replay
        self.recording = recording
        self.extrinsic = extrinsic or os.path.join(os.path.dirname(os.path.abspath(recording)), "femto_extrinsic.json")
        if not os.path.isfile(self.extrinsic):
            raise IOError("RecordedPerception: no extrinsic %s (track_replay's femto_extrinsic.json beside the "
                          "recording, or say one)" % self.extrinsic)
        self.frames = track_replay.frames(recording, self.extrinsic, crouch)       # [(t, people, hands)], robot frame
        if not self.frames:
            raise ValueError("RecordedPerception: %s has no frames" % recording)
        t0 = self.frames[0][0]
        self.frames = [(t - t0, p, h) for t, p, h in self.frames]
        self.times = [f[0] for f in self.frames]
        self.length = self.times[-1] + 1.0 / 30.0
        self.latency_s, self.loop, self.start = float(latency_s), bool(loop), float(start)
        self.ext_id = "%s@%s" % (os.path.basename(self.extrinsic), _digest(self.extrinsic))
        self.source = "femto_recording:%s" % os.path.basename(recording)

    def info(self):
        n = len(self.frames)
        rate = (n - 1) / self.times[-1] if n > 1 and self.times[-1] > 0 else 30.0
        return PC.SourceInfo(self.source, "recording", round(rate, 2), self.ext_id, self.latency_s,
                             notes="%d frames, %.1f s%s" % (n, self.length, ", looped" if self.loop else ""))

    def poll(self, now):
        s = now - self.start - self.latency_s                  # the recording's time that has arrived
        if s < 0.0:
            return None
        k_loop = 0
        if self.loop:
            k_loop, s = divmod(s, self.length)
        elif s > self.times[-1] + 1.0:
            s = self.times[-1]                                 # past the end: the last frame stays
        i = bisect.bisect_right(self.times, s) - 1
        if i < 0:
            return None
        t, people, hands = self.frames[i]
        t += k_loop * self.length
        return PC.SceneState.from_tuples(t, people, hands, source=self.source,
                                         received=self.start + t + self.latency_s, extrinsic=self.ext_id)

    def close(self):
        pass

    def duration(self):
        return self.length


def write_synthetic(directory, seconds=6.0, fps=30.0):
    """A small Femto-format recording and its extrinsic in directory (for tests): one person walking
    across the audience side (y = -2 m), a hand up from 2 s to 12 s of every 20 s; a second one from 3 s.
    Returns (csv path, extrinsic path)."""
    import json
    import femto_format as FF
    R = [[-1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, -1.0, 0.0]]      # femto_format's self-test camera
    p = [0.0, -1.0, 2.0]

    def cam(q):                                                     # robot -> camera (R orthogonal)
        return tuple(sum(R[i][j] * (q[i] - p[i]) for i in range(3)) for j in range(3))
    frames = []
    n = int(seconds * fps)
    for k in range(n):
        t = k / fps
        x = -1.0 + 2.0 * k / max(1, n - 1)
        head = (x, -2.0, 1.6)
        wrist = (x + 0.2, -2.0, 1.85 if 2.0 <= t % 20.0 < 12.0 else 1.0)
        bodies = [(3, {"head": cam(head) + (FF.MEDIUM,), "wrist_r": cam(wrist) + (FF.MEDIUM,)})]
        if t >= 3.0:
            bodies.append((8, {"head": cam((0.5, -2.6, 1.7)) + (FF.MEDIUM,)}))
        frames.append((k, t, bodies))
    os.makedirs(directory, exist_ok=True)
    rec = FF.write(os.path.join(directory, "femto_rec_synthetic.csv"), frames, players=2)
    ext = os.path.join(directory, "femto_extrinsic.json")
    with open(ext, "w") as f:
        json.dump({"R": R, "p": p}, f)
    return rec, ext


def self_test():
    import tempfile
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)
    rec, ext = write_synthetic(tempfile.mkdtemp())
    src = RecordedPerception(rec)
    check("a PerceptionSource", isinstance(src, PC.PerceptionSource), src.info())
    check("nothing before the start", src.poll(-0.1) is None)
    s = src.poll(1.0)
    check("the frame at 1 s: one person, the robot frame (2 m in front), a contract-valid scene",
          s is not None and len(s.people) == 1 and abs(s.people[0].y + 2.0) < 1e-6 and PC.validate(s.to_dict()) == [],
          s and PC.validate(s.to_dict()))
    check("... the same numbers as track_replay.frames", s.people[0].p == tuple(src.frames[30][1][0][1:4]))
    m = PC.summarize(src.poll(2.5), now=2.5)
    check("a hand up from 2 s (summarize)", m.hands_up == 1 and m.n_people == 1, m)
    check("two people from 3 s", len(src.poll(3.5).people) == 2)
    lat = RecordedPerception(rec, latency_s=0.1)
    check("latency: a frame arrives that much later", lat.poll(0.05) is None and lat.poll(1.1).t == src.poll(1.0).t)
    lp = RecordedPerception(rec, loop=True)
    check("loop: past the end it starts again, its time running on", lp.poll(src.length + 0.5).t > src.length)
    check("not looped: past the end the last frame stays", src.poll(1e6).t == src.times[-1])
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
