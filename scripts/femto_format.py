"""The Femto Mega's body tracking as TouchDesigner gives it -- the Kinect
Azure CHOP (Hardware: Orbbec, over USB) -- the one format the simulation
and the real camera share, so either feeds the same path:

    frame  timestamp  p1/id  p1/pelvis:tx  p1/pelvis:ty  p1/pelvis:tz  p1/pelvis:confidence  ...  p2/id ...

(read from TD 2025.32280 itself: 32 joints in the Azure Kinect body
tracking's order, World Space on, Confidence on; an empty player slot has
id 0; the bodies fill the slots from p1, so a body leaving moves the others
up.) A recording is a CSV with those channel names as the header, a row a
frame at 30 fps -- what TD's people_track records from the real camera and
what mocap_scenes.py writes as the simulation.

Positions are in the CHOP's space. The extrinsic (R, p) takes them to the
robot base frame, q = R c + p: the simulation's is known; the real one is
solved on site from known floor marks (solve_extrinsic), so the CHOP's axis
convention (TD does not document it) never needs knowing.

What the tracking layer gets of a frame (messages): each body with its id
-- its head (a person) and its higher wrist, if seen (a hand) -- as
/track/people and /track/hands. TD's people_track does the same
(TD-ROBOT-UVSCAN td-modules/people_track/people_track.py; both tested on
the same numbers).

    python scripts/femto_format.py          self-test
"""

import csv
import math
import random
import sys

JOINTS = ("pelvis", "spine_navel", "spine_chest", "neck", "clavicle_l", "shoulder_l", "elbow_l", "wrist_l",
          "hand_l", "handtip_l", "thumb_l", "clavicle_r", "shoulder_r", "elbow_r", "wrist_r", "hand_r", "handtip_r",
          "thumb_r", "hip_l", "knee_l", "ankle_l", "foot_l", "hip_r", "knee_r", "ankle_r", "foot_r", "head", "nose",
          "eye_l", "ear_l", "eye_r", "ear_r")
PLAYERS = 6                       # the CHOP's Max Players for the test
FPS = 30.0
NONE, LOW, MEDIUM, HIGH = 0, 1, 2, 3      # the body tracker's confidence levels (LOW: predicted, not seen)
CONF = {NONE: 0.0, LOW: 0.4, MEDIUM: 0.9, HIGH: 1.0}     # as the tracking layer's conf (its MIN_CONF 0.5)


def header(players=PLAYERS):
    """The CHOP's channel names, in its order."""
    out = ["frame", "timestamp"]
    for k in range(1, players + 1):
        out.append("p%d/id" % k)
        for j in JOINTS:
            out += ["p%d/%s:%s" % (k, j, c) for c in ("tx", "ty", "tz", "confidence")]
    return out


def row(frame, t, bodies, players=PLAYERS):
    """A frame's values in header order. bodies: [(id, {joint: (x, y, z, confidence level)})] -- the slots in
    this order; a joint left out is (0, 0, 0, NONE); slots beyond the bodies are empty (id 0)."""
    out = [frame, round(t, 4)]
    for k in range(players):
        if k < len(bodies):
            bid, joints = bodies[k]
            out.append(int(bid))
            for j in JOINTS:
                x, y, z, c = joints.get(j, (0.0, 0.0, 0.0, NONE))
                out += [round(x, 4), round(y, 4), round(z, 4), int(c)]
        else:
            out += [0] + [0] * (4 * len(JOINTS))
    return out


def write(path, frames, players=PLAYERS):
    """A recording: frames [(frame, t, bodies)] (row's bodies)."""
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header(players))
        for fr, t, bodies in frames:
            w.writerow(row(fr, t, bodies, players))
    return path


def bodies_of(values):
    """[(id, {joint: (x, y, z, level)})] of a frame -- values: {channel name: value} (a CSV row, or a CHOP's
    channels) -- the slots with an id, in slot order."""
    out, k = [], 1
    while "p%d/id" % k in values:
        bid = int(round(float(values["p%d/id" % k] or 0)))
        if bid > 0:
            joints = {}
            for j in JOINTS:
                pre = "p%d/%s:" % (k, j)
                if pre + "tx" in values:
                    conf = values.get(pre + "confidence")
                    joints[j] = (float(values[pre + "tx"]), float(values[pre + "ty"]), float(values[pre + "tz"]),
                                 int(round(float(conf))) if conf not in (None, "") else MEDIUM)
            out.append((bid, joints))
        k += 1
    return out


def read(path):
    """[(frame, t, bodies)] of a recording."""
    out = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            out.append((int(float(r["frame"])), float(r["timestamp"]), bodies_of(r)))
    return out


def to_robot(R, p, c):
    return tuple(p[i] + sum(R[i][j] * c[j] for j in range(3)) for i in range(3))


def messages(bodies, R, p):
    """(people, hands) of a frame's bodies, robot frame: [(id, x, y, z, conf)] -- each body's head and its
    higher wrist, seen or predicted: the conf says which (a predicted one, under the tracking layer's
    MIN_CONF, is left out there -- the lower wrist never stands in for a raised one that is hidden)."""
    people, hands = [], []
    for bid, joints in bodies:
        h = joints.get("head")
        if h is None or h[3] == NONE:
            continue
        people.append((bid,) + tuple(round(v, 4) for v in to_robot(R, p, h[:3])) + (CONF[h[3]],))
        ws = [(to_robot(R, p, w[:3]), w[3]) for w in (joints.get("wrist_l"), joints.get("wrist_r"))
              if w is not None and w[3] > NONE]
        if ws:
            q, lv = max(ws, key=lambda x: x[0][2])
            hands.append((bid,) + tuple(round(v, 4) for v in q) + (CONF[lv],))
    return people, hands


def events(frames, R, p, seed=1, latency=(0.07, 0.11)):
    """track_sim's events of a recording: /track/hands then /track/people a frame, measured at the frame's
    time, arriving latency later (the camera, the body tracker, TD, the network)."""
    rng = random.Random(seed)
    out = []
    t0 = frames[0][1] if frames else 0.0
    for fr, t, bodies in frames:
        tm = t - t0
        people, hands = messages(bodies, R, p)
        arrive = tm + rng.uniform(*latency)
        flat = lambda items: [x for it in items for x in it]      # noqa: E731
        out.append({"t": arrive, "addr": "/track/hands", "args": [round(tm, 4), len(hands)] + flat(hands)})
        out.append({"t": arrive + 1e-4, "addr": "/track/people", "args": [round(tm, 4), len(people)] + flat(people)})
    return out


def _eig_sym(A):
    """(eigenvalues, eigenvectors as columns) of a symmetric 3x3 (Jacobi rotations)."""
    A = [list(map(float, r)) for r in A]
    V = [[1.0 if i == j else 0.0 for j in range(3)] for i in range(3)]
    for _ in range(60):
        if sum(A[i][j] ** 2 for i in range(3) for j in range(i + 1, 3)) < 1e-30:
            break
        for p, q in ((0, 1), (0, 2), (1, 2)):
            if abs(A[p][q]) < 1e-300:
                continue
            th = (A[q][q] - A[p][p]) / (2.0 * A[p][q])
            t = (1.0 if th >= 0 else -1.0) / (abs(th) + math.sqrt(th * th + 1.0))
            c = 1.0 / math.sqrt(t * t + 1.0)
            sn = t * c
            for k in range(3):
                a, b = A[k][p], A[k][q]
                A[k][p], A[k][q] = c * a - sn * b, sn * a + c * b
            for k in range(3):
                a, b = A[p][k], A[q][k]
                A[p][k], A[q][k] = c * a - sn * b, sn * a + c * b
            for k in range(3):
                a, b = V[k][p], V[k][q]
                V[k][p], V[k][q] = c * a - sn * b, sn * a + c * b
    return [A[i][i] for i in range(3)], V


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def solve_extrinsic(pairs, heads):
    """(R, p, rms m) with q = R c + p, from pairs [(c the CHOP's space, q robot frame)] -- someone's ankles on
    3+ floor marks, not in a line -- and their heads there (the CHOP's space). Kabsch (plain Python), both
    ways round the third axis -- a rotation and a mirrored one: the CHOP's handedness is not documented,
    and floor marks lie in one plane, so both can fit them -- keeping, of those that put the heads above
    the feet, the one that fits best. (TD's people_track solves it the same way.)"""
    n = len(pairs)
    if n < 3:
        raise ValueError("solve_extrinsic: 3 marks at least, got %d" % n)
    cc = [sum(c[i] for c, _ in pairs) / n for i in range(3)]
    cq = [sum(q[i] for _, q in pairs) / n for i in range(3)]
    H = [[sum((c[i] - cc[i]) * (q[j] - cq[j]) for c, q in pairs) for j in range(3)] for i in range(3)]
    w, V = _eig_sym([[sum(H[k][i] * H[k][j] for k in range(3)) for j in range(3)] for i in range(3)])
    order = sorted(range(3), key=lambda k: -w[k])
    Vc = [[V[r][k] for r in range(3)] for k in order]                    # V's columns, largest first
    U = []
    for v in Vc[:2]:
        u = [sum(H[r][k] * v[k] for k in range(3)) for r in range(3)]
        nu = math.sqrt(sum(x * x for x in u)) or 1.0
        U.append([x / nu for x in u])
    U.append(_cross(U[0], U[1]))                                         # the plane's normal (marks: a plane)
    cands = []
    for d in (1.0, -1.0):
        R = [[sum(Vc[k][i] * U[k][j] * (d if k == 2 else 1.0) for k in range(3)) for j in range(3)]
             for i in range(3)]                                          # R = V diag(1, 1, d) U^T
        p = [cq[i] - sum(R[i][j] * cc[j] for j in range(3)) for i in range(3)]
        rms = math.sqrt(sum(math.dist(to_robot(R, p, c), q) ** 2 for c, q in pairs) / n)
        rise = sum(to_robot(R, p, h)[2] - to_robot(R, p, c)[2] for h, (c, _) in zip(heads, pairs)) / n
        cands.append((rise <= 0.0, rms, R, p))
    _, rms, R, p = min(cands, key=lambda x: (x[0], x[1]))
    return R, p, rms


def self_test():
    import os
    import tempfile
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    h = header(2)
    check("the header: TD's Kinect Azure CHOP, 2 players, confidence on (260 channels)",
          len(h) == 260 and h[:4] == ["frame", "timestamp", "p1/id", "p1/pelvis:tx"] and h[6] == "p1/pelvis:confidence"
          and h[131] == "p2/id" and h[-1] == "p2/ear_r:confidence", (len(h), h[131], h[-1]))
    # a camera 2 m up at the robot's (0, -1), level, looking along -y: x right = -x, y down = -z, z forward = -y
    R = [[-1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, -1.0, 0.0]]
    P = [0.0, -1.0, 2.0]
    body = {"head": (0.1, 0.4, 0.8, MEDIUM), "wrist_l": (0.3, 0.9, 0.8, MEDIUM), "wrist_r": (-0.1, 0.1, 0.8, LOW)}
    frames = [(0, 0.0, [(7, body), (3, {"head": (-0.5, 0.4, 1.2, MEDIUM)})]), (1, 1 / 30.0, [(3, {"head": (-0.5, 0.4, 1.2, LOW)})])]
    path = os.path.join(tempfile.mkdtemp(), "rec.csv")
    write(path, frames, players=3)
    back = read(path)
    check("a recording read back: the bodies in their slots, their ids, an empty slot left out",
          [[b[0] for b in f[2]] for f in back] == [[7, 3], [3]] and back[0][2][0][1]["head"] == (0.1, 0.4, 0.8, MEDIUM),
          [[b[0] for b in f[2]] for f in back])
    people, hands = messages(back[0][2], R, P)
    check("messages: the heads in the robot frame, conf by the level",
          people == [(7, -0.1, -1.8, 1.6, 0.9), (3, 0.5, -2.2, 1.6, 0.9)], people)
    check("... the hand: the higher wrist, even if only predicted (the right one: its conf says so)",
          hands == [(7, 0.1, -1.8, 1.9, 0.4)], hands)
    people, _ = messages(back[1][2], R, P)
    check("... a predicted head: there, with a conf under the tracking layer's floor", people == [(3, 0.5, -2.2, 1.6, 0.4)],
          people)
    ev = events(back, R, P)
    check("events: /track/hands then /track/people a frame, 70-110 ms late",
          [e["addr"] for e in ev] == ["/track/hands", "/track/people"] * 2 and 0.07 <= ev[0]["t"] <= 0.11
          and ev[1]["args"][:3] == [0.0, 2, 7], [e["args"][:3] for e in ev])
    rng = random.Random(3)
    marks = [(0.0, -1.8), (-0.6, -1.8), (0.6, -1.8), (0.0, -1.45)]            # floor marks: one plane
    feet_q = [(x, y, 0.08) for x, y in marks]
    head_q = [(x, y, 1.65) for x, y in marks]
    Rt = [[0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]]
    pt = [0.4, -1.2, 2.3]
    for label, Rc in (("a right-handed CHOP space", Rt),
                      ("a left-handed one (y flipped)", [[r[0], -r[1], r[2]] for r in Rt])):
        def inv(q, Rc=Rc):                                     # robot -> the CHOP's space (Rc orthogonal)
            return tuple(sum(Rc[i][j] * (q[i] - pt[i]) for i in range(3)) for j in range(3))
        pairs = [(tuple(x + rng.gauss(0, 0.01) for x in inv(q)), q) for q in feet_q]
        heads = [inv(q) for q in head_q]
        Rs, ps, rms = solve_extrinsic(pairs, heads)
        up = to_robot(Rs, ps, inv((0.3, -2.0, 1.7)))
        check("solve_extrinsic, %s: 4 floor marks, 1 cm noise -> back to within 3 cm, heads up" % label,
              math.dist(up, (0.3, -2.0, 1.7)) < 0.03 and rms < 0.03,
              (round(math.dist(up, (0.3, -2.0, 1.7)), 3), round(rms, 3)))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
