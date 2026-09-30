"""CMU motion capture (mocap.cs.cmu.edu, ASF/AMC) as the points tracking
needs: each frame's head and wrists in metres, Z up -- real people walking,
standing, waving, talking with their hands, for testing the tracking layer
(track_sim's mocap scenarios) before the Femto sees anyone.

    sk = read_asf("geo/mocap/cmu/141.asf")
    frames = read_amc("geo/mocap/cmu/141_16.amc")
    pts = points(sk, frames[0])          # {"head": (x, y, z), "lwrist": ..., "rwrist": ..., "root": ...}
    t, tracks = track(sk, frames, fps=30)   # the clip at 30 fps (from 120)

    python scripts/mocap_cmu.py --self-test

Units: the ASF's lengths are inches / 0.45 (CMU); here metres. Axes: CMU's
Y up becomes Z up (x, y, z) -> (x, -z, y), right-handed both.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
CMU_DIR = os.path.join(ROOT, "geo", "mocap", "cmu")
SCALE = 0.0254 / 0.45                  # the ASF's length unit to metres
CMU_FPS = 120.0
POINTS = ("head", "lwrist", "rwrist", "root", "lclavicle", "rclavicle")
BONES = ("root", "lowerback", "upperback", "thorax", "lowerneck", "upperneck", "head",
         "lclavicle", "lhumerus", "lradius", "lwrist", "lhand", "lfingers", "lthumb",
         "rclavicle", "rhumerus", "rradius", "rwrist", "rhand", "rfingers", "rthumb",
         "lhipjoint", "lfemur", "ltibia", "lfoot", "rhipjoint", "rfemur", "rtibia", "rfoot")


def _rot(axis, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    if axis == "x":
        return [[1, 0, 0], [0, c, -s], [0, s, c]]
    if axis == "y":
        return [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def _mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _vec(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def _t(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def _euler(x, y, z):
    """ASF's XYZ order: Rz . Ry . Rx (x applied first)."""
    return _mul(_rot("z", z), _mul(_rot("y", y), _rot("x", x)))


def read_asf(path):
    """{"bones": {name: {"dir", "len", "C", "dof"}}, "children": {name: [..]}} of an ASF skeleton."""
    bones, children, sec, cur = {}, {}, None, None
    for raw in open(path, encoding="latin1"):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith(":"):
            sec = line.split()[0][1:]
            continue
        f = line.split()
        if sec == "bonedata":
            if f[0] == "begin":
                cur = {"dof": []}
            elif f[0] == "end":
                bones[cur["name"]] = cur
            elif f[0] == "name":
                cur["name"] = f[1]
            elif f[0] == "direction":
                cur["dir"] = [float(x) for x in f[1:4]]
            elif f[0] == "length":
                cur["len"] = float(f[1]) * SCALE
            elif f[0] == "axis":
                cur["C"] = _euler(*[float(x) for x in f[1:4]])
            elif f[0] == "dof":
                cur["dof"] = f[1:]
        elif sec == "hierarchy" and f[0] not in ("begin", "end"):
            children.setdefault(f[0], []).extend(f[1:])
    return {"bones": bones, "children": children}


def read_amc(path):
    """[{bone: [values]}] of an AMC motion, one dict a frame (root: tx ty tz rx ry rz)."""
    frames, cur = [], None
    for raw in open(path, encoding="latin1"):
        line = raw.strip()
        if not line or line.startswith("#") or line.startswith(":"):
            continue
        f = line.split()
        if len(f) == 1 and f[0].isdigit():
            cur = {}
            frames.append(cur)
        elif cur is not None:
            cur[f[0]] = [float(x) for x in f[1:]]
    return frames


def _zup(p):
    return (p[0], -p[2], p[1])


def points(sk, frame, want=POINTS):
    """{point: (x, y, z) m, Z up}: the end of each named bone ("root": the pelvis), by forward kinematics."""
    r = frame["root"]
    pos = {"root": [r[0] * SCALE, r[1] * SCALE, r[2] * SCALE]}
    rot = {"root": _euler(r[3], r[4], r[5])}
    stack = ["root"]
    while stack:
        par = stack.pop()
        for ch in sk["children"].get(par, []):
            b = sk["bones"][ch]
            v = {"rx": 0.0, "ry": 0.0, "rz": 0.0}
            for k, a in zip(b["dof"], frame.get(ch, [])):
                v[k] = a
            m = _mul(rot[par], _mul(b["C"], _mul(_euler(v["rx"], v["ry"], v["rz"]), _t(b["C"]))))
            rot[ch] = m
            pos[ch] = [p + b["len"] * d for p, d in zip(pos[par], _vec(m, b["dir"]))]
            stack.append(ch)
    return {k: _zup(pos[k]) for k in want if k in pos}


def track(sk, frames, fps=30.0, want=POINTS):
    """(times s, [points a frame]) of the clip resampled to fps (every 120/fps-th frame)."""
    step = max(1, int(round(CMU_FPS / fps)))
    out = [points(sk, frames[i], want) for i in range(0, len(frames), step)]
    return [i / fps for i in range(len(out))], out


def clip(name, fps=30.0, root=CMU_DIR, want=POINTS):
    """A clip by its CMU name ("141_16"): (times, points a frame)."""
    subject = name.split("_")[0]
    return track(read_asf(os.path.join(root, subject + ".asf")), read_amc(os.path.join(root, name + ".amc")), fps,
                 want)


def _mix(a, b, f):
    return tuple(x + (y - x) * f for x, y in zip(a, b))


def k4a(pts):
    """The Azure Kinect body tracking's 32 joints (femto_format.JOINTS) of a frame's bone ends (points with
    want=BONES; Z up): each where that tracker puts it -- a bone's end, or between two; the face (nose,
    eyes, ears) about the head, facing where the shoulders face."""
    P = pts
    out = {"pelvis": P["root"], "spine_navel": P["lowerback"], "spine_chest": P["upperback"], "neck": P["lowerneck"],
           "head": _mix(P["upperneck"], P["head"], 0.5)}
    for s in "lr":
        out.update({"clavicle_" + s: _mix(P["thorax"], P[s + "clavicle"], 0.35), "shoulder_" + s: P[s + "clavicle"],
                    "elbow_" + s: P[s + "humerus"], "wrist_" + s: P[s + "radius"], "hand_" + s: P[s + "hand"],
                    "handtip_" + s: P[s + "fingers"], "thumb_" + s: P[s + "thumb"], "hip_" + s: P[s + "hipjoint"],
                    "knee_" + s: P[s + "femur"], "ankle_" + s: P[s + "tibia"], "foot_" + s: P[s + "foot"]})
    lr = (P["rclavicle"][0] - P["lclavicle"][0], P["rclavicle"][1] - P["lclavicle"][1])
    n = math.hypot(*lr) or 1.0
    right = (lr[0] / n, lr[1] / n, 0.0)
    fwd = (-right[1], right[0], 0.0)                       # up x right
    h = out["head"]
    at = lambda f, r, u: tuple(h[i] + f * fwd[i] + r * right[i] + (u if i == 2 else 0.0) for i in range(3))  # noqa: E731
    out.update({"nose": at(0.10, 0.0, -0.02), "eye_l": at(0.08, -0.03, 0.03), "eye_r": at(0.08, 0.03, 0.03),
                "ear_l": at(0.0, -0.075, 0.0), "ear_r": at(0.0, 0.075, 0.0)})
    return out


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    if not os.path.exists(os.path.join(CMU_DIR, "07_01.amc")):
        print("skip: the CMU clips are not in %s" % CMU_DIR)
        return 0
    t, tr = clip("07_01")
    head = [p["head"] for p in tr]
    feet = min(p["root"][2] for p in tr)
    dist = math.dist(head[0][:2], head[-1][:2])
    speed = dist / t[-1]
    check("a walk (07_01): the head 1.5-1.9 m up (Z), above the pelvis", all(1.3 < h[2] - feet + 0.95 < 2.0 for h in head)
          and all(p["head"][2] > p["root"][2] + 0.4 for p in tr), (round(head[0][2], 2), round(feet, 2)))
    check("... walking at 0.9-1.8 m/s", 0.9 < speed < 1.8, round(speed, 2))
    check("... the wrists below the shoulders", all(max(p["lwrist"][2], p["rwrist"][2]) < p["lclavicle"][2]
                                                  for p in tr), None)
    t, tr = clip("141_16")
    up = [max(p["lwrist"][2], p["rwrist"][2]) > p["lclavicle"][2] for p in tr]
    check("a wave (141_16): a wrist above the shoulders a good part of the clip", sum(up) > 0.3 * len(up),
          "%d of %d frames" % (sum(up), len(up)))
    hi = [p["rwrist"] if p["rwrist"][2] > p["lwrist"][2] else p["lwrist"] for p, u in zip(tr, up) if u]
    swing = max(max(h[i] for h in hi) - min(h[i] for h in hi) for i in range(2)) if hi else 0.0
    check("... swinging side to side over 0.15 m", swing > 0.15, round(swing, 2))
    t, tr = clip("141_20")
    moved = math.dist(tr[0]["root"][:2], tr[-1]["root"][:2])
    check("waiting (141_20): stays within 0.5 m", moved < 0.5, round(moved, 2))
    import femto_format as FF
    t, tr = clip("07_01", want=BONES)
    ks = [k4a(p) for p in tr]
    k = ks[len(ks) // 2]
    check("the Azure Kinect's 32 joints of a CMU frame", sorted(k) == sorted(FF.JOINTS), sorted(set(FF.JOINTS) ^ set(k)))
    check("... head over neck over chest over pelvis over the knees over the ankles",
          k["head"][2] > k["neck"][2] > k["spine_chest"][2] > k["pelvis"][2] > k["knee_l"][2] > k["ankle_l"][2],
          [round(k[j][2], 2) for j in ("head", "neck", "spine_chest", "pelvis", "knee_l", "ankle_l")])
    walk = (tr[-1]["root"][0] - tr[0]["root"][0], tr[-1]["root"][1] - tr[0]["root"][1])
    face = [(q["nose"][0] - q["head"][0]) * walk[0] + (q["nose"][1] - q["head"][1]) * walk[1] for q in ks]
    check("... walking: the nose ahead of the head, the way they walk", sum(f > 0 for f in face) > 0.9 * len(face),
          "%d of %d" % (sum(f > 0 for f in face), len(face)))
    arm = math.dist(k["shoulder_l"], k["elbow_l"]) + math.dist(k["elbow_l"], k["wrist_l"])
    check("... an arm 0.45-0.7 m shoulder to wrist", 0.45 < arm < 0.7, round(arm, 2))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
