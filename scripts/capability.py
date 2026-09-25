"""What the arm can do at a point in its workspace -- the Stage 2 atlas core.

For a TCP target (point + tool direction, URDF base frame, metres) this
measures, over every IK branch within the joint limits:

    reachable       1 if any branch reaches the pose, else 0
    nsol            how many branches do
    wrist           |sin q5| of the best branch: 0 at the wrist singularity,
                    where J4/J6 amplify any change of the path (the FR20 test
                    clip's J6 needed 3x its acceleration limit there)
    margin_deg      smallest distance of any joint to its limit, degrees
    headroom_mps    fastest TCP speed, in the WORST direction, holding the
                    tool's orientation, before any joint hits its velocity
                    limit, m/s. 0 at a singularity. From q' = J^-1 [v; 0]:
                    worst |v| = min_i vlim_i / |row i of J^-1[:, :3]|.

"Best branch" is the one with the most headroom -- the atlas shows what the
arm CAN do at a point, not what one particular solve happens to pick.

The capability index of a point is the fraction of tool directions (sampled
evenly over the sphere) that are reachable there -- a reachability map in
the sense of Zacharias et al., "Capturing robot workspace structure", 2007.

Pure Python, no hou. The Houdini bake (scripts/hda/atlas_sop.py) calls
measure() / capability_index() per grid point. Tests:
    python scripts/capability.py
"""

import math

import ur_ik
import urdf_rig as U


def load_fr20(root):
    """(model, chain, flange_offset, vel_limits) for the FR20 profile."""
    import json
    import os
    prof = json.load(open(os.path.join(root, "profiles", "fr20.json")))
    chain = U.parse_urdf(os.path.join(root, prof["rig"]["urdf"]))["chain"]
    model = ur_ik.analyse(chain)
    vel = prof["robot"]["max_velocity_deg_s"]
    vel = list(vel) if isinstance(vel, list) else [float(vel)] * 6
    return model, chain, float(prof["rig"].get("flange_offset_m", 0.0)), vel


def tool_frame(direction, roll_deg=0.0):
    """Rotation whose z axis (the flange normal, urdf_rig.flange_point) is
    direction; x is chosen from world x (or y when direction is near x),
    then turned by roll about z."""
    z = U._normalize(direction)
    ref = (1.0, 0.0, 0.0) if abs(z[0]) < 0.9 else (0.0, 1.0, 0.0)
    x = U._normalize(U._sub(ref, U._scale(z, U._dot(ref, z))))
    y = U._cross(z, x)
    if roll_deg:
        c, s = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
        x, y = (tuple(c * a + s * b for a, b in zip(x, y)),
                tuple(-s * a + c * b for a, b in zip(x, y)))
    # columns x, y, z
    return tuple((x[i], y[i], z[i]) for i in range(3))


def _jacobian(chain, q, tcp_local):
    """6x6 geometric Jacobian at the TCP (rows vx vy vz wx wy wz, per rad)."""
    fk = U.forward_kinematics(chain, q)
    last = fk[-1]
    tcp = U._add(last["link_p"], U._mat_vec(last["link_R"], tcp_local))
    cols = []
    for j in fk:
        a = j["axis"]
        cols.append(U._cross(a, U._sub(tcp, j["position"])) + a)
    return [[cols[c][r] for c in range(6)] for r in range(6)], tcp


def _inverse(m):
    """Gauss-Jordan inverse of a square matrix, or None if singular."""
    n = len(m)
    a = [list(row) + [1.0 if i == j else 0.0 for j in range(n)] for i, row in enumerate(m)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(a[r][c]))
        if abs(a[p][c]) < 1e-12:
            return None
        a[c], a[p] = a[p], a[c]
        piv = a[c][c]
        a[c] = [x / piv for x in a[c]]
        for r in range(n):
            if r != c and a[r][c] != 0.0:
                f = a[r][c]
                a[r] = [x - f * y for x, y in zip(a[r], a[c])]
    return [row[n:] for row in a]


def headroom(chain, q, tcp_local, vel_deg_s):
    """Worst-direction TCP speed (m/s) with the orientation held, before a
    joint reaches its velocity limit. 0 when J is singular."""
    J, _ = _jacobian(chain, q, tcp_local)
    Ji = _inverse(J)
    if Ji is None:
        return 0.0
    best = math.inf
    for i in range(6):
        n = math.sqrt(sum(Ji[i][k] ** 2 for k in range(3)))
        if n > 1e-12:
            best = min(best, math.radians(vel_deg_s[i]) / n)
    return 0.0 if best == math.inf else best


def _margin(q, limits):
    return min(min(x - lo, hi - x) for x, (lo, hi) in zip(q, limits))


def measure(model, chain, point, direction, flange_offset, vel_deg_s, tool_len=0.0, roll_deg=0.0, frame=None,
            cell=None):
    """The atlas values at one TCP target (see the module docstring).
    frame: a full rotation instead of direction + roll. cell: (collision
    model, env) -- then also "clear" (some in-limit branch reaches it without
    touching the room or itself) and "clearance_m" (of the branch chosen);
    the branch chosen is then the clear one with the most headroom."""
    R = frame or tool_frame(direction, roll_deg)
    reach = flange_offset + tool_len
    tcp_local = (0.0, 0.0, reach)
    p6 = U._sub(point, U._mat_vec(R, tcp_local))
    sols = ur_ik.within_limits(model, ur_ik.solve(model, R, p6))
    out = {"reachable": 0, "nsol": len(sols), "wrist": 0.0, "margin_deg": 0.0, "headroom_mps": 0.0, "q": None}
    if cell is not None:
        out.update(clear=0, clearance_m=0.0)
    if not sols:
        return out
    limits = model["limits"]
    ranked = sorted(((headroom(chain, s["q"], tcp_local, vel_deg_s), s["q"]) for s in sols), key=lambda x: -x[0])
    h, q = ranked[0]
    out.update(reachable=1, headroom_mps=h, q=list(q),
               wrist=abs(math.sin(math.radians(q[4]))), margin_deg=_margin(q, limits))
    if cell is not None:
        import collision
        cmodel, env = cell
        best_c = None
        for h_, q_ in ranked:
            c, ok = collision.pose_clearance(cmodel, env, q_)
            best_c = c if best_c is None else max(best_c, c)
            if ok:
                out.update(clear=1, clearance_m=c, headroom_mps=h_, q=list(q_),
                           wrist=abs(math.sin(math.radians(q_[4]))), margin_deg=_margin(q_, limits))
                break
        else:
            out["clearance_m"] = best_c
    return out


def sphere_directions(n):
    """n unit vectors spread evenly over the sphere (Fibonacci lattice)."""
    g = math.pi * (3.0 - math.sqrt(5.0))
    out = []
    for i in range(n):
        z = 1.0 - 2.0 * (i + 0.5) / n
        r = math.sqrt(max(0.0, 1.0 - z * z))
        out.append((r * math.cos(g * i), r * math.sin(g * i), z))
    return out


def capability_index(model, chain, point, flange_offset, directions, tool_len=0.0, rolls=(0.0, 180.0)):
    """Fraction of the directions from which the TCP can reach point, at any
    of the rolls: J6 spans 350 deg on FR20, so one roll can fall in its gap
    where the opposite one does not."""
    ok = 0
    for d in directions:
        for roll in rolls:
            R = tool_frame(d, roll)
            p6 = U._sub(point, U._mat_vec(R, (0.0, 0.0, flange_offset + tool_len)))
            if ur_ik.within_limits(model, ur_ik.solve(model, R, p6)):
                ok += 1
                break
    return ok / float(len(directions))


if __name__ == "__main__":
    import os
    import random
    import sys
    import time

    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + detail) if detail else ""))
        if not ok:
            fails.append(label)

    ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    model, chain, fo, vel = load_fr20(ROOT)
    lim = model["limits"]
    tcp_local = (0.0, 0.0, fo)

    # 1. a pose made by FK is reachable from its own direction, and the
    #    measured TCP matches the target
    random.seed(7)
    n_ok, n = 0, 200
    t0 = time.perf_counter()
    for _ in range(n):
        q = [random.uniform(lo + 5, hi - 5) for lo, hi in lim]
        q[4] = random.choice([-1, 1]) * random.uniform(15, 150)         # away from the wrist singularity
        R, p6 = ur_ik.pose_of(chain, q)
        tcp = U._add(p6, U._mat_vec(R, tcp_local))
        m = measure(model, chain, tcp, None, fo, vel, frame=R)
        _, got = _jacobian(chain, m["q"], tcp_local) if m["q"] else (None, None)
        if m["reachable"] and got and max(abs(a - b) for a, b in zip(got, tcp)) < 1e-6:
            n_ok += 1
    check("FK-made targets are reachable and the best branch lands on them", n_ok == n,
          "%d/%d, %.1f ms a point" % (n_ok, n, (time.perf_counter() - t0) / n * 1000))

    # 2. beyond the arm's reach: unreachable from every direction
    dirs = sphere_directions(26)
    ci = capability_index(model, chain, (2.3, 0.0, 0.6), fo, dirs)
    check("a point 2.3 m out is unreachable from every direction", ci == 0.0, "index %.2f" % ci)
    ci = capability_index(model, chain, (0.8, 0.2, 0.6), fo, dirs)
    check("a point well inside the workspace is reachable from most directions", ci > 0.5, "index %.2f" % ci)

    # 3. headroom: the closed form is the worst of all directions (brute force)
    q = [20.0, -70.0, 100.0, -40.0, 60.0, 10.0]
    h = headroom(chain, q, tcp_local, vel)
    J, _ = _jacobian(chain, q, tcp_local)
    Ji = _inverse(J)
    worst = math.inf
    for d in sphere_directions(4000):
        qd = [sum(Ji[i][k] * d[k] for k in range(3)) for i in range(6)]
        worst = min(worst, min(math.radians(v) / abs(x) for v, x in zip(vel, qd) if abs(x) > 1e-12))
    check("headroom = worst direction over 4000 sampled directions", abs(worst - h) / h < 0.01,
          "%.4f vs %.4f m/s" % (h, worst))

    # 4. headroom falls to ~0 at the wrist singularity (q5 = 0)
    hs = [headroom(chain, [20.0, -70.0, 100.0, -40.0, q5, 10.0], tcp_local, vel) for q5 in (60.0, 10.0, 1.0, 0.0)]
    check("headroom falls towards the wrist singularity and is ~0 on it",
          hs[0] > hs[1] > hs[2] and hs[3] < 1e-3, " / ".join("%.4f" % x for x in hs))

    # 5. elbow straight (q3 = 0) is singular too
    h = headroom(chain, [20.0, -70.0, 0.0, -40.0, 60.0, 10.0], tcp_local, vel)
    check("headroom ~0 with the elbow straight (q3 = 0)", h < 1e-3, "%.5f m/s" % h)

    # 6. margin
    # J3 at 0 in +-162 is the nearest to a limit (162); J2 / J4 at -90 in
    # [-265, 85] and the rest at 0 in +-175 are 175 away
    m = _margin([0.0, -90.0, 0.0, -90.0, 0.0, 0.0], lim)
    check("joint-limit margin is the nearest joint's distance to its limit", abs(m - lim[2][1]) < 1e-9,
          "%.3f deg (URDF: %.3f)" % (m, lim[2][1]))

    # 7. with a cell: a point by the floor is reachable but not clear; one mid-air is both
    import collision
    cm = collision.load_model("fr20")
    floor = {"schema": collision.ENV_SCHEMA, "margin_m": 0.05,
             "objects": [{"name": "floor", "type": "halfspace", "normal": [0, 0, 1], "offset": -0.02, "role": "obstacle"}]}
    low = measure(model, chain, (-1.6, 0.0, 0.05), (0, 0, -1), fo, vel, cell=(cm, floor))
    mid = measure(model, chain, (-0.8, 0.2, 0.9), (0, 0, -1), fo, vel, cell=(cm, floor))
    check("a cell separates 'reachable' from 'reachable without touching the room'",
          low["reachable"] and not low["clear"] and low["clearance_m"] < 0 and mid["clear"] and mid["clearance_m"] > 0,
          "low: reach %d clear %d (%.0f mm); mid: clear %d (%.0f mm)"
          % (low["reachable"], low["clear"], low["clearance_m"] * 1000, mid["clear"], mid["clearance_m"] * 1000))

    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    sys.exit(1 if fails else 0)
