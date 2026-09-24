"""Closed-form inverse kinematics for UR-type 6-axis arms, from a URDF chain.

UR-type means: J1 vertical-ish and perpendicular to J2; J2, J3, J4 parallel;
J5 perpendicular to J4; J6 perpendicular to J5; joint_6's origin on the J5
axis. FR20 is this layout (as are UR, Fairino FR3-FR30, Aubo, Elite CS). Such
an arm has up to 8 solutions for a pose -- shoulder left/right x elbow
up/down x wrist flip -- all of which this module returns.

The solution is derived from the URDF's own geometry at the zero pose rather
than from a DH table, so there are no angle offsets or sign conventions to
transcribe: every constant below is measured with urdf_rig.forward_kinematics,
and every returned solution is checked by pushing it back through that same
forward kinematics. A branch that does not reproduce the target is dropped,
never returned.

Why not Houdini's Full Body IK for FR20: FBIK jams on joint ranges that cross
+-180, fails at wrist flips, and restarts every frame from the URDF zero pose,
which is fully stretched and singular (measured -- see README Known issues).

Pure Python, no hou. Run the tests with:  python scripts/ur_ik.py
"""

import math

import urdf_rig as U

TOL_POS = 1e-6       # metres: a solution must reproduce the target this well
TOL_ROT = 1e-6       # radians
# Structure checks on the URDF. Not 1e-6: exporters round, and FR20's URDF
# writes pi/2 as 1.5708 -- 3.7e-6 rad off -- so its "perpendicular" axes are
# not exactly perpendicular. The closed form assumes exact UR geometry; the
# Newton polish below absorbs the difference against the URDF as written.
TOL_GEOM = 1e-3
SEED_TOL = 5e-2      # metres / radians: a closed-form seed this close gets polished.
                     # Loose on purpose: near the shoulder singularity the URDF's
                     # rounding is amplified into mm-level seed error. Correctness
                     # is the strict TOL_POS / TOL_ROT check after the polish.


class NotURType(ValueError):
    """The chain is not a UR-type arm; this solver does not apply."""


# --------------------------------------------------------------------------
# small helpers on top of urdf_rig's linear algebra
# --------------------------------------------------------------------------

def _transpose(m):
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))


def _rot_about(axis, angle, v):
    return U._mat_vec(U.axis_angle_matrix(axis, angle), v)


def _signed_angle(a, b, axis):
    """Angle turning a onto b about axis (right-handed). Both are projected
    onto the plane normal to axis first -- without that, any component along
    the axis leaks into the dot product and the angle comes out wrong."""
    pa = U._sub(a, U._scale(axis, U._dot(a, axis)))
    pb = U._sub(b, U._scale(axis, U._dot(b, axis)))
    return math.atan2(U._dot(U._cross(pa, pb), axis), U._dot(pa, pb))


def _joint_transform(joint, q_rad):
    """(R, p) of the child link in the parent link frame."""
    R = U._mat_mul(U.rpy_to_matrix(joint["rpy"]),
                   U.axis_angle_matrix(joint["axis"], q_rad))
    return R, tuple(joint["xyz"])


def _compose(a, b):
    Ra, pa = a
    Rb, pb = b
    return U._mat_mul(Ra, Rb), U._add(pa, U._mat_vec(Ra, pb))


def _inverse(t):
    R, p = t
    Rt = _transpose(R)
    return Rt, U._scale(U._mat_vec(Rt, p), -1.0)


def _rot_error(Ra, Rb):
    """Angle of Ra^T Rb, radians.

    atan2, not acos: acos(1 - e) ~ sqrt(2e), so near zero it turns float32
    noise in a goal matrix (~1e-7) into ~4.5e-4 rad and every branch failed
    the tolerance on whichever frames happened to be noisier."""
    M = U._mat_mul(_transpose(Ra), Rb)
    c = (M[0][0] + M[1][1] + M[2][2] - 1.0) / 2.0
    s = 0.5 * math.sqrt((M[2][1] - M[1][2]) ** 2 + (M[0][2] - M[2][0]) ** 2
                        + (M[1][0] - M[0][1]) ** 2)
    return math.atan2(s, c)


def orthonormalize(R):
    """The rotation nearest R, by Gram-Schmidt on its columns (right-handed).
    Goals arrive from Houdini's float32 attributes, so they are rotations to
    ~1e-7 only; the solve works on the exact rotation they stand for."""
    c0 = U._normalize((R[0][0], R[1][0], R[2][0]))
    c1 = (R[0][1], R[1][1], R[2][1])
    c1 = U._normalize(U._sub(c1, U._scale(c0, U._dot(c0, c1))))
    c2 = U._cross(c0, c1)
    return tuple(tuple((c0, c1, c2)[j][i] for j in range(3)) for i in range(3))


def _wrap(a):
    """Degrees into (-180, 180]."""
    a = math.fmod(a, 360.0)
    if a <= -180.0:
        a += 360.0
    elif a > 180.0:
        a -= 360.0
    return a


def pose_of(chain, q_deg):
    """(R, p) of the last link (joint_6 frame) in the URDF base frame."""
    last = U.forward_kinematics(chain, q_deg)[-1]
    return last["link_R"], last["link_p"]


def _rotvec(R):
    """Axis-angle vector of R (radians), for small-to-moderate angles.
    atan2 for the angle, as in _rot_error."""
    v = (R[2][1] - R[1][2], R[0][2] - R[2][0], R[1][0] - R[0][1])
    s = 0.5 * U._norm(v)
    angle = math.atan2(s, (R[0][0] + R[1][1] + R[2][2] - 1.0) / 2.0)
    if s < 1e-15:
        return (0.0, 0.0, 0.0)
    k = angle / (2.0 * s)
    return (k * v[0], k * v[1], k * v[2])


def _pose_error(chain, q_deg, R6, p6):
    Rq, pq = pose_of(chain, q_deg)
    dp = U._sub(p6, pq)
    dr = _rotvec(U._mat_mul(R6, _transpose(Rq)))
    return list(dp) + list(dr)


def _solve_linear(A, b):
    """Gaussian elimination with partial pivoting; A is n x n."""
    n = len(b)
    M = [list(A[i]) + [b[i]] for i in range(n)]
    for c in range(n):
        piv = max(range(c, n), key=lambda r: abs(M[r][c]))
        M[c], M[piv] = M[piv], M[c]
        if abs(M[c][c]) < 1e-15:
            raise ZeroDivisionError
        for r in range(c + 1, n):
            f = M[r][c] / M[c][c]
            for k in range(c, n + 1):
                M[r][k] -= f * M[c][k]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        x[r] = (M[r][n] - sum(M[r][k] * x[k] for k in range(r + 1, n))) / M[r][r]
    return x


def _polish(chain, q_deg, R6, p6, iterations=20, damping=1e-9):
    """Damped Newton steps on the exact URDF from a closed-form seed. The seed
    is already within micrometres, so this converges in two or three steps."""
    q = list(q_deg)
    for _ in range(iterations):
        e = _pose_error(chain, q, R6, p6)
        if max(abs(x) for x in e) < 1e-12:
            break
        h = 1e-6
        J = [[0.0] * 6 for _ in range(6)]
        for k in range(6):
            dq = list(q)
            dq[k] += math.degrees(h)
            ek = _pose_error(chain, dq, R6, p6)
            for i in range(6):
                J[i][k] = (e[i] - ek[i]) / h          # d(pose)/dq
        # (J^T J + damping I) dx = J^T e
        JtJ = [[sum(J[r][i] * J[r][j] for r in range(6)) + (damping if i == j else 0.0)
                for j in range(6)] for i in range(6)]
        Jte = [sum(J[r][i] * e[r] for r in range(6)) for i in range(6)]
        try:
            dx = _solve_linear(JtJ, Jte)
        except ZeroDivisionError:
            break
        q = [a + math.degrees(d) for a, d in zip(q, dx)]
    return q


# --------------------------------------------------------------------------
# the model: constants measured once from the URDF
# --------------------------------------------------------------------------

def analyse(chain):
    """Check the chain is UR-type and measure the constants the solve needs.

    Raises NotURType naming the first structural property that fails."""
    if len(chain) != 6:
        raise NotURType("need 6 revolute joints, got %d" % len(chain))
    fk = U.forward_kinematics(chain)
    a = [f["axis"] for f in fk]
    p = [f["position"] for f in fk]

    def perpendicular(i, j):
        return abs(U._dot(a[i], a[j])) < TOL_GEOM

    def parallel(i, j):
        return abs(abs(U._dot(a[i], a[j])) - 1.0) < TOL_GEOM

    checks = [("J1 perpendicular to J2", perpendicular(0, 1)),
              ("J2 parallel to J3", parallel(1, 2)),
              ("J2 parallel to J4", parallel(1, 3)),
              ("J5 perpendicular to J4", perpendicular(4, 3)),
              ("J6 perpendicular to J5", perpendicular(5, 4))]
    for name, ok in checks:
        if not ok:
            raise NotURType("not a UR-type arm: %s fails" % name)

    # joint_6's origin must lie on the J5 axis, so that J5 and J6 turning
    # leaves it where it is -- the property the J1 solve relies on
    off = U._sub(p[5], p[4])
    perp = U._sub(off, U._scale(a[4], U._dot(off, a[4])))
    if U._norm(perp) > TOL_GEOM:
        raise NotURType("not a UR-type arm: joint_6 is %.6f m off the J5 axis"
                        % U._norm(perp))

    u2 = a[1]
    # a6 . u2 as J5 turns is k * cos(q5); measure k and confirm the form
    k6 = U._dot(a[5], u2)
    fk90 = U.forward_kinematics(chain, [0, 0, 0, 0, 90.0, 0])
    if abs(abs(k6) - 1.0) > TOL_GEOM or abs(U._dot(fk90[5]["axis"], u2)) > TOL_GEOM:
        raise NotURType("not a UR-type arm: J6 axis does not start parallel "
                        "to J2 and swing with J5")

    v23, v34 = U._sub(p[2], p[1]), U._sub(p[3], p[2])
    e1 = U._normalize(U._sub(v23, U._scale(u2, U._dot(v23, u2))))
    e2 = U._cross(u2, e1)
    to2d = lambda v: (U._dot(v, e1), U._dot(v, e2))
    v23_2d, v34_2d = to2d(v23), to2d(v34)

    return {
        "chain": chain,
        "a1": a[0], "p1": p[0], "u2": u2, "p2": p[1],
        "axis6_local": U._normalize(chain[5]["axis"]),
        "dh": U._dot(U._sub(p[5], p[0]), u2),      # h-offset of joint_6
        "k6": k6,
        "sigma3": 1.0 if U._dot(a[2], u2) > 0 else -1.0,
        "sigma4": 1.0 if U._dot(a[3], u2) > 0 else -1.0,
        "v23": v23, "v34": v34, "e1": e1, "e2": e2,
        "L1": math.hypot(*v23_2d), "L2": math.hypot(*v34_2d),
        "alpha": math.atan2(v23_2d[1], v23_2d[0]),
        "beta": math.atan2(v34_2d[1], v34_2d[0]),
        "limits": [(j["lower_deg"], j["upper_deg"]) for j in chain],
    }


# --------------------------------------------------------------------------
# solve
# --------------------------------------------------------------------------

def solve(model, R6, p6, q6_when_singular=0.0):
    """Every branch that reproduces the joint_6 pose (R6, p6), URDF base
    frame, metres. Angles in degrees wrapped to (-180, 180], limits NOT yet
    applied (see within_limits). Each result is a dict:

        q         [q1..q6] degrees
        singular  True when the wrist is singular (q5 ~ 0): q6 is then not
                  determined by the pose and takes q6_when_singular
        branch    (shoulder, wrist, elbow) as +1/-1, for reporting
    """
    m = model
    chain = m["chain"]
    R6 = orthonormalize(R6)
    a1, p1, u2_0 = m["a1"], m["p1"], m["u2"]
    a6 = U._normalize(U._mat_vec(R6, m["axis6_local"]))

    # J1: joint_6 sits at a fixed offset dh along the J2 axis direction
    w = U._sub(p6, p1)
    A = U._dot(w, u2_0)
    B = U._dot(w, U._cross(a1, u2_0))
    r = math.hypot(A, B)
    # r == |dh| is the shoulder singularity (the wrist beside the J1 axis);
    # rounding can put r a hair under |dh| there, so allow a small overshoot
    if r < 1e-12 or abs(m["dh"]) > r * (1.0 + 1e-4):
        return []
    phi = math.atan2(B, A)
    acos1 = math.acos(max(-1.0, min(1.0, m["dh"] / r)))

    out = []
    for s1 in (1.0, -1.0):
        q1 = phi + s1 * acos1
        u2 = _rot_about(a1, q1, u2_0)

        # J5 from how far the tool axis leans off the J2 axis direction
        c5 = U._dot(a6, u2) / m["k6"]
        if abs(c5) > 1.0 + 1e-4:
            continue
        acos5 = math.acos(max(-1.0, min(1.0, c5)))

        for s5 in (1.0, -1.0):
            q5 = s5 * acos5
            # Near q5 = 0 the J6 estimate is ill-conditioned (and the URDF's
            # rounding alone puts q5 ~0.08 deg off there), so right at the
            # singularity seed q6 from the caller and let the polish settle it.
            near_singular = abs(math.sin(q5)) < 1e-4

            # J6 from where the J2 axis direction points in the joint_6 frame
            if near_singular:
                q6 = math.radians(q6_when_singular)
            else:
                R6_q5 = U.forward_kinematics(chain, [0, 0, 0, 0, math.degrees(q5), 0])[5]["link_R"]
                w_local = U._mat_vec(_transpose(R6_q5), u2_0)
                h_target = U._mat_vec(_transpose(R6), u2)
                q6 = _signed_angle(h_target, w_local, m["axis6_local"])

            # joint_4's frame, peeling J5 and J6 off the target
            T56 = _compose(_joint_transform(chain[4], q5), _joint_transform(chain[5], q6))
            R4, p4 = _compose((R6, p6), _inverse(T56))

            # J2, J3: a planar two-link problem about u2
            P2 = U._add(p1, _rot_about(a1, q1, U._sub(m["p2"], p1)))
            e1 = _rot_about(a1, q1, m["e1"])
            e2 = U._cross(u2, e1)
            t = U._sub(p4, P2)
            tx, ty = U._dot(t, e1), U._dot(t, e2)
            L1, L2 = m["L1"], m["L2"]
            D = (tx * tx + ty * ty - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
            # straight elbow: URDF rounding alone can push D just past 1;
            # clamp small overshoots and let the polish settle them
            if abs(D) > 1.0 + 1e-4:
                continue
            acos3 = math.acos(max(-1.0, min(1.0, D)))

            for s3 in (1.0, -1.0):
                gamma = s3 * acos3
                q3 = m["sigma3"] * (gamma - (m["beta"] - m["alpha"]))
                cx = L1 * math.cos(m["alpha"]) + L2 * math.cos(m["alpha"] + gamma)
                cy = L1 * math.sin(m["alpha"]) + L2 * math.sin(m["alpha"] + gamma)
                q2 = math.atan2(ty, tx) - math.atan2(cy, cx)

                # J4 takes whatever turn about u2 is left for joint_4's frame
                q_deg = [math.degrees(q1), math.degrees(q2), math.degrees(q3), 0.0]
                R4_pred = U.forward_kinematics(chain, q_deg + [0.0, 0.0])[3]["link_R"]
                dR = U._mat_mul(R4, _transpose(R4_pred))
                phi4 = _signed_angle(e1, U._mat_vec(dR, e1), u2)
                q4 = m["sigma4"] * phi4

                q = [math.degrees(x) for x in (q1, q2, q3, q4, q5, q6)]
                Rq, pq = pose_of(chain, q)
                if U._norm(U._sub(pq, p6)) > SEED_TOL or _rot_error(Rq, R6) > SEED_TOL:
                    continue
                # at the wrist singularity J4 and J6 share an axis; stronger
                # damping keeps the polish from wandering along that null space
                q = _polish(chain, q, R6, p6, damping=1e-5 if near_singular else 1e-9)
                q = [_wrap(x) for x in q]
                singular = abs(math.sin(math.radians(q[4]))) < 1e-4
                Rq, pq = pose_of(chain, q)
                if U._norm(U._sub(pq, p6)) > TOL_POS or _rot_error(Rq, R6) > TOL_ROT:
                    continue
                out.append({"q": q, "singular": singular,
                            "branch": (int(s1), int(s5), int(s3))})
    return out


def within_limits(model, solutions):
    """Each solution in every +-360 representation that fits the joint
    limits. A joint range under 360 deg admits at most one; wider ranges may
    admit several."""
    out = []
    for sol in solutions:
        options = []
        for q, (lo, hi) in zip(sol["q"], model["limits"]):
            reps = [q + 360.0 * k for k in range(-2, 3) if lo - 1e-9 <= q + 360.0 * k <= hi + 1e-9]
            options.append(reps)
        if not all(options):
            continue
        combos = [[]]
        for reps in options:
            combos = [c + [r] for c in combos for r in reps]
        for q in combos:
            out.append(dict(sol, q=q))
    return out


def closest(model, R6, p6, seed, iterations=100, orient_weight=0.3):
    """When no branch reaches (R6, p6): the in-limit pose that gets nearest,
    starting from seed (degrees) and clamping to the joint limits. Continuous
    with the seed, which is the point -- the old fallback held a fixed
    reference pose and the arm jumped to it.

    Levenberg-Marquardt: a step is taken only if it lowers the error, and the
    damping adapts. Plain damped Newton oscillated here -- an out-of-reach
    target sits at the stretched-elbow singularity, and the elbow flipped
    back and forth across zero, ending worse after 60 steps than after 5.

    Returns (q, position error m, orientation error rad)."""
    chain = model["chain"]
    lim = model["limits"]
    R6 = orthonormalize(R6)
    clamp = lambda qq: [min(max(a, lo), hi) for a, (lo, hi) in zip(qq, lim)]

    def err(qq):
        e = _pose_error(chain, qq, R6, p6)
        return e[:3] + [orient_weight * x for x in e[3:]]

    def cost(e):
        return sum(x * x for x in e)

    q = clamp(seed)
    e = err(q)
    c = cost(e)
    lam = 1e-3
    for _ in range(iterations):
        h = 1e-6
        J = [[0.0] * 6 for _ in range(6)]
        for k in range(6):
            dq = list(q)
            dq[k] += math.degrees(h)
            ek = err(dq)
            for i in range(6):
                J[i][k] = (e[i] - ek[i]) / h
        JtJ = [[sum(J[r][i] * J[r][j] for r in range(6)) for j in range(6)] for i in range(6)]
        Jte = [sum(J[r][i] * e[r] for r in range(6)) for i in range(6)]
        improved = False
        while lam < 1e6:
            A = [[JtJ[i][j] + (lam * (1.0 + JtJ[i][i]) if i == j else 0.0) for j in range(6)]
                 for i in range(6)]
            try:
                dx = _solve_linear(A, Jte)
            except ZeroDivisionError:
                lam *= 10.0
                continue
            qn = clamp([a + math.degrees(d) for a, d in zip(q, dx)])
            en = err(qn)
            cn = cost(en)
            if cn < c:
                step = max(abs(a - b) for a, b in zip(qn, q))
                q, e, c = qn, en, cn
                lam = max(lam * 0.3, 1e-9)
                improved = True
                break
            lam *= 10.0
        if not improved or step < 1e-9:
            break
    Rq, pq = pose_of(chain, q)
    return q, U._norm(U._sub(pq, p6)), _rot_error(Rq, R6)


def nearest(solutions, reference):
    """The solution closest to reference (degrees, L2 over joints)."""
    if not solutions:
        return None
    return min(solutions,
               key=lambda s: sum((a - b) ** 2 for a, b in zip(s["q"], reference)))


# --------------------------------------------------------------------------
# Houdini frame adapter (pure math -- the KineFX convention of urdf_rig)
# --------------------------------------------------------------------------

def urdf_pose_from_kinefx(goal_rows, goal_P, rest_rows):
    """joint_6's KineFX goal (transform rows, P, Houdini Y-up) -> URDF (R6, p6).

    urdf_rig.posed_skeleton builds each posed joint as rows' = delta . rows
    with delta = C R_q R_0^T C^T. So delta = M'^T M_rest, and
    R_q = C^T delta C R_0, where R_0 is joint_6's URDF frame at zero."""
    C = U.Z_UP_TO_Y_UP
    Ct = _transpose(C)
    delta = U._mat_mul(_transpose(goal_rows), rest_rows)
    return delta, U._mat_vec(Ct, goal_P), Ct, C


def urdf_pose_from_kinefx_full(chain, goal_rows, goal_P, rest_rows):
    delta, p6, Ct, C = urdf_pose_from_kinefx(goal_rows, goal_P, rest_rows)
    R0 = U.forward_kinematics(chain)[-1]["link_R"]
    R6 = U._mat_mul(Ct, U._mat_mul(delta, U._mat_mul(C, R0)))
    return R6, p6


# --------------------------------------------------------------------------
# tests -- python scripts/ur_ik.py
# --------------------------------------------------------------------------

if __name__ == "__main__":
    import os
    import random
    import sys
    import time

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    urdf = os.path.join(root, "assets", "fairino_description", "urdf", "fairino20_v6.urdf")
    chain = U.parse_urdf(urdf)["chain"]
    failures = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + detail) if detail else ""))
        if not ok:
            failures.append(label)

    model = analyse(chain)
    check("FR20 is UR-type", True, "dh=%.4f m, L1=%.4f, L2=%.4f" % (model["dh"], model["L1"], model["L2"]))

    # 1. round trip: random in-limit poses; every branch reproduces the
    #    target, and the pose's own q is among the branches
    rng = random.Random(7)
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 10000
    worst_pos = worst_rot = 0.0
    missing, near_singular, counts, t0 = 0, 0, {}, time.time()

    def singular_neighbourhood(q, p6):
        """Wrist (q5 ~ 0), elbow (arm straight) or shoulder (joint_6 beside
        the J1 axis, r == |dh|). There the branches merge or a whole family
        of solutions reaches the same pose, so the returned ones are exact
        but need not include this particular q."""
        if abs(math.sin(math.radians(q[4]))) < 1e-2:
            return True
        if abs(math.sin(math.radians(q[2]))) < 1e-2:
            return True
        r = math.hypot(*U._sub(p6, model["p1"])[:2])
        return r - abs(model["dh"]) < 1e-3

    for _ in range(N):
        q = [rng.uniform(lo, hi) for lo, hi in model["limits"]]
        R6, p6 = pose_of(chain, q)
        sols = solve(model, R6, p6)
        counts[len(sols)] = counts.get(len(sols), 0) + 1
        for s in sols:
            Rq, pq = pose_of(chain, s["q"])
            worst_pos = max(worst_pos, U._norm(U._sub(pq, p6)))
            worst_rot = max(worst_rot, _rot_error(Rq, R6))
        if singular_neighbourhood(q, p6):
            near_singular += 1
            continue
        wrapped = [_wrap(x) for x in q]
        if not any(max(abs(_wrap(a - b)) for a, b in zip(s["q"], wrapped)) < 1e-6 for s in sols):
            missing += 1
    dt = time.time() - t0
    check("every branch reproduces its target (%d poses)" % N,
          worst_pos < 1e-6 and worst_rot < math.radians(1e-3),
          "worst %.2e m, %.2e deg" % (worst_pos, math.degrees(worst_rot)))
    check("away from singularities, the pose's own q is always among the branches",
          missing == 0, "%d missing; %d poses in a singular neighbourhood checked for "
          "exactness only" % (missing, near_singular))
    # fewer than 8 is physics, not failure: for the other shoulder branch the
    # wrist centre can sit beyond the arm's reach. Report the distribution;
    # require all 8 only where every branch is clearly reachable.
    print("      branch counts %s, %.2f ms/solve" % (dict(sorted(counts.items())), 1000 * dt / N))
    q = [10, -80, 60, -70, 50, 30]
    R6, p6 = pose_of(chain, q)
    n_interior = len(solve(model, R6, p6))
    check("an interior pose has all 8 branches", n_interior == 8, "%d" % n_interior)

    # 2. limits
    q = [30, -90, 90, -90, -90, 0]
    R6, p6 = pose_of(chain, q)
    lim = within_limits(model, solve(model, R6, p6))
    inside = all(lo - 1e-9 <= a <= hi + 1e-9 for s in lim for a, (lo, hi) in zip(s["q"], model["limits"]))
    check("within_limits keeps only in-range representations", inside and len(lim) > 0,
          "%d in-limit solutions" % len(lim))
    q_ref = [a + 3.0 for a in q]
    best = nearest(lim, q_ref)
    check("nearest() recovers the pose from a nearby reference",
          best is not None and max(abs(a - b) for a, b in zip(best["q"], q)) < 1e-6,
          str([round(a, 4) for a in best["q"]]) if best else "none")

    # 2b. float32 noise in the goal (Houdini attributes): same branches
    q = [10, -80, 60, -70, 50, 30]
    R6, p6 = pose_of(chain, q)
    clean = len(solve(model, R6, p6))
    worst_noisy = clean
    for trial in range(200):
        noisy = tuple(tuple(v + rng.uniform(-1e-7, 1e-7) for v in row) for row in R6)
        worst_noisy = min(worst_noisy, len(solve(model, noisy, p6)))
    check("float32-level noise on the goal keeps every branch", worst_noisy == clean,
          "clean %d, worst noisy %d over 200 trials" % (clean, worst_noisy))

    # 3. unreachable
    far = (5.0, 0.0, 0.0)
    check("unreachable target returns no solution", solve(model, R6, far) == [])

    # 3b. closest(): a target just beyond reach -> a near-straight arm that
    #     gets within the overshoot, starting from a far-off seed, in limits
    q = [15, -30, 5, -60, 70, 20]
    R6, p6 = pose_of(chain, q)
    fk = U.forward_kinematics(chain, q)
    shoulder = fk[1]["position"]
    out = U._normalize(U._sub(p6, shoulder))
    p_far = U._add(p6, U._scale(out, 0.05))          # 50 mm past a near-stretched pose
    none_found = within_limits(model, solve(model, R6, p_far)) == []
    seed = [0, -90, 90, -90, -90, 0]
    qc, perr, rerr = closest(model, R6, p_far, seed)
    seed_err = U._norm(U._sub(pose_of(chain, seed)[1], p_far))
    in_lim = all(lo - 1e-9 <= a <= hi + 1e-9 for a, (lo, hi) in zip(qc, model["limits"]))
    check("closest(): unreachable target gets near, in limits",
          none_found and in_lim and perr < 0.06 and perr < seed_err,
          "no exact branch: %s; error %.1f mm (seed was %.1f mm)" % (none_found, perr * 1000, seed_err * 1000))

    # 4. singular wrist (q5 = 0): finite, still reproduces the pose
    q = [20, -70, 80, -40, 0, 0]
    R6, p6 = pose_of(chain, q)
    sols = solve(model, R6, p6, q6_when_singular=15.0)
    at_zero = [s for s in sols if abs(s["q"][4]) < 0.01]
    finite = all(math.isfinite(a) for s in sols for a in s["q"])
    check("singular wrist: finite solutions, those at q5=0 flagged singular",
          bool(at_zero) and finite and all(s["singular"] for s in at_zero),
          "%d solutions (every one reproduced the pose), %d at q5=0, q6 there %s"
          % (len(sols), len(at_zero), [round(s["q"][5], 3) for s in at_zero]))

    # 5. a non-UR chain is refused
    bent = [dict(j) for j in chain]
    bent[3] = dict(bent[3], rpy=(bent[3]["rpy"][0] + 0.3, bent[3]["rpy"][1], bent[3]["rpy"][2]))
    try:
        analyse(bent)
        check("non-UR chain is refused", False, "analyse() accepted it")
    except NotURType as e:
        check("non-UR chain is refused", True, str(e))

    # 6. Houdini adapter: posed_skeleton's joint_6 -> URDF pose -> q back
    flange = 0.12
    rest = U.build_rest_skeleton(chain, flange)
    rest6 = [j for j in rest if j["name"] == "joint_6"][0]["transform"]
    q = [-45, -60, 110, -30, 60, 45]
    posed6 = [j for j in U.posed_skeleton(chain, flange, q) if j["name"] == "joint_6"][0]
    R6, p6 = urdf_pose_from_kinefx_full(chain, posed6["transform"], posed6["P"], rest6)
    best = nearest(within_limits(model, solve(model, R6, p6)), q)
    check("KineFX joint_6 goal round-trips to the same q",
          best is not None and max(abs(a - b) for a, b in zip(best["q"], q)) < 1e-6,
          str([round(a, 4) for a in best["q"]]) if best else "none")

    print()
    if failures:
        print("FAILED: %d -- %s" % (len(failures), "; ".join(failures)))
        sys.exit(1)
    print("OK: closed-form IK reproduces every target and finds the original pose")
