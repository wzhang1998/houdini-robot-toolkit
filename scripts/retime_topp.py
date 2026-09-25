"""Time-parameterise a joint path under velocity AND acceleration limits.

Retime used to budget velocity only. A clip could then pass every velocity
check in Houdini and still need 3000 deg/s^2 where the path swings the wrist
near its singularity -- and the player, which does check acceleration, slowed
the WHOLE clip about 6x to cover one stretch. This plans the timing so the
clip is feasible where it is made.

The path is sampled at u_i = i/N (q_i, degrees, unwrapped). With x = (du/dt)^2
and y = d2u/dt2, joint j's velocity is q'_j sqrt(x) and its acceleration
q''_j x + q'_j y. At each sample:

    velocity       x <= (vlim_j / |q'_j|)^2
    acceleration   -alim_j <= q''_j x + q'_j y <= alim_j

A forward pass accelerates from rest as hard as the limits allow; a backward
pass does the same from the end, so every slow-down starts in time; the
smaller of the two at each sample is the profile (the usual numerical
time-optimal path parameterisation). Both ends are at rest.

    plan(q, vel_limits, acc_limits) -> times, one per sample, t[0] = 0

Pure Python, no hou. Tests:  python scripts/retime_topp.py
"""

import math

EPS = 1e-9


def _derivs(q):
    """dq/du and d2q/du2 per sample, u in [0, 1]."""
    n = len(q) - 1
    du = 1.0 / n
    qp, qpp = [], []
    for i in range(n + 1):
        a, b = max(i - 1, 0), min(i + 1, n)
        qp.append([(q[b][j] - q[a][j]) / ((b - a) * du) for j in range(len(q[0]))])
        c = min(max(i, 1), n - 1)
        qpp.append([(q[c + 1][j] - 2 * q[c][j] + q[c - 1][j]) / (du * du) for j in range(len(q[0]))])
    return qp, qpp, du


def _y_bounds(qp, qpp, x, alim):
    """Range of path acceleration y allowed at state x, or None if none is."""
    lo, hi = -math.inf, math.inf
    for p, pp, a in zip(qp, qpp, alim):
        if abs(p) < EPS:
            if abs(pp) * x > a * (1 + 1e-9):
                return None
            continue
        b1, b2 = (-a - pp * x) / p, (a - pp * x) / p
        lo, hi = max(lo, min(b1, b2)), min(hi, max(b1, b2))
    if lo > hi:
        return None
    return lo, hi


def _x_max(qp, qpp, vlim, alim):
    """Largest x at a sample that the velocity limits allow and some
    acceleration can hold."""
    xv = math.inf
    for p, v in zip(qp, vlim):
        if abs(p) > EPS:
            xv = min(xv, (v / abs(p)) ** 2)
    if xv == math.inf:                   # path does not move here
        xv = 1e12
    if _y_bounds(qp, qpp, xv, alim) is not None:
        return xv
    lo, hi = 0.0, xv
    for _ in range(60):
        mid = (lo + hi) / 2.0
        if _y_bounds(qp, qpp, mid, alim) is None:
            hi = mid
        else:
            lo = mid
    return lo


def plan(q, vel_limits, acc_limits):
    """Times (seconds) at each path sample. q: list of joint vectors
    (degrees), at least 2; limits: one per joint."""
    n = len(q) - 1
    if n < 1:
        return [0.0]
    qp, qpp, du = _derivs(q)
    xmax = [_x_max(qp[i], qpp[i], vel_limits, acc_limits) for i in range(n + 1)]

    xf = [0.0] * (n + 1)
    for i in range(n):
        b = _y_bounds(qp[i], qpp[i], xf[i], acc_limits)
        y = b[1] if b else 0.0
        xf[i + 1] = min(max(xf[i] + 2.0 * du * y, 0.0), xmax[i + 1])

    x = list(xf)
    x[n] = 0.0
    for i in range(n, 0, -1):
        b = _y_bounds(qp[i], qpp[i], x[i], acc_limits)
        y = b[0] if b else 0.0
        x[i - 1] = min(x[i - 1], max(x[i] - 2.0 * du * y, 0.0))
    x[0] = 0.0

    t = [0.0]
    for i in range(n):
        s = math.sqrt(x[i]) + math.sqrt(x[i + 1])
        t.append(t[-1] + (2.0 * du / s if s > EPS else 0.0))
    return t


def frame_u(cum, fps):
    """Path position (fractional sample index) of every frame of a timing:
    frame f is at f/fps seconds, linear between path samples as the linear
    progress keys make it. cum: time (s) of each path sample."""
    n = len(cum) - 1
    nf = max(1, int(math.ceil(cum[-1] * fps - 1e-9)))
    out, k = [], 0
    for fi in range(nf + 1):
        tt = min(fi / fps, cum[-1])
        while k < n - 1 and cum[k + 1] <= tt:
            k += 1
        span = cum[k + 1] - cum[k]
        out.append(k + (0.0 if span <= 0 else min(1.0, (tt - cum[k]) / span)))
    return out


def frames(cum, q, fps):
    """The frames of a timing estimated from the path samples (joints linear
    between samples): (times, joints). Houdini cooks the real ones."""
    ts, qs = [], []
    for fi, x in enumerate(frame_u(cum, fps)):
        k = min(int(x), len(q) - 2)
        f = x - k
        ts.append(fi / fps)
        qs.append([a + f * (b - a) for a, b in zip(q[k], q[k + 1])])
    return ts, qs


def stretch(cum, bad, width_s=0.4):
    """Slow the timing around each (time_s, need) in bad: a raised-cosine
    window, need x 1.02 at its centre, the largest where windows overlap."""
    mult = [1.0] * (len(cum) - 1)
    j0 = 0
    for t, need in sorted(bad):
        k = need * 1.02
        while j0 < len(mult) and 0.5 * (cum[j0] + cum[j0 + 1]) < t - width_s:
            j0 += 1
        i = j0
        while i < len(mult) and 0.5 * (cum[i] + cum[i + 1]) < t + width_s:
            c = 0.5 * (cum[i] + cum[i + 1])
            mult[i] = max(mult[i], 1 + (k - 1) * 0.5 * (1 + math.cos(math.pi * (c - t) / width_s)))
            i += 1
    out = [0.0]
    for i, m in enumerate(mult):
        out.append(out[-1] + (cum[i + 1] - cum[i]) * m)
    return out


def fit(cum, frames_of, need_of, max_iter=40):
    """Stretch the timing where the frames still break a limit.

    The plan holds on the path samples, but the robot plays the 24 fps
    frames through the player's cubic. Where the path has a corner -- a
    polyline goal curve's vertex, amplified by the wrist beside its
    singularity -- a joint's velocity jumps between two frames and reads far
    over the limit, and the player would slow the WHOLE clip for it.
    frames_of(cum) -> (times, joints) are the frames of a timing;
    need_of(times, joints) -> [(time_s, need)], need > 1 being how much
    slower that moment must be (the player's own measure). Every such moment
    is slowed locally until none is left. Returns (cum, passes, done)."""
    for it in range(max_iter):
        bad = [b for b in need_of(*frames_of(cum)) if b[1] > 1.0]
        if not bad:
            return cum, it, True
        cum = stretch(cum, bad)
    return cum, max_iter, False


def check_limits(t, q):
    """Max velocity and acceleration per joint of the timed samples
    (finite differences on the nonuniform times), for tests and reports."""
    nj = len(q[0])
    v = []
    for i in range(len(t) - 1):
        h = t[i + 1] - t[i]
        v.append([(q[i + 1][j] - q[i][j]) / h if h > 0 else 0.0 for j in range(nj)])
    vmax = [max(abs(r[j]) for r in v) for j in range(nj)]
    amax = [0.0] * nj
    for i in range(len(v) - 1):
        h = (t[i + 2] - t[i]) / 2.0
        for j in range(nj):
            amax[j] = max(amax[j], abs(v[i + 1][j] - v[i][j]) / h if h > 0 else 0.0)
    return vmax, amax, v


if __name__ == "__main__":
    import sys
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + detail) if detail else ""))
        if not ok:
            fails.append(label)

    N = 2000
    # long single-joint move: trapezoid, theta/v + v/a
    th, v, a = 90.0, 60.0, 120.0
    q = [[th * i / N, 0.0] for i in range(N + 1)]
    t = plan(q, [v, 60.0], [a, 120.0])
    want = th / v + v / a
    check("trapezoid: 90 deg at 60 deg/s, 120 deg/s^2 takes theta/v + v/a",
          abs(t[-1] - want) / want < 0.01, "%.4f s vs %.4f" % (t[-1], want))
    # short move: triangle, 2 sqrt(theta/a)
    th = 10.0
    q = [[th * i / N, 0.0] for i in range(N + 1)]
    t = plan(q, [v, 60.0], [a, 120.0])
    want = 2.0 * math.sqrt(th / a)
    check("triangle: 10 deg never reaches 60 deg/s", abs(t[-1] - want) / want < 0.01,
          "%.4f s vs %.4f" % (t[-1], want))
    # a curved multi-joint path: limits hold everywhere, rest at the ends
    q = [[40 * math.sin(3.0 * u), 30 * (1 - math.cos(2.0 * u)), 25 * u * u, 60 * math.sin(7.0 * u) * u,
          10 * u, 50 * math.cos(5.0 * u)] for u in (i / N for i in range(N + 1))]
    vl = [120, 120, 120, 180, 180, 180]
    al = [150] * 6
    t = plan(q, vl, al)
    vmax, amax, vs = check_limits(t, q)
    check("curved 6-joint path: every joint within its velocity limit",
          all(x <= l * 1.01 for x, l in zip(vmax, vl)), "worst %.1f%%" % (100 * max(x / l for x, l in zip(vmax, vl))))
    check("curved 6-joint path: every joint within the acceleration limit",
          all(x <= l * 1.05 for x, l in zip(amax, al)), "worst %.1f%%" % (100 * max(x / l for x, l in zip(amax, al))))
    check("starts and ends at rest", max(abs(x) for x in vs[0]) < 5.0 and max(abs(x) for x in vs[-1]) < 5.0,
          "first/last step %.2f / %.2f deg/s" % (max(abs(x) for x in vs[0]), max(abs(x) for x in vs[-1])))
    check("limits actually bind (not needlessly slow)",
          max(max(x / l for x, l in zip(vmax, vl)), max(x / l for x, l in zip(amax, al))) > 0.9,
          "%.2f s" % t[-1])
    # a path that does not move
    t = plan([[1.0, 2.0]] * 50, [10, 10], [10, 10])
    check("a stationary path takes (next to) no time and does not fail", t[-1] < 1e-3, "%.2e s" % t[-1])
    # a wrist-like sharp turn: planned on the samples, but the 24 fps frames
    # through the player's cubic break the limit -- fit() slows only there
    import fairino_player
    N = 1500
    q = []
    for i in range(N + 1):
        u = i / N
        s = 60.0 * u + 1.2 * math.sqrt((u - 0.8) ** 2 + 1e-5)        # J6 folds back at u = 0.8
        q.append([20.0 * u, 10.0 * u, 5.0 * u, -s, 30.0 * u, s])
    vl, al = [120, 120, 120, 180, 180, 180], [150.0] * 6
    t = plan(q, vl, [a * 0.8 for a in al])
    need_of = lambda ts, qs: fairino_player.need_profile(ts, qs, 125.0, vl, al)
    lim = lambda c: fairino_player.limiting(*frames(c, q, 24.0), 125.0, vl, al)["scale_needed"]
    before = lim(t)
    cum, passes, done = fit(t, lambda c: frames(c, q, 24.0), need_of)
    after = lim(cum)
    check("fit: frames that needed a stretch now play at their own speed",
          before > 1.0 and done and after <= 1.0, "x%.2f -> x%.3f in %d passes" % (before, after, passes))
    check("fit: the clip grows by less than the uniform stretch the player would apply",
          cum[-1] < t[-1] * before, "%.2f s vs %.2f s" % (cum[-1], t[-1] * before))
    # several corners, as a polyline goal curve makes: all fixed, not just the worst
    q = []
    for i in range(N + 1):
        u = i / N
        s = 60.0 * u + sum(1.0 * math.sqrt((u - c) ** 2 + 1e-5) for c in (0.2, 0.45, 0.7, 0.9))
        q.append([20.0 * u, 10.0 * u, 5.0 * u, -s, 30.0 * u, s])
    t = plan(q, vl, [a * 0.8 for a in al])
    before = lim(t)
    cum, passes, done = fit(t, lambda c: frames(c, q, 24.0), need_of)
    check("fit: four corners all fixed", before > 1.0 and done and lim(cum) <= 1.0,
          "x%.2f -> x%.3f in %d passes" % (before, lim(cum), passes))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    sys.exit(1 if fails else 0)
