"""A person's arm motion -> an FR20 clip, two ways.

Input: 3D keypoints of one arm per frame (an OAK-D body-pose capture, a
mocap BVH export, AIST++ dance data ...), schema motionlab.keypoints/1:

    {"schema": "motionlab.keypoints/1", "fps": 30, "units": "m",
     "frame": "performer: x forward, y left, z up",
     "side": "right",
     "joints": {"shoulder": [[x, y, z], ...], "elbow": [...], "wrist": [...],
                "hand": [...]}}                              hand optional

    direct   the wrist's motion about its own centre, 1:1 in metres (size),
             placed in front of the robot, is the TCP; the tool points
             outward from the robot's shoulder (or, orient="forearm", along
             the performer's smoothed forearm). The performer
             faces the audience as the robot does, so forward -> the robot's
             front (-X), left -> the robot's left (-Y); mirror=True swaps
             sides. IK frame by frame on the branch nearest the last one.
             Timing is the performer's where the arm can follow it: the
             motion is shrunk towards its centre first (keeps the rhythm),
             the tempo stretched only after that.
    effort   the performer's Laban efforts per window (proximal = elbow
             speed, time = wrist acceleration / speed, space = wrist
             directness, flow = stillness -- each relative to the rest of the
             take), the nearest action per window, and choreo.py composes a
             robot-native phrase with that action sequence and tempo. A
             style transfer that does not care how different the arms are.

    smooth(kp, sigma_s)   -> keypoints     zero-phase Gaussian, before any timing
    direct(kp, env=None)  -> clip          effort(kp, env=None) -> clip
    synthetic_take(...)   -> keypoints (tests)

Pure Python. Tests: python scripts/retarget.py
"""

import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import capability as CAP  # noqa: E402
import choreo as CH  # noqa: E402
import collision as CL  # noqa: E402
import fairino_player as P  # noqa: E402
import motion_clip as M  # noqa: E402
import motion_labels as L  # noqa: E402
import ur_ik  # noqa: E402
import urdf_rig as U  # noqa: E402

SCHEMA = "motionlab.keypoints/1"
SHOULDER = (0.0, 0.0, 0.215 + 0.2)    # FR20 J2 axis height, raised: the robot's "shoulder" for retargeting
REACH = 1.35                          # usable reach from there (datasheet 1.854 m, minus wrist / margin)


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _median(xs):
    s = sorted(xs)
    return s[len(s) // 2]


def smooth(kp, sigma_s=0.1):
    """Zero-phase Gaussian smoothing of every keypoint track (sigma in
    seconds). Tracking jitter is small in position and huge after two
    derivatives: 2 mm of noise at 30 fps read as 9 m/s^2 at the TCP and put
    J3 at 41x its acceleration limit. An OAK-D body-pose track is noisier
    still -- smooth before anything is timed."""
    if sigma_s <= 0:
        return kp
    fps = float(kp["fps"])
    sig = sigma_s * fps
    h = max(1, int(3 * sig))
    w = [math.exp(-0.5 * (k / sig) ** 2) for k in range(-h, h + 1)]
    out = dict(kp)
    out["joints"] = {}
    for name, track in kp["joints"].items():
        n = len(track)
        sm = []
        for i in range(n):
            acc, tot = [0.0, 0.0, 0.0], 0.0
            for k, wt in enumerate(w):
                j = min(n - 1, max(0, i + k - h))            # hold the ends
                for c in range(3):
                    acc[c] += wt * track[j][c]
                tot += wt
            sm.append([x / tot for x in acc])
        out["joints"][name] = sm
    out["smoothed_s"] = sigma_s
    return out


def arm_length(kp):
    j = kp["joints"]
    up = _median([math.dist(a, b) for a, b in zip(j["shoulder"], j["elbow"])])
    fore = _median([math.dist(a, b) for a, b in zip(j["elbow"], j["wrist"])])
    return up + fore


def _to_robot(v, mirror):
    """performer (x forward, y left, z up) -> robot base: forward = -X, left = -Y."""
    return (-v[0], (v[1] if mirror else -v[1]), v[2])


def _lowpass(vs, fps, tau):
    """First-order low-pass of unit vectors, forwards then backwards (no lag)."""
    if tau <= 0:
        return list(vs)
    a = 1.0 - math.exp(-1.0 / (fps * tau))
    out = list(vs)
    for rng_ in (range(1, len(out)), range(len(out) - 2, -1, -1)):
        for i in rng_:
            j = i - 1 if rng_.step == 1 else i + 1
            v = tuple(out[j][k] + a * (out[i][k] - out[j][k]) for k in range(3))
            n = math.sqrt(sum(x * x for x in v)) or 1.0
            out[i] = tuple(x / n for x in v)
    return out


def targets(kp, gain=1.0, mirror=False, size=1.0, orient="outward", tau=0.5):
    """TCP points and tool directions per frame, robot base frame.

    size: metres of TCP motion per metre of wrist motion -- 1.0 moves the
    tool as far as the performer's hand. Scaling a human arm up to the
    robot's reach (x2.3) made even slow sweeps 1-1.6 m/s at the TCP.
    orient: "outward" -- the tool points away from the robot's shoulder, so
    the position leads and the wrist stays calm; "forearm" -- the
    performer's forearm (or hand) direction, low-passed over tau seconds.
    Copying the forearm raw drove FR20's J5 to 300 deg/s: a human wrist is
    not a UR-type wrist."""
    j = kp["joints"]
    tip = j.get("hand") or j["wrist"]
    base = j["elbow"] if not j.get("hand") else j["wrist"]
    rel = [_sub(w, s_) for w, s_ in zip(j["wrist"], j["shoulder"])]
    centre = tuple(sum(r[i] for r in rel) / len(rel) for i in range(3))
    anchor = (-0.9, 0.0, 1.0)                     # where the take's centre lands: in front of the robot
    pts, dirs = [], []
    for r, a, b in zip(rel, base, tip):
        rr = tuple(gain * (x - c) for x, c in zip(r, centre))
        p = tuple(anchor[i] + size * _to_robot(rr, mirror)[i] for i in range(3))
        pts.append(p)
        if orient == "forearm":
            d = _to_robot(_sub(b, a), mirror)
        else:
            d = _blend(_sub(p, SHOULDER), (0.0, 0.0, -1.0), 0.3)
        n = math.sqrt(sum(x * x for x in d)) or 1.0
        dirs.append(tuple(x / n for x in d))
    if orient == "forearm":
        dirs = _lowpass(dirs, float(kp["fps"]), tau)
    return pts, dirs


def _frame(z, x_prev):
    """Tool frame with z = z, its roll carried over from the previous frame
    (x_prev projected off z). The forearm's roll is not in the capture, and
    a frame built from a fixed world axis flips its roll as z passes near
    that axis -- J6 spun 130-175 deg between frames."""
    z = U._normalize(z)
    x = U._sub(x_prev, U._scale(z, U._dot(x_prev, z)))
    if math.sqrt(U._dot(x, x)) < 1e-6:
        return CAP.tool_frame(z)
    x = U._normalize(x)
    y = U._cross(z, x)
    return tuple((x[i], y[i], z[i]) for i in range(3))


def _blend(a, b, w):
    v = tuple((1 - w) * x + w * y for x, y in zip(a, b))
    n = math.sqrt(sum(x * x for x in v)) or 1.0
    return tuple(x / n for x in v)


def _needed_blend(kin, pts, dirs, fps=30.0, window_s=0.6):
    """How far each frame's tool direction must lean outward to be
    reachable (0, .25 .. 1), then smoothed in time -- a running maximum over
    window_s, then a Hann average -- so the lean never jumps between frames
    (it did, in 0.25 steps: J5 read 180x its acceleration limit there)."""
    raw = []
    for p, d in zip(pts, dirs):
        out = _blend(U._sub(p, SHOULDER), (0.0, 0.0, -1.0), 0.3)
        need = 1.0
        for w in (0.0, 0.25, 0.5, 0.75, 1.0):
            R = CAP.tool_frame(_blend(d, out, w))
            p6 = U._sub(p, U._mat_vec(R, (0.0, 0.0, kin.fo)))
            if ur_ik.within_limits(kin.model, ur_ik.solve(kin.model, R, p6)):
                need = w
                break
        raw.append(need)
    h = max(1, int(window_s * fps / 2))
    mx = [max(raw[max(0, i - h):i + h + 1]) for i in range(len(raw))]
    wts = [0.5 - 0.5 * math.cos(2 * math.pi * (k + 1) / (2 * h + 2)) for k in range(2 * h + 1)]
    out = []
    for i in range(len(mx)):
        acc = tot = 0.0
        for k, wt in enumerate(wts):
            j = i + k - h
            if 0 <= j < len(mx):
                acc += wt * mx[j]
                tot += wt
        out.append(acc / tot)
    return [min(1.0, max(o, r)) for o, r in zip(out, raw)]


def _ik_path(kin, pts, dirs, start=None, max_step=45.0):
    """Joint path, nearest branch to the previous frame, the tool's roll
    carried from frame to frame. The wrist position is what matters; when
    the forearm's direction cannot be reached there, the tool direction is
    blended towards pointing outward from the robot's shoulder until it can
    (a human forearm points where FR20's wrist often cannot). A frame with
    no solution keeps the last pose; the allowed step grows while it waits,
    so the path can catch up. Returns (joints, misses, mean_blend)."""
    ws = _needed_blend(kin, pts, dirs)
    prev = list(start or CH.HOME)
    R0 = CAP.tool_frame(dirs[0])
    x_prev = tuple(R0[i][0] for i in range(3))
    qs, misses, first, held, blends = [], 0, True, 0, []
    for p, d, w0 in zip(pts, dirs, ws):
        out = _blend(U._sub(p, SHOULDER), (0.0, 0.0, -1.0), 0.3)
        q, used = None, None
        for w in [w0] + [x for x in (0.25, 0.5, 0.75, 1.0) if x > w0]:
            R = _frame(_blend(d, out, w), x_prev)
            p6 = U._sub(p, U._mat_vec(R, (0.0, 0.0, kin.fo)))
            sols = ur_ik.within_limits(kin.model, ur_ik.solve(kin.model, R, p6, q6_when_singular=prev[5]))
            best = ur_ik.nearest(sols, prev)
            if best is None:
                continue
            cand = [b - 360.0 * round((b - a) / 360.0) for a, b in zip(prev, best["q"])]
            if first or max(abs(a - b) for a, b in zip(cand, prev)) <= max_step * (1 + held):
                q, used = cand, (R, w)
                break
        if q is None:
            misses += 1
            held += 1
            q = list(prev)
        else:
            held = 0
            x_prev = tuple(used[0][i][0] for i in range(3))
            blends.append(used[1])
        first = False
        qs.append(q)
        prev = q
    return qs, misses, (sum(blends) / len(blends) if blends else 1.0)


def _resample(ts, qs, fps=24.0, ease=0.0):
    """24 fps frames of a timed joint path. ease (s): half-cosine ramps at
    both ends (retime_ease), so a take that starts and ends mid-motion
    starts and ends at rest -- its velocity jumped where the lead-in met it,
    and the player read that as 9x the acceleration limit."""
    import retime_ease
    total = ts[-1]
    ease = min(ease, total / 2.0)
    dur = retime_ease.eased_duration(total, ease, ease)
    out_t = [i / fps for i in range(int(dur * fps) + 1)]
    out_q, k = [], 0
    for tau in out_t:
        t = min(total, retime_ease.source_time(tau, total, ease, ease))
        k = 0 if t < ts[k] else k
        while k < len(ts) - 2 and ts[k + 1] <= t:
            k += 1
        f = (t - ts[k]) / (ts[k + 1] - ts[k])
        out_q.append([x + f * (y - x) for x, y in zip(qs[k], qs[k + 1])])
    return out_t, out_q


def _ease_in_out(ts, qs, home, kin=None, secs=1.5):
    """A move from home to the first pose and back, so the clip starts and
    ends at rest where every clip does; as long as the joint limits need."""
    need_in = CH._travel_time(kin, home, qs[0]) * 1.3 if kin else secs
    need_out = CH._travel_time(kin, qs[-1], home) * 1.3 if kin else secs
    n_in, n_out = max(12, int(need_in * 24)), max(12, int(need_out * 24))
    pre = [[h + (a - h) * CH._minjerk(i / float(n_in)) for h, a in zip(home, qs[0])] for i in range(n_in)]
    post = [[a + (h - a) * CH._minjerk((i + 1) / float(n_out)) for h, a in zip(home, qs[-1])] for i in range(n_out)]
    allq = pre + qs + post
    return [i / 24.0 for i in range(len(allq))], allq


def _finish(clip, ts, qs, kin, env, extra):
    clip["points"] = [{"t": round(t, 6), "q": [round(x, 5) for x in q]} for t, q in zip(ts, qs)]
    clip["tcp"] = M._tcp_path("fr20", qs)
    xs = list(zip(*clip["tcp"]))
    clip["meta"]["bounds"] = {"min": [min(a) for a in xs], "max": [max(a) for a in xs]}
    clip["meta"]["duration_s"] = round(ts[-1], 6)
    clip["style"].update(extra)
    M.measure(clip, acc=kin.acc)
    if env is not None:
        rep = CL.check(kin.col, env, ts, qs)
        clip["safety"]["collision"] = CL.describe(rep)
        clip["safety"]["min_clearance_m"] = rep["min_env_clearance_m"]
        if not rep["ok"]:
            clip["safety"]["ok"] = False
            clip["safety"]["reasons"] = ["cell: " + CL.describe(rep)] + clip["safety"]["reasons"]
    clip["labels"] = L.label(clip)
    return clip


def _clip_shell(kp, cid, how):
    return {"schema": M.SCHEMA, "id": cid, "robot": "fr20", "joint_names": ["j%d" % i for i in range(1, 7)],
            "units": {"angle": "deg", "time": "s", "length": "m"}, "points": [], "tcp": None,
            "style": {"generator": "retarget.py", "method": how},
            "meta": {"duration_s": 0.0, "bounds": None, "tags": ["retarget", how],
                     "source": {"keypoints": kp.get("source", "?"), "fps": kp["fps"]}}}


def direct(kp, env=None, kin=None, mirror=False, clip_id="retarget_direct", safety=0.9,
           size=1.0, orient="outward", sigma_s=0.1):
    kin = kin or CH.Kin()
    kp = smooth(kp, sigma_s)
    n = len(kp["joints"]["wrist"])
    ts0 = [i / float(kp["fps"]) for i in range(n)]
    gain, tempo, need = 1.0, 1.0, None
    for _ in range(10):
        pts, dirs = targets(kp, gain, mirror, size, orient)
        qs, misses, blend = _ik_path(kin, pts, dirs)
        ts, q24 = _resample([t * tempo for t in ts0], qs, ease=1.0)
        ts, q24 = _ease_in_out(ts, q24, CH.HOME, kin)
        need = P.limiting(ts, q24, 125.0, [v * safety for v in kin.vel], [a * safety for a in kin.acc])["scale_needed"]
        if need <= 1.0:
            break
        if gain > 0.5:
            gain = max(0.5, gain / need)           # acceleration ~ size at the same timing
        elif tempo * need > 3.0:
            break                                  # a third of the performer's speed at half the size: give up
        else:
            tempo *= need * 1.02
    clip = _clip_shell(kp, clip_id, "direct")
    if need > 1.0:
        clip["safety"] = {"ok": False, "playback_scale": None,
                          "reasons": ["too fast for this arm: needs x%.1f slower even at half size" % (tempo * need)]}
        return clip
    return _finish(clip, ts, q24, kin, env, {"gain": round(gain, 3), "tempo_scale": round(tempo, 3),
                                             "ik_misses": misses, "tool_blend": round(blend, 3), "mirror": mirror,
                                             "size": size, "orient": orient})


def _strokes_directness(pts, speeds, frac=0.1):
    """chord / arc of each stroke between stops, summed: an out-and-back jab
    is two straight strokes, not one that went nowhere."""
    pk = max(speeds) or 1e-9
    chord = arc = 0.0
    start, sarc = None, 0.0
    for i, v in enumerate(speeds):
        if v > frac * pk:
            if start is None:
                start, sarc = i, 0.0
            sarc += math.dist(pts[i + 1], pts[i])
        elif start is not None:
            chord += math.dist(pts[i], pts[start])
            arc += sarc
            start = None
    if start is not None:
        chord += math.dist(pts[-1], pts[start])
        arc += sarc
    return chord / arc if arc > 1e-9 else 1.0


def effort_windows(kp, window_s=2.0, sigma_s=0.05):
    """Per window of the take: raw efforts, the same relative to the rest of
    the take (Laban qualities are contrasts), and the nearest action.
    weight = wrist speed (p90), time = wrist acceleration / speed, space =
    stroke directness, still = share of the window spent (nearly) still."""
    j = smooth(kp, sigma_s)["joints"]
    fps = float(kp["fps"])
    n = len(j["wrist"])
    dt = 1.0 / fps
    wr = j["wrist"]
    vw = [math.dist(wr[i + 1], wr[i]) / dt for i in range(n - 1)]
    vec = [tuple((wr[i + 1][k] - wr[i][k]) / dt for k in range(3)) for i in range(n - 1)]
    aw = [math.dist(vec[i + 1], vec[i]) / dt for i in range(n - 2)]
    peak_all = sorted(vw)[int(0.95 * (len(vw) - 1))] or 1e-9
    w = max(8, int(window_s * fps))
    raw = []
    for s0 in range(0, n - w // 2, w):
        e = min(n - 1, s0 + w)
        vv, aa = vw[s0:e] or [0.0], aw[s0:e - 1] or [0.0]
        p90 = sorted(vv)[int(0.9 * (len(vv) - 1))] or 1e-9
        raw.append({"t0": s0 / fps, "t1": e / fps, "weight": p90,
                    "time": sorted(aa)[int(0.9 * (len(aa) - 1))] / p90,
                    "space": _strokes_directness(wr[s0:e + 1], vv),
                    "still": sum(1 for v in vv if v < 0.1 * peak_all) / len(vv)})
    for key in ("weight", "time", "space", "still"):
        vals = sorted(r[key] for r in raw)
        mid = vals[len(vals) // 2]
        spread = (vals[-1] - vals[0]) / 2 or 1.0
        for r in raw:
            r[key + "_rel"] = max(-1.0, min(1.0, (r[key] - mid) / spread))
    for r in raw:
        r["action"] = L.nearest_action(r["weight_rel"], r["time_rel"], r["space_rel"])
        r["flow"] = -r["still_rel"]
    return raw


def effort(kp, env=None, kin=None, clip_id="retarget_effort", seed=0, bpm=None):
    kin = kin or CH.Kin()
    wins = effort_windows(kp)
    # a bar per window; tempo from the window length unless given (4 beats a window)
    bpm = bpm or int(round(60.0 * 4 / max(1e-6, wins[0]["t1"] - wins[0]["t0"])))
    bpm = max(60, min(130, bpm))
    flow = sum(w["flow"] for w in wins) / len(wins)
    spec = {"bars": [{"action": w["action"]} for w in wins], "bpm": bpm, "flow": round(flow, 2)}
    clip = CH.make_clip(spec, seed=seed, env=env, clip_id=clip_id, kin=kin, tags=("retarget", "effort"))
    clip["style"]["method"] = "effort"
    clip["style"]["windows"] = [{k: (round(v, 3) if isinstance(v, float) else v) for k, v in w.items()} for w in wins]
    return clip


# --------------------------------------------------------------------------
# a synthetic take, for tests (and as a template for real captures)
# --------------------------------------------------------------------------

def synthetic_take(fps=30, parts=(("float", 4.0), ("punch", 4.0), ("float", 4.0)), seed=0, fade=0.5):
    """A right arm: slow floating sweeps and quick guarded jabs, parts
    cross-faded over `fade` seconds so the take is continuous, as a real
    capture is. Shoulder fixed at 1.4 m, upper arm 0.3 m, forearm 0.28 m,
    2 mm tracker noise."""
    rng = random.Random(seed)
    sh = (0.0, -0.2, 1.4)
    up_len, fore_len = 0.3, 0.28

    def pose(kind, t):
        if kind == "punch":                                  # a jab a second, from a guard
            k = t % 1.0
            if k < 0.2:
                ext = CH._minjerk(k / 0.2)
            elif k < 0.3:
                ext = 1.0
            elif k < 0.7:
                ext = 1.0 - CH._minjerk((k - 0.3) / 0.4)
            else:
                ext = 0.0
            return 0.15, 0.05, 1.5 - 1.35 * ext
        return (0.45 * math.sin(2 * math.pi * t / 4.0), 0.25 * math.sin(2 * math.pi * t / 4.0 + 1.0),
                0.7 + 0.2 * math.sin(2 * math.pi * t / 3.0))

    bounds, t0 = [], 0.0
    for kind, dur in parts:
        bounds.append((kind, t0, t0 + dur))
        t0 += dur
    total = t0
    J = {"shoulder": [], "elbow": [], "wrist": [], "hand": []}
    for i in range(int(total * fps)):
        t = i / float(fps)
        ws = []
        for kind, a0, b0 in bounds:
            wa = 1.0 if a0 == 0 else CH._minjerk((t - a0 + fade / 2) / fade)
            wb = 1.0 if b0 == total else 1.0 - CH._minjerk((t - b0 + fade / 2) / fade)
            ws.append((kind, max(0.0, min(wa, wb))))
        tot = sum(w for _, w in ws) or 1.0
        az = sum(w * pose(k, t)[0] for k, w in ws) / tot
        el = sum(w * pose(k, t)[1] for k, w in ws) / tot
        bend = sum(w * pose(k, t)[2] for k, w in ws) / tot
        d_up = (math.cos(el) * math.cos(az - 0.3), math.cos(el) * math.sin(az - 0.3), math.sin(el) - 0.2)
        nu = math.sqrt(sum(x * x for x in d_up))
        d_up = tuple(x / nu for x in d_up)
        el_pt = tuple(s_ + up_len * d for s_, d in zip(sh, d_up))
        d_fore = (d_up[0] * math.cos(bend), d_up[1], d_up[2] * math.cos(bend) + 0.8 * math.sin(bend))
        nf = math.sqrt(sum(x * x for x in d_fore))
        d_fore = tuple(x / nf for x in d_fore)
        wr = tuple(e + fore_len * d for e, d in zip(el_pt, d_fore))
        hd = tuple(w_ + 0.08 * d for w_, d in zip(wr, d_fore))
        for k_, v in (("shoulder", sh), ("elbow", el_pt), ("wrist", wr), ("hand", hd)):
            J[k_].append([round(x + rng.gauss(0, 0.002), 5) for x in v])
    return {"schema": SCHEMA, "fps": fps, "units": "m", "frame": "performer: x forward, y left, z up",
            "side": "right", "source": "synthetic_take %s" % "-".join(p for p, _ in parts), "joints": J}


if __name__ == "__main__":
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + detail) if detail else ""))
        if not ok:
            fails.append(label)

    env = CL.load_env(os.path.join(ROOT, "envs", "volvox_lab.json"))
    kin = CH.Kin()
    waves = synthetic_take(parts=(("float", 8.0),))
    c = direct(waves, env, kin)
    st = c["style"]
    check("direct: a slow take (waves) becomes a clip that plays at its own speed and clears the cell",
          c["safety"]["ok"], "gain %s, tempo x%s, %s IK misses, tool blend %s, %.1f s; %s"
          % (st.get("gain"), st.get("tempo_scale"), st.get("ik_misses"), st.get("tool_blend"),
             c["meta"]["duration_s"], c["safety"].get("reasons")))
    if c["safety"]["ok"]:
        pts, _ = targets(smooth(waves, 0.1), st["gain"], False, st["size"], st["orient"])
        # every target point should lie on the clip's TCP path (timing aside)
        err = sorted(min(math.dist(p, x) for x in c["tcp"]) for p in pts[::3])
        med = err[len(err) // 2] if err else 1.0
        check("direct: the TCP follows the performer's scaled wrist", med < 0.03, "median %.1f mm" % (med * 1000))
    jabs = synthetic_take(parts=(("punch", 4.0),))
    j = direct(jabs, env, kin)
    js = j["style"]
    check("direct: human-speed jabs only fit smaller and slower -- and say so",
          j["safety"]["ok"] and (js.get("gain", 1.0) < 1.0 or js.get("tempo_scale", 1.0) > 1.0),
          "gain %s, tempo x%s, measured %s" % (js.get("gain"), js.get("tempo_scale"),
                                              j.get("labels", {}).get("measured", {}).get("action")))
    kp = synthetic_take()
    wins = effort_windows(kp)
    seq = [w["action"] for w in wins]
    jab = [w for w in wins if 4.0 <= w["t0"] < 8.0]
    wave = [w for w in wins if w["t1"] <= 4.0 or w["t0"] >= 8.0]
    check("effort: the quick jabs read as more sudden and more direct than the waves",
          min(w["time_rel"] for w in jab) > max(w["time_rel"] for w in wave)
          and min(w["space_rel"] for w in jab) > max(w["space_rel"] for w in wave),
          "windows %s" % seq)
    e = effort(kp, env, kin)
    check("effort: a robot-native phrase with the take's action sequence", e["safety"].get("ok"),
          "%s -> %s" % (seq, e["labels"].get("sequence")))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    sys.exit(1 if fails else 0)
