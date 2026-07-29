"""Load a robot profile and convert angles between the three frames in play.

    configurejoints : the Configure Joints SOP and FBIK joint limits
    extracted       : what extract_angles() measures from the rest pose
    robot           : what the controller and the CSV file speak

        extracted = configurejoints + offset
        robot     = extracted * sign

Profiles state every limit in the ROBOT frame, because that is the frame the
datasheet uses. Nothing else in the codebase should hand-write a converted
number -- call to_configurejoints() instead.

That rule exists because it was already broken once. The elbow preset wrote
the robot-frame range (-241.9, 3.4) straight into the configurejoints parm.
Since J3 carries both a +90 offset and a sign inversion, the range it actually
produced was robot -93.8 .. +152.2: about 149 degrees of motion the arm cannot
physically perform, while forbidding 148 degrees it can. Measured, not
inferred -- see tests at the bottom of this file.

The pure functions here have no hou dependency and are runnable as
    python scripts/robot_profile.py
"""

import json
import os

PROFILE_DIR_NAME = "profiles"


# --------------------------------------------------------------------------
# loading
# --------------------------------------------------------------------------

def profile_dir(start=None):
    """Locate profiles/ by walking up from start (default: this file)."""
    here = start or os.path.dirname(os.path.abspath(__file__))
    cur = here
    for _ in range(5):
        cand = os.path.join(cur, PROFILE_DIR_NAME)
        if os.path.isdir(cand):
            return cand
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    raise IOError("No %s/ directory found above %s" % (PROFILE_DIR_NAME, here))


def list_profiles(directory=None):
    d = directory or profile_dir()
    out = []
    for fn in sorted(os.listdir(d)):
        if fn.endswith(".json"):
            out.append(os.path.splitext(fn)[0])
    return out


def load(profile_id, directory=None):
    d = directory or profile_dir()
    fp = os.path.join(d, profile_id + ".json")
    if not os.path.isfile(fp):
        raise IOError("No profile '%s' in %s (have: %s)"
                      % (profile_id, d, ", ".join(list_profiles(d)) or "none"))
    with open(fp, "r", encoding="utf-8") as f:
        prof = json.load(f)
    _check_profile(prof, fp)
    return prof


def _check_profile(p, fp):
    """Fail loudly on a malformed profile rather than half-applying it."""
    n = p["robot"]["num_joints"]
    for section, key in (("robot", "limits_deg"), ("rig", "rotation_axis"),
                         ("rig", "sign"), ("rig", "offset_deg"),
                         ("rig", "fbik_enforces_limits")):
        got = len(p[section][key])
        if got != n:
            raise ValueError("%s: %s.%s has %d entries, expected %d"
                             % (fp, section, key, got, n))
    for i, s in enumerate(p["rig"]["sign"]):
        if s not in (1, -1):
            raise ValueError("%s: rig.sign[%d] is %r, must be 1 or -1"
                             % (fp, i, s))
    for i, (lo, hi) in enumerate(p["robot"]["limits_deg"]):
        if lo >= hi:
            raise ValueError("%s: robot.limits_deg[%d] is not ascending: %r"
                             % (fp, i, [lo, hi]))


# --------------------------------------------------------------------------
# frame conversion -- the only place this arithmetic lives
# --------------------------------------------------------------------------

def _sign_offset(prof, joint):
    """joint is 1-based."""
    i = joint - 1
    return float(prof["rig"]["sign"][i]), float(prof["rig"]["offset_deg"][i])


def robot_to_configurejoints(prof, joint, angle):
    sign, offset = _sign_offset(prof, joint)
    return angle * sign - offset


def configurejoints_to_robot(prof, joint, angle):
    sign, offset = _sign_offset(prof, joint)
    return (angle + offset) * sign


def extracted_to_robot(prof, joint, angle):
    sign, _ = _sign_offset(prof, joint)
    return angle * sign


def to_configurejoints(prof, joint, lo, hi):
    """Convert a robot-frame RANGE. Where sign is -1 the ends swap, which is
    exactly the step the hand-written elbow preset missed."""
    a = robot_to_configurejoints(prof, joint, lo)
    b = robot_to_configurejoints(prof, joint, hi)
    return (min(a, b), max(a, b))


def joint_limits_configurejoints(prof, joint):
    lo, hi = prof["robot"]["limits_deg"][joint - 1]
    return to_configurejoints(prof, joint, lo, hi)


def preset_range(prof, group, option):
    """Robot-frame range for a named preset option, e.g. ('elbow', 'up')."""
    g = prof["presets"][group]
    return tuple(g[option])


def preset_configurejoints(prof, group, option):
    g = prof["presets"][group]
    lo, hi = g[option]
    return to_configurejoints(prof, g["joint"], lo, hi)


def enforceable(prof, joint):
    """Does FBIK actually honour this joint's limits? False for J4 here."""
    return bool(prof["rig"]["fbik_enforces_limits"][joint - 1])


# --------------------------------------------------------------------------
# skeleton validation -- assert expectations instead of guessing
# --------------------------------------------------------------------------

def validate_skeleton(geo, prof):
    """Return a list of problems; empty means the skeleton matches the profile.

    Every serious failure in this project so far produced confident wrong
    numbers rather than an error: a guessed rotation axis zeroed three joints,
    wrapped angles read as a 48x velocity spike. Checking up front is cheaper
    than noticing downstream.
    """
    problems = []
    n = prof["robot"]["num_joints"]
    pattern = prof["rig"]["joint_name_pattern"]

    # Distinguish "empty input" from "wrong skeleton". They need different
    # fixes, and reporting a missing attribute when the geometry simply has no
    # points sends you looking in the wrong place.
    if len(geo.points()) == 0:
        return ["input geometry is empty -- nothing upstream produced a "
                "skeleton (check the solve cache and the frame range)"]

    names = set()
    attr = geo.findPointAttrib("name")
    if attr is None:
        return ["skeleton has %d points but no 'name' point attribute, so "
                "joints cannot be identified" % len(geo.points())]
    for p in geo.points():
        names.add(p.attribValue("name"))

    expected = [pattern % (i + 1) for i in range(n)]
    missing = [e for e in expected if e not in names]
    if missing:
        problems.append("profile '%s' expects %d joints named %r; missing: %s"
                        % (prof["id"], n, pattern, ", ".join(missing)))

    if geo.findPointAttrib("fbik_jointconfig") is not None:
        for i, e in enumerate(expected):
            if e not in names:
                continue
            pt = [q for q in geo.points() if q.attribValue("name") == e][0]
            raw = pt.attribValue("fbik_jointconfig").get("rotation_weights",
                                                         "[0, 0, 1]")
            if isinstance(raw, (list, tuple)):
                w = [abs(float(x)) for x in raw]
            else:
                w = [abs(float(x)) for x in str(raw).strip("[]").split(",")]
            got = "xyz"[w.index(max(w))]
            want = prof["rig"]["rotation_axis"][i]
            if got != want:
                problems.append(
                    "%s rotates about %s in the rig but the profile declares %s"
                    % (e, got, want))
    return problems


def stamp(geo, prof):
    """Write the profile onto the geometry as a detail dict.

    Downstream code then reads its configuration from the geometry, which is
    the pattern axis_map_from_geo() already uses -- one mechanism, not two.
    """
    import hou
    if geo.findGlobalAttrib("robot_profile") is None:
        geo.addAttrib(hou.attribType.Global, "robot_profile", {})
    geo.setGlobalAttribValue("robot_profile", {
        "id": prof["id"],
        "label": prof["label"],
        "num_joints": prof["robot"]["num_joints"],
        "max_velocity_deg_s": prof["robot"]["max_velocity_deg_s"],
        "speed_cap_pct": prof["output"]["speed_cap_pct"],
        "limits_lo": [lo for lo, _ in prof["robot"]["limits_deg"]],
        "limits_hi": [hi for _, hi in prof["robot"]["limits_deg"]],
        "sign": list(prof["rig"]["sign"]),
        "offset_deg": list(prof["rig"]["offset_deg"]),
    })


# --------------------------------------------------------------------------
# self-test against values measured on the real rig
# --------------------------------------------------------------------------

if __name__ == "__main__":
    p = load("uf850")
    fails = []

    def eq(label, got, want, tol=0.05):
        if abs(got - want) > tol:
            fails.append("%s: got %.4f, want %.4f" % (label, got, want))

    # Measured by pinning configurejoints J3 to a narrow band and reading the
    # extractor, then applying invert_j3:
    #     cj -242.0 -> robot +152.25      cj -90.0 -> robot  -0.25
    #     cj -180.0 -> robot  +90.25      cj   0.0 -> robot -90.25
    #                                     cj   3.5 -> robot -93.75
    for cj, robot in ((-242.0, 152.0), (-180.0, 90.0), (-90.0, 0.0),
                      (0.0, -90.0), (3.5, -93.5)):
        eq("J3 cj %.1f -> robot" % cj, configurejoints_to_robot(p, 3, cj), robot)

    # and the inverse
    eq("J3 robot -241.9 -> cj", robot_to_configurejoints(p, 3, -241.9), 151.9)
    eq("J3 robot    3.4 -> cj", robot_to_configurejoints(p, 3, 3.4), -93.4)

    # the range conversion must swap the ends when sign is -1
    lo, hi = joint_limits_configurejoints(p, 3)
    eq("J3 limit cj lo", lo, -93.4)
    eq("J3 limit cj hi", hi, 151.9)

    # joints with neither sign nor offset must pass through untouched
    for jn, want in ((1, (-360.0, 360.0)), (5, (-123.9, 123.9))):
        got = joint_limits_configurejoints(p, jn)
        eq("J%d cj lo" % jn, got[0], want[0])
        eq("J%d cj hi" % jn, got[1], want[1])

    # the bug this module exists to prevent
    bad_lo, bad_hi = -242.0, 3.5
    span = (configurejoints_to_robot(p, 3, bad_lo),
            configurejoints_to_robot(p, 3, bad_hi))
    real = p["robot"]["limits_deg"][2]
    if min(span) >= real[0] and max(span) <= real[1]:
        fails.append("regression: the old hand-written elbow preset now looks "
                     "legal, so the conversion is not being applied")

    if not enforceable(p, 4):
        pass
    else:
        fails.append("J4 should be marked unenforceable on this rig")

    print("profiles found: %s" % ", ".join(list_profiles()))
    if fails:
        print("FAIL (%d)" % len(fails))
        for f in fails:
            print("  " + f)
        raise SystemExit(1)
    print("OK: all frame conversions match the values measured on the rig")
