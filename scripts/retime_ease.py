"""Ease-in / ease-out for a retimed clip: start and end at rest.

Retime plans how fast the path may be traversed; time-optimal mode runs it at
the limit from the first frame, so the exported clip starts already moving
(measured: 91 deg/s between frames 1 and 2). A controller has to get there
from rest, and the playback conditioner can only do that by slowing the
WHOLE clip -- 9.7 s became 58 s at 300 deg/s^2.

This warps playback time instead: over the first ease_in seconds the planned
speed ramps from 0 to 1 (half-cosine), over the last ease_out seconds back to
0, and in between the plan is untouched. A ramp covers half its length of
planned time, so the clip grows by (ease_in + ease_out) / 2.

    source_time(tau)   planned time reached at output time tau
    eased_duration()   output length for a planned length

Pure Python, no hou -- run the tests with:  python scripts/retime_ease.py
"""

import math


def _fit(total, ease_in, ease_out):
    """Ramps longer than the plan allows are shrunk in proportion: together
    they may cover at most the whole planned time."""
    ease_in, ease_out = max(0.0, ease_in), max(0.0, ease_out)
    covered = (ease_in + ease_out) / 2.0
    if covered > total > 0.0:
        k = total / covered
        ease_in, ease_out = ease_in * k, ease_out * k
    return ease_in, ease_out


def eased_duration(total, ease_in, ease_out):
    """Output duration for a planned duration total."""
    a, b = _fit(total, ease_in, ease_out)
    return total + (a + b) / 2.0


def source_time(tau, total, ease_in, ease_out):
    """Planned time reached at output time tau (both in seconds)."""
    a, b = _fit(total, ease_in, ease_out)
    out = total + (a + b) / 2.0
    if tau <= 0.0:
        return 0.0
    if tau >= out:
        return total
    if a > 0.0 and tau < a:                        # speed (1 - cos(pi tau/a)) / 2
        return (tau - a / math.pi * math.sin(math.pi * tau / a)) / 2.0
    t = a / 2.0 + (tau - a)
    tail = out - b
    if b > 0.0 and tau > tail:                     # speed (1 + cos(pi s/b)) / 2
        s = tau - tail
        t = a / 2.0 + (tail - a) + s / 2.0 + b / (2.0 * math.pi) * math.sin(math.pi * s / b)
    return min(t, total)


if __name__ == "__main__":
    import sys
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + detail) if detail else ""))
        if not ok:
            fails.append(label)

    T, a, b = 5.0, 0.5, 0.8
    out = eased_duration(T, a, b)
    check("duration grows by (in + out) / 2", abs(out - (T + 0.65)) < 1e-12, "%.4f s" % out)
    n = 20000
    taus = [out * k / n for k in range(n + 1)]
    ts = [source_time(x, T, a, b) for x in taus]
    check("ends map to ends", ts[0] == 0.0 and abs(ts[-1] - T) < 1e-12)
    check("monotonic", all(y >= x - 1e-15 for x, y in zip(ts, ts[1:])))
    h = out / n
    v0 = (ts[1] - ts[0]) / h
    v1 = (ts[-1] - ts[-2]) / h
    vmid = (source_time(2.5 + 1e-4, T, a, b) - source_time(2.5 - 1e-4, T, a, b)) / 2e-4
    check("at rest at both ends, full speed in the middle",
          v0 < 1e-3 and v1 < 1e-3 and abs(vmid - 1.0) < 1e-9, "%.2e, %.2e, %.6f" % (v0, v1, vmid))
    jumps = max(abs(ts[k + 1] - 2 * ts[k] + ts[k - 1]) / (h * h) for k in range(1, n))
    check("speed continuous (bounded acceleration)", jumps < 2 * math.pi / min(a, b) + 1e-3,
          "max d2t/dtau2 %.3f" % jumps)
    check("no easing is the identity",
          all(abs(source_time(x, T, 0, 0) - x) < 1e-12 for x in (0.0, 1.0, 4.9, 5.0)))
    short = eased_duration(0.4, 0.5, 0.5)
    check("ramps longer than the clip are shrunk", abs(short - 0.8) < 1e-12, "%.3f s" % short)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    sys.exit(1 if fails else 0)
