"""How fast the controller's clock runs against this PC's -- read only, the arm does not move.

    uv run scripts/controller_clock.py --ip 192.168.58.2 [--seconds 120] [--write]

--write keeps the result in playback.toml under [controller_clock_ppm], by
the controller's IP; show_stream.py paces its points by it (GetSystemClock
cannot be read while streaming: it takes ~300 ms in servo mode).

Reads the controller's own clock (GetSystemClock, ms, Fairino SDK) twice a
second, each read stamped at the middle of its round trip on this PC's
clock, and fits a line: the slope minus one is the drift in ppm (a negative
drift: the controller's clock is slower, so ServoJ points -- played one per
cmdT on that clock -- are played slower than this PC sends them, and a
PC-paced stream piles up in the controller; the real FR20 streams showed
~ -950 ppm, 2026-09-28). SimMachine: ~ +14 ppm.

    python scripts/controller_clock.py --self-test
"""

import argparse
import sys
import time
import xmlrpc.client


def read_pair(rpc):
    """(PC time at the middle of the call, controller ms), or None."""
    t0 = time.perf_counter()
    ret = rpc.GetSystemClock()
    t1 = time.perf_counter()
    if not isinstance(ret, (list, tuple)) or ret[0] != 0:
        return None
    return (t0 + t1) / 2.0, float(ret[1]) / 1000.0, t1 - t0


def fit_ppm(pairs):
    """The drift (ppm) of the controller's clock against the PC's: the slope
    of controller time on PC time, least squares, minus one."""
    n = len(pairs)
    mx = sum(p[0] for p in pairs) / n
    my = sum(p[1] for p in pairs) / n
    sxx = sum((p[0] - mx) ** 2 for p in pairs)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pairs)
    return (sxy / sxx - 1.0) * 1e6


def measure(ip, seconds, every=0.5, log=print):
    rpc = xmlrpc.client.ServerProxy("http://%s:20003" % ip)
    pairs, slow = [], 0
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        p = read_pair(rpc)
        if p is None:
            raise SystemExit("GetSystemClock did not answer: this controller cannot be paced by its clock")
        if p[2] <= 0.02:                              # a slow round trip: when it was read is not known well
            pairs.append(p[:2])
        else:
            slow += 1
        time.sleep(every)
    ppm = fit_ppm(pairs)
    log("controller clock vs this PC over %.0f s (%d reads, %d slow ones left out): %+.0f ppm" % (
        seconds, len(pairs), slow, ppm))
    log("  = %+.1f ms per minute; a PC-paced stream would pile up %.1f ms per minute in the controller"
        % (ppm * 60e-3, max(0.0, -ppm) * 60e-3))
    return ppm


def write_toml(ip, ppm, path=None):
    """playback.toml [controller_clock_ppm] "<ip>" = ppm (the section made
    if missing, the IP's line replaced)."""
    import os
    path = path or os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "playback.toml")
    lines = open(path, encoding="utf8").read().splitlines() if os.path.exists(path) else []
    key = '"%s" = %.1f' % (ip, ppm)
    if "[controller_clock_ppm]" not in [l.split("#")[0].strip() for l in lines]:
        lines += ["", "[controller_clock_ppm]          # scripts/controller_clock.py --write: the controller's clock vs "
                      "this PC's, by IP", key]
    else:
        i = [l.split("#")[0].strip() for l in lines].index("[controller_clock_ppm]") + 1
        j = i
        while j < len(lines) and not lines[j].strip().startswith("["):
            if lines[j].strip().startswith('"%s"' % ip):
                lines[j] = key
                break
            j += 1
        else:
            lines.insert(i, key)
    with open(path, "w", encoding="utf8", newline="\n") as f:
        f.write("\n".join(lines) + "\n")
    return path


def self_test():
    fails = []
    pairs = [(t, 5.0 + t * (1.0 - 950e-6)) for t in [i * 0.5 for i in range(120)]]
    ppm = fit_ppm(pairs)
    if abs(ppm + 950.0) > 1e-3:
        fails.append("fit %.3f" % ppm)
    print("ok    a controller 950 ppm slow is measured as -950 ppm" if not fails else "FAIL  %s" % fails)
    import os
    import tempfile
    import tomllib
    p = os.path.join(tempfile.mkdtemp(), "playback.toml")
    open(p, "w").write('[robot]\ntarget = "sim"\n')
    write_toml("10.0.0.9", -950.25, p)
    write_toml("10.0.0.8", 14.0, p)
    write_toml("10.0.0.9", -948.0, p)
    got = tomllib.load(open(p, "rb")).get("controller_clock_ppm", {})
    ok = got == {"10.0.0.9": -948.0, "10.0.0.8": 14.0}
    print(("ok  " if ok else "FAIL") + "  --write keeps one value per controller in playback.toml  -- %s" % got)
    fails += [] if ok else ["toml"]
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", required=True)
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--write", action="store_true", help="keep it in playback.toml for this IP")
    a = ap.parse_args()
    ppm = measure(a.ip, a.seconds)
    if a.write:
        print("written to", write_toml(a.ip, ppm))
