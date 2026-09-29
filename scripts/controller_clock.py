"""How fast the controller's clock runs against this PC's -- read only, the arm does not move.

    uv run scripts/controller_clock.py --ip 192.168.58.2 [--seconds 120]

A diagnostic: the clock is only part of how fast the controller plays
ServoJ. show_stream.py paces by the playback rate (playback.toml
[controller_playback_ppm] by IP), which a stream run's report measures
(playback_ppm_suggested) -- the real FR20: clock -126 ppm, playback -950.
(An earlier --write put this clock's number where the playback rate
belongs; gone.)

Reads the controller's own clock (GetSystemClock, ms, Fairino SDK) twice a
second, each read stamped at the middle of its round trip on this PC's
clock, and fits a line: the slope minus one is the drift in ppm (a negative
drift: the controller's clock is slower). SimMachine: ~ +14 ppm, and that
is also how much slower it plays ServoJ. The real FR20 (2026-09-28): its
clock -126 ppm, but it plays ServoJ ~950 ppm slower than cmdT says, so the
clock is only part of it.

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


def self_test():
    fails = []
    pairs = [(t, 5.0 + t * (1.0 - 950e-6)) for t in [i * 0.5 for i in range(120)]]
    ppm = fit_ppm(pairs)
    if abs(ppm + 950.0) > 1e-3:
        fails.append("fit %.3f" % ppm)
    print("ok    a controller 950 ppm slow is measured as -950 ppm" if not fails else "FAIL  %s" % fails)
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", required=True)
    ap.add_argument("--seconds", type=float, default=120.0)
    a = ap.parse_args()
    measure(a.ip, a.seconds)
