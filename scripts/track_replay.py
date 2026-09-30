"""A Femto recording (TD people_track's Record, femto_format's CSV) replayed as TouchDesigner sends it -- /track/hands
then /track/people a frame, robot frame, in real time -- to Isaac's run_tracking --osc-in (9010) or track_mode (9011),
without TD. Optionally a crouch added (heads and hands lowered, smoothly) to try the interactive mode's.

    python scripts/track_replay.py geo/tracking/femto_rec_20260930-150731.csv
    python scripts/track_replay.py geo/tracking/femto_rec_20260930-150731.csv --crouch 13-18 --port 9010
    python scripts/track_replay.py --self-test

With Isaac (the interactive mode, TD's LEDs on the strip: TD pixel_scan > LEDs to Isaac on):

    C:/isaacsim6/python.bat scripts/isaac/run_tracking.py --live --osc-in 9010 --engage --td

The extrinsic: femto_extrinsic.json beside the recording (--extrinsic to say another).
"""

import argparse
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import femto_format as FF  # noqa: E402

CROUCH_M, CROUCH_RAMP_S = 0.5, 1.0


def crouch_drop(t, span, depth=CROUCH_M, ramp=CROUCH_RAMP_S):
    """How far down (m) a crouch over span (t0, t1) is at t: 0 outside it, depth inside, eased in and out over ramp."""
    if span is None:
        return 0.0
    t0, t1 = span
    if t <= t0 - ramp or t >= t1 + ramp:
        return 0.0
    f = (t - (t0 - ramp)) / ramp if t < t0 else (1.0 - (t - t1) / ramp if t > t1 else 1.0)
    return depth * 0.5 * (1.0 - math.cos(math.pi * f))


def frames(rec, extrinsic, crouch=None):
    """[(t, people, hands)] of a recording in the robot frame (femto_format.messages), a crouch added."""
    e = json.load(open(extrinsic))
    out = []
    for _, t, bodies in FF.read(rec):
        people, hands = FF.messages(bodies, e["R"], e["p"])
        dz = crouch_drop(t, crouch)
        out.append((t, [(p, x, y, z - dz, c) for p, x, y, z, c in people],
                    [(p, x, y, z - dz, c) for p, x, y, z, c in hands]))
    return out


def osc_frame(t, people, hands):
    """[(address, args)] of a frame, as TD's people_track sends it: the hands first."""
    flat = lambda items: [v for it in items for v in it]  # noqa: E731
    return [("/track/hands", [round(t, 4), len(hands)] + flat(hands)),
            ("/track/people", [round(t, 4), len(people)] + flat(people))]


def play(fr, host, port, loops=1):
    from pythonosc import udp_client
    c = udp_client.SimpleUDPClient(host, port)
    length = fr[-1][0] + 1.0 / FF.FPS
    wall0 = time.monotonic()
    for k in range(loops):
        for t, people, hands in fr:
            ahead = k * length + t - (time.monotonic() - wall0)
            if ahead > 0:
                time.sleep(ahead)
            for addr, args in osc_frame(t + k * length, people, hands):
                c.send_message(addr, args)


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)
    span = (13.0, 18.0)
    check("the crouch: none before or after it, all of it in the middle, eased (half at the ramps' middles)",
          crouch_drop(11.0, span) == 0.0 and crouch_drop(20.0, span) == 0.0 and crouch_drop(15.0, span) == CROUCH_M
          and abs(crouch_drop(12.5, span) - CROUCH_M / 2) < 1e-9 and abs(crouch_drop(18.5, span) - CROUCH_M / 2) < 1e-9
          and crouch_drop(15.0, None) == 0.0)
    msgs = osc_frame(1.5, [(4, 0.1, -2.0, 1.6, 0.9)], [(4, 0.3, -2.0, 1.9, 0.9)])
    check("a frame as people_track sends it: /track/hands, then /track/people: t, n, (id x y z conf) each",
          [a for a, _ in msgs] == ["/track/hands", "/track/people"]
          and msgs[1][1] == [1.5, 1, 4, 0.1, -2.0, 1.6, 0.9], msgs)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording", nargs="?")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9010, help="9010 Isaac's run_tracking, 9011 track_mode")
    ap.add_argument("--extrinsic", default=None)
    ap.add_argument("--crouch", default=None, metavar="T0-T1", help="lower heads and hands 0.5 m from T0 to T1 (s)")
    ap.add_argument("--loops", type=int, default=1)
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args(argv)
    if a.self_test:
        return self_test()
    if not a.recording:
        ap.error("which recording?")
    ext = a.extrinsic or os.path.join(os.path.dirname(os.path.abspath(a.recording)), "femto_extrinsic.json")
    span = tuple(float(x) for x in a.crouch.split("-")) if a.crouch else None
    fr = frames(a.recording, ext, span)
    print("%d frames, %.1f s, to %s:%d%s" % (len(fr), fr[-1][0], a.host, a.port,
                                             ", a crouch %g-%g s" % span if span else ""), flush=True)
    play(fr, a.host, a.port, a.loops)
    return 0


if __name__ == "__main__":
    sys.exit(main())
