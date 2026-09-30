"""The tracking layer's input live over OSC: /track/people and /track/hands
from TouchDesigner's people_track (TD-ROBOT-UVSCAN; the Femto Mega's body
tracking, for now simulated from real people) -- the newest frame, never
blocking, and the tracker's ids kept on a fixed number of slots (Isaac's
head prims, a test's people).

    rx = TrackIn(9010)
    got = rx.poll()               # (people, hands) of a frame new since the last poll, else None
    slots = Slots(6)
    where = slots.update([pid for pid, *_ in people], now)     # {pid: slot}

    /track/hands   t n  (id x y z conf) * n    each person's higher wrist (sent first)
    /track/people  t n  (id x y z conf) * n    the heads, robot base frame, metres

    python scripts/track_osc.py              self-test (localhost, port 9019)

Needs python-osc (Isaac's Python has it).
"""

import sys
import time

from udp_latest import LatestReceiver


def people_of(args):
    """[(pid, x, y, z, conf)] of a /track/people (or /track/hands) message's arguments."""
    n = int(args[1])
    return [(int(args[2 + 5 * k]),) + tuple(float(v) for v in args[3 + 5 * k:7 + 5 * k]) for k in range(n)]


class TrackIn:
    """The newest /track/people frame on a UDP port, with the newest /track/hands before it."""

    def __init__(self, port, host="127.0.0.1"):
        from pythonosc.osc_message import OscMessage
        self._msg, self._hands, self.seen = OscMessage, [], 0
        self.rx = LatestReceiver(port, self._parse, host)

    def _parse(self, packet):
        try:
            m = self._msg(packet)
        except Exception:                                  # not OSC, or not ours: skipped
            return None
        if m.address == "/track/hands":
            self._hands = people_of(m.params)
            return None
        if m.address == "/track/people":
            return people_of(m.params), list(self._hands)
        return None

    def poll(self):
        """(people, hands) of a frame new since the last poll, else None."""
        v = self.rx.poll()
        if self.rx.packets == self.seen:
            return None
        self.seen = self.rx.packets
        return v


class Slots:
    """The tracker's ids on n slots: an id keeps its slot while it is seen
    (gone forget_s: the slot is free again); a new id takes the free slot
    freed longest ago; none free: it is left out."""

    def __init__(self, n, forget_s=0.5):
        self.n, self.forget_s = n, forget_s
        self.pid = [None] * n
        self.last = [-1e9] * n

    def update(self, pids, now):
        """{pid: slot} of the ids seen now."""
        for s in range(self.n):
            if self.pid[s] is not None and now - self.last[s] > self.forget_s:
                self.pid[s] = None
        out = {}
        for pid in pids:
            if pid in self.pid:
                s = self.pid.index(pid)
            else:
                free = [s for s in range(self.n) if self.pid[s] is None]
                if not free:
                    continue
                s = min(free, key=lambda k: self.last[k])
                self.pid[s] = pid
            self.last[s] = now
            out[pid] = s
        return out


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    from pythonosc import udp_client
    rx = TrackIn(9019)
    tx = udp_client.SimpleUDPClient("127.0.0.1", 9019)
    check("nothing yet: None", rx.poll() is None)
    tx.send_message("/track/hands", [1.0, 1, 7, 0.1, -1.8, 1.9, 0.9])
    tx.send_message("/track/people", [1.0, 2, 7, 0.0, -1.8, 1.6, 0.9, 3, 0.5, -2.0, 1.7, 0.8])
    time.sleep(0.05)
    got = rx.poll()
    check("a frame: the people and the hands sent before them", got is not None and [p[0] for p in got[0]] == [7, 3]
          and got[1] and got[1][0][0] == 7 and abs(got[1][0][3] - 1.9) < 1e-5, got)
    check("... once: the next poll None", rx.poll() is None)
    for k in range(3):
        tx.send_message("/track/people", [2.0 + k, 1, 7, 0.1 * k, -1.8, 1.6, 0.9])
    tx.send_message("/robot/state", ["IDLE"])
    time.sleep(0.05)
    got = rx.poll()
    check("several waiting: the newest (other addresses skipped)", got is not None and abs(got[0][0][1] - 0.2) < 1e-5,
          got)
    tx.send_message("/track/people", [5.0, 0])
    time.sleep(0.05)
    got = rx.poll()
    check("nobody: an empty frame", got is not None and got[0] == [], got)

    sl = Slots(2)
    a = sl.update([7, 3], 0.0)
    b = sl.update([3, 7], 0.1)
    check("an id keeps its slot", a == b and sorted(a.values()) == [0, 1], (a, b))
    c = sl.update([7, 3, 9], 0.2)
    check("no slot free: the new id left out", 9 not in c, c)
    sl.update([7], 0.5)
    d = sl.update([7, 9], 0.8)
    check("an id gone past forget_s frees its slot for a new one", d.get(9) == a[3] and d[7] == a[7], d)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
