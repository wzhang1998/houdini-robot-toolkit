"""A show as it played live with TouchDesigner, kept to render later: Isaac
records (run_show --record) the arm, the show's status and what TD sent back
-- the strip's 60 levels (Art-Net) and the canvas preview (canvas_link) --
and replay_render.py renders it at leisure: TD ran in real time, as in the
show, and the video need not.

    <dir>/capture.json    the show, the seed, when
    <dir>/frames.csv      t, state, clip, scan, scan_u, j1..j6, led0..led59, ceiling     (30 a second)
    <dir>/canvas.bin      records: t f64 | w u16 | h u16 | n u32 | zlib(RGB8, rows top first)

    w = Writer(dir, meta); w.frame(t, status, q, leds, ceiling); w.canvas(t, w, h, rgb); w.close()
    r = Reader(dir); r.frames; r.canvas_at(t)

    python scripts/td_capture.py --self-test
"""

import bisect
import csv
import json
import os
import struct
import sys
import zlib

LEDS = 60
HEAD = struct.Struct("<dHHI")


class Writer:
    def __init__(self, path, meta):
        os.makedirs(path, exist_ok=True)
        self.path = path
        with open(os.path.join(path, "capture.json"), "w") as f:
            json.dump(meta, f, indent=1)
        self.f = open(os.path.join(path, "frames.csv"), "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t", "state", "clip", "scan", "scan_u"] + ["j%d" % i for i in range(1, 7)]
                        + ["led%d" % i for i in range(LEDS)] + ["ceiling"])
        self.c = open(os.path.join(path, "canvas.bin"), "wb")
        self.frames = self.images = 0

    def frame(self, t, status, q, leds, ceiling=None):
        """One moment: the show's status (Runner.status), the joints (deg), the
        strip's levels (None: nothing from TD yet -> dark), the ceiling's level
        (TD ceiling_light; None: nothing from TD -> full, the room as lit)."""
        lv = list(leds or [])[:LEDS] + [0.0] * (LEDS - len(leds or []))
        self.w.writerow(["%.4f" % t, status["state"], status["clip"], "%.4f" % status["scan"],
                         "%.4f" % status["scan_u"]] + ["%.4f" % x for x in q] + ["%.4f" % x for x in lv]
                        + ["%.4f" % (1.0 if ceiling is None else ceiling)])
        self.frames += 1

    def canvas(self, t, w, h, rgb):
        z = zlib.compress(bytes(rgb), 6)
        self.c.write(HEAD.pack(t, w, h, len(z)) + z)
        self.images += 1

    def close(self):
        self.f.close()
        self.c.close()


class Reader:
    def __init__(self, path):
        self.path = path
        self.meta = json.load(open(os.path.join(path, "capture.json")))
        self.frames = []
        with open(os.path.join(path, "frames.csv"), newline="") as f:
            for row in csv.DictReader(f):
                self.frames.append({"t": float(row["t"]), "state": row["state"], "clip": row["clip"],
                                    "scan": float(row["scan"]), "scan_u": float(row["scan_u"]),
                                    "q": [float(row["j%d" % i]) for i in range(1, 7)],
                                    "leds": [float(row["led%d" % i]) for i in range(LEDS)],
                                    "ceiling": float(row.get("ceiling") or 1.0)})    # older captures: full
        self.times = [fr["t"] for fr in self.frames]
        self.canvas_t, self._canvas = [], []
        p = os.path.join(path, "canvas.bin")
        if os.path.exists(p):
            data = open(p, "rb").read()
            k = 0
            while k + HEAD.size <= len(data):
                t, w, h, n = HEAD.unpack_from(data, k)
                k += HEAD.size
                if k + n > len(data):
                    break                                        # a record cut short: the run was stopped
                self.canvas_t.append(t)
                self._canvas.append((w, h, data[k:k + n]))
                k += n

    def frame_at(self, t):
        """The frame at or just before t."""
        return self.frames[max(0, bisect.bisect_right(self.times, t) - 1)]

    def q_at(self, t):
        """The joints at t, straight between the recorded frames: the frames
        fall on the physics ticks nearest each 1/30 s, not on it, so taking
        the one before t repeats one and skips the next (a stutter)."""
        i = bisect.bisect_right(self.times, t)
        if i <= 0:
            return list(self.frames[0]["q"])
        if i >= len(self.frames):
            return list(self.frames[-1]["q"])
        a, b = self.frames[i - 1], self.frames[i]
        f = (t - a["t"]) / (b["t"] - a["t"]) if b["t"] > a["t"] else 0.0
        return [x + (y - x) * f for x, y in zip(a["q"], b["q"])]

    def canvas_index(self, t):
        """The index of the newest canvas image at or before t; None before the first."""
        i = bisect.bisect_right(self.canvas_t, t) - 1
        return None if i < 0 else i

    def canvas(self, i):
        """(w, h, rgb) of canvas image i."""
        w, h, z = self._canvas[i]
        return w, h, zlib.decompress(z)

    def canvas_at(self, t):
        """(w, h, rgb) of the newest canvas image at or before t; None before the first."""
        i = self.canvas_index(t)
        return None if i is None else self.canvas(i)

    def duration(self):
        return self.times[-1] if self.times else 0.0


def self_test():
    import tempfile
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    d = tempfile.mkdtemp()
    w = Writer(d, {"config": "shows/party.json", "seed": 1})
    st = {"state": "IDLE", "clip": "greet_01_wave", "scan": -1.0, "scan_u": -1.0}
    w.frame(0.0, st, [0, -90, 90, -90, -90, 0], None)
    w.frame(1 / 30.0, dict(st, state="SCAN", clip="scan", scan=0.5, scan_u=0.5), [1, 2, 3, 4, 5, 6], [0.5] * 60,
            ceiling=0.25)
    img = bytes([200, 100, 50]) * (4 * 3)
    w.canvas(0.02, 4, 3, img)
    w.canvas(0.5, 4, 3, bytes(36))
    w.close()
    r = Reader(d)
    check("frames come back: status, joints, levels (none from TD yet: dark)",
          len(r.frames) == 2 and r.frames[0]["leds"] == [0.0] * 60 and r.frames[1]["state"] == "SCAN"
          and r.frames[1]["q"] == [1, 2, 3, 4, 5, 6] and r.frames[1]["leds"][59] == 0.5 and r.meta["seed"] == 1)
    check("the frame at a time is the one at or just before it", r.frame_at(0.02)["state"] == "IDLE"
          and r.frame_at(5.0)["state"] == "SCAN")
    check("the canvas at a time is the newest image by then; none before the first",
          r.canvas_at(0.01) is None and r.canvas_at(0.3) == (4, 3, img) and r.canvas_at(9.0)[2] == bytes(36))
    check("... by index too (the replay redraws only when it changes)",
          r.canvas_index(0.01) is None and r.canvas_index(0.3) == 0 and r.canvas_index(0.6) == 1
          and r.canvas(1) == (4, 3, bytes(36)))
    q = r.q_at(0.5 / 30.0)
    check("the joints between two frames: straight between them; the ends held",
          abs(q[0] - 0.5) < 0.01 and abs(q[1] + 44.0) < 0.1 and r.q_at(-1.0)[0] == 0 and r.q_at(9.0)[5] == 6, q)
    with open(os.path.join(d, "canvas.bin"), "ab") as f:
        f.write(HEAD.pack(1.0, 4, 3, 999) + b"xx")                   # a run stopped mid-record
    check("a record cut short at the end is left out", len(Reader(d).canvas_t) == 2)
    check("the ceiling's level kept (none from TD: full, the room as lit)",
          r.frames[0]["ceiling"] == 1.0 and r.frames[1]["ceiling"] == 0.25, [f["ceiling"] for f in r.frames])
    old = os.path.join(d, "frames.csv")
    rows = open(old).read().splitlines()
    cut = lambda line: ",".join(line.split(",")[:-1])  # noqa: E731
    open(old, "w").write("\n".join(cut(x) for x in rows) + "\n")
    check("... a capture from before it: full", Reader(d).frames[1]["ceiling"] == 1.0)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    print(__doc__)
