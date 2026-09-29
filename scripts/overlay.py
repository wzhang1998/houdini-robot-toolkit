"""Text burnt into a recording: what the show was doing, frame by frame.

A recording (Isaac Sim: run_show --video, later a real run's replay) writes
its timeline -- (time s, text) whenever the text changes -- and this turns
it into an ASS subtitle file that ffmpeg burns in, bottom right:

    events = compress([(t, label(state, clip, scan)) for ...])
    open("overlay.ass", "w").write(ass(events, 1280, 720, end_s))
    ffmpeg -i frames/f_%05d.png -vf ass=overlay.ass ...

    python scripts/overlay.py --self-test
"""

import re
import sys

STYLE = "Consolas"


def label(state, clip, scan=-1.0, speed=None):
    """One line for the corner: the state, the clip (unless it is the state's
    own, as the scan), the scan's progress."""
    text = state if clip.lower() == state.lower() else "%s   %s" % (state, clip)
    if scan is not None and 0.0 <= scan <= 1.0:
        text += "   scan %3.0f%%" % (100.0 * scan)
    if speed is not None:
        text += "   x%.2f" % speed
    return text


def _base(text):
    """The text without the scan's percent (what a percent tick leaves the same)."""
    return re.sub(r"\s+scan\s+\d+%$", "", text)


def compress(rows, min_step_s=0.0):
    """[(t, text)] -> the moments the text changes: [(t, text)]; a change
    sooner than min_step_s after the last kept one waits (a scan's percent
    ticking at the physics rate would make thousands of events)."""
    out = []
    for t, text in rows:
        if out and text == out[-1][1]:
            continue
        if out and t - out[-1][0] < min_step_s and _base(text) == _base(out[-1][1]):
            continue
        out.append((t, text))
    return out


def _clock(t):
    h, rest = divmod(max(0.0, t), 3600.0)
    m, s = divmod(rest, 60.0)
    return "%d:%02d:%05.2f" % (h, m, s)


def ass(events, width, height, end_s, size=None):
    """An ASS subtitle file: each event's text from its time to the next's
    (the last to end_s), bottom right, white on a dark box."""
    size = size or max(16, height // 30)
    head = ["[Script Info]", "ScriptType: v4.00+", "PlayResX: %d" % width, "PlayResY: %d" % height, "",
            "[V4+ Styles]",
            "Format: Name, Fontname, Fontsize, PrimaryColour, OutlineColour, BackColour, Bold, BorderStyle, "
            "Outline, Shadow, Alignment, MarginL, MarginR, MarginV",
            "Style: Corner,%s,%d,&H00FFFFFF,&H00000000,&H90000000,0,3,6,0,3,20,20,18" % (STYLE, size), "",
            "[Events]", "Format: Layer, Start, End, Style, Text"]
    lines = []
    for k, (t, text) in enumerate(events):
        t1 = events[k + 1][0] if k + 1 < len(events) else end_s
        if t1 > t:
            lines.append("Dialogue: 0,%s,%s,Corner,%s" % (_clock(t), _clock(t1), text.replace("\n", "\\N")))
    return "\n".join(head + lines) + "\n"


def self_test():
    fails = []

    def check(name, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", name, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(name)

    check("a line: state and clip; the scan's percent only while it runs",
          label("IDLE", "rest_02_glide") == "IDLE   rest_02_glide"
          and label("SCAN", "scan", 0.425) == "SCAN   scan  42%" and label("TO_SCAN", "to_scan") == "TO_SCAN" and label("SCAN", "scan", -1.0) == "SCAN",
          (label("SCAN", "scan", 0.425), label("TO_SCAN", "to_scan")))
    rows = [(0.0, label("IDLE", "a")), (0.01, label("IDLE", "a")), (1.0, label("SCAN", "scan", 0.0)),
            (1.1, label("SCAN", "scan", 0.01)), (1.6, label("SCAN", "scan", 0.06)), (2.0, label("IDLE", "b"))]
    ev = compress(rows, 0.5)
    check("the moments the text changes, the scan's percent at most every 0.5 s, a new clip at once",
          [t for t, _ in ev] == [0.0, 1.0, 1.6, 2.0] and _base(label("SCAN", "scan", 0.07)) == "SCAN", ev)
    doc = ass(ev, 1280, 720, 3.0)
    dl = [x for x in doc.splitlines() if x.startswith("Dialogue")]
    check("an ASS file: bottom right (alignment 3), one line per event, each to the next, the last to the end",
          ",3," in doc.split("Style: Corner")[1].splitlines()[0] and len(dl) == 4
          and dl[0] == "Dialogue: 0,0:00:00.00,0:00:01.00,Corner,IDLE   a"
          and dl[-1].startswith("Dialogue: 0,0:00:02.00,0:00:03.00,"), dl)
    check("times past a minute", _clock(75.5) == "0:01:15.50", _clock(75.5))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
