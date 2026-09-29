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


def clip_card(name, kind, hub, duration, labels, index=None, total=None, sim=None):
    """A library clip's metadata as short lines, for the review's corner (as
    render_clip_review's tile header): name, length, place in the library;
    hub, family / intent, measured action; tempo, energy (or a showpiece's
    size and speed, the scan's pass); motion stats and the room's clearance;
    sim {"track_deg", "contacts"} when the simulation has played it."""
    lab = labels or {}
    head = "%s   %.2f s" % (name, duration)
    if index is not None and total:
        head += "   %d/%d" % (index, total)
    lines = [head]
    parts = []
    if kind == "scan":
        lines.append("scan %s   %.2f m/s   %.2f m" % (lab.get("direction", "?"), lab.get("speed_mps", 0.0),
                                                   lab.get("exposed_m", 0.0)))
        on = lab.get("led_on_s") or [0.0, 0.0]
        lines.append("LEDs %.2f-%.2f s   gap %d mm   roll %d" % (on[0], on[1], round(1000 * lab.get("led_gap_m", 0.0)),
                                                              round(lab.get("roll_deg", 0.0))))
    else:
        intent = "-".join(lab.get("intent") or [])
        second = ["hub %s" % hub]
        if lab.get("family"):
            second.append("family %s" % lab["family"])
        if intent and intent not in (lab.get("family"), "showpiece"):
            second.append("intent %s" % intent)
        if lab.get("action"):
            second.append("action %s" % lab["action"])
        if intent == "showpiece":
            second.append("showpiece")
        lines.append("   ".join(second))
        third = []
        if lab.get("width_m") is not None:
            third.append("%.2f x %.2f m" % (lab["width_m"], lab.get("height_m", 0.0)))
            third.append("%.3f m/s" % lab.get("speed_mps", 0.0))
        if lab.get("bpm"):
            played = lab["bpm"] / float((lab.get("params") or {}).get("slowed") or 1.0)
            third.append("bpm %d" % lab["bpm"] + (" (%d played)" % round(played) if round(played) != lab["bpm"] else ""))
        if lab.get("energy") is not None:
            third.append("energy %.2f" % lab["energy"])
        if lab.get("intensity") is not None:
            third.append("int %.2f" % lab["intensity"])
        lines.append("   ".join(third))
        st = lab.get("stats") or {}
        if st.get("v_peak") is not None:
            parts.append("v %.2f m/s" % st["v_peak"])
        if st.get("z_min") is not None:
            parts.append("z %.2f-%.2f m" % (st["z_min"], st["z_max"]))
    if lab.get("clearance_m") is not None:
        parts.append("room %d mm" % round(1000 * lab["clearance_m"]))
    if lab.get("wrist_share") is not None:
        parts.append("wrist %d%%" % round(100 * lab["wrist_share"]))
    lines.append("   ".join(parts))
    if sim:
        lines.append("sim: tracking %.2f deg   contacts %d" % (sim["track_deg"], sim["contacts"]))
    return lines


def block(lines):
    """Lines as one corner text: each padded with ASS hard spaces (\\h) to the
    longest, so the bottom-right corner's box is one rectangle and the lines
    start in one column (Consolas: every character as wide)."""
    w = max(len(x) for x in lines)
    return "\n".join(x + "\\h" * (w - len(x)) for x in lines)


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
          and label("SCAN", "scan", 0.425) == "SCAN   scan  42%" and label("TO_SCAN", "to_scan") == "TO_SCAN"
          and label("SCAN", "scan", -1.0) == "SCAN",
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

    gesture = {"action": "slash", "intent": ["wipe"], "family": "wipe", "bpm": 90, "intensity": 0.641,
               "clearance_m": 0.0589, "wrist_share": 0.78, "params": {"slowed": 1.256},
               "stats": {"z_min": 0.641, "z_max": 1.108, "v_peak": 1.841}, "energy": 0.638}
    card = clip_card("greet_16_wipe", "idle", "greet", 11.92, gesture, 17, 73)
    check("a clip's card: name, length, place in the library; hub, family, action",
          card[0] == "greet_16_wipe   11.92 s   17/73" and card[1] == "hub greet   family wipe   action slash", card)
    check("... the tempo made and played (slowed), energy, intensity",
          card[2] == "bpm 90 (72 played)   energy 0.64   int 0.64", card)
    check("... peak TCP speed, heights, the room's clearance, the wrist's share",
          card[3] == "v 1.84 m/s   z 0.64-1.11 m   room 59 mm   wrist 78%", card)
    dance = {"action": "wring", "intent": ["press", "slash"], "bpm": 104, "clearance_m": 0.0572, "energy": 0.942,
             "stats": {"z_min": 1.096, "z_max": 1.426, "v_peak": 1.841}}
    card = clip_card("rest_01_press-slash", "idle", "rest", 12.0, dance)
    check("a dance: its intended actions bar by bar and the measured one; no slowing, no wrist share",
          card[0] == "rest_01_press-slash   12.00 s" and card[1] == "hub rest   intent press-slash   action wring"
          and card[2] == "bpm 104   energy 0.94" and card[3] == "v 1.84 m/s   z 1.10-1.43 m   room 57 mm", card)
    wipe = {"family": "wipe_rows", "intent": ["showpiece"], "width_m": 1.45, "height_m": 0.655,
            "speed_mps": 0.153, "clearance_m": 0.0531, "energy": 0.535}
    card = clip_card("low_wipe_rows", "idle", "low", 21.3, wipe)
    check("a showpiece: family, its size and speed over the wall",
          card[1] == "hub low   family wipe_rows   showpiece" and card[2] == "1.45 x 0.66 m   0.153 m/s   energy 0.54"
          and card[3] == "room 53 mm", card)
    scan = {"speed_mps": 0.2, "exposed_m": 1.0, "direction": "top_to_bottom", "roll_deg": 90.0, "led_gap_m": 0.06,
            "led_on_s": [0.4927, 5.6197], "clearance_m": 0.0473}
    card = clip_card("scan", "scan", "scan_start", 6.11, scan, 73, 73, sim={"track_deg": 0.412, "contacts": 0})
    check("the scan: direction, speed, the paper exposed, the LEDs' window; the simulation's tracking and contacts",
          card[1] == "scan top_to_bottom   0.20 m/s   1.00 m" and card[2] == "LEDs 0.49-5.62 s   gap 60 mm   roll 90"
          and card[3] == "room 47 mm" and card[4] == "sim: tracking 0.41 deg   contacts 0", card)
    check("every card line fits 52 characters (the Houdini review's tiles) and has no ASS braces",
          all(len(x) <= 52 and "{" not in x for c in (clip_card("greet_19_spin_sweep", "idle", "greet", 11.46, gesture, 99, 99,
                                                                sim={"track_deg": 12.345, "contacts": 3}),
                                                      clip_card("scan", "scan", "scan_start", 6.11, scan)) for x in c))
    b = block(["ab", "abcd"])
    check("a card as one corner block: lines padded with ASS hard spaces to the longest, so the right-aligned "
          "corner reads left-aligned", b == "ab\\h\\h\nabcd"
          and ass([(0.0, b)], 1280, 720, 1.0).rstrip().endswith("ab\\h\\h\\Nabcd"), b)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
