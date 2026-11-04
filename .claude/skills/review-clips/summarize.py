"""Check the reviews and write reviews/SUMMARY.md (fix and drop first).

    python .claude/skills/review-clips/summarize.py              # check every reviews/*.json, write SUMMARY.md
    python .claude/skills/review-clips/summarize.py --check      # only check
    python .claude/skills/review-clips/summarize.py --notes FILE # cross-clip notes (markdown) put in the summary
    python .claude/skills/review-clips/summarize.py --self-test

A review is refused (exit 1, nothing written) when its verdict is not
keep / fix / drop, a fix or drop has no suggestion (a drop may instead name
its replacement in `replace_with`), a field the rubric asks for is missing,
the expressiveness score is not 1-5 with evidence, or a suggestion reads as joint angles (docs/review_rubric.md, 5: generator
parameters only -- no "J5 to 30 deg", no pose in degrees, no hub q / near).
Standard library only.
"""

import argparse
import glob
import json
import os
import re
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
REVIEWS = os.path.join(ROOT, "reviews")
VERDICTS = ("fix", "drop", "keep")
KINDS = ("wrist_flip", "elbow_flip", "twitch", "too_hard", "strip_orientation", "mechanical", "crowding")
REQUIRED = ("id", "verdict", "confidence", "readability", "awkward", "audience", "expression", "suggestions",
            "needs_live_check", "evidence", "reviewed_from")
LACKS = ("anticipation", "overshoot", "follow_through", "phrasing", "dynamics", "contrast", "timing_variety",
         "body_involvement")
# a suggestion that sets joints: "J5 30 deg", "j3=-40", "joint 4 to 90", "[-60, -90, 90, ...]", hub q / near
JOINT_PATTERNS = (
    re.compile(r"\bj(?:oint)?\s*[1-6]\b[^.;,]{0,25}?(?:-?\d+(?:\.\d+)?\s*(?:deg|°|degrees)|[=:]\s*-?\d)", re.I),
    re.compile(r"\[\s*-?\d+(?:\.\d+)?\s*(?:,\s*-?\d+(?:\.\d+)?\s*){5}\]"),
    re.compile(r"\bhubs\.\w+\.(?:q|near)\b"),
    re.compile(r"\bjoint angles?\b\s*(?:to|=|:)", re.I),
)


def joint_like(text):
    return any(p.search(text or "") for p in JOINT_PATTERNS)


def problems(r):
    """[what is wrong with review r] (empty: fine)."""
    out = ["missing %s" % k for k in REQUIRED if k not in r]
    if out:
        return out
    if r["verdict"] not in VERDICTS:
        out.append("verdict %r" % r["verdict"])
    if r["confidence"] not in ("high", "medium", "low"):
        out.append("confidence %r" % r["confidence"])
    if r["verdict"] == "fix" and not r["suggestions"]:
        out.append("a fix without a suggestion")
    if r["verdict"] == "drop" and not (r["suggestions"] or r.get("replace_with")):
        out.append("a drop without a suggestion or replace_with")
    x = r["expression"]
    if not isinstance(x, dict) or x.get("score") not in (1, 2, 3, 4, 5):
        out.append("expression score must be 1-5")
    else:
        if not x.get("evidence"):
            out.append("expression without evidence")
        bad = [k for k in x.get("lacks", []) if k not in LACKS]
        if bad:
            out.append("expression lacks %s (not in %s)" % (bad, LACKS))
    if r.get("verdict_before_expression") not in (None,) + VERDICTS:
        out.append("verdict_before_expression %r" % r.get("verdict_before_expression"))
    for a in r["awkward"]:
        if a.get("kind") not in KINDS:
            out.append("awkward kind %r" % a.get("kind"))
    for s in r["suggestions"]:
        for k in ("target", "change", "why"):
            if not s.get(k):
                out.append("suggestion without %s" % k)
        if joint_like(s.get("target", "") + " " + s.get("change", "")):
            out.append("suggestion sets joints, not a generator parameter: %r" % s.get("change"))
    return out


def load(directory=REVIEWS):
    rows = []
    for p in sorted(glob.glob(os.path.join(directory, "*.json"))):
        r = json.load(open(p))
        if os.path.splitext(os.path.basename(p))[0] != r.get("id"):
            rows.append((p, r, ["file name is not its id %r" % r.get("id")]))
            continue
        rows.append((p, r, problems(r)))
    return rows


def _hub(cid):
    return cid.split("_")[0]


def _cell(text):
    return str(text).replace("|", "/").replace("\n", " ")


def summary(reviews, notes=None):
    """SUMMARY.md text: counts, then fix and drop clips with their suggestions, keep last."""
    by = {v: [r for r in reviews if r["verdict"] == v] for v in VERDICTS}
    hubs = sorted({_hub(r["id"]) for r in reviews})
    out = ["# Clip review summary", "",
           "Reviewed from static sheets (`scripts/review_render.py`) against `docs/review_rubric.md`; "
           "per clip: `reviews/<id>.json`, the sheet's numbers in `reviews/sheets/<id>.labels.json` "
           "(the PNGs are not in git: `scripts/review_render.py` renders them again).", "",
           "| hub | keep | fix | drop |", "|---|---|---|---|"]
    for h in hubs:
        out.append("| %s | %s |" % (h, " | ".join(str(sum(1 for r in by[v] if _hub(r["id"]) == h))
                                                   for v in ("keep", "fix", "drop"))))
    out.append("| **all** | %d | %d | %d |" % (len(by["keep"]), len(by["fix"]), len(by["drop"])))
    out.append("")
    sc = lambda r: r["expression"]["score"]  # noqa: E731
    out += ["Expressiveness (1 inert .. 5 alive, rubric 4):", "",
            "| hub | mean | 1 | 2 | 3 | 4 | 5 |", "|---|---|---|---|---|---|---|"]
    for h in hubs + [None]:
        rs = [r for r in reviews if h is None or _hub(r["id"]) == h]
        out.append("| %s | %.1f | %s |" % (h or "**all**", sum(sc(r) for r in rs) / max(1, len(rs)),
                                           " | ".join(str(sum(1 for r in rs if sc(r) == k)) for k in range(1, 6))))
    lacks = {}
    for r in reviews:
        for k in r["expression"].get("lacks", []):
            lacks[k] = lacks.get(k, 0) + 1
    if lacks:
        out += ["", "Most often missing: " + ", ".join("%s %d" % (k, n) for k, n in
                                                       sorted(lacks.items(), key=lambda kv: -kv[1]))]
    changed = [r for r in reviews if r.get("verdict_before_expression") not in (None, r["verdict"])]
    if changed:
        out += ["", "Verdicts the expressiveness pass changed: " + ", ".join(
            "%s %s -> %s" % (r["id"], r["verdict_before_expression"], r["verdict"]) for r in sorted(changed, key=lambda r: r["id"]))]
    out.append("")
    if notes:
        out += ["## Across the library", "", notes.strip(), ""]
    for v, title in (("fix", "Fix"), ("drop", "Drop")):
        out += ["## %s (%d)" % (title, len(by[v])), ""]
        if not by[v]:
            out += ["None.", ""]
            continue
        for r in sorted(by[v], key=lambda r: ({"low": 2, "medium": 1, "high": 0}[r["confidence"]], r["id"])):
            kinds = ", ".join(sorted({a["kind"] for a in r["awkward"]})) or "-"
            out.append("### %s  (%s confidence; expression %d; %s)" % (r["id"], r["confidence"],
                                                                     r["expression"]["score"], kinds))
            out.append("")
            out.append("- expression %d%s: %s" % (r["expression"]["score"],
                                                 (" (lacks " + ", ".join(r["expression"].get("lacks", [])) + ")")
                                                 if r["expression"].get("lacks") else "",
                                                 _cell(r["expression"].get("note") or "; ".join(r["expression"]["evidence"]))))
            out.append("- reads as: %s%s" % (_cell(r["readability"].get("reads_as")),
                                             (": " + _cell(r["readability"].get("note"))) if r["readability"].get("note") else ""))
            for a in r["awkward"]:
                out.append("- %s at %s: %s" % (a["kind"], _cell(a.get("where", "-")), _cell(a.get("evidence", ""))))
            for s in r["suggestions"]:
                out.append("- **change** `%s`: %s (%s; scope %s)" % (s["target"], _cell(s["change"]), _cell(s["why"]),
                                                                    s.get("scope", "-")))
            if r.get("replace_with"):
                out.append("- replace with: %s" % _cell(r["replace_with"]))
            if r["needs_live_check"]:
                out.append("- live check: %s" % "; ".join(_cell(x) for x in r["needs_live_check"]))
            out.append("- sheet: `%s`" % r["evidence"].get("image", ""))
            out.append("")
    out += ["## Keep (%d)" % len(by["keep"]), "", "| clip | expr. | reads as | notes | live check |",
            "|---|---|---|---|---|"]
    for r in sorted(by["keep"], key=lambda r: r["id"]):
        out.append("| %s | %d | %s | %s | %s |" % (r["id"], r["expression"]["score"], _cell(r["readability"].get("reads_as")),
                                              _cell(r["readability"].get("note") or "") + ("; " if r["awkward"] else "")
                                              + "; ".join(a["kind"] for a in r["awkward"]),
                                              _cell("; ".join(r["needs_live_check"]))))
    out.append("")
    return "\n".join(out)


def self_test():
    ok = True
    good = {"id": "greet_01_wave", "verdict": "fix", "confidence": "medium", "readability": {"reads_as": "wave"},
            "awkward": [{"kind": "twitch", "where": "t 2 s", "evidence": "J4 2 twitches"}],
            "audience": {}, "needs_live_check": [], "evidence": {"image": "x.png"}, "reviewed_from": "static sheet",
            "expression": {"score": 3, "lacks": ["anticipation"], "evidence": ["A 0 of 3 big moves"]},
            "suggestions": [{"target": "shows/party.json library.intensity", "change": "[0.2, 1.0] -> [0.2, 0.8]",
                             "why": "the lively draws twitch", "scope": "library"}]}
    ok &= problems(good) == []
    for bad_change in ("J5 to 30 deg", "set j3=-40", "q [-60, -90, 90, -90, -90, -15]", "joint 4 to 90 degrees"):
        b = json.loads(json.dumps(good))
        b["suggestions"][0]["change"] = bad_change
        ok &= any("sets joints" in x for x in problems(b))
    b = json.loads(json.dumps(good))
    b["suggestions"][0]["target"] = "shows/party.json hubs.greet.near"
    ok &= any("sets joints" in x for x in problems(b))
    for fine in ("GAZE_CONE_DEG 50 -> 35", "SPIN_W_DEG_S (100, 130) -> (80, 110)", "bpm [70, 140] -> [70, 110]",
                 "drop 'mop' from hubs.greet.families", "zones.greet.size [1.3, 0.5, 0.7] -> [1.0, 0.5, 0.7]"):
        b = json.loads(json.dumps(good))
        b["suggestions"][0]["change"] = fine
        ok &= problems(b) == []
    b = dict(good, suggestions=[])
    ok &= "a fix without a suggestion" in problems(b)
    ok &= "expression score must be 1-5" in problems(dict(good, expression={"score": 7, "evidence": ["x"]}))
    ok &= any("lacks" in x for x in problems(dict(good, expression={"score": 2, "evidence": ["x"], "lacks": ["soul"]})))
    ok &= "expression without evidence" in problems(dict(good, expression={"score": 2, "evidence": []}))
    ok &= problems(dict(good, verdict="drop", suggestions=[], replace_with="greet_02_nod")) == []
    text = summary([good, dict(good, id="low_00_look", verdict="keep", suggestions=[], awkward=[])])
    ok &= text.index("## Fix") < text.index("## Drop") < text.index("## Keep") and "greet_01_wave" in text
    ok &= "Most often missing: anticipation 2" in text and "expression 3" in text
    print("self-test %s" % ("passed" if ok else "FAILED"))
    return 0 if ok else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", default=REVIEWS)
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--notes", help="a markdown file of cross-clip notes for the summary")
    a = ap.parse_args(argv)
    rows = load(a.dir)
    bad = [(p, why) for p, _, why in rows if why]
    for p, why in bad:
        print("%s: %s" % (os.path.relpath(p, ROOT), "; ".join(why)))
    print("%d reviews, %d with problems" % (len(rows), len(bad)))
    if bad:
        return 1
    if a.check:
        return 0
    notes = open(a.notes, encoding="utf-8").read() if a.notes else None
    with open(os.path.join(a.dir, "SUMMARY.md"), "w", encoding="utf-8") as f:
        f.write(summary([r for _, r, _ in rows], notes))
    print("wrote %s" % os.path.relpath(os.path.join(a.dir, "SUMMARY.md"), ROOT))
    return 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    sys.exit(main())
