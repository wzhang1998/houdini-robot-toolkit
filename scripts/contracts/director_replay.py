"""Two Directors on the same show and the same room, compared from their
decision logs. No arm, no Isaac, no TouchDesigner: the Runner on a dry
clock (show.dry_run's), the room from a Femto recording (RecordedPerception,
track_replay's frames), a decision log each. docs/architecture/decision_log.md.

Closed loop -- each Director drives its own run; what it chose changes what
it is asked next (the hub, the recent clips), so the runs part ways:

    python scripts/contracts/director_replay.py shows/party.json --recording geo/tracking/femto_rec_X.csv \
        --a baseline --b attentive --minutes 10 --out logs/decisions

Open loop -- B is asked every question A was asked, in A's log; agreement
and validity, but not what B's choices would have led to:

    python scripts/contracts/director_replay.py --open-loop logs/decisions/a_baseline.jsonl --b attentive

Without --recording the room is empty (summarize(None)); --synthetic uses
recorded_perception.write_synthetic's walk-by. Directors by name: baseline
(BaselineDirector), attentive (the example below), or module:Class.

    python scripts/contracts/director_replay.py --self-test
"""

import argparse
import importlib
import json
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
ROOT = os.path.dirname(SCRIPTS)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import show as S  # noqa: E402
from contracts import decision_log as DL  # noqa: E402
from contracts import director as D  # noqa: E402
from contracts import perception as PC  # noqa: E402
from contracts.baseline_director import BaselineDirector  # noqa: E402


class AttentiveDirector:
    """AN EXAMPLE, not a show Director: someone with a hand up and the arm not at greet -> TRIGGER
    greet (once until they are gone); somebody near at greet -> PLAY_LIKE a look / wave; else DEFER.
    It exists to show a Director that reads the room going through the same contract, log and
    comparison as the baseline."""
    name = "example_attentive"
    version = "0"

    def __init__(self, near_m=3.0):
        self.near_m, self.called = near_m, False

    def decide(self, inp):
        sc, show = inp.scene, inp.show
        if sc.stale or sc.n_people == 0:
            self.called = False
            return D.Intent.defer("nobody")
        if sc.hands_up and show.hub != "greet" and not self.called and "greet" in inp.library.triggers:
            self.called = True
            return D.Intent(D.IntentKind.TRIGGER, trigger="greet", reason="a hand up")
        if show.hub == "greet" and sc.nearest and sc.nearest["dist_m"] < self.near_m:
            fams = {e.family for e in inp.library.at(show.hub)}
            for f in ("wave", "look", "nod"):
                if f in fams:
                    return D.Intent(D.IntentKind.PLAY_LIKE, family=f, reason="someone %.1f m away"
                                    % sc.nearest["dist_m"])
        return D.Intent.defer("nothing to answer")


def make_director(spec, graph, seed):
    if spec == "baseline":
        return BaselineDirector.from_graph(graph, seed)
    if spec == "attentive":
        return AttentiveDirector()
    mod, _, cls = spec.partition(":")
    return getattr(importlib.import_module(mod), cls)()


def library_from_dict(d):
    lib = D.LibraryIndex([D.ClipEntry(**dict(c, tags=tuple(c.get("tags") or ()))) for c in d["clips"]],
                         list(d["triggers"]), list(d["hubs"]))
    lib.digest = d["digest"]
    return lib


def closed_loop(graph, director, perception=None, minutes=10.0, dt=0.05, seed=1, trigger_every=None,
                triggers=("scan",), budget_s=D.DEFAULT_BUDGET_S, log_path=None, meta=None):
    """One run of the show on a dry clock with this Director. Returns (DecisionLog, Runner, DirectorSelector)."""
    log = DL.DecisionLog(log_path)
    lib = D.LibraryIndex.from_graph(graph)
    log.start(lib, dict(meta or {}, seed=seed, minutes=minutes, dt=dt, budget_s=budget_s,
                        director={"name": director.name, "version": director.version},
                        perception=(perception.info().__dict__ if perception else None)))
    scene_fn = (lambda clock: PC.summarize(perception.poll(clock), clock)) if perception else None
    r, sel = D.runner_with(graph, director, seed=seed, budget_s=budget_s, scene_fn=scene_fn, log=log)
    rng = random.Random(seed)
    next_trig = trigger_every * (0.5 + rng.random()) if trigger_every else float("inf")
    t, k = 0.0, 0
    while t < minutes * 60.0:
        t += dt
        if t >= next_trig:
            r.trigger(triggers[k % len(triggers)])
            k += 1
            next_trig = t + trigger_every * (0.5 + rng.random())
        r.step(dt)
    log.end({"history": r.history, "counts": sel.counts,
             "metrics": DL.metrics(log.records[1:], log.records[0]["library"])})
    sel.close()
    return log, r, sel


def open_loop(log_path, director, budget_s=D.DEFAULT_BUDGET_S):
    """B asked A's questions: [{seq, a_clip, b_intent, b_fallback, agree}] and a summary."""
    header, decisions, _ = DL.read(log_path)
    lib = library_from_dict(header["library"])
    g = D.GuardedDirector(director, budget_s)
    rows = []
    for rec in decisions:
        inp = D.DirectorInput.from_dict(rec["input"], lib)
        intent, out = g.decide(inp)
        b_clip = intent.clip if intent is not None and out["fallback"] is None and intent.kind == D.IntentKind.PLAY_CLIP \
            else None
        rows.append({"seq": rec["seq"], "a_clip": rec["applied"]["clip"],
                     "b_intent": intent.to_dict() if intent is not None else None, "b_fallback": out["fallback"],
                     "b_latency_ms": out["latency_ms"], "agree": b_clip == rec["applied"]["clip"]})
    g.close()
    n = len(rows) or 1
    return rows, {"questions": len(rows), "b_applied": round(sum(1 for x in rows if x["b_fallback"] is None) / n, 4),
                  "agree_on_clip": round(sum(1 for x in rows if x["agree"]) / n, 4),
                  "b_fallback_by_reason": {k: sum(1 for x in rows if x["b_fallback"] == k) for k in D.FALLBACK_REASONS
                                           if any(x["b_fallback"] == k for x in rows)}}


def compare_table(ma, mb, la="A", lb="B"):
    keys = [k for k in ma if k in mb]
    w = max(len(k) for k in keys)
    out = ["%-*s  %-24s %-24s" % (w, "", la[:24], lb[:24])]
    for k in keys:
        out.append("%-*s  %-24s %-24s" % (w, k, json.dumps(ma[k])[:24], json.dumps(mb[k])[:24]))
    return "\n".join(out)


def self_test():
    import tempfile
    from contracts.recorded_perception import RecordedPerception, write_synthetic
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)
    g = S.Graph.load(os.path.join(ROOT, "shows", "party.compiled.json"))
    d = tempfile.mkdtemp()
    rec, ext = write_synthetic(d, seconds=60.0)
    pa, pb = os.path.join(d, "a.jsonl"), os.path.join(d, "b.jsonl")
    la, ra, _ = closed_loop(g, BaselineDirector.from_graph(g, 1), RecordedPerception(rec, loop=True), minutes=5,
                            seed=1, trigger_every=60, log_path=pa, budget_s=1.0)
    lb, rb, sb = closed_loop(g, AttentiveDirector(), RecordedPerception(rec, loop=True), minutes=5, seed=1,
                             trigger_every=60, log_path=pb, budget_s=1.0)
    ha, da, ea = DL.read(pa)
    check("a log: run_start, a decision a pick, run_end; every line keeps the schema",
          ha and ea and len(da) >= 10 and all(DL.validate_record(x) == [] for x in [ha] + da + [ea]),
          [DL.validate_record(x) for x in da if DL.validate_record(x)][:2])
    check("the baseline: every pick its own, none fallen back", all(not x["fallback"]["used"] for x in da))
    _, db, eb = DL.read(pb)
    reasons = {x["fallback"]["reason"] for x in db}
    check("the example reads the room: some picks its own, the rest deferred to the baseline",
          any(not x["fallback"]["used"] for x in db) and "defer" in reasons, eb["summary"]["counts"])
    check("... a hand up became a greet trigger the Runner took", any(x["applied"]["trigger"] == "greet" for x in db)
          and any(n.startswith("greet_") for _, n in rb.history), [x["applied"] for x in db if x["applied"]["trigger"]])
    ma, mb = DL.metrics(da, ha["library"]), DL.metrics(db, ha["library"])
    check("both runs measured on the same terms", set(ma) == set(mb), sorted(set(ma) ^ set(mb)))
    rows, summ = open_loop(pa, BaselineDirector.from_graph(g, 99), budget_s=1.0)
    check("open loop: every question of A's log asked again", summ["questions"] == len(da) and summ["b_applied"] == 1.0,
          summ)
    print(compare_table(ma, mb, "baseline", "example_attentive"))
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("show", nargs="?", help="shows/<name>.json (its .compiled.json is played)")
    ap.add_argument("--recording", help="a Femto recording (femto_format CSV)")
    ap.add_argument("--extrinsic", help="default: femto_extrinsic.json beside the recording")
    ap.add_argument("--synthetic", action="store_true", help="a synthetic walk-by in place of a recording")
    ap.add_argument("--latency", type=float, default=0.09, help="s from measurement to arrival (track_sim: 60-110 ms)")
    ap.add_argument("--a", default="baseline")
    ap.add_argument("--b", default="attentive")
    ap.add_argument("--minutes", type=float, default=10.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--trigger-every", type=float, default=None, help="a scan every ~N s, as show.py dry-run")
    ap.add_argument("--budget-ms", type=float, default=50.0)
    ap.add_argument("--out", default=os.path.join(ROOT, "logs", "decisions"))
    ap.add_argument("--open-loop", metavar="LOG", help="ask --b every question in this log")
    ap.add_argument("--self-test", action="store_true")
    a = ap.parse_args(argv)
    if a.self_test:
        return self_test()
    budget = a.budget_ms / 1000.0
    if a.open_loop:
        header, _, _ = DL.read(a.open_loop)
        g = S.Graph.load(S.compiled_path(os.path.join(ROOT, header["meta"]["show"]))) if header["meta"].get("show") \
            else None
        rows, summ = open_loop(a.open_loop, make_director(a.b, g, a.seed), budget)
        print(json.dumps(summ, indent=1))
        return 0
    if not a.show:
        ap.error("which show?")
    g = S.Graph.load(S.compiled_path(a.show))
    if S.stale_inputs(g.info, a.show):
        print("note: %s is older than its inputs (fine for a replay: nothing moves)" % S.compiled_path(a.show))

    def source():
        from contracts.recorded_perception import RecordedPerception, write_synthetic
        if a.recording:
            return RecordedPerception(a.recording, a.extrinsic, a.latency, loop=True)
        if a.synthetic:
            import tempfile
            rec, _ = write_synthetic(tempfile.mkdtemp(), seconds=120.0)
            return RecordedPerception(rec, latency_s=a.latency, loop=True)
        return None
    runs = []
    for tag, spec in (("a", a.a), ("b", a.b)):
        d = make_director(spec, g, a.seed)
        path = os.path.join(a.out, "%s_%s.jsonl" % (tag, d.name))
        log, r, sel = closed_loop(g, d, source(), a.minutes, seed=a.seed, trigger_every=a.trigger_every,
                                  budget_s=budget, log_path=path,
                                  meta={"show": os.path.relpath(os.path.abspath(a.show), ROOT).replace("\\", "/")})
        runs.append((d.name, log.records[-1]["summary"]["metrics"], path))
        print("%s: %s" % (d.name, path))
    print(compare_table(runs[0][1], runs[1][1], runs[0][0], runs[1][0]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
