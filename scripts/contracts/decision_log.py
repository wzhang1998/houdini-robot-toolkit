"""The decision log: one JSON line a Director decision (motionlab.decision/1),
a run_start line before them and a run_end line after.
docs/architecture/decision_log.md has the schema in full.

    log = DecisionLog("logs/decisions/run.jsonl", run_id="...")
    log.start(library, {"show": ..., "director": ..., "perception": ...})
    ... DirectorSelector(..., log=log) writes a line per pick ...
    log.end({...})

    header, decisions, end = read(path)
    metrics(decisions, library_dict)      -> fallback rate, latency, variety, energy tracking, responsiveness
    validate_record(rec)                  -> [errors]
"""

import json
import os
import statistics
import sys
import time
import uuid

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

SCHEMA = "motionlab.decision/1"
KINDS = ("run_start", "decision", "run_end")


class DecisionLog:
    """Lines to a JSONL file (path) and/or kept in memory (.records)."""

    def __init__(self, path=None, run_id=None, keep=True):
        self.path, self.run_id, self.keep = path, run_id or uuid.uuid4().hex[:12], keep
        self.records = []
        self.f = None
        if path:
            os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
            self.f = open(path, "w", encoding="utf-8")

    def _write(self, rec):
        rec = dict({"schema": SCHEMA, "run_id": self.run_id}, **rec)
        if self.keep:
            self.records.append(rec)
        if self.f:
            self.f.write(json.dumps(rec, sort_keys=True) + "\n")
            self.f.flush()
        return rec

    def start(self, library, meta=None):
        return self._write({"kind": "run_start", "t_wall": time.time(), "library": library.to_dict(),
                            "meta": meta or {}})

    def decision(self, inp, intent, outcome, applied, director, t_wall=None):
        errs = list(outcome.get("errors") or [])
        return self._write({
            "kind": "decision", "seq": inp.seq, "t_wall": t_wall if t_wall is not None else time.time(),
            "show_clock": inp.show.clock,
            "director": {"name": getattr(director, "name", type(director).__name__),
                         "version": str(getattr(director, "version", "?"))},
            "input": inp.to_dict(), "input_digest": inp.digest(),
            "output": {"intent": intent.to_dict() if intent is not None else outcome.get("rejected"),
                       "raw": outcome.get("raw")},
            "validation": {"ok": not errs, "errors": errs},
            "fallback": {"used": outcome.get("fallback") is not None, "reason": outcome.get("fallback")},
            "latency_ms": outcome.get("latency_ms"), "budget_ms": outcome.get("budget_ms"),
            "applied": applied})

    def end(self, summary=None):
        rec = self._write({"kind": "run_end", "t_wall": time.time(), "summary": summary or {}})
        if self.f:
            self.f.close()
            self.f = None
        return rec


def read(path):
    """(run_start, [decision], run_end or None) of a JSONL log."""
    header, decisions, end = None, [], None
    with open(path, encoding="utf-8") as f:
        for line in f:
            if not line.strip():
                continue
            rec = json.loads(line)
            if rec.get("kind") == "run_start":
                header = rec
            elif rec.get("kind") == "decision":
                decisions.append(rec)
            elif rec.get("kind") == "run_end":
                end = rec
    return header, decisions, end


REQUIRED = {"decision": ("seq", "t_wall", "show_clock", "director", "input", "input_digest", "output", "validation",
                         "fallback", "latency_ms", "budget_ms", "applied"),
            "run_start": ("t_wall", "library", "meta"), "run_end": ("t_wall", "summary")}


def validate_record(rec):
    errs = []
    if rec.get("schema") != SCHEMA:
        errs.append("schema %r" % rec.get("schema"))
    kind = rec.get("kind")
    if kind not in KINDS:
        return errs + ["kind %r" % kind]
    errs += ["%s: missing %s" % (kind, k) for k in REQUIRED[kind] if k not in rec]
    if kind == "decision" and not errs:
        if rec["fallback"]["used"] != (rec["fallback"]["reason"] is not None):
            errs.append("fallback.used disagrees with fallback.reason")
        if rec["applied"].get("source") not in ("director", "fallback"):
            errs.append("applied.source %r" % rec["applied"].get("source"))
        if rec["applied"].get("clip") is None:
            errs.append("applied.clip missing: every pick plays a clip")
        if rec["fallback"]["used"] and rec["applied"].get("source") != "fallback":
            errs.append("a fallback applied as the director's")
    return errs


def _pct(xs, p):
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    return round(xs[min(len(xs) - 1, int(p * len(xs)))], 3)


def metrics(decisions, library=None, no_repeat=6):
    """What two runs (or two Directors on one input stream) are compared on. library: the run_start's
    library dict (for clip energy and family); without it those metrics are None."""
    n = len(decisions)
    if not n:
        return {"decisions": 0}
    by_id = {c["id"]: c for c in (library or {}).get("clips", [])}
    reasons = {}
    for d in decisions:
        r = d["fallback"]["reason"] or "none"
        reasons[r] = reasons.get(r, 0) + 1
    clips = [d["applied"]["clip"] for d in decisions]
    repeats = sum(1 for i, c in enumerate(clips) if c in clips[max(0, i - no_repeat):i])
    fam = [by_id[c]["family"] for c in clips if c in by_id] if by_id else []
    errs = [abs(by_id[d["applied"]["clip"]]["energy"] - d["input"]["show"]["energy_target"]) for d in decisions
            if by_id and d["input"]["show"].get("energy_target") is not None
            and (by_id.get(d["applied"]["clip"]) or {}).get("energy") is not None]
    with_people = [d for d in decisions if (d["input"]["scene"].get("n_people") or 0) > 0]
    return {
        "decisions": n,
        "director_applied": round(sum(1 for d in decisions if not d["fallback"]["used"]) / n, 4),
        "fallback_by_reason": reasons,
        "latency_ms_p50": _pct([d["latency_ms"] for d in decisions], 0.5),
        "latency_ms_p95": _pct([d["latency_ms"] for d in decisions], 0.95),
        "latency_ms_max": _pct([d["latency_ms"] for d in decisions], 1.0),
        "distinct_clips": len(set(clips)),
        "repeats_within_no_repeat": repeats,
        "same_family_twice": sum(1 for a, b in zip(fam, fam[1:]) if a == b) if fam else None,
        "energy_abs_error_mean": round(statistics.mean(errs), 4) if errs else None,
        "triggers_issued": sum(1 for d in decisions if d["applied"].get("trigger")),
        "decisions_with_people": len(with_people),
        "director_applied_with_people": round(sum(1 for d in with_people if not d["fallback"]["used"])
                                              / len(with_people), 4) if with_people else None,
    }
