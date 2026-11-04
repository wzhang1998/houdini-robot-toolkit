"""The Director contract: who chooses what plays next, and how little they
are allowed to say. docs/architecture/contracts.md, Director; ADR 001.

A Director sees a DirectorInput -- a summary of the room (perception.summarize),
the library at the current hub (LibraryIndex), the show's state (ShowState)
-- and answers with an Intent, one of a closed set of kinds:

    PLAY_CLIP  clip      an idle clip at the current hub, by id
    PLAY_LIKE  action / family / energy
                         a kind of clip at the current hub; the show picks one (show.Selector's weights)
    TRIGGER    trigger   a name /robot/trigger takes (scan, a sequence, a showpiece): the Runner routes and
                         refuses as it does an operator's
    DEFER                no opinion: the baseline picks

Never joint angles, poses, speeds or routes: everything a Director can name
was built and checked (show.py build) before the show started, so no answer
-- wrong, late, adversarial -- can move the arm anywhere it was not already
allowed to go. The Runner keeps the final say: GuardedDirector gives a
Director a time budget and checks its answer; on a timeout, an exception,
an answer that does not validate, or DEFER, the existing show.Selector
picks, and the decision log says which happened.

    DirectorSelector(clips, director, ...)   a show.Selector whose pick() asks the Director first --
                                             drop it into show.Runner in place of Selector (nothing in
                                             show.py changes)

    python scripts/contracts/director.py     self-test
"""

import concurrent.futures
import hashlib
import json
import os
import sys
import time
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import List, Optional, Protocol, runtime_checkable

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import show as S  # noqa: E402
from contracts import perception as PC  # noqa: E402

INTENT_SCHEMA = "motionlab.intent/1"
INPUT_SCHEMA = "motionlab.director_input/1"
REASON_MAX = 200
DEFAULT_BUDGET_S = 0.05


class IntentKind(str, Enum):
    PLAY_CLIP = "play_clip"
    PLAY_LIKE = "play_like"
    TRIGGER = "trigger"
    DEFER = "defer"


FIELDS = {IntentKind.PLAY_CLIP: {"clip"}, IntentKind.PLAY_LIKE: {"action", "family", "energy"},
          IntentKind.TRIGGER: {"trigger"}, IntentKind.DEFER: set()}


@dataclass(frozen=True)
class Intent:
    kind: IntentKind
    clip: Optional[str] = None
    action: Optional[str] = None
    family: Optional[str] = None
    energy: Optional[float] = None
    trigger: Optional[str] = None
    reason: str = ""

    def to_dict(self):
        d = {"schema": INTENT_SCHEMA, "kind": self.kind.value}
        for k in ("clip", "action", "family", "energy", "trigger"):
            if getattr(self, k) is not None:
                d[k] = getattr(self, k)
        if self.reason:
            d["reason"] = self.reason
        return d

    @staticmethod
    def from_dict(d):
        """An Intent from a dict (a model's JSON answer). Raises ValueError on an unknown kind or field."""
        if not isinstance(d, dict):
            raise ValueError("an intent is an object, got %s" % type(d).__name__)
        extra = set(d) - {"schema", "kind", "clip", "action", "family", "energy", "trigger", "reason"}
        if extra:
            raise ValueError("unknown fields %s" % sorted(extra))
        if d.get("schema", INTENT_SCHEMA) != INTENT_SCHEMA:
            raise ValueError("schema %r, expected %r" % (d.get("schema"), INTENT_SCHEMA))
        try:
            kind = IntentKind(d.get("kind"))
        except ValueError:
            raise ValueError("kind %r is not one of %s" % (d.get("kind"), [k.value for k in IntentKind]))
        for k in ("clip", "action", "family", "trigger", "reason"):
            if d.get(k) is not None and not isinstance(d[k], str):
                raise ValueError("%s must be a string, got %s" % (k, type(d[k]).__name__))
        e = d.get("energy")
        if e is not None:
            if isinstance(e, str):                         # a model's "0.8": taken if it is a number
                try:
                    e = float(e)
                except ValueError:
                    raise ValueError("energy %r is not a number" % (e,))
            elif isinstance(e, bool) or not isinstance(e, (int, float)):
                raise ValueError("energy must be a number, got %s" % type(e).__name__)
            e = float(e)
        return Intent(kind, d.get("clip"), d.get("action"), d.get("family"), e, d.get("trigger"),
                      (d.get("reason") or "")[:REASON_MAX])

    @staticmethod
    def defer(reason=""):
        return Intent(IntentKind.DEFER, reason=reason)


# --------------------------------------------------------------------------
# the input
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class ClipEntry:
    id: str
    hub: str
    family: str
    action: Optional[str]
    energy: Optional[float]
    duration_s: float
    tags: tuple = ()


@dataclass
class LibraryIndex:
    """The idle clips a Director may name, and the triggers; static for a compiled show."""
    clips: List[ClipEntry]
    triggers: List[str]
    hubs: List[str]
    digest: str = ""

    @staticmethod
    def from_graph(graph, clips=None):
        segs = clips if clips is not None else graph.idle()
        entries = [ClipEntry(s.name, s.start, S.family_of(s), (s.labels or {}).get("action"),
                             (s.labels or {}).get("energy"), round(s.duration, 3),
                             tuple((s.labels or {}).get("intent") or ())) for s in segs]
        showpieces = [s.name for s in segs if "showpiece" in ((s.labels or {}).get("intent") or [])]
        has_scan = any(s.kind == "scan" for s in graph.segments)
        trig = (["scan"] if has_scan else []) + sorted((graph.info or {}).get("sequences", {})) + showpieces
        idx = LibraryIndex(entries, trig, sorted({e.hub for e in entries}))
        idx.digest = hashlib.sha256(json.dumps([asdict(e) for e in entries] + trig, sort_keys=True)
                                    .encode()).hexdigest()[:16]
        return idx

    def at(self, hub):
        return [e for e in self.clips if e.hub == hub]

    def to_dict(self, hub=None):
        return {"digest": self.digest, "triggers": list(self.triggers), "hubs": list(self.hubs),
                "clips": [asdict(e) for e in (self.at(hub) if hub else self.clips)]}


@dataclass
class ShowState:
    clock: float                          # show seconds
    hub: str                              # where the arm is (the next clip starts there)
    state: str = "IDLE"                   # show.Runner.state
    sequence: Optional[str] = None        # the sequence running, if any
    mood: Optional[str] = None            # the mood this pick is asked for (a sequence's), else the operator's
    operator_mood: Optional[str] = None   # /robot/mood (Selector.mood)
    operator_energy: Optional[float] = None   # /robot/energy (Selector.energy; < 0: the arc)
    energy_target: Optional[float] = None     # what the show wants now (Selector.target_energy)
    recent: List[str] = field(default_factory=list)          # Selector.recent (picks, and showpieces played)
    recent_picks: List[str] = field(default_factory=list)    # Selector.recent_segs (picks only, last 6)
    pending: List[str] = field(default_factory=list)         # triggers waiting


@dataclass
class DirectorInput:
    scene: PC.SceneSummary
    library: LibraryIndex
    show: ShowState
    candidates: List[str]                 # idle clip ids at show.hub -- the only ones PLAY_CLIP may name
    seq: int = 0                          # the decision's number in the run

    def to_dict(self):
        """The snapshot the decision log keeps: the library by digest and the candidates by id."""
        return {"schema": INPUT_SCHEMA, "seq": self.seq, "scene": self.scene.to_dict(),
                "library": {"digest": self.library.digest, "triggers": list(self.library.triggers)},
                "show": asdict(self.show), "candidates": list(self.candidates)}

    @staticmethod
    def from_dict(d, library):
        return DirectorInput(PC.SceneSummary(**d["scene"]), library, ShowState(**d["show"]), list(d["candidates"]),
                             d.get("seq", 0))

    def digest(self):
        return hashlib.sha256(json.dumps(self.to_dict(), sort_keys=True).encode()).hexdigest()[:16]


@runtime_checkable
class Director(Protocol):
    name: str
    version: str

    def decide(self, inp: DirectorInput) -> Intent:
        """An Intent for the next pick. May be slow or fail: GuardedDirector contains it. Must not
        keep a reference to inp or change it."""


def validate_intent(intent, inp: DirectorInput) -> List[str]:
    """Why an answer cannot be carried out at this pick ([] when it can)."""
    if not isinstance(intent, Intent):
        return ["not an Intent: %s" % type(intent).__name__]
    errs = []
    allowed = FIELDS[intent.kind]
    for k in ("clip", "action", "family", "energy", "trigger"):
        if getattr(intent, k) is not None and k not in allowed:
            errs.append("%s: field %r not allowed" % (intent.kind.value, k))
    if len(intent.reason) > REASON_MAX:
        errs.append("reason longer than %d" % REASON_MAX)
    if intent.kind == IntentKind.PLAY_CLIP:
        if intent.clip not in inp.candidates:
            errs.append("clip %r is not an idle clip at %s" % (intent.clip, inp.show.hub))
    elif intent.kind == IntentKind.PLAY_LIKE:
        if intent.action is None and intent.family is None and intent.energy is None:
            errs.append("play_like names nothing (action, family or energy)")
        if intent.energy is not None and not (isinstance(intent.energy, (int, float)) and 0.0 <= intent.energy <= 1.0):
            errs.append("energy %r not in [0, 1]" % (intent.energy,))
        here = inp.library.at(inp.show.hub)
        if intent.action is not None and intent.action not in {e.action for e in here}:
            errs.append("no clip at %s has action %r" % (inp.show.hub, intent.action))
        if intent.family is not None and intent.family not in {e.family for e in here}:
            errs.append("no clip at %s is of family %r" % (inp.show.hub, intent.family))
    elif intent.kind == IntentKind.TRIGGER:
        if intent.trigger not in inp.library.triggers:
            errs.append("trigger %r is not one of %s" % (intent.trigger, inp.library.triggers))
    return errs


# --------------------------------------------------------------------------
# the guard: budget, validation, fallback
# --------------------------------------------------------------------------

FALLBACK_REASONS = ("timeout", "error", "invalid", "busy", "defer")


class GuardedDirector:
    """Runs a Director with a time budget on a worker thread. decide() -> (intent or None, outcome).
    Carry out the intent only when outcome["fallback"] is None; otherwise it says why not
    (FALLBACK_REASONS): the intent is None then, except DEFER (returned as given). An answer refused as
    invalid is kept in outcome["rejected"] (its dict) for the log, never returned. A Director still
    running after its budget is left to finish; until it does every call is "busy" (no pile-up)."""

    def __init__(self, director, budget_s=DEFAULT_BUDGET_S):
        self.director, self.budget_s = director, float(budget_s)
        self.pool = concurrent.futures.ThreadPoolExecutor(max_workers=1, thread_name_prefix="director")
        self.running = None

    def decide(self, inp):
        out = {"latency_ms": 0.0, "budget_ms": round(self.budget_s * 1000.0, 3), "fallback": None, "errors": [],
               "raw": None, "rejected": None}
        if self.running is not None and not self.running.done():
            out["fallback"] = "busy"
            return None, out
        t0 = time.perf_counter()
        fut = self.pool.submit(self.director.decide, inp)
        self.running = fut
        try:
            ans = fut.result(timeout=self.budget_s)
        except concurrent.futures.TimeoutError:
            out["latency_ms"] = round((time.perf_counter() - t0) * 1000.0, 3)
            out["fallback"] = "timeout"
            return None, out
        except Exception as e:                                     # the Director's own failure
            out["latency_ms"] = round((time.perf_counter() - t0) * 1000.0, 3)
            out["fallback"], out["errors"] = "error", ["%s: %s" % (type(e).__name__, str(e)[:200])]
            return None, out
        out["latency_ms"] = round((time.perf_counter() - t0) * 1000.0, 3)
        if isinstance(ans, dict):
            out["raw"] = json.dumps(ans)[:500]
            try:
                ans = Intent.from_dict(ans)
            except ValueError as e:
                out["fallback"], out["errors"] = "invalid", [str(e)]
                return None, out
        errs = validate_intent(ans, inp)
        if errs:
            out["fallback"], out["errors"] = "invalid", errs
            if isinstance(ans, Intent):
                out["rejected"] = ans.to_dict()
            return None, out
        if ans.kind == IntentKind.DEFER:
            out["fallback"] = "defer"
        return ans, out

    def close(self):
        self.pool.shutdown(wait=False, cancel_futures=True)


@dataclass
class TriggerPolicy:
    """Limits on what a Director (never the operator) may trigger. The defaults are the safe answers:
    the paper is exposed once (show.Runner's own guard only refuses a scan while one runs or waits,
    so a Director asking at every pick would start one as soon as the last ended)."""
    scan_per_run: int = 1             # Director-issued scans in a run
    cooldown_s: float = 120.0         # show seconds between two Director triggers of the same name
    enforce_no_repeat: bool = True    # PLAY_CLIP of a clip within the last no_repeat is refused (as the
                                      # Selector's pool excludes it), unless every clip here is that recent

    def refuse(self, name, issued, clock):
        """Why a Director's trigger of name is refused now, or None. issued: [(clock, name)] so far."""
        mine = [t for t, n in issued if n == name]
        if name == "scan" and len(mine) >= self.scan_per_run:
            return "trigger scan refused: %d Director scan(s) a run (TriggerPolicy.scan_per_run)" % self.scan_per_run
        if mine and clock - mine[-1] < self.cooldown_s:
            return "trigger %s refused: %.0f s since the last, cooldown %.0f s" % (name, clock - mine[-1],
                                                                                 self.cooldown_s)
        return None


class DirectorSelector(S.Selector):
    """show.Selector whose pick() asks a Director first and falls back to its own weighted random
    pick (the inherited one) -- what show.Runner calls, so the Runner, its routes, its triggers'
    checks and every built clip stay as they are.

    scene_fn(clock) -> SceneSummary or None; log: a decision_log.DecisionLog (or None);
    on_trigger(name): what a TRIGGER intent calls (the Runner's trigger, wired by attach());
    policy: TriggerPolicy (the default: one Director scan a run, a cooldown per trigger, no_repeat held)."""

    def __init__(self, clips, director, graph, no_repeat=6, seed=None, arc=None, budget_s=DEFAULT_BUDGET_S,
                 scene_fn=None, log=None, policy=None):
        super().__init__(clips, no_repeat, seed, arc)
        self.policy = policy or TriggerPolicy()
        self.issued = []                                     # [(clock, name)] of the Director's triggers
        self.guard = GuardedDirector(director, budget_s)
        self.director = director
        self.library = LibraryIndex.from_graph(graph, clips)
        self.by_name = {c.name: c for c in clips}
        self.scene_fn, self.dlog = scene_fn, log
        self.runner, self.on_trigger, self.seq = None, None, 0
        self.counts = {k: 0 for k in ("director",) + FALLBACK_REASONS}

    def attach(self, runner):
        self.runner, self.on_trigger = runner, runner.trigger
        return self

    def _input(self, hub, mood, clock):
        r = self.runner
        show = ShowState(round(clock, 4), hub, getattr(r, "state", "IDLE"), getattr(r, "sequence", None),
                         mood or self.mood, self.mood, self.energy,
                         None if self.target_energy(clock) is None else round(self.target_energy(clock), 4),
                         list(self.recent), [c.name for c in self.recent_segs], list(getattr(r, "pending", [])))
        scene = (self.scene_fn(clock) if self.scene_fn else None) or PC.SceneSummary()
        return DirectorInput(scene, self.library, show, [c.name for c in self.clips if hub is None or c.start == hub],
                             self.seq)

    def _like(self, intent, at, mood, clock):
        pool = [c for c in at if (intent.action is None or (c.labels or {}).get("action") == intent.action)
                and (intent.family is None or S.family_of(c) == intent.family)]
        pool = [c for c in pool if c.name not in self.recent[-self.no_repeat:]] or pool
        if not pool:
            return None
        saved = self.energy
        if intent.energy is not None:
            self.energy = float(intent.energy)
        try:
            w = [self.weight(x, intent.action or mood, clock) for x in pool]
        finally:
            self.energy = saved
        return self.rng.choices(pool, weights=w)[0]

    def pick(self, hub=None, mood=None, clock=0.0):
        self.seq += 1
        inp = self._input(hub, mood, clock)
        t_wall = time.time()
        intent, out = self.guard.decide(inp)
        at = [c for c in self.clips if hub is None or c.start == hub]
        chosen, trig = None, None
        if out["fallback"] is None:
            if intent.kind == IntentKind.PLAY_CLIP:
                window = self.recent[-self.no_repeat:] if self.no_repeat else []
                fresh = [c for c in at if c.name not in window]
                if self.policy.enforce_no_repeat and intent.clip in window and fresh:
                    out["fallback"], out["errors"] = "invalid", [
                        "clip %s repeats within the last %d (no_repeat)" % (intent.clip, self.no_repeat)]
                else:
                    chosen = self.by_name[intent.clip]
            elif intent.kind == IntentKind.PLAY_LIKE:
                chosen = self._like(intent, at, mood, clock)
                if chosen is None:
                    out["fallback"], out["errors"] = "invalid", ["nothing at %s matches %s" % (hub, intent.to_dict())]
            elif intent.kind == IntentKind.TRIGGER:
                why = self.policy.refuse(intent.trigger, self.issued, clock)
                if why:
                    out["fallback"], out["errors"] = "invalid", [why]
                else:
                    trig = intent.trigger
                    self.issued.append((clock, trig))
                    if self.on_trigger is not None:
                        self.on_trigger(trig)                # the Runner checks and routes it, after this clip
        if chosen is not None:
            self.recent.append(chosen.name)                  # show.Selector.pick's bookkeeping
            self.recent_segs = list(self.recent_segs)[-5:] + [chosen]
            clip_by = "director"
        else:
            chosen = super().pick(hub, mood, clock)
            clip_by = "selector"
        self.counts["director" if out["fallback"] is None else out["fallback"]] += 1
        if self.dlog is not None:
            applied = {"source": "director" if out["fallback"] is None else "fallback", "clip": chosen.name,
                       "clip_by": clip_by, "trigger": trig}
            self.dlog.decision(inp, intent, out, applied, self.director, t_wall)
        return chosen

    def close(self):
        self.guard.close()


def selector_for(graph, director, seed=None, budget_s=DEFAULT_BUDGET_S, scene_fn=None, log=None, policy=None):
    """A DirectorSelector over the graph's idle clips, with the show's own select settings (as show.runner_for)."""
    sel = graph.info.get("select", {})
    return DirectorSelector(graph.idle(), director, graph, sel.get("no_repeat", 6), seed=seed, arc=sel.get("arc"),
                            budget_s=budget_s, scene_fn=scene_fn, log=log, policy=policy)


def runner_with(graph, director, seed=None, budget_s=DEFAULT_BUDGET_S, scene_fn=None, log=None, run_log=None,
                policy=None):
    """show.runner_for with a Director: the same Runner, hub_stay and seeds; the Selector a DirectorSelector."""
    sel = selector_for(graph, director, seed, budget_s, scene_fn, log, policy)
    r = S.Runner(graph, sel, hub_stay=tuple(graph.info.get("select", {}).get("hub_stay", (2, 4))), seed=seed,
                 log=run_log)
    return r, sel.attach(r)


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)
    g = S.Graph.load(os.path.join(os.path.dirname(SCRIPTS), "shows", "party.compiled.json"))
    lib = LibraryIndex.from_graph(g)
    rest = [e.id for e in lib.at("rest")]
    inp = DirectorInput(PC.SceneSummary(), lib, ShowState(0.0, "rest"), rest)
    check("the library: every idle clip, the triggers the show takes", len(lib.clips) == len(g.idle())
          and "scan" in lib.triggers and "greet" in lib.triggers, lib.triggers)
    check("PLAY_CLIP of a clip here: valid", validate_intent(Intent(IntentKind.PLAY_CLIP, clip=rest[0]), inp) == [])
    other = next(e.id for e in lib.clips if e.hub != "rest")
    check("PLAY_CLIP of a clip at another hub: refused",
          validate_intent(Intent(IntentKind.PLAY_CLIP, clip=other), inp) != [])
    check("a field the kind does not take: refused",
          validate_intent(Intent(IntentKind.TRIGGER, trigger="scan", clip=rest[0]), inp) != [])
    for bad in ({"kind": "joints", "q": [0] * 6}, {"kind": "play_clip", "clip": rest[0], "speed": 2.0}):
        try:
            Intent.from_dict(bad)
            check("an answer outside the enum is not parsed: %s" % bad, False)
        except ValueError as e:
            check("an answer outside the enum is not parsed: %s" % bad, True, e)
    check("unknown trigger refused; a known one taken",
          validate_intent(Intent(IntentKind.TRIGGER, trigger="dance"), inp) != []
          and validate_intent(Intent(IntentKind.TRIGGER, trigger="scan"), inp) == [])
    for bad in ({"kind": "play_like", "energy": True}, {"kind": "play_clip", "clip": 3},
                {"kind": "play_like", "energy": "lots"}, {"kind": "trigger", "trigger": ["scan"]}):
        try:
            Intent.from_dict(bad)
            check("a field of the wrong type is not parsed: %s" % bad, False)
        except ValueError as e:
            check("a field of the wrong type is not parsed: %s" % bad, True, e)
    check("... but a number given as a string is taken (energy \"0.8\")",
          Intent.from_dict({"kind": "play_like", "energy": "0.8"}).energy == 0.8)
    check("an intent round-trips through JSON",
          Intent.from_dict(json.loads(json.dumps(Intent(IntentKind.PLAY_LIKE, action="float", energy=0.2).to_dict())))
          == Intent(IntentKind.PLAY_LIKE, action="float", energy=0.2))

    class Slow:
        name, version = "slow", "0"

        def decide(self, inp):
            time.sleep(0.2)
            return Intent(IntentKind.PLAY_CLIP, clip=inp.candidates[0])

    class Boom:
        name, version = "boom", "0"

        def decide(self, inp):
            raise RuntimeError("model offline")

    class Liar:
        name, version = "liar", "0"

        def decide(self, inp):
            return {"kind": "play_clip", "clip": "not_a_clip"}

    gd = GuardedDirector(Slow(), budget_s=0.02)
    a, o = gd.decide(inp)
    a2, o2 = gd.decide(inp)
    check("over budget: timeout, and while it still runs the next call is busy (no pile-up)",
          a is None and o["fallback"] == "timeout" and o2["fallback"] == "busy", (o, o2))
    time.sleep(0.25)
    check("... once it has finished, it is asked again", gd.decide(inp)[1]["fallback"] == "timeout")
    gd.close()
    check("an exception: error", GuardedDirector(Boom()).decide(inp)[1]["fallback"] == "error")
    a, o = GuardedDirector(Liar()).decide(inp)
    check("a clip that does not exist: invalid, nothing returned to carry out", a is None and o["fallback"] == "invalid")

    class WrongHub:
        name, version = "wrong_hub", "0"

        def decide(self, inp):
            return Intent(IntentKind.PLAY_CLIP, clip=other)
    a, o = GuardedDirector(WrongHub()).decide(inp)
    check("... an Intent refused is kept in the outcome (for the log), not returned",
          a is None and o["rejected"] == {"schema": INTENT_SCHEMA, "kind": "play_clip", "clip": other}, o)

    sel, _ = None, None
    r, sel = runner_with(g, Boom(), seed=3)
    for _ in range(int(120 / 0.05)):
        r.step(0.05)
    check("a Director that always fails: the show plays on, every pick the Selector's",
          len(r.history) > 5 and sel.counts["error"] == sel.seq and sel.counts["director"] == 0, sel.counts)
    sel.close()
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
