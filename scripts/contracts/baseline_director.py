"""BaselineDirector: the show's existing choice -- show.Selector's weighted
random pick (no repeat within no_repeat, towards the mood and the energy,
away from the family just played) -- behind the Director contract.

It keeps its own show.Selector and, before each pick, sets it to the state
the DirectorInput carries (recent picks, the operator's mood and energy), so
its seeded random draws are the Selector's own: a show.Runner with
DirectorSelector(BaselineDirector(seed=s)) plays exactly the clips a
show.Runner with Selector(seed=s) plays (conformance.py checks this on the
party show). It is what every other Director is measured against, and it
reads nothing of the room (scene) -- as the show does today.

    d = BaselineDirector.from_graph(graph, seed=1)
    intent = d.decide(inp)          -> Intent(PLAY_CLIP, clip=...)
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

import show as S  # noqa: E402
from contracts import director as D  # noqa: E402


class BaselineDirector:
    name = "baseline_selector"
    version = "1"

    def __init__(self, clips, no_repeat=6, seed=None, arc=None):
        self.sel = S.Selector(clips, no_repeat, seed, arc)
        self.by_name = {c.name: c for c in clips}

    @staticmethod
    def from_graph(graph, seed=None):
        sel = graph.info.get("select", {})
        return BaselineDirector(graph.idle(), sel.get("no_repeat", 6), seed, sel.get("arc"))

    def decide(self, inp):
        s = inp.show
        self.sel.recent = list(s.recent)
        self.sel.recent_segs = [self.by_name[n] for n in s.recent_picks if n in self.by_name]
        self.sel.mood, self.sel.energy = s.operator_mood, s.operator_energy
        c = self.sel.pick(s.hub, s.mood, s.clock)
        return D.Intent(D.IntentKind.PLAY_CLIP, clip=c.name, reason="show.Selector: weighted random")
