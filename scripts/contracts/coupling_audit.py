"""The numbers in docs/architecture/coupling.md, measured again: who imports
fairino_player (and for what), where the string "fr20" is hard-coded (and
what it is passed to), who depends on the Femto's format. Reads the source
with ast; imports nothing of it.

    python scripts/contracts/coupling_audit.py            the tables
    python scripts/contracts/coupling_audit.py --sites    every site, file:line
"""

import ast
import collections
import glob
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)

DRIVER = {"Controller", "Feedback", "play", "goto", "run_path", "move_checked", "move_plan", "SIM_NET",
          "RPC_TIMEOUT_S", "MOVE_ERRORS", "Stopped", "_require_no_error", "_current_frames", "check", "main"}


def sources():
    for f in sorted(glob.glob(os.path.join(SCRIPTS, "**", "*.py"), recursive=True)):
        rel = os.path.relpath(f, SCRIPTS).replace("\\", "/")
        if rel.startswith("contracts/"):
            continue
        with open(f, encoding="utf-8") as fh:
            yield rel, fh.read()


def player_uses():
    """{file: Counter(symbol)} of fairino_player's symbols used outside it."""
    out = {}
    for rel, s in sources():
        if rel == "fairino_player.py":
            continue
        c = collections.Counter()
        aliases = {m.group(1) or "fairino_player" for m in re.finditer(r"import fairino_player(?: as (\w+))?", s)}
        for m in re.finditer(r"from fairino_player import ([\w, ]+)", s):
            for n in m.group(1).split(","):
                c[n.strip()] += 1
        for a in aliases:
            for m in re.finditer(r"\b%s\.(\w+)" % a, s):
                if m.group(1) != "py":
                    c[m.group(1)] += 1
        if re.search(r"[\"']fairino_player\.py[\"']", s) and not c:
            c["(runs fairino_player.py)"] += 1
        if c:
            out[rel] = c
    return out


def fr20_sites():
    """[(file, line, category, passed_to)] of every "fr20" string constant."""
    rows = []
    for rel, s in sources():
        tree = ast.parse(s)
        parents = {c: n for n in ast.walk(tree) for c in ast.iter_child_nodes(n)}
        mains = [(n.lineno, n.end_lineno) for n in ast.walk(tree) if isinstance(n, ast.If)
                 and isinstance(n.test, ast.Compare) and getattr(n.test.left, "id", None) == "__name__"]
        for n in ast.walk(tree):
            if not (isinstance(n, ast.Constant) and n.value == "fr20"):
                continue
            p, fn, default = n, None, False
            while p in parents:
                q = parents[p]
                if isinstance(q, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    default = n in q.args.defaults or n in q.args.kw_defaults
                    fn = q.name
                    break
                p = q
            par = parents.get(n)
            to = None
            if isinstance(par, ast.Call):
                to = getattr(par.func, "attr", None) or getattr(par.func, "id", None)
            elif isinstance(par, ast.keyword) and isinstance(parents.get(par), ast.Call):
                f = parents[par].func
                to = "%s(%s=)" % (getattr(f, "attr", None) or getattr(f, "id", None), par.arg)
            elif isinstance(par, ast.Dict):
                to = "a dict value (clip['robot'])"
            if default:
                cat, to = "default parameter", "def %s(robot='fr20')" % fn
            elif to and "add_argument" in to:
                cat = "CLI default"
            elif (fn and "test" in fn) or any(a <= n.lineno <= b for a, b in mains):
                cat = "test"
            else:
                cat = "code"
            rows.append((rel, n.lineno, cat, to or type(par).__name__))
    return rows


def camera_sites():
    pats = {"femto_format (Kinect Azure CHOP layout)": r"import femto_format",
            "/track/people, /track/hands (robot frame)": r'"/track/(people|hands)"',
            "/track/target, /track/lost (the OAK-D proposal)": r'"/track/(target|lost)"|TargetInput',
            "femto_*extrinsic.json by name": r"femto_(sim_)?extrinsic"}
    out = collections.defaultdict(list)
    for rel, s in sources():
        for k, p in pats.items():
            if re.search(p, s) and not (k.startswith("femto_format") and rel == "femto_format.py"):
                out[k].append(rel)
    return out


def main(argv=None):
    sites = "--sites" in (argv or sys.argv[1:])
    uses = player_uses()
    drv = {f: c for f, c in uses.items() if set(c) & (DRIVER | {"(runs fairino_player.py)"})}
    math_only = {f: c for f, c in uses.items() if f not in drv}
    print("fairino_player used by %d files: %d for the driver, %d for trajectory math only" % (
        len(uses), len(drv), len(math_only)))
    for title, group in (("driver", drv), ("trajectory math only", math_only)):
        print("  %s:" % title)
        for f, c in sorted(group.items()):
            print("    %-28s %s" % (f, ", ".join(sorted(c))))
    rows = fr20_sites()
    by_cat = collections.Counter(r[2] for r in rows)
    print('\n"fr20" string constants: %d in %d files -- %s' % (len(rows), len({r[0] for r in rows}),
                                                              dict(by_cat)))
    code = collections.Counter("def f(robot='fr20') -- a default parameter" if r[2] == "default parameter"
                               else r[3] for r in rows if r[2] != "test")
    for to, k in code.most_common():
        print("    %3d  %s" % (k, to))
        if sites:
            print("         " + " ".join("%s:%d" % (r[0], r[1]) for r in rows if r[2] != "test" and (
                r[3] == to or (r[2] == "default parameter" and to.startswith("def f(")))))
    print("\ncamera format:")
    for k, v in camera_sites().items():
        print("  %-48s %d: %s" % (k, len(v), " ".join(sorted(v))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
