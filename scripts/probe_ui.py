"""A window for probe_env.py: measure the room with the arm's tool tip.

    python scripts/probe_ui.py
    python scripts/probe_ui.py --self-test

Read-only -- it never commands a motion. Put the arm in hand-guiding (drag
teach) from the WebUI, bring the tool tip onto a point, pick the object and
press Record (or Enter). Points go to the same file probe_env.py writes
(default envs/volvox_lab_points.json), one record per press.

    Live         the arm's TCP by this toolkit's URDF and by the controller,
                 and how far apart they are (the FK cross-check, every pose)
    Objects      how many points each object has and how many its fit needs
    Preview fit  what env_from_points.py would change, without writing
    Write env    fit and write the env file (the old one kept as .bak)

The controller IP starts from playback.toml ([robot] ip), the file
play_ui.py uses.
"""

import copy
import http.client
import json
import math
import os
import sys
import threading
import time
import tkinter as tk
import xmlrpc.client
from tkinter import filedialog, messagebox, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import collision as CL  # noqa: E402
import env_from_points as EFP  # noqa: E402
import probe_env  # noqa: E402

POINTS = os.path.join(ROOT, "envs", "volvox_lab_points.json")
ENV = os.path.join(ROOT, "envs", "volvox_lab.json")
NEED = {"plane": 3, "box": 3, "cylinder": 3, "point": 1, "bottom": 1, "top": 1}
PRESETS = ["floor:plane", "wall_tv:plane", "partition_left:plane", "control_cart:box",
           "control_cart:bottom", "operator:cylinder", "stage:point"]


class _Timeout(xmlrpc.client.Transport):
    """XML-RPC with a socket timeout, so a dead link does not hang the window."""

    def __init__(self, timeout):
        super().__init__()
        self.timeout = timeout

    def make_connection(self, host):
        return http.client.HTTPConnection(host, timeout=self.timeout)


def proxy(ip, timeout=2.0):
    return xmlrpc.client.ServerProxy("http://%s:20003" % ip, transport=_Timeout(timeout))


def fk_gap_mm(rec):
    """Distance between the URDF TCP and the controller's TCP, mm (None when
    the controller pose is missing). Only meaningful when the controller's
    active tool matches Tool Length."""
    c = rec.get("controller_tcp_mm_deg")
    if not c:
        return None
    return 1000.0 * math.dist(rec["tcp_m"], [x / 1000.0 for x in c[:3]])


def group_status(points):
    """[(name:kind, count, needed)] in first-recorded order."""
    order, count = [], {}
    for p in points:
        n = p["name"]
        if n not in count:
            order.append(n)
            count[n] = 0
        count[n] += 1
    return [(n, count[n], NEED.get(n.partition(":")[2] or "point", 1)) for n in order]


def preview(env, points):
    """What Write env would do: (change lines, validation errors). env is untouched."""
    return EFP.update_env(copy.deepcopy(env), points)


def load_ip():
    try:
        import play
        return play.load()["robot"].get("ip", "")
    except Exception:
        return "192.168.58.2"


class App:
    def __init__(self, root):
        self.root = root
        root.title("FR20 room probe (read-only)")
        self.data = None
        self.live_q = None
        self.v = {"ip": tk.StringVar(value=load_ip()),
                  "tool_len": tk.DoubleVar(value=0.0),
                  "points": tk.StringVar(value=POINTS),
                  "env": tk.StringVar(value=ENV),
                  "name": tk.StringVar(value=PRESETS[2]),
                  "live": tk.BooleanVar(value=True)}

        f = ttk.Frame(root, padding=10)
        f.grid(sticky="nsew")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        f.columnconfigure(1, weight=1)

        r = 0
        ttk.Label(f, text="Controller IP").grid(row=r, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.v["ip"], width=18).grid(row=r, column=1, sticky="w")
        ttk.Label(f, text="Tool length m").grid(row=r, column=2, sticky="e")
        ttk.Entry(f, textvariable=self.v["tool_len"], width=7).grid(row=r, column=3, sticky="w")
        r += 1
        for key, label in (("points", "Points file"), ("env", "Env file")):
            ttk.Label(f, text=label).grid(row=r, column=0, sticky="w")
            ttk.Entry(f, textvariable=self.v[key]).grid(row=r, column=1, columnspan=2, sticky="we")
            ttk.Button(f, text="Browse...", command=lambda k=key: self._browse(k)).grid(row=r, column=3, sticky="w")
            r += 1

        lf = ttk.LabelFrame(f, text="Live (read-only)", padding=6)
        lf.grid(row=r, column=0, columnspan=4, sticky="we", pady=6)
        ttk.Checkbutton(lf, text="poll", variable=self.v["live"], command=self._live_toggle).grid(row=0, column=0, sticky="w")
        self.live = ttk.Label(lf, text="--", font=("Consolas", 9), justify="left")
        self.live.grid(row=0, column=1, sticky="w", padx=8)
        r += 1

        rf = ttk.Frame(f)
        rf.grid(row=r, column=0, columnspan=4, sticky="we")
        ttk.Label(rf, text="Object  name:kind").grid(row=0, column=0, sticky="w")
        cb = ttk.Combobox(rf, textvariable=self.v["name"], values=PRESETS, width=28)
        cb.grid(row=0, column=1, sticky="w", padx=4)
        tk.Button(rf, text="Record point  (Enter)", bg="#1f6feb", fg="white", font=("Segoe UI", 10, "bold"),
                  command=self.record).grid(row=0, column=2, padx=6)
        ttk.Button(rf, text="Undo last", command=self.undo).grid(row=0, column=3)
        ttk.Button(rf, text="Delete selected", command=self.delete).grid(row=0, column=4, padx=4)
        ttk.Label(rf, text="kinds: plane (3+ on a wall / floor), box (3+ top corners; :bottom one on its base "
                           "if not on the floor), cylinder (3+ round its foot), point",
                  foreground="#555").grid(row=1, column=0, columnspan=5, sticky="w", pady=(2, 0))
        root.bind("<Return>", lambda e: self.record())
        r += 1

        cols = ("name", "x", "y", "z", "fk")
        self.tree = ttk.Treeview(f, columns=cols, show="headings", height=10)
        for c, w, t in zip(cols, (200, 80, 80, 80, 150), ("name:kind", "x m", "y m", "z m", "URDF vs controller mm")):
            self.tree.heading(c, text=t)
            self.tree.column(c, width=w, anchor="w" if c == "name" else "e")
        self.tree.grid(row=r, column=0, columnspan=4, sticky="nsew", pady=6)
        f.rowconfigure(r, weight=1)
        r += 1
        self.groups = ttk.Label(f, text="", justify="left")
        self.groups.grid(row=r, column=0, columnspan=4, sticky="w")
        r += 1

        bf = ttk.Frame(f)
        bf.grid(row=r, column=0, columnspan=4, sticky="w", pady=6)
        ttk.Button(bf, text="Preview fit", command=self.preview).grid(row=0, column=0)
        ttk.Button(bf, text="Write env", command=self.write_env).grid(row=0, column=1, padx=6)
        r += 1
        self.log = tk.Text(f, height=8, font=("Consolas", 9))
        self.log.grid(row=r, column=0, columnspan=4, sticky="nsew")
        self.status = ttk.Label(f, text="ready")
        self.status.grid(row=r + 1, column=0, columnspan=4, sticky="w")

        self._load()
        self._live_toggle()

    # -- file ----------------------------------------------------------------
    def _browse(self, key):
        p = filedialog.askopenfilename(initialdir=os.path.dirname(self.v[key].get()) or ROOT,
                                       filetypes=[("JSON", "*.json"), ("All", "*.*")])
        if p:
            self.v[key].set(p)
            if key == "points":
                self._load()

    def _load(self):
        p = self.v["points"].get()
        self.data = {"schema": "motionlab.points/1", "tool_len_m": self.v["tool_len"].get(), "points": []}
        if os.path.exists(p):
            with open(p) as fh:
                self.data = json.load(fh)
            self.v["tool_len"].set(self.data.get("tool_len_m", 0.0))
        self._refresh()

    def _save(self):
        p = self.v["points"].get()
        os.makedirs(os.path.dirname(os.path.abspath(p)), exist_ok=True)
        with open(p, "w") as fh:
            json.dump(self.data, fh, indent=1)

    def _refresh(self):
        self.tree.delete(*self.tree.get_children())
        for i, rec in enumerate(self.data["points"]):
            gap = fk_gap_mm(rec)
            self.tree.insert("", "end", iid=str(i), values=(
                rec["name"], *("%.4f" % x for x in rec["tcp_m"]), "--" if gap is None else "%.2f" % gap))
        lines = ["%s  %d/%d %s" % (n, c, need, "ok" if c >= need else "needs %d more" % (need - c))
                 for n, c, need in group_status(self.data["points"])]
        self.groups.config(text="Objects:  " + ("   |   ".join(lines) if lines else "none yet"))

    def _say(self, text):
        self.log.insert("end", text + "\n")
        self.log.see("end")

    # -- probe ---------------------------------------------------------------
    def record(self):
        name = self.v["name"].get().strip()
        if not name:
            return
        tl = self.v["tool_len"].get()
        if self.data["points"] and abs(self.data.get("tool_len_m", 0.0) - tl) > 1e-9:
            if not messagebox.askyesno("Tool length", "This file's points were taken with tool length %g m; "
                                       "now %g m. Record anyway?" % (self.data.get("tool_len_m", 0.0), tl)):
                return
        self.data["tool_len_m"] = tl
        try:
            rec = dict(probe_env.read_pose(proxy(self.v["ip"].get()), CL.load_model("fr20", tool_len=tl)), name=name)
        except Exception as e:
            self.status.config(text="read failed: %s" % e)
            self._say("!! could not read the pose: %s" % e)
            return
        self.data["points"].append(rec)
        self._save()
        self._refresh()
        gap = fk_gap_mm(rec)
        self._say("%-26s TCP %s m%s" % (name, rec["tcp_m"], "" if gap is None else "   URDF vs controller %.2f mm" % gap))
        self.status.config(text="recorded %s (%d points)" % (name, len(self.data["points"])))

    def undo(self):
        if self.data["points"]:
            rec = self.data["points"].pop()
            self._save()
            self._refresh()
            self._say("removed %s" % rec["name"])

    def delete(self):
        sel = sorted((int(i) for i in self.tree.selection()), reverse=True)
        for i in sel:
            self._say("removed %s" % self.data["points"].pop(i)["name"])
        if sel:
            self._save()
            self._refresh()

    # -- fit -----------------------------------------------------------------
    def _env(self):
        with open(self.v["env"].get()) as fh:
            return json.load(fh)

    def preview(self):
        try:
            lines, errs = preview(self._env(), self.data["points"])
        except Exception as e:
            self._say("!! preview failed: %s" % e)
            return
        self._say("-- preview (nothing written):")
        for line in lines or ["no object has enough points yet"]:
            self._say("   " + line)
        if errs:
            self._say("   env would be INVALID: %s" % errs)

    def write_env(self):
        path = self.v["env"].get()
        try:
            env = self._env()
            lines, errs = EFP.update_env(env, self.data["points"])
        except Exception as e:
            self._say("!! fit failed: %s" % e)
            return
        if errs:
            self._say("!! not written, env would be invalid: %s" % errs)
            return
        if not lines:
            self._say("nothing to write: no object has enough points yet")
            return
        if not messagebox.askyesno("Write env", "Update %s?\n\n%s\n\nThe old file is kept as .bak."
                                   % (os.path.basename(path), "\n".join(lines))):
            return
        import shutil
        shutil.copyfile(path, path + ".bak")
        with open(path, "w") as fh:
            json.dump(env, fh, indent=1)
        self._say("-- wrote %s (previous: .bak):" % path)
        for line in lines:
            self._say("   " + line)
        self.status.config(text="env written; check it in Houdini (robot_arm > Display > Cell)")

    # -- live ----------------------------------------------------------------
    def _live_toggle(self):
        if self.v["live"].get() and self.live_q is None:
            self.live_q = {"stop": False, "text": "connecting..."}
            threading.Thread(target=self._poll, args=(self.live_q,), daemon=True).start()
            self._show_live()
        elif not self.v["live"].get() and self.live_q is not None:
            self.live_q["stop"] = True
            self.live_q = None
            self.live.config(text="--")

    def _poll(self, q):
        while not q["stop"]:
            try:
                tl = float(self.v["tool_len"].get())
                rec = probe_env.read_pose(proxy(self.v["ip"].get(), 1.0), CL.load_model("fr20", tool_len=tl))
                gap = fk_gap_mm(rec)
                c = rec["controller_tcp_mm_deg"]
                q["text"] = ("joints  %s deg\nURDF TCP  %s m\ncontroller TCP  %s mm   gap %s" % (
                    " ".join("%7.2f" % x for x in rec["joints_deg"]),
                    " ".join("%8.4f" % x for x in rec["tcp_m"]),
                    " ".join("%8.1f" % x for x in c[:3]) if c else "--",
                    "--" if gap is None else "%.2f mm" % gap))
            except Exception as e:
                q["text"] = "no connection: %s" % e
            time.sleep(0.5)

    def _show_live(self):
        if self.live_q is not None:
            self.live.config(text=self.live_q["text"])
            self.root.after(300, self._show_live)


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    rec = {"tcp_m": [0.5, -1.0, 1.2], "controller_tcp_mm_deg": [500.0, -1000.0, 1203.0, 0, 0, 0]}
    check("URDF vs controller gap in mm", abs(fk_gap_mm(rec) - 3.0) < 1e-9, fk_gap_mm(rec))
    check("no controller pose -> no gap", fk_gap_mm({"tcp_m": [0, 0, 0], "controller_tcp_mm_deg": None}) is None)
    pts = [{"name": n, "tcp_m": p} for n, p in (
        ("partition_left:plane", [-0.5, -1.1, 0.5]), ("partition_left:plane", [0.5, -1.1, 0.6]),
        ("control_cart:box", [0.9, 1.0, 0.8]), ("partition_left:plane", [0.0, -1.1, 1.5]))]
    g = group_status(pts)
    check("objects counted in recorded order, with what each fit needs",
          g == [("partition_left:plane", 3, 3), ("control_cart:box", 1, 3)], g)
    with open(ENV) as fh:
        env = json.load(fh)
    before = json.dumps(env, sort_keys=True)
    lines, errs = preview(env, pts)
    check("preview fits the wall and leaves the env untouched",
          not errs and any("partition_left" in s for s in lines) and json.dumps(env, sort_keys=True) == before,
          lines + errs)

    class FakeRPC:
        def GetActualJointPosDegree(self, flag):
            return [0, 0.0, -90.0, 90.0, -90.0, -90.0, 0.0]

        def GetActualTCPPose(self, flag):
            return [0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    r = probe_env.read_pose(FakeRPC(), CL.load_model("fr20", tool_len=0.0))
    check("a pose read at HOME is in front of and above the base (URDF -X is the front)",
          r["tcp_m"][0] < -0.5 and r["tcp_m"][2] > 0.8, r["tcp_m"])
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    root = tk.Tk()
    App(root)
    root.mainloop()
