"""A window for accel_probe.py: how hard can each joint accelerate.

    python scripts/accel_ui.py
    python scripts/accel_ui.py --self-test

Pick the joint, amplitude, direction and levels; 1 Check reads the pose
and checks the test from it (joint limits, the room) without moving; 2 Run
checks again, then runs the levels one by one -- on the real arm it asks
before each -- and stops at the first level with a controller error or
tracking past Max Error. Each level is its own accel_probe.py process, so
STOP (StopMotion + ServoMoveEnd, then the process ends) can always cut in;
the E-stop is the safety device. Results go to tests/accel/.

Target and IP start from playback.toml (play_ui.py's file); this window
does not write it.
"""

import json
import os
import queue
import subprocess
import sys
import threading
import time
import tkinter as tk
import xmlrpc.client
from tkinter import messagebox, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
PROBE = os.path.join(HERE, "accel_probe.py")
OUT = os.path.join(ROOT, "tests", "accel")


def last_json(text):
    """The JSON report accel_probe prints last."""
    for line in reversed(text.strip().splitlines()):
        line = line.strip()
        if line.startswith("{"):
            try:
                return json.loads(line)
            except ValueError:
                return None
    return None


def argv_for(g, level=None, check_only=False):
    a = ["--" + g["target"], "--ip", g["ip"], "--joint", str(g["joint"]), "--amp", str(g["amp"]),
         "--direction", "1" if g["direction"] == "+" else "-1", "--max-error", str(g["max_error"]),
         "--env", g["env"]]
    if check_only:
        return a + ["--check-only"]
    return a + ["--levels", str(level), "--yes"]


def parse_levels(text):
    vals = [float(x) for x in text.replace(",", " ").split()]
    if not vals or any(v <= 0 for v in vals) or vals != sorted(vals):
        raise ValueError("levels: positive numbers, lowest first")
    return vals


class App:
    def __init__(self, root):
        self.root = root
        root.title("FR20 acceleration probe")
        self.proc = None
        self.out = queue.Queue()
        self.buffer = ""
        self.queue_levels = []
        self.results = []
        try:
            import play
            r = play.load()["robot"]
        except Exception:
            r = {}
        self.v = {"target": tk.StringVar(value=r.get("target", "sim")), "ip": tk.StringVar(value=r.get("ip", "")),
                  "joint": tk.IntVar(value=6), "amp": tk.DoubleVar(value=3.0), "direction": tk.StringVar(value="+"),
                  "levels": tk.StringVar(value="150 225 300 450 600 900"), "max_error": tk.DoubleVar(value=0.5),
                  "env": tk.StringVar(value=os.path.join(ROOT, "envs", "volvox_lab.json"))}
        f = ttk.Frame(root, padding=10)
        f.grid(sticky="nsew")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        f.columnconfigure(1, weight=1)
        r0 = 0
        tf = ttk.Frame(f)
        tf.grid(row=r0, column=0, columnspan=4, sticky="w")
        ttk.Label(tf, text="Target").grid(row=0, column=0)
        for i, (val, txt) in enumerate((("sim", "SimMachine"), ("hardware", "Hardware (real arm)"))):
            ttk.Radiobutton(tf, text=txt, value=val, variable=self.v["target"],
                            command=self._target_changed).grid(row=0, column=1 + i, padx=4)
        ttk.Label(tf, text="IP").grid(row=0, column=3, padx=(12, 2))
        ttk.Entry(tf, textvariable=self.v["ip"], width=16).grid(row=0, column=4)
        self.warn = ttk.Label(tf, text="", foreground="#c00000")
        self.warn.grid(row=1, column=0, columnspan=5, sticky="w")
        r0 += 1
        pf = ttk.LabelFrame(f, text="Test", padding=6)
        pf.grid(row=r0, column=0, columnspan=4, sticky="we", pady=6)
        ttk.Label(pf, text="Joint").grid(row=0, column=0)
        ttk.Spinbox(pf, from_=1, to=6, textvariable=self.v["joint"], width=4,
                    command=self._joint_changed).grid(row=0, column=1)
        ttk.Label(pf, text="Amplitude deg").grid(row=0, column=2, padx=(10, 2))
        ttk.Entry(pf, textvariable=self.v["amp"], width=6).grid(row=0, column=3)
        ttk.Label(pf, text="Direction").grid(row=0, column=4, padx=(10, 2))
        ttk.Combobox(pf, textvariable=self.v["direction"], values=("+", "-"), width=3, state="readonly").grid(row=0, column=5)
        ttk.Label(pf, text="Max error deg").grid(row=0, column=6, padx=(10, 2))
        ttk.Entry(pf, textvariable=self.v["max_error"], width=6).grid(row=0, column=7)
        ttk.Label(pf, text="Levels deg/s²").grid(row=1, column=0, columnspan=2, sticky="w", pady=(6, 0))
        ttk.Entry(pf, textvariable=self.v["levels"], width=40).grid(row=1, column=2, columnspan=6, sticky="w", pady=(6, 0))
        ttk.Label(pf, text="Room").grid(row=2, column=0, sticky="w")
        ttk.Entry(pf, textvariable=self.v["env"], width=60).grid(row=2, column=1, columnspan=7, sticky="we")
        ttk.Label(pf, text="J4-J6 first (wrist: pose hardly matters), 3 deg; then J1-J3 at 2 deg from HOME (their "
                           "result holds for poses like it). J6 turns the bare flange: see 'moved deg', or tape a flag on it.",
                  foreground="#555").grid(row=3, column=0, columnspan=8, sticky="w", pady=(4, 0))
        r0 += 1
        bf = ttk.Frame(f)
        bf.grid(row=r0, column=0, columnspan=4, sticky="w")
        self.buttons = [ttk.Button(bf, text="1  Check (no motion)", command=self.check),
                        ttk.Button(bf, text="2  Run levels", command=self.run)]
        for i, b in enumerate(self.buttons):
            b.grid(row=0, column=i, padx=(0, 6))
        tk.Button(bf, text="STOP", bg="#c00000", fg="white", font=("Segoe UI", 10, "bold"),
                  command=self.stop).grid(row=0, column=2, padx=12)
        r0 += 1
        cols = ("acc", "moved", "move", "vel", "track", "lag", "errors", "state")
        self.tree = ttk.Treeview(f, columns=cols, show="headings", height=7)
        for c, w, t in zip(cols, (80, 90, 60, 80, 100, 60, 80, 60), ("deg/s²", "moved deg", "move s", "peak deg/s",
                                                                      "tracking deg", "lag ms", "errors", "")):
            self.tree.heading(c, text=t)
            self.tree.column(c, width=w, anchor="e")
        self.tree.grid(row=r0, column=0, columnspan=4, sticky="we", pady=6)
        r0 += 1
        self.log = tk.Text(f, height=12, font=("Consolas", 9))
        self.log.grid(row=r0, column=0, columnspan=4, sticky="nsew")
        f.rowconfigure(r0, weight=1)
        self.status = ttk.Label(f, text="ready")
        self.status.grid(row=r0 + 1, column=0, columnspan=4, sticky="w", pady=(6, 0))
        self._target_changed()
        root.after(100, self._pump)

    def _g(self):
        g = {k: v.get() for k, v in self.v.items()}
        g["ip"] = g["ip"].strip()
        if not 1 <= int(g["joint"]) <= 6 or not 0 < float(g["amp"]) <= 10:
            raise ValueError("Joint 1-6, amplitude in (0, 10] deg")
        return g

    def _target_changed(self):
        self.warn.config(text="REAL ARM -- clear the area, hand on the E-stop; the plate must be fixed"
                         if self.v["target"].get() == "hardware" else "")

    def _joint_changed(self):
        j = self.v["joint"].get()
        self.v["amp"].set(2.0 if j <= 3 else 3.0)

    # -- processes -----------------------------------------------------
    def _spawn(self, argv, then):
        self.buffer = ""
        self.then = then
        self.log.insert("end", "\n> accel_probe.py %s\n" % " ".join(argv))
        self.log.see("end")
        for b in self.buttons:
            b.state(["disabled"])
        flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        self.proc = subprocess.Popen([sys.executable, "-u", PROBE] + argv, cwd=ROOT, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, creationflags=flags)
        threading.Thread(target=self._reader, args=(self.proc,), daemon=True).start()

    def _reader(self, proc):
        for line in proc.stdout:
            self.out.put(line)
        proc.wait()
        self.out.put(None)

    def _pump(self):
        try:
            while True:
                line = self.out.get_nowait()
                if line is None:
                    code = self.proc.returncode if self.proc else None
                    self.proc = None
                    for b in self.buttons:
                        b.state(["!disabled"])
                    then, self.then = self.then, None
                    if then:
                        then(code, last_json(self.buffer))
                    continue
                self.buffer += line
                if not line.lstrip().startswith("{"):
                    self.log.insert("end", line)
                    self.log.see("end")
        except queue.Empty:
            pass
        self.root.after(100, self._pump)

    # -- steps ---------------------------------------------------------
    def check(self, after=None):
        if self.proc is not None:
            return
        try:
            g = self._g()
        except (ValueError, tk.TclError) as e:
            messagebox.showerror("Settings", str(e))
            return

        def done(code, rep):
            ok = code == 0 and rep is not None and rep.get("precheck") == "ok"
            if rep:
                self.status.config(text="pre-check %s: J%d at %.2f deg, tool at %s m" % (
                    "ok" if ok else "FAILED", g["joint"], rep["pose_deg"][g["joint"] - 1], rep.get("tcp_m")))
            else:
                self.status.config(text="check failed (see the log)")
            if not ok and rep:
                messagebox.showwarning("Pre-check", "Not safe to test from this pose:\n\n%s" % rep.get("stopped", ""))
            if ok and after:
                after(g, rep)
        self._spawn(argv_for(g, check_only=True), done)

    def run(self):
        if self.proc is not None:
            return
        try:
            levels = parse_levels(self.v["levels"].get())
        except ValueError as e:
            messagebox.showerror("Levels", str(e))
            return
        self.tree.delete(*self.tree.get_children())
        self.results = []

        def start(g, rep):
            self.pose = rep
            self.queue_levels = list(levels)
            self._next(g)
        self.check(after=start)

    def _next(self, g):
        if not self.queue_levels:
            return self._finish(g, "all levels clean")
        acc = self.queue_levels.pop(0)
        if g["target"] == "hardware":
            if not messagebox.askyesno("Move the REAL ARM?",
                                       "J%d %s%g deg and back at %g deg/s²\n\nArea clear, hand on the E-stop?"
                                       % (g["joint"], g["direction"], g["amp"], acc), icon="warning"):
                return self._finish(g, "not confirmed at %g" % acc)

        def done(code, rep):
            lv = (rep or {}).get("levels") or []
            if not lv:
                return self._finish(g, "no result at %g (%s)" % (acc, (rep or {}).get("stopped", "see the log")))
            row = lv[0]
            bad = rep.get("clean_up_to") is None
            self.results.append(row)
            self.tree.insert("", "end", values=(
                "%g" % acc, row.get("actual_travel_deg", ""), "%.2f" % row.get("move_s", 0), "%.0f" % row.get("peak_vel_deg_s", 0),
                row.get("tracking_after_lag_max_deg", row.get("error", "")), row.get("lag_ms", ""),
                row.get("controller_error_after", ""), "STOP" if bad else "ok"))
            if bad:
                return self._finish(g, rep.get("stopped", "stopped at %g" % acc))
            time.sleep(0.3)
            self._next(g)
        self.status.config(text="J%d at %g deg/s^2 ..." % (g["joint"], acc))
        self._spawn(argv_for(g, level=acc), done)

    def _finish(self, g, why):
        clean = [r["acc"] for r in self.results if r.get("tracking_after_lag_max_deg") is not None
                 and r["tracking_after_lag_max_deg"] <= g["max_error"] and not any(r.get("controller_error_after") or [])]
        best = max(clean) if clean else None
        rep = {"target": g["target"], "joint": g["joint"], "amp_deg": g["amp"] * (1 if g["direction"] == "+" else -1),
               "pose_deg": getattr(self, "pose", {}).get("pose_deg"), "tcp_m": getattr(self, "pose", {}).get("tcp_m"),
               "levels": self.results, "clean_up_to": best, "stopped": why,
               "time": time.strftime("%Y-%m-%d %H:%M:%S")}
        os.makedirs(OUT, exist_ok=True)
        path = os.path.join(OUT, "accel_j%d_%s_%s.json" % (g["joint"], g["target"], time.strftime("%Y%m%d-%H%M%S")))
        with open(path, "w") as fh:
            json.dump(rep, fh, indent=1)
        msg = "J%d clean up to %s deg/s^2 (%s) -- %s" % (g["joint"], best, why, os.path.relpath(path, ROOT))
        self.log.insert("end", "\n== %s\n" % msg)
        self.log.see("end")
        self.status.config(text=msg)

    def stop(self):
        self.queue_levels = []
        msg = []
        try:
            rpc = xmlrpc.client.ServerProxy("http://%s:20003" % self.v["ip"].get().strip())
            msg.append("StopMotion -> %s" % (rpc.StopMotion(),))
            msg.append("ServoMoveEnd -> %s" % (rpc.ServoMoveEnd(),))
        except Exception as e:
            msg.append("controller stop failed: %s" % e)
        if self.proc is not None:
            self.then = None
            self.proc.kill()
            msg.append("probe process ended")
        self.log.insert("end", "\n!! STOP: %s\n" % "; ".join(msg))
        self.log.see("end")
        self.status.config(text="stopped")


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    g = {"target": "hardware", "ip": "192.168.58.2", "joint": 2, "amp": 2.0, "direction": "-", "max_error": 0.5,
         "env": "envs/volvox_lab.json"}
    a = argv_for(g, level=300)
    check("a level runs one probe process: that level, confirmed here, the other way",
          a[:2] == ["--hardware", "--ip"] and a[a.index("--levels") + 1] == "300" and "--yes" in a
          and a[a.index("--direction") + 1] == "-1", a)
    check("Check never moves", "--check-only" in argv_for(g, check_only=True) and "--yes" not in argv_for(g, check_only=True))
    check("levels parse, lowest first", parse_levels("150, 225 300") == [150.0, 225.0, 300.0])
    try:
        parse_levels("300 150")
        check("levels out of order are refused", False)
    except ValueError:
        check("levels out of order are refused", True)
    check("the report is the last JSON line", last_json('x\n{"a": 1}\nclean up\n{"b": 2}\n') == {"b": 2})
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    root = tk.Tk()
    App(root)
    root.mainloop()
