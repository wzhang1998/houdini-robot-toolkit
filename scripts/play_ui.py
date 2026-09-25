"""A small window for fairino_player.py: pick a CSV, set parameters, run a step.

    python scripts/play_ui.py
    python scripts/play_ui.py --self-test

Settings are saved to playback.toml (the same file scripts/play.py reads),
so the menu and this window stay in step. Arguments are built by
play.build_argv(); the player runs as its own process so the window stays
responsive, and its output streams into the log.

Hardware: every step that moves the arm asks for confirmation first. STOP
sends StopMotion and ServoMoveEnd to the controller and ends the player
process. It is a software stop -- the E-stop is the safety device.
"""

import json
import os
import queue
import subprocess
import sys
import threading
import tkinter as tk
import xmlrpc.client
from tkinter import filedialog, messagebox, ttk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import play  # noqa: E402  (build_argv, load, CONFIG, ROOT)

PLAYER = os.path.join(HERE, "fairino_player.py")
MOVING = {"goto-start", "wiggle", "play", "home"}

TEMPLATE = """# Playback settings for scripts/play.py and scripts/play_ui.py.
# play_ui.py rewrites this file when you run a step.

[robot]
target = "{target}"              # "sim" (SimMachine) or "hardware" (the real arm)
ip = "{ip}"
profile = "{profile}"

[clip]
csv = "{csv}"   # absolute, or relative to the toolkit root
record = {record}               # save the actual joints next to the clip
report = {report}               # save the JSON report next to the clip

[motion]
speed = {speed}                 # fraction of the velocity/acceleration envelope; 0.3 default, 1.0 = as designed
rate_hz = {rate_hz}               # ServoJ rate
acc_limit = {acc_limit}             # deg/s^2 cap on the profile's per-joint limits; 0 = the profile's (FR20: J1-3 300, J4-6 600)
move_vel = {move_vel}               # MoveJ % to the clip's first pose. Hardware: 10
confirm = true              # hardware only: ask before moving (keep true)

[wiggle]                    # one joint out and back from where the arm is
joint = {joint}
amp_deg = {amp_deg}
period_s = {period_s}
cycles = {cycles}
"""


def render(cfg):
    """cfg dict -> playback.toml text."""
    r, c, m, w = cfg["robot"], cfg["clip"], cfg["motion"], cfg["wiggle"]
    return TEMPLATE.format(
        target=r["target"], ip=r["ip"], profile=r.get("profile", "fr20"),
        csv=c["csv"].replace("\\", "/"), record=str(bool(c["record"])).lower(),
        report=str(bool(c["report"])).lower(),
        speed=m["speed"], rate_hz=m["rate_hz"], acc_limit=m["acc_limit"], move_vel=m["move_vel"],
        joint=w["joint"], amp_deg=w["amp_deg"], period_s=w["period_s"], cycles=w["cycles"])


def summary(text):
    """One line from the player's JSON report at the end of its output."""
    i = text.rfind("\n{")
    if i < 0:
        i = text.find("{")
    try:
        rep = json.loads(text[i:].strip())
    except Exception:
        return None
    if "check" in rep:
        ch = rep["check"]
        fk = ch.get("fk_vs_urdf", {})
        return "CHECK  %s  errors %s  FK vs URDF %s (%s mm, %s deg)" % (
            rep.get("controller_model"), ch.get("error_code"), fk.get("verdict"),
            fk.get("max_position_mm"), fk.get("max_orientation_deg"))
    cond = rep.get("conditioning", {})
    line = "clip %.1f s -> %.1f s (scale %.2f, speed %g)" % (
        cond.get("source_duration_s", 0), cond.get("played_duration_s", 0),
        cond.get("time_scale", 0), cond.get("speed", 0))
    pb = rep.get("playback")
    if pb:
        line += "  |  %d sends, %d skipped, %.1f Hz, lag %s ms, tracking %s deg, controller error after %s" % (
            pb["sends"], pb["skipped"], pb["effective_send_hz"] or 0, pb.get("best_lag_ms"),
            pb.get("tracking_after_lag_max_deg"), pb.get("controller_error_after"))
    wa = rep.get("wiggle_actual")
    if wa:
        line += "\nwiggle: J%d moved %.2f deg (commanded %+g)" % (wa["joint"], wa["actual_travel_deg"], wa["commanded_deg"])
    if rep.get("recorded"):
        line += "\nrecorded: " + rep["recorded"]
    if "goto_home_off_deg" in rep:
        line += "  |  at HOME (%.2f deg off); path %s" % (rep["goto_home_off_deg"], rep.get("home_path"))
    elif rep.get("home_path") and rep.get("aborted"):
        line += "\nREFUSED -- " + rep["aborted"]
    if "goto_start_off_deg" in rep:
        line += "  |  at start pose (%.2f deg off)" % rep["goto_start_off_deg"]
    return line


class App:
    def __init__(self, root):
        self.root = root
        root.title("FR20 playback")
        self.proc = None
        self.out = queue.Queue()
        self.buffer = ""
        cfg = play.load()
        r, c, m = cfg["robot"], cfg.get("clip", {}), cfg.get("motion", {})
        w = cfg.get("wiggle", {})

        self.v = {
            "target": tk.StringVar(value=r.get("target", "sim")),
            "ip": tk.StringVar(value=r.get("ip", "")),
            "profile": tk.StringVar(value=r.get("profile", "fr20")),
            "csv": tk.StringVar(value=c.get("csv", "")),
            "record": tk.BooleanVar(value=c.get("record", True)),
            "report": tk.BooleanVar(value=c.get("report", True)),
            "speed": tk.DoubleVar(value=m.get("speed", 0.3)),
            "rate_hz": tk.DoubleVar(value=m.get("rate_hz", 125)),
            "acc_limit": tk.DoubleVar(value=m.get("acc_limit", 0)),
            "move_vel": tk.DoubleVar(value=m.get("move_vel", 20)),
            "joint": tk.IntVar(value=w.get("joint", 6)),
            "amp_deg": tk.DoubleVar(value=w.get("amp_deg", 5)),
            "period_s": tk.DoubleVar(value=w.get("period_s", 4)),
            "cycles": tk.IntVar(value=w.get("cycles", 2)),
        }

        f = ttk.Frame(root, padding=10)
        f.grid(sticky="nsew")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        row = 0
        ttk.Label(f, text="Target").grid(row=row, column=0, sticky="w")
        tf = ttk.Frame(f)
        tf.grid(row=row, column=1, columnspan=3, sticky="w")
        ttk.Radiobutton(tf, text="SimMachine", value="sim", variable=self.v["target"],
                        command=self._target_changed).pack(side="left")
        ttk.Radiobutton(tf, text="Hardware (real arm)", value="hardware", variable=self.v["target"],
                        command=self._target_changed).pack(side="left", padx=10)
        self.warn = ttk.Label(tf, text="", foreground="#c00000")
        self.warn.pack(side="left", padx=10)

        row += 1
        ttk.Label(f, text="Controller IP").grid(row=row, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.v["ip"], width=18).grid(row=row, column=1, sticky="w")
        ttk.Label(f, text="Profile").grid(row=row, column=2, sticky="e")
        ttk.Entry(f, textvariable=self.v["profile"], width=8).grid(row=row, column=3, sticky="w")

        row += 1
        ttk.Label(f, text="Clip CSV").grid(row=row, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.v["csv"], width=60).grid(row=row, column=1, columnspan=2, sticky="we")
        ttk.Button(f, text="Browse...", command=self._browse).grid(row=row, column=3, sticky="w")

        row += 1
        mf = ttk.LabelFrame(f, text="Motion", padding=6)
        mf.grid(row=row, column=0, columnspan=4, sticky="we", pady=6)
        for i, (key, label, width) in enumerate((("speed", "Speed (0-1)", 6), ("move_vel", "MoveJ %", 6),
                                                 ("acc_limit", "Acc cap (0 = profile)", 7), ("rate_hz", "Rate Hz", 6))):
            ttk.Label(mf, text=label).grid(row=0, column=2 * i, sticky="e", padx=(8 if i else 0, 2))
            ttk.Entry(mf, textvariable=self.v[key], width=width).grid(row=0, column=2 * i + 1, sticky="w")
        ttk.Checkbutton(mf, text="Record actual joints", variable=self.v["record"]).grid(row=1, column=0, columnspan=3, sticky="w", pady=(6, 0))
        ttk.Checkbutton(mf, text="Save report", variable=self.v["report"]).grid(row=1, column=3, columnspan=3, sticky="w", pady=(6, 0))

        row += 1
        wf = ttk.LabelFrame(f, text="Wiggle (one joint out and back from the current pose)", padding=6)
        wf.grid(row=row, column=0, columnspan=4, sticky="we")
        for i, (key, label) in enumerate((("joint", "Joint"), ("amp_deg", "Amplitude deg"),
                                          ("period_s", "Period s"), ("cycles", "Cycles"))):
            ttk.Label(wf, text=label).grid(row=0, column=2 * i, sticky="e", padx=(8 if i else 0, 2))
            ttk.Entry(wf, textvariable=self.v[key], width=6).grid(row=0, column=2 * i + 1, sticky="w")

        row += 1
        bf = ttk.Frame(f)
        bf.grid(row=row, column=0, columnspan=4, sticky="we", pady=8)
        self.buttons = []
        for step, label in (("check", "1  Check (read-only)"), ("dry-run", "2  Dry run"),
                            ("goto-start", "3  Go to start"), ("wiggle", "4  Wiggle"), ("play", "5  Play clip"),
                            ("home", "6  Go HOME")):
            b = ttk.Button(bf, text=label, command=lambda s=step: self.run(s))
            b.pack(side="left", padx=(0, 6))
            self.buttons.append(b)
        tk.Button(bf, text="STOP", bg="#c00000", fg="white", font=("Segoe UI", 10, "bold"),
                  command=self.stop, width=8).pack(side="right")

        row += 1
        self.log = tk.Text(f, height=18, width=110, font=("Consolas", 9))
        self.log.grid(row=row, column=0, columnspan=4, sticky="nsew")
        f.rowconfigure(row, weight=1)
        f.columnconfigure(1, weight=1)
        self.status = ttk.Label(f, text="ready")
        self.status.grid(row=row + 1, column=0, columnspan=4, sticky="w", pady=(6, 0))

        self._target_changed()
        root.after(100, self._pump)

    # -- settings ------------------------------------------------------
    def cfg(self):
        g = {k: v.get() for k, v in self.v.items()}
        csv = g["csv"].strip()
        # a clip inside the toolkit is saved relative, so playback.toml works
        # on any machine the repo is cloned to
        if os.path.isabs(csv):
            rel = os.path.relpath(csv, play.ROOT)
            if not rel.startswith(".."):
                csv = rel.replace("\\", "/")
        return {"robot": {"target": g["target"], "ip": g["ip"].strip(), "profile": g["profile"].strip()},
                "clip": {"csv": csv, "record": g["record"], "report": g["report"]},
                "motion": {"speed": g["speed"], "rate_hz": g["rate_hz"], "acc_limit": g["acc_limit"],
                           "move_vel": g["move_vel"], "confirm": True},
                "wiggle": {"joint": g["joint"], "amp_deg": g["amp_deg"],
                           "period_s": g["period_s"], "cycles": g["cycles"]}}

    def save(self):
        with open(play.CONFIG, "w", encoding="utf-8", newline="\n") as fh:
            fh.write(render(self.cfg()))

    def _target_changed(self):
        hw = self.v["target"].get() == "hardware"
        self.warn.config(text="REAL ARM -- clear the area, hand on the E-stop" if hw else "")
        if hw and self.v["speed"].get() > 0.3:
            self.v["speed"].set(0.3)
        if hw and self.v["move_vel"].get() > 10:
            self.v["move_vel"].set(10)

    def _browse(self):
        start = os.path.dirname(self.v["csv"].get()) or os.path.join(play.ROOT, "tests", "csv")
        if not os.path.isabs(start):
            start = os.path.join(play.ROOT, start)
        p = filedialog.askopenfilename(title="Clip CSV", initialdir=start,
                                       filetypes=[("CSV", "*.csv"), ("All files", "*.*")])
        if p:
            self.v["csv"].set(p)

    # -- running -------------------------------------------------------
    def run(self, step):
        if self.proc is not None:
            return
        try:
            cfg = self.cfg()
            if not 0 < cfg["motion"]["speed"] <= 1:
                raise ValueError("Speed must be in (0, 1]")
            argv = play.build_argv(cfg, step)
        except Exception as e:
            messagebox.showerror("Settings", str(e))
            return
        if cfg["robot"]["target"] == "hardware" and step in MOVING:
            what = {"goto-start": "MoveJ to the clip's first pose",
                    "home": "MoveJ to HOME (upper arm up, forearm forward, tool down)",
                    "wiggle": "wiggle J%d %+g deg" % (cfg["wiggle"]["joint"], cfg["wiggle"]["amp_deg"]),
                    "play": "play " + os.path.basename(cfg["clip"]["csv"])}[step]
            if not messagebox.askyesno(
                    "Move the REAL ARM?",
                    "%s\n\nController %s\nSpeed %g, MoveJ %g %%\n\n"
                    "Area clear, hand on the E-stop?" % (what, cfg["robot"]["ip"],
                                                         cfg["motion"]["speed"], cfg["motion"]["move_vel"]),
                    icon="warning"):
                return
            argv.append("--yes")      # confirmed here; the player has no console to ask on
        self.save()
        self.buffer = ""
        self.log.insert("end", "\n> fairino_player.py %s\n" % " ".join(argv))
        self.log.see("end")
        for b in self.buttons:
            b.state(["disabled"])
        self.status.config(text="running: " + step)
        flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        self.proc = subprocess.Popen([sys.executable, "-u", PLAYER] + argv, cwd=play.ROOT,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, creationflags=flags)
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
                    self._finished()
                    continue
                self.buffer += line
                self.log.insert("end", line)
                self.log.see("end")
        except queue.Empty:
            pass
        self.root.after(100, self._pump)

    def _finished(self):
        code = self.proc.returncode if self.proc else None
        self.proc = None
        for b in self.buttons:
            b.state(["!disabled"])
        s = summary(self.buffer)
        if s:
            self.log.insert("end", "\n== " + s + "\n")
            self.log.see("end")
        self.status.config(text="done" if code == 0 else "ended with code %s" % code)

    def stop(self):
        ip = self.v["ip"].get().strip()
        msg = []
        try:
            rpc = xmlrpc.client.ServerProxy("http://%s:20003" % ip)
            msg.append("StopMotion -> %s" % (rpc.StopMotion(),))
            msg.append("ServoMoveEnd -> %s" % (rpc.ServoMoveEnd(),))
        except Exception as e:
            msg.append("controller stop failed: %s" % e)
        if self.proc is not None:
            self.proc.kill()
            msg.append("player process ended")
        self.log.insert("end", "\n!! STOP: %s\n" % "; ".join(msg))
        self.log.see("end")
        self.status.config(text="stopped")


def self_test():
    import tempfile
    import tomllib
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    cfg = {"robot": {"target": "hardware", "ip": "192.168.58.2", "profile": "fr20"},
           "clip": {"csv": "D:\\clips\\wave.csv", "record": True, "report": False},
           "motion": {"speed": 0.3, "rate_hz": 125.0, "acc_limit": 300.0, "move_vel": 10.0, "confirm": True},
           "wiggle": {"joint": 6, "amp_deg": 5.0, "period_s": 4.0, "cycles": 2}}
    back = tomllib.loads(render(cfg))
    same = (back["robot"] == cfg["robot"] and back["clip"]["csv"] == "D:/clips/wave.csv"
            and back["clip"]["record"] is True and back["clip"]["report"] is False
            and back["motion"] == cfg["motion"] and back["wiggle"] == cfg["wiggle"])
    check("settings round-trip through playback.toml", same, back if not same else "")
    rep = {"conditioning": {"source_duration_s": 9.7, "played_duration_s": 58.3, "time_scale": 6.0, "speed": 1.0},
           "playback": {"sends": 7288, "skipped": 0, "effective_send_hz": 125.0, "best_lag_ms": 40.0,
                        "tracking_after_lag_max_deg": 0.13, "controller_error_after": [0, 0, 0]}}
    s = summary("> run\n" + json.dumps(rep, indent=1))
    check("summary line from the player's report", s is not None and "0 skipped" in s, s)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    root = tk.Tk()
    App(root)
    if "--smoke" in sys.argv:          # open, lay out, close -- for automated checks
        root.update()
        print("window %dx%d" % (root.winfo_width(), root.winfo_height()))
        root.destroy()
    else:
        root.mainloop()
