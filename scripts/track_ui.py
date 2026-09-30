"""The window for the interactive mode (track_mode.py), on SimMachine or the
real arm -- apart from the party show's show_ui.

    python scripts/track_ui.py
    python scripts/track_ui.py --self-test

Start: the checked MoveJ to the start pose, then the v9 library's idle clips
at random (facing the guests less than the show does); somebody on the spot
in front of greet (or waving near the glass) and the arm comes to greet and
turns to them. End: the people let go, the clip finishes at a hub, and the
arm goes back to the start pose. STOP: a software stop now (StopMotion
straight to the controller too). The people come from TouchDesigner's
people_track (OSC 9011, Track > Active on).

The real arm: the window turns red; a checklist must be ticked for every
run; the clips start at speed 0.3 and the interactive mode at 0.2 of the
joints' limits; every move (to the start pose, the plan, back) is shown and
confirmed in a dialog first. Closing the window (or it dying) ends the
input track_mode reads, which stops it.

Run it with the Python that has numpy, ruckig, python-osc and OpenUSD (the
system one here; `uv run` has no numpy).
"""

import json
import os
import queue
import subprocess
import sys
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

MODE = os.path.join(HERE, "track_mode.py")
REQUIRED = ("numpy", "ruckig", "pythonosc", "pxr")
LIMITS = {"speed": (0.05, 1.0), "share": (0.05, 0.35), "move_vel": (3.0, 30.0)}
DEFAULTS = {"sim": {"speed": 1.0, "share": 0.35, "move_vel": 20.0},
            "hardware": {"speed": 0.3, "share": 0.2, "move_vel": 10.0}}
from fairino_player import SIM_NET  # noqa: E402  (SimMachine, VMware NAT: one place)


def missing_packages(names=REQUIRED):
    import importlib.util
    return [n for n in names if importlib.util.find_spec(n) is None]


def mode_argv(target, ip, speed, share, move_vel, port=9011, python=sys.executable):
    """track_mode.py's command; ValueError for what the window does not allow."""
    if target not in ("sim", "hardware"):
        raise ValueError("target is sim or hardware, not %r" % target)
    ip = (ip or "").strip()
    if not ip:
        raise ValueError("no IP")
    if target == "sim" and not ip.startswith(SIM_NET):
        raise ValueError("SimMachine is on %sx; %s looks like another controller" % (SIM_NET, ip))
    if target == "hardware" and ip.startswith(SIM_NET):
        raise ValueError("%s is SimMachine's network: pick SimMachine" % ip)
    for k, v in (("speed", speed), ("share", share), ("move_vel", move_vel)):
        lo, hi = LIMITS[k]
        if not lo <= v <= hi:
            raise ValueError("%s %g: %g..%g" % (k, v, lo, hi))
    return [python, MODE, "--" + target, "--ip", ip, "--speed", "%g" % speed, "--engage-share", "%g" % share,
            "--move-vel", "%g" % move_vel, "--osc-in", str(int(port)), "--stdin-control"]


class ModeLink:
    """track_mode's process: its output to log, [ask] lines to asks, [status] to status; end / stop / answers
    to its stdin."""

    def __init__(self):
        self.proc, self.ip = None, None
        self.log, self.asks = queue.Queue(), queue.Queue()
        self.status = {}

    @property
    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, argv, ip):
        from show_stream import ASK
        if self.running:
            return
        self.ip = ip
        self.log.put("$ " + " ".join(os.path.basename(a) if a == MODE else a for a in argv[1:]))
        self.proc = subprocess.Popen(argv, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     stdin=subprocess.PIPE, text=True, bufsize=1)

        def pump(p):
            for line in p.stdout:
                if line.startswith(ASK):
                    self.asks.put(json.loads(line[len(ASK):]))
                elif line.startswith("[status] "):
                    try:
                        self.status = json.loads(line[len("[status] "):])
                    except ValueError:
                        pass
                else:
                    self.log.put(line.rstrip())
            self.log.put("(track_mode ended, code %s)" % p.wait())
        threading.Thread(target=pump, args=(self.proc,), daemon=True).start()

    def _send(self, line):
        if self.running:
            try:
                self.proc.stdin.write(line + "\n")
                self.proc.stdin.flush()
            except OSError:
                pass

    def answer(self, yes):
        self.log.put("   -> %s" % ("confirmed" if yes else "not confirmed: nothing moves"))
        self._send("yes" if yes else "no")

    def end(self):
        self.log.put("END: letting the people go, finishing at a hub, then back to the start pose")
        self._send("end")

    def stop(self):
        """stop to the mode, and StopMotion straight to the controller (the mode may be in a MoveJ)."""
        if not self.running:
            return
        self._send("stop")
        if self.ip:
            threading.Thread(target=self._stop_motion, args=(self.ip,), daemon=True).start()

    def _stop_motion(self, ip):
        try:
            import fairino_player as P
            self.log.put("StopMotion sent to %s: %s" % (ip, P.Controller(ip, timeout_s=P.RPC_TIMEOUT_S).stop()))
        except Exception as e:
            self.log.put("StopMotion to %s failed: %s" % (ip, e))

    def close(self):
        self.stop()
        if self.proc is not None:
            try:
                self.proc.stdin.close()                   # its input ends: track_mode stops
            except OSError:
                pass


def run_window():
    import tkinter as tk
    from tkinter import messagebox, ttk
    from show_ui import default_ip
    link = ModeLink()
    root = tk.Tk()
    root.title("Interactive mode (track_mode)")
    target = tk.StringVar(value="sim")
    ip = tk.StringVar(value=default_ip("sim") or "192.168.116.128")
    speed = tk.DoubleVar(value=DEFAULTS["sim"]["speed"])
    share = tk.DoubleVar(value=DEFAULTS["sim"]["share"])
    move_vel = tk.DoubleVar(value=DEFAULTS["sim"]["move_vel"])
    port = tk.IntVar(value=9011)
    checked = tk.BooleanVar(value=False)
    frm = ttk.Frame(root, padding=10)
    frm.grid(sticky="nsew")
    banner = tk.Label(frm, text="SimMachine", bg="#2f6db5", fg="white", font=("Segoe UI", 12, "bold"))
    banner.grid(row=0, column=0, columnspan=4, sticky="ew", pady=(0, 8))
    ttk.Radiobutton(frm, text="SimMachine", variable=target, value="sim").grid(row=1, column=0, sticky="w")
    ttk.Radiobutton(frm, text="Real arm (FR20)", variable=target, value="hardware").grid(row=1, column=1, sticky="w")
    for r, (label, var) in enumerate((("IP", ip), ("Clip speed", speed), ("Interactive share of limits", share),
                                      ("MoveJ %", move_vel), ("People OSC port", port)), start=2):
        ttk.Label(frm, text=label).grid(row=r, column=0, sticky="w")
        ttk.Entry(frm, textvariable=var, width=18).grid(row=r, column=1, sticky="w")
    chk = ttk.Checkbutton(frm, text="Area clear, a hand on the E-stop, no controller alarms", variable=checked)
    chk.grid(row=7, column=0, columnspan=3, sticky="w", pady=4)
    status = tk.StringVar(value="not running")
    tk.Label(frm, textvariable=status, font=("Consolas", 10), justify="left", anchor="w").grid(
        row=8, column=0, columnspan=4, sticky="ew", pady=4)
    btns = ttk.Frame(frm)
    btns.grid(row=9, column=0, columnspan=4, sticky="w", pady=4)
    log = tk.Text(frm, width=100, height=16, font=("Consolas", 9))
    log.grid(row=10, column=0, columnspan=4, sticky="nsew")
    miss = missing_packages()
    if miss:
        log.insert("end", "This Python (%s) lacks %s: track_mode would fail. Run the window with the Python "
                          "that has them.\n" % (sys.executable, ", ".join(miss)))

    def target_changed(*_):
        t = target.get()
        d = DEFAULTS[t]
        speed.set(d["speed"])
        share.set(d["share"])
        move_vel.set(d["move_vel"])
        ip.set(default_ip(t) or ("192.168.116.128" if t == "sim" else ""))
        checked.set(False)
        banner.config(text="REAL ARM -- every move is confirmed" if t == "hardware" else "SimMachine",
                      bg="#c0392b" if t == "hardware" else "#2f6db5")
    target.trace_add("write", target_changed)

    def start():
        if link.running:
            return
        t = target.get()
        if t == "hardware" and not checked.get():
            messagebox.showwarning("Real arm", "Tick the checklist first (every run).")
            return
        try:
            argv = mode_argv(t, ip.get(), float(speed.get()), float(share.get()), float(move_vel.get()),
                             int(port.get()))
        except (ValueError, tk.TclError) as e:
            messagebox.showerror("Not started", str(e))
            return
        checked.set(False)                               # ticked again for the next run
        link.start(argv, ip.get().strip())

    ttk.Button(btns, text="Start", command=start).grid(row=0, column=0, padx=4)
    ttk.Button(btns, text="End (back to start pose)", command=link.end).grid(row=0, column=1, padx=4)
    tk.Button(btns, text="STOP", command=link.stop, bg="#c0392b", fg="white", width=10,
              font=("Segoe UI", 10, "bold")).grid(row=0, column=2, padx=12)

    def tick():
        while not link.log.empty():
            log.insert("end", link.log.get() + "\n")
            log.see("end")
        while not link.asks.empty():
            text = link.asks.get()
            link.answer(messagebox.askyesno("Confirm the move", text + "\n\nGo on?"))
        s = link.status
        if link.running and s:
            status.set("%s  %-7s  clip %-24s hub %-6s  interactive %-6s who %s  people %s  engaged %s  "
                       "refused %s  frames %s  slips %s%s" % (
                           "t %6.1f" % s.get("t", 0), s.get("mode"), s.get("clip"), s.get("hub"), s.get("engage"),
                           s.get("who"), s.get("people"), s.get("engaged"), s.get("refused"), s.get("frames"),
                           s.get("slips"), "  ENDING" if s.get("ending") else ""))
        elif not link.running:
            status.set("not running")
        root.after(100, tick)

    def on_close():
        if link.running and not messagebox.askyesno("Close", "The mode is running: STOP it and close?"):
            return
        link.close()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.bind("<F12>", lambda e: link.stop())
    tick()
    root.mainloop()


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    a = mode_argv("sim", "192.168.116.128", 1.0, 0.35, 20.0)
    check("SimMachine: the mode's command; its stdin carries end / stop / answers, its end stops it",
          a[1:4] == [MODE, "--sim", "--ip"] and "--engage-share" in a and "--stdin-control" in a, a[1:])

    def refused(*args):
        try:
            mode_argv(*args)
            return False
        except ValueError:
            return True
    check("refused: SimMachine on another network (the real arm's IP by mistake)",
          refused("sim", "192.168.58.2", 1.0, 0.35, 20.0))
    check("refused: the real arm on SimMachine's network", refused("hardware", "192.168.116.128", 0.3, 0.2, 10.0))
    check("refused: no IP; a speed, a share or a MoveJ % out of range",
          refused("hardware", "", 0.3, 0.2, 10.0) and refused("sim", "192.168.116.128", 1.5, 0.35, 20.0)
          and refused("sim", "192.168.116.128", 1.0, 0.5, 20.0) and refused("hardware", "10.0.0.2", 0.3, 0.2, 50.0))
    check("the real arm's defaults: clips 0.3, the interactive mode 0.2, MoveJ 10 %",
          DEFAULTS["hardware"] == {"speed": 0.3, "share": 0.2, "move_vel": 10.0})
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    run_window()
