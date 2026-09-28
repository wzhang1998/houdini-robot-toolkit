"""A small window to run the show on SimMachine and trigger it by hand, the
way TouchDesigner will: start show_stream.py, press sequences, pause,
resume, set the mood and energy, watch the state.

    uv run scripts/show_ui.py [shows/party.json]
    uv run scripts/show_ui.py --self-test

The window starts `show_stream.py <config> --sim --ip <IP> --osc` as its own
process and talks to it over the show's OSC ports (the config's "osc":
commands to listen_port, status back on send_port) -- the same contract TD
uses, so this is also a stand-in for the TD panel.

TouchDesigner can join while it runs: TD sends to the same listen_port
(/robot/trigger, /robot/speed, ...), and the status is also sent to TD's
port ("Status also to", default 127.0.0.1:9002). The window shows what TD
changes, since it reads the show's own status.

Speed is the global show speed, set before Start and fixed for the run
(it is not changed while the arm moves). The window shows the state, the
clip and its time left, what is next, the queue, waiting triggers, and the
ticks skipped so far (a stall shows at once).

Target: SimMachine, or the real FR20. For the real arm the window turns red
and asks more:
- a checklist before anything moves: the work area clear, a hand on the
  E-stop, the controller without alarms (ticked again for every run);
- the speed starts at 0.3 (the test plan goes 0.3, 0.6, then 1.0);
- "Move to start": only the checked MoveJ from where the arm is to the
  start hub, at the Move speed (%) set beside it (slow: 3..30 %), then it
  ends -- so Start can begin from the hub;
- every move is shown and confirmed in a dialog first: show_stream's own
  hardware questions (the route, the plan), passed through as `[ask]`
  lines, answered by the window.
STOP sends /robot/stop and also StopMotion straight to the controller on
its own connection, so it stops the arm during the move to the start hub
too (before the show's OSC is up). A software stop: the E-stop is the
safety.
"""

import json
import os
import queue
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

STREAM = os.path.join(HERE, "show_stream.py")
DEFAULT_CONFIG = os.path.join(ROOT, "shows", "party.json")
MOODS = ["", "float", "glide", "wring", "press", "punch", "slash", "dab", "flick"]   # Laban actions


HARDWARE_MAX = 1.0                         # the show speed the window allows on the real arm (0.3 first, then up)
MOVE_VEL = (3.0, 30.0)                     # MoveJ % to the start hub the window allows
SCAN_SPEED = (0.1, 1.0)                    # the scan's speed, of the built one (shows/<show>.json scan.speed_mps)
MOVE_VEL_DEFAULT = {"sim": 20.0, "hardware": 10.0}
REPORT_WAIT_S = 60.0                       # after STOP, the arm stopped: time for the show to write its report


def default_ip(target="sim"):
    """playback.toml's IP when its target is this one ("sim" / "hardware"),
    else '' (the user types it: never guess a robot's address)."""
    toml_target, ip = None, ""
    path = os.path.join(ROOT, "playback.toml")
    if not os.path.exists(path):
        return ""
    for line in open(path):
        s = line.split("#")[0].strip()
        if s.startswith("target") and "=" in s:
            toml_target = s.split("=", 1)[1].strip().strip('"')
        if s.startswith("ip") and "=" in s:
            ip = s.split("=", 1)[1].strip().strip('"')
    return ip if toml_target == target else ""


REQUIRED = ("pxr", "pythonosc")            # what show_stream needs in this Python: the room (OpenUSD), OSC


def missing_packages(names=REQUIRED):
    """The modules show_stream would fail to import in this Python."""
    import importlib.util
    return [n for n in names if importlib.util.find_spec(n) is None]


def stream_argv(config, ip, minutes, speed, also=(), python=sys.executable, target="sim", move_vel=None,
                goto_start=False, scan_speed=1.0):
    """The show_stream.py command: SimMachine or the real arm, OSC on, the IP
    given; status also to each HOST:PORT in `also` (TouchDesigner).
    goto_start: only the move to the start hub. scan_speed: the scan alone
    slower than built (tuning an exposure). Refuses what the window does
    not allow on the real arm (ValueError)."""
    if not SCAN_SPEED[0] <= scan_speed <= SCAN_SPEED[1]:
        raise ValueError("scan speed %g: %g..%g of the built scan's" % ((scan_speed,) + SCAN_SPEED))
    if target not in ("sim", "hardware"):
        raise ValueError("target is sim or hardware, not %r" % target)
    if not ip:
        raise ValueError("no IP")
    if target == "hardware" and speed > HARDWARE_MAX + 1e-9:
        raise ValueError("speed %.2f: the window allows at most %.1f on the real arm" % (speed, HARDWARE_MAX))
    move_vel = MOVE_VEL_DEFAULT[target] if move_vel is None else move_vel
    if not MOVE_VEL[0] <= move_vel <= MOVE_VEL[1]:
        raise ValueError("move speed %g %%: %g..%g %%" % (move_vel, MOVE_VEL[0], MOVE_VEL[1]))
    argv = [python, STREAM, config, "--" + target, "--ip", ip, "--move-vel", "%g" % move_vel]
    if goto_start:
        return argv + ["--goto-start"]
    argv += ["--osc", "--minutes", "%g" % minutes, "--speed", "%g" % speed, "--scan-speed", "%g" % scan_speed]
    for t in also:
        argv += ["--osc-out", t]
    return argv


class ShowLink:
    """The show process and its OSC link, without any window."""

    def __init__(self, config, listen_port=None, send_host=None, send_port=None):
        from pythonosc import dispatcher, osc_server, udp_client
        self.config = config
        osc = json.load(open(config))["osc"]
        self.status = {"state": "-", "clip": "-", "hub": "-", "progress": 0.0, "scan": -1.0, "speed_now": 0.0,
                       "sequence": "", "next": "", "queue": "", "pending": "", "time_left": 0.0, "fault": "",
                       "skipped": 0, "energy_now": 0.0, "clip_energy": 0.0}
        self.log = queue.Queue()
        self.asks = queue.Queue()                   # show_stream's questions before the real arm moves
        self.proc, self.ip, self.target, self.goto_start = None, None, None, False
        d = dispatcher.Dispatcher()
        for key in ("state", "clip", "hub", "progress", "scan", "speed_now", "sequence", "next", "queue", "pending",
                    "time_left", "fault", "skipped", "energy_now", "clip_energy"):
            d.map("/robot/" + key, self._status_setter(key))
        self.server = osc_server.ThreadingOSCUDPServer(("127.0.0.1", send_port or osc["send_port"]), d)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.client = udp_client.SimpleUDPClient(send_host or "127.0.0.1", listen_port or osc["listen_port"])
        self.last_status = 0.0

    def _status_setter(self, key):
        def set_(addr, *v):
            if v:
                self.status[key] = v[0]
                self.last_status = time.time()
        return set_

    @property
    def running(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self, ip, minutes, speed, also=(), target="sim", move_vel=None, goto_start=False, scan_speed=1.0):
        if self.running:
            return
        argv = stream_argv(self.config, ip, minutes, speed, also, target=target, move_vel=move_vel,
                           goto_start=goto_start, scan_speed=scan_speed)
        self.ip, self.target, self.goto_start = ip, target, goto_start
        self.spawn(argv)

    def spawn(self, argv):
        """Run argv; its output to the log, its [ask] lines to self.asks
        (answered with answer())."""
        from show_stream import ASK
        self.log.put("$ " + " ".join(os.path.basename(a) if a == STREAM else a for a in argv[1:]))
        self.proc = subprocess.Popen(argv, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     stdin=subprocess.PIPE, text=True, bufsize=1)

        def pump(p):
            for line in p.stdout:
                if line.startswith(ASK):
                    self.asks.put(json.loads(line[len(ASK):]))
                else:
                    self.log.put(line.rstrip())
            self.log.put("(show process ended, code %s)" % p.wait())
        threading.Thread(target=pump, args=(self.proc,), daemon=True).start()

    def answer(self, yes):
        """The answer to the question show_stream asked: yes goes on, no ends it."""
        if self.running:
            self.log.put("   -> %s" % ("confirmed" if yes else "not confirmed: nothing moves"))
            self.proc.stdin.write("yes\n" if yes else "no\n")
            self.proc.stdin.flush()

    def send(self, address, *args):
        self.client.send_message(address, list(args) if args else [])

    def trigger(self, name):
        self.send("/robot/trigger", name)

    def pause(self):
        self.send("/robot/pause", 1)

    def resume(self):
        self.send("/robot/resume", 1)

    def reset(self):
        self.send("/robot/reset", 1)

    def mood(self, action):
        self.send("/robot/mood", action or "")

    def energy(self, value):
        self.send("/robot/energy", float(value))

    def stop(self, wait_s=None):
        """/robot/stop, and StopMotion straight to the controller on its own
        connection (the show's OSC is not up during the move to the start
        hub); then wait for the process to end. The arm is stopped at once;
        a show run then writes its report (joints first, then seconds of
        tracking analysis), so it gets up to REPORT_WAIT_S before it is
        ended -- a Move to start writes none and gets 5 s."""
        if not self.running:
            return
        self.send("/robot/stop", 1)
        if self.ip:
            threading.Thread(target=self._stop_motion, args=(self.ip,), daemon=True).start()
        if wait_s is None:
            wait_s = 5.0 if self.goto_start else REPORT_WAIT_S
        if not self.goto_start:
            self.log.put("stopping; waiting for the show to write its report (up to %d s)" % wait_s)
        try:
            self.proc.wait(timeout=wait_s)
        except subprocess.TimeoutExpired:
            self.log.put("the show process did not end after /robot/stop; terminating it")
            self.proc.terminate()

    def _stop_motion(self, ip):
        try:
            import fairino_player as P
            ret = P.Controller(ip).stop()
            self.log.put("StopMotion sent to %s: %s" % (ip, ret))
        except Exception as e:                       # the show's own stop (OSC) still went
            self.log.put("StopMotion to %s failed: %s" % (ip, e))

    def close(self):
        self.stop()
        self.server.shutdown()


STATE_COLOUR = {"IDLE": "#2e8b57", "MOVE": "#2f6db5", "TO_SCAN": "#b8860b", "SCAN": "#b8860b",
                "FROM_SCAN": "#b8860b", "PAUSED": "#d2691e", "FAULT": "#c0392b", "HOLD": "#c0392b"}
STATE_WORDS = {"IDLE": "idling at a hub", "MOVE": "moving between hubs", "TO_SCAN": "going to the scan",
               "SCAN": "scanning", "FROM_SCAN": "back from the scan", "PAUSED": "paused at a hub",
               "FAULT": "stopped: fault", "HOLD": "held after a fault"}


def run_window(config):
    import tkinter as tk
    from tkinter import messagebox, ttk
    link = ShowLink(config)
    cfg = json.load(open(config))
    root = tk.Tk()
    name = cfg.get("name", os.path.basename(config))
    root.title("Show -- %s" % name)
    pad = {"padx": 8, "pady": 4}

    # --- run: set before Start, locked while it runs -------------------------
    target = tk.StringVar(value="sim")
    banner = tk.Label(root, text="", fg="white", bg="#c0392b", font=("Segoe UI", 11, "bold"))
    top = ttk.LabelFrame(root, text="Run  (set before Start)")
    top.pack(fill="x", **pad)
    ip = tk.StringVar(value=default_ip("sim"))
    minutes = tk.DoubleVar(value=10.0)
    speed = tk.DoubleVar(value=0.5)
    move_vel = tk.DoubleVar(value=MOVE_VEL_DEFAULT["sim"])
    scan_speed = tk.DoubleVar(value=1.0)
    td_on = tk.BooleanVar(value=True)
    td_target = tk.StringVar(value="127.0.0.1:9002")
    before_start = []

    tg = ttk.Frame(top)
    tg.grid(row=0, column=0, columnspan=8, sticky="w", padx=6)
    ttk.Label(tg, text="Target").pack(side="left")
    for value, label in (("sim", "SimMachine"), ("hardware", "Real FR20")):
        rb = ttk.Radiobutton(tg, text=label, value=value, variable=target, command=lambda: target_changed())
        rb.pack(side="left", padx=4)
        before_start.append(rb)

    def field(label, widget, row, col):
        ttk.Label(top, text=label).grid(row=row, column=col, sticky="e", padx=(8, 2))
        widget.grid(row=row, column=col + 1, sticky="w")
        before_start.append(widget)
    field("Controller IP", ttk.Entry(top, textvariable=ip, width=16), 1, 0)
    field("Minutes", ttk.Spinbox(top, from_=0.5, to=240, increment=0.5, textvariable=minutes, width=6), 1, 2)
    speed_box = ttk.Spinbox(top, from_=0.05, to=1.0, increment=0.05, textvariable=speed, width=5)
    field("Show speed", speed_box, 1, 4)
    field("Move speed %", ttk.Spinbox(top, from_=MOVE_VEL[0], to=MOVE_VEL[1], increment=1, textvariable=move_vel,
                                      width=5), 2, 4)
    field("Scan speed", ttk.Spinbox(top, from_=SCAN_SPEED[0], to=SCAN_SPEED[1], increment=0.05,
                                    textvariable=scan_speed, width=5), 1, 6)
    td_check = ttk.Checkbutton(top, text="Status also to TD at", variable=td_on)
    td_check.grid(row=2, column=0, columnspan=2, sticky="w", padx=8)
    td_entry = ttk.Entry(top, textvariable=td_target, width=16)
    td_entry.grid(row=2, column=2, columnspan=2, sticky="w")
    before_start += [td_check, td_entry]

    # the real arm: a checklist, ticked again for every run
    checks = ttk.LabelFrame(root, text="Before the real arm moves")
    ticks = [tk.BooleanVar(value=False) for _ in range(3)]
    for k, text_ in enumerate(("The work area is clear: nobody inside the barrier",
                               "A hand is on the E-stop",
                               "The controller shows no alarm (WebApp)")):
        cb = ttk.Checkbutton(checks, text=text_, variable=ticks[k])
        cb.pack(anchor="w", padx=8)
        before_start.append(cb)

    def target_changed():
        hw = target.get() == "hardware"
        ip.set(default_ip(target.get()))
        speed.set(0.3 if hw else 0.5)
        move_vel.set(MOVE_VEL_DEFAULT[target.get()])
        speed_box.configure(to=HARDWARE_MAX if hw else 1.0)
        for v in ticks:
            v.set(False)
        if hw:
            banner.config(text="REAL FR20 -- the arm moves. Hand on the E-stop. STOP is a software stop.")
            banner.pack(fill="x", before=top)
            checks.pack(fill="x", after=top, **pad)
            root.title("Show on the REAL FR20 -- %s" % name)
        else:
            banner.pack_forget()
            checks.pack_forget()
            root.title("Show on SimMachine -- %s" % name)

    def ready(what):
        """Why the run cannot start, or None."""
        hw = target.get() == "hardware"
        if not ip.get().strip():
            return "Type the controller's IP."
        if not 0.05 <= speed.get() <= (HARDWARE_MAX if hw else 1.0):
            return "Show speed is 0.05 .. %.1f%s." % (HARDWARE_MAX if hw else 1.0, " on the real arm" if hw else "")
        if not MOVE_VEL[0] <= move_vel.get() <= MOVE_VEL[1]:
            return "Move speed is %g .. %g %%." % MOVE_VEL
        if not SCAN_SPEED[0] <= scan_speed.get() <= SCAN_SPEED[1]:
            return "Scan speed is %g .. %g of the built scan's (1: as built)." % SCAN_SPEED
        if hw and not all(v.get() for v in ticks):
            return "Tick every line of the checklist before the real arm moves (%s)." % what
        miss = missing_packages()
        if miss:
            return ("This Python (%s) has no %s.\n\nRun the window in the toolkit's environment:\nuv sync\n"
                    "uv run scripts/show_ui.py" % (sys.executable, ", ".join(miss)))
        return None

    def launch(goto_start):
        why = ready("Move to start" if goto_start else "Start")
        if why:
            messagebox.showerror("Not yet", why)
            return
        link.start(ip.get().strip(), minutes.get(), speed.get(),
                   [td_target.get().strip()] if td_on.get() and td_target.get().strip() else [],
                   target=target.get(), move_vel=move_vel.get(), goto_start=goto_start, scan_speed=scan_speed.get())
        for v in ticks:                              # the checklist again next time
            v.set(False)

    goto_b = ttk.Button(top, text="Move to start", command=lambda: launch(True))
    goto_b.grid(row=2, column=6, padx=10)
    start_b = ttk.Button(top, text="Start", command=lambda: launch(False))
    start_b.grid(row=1, column=6, padx=10)
    tk.Button(top, text="STOP", bg="#c0392b", fg="white", width=8, font=("Segoe UI", 10, "bold"),
              command=lambda: threading.Thread(target=link.stop, daemon=True).start()).grid(row=1, column=7, rowspan=2)

    # --- now --------------------------------------------------------------------
    now = ttk.LabelFrame(root, text="Now")
    now.pack(fill="x", **pad)
    state_l = tk.Label(now, text="NOT RUNNING", fg="white", bg="#777777", font=("Segoe UI", 16, "bold"), width=14)
    state_l.grid(row=0, column=0, rowspan=2, padx=6, pady=4, sticky="ns")
    state_w = ttk.Label(now, text="", font=("Segoe UI", 10))
    state_w.grid(row=2, column=0, padx=6)
    clip_l = ttk.Label(now, text="-", font=("Consolas", 14, "bold"))
    clip_l.grid(row=0, column=1, columnspan=3, sticky="w")
    prog = ttk.Progressbar(now, length=380, maximum=1.0)
    prog.grid(row=1, column=1, columnspan=2, sticky="w")
    left_l = ttk.Label(now, text="", font=("Consolas", 11))
    left_l.grid(row=1, column=3, sticky="w", padx=6)
    where_l = ttk.Label(now, text="", font=("Segoe UI", 10))
    where_l.grid(row=2, column=1, columnspan=3, sticky="w")

    # --- next ---------------------------------------------------------------------
    nxt = ttk.LabelFrame(root, text="Next")
    nxt.pack(fill="x", **pad)
    next_l = ttk.Label(nxt, text="-", font=("Consolas", 12, "bold"))
    next_l.grid(row=0, column=0, sticky="w", padx=6)
    queue_l = ttk.Label(nxt, text="", font=("Consolas", 10))
    queue_l.grid(row=1, column=0, sticky="w", padx=6)
    pend_l = ttk.Label(nxt, text="", font=("Segoe UI", 10), foreground="#b8860b")
    pend_l.grid(row=2, column=0, sticky="w", padx=6)

    # --- triggers -------------------------------------------------------------------
    tr = ttk.LabelFrame(root, text="Trigger  (taken when the running clip ends)")
    tr.pack(fill="x", **pad)
    for i, name in enumerate(list(cfg.get("sequences", {})) + (["scan"] if cfg.get("scan") else [])):
        ttk.Button(tr, text=name, width=10, command=lambda n=name: link.trigger(n)).grid(row=0, column=i, padx=3, pady=3)
    ctl = ttk.Frame(tr)
    ctl.grid(row=1, column=0, columnspan=8, sticky="w", pady=4)
    ttk.Button(ctl, text="Pause at the next hub", command=link.pause).pack(side="left", padx=3)
    ttk.Button(ctl, text="Resume", command=link.resume).pack(side="left", padx=3)
    ttk.Button(ctl, text="Reset fault", command=link.reset).pack(side="left", padx=3)
    mood = tk.StringVar(value="")
    ttk.Label(ctl, text="   Mood").pack(side="left")
    box = ttk.Combobox(ctl, values=MOODS, textvariable=mood, width=8, state="readonly")
    box.pack(side="left")
    box.bind("<<ComboboxSelected>>", lambda e: link.mood(mood.get()))
    energy = tk.DoubleVar(value=0.5)
    auto = tk.BooleanVar(value=True)                # the show's energy arc; unticked: the slider's value

    def set_energy(*_):
        link.energy(-1.0 if auto.get() else energy.get())
    ttk.Label(ctl, text="   Energy").pack(side="left")
    ttk.Checkbutton(ctl, text="Auto (the show's arc)", variable=auto, command=set_energy).pack(side="left")
    sc = ttk.Scale(ctl, from_=0.0, to=1.0, variable=energy, length=140)
    sc.pack(side="left")

    def slid(_):
        auto.set(False)                              # moving the slider takes over from the arc
        set_energy()
    sc.bind("<ButtonRelease-1>", slid)

    # --- health and log ----------------------------------------------------------------
    health = ttk.Label(root, text="", font=("Segoe UI", 9))
    health.pack(fill="x", padx=10)
    logf = ttk.LabelFrame(root, text="Show process")
    logf.pack(fill="both", expand=True, **pad)
    text = tk.Text(logf, height=12, width=100, font=("Consolas", 9))
    text.pack(fill="both", expand=True)

    def tick():
        s = link.status
        running = link.running
        for w in before_start:
            w.configure(state="disabled" if running else "normal")
        start_b.configure(state="disabled" if running else "normal")
        goto_b.configure(state="disabled" if running else "normal")
        try:
            question = link.asks.get_nowait()          # show_stream asks before the real arm moves
        except queue.Empty:
            question = None
        if question is not None:
            yes = messagebox.askyesno("Confirm -- the REAL arm will move", question + "\n\nGo on?", icon="warning",
                                      default="no")
            link.answer(yes)
        age = time.time() - link.last_status if link.last_status else None
        live = running and age is not None and age < 1.0
        state = str(s["state"]) if live else ("STARTING" if running else "NOT RUNNING")
        state_l.config(text=state, bg=STATE_COLOUR.get(state, "#777777"))
        state_w.config(text=STATE_WORDS.get(state, "moving to the start hub, loading" if running else ""))
        if live:
            clip_l.config(text=str(s["clip"]))
            prog["value"] = float(s.get("progress") or 0.0)
            left_l.config(text="%.1f s left" % float(s.get("time_left") or 0.0))
            seq = s.get("sequence") or ""
            where_l.config(text="hub %s%s%s" % (s["hub"], ("   |   sequence: %s" % seq) if seq else "",
                                                   ("   |   FAULT: %s" % s["fault"]) if s.get("fault") else ""))
            next_l.config(text="next:  " + str(s.get("next") or "-"))
            q = [x for x in str(s.get("queue") or "").split(" | ") if x]
            queue_l.config(text=("then:  " + "  >  ".join(q[1:])) if len(q) > 1 else "")
            pend = [x for x in str(s.get("pending") or "").split(",") if x]
            pend_l.config(text=("waiting triggers:  " + ", ".join(pend)) if pend else "")
        else:
            for w in (clip_l, left_l, where_l, queue_l, pend_l):
                w.config(text="")
            next_l.config(text="-")
            prog["value"] = 0.0
        skipped = int(s.get("skipped") or 0)
        health.config(text=("speed %.2f   |   energy wanted %.2f, this clip %.2f   |   skipped ticks %d%s   |   "
                            "last status %.1f s ago"
                            % (float(s.get("speed_now") or 0.0), float(s.get("energy_now") or 0.0),
                               float(s.get("clip_energy") or 0.0), skipped, "  (a stall!)" if skipped else "", age))
                      if live else "", foreground="#c0392b" if skipped else "#333333")
        while True:
            try:
                line = link.log.get_nowait()
            except queue.Empty:
                break
            text.insert("end", line + "\n")
            text.see("end")
        root.after(100, tick)

    def on_close():
        if link.running and not messagebox.askyesno("Stop the show?", "The show is running. Stop it and close?"):
            return
        link.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    tick()
    root.mainloop()


def self_test():
    """The OSC mapping and the command, with a fake show on spare ports."""
    from pythonosc import dispatcher, osc_server, udp_client
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    check("the packages show_stream needs are in this Python", not missing_packages(), missing_packages())
    check("a missing package is named", missing_packages(("pythonosc", "no_such_module_x")) == ["no_such_module_x"])
    got = []
    d = dispatcher.Dispatcher()
    d.set_default_handler(lambda addr, *v: got.append((addr, v)))
    fake = osc_server.ThreadingOSCUDPServer(("127.0.0.1", 19100), d)          # the show's listen port
    threading.Thread(target=fake.serve_forever, daemon=True).start()
    link = ShowLink(DEFAULT_CONFIG, listen_port=19100, send_port=19101)
    link.trigger("greet")
    link.pause()
    link.resume()
    link.mood("punch")
    link.energy(0.8)
    time.sleep(0.3)
    addrs = [a for a, _ in got]
    check("commands reach the show's port as its OSC contract",
          addrs == ["/robot/trigger", "/robot/pause", "/robot/resume", "/robot/mood", "/robot/energy"]
          and got[0][1] == ("greet",) and abs(got[4][1][0] - 0.8) < 1e-6, got)
    out = udp_client.SimpleUDPClient("127.0.0.1", 19101)                   # the show's status out
    out.send_message("/robot/state", "IDLE")
    out.send_message("/robot/clip", "rest_02_punch")
    out.send_message("/robot/progress", 0.25)
    out.send_message("/robot/next", "greet: a clip at greet")
    out.send_message("/robot/skipped", 3)
    time.sleep(0.3)
    check("status from the show is shown (state, clip, progress, next, skipped)",
          link.status["state"] == "IDLE" and link.status["clip"] == "rest_02_punch"
          and abs(link.status["progress"] - 0.25) < 1e-6 and link.status["next"] == "greet: a clip at greet"
          and link.status["skipped"] == 3, link.status)
    argv = stream_argv(DEFAULT_CONFIG, "192.168.116.128", 5, 0.3, ["127.0.0.1:9002"])
    check("the command is SimMachine, OSC, the IP given, the speed set at start, status also to TD",
          "--sim" in argv and "--osc" in argv and "--hardware" not in argv
          and argv[argv.index("--ip") + 1] == "192.168.116.128" and argv[argv.index("--speed") + 1] == "0.3"
          and argv[argv.index("--osc-out") + 1] == "127.0.0.1:9002", argv)
    try:
        stream_argv(DEFAULT_CONFIG, "", 5, 0.5)
        check("no IP is refused", False)
    except ValueError:
        check("no IP is refused", True)
    hw = stream_argv(DEFAULT_CONFIG, "10.0.0.9", 5, 0.3, target="hardware")
    check("the real arm: --hardware (never --sim), its slow move to the start hub (10 %)",
          "--hardware" in hw and "--sim" not in hw and hw[hw.index("--move-vel") + 1] == "10", hw)
    sc = stream_argv(DEFAULT_CONFIG, "10.0.0.9", 5, 0.3, target="hardware", scan_speed=0.4)
    check("the scan speed goes to the stream (the scan only, for tuning an exposure)",
          sc[sc.index("--scan-speed") + 1] == "0.4", sc)
    try:
        stream_argv(DEFAULT_CONFIG, "10.0.0.9", 5, 0.3, scan_speed=1.2)
        check("a scan faster than built is refused", False)
    except ValueError:
        check("a scan faster than built is refused", True)
    go = stream_argv(DEFAULT_CONFIG, "10.0.0.9", 5, 0.3, target="hardware", move_vel=5, goto_start=True)
    check("Move to start: only the move, at the speed set, no show",
          "--goto-start" in go and "--osc" not in go and go[go.index("--move-vel") + 1] == "5", go)
    for show_speed, vel, label in ((1.2, None, "a show speed over %.1f on the real arm is refused" % HARDWARE_MAX),
                                   (0.3, 50.0, "a move speed over %g %% is refused" % MOVE_VEL[1])):
        try:
            stream_argv(DEFAULT_CONFIG, "10.0.0.9", 5, show_speed, target="hardware", move_vel=vel)
            check(label, False)
        except ValueError:
            check(label, True)
    # a question from the show process reaches the window, the answer goes back
    from show_stream import ASK
    child = ("import json, sys; print(%r + json.dumps('MoveJ to rest\\nat 10 %%'), flush=True); "
             "print('got ' + sys.stdin.readline().strip(), flush=True)" % ASK)
    link.spawn([sys.executable, "-c", child])
    q = None
    try:
        q = link.asks.get(timeout=10)
    except queue.Empty:
        pass
    link.answer(True)
    link.proc.wait(timeout=10)
    time.sleep(0.2)
    lines = []
    while not link.log.empty():
        lines.append(link.log.get())
    check("show_stream's question reaches the window whole, and the yes goes back",
          q == "MoveJ to rest\nat 10 %" and "got yes" in lines, (q, lines))
    link.close()
    fake.shutdown()
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    run_window(os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_CONFIG)
