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

SimMachine only. The IP is shown and must be SimMachine's: --sim does not
check what is at that address. Hardware runs stay on the command line
(`show_stream.py --hardware`, which asks before moving). STOP sends
/robot/stop: StopMotion, a software stop.
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


def default_sim_ip():
    """playback.toml's IP when its target is SimMachine, else '' (the user
    types it: never guess a robot's address)."""
    target, ip = None, ""
    for line in open(os.path.join(ROOT, "playback.toml")):
        s = line.split("#")[0].strip()
        if s.startswith("target") and "=" in s:
            target = s.split("=", 1)[1].strip().strip('"')
        if s.startswith("ip") and "=" in s:
            ip = s.split("=", 1)[1].strip().strip('"')
    return ip if target == "sim" else ""


REQUIRED = ("pxr", "pythonosc")            # what show_stream needs in this Python: the room (OpenUSD), OSC


def missing_packages(names=REQUIRED):
    """The modules show_stream would fail to import in this Python."""
    import importlib.util
    return [n for n in names if importlib.util.find_spec(n) is None]


def stream_argv(config, ip, minutes, speed, also=(), python=sys.executable):
    """The show_stream.py command: SimMachine, OSC on, the IP given; status
    also to each HOST:PORT in `also` (TouchDesigner)."""
    if not ip:
        raise ValueError("no SimMachine IP")
    argv = [python, STREAM, config, "--sim", "--ip", ip, "--osc", "--minutes", "%g" % minutes,
            "--speed", "%g" % speed]
    for target in also:
        argv += ["--osc-out", target]
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
        self.proc = None
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

    def start(self, ip, minutes, speed, also=()):
        if self.running:
            return
        argv = stream_argv(self.config, ip, minutes, speed, also)
        self.log.put("$ " + " ".join(os.path.basename(a) if a == STREAM else a for a in argv[1:]))
        self.proc = subprocess.Popen(argv, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, text=True, bufsize=1)

        def pump(p):
            for line in p.stdout:
                self.log.put(line.rstrip())
            self.log.put("(show process ended, code %s)" % p.wait())
        threading.Thread(target=pump, args=(self.proc,), daemon=True).start()

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

    def stop(self, wait_s=5.0):
        """/robot/stop, then wait for the process to end (it sends StopMotion)."""
        if not self.running:
            return
        self.send("/robot/stop", 1)
        try:
            self.proc.wait(timeout=wait_s)
        except subprocess.TimeoutExpired:
            self.log.put("the show process did not end after /robot/stop; terminating it")
            self.proc.terminate()

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
    root.title("Show on SimMachine -- %s" % cfg.get("name", os.path.basename(config)))
    pad = {"padx": 8, "pady": 4}

    # --- run: set before Start, locked while it runs -------------------------
    top = ttk.LabelFrame(root, text="Run  (SimMachine only; set before Start)")
    top.pack(fill="x", **pad)
    ip = tk.StringVar(value=default_sim_ip())
    minutes = tk.DoubleVar(value=10.0)
    speed = tk.DoubleVar(value=0.5)
    td_on = tk.BooleanVar(value=True)
    td_target = tk.StringVar(value="127.0.0.1:9002")
    before_start = []

    def field(label, widget, col):
        ttk.Label(top, text=label).grid(row=0, column=col, sticky="e", padx=(8, 2))
        widget.grid(row=0, column=col + 1, sticky="w")
        before_start.append(widget)
    field("SimMachine IP", ttk.Entry(top, textvariable=ip, width=16), 0)
    field("Minutes", ttk.Spinbox(top, from_=0.5, to=240, increment=0.5, textvariable=minutes, width=6), 2)
    field("Speed", ttk.Spinbox(top, from_=0.05, to=1.0, increment=0.05, textvariable=speed, width=5), 4)
    td_check = ttk.Checkbutton(top, text="Status also to TD at", variable=td_on)
    td_check.grid(row=1, column=0, columnspan=2, sticky="w", padx=8)
    td_entry = ttk.Entry(top, textvariable=td_target, width=16)
    td_entry.grid(row=1, column=2, columnspan=2, sticky="w")
    before_start += [td_check, td_entry]

    def start():
        if not ip.get().strip():
            messagebox.showerror("No IP", "Type SimMachine's IP (this window never drives the real arm).")
            return
        if not 0.05 <= speed.get() <= 1.0:
            messagebox.showerror("Speed", "Speed is 0.05 .. 1.0.")
            return
        miss = missing_packages()
        if miss:
            messagebox.showerror("Missing packages", "This Python (%s) has no %s.\n\nRun the window in the toolkit's "
                                 "environment:\nuv sync\nuv run scripts/show_ui.py" % (sys.executable, ", ".join(miss)))
            return
        link.start(ip.get().strip(), minutes.get(), speed.get(),
                   [td_target.get().strip()] if td_on.get() and td_target.get().strip() else [])

    start_b = ttk.Button(top, text="Start", command=start)
    start_b.grid(row=0, column=6, padx=10)
    tk.Button(top, text="STOP", bg="#c0392b", fg="white", width=8, font=("Segoe UI", 10, "bold"),
              command=lambda: threading.Thread(target=link.stop, daemon=True).start()).grid(row=0, column=7, rowspan=2)

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
    link.close()
    fake.shutdown()
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    run_window(os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_CONFIG)
