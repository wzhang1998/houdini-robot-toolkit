"""A small window to run the show on SimMachine and trigger it by hand, the
way TouchDesigner will: start show_stream.py, press sequences, pause,
resume, set the mood and energy, watch the state.

    python scripts/show_ui.py [shows/party.json]
    python scripts/show_ui.py --self-test

The window starts `show_stream.py <config> --sim --ip <IP> --osc` as its own
process and talks to it over the show's OSC ports (the config's "osc":
commands to listen_port, status back on send_port) -- the same contract TD
uses, so this is also a stand-in for the TD panel.

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


def stream_argv(config, ip, minutes, speed, python=sys.executable):
    """The show_stream.py command: SimMachine, OSC on, the IP given."""
    if not ip:
        raise ValueError("no SimMachine IP")
    return [python, STREAM, config, "--sim", "--ip", ip, "--osc", "--minutes", "%g" % minutes,
            "--speed", "%g" % speed]


class ShowLink:
    """The show process and its OSC link, without any window."""

    def __init__(self, config, listen_port=None, send_host=None, send_port=None):
        from pythonosc import dispatcher, osc_server, udp_client
        self.config = config
        osc = json.load(open(config))["osc"]
        self.status = {"state": "-", "clip": "-", "hub": "-", "progress": 0.0, "scan": -1.0}
        self.log = queue.Queue()
        self.proc = None
        d = dispatcher.Dispatcher()
        for key in ("state", "clip", "hub", "progress", "scan"):
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

    def start(self, ip, minutes, speed):
        if self.running:
            return
        argv = stream_argv(self.config, ip, minutes, speed)
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


def run_window(config):
    import tkinter as tk
    from tkinter import messagebox, ttk
    link = ShowLink(config)
    cfg = json.load(open(config))
    root = tk.Tk()
    root.title("Show on SimMachine -- %s" % cfg.get("name", os.path.basename(config)))
    pad = {"padx": 6, "pady": 3}

    top = ttk.LabelFrame(root, text="Run (SimMachine only)")
    top.pack(fill="x", **pad)
    ip = tk.StringVar(value=default_sim_ip())
    minutes = tk.DoubleVar(value=10.0)
    speed = tk.DoubleVar(value=1.0)
    ttk.Label(top, text="SimMachine IP").grid(row=0, column=0, sticky="w")
    ttk.Entry(top, textvariable=ip, width=16).grid(row=0, column=1, sticky="w")
    ttk.Label(top, text="Minutes").grid(row=0, column=2, sticky="e")
    ttk.Spinbox(top, from_=0.5, to=240, increment=0.5, textvariable=minutes, width=6).grid(row=0, column=3)
    ttk.Label(top, text="Speed").grid(row=0, column=4, sticky="e")
    ttk.Spinbox(top, from_=0.1, to=1.0, increment=0.1, textvariable=speed, width=5).grid(row=0, column=5)

    def start():
        if not ip.get().strip():
            messagebox.showerror("No IP", "Type SimMachine's IP (this window never drives the real arm).")
            return
        link.start(ip.get().strip(), minutes.get(), speed.get())

    ttk.Button(top, text="Start", command=start).grid(row=0, column=6, padx=6)
    tk.Button(top, text="STOP", bg="#c0392b", fg="white", width=8, command=lambda: threading.Thread(
        target=link.stop, daemon=True).start()).grid(row=0, column=7, padx=6)

    st = ttk.LabelFrame(root, text="Status (from the show, over OSC)")
    st.pack(fill="x", **pad)
    labels = {}
    for i, key in enumerate(("state", "clip", "hub")):
        ttk.Label(st, text=key.capitalize()).grid(row=0, column=2 * i, sticky="w")
        labels[key] = ttk.Label(st, text="-", width=22, font=("Consolas", 10, "bold"))
        labels[key].grid(row=0, column=2 * i + 1, sticky="w")
    prog = ttk.Progressbar(st, length=420, maximum=1.0)
    prog.grid(row=1, column=0, columnspan=6, sticky="we", pady=4)
    alive = ttk.Label(st, text="")
    alive.grid(row=2, column=0, columnspan=6, sticky="w")

    tr = ttk.LabelFrame(root, text="Triggers (taken at the end of the running clip)")
    tr.pack(fill="x", **pad)
    for i, name in enumerate(list(cfg.get("sequences", {})) + (["scan"] if cfg.get("scan") else [])):
        ttk.Button(tr, text=name, command=lambda n=name: link.trigger(n)).grid(row=0, column=i, padx=3)
    ctl = ttk.Frame(tr)
    ctl.grid(row=1, column=0, columnspan=8, sticky="w", pady=4)
    ttk.Button(ctl, text="Pause (at a hub)", command=link.pause).pack(side="left", padx=3)
    ttk.Button(ctl, text="Resume", command=link.resume).pack(side="left", padx=3)
    ttk.Button(ctl, text="Reset fault", command=link.reset).pack(side="left", padx=3)
    mood = tk.StringVar(value="")
    ttk.Label(ctl, text="  Mood").pack(side="left")
    box = ttk.Combobox(ctl, values=MOODS, textvariable=mood, width=8, state="readonly")
    box.pack(side="left")
    box.bind("<<ComboboxSelected>>", lambda e: link.mood(mood.get()))
    energy = tk.DoubleVar(value=0.5)
    ttk.Label(ctl, text="  Energy").pack(side="left")
    sc = ttk.Scale(ctl, from_=0.0, to=1.0, variable=energy, length=140)
    sc.pack(side="left")
    sc.bind("<ButtonRelease-1>", lambda e: link.energy(energy.get()))

    logf = ttk.LabelFrame(root, text="Show process")
    logf.pack(fill="both", expand=True, **pad)
    text = tk.Text(logf, height=16, width=100, font=("Consolas", 9))
    text.pack(fill="both", expand=True)

    def tick():
        s = link.status
        for key in ("state", "clip", "hub"):
            labels[key].config(text=str(s[key]))
        prog["value"] = float(s.get("progress") or 0.0)
        age = time.time() - link.last_status if link.last_status else None
        alive.config(text=("running" if link.running else "not running") +
                     ("" if age is None else ", last status %.1f s ago" % age))
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
    time.sleep(0.3)
    check("status from the show is shown", link.status["state"] == "IDLE" and link.status["clip"] == "rest_02_punch"
          and abs(link.status["progress"] - 0.25) < 1e-6, link.status)
    argv = stream_argv(DEFAULT_CONFIG, "192.168.116.128", 5, 0.5)
    check("the command is SimMachine, OSC, the IP given", "--sim" in argv and "--osc" in argv and "--hardware" not in argv
          and argv[argv.index("--ip") + 1] == "192.168.116.128", argv)
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
