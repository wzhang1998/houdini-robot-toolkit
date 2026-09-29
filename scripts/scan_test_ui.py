"""The scan, step by step, in a window: for lining the LED strip up with the
real frame and trying exposures on the paper.

    uv run scripts/scan_test_ui.py [shows/party.json]
    uv run scripts/scan_test_ui.py --self-test

1. Move to start pos   the checked MoveJ to the scan's home hub (as the show
                       window's "Move to start"), then it ends.
2. Start               show_stream.py --scan-test: one ServoJ stream that
                       holds the arm still between the steps below.
3. To scan start       to_scan: the arm stops at the scan's first frame --
                       look at the strip against the frame and the paper.
   Back                to_scan in reverse, when it is not right (move the
                       canvas in the show config, rebuild, try again).
4. Scan                the scan at `Scan speed` of its built speed (0.1 of
                       0.2 m/s is 0.02 m/s: an exposure); TouchDesigner
                       lights the strip from the status, as in the show.
5. Return              from_scan: back to the start pos. Then Scan again.
6. Finish              back to the start pos the way it came; the stream
                       ends there (it writes its report).
Only the steps allowed where the arm is are enabled (the stream says which).
STOP (as the show window's): /robot/stop and StopMotion straight to the
controller -- a software stop; the E-stop is the safety. After a STOP, Move
to start pos brings it back by the checked route.

Real FR20: the window turns red, the checklist before anything moves, every
move confirmed in a dialog first (show_stream's own questions).
"""

import json
import os
import queue
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import show_ui as UI  # noqa: E402

STEP_BUTTONS = (("to_scan", "3  To scan start"), ("back", "Back to start pos"), ("scan", "4  Scan"),
                ("return", "5  Return"))


def step_command(name, scan_speed):
    """The /robot/trigger text of a step ("scan" carries its speed)."""
    return "scan %g" % scan_speed if name == "scan" else name


def run_window(config):
    import tkinter as tk
    from tkinter import messagebox, ttk
    link = UI.ShowLink(config)
    cfg = json.load(open(config))
    built_mps = float(cfg.get("scan", {}).get("speed_mps", 0.2))
    root = tk.Tk()
    root.title("Scan test -- %s" % cfg.get("name", os.path.basename(config)))
    pad = {"padx": 8, "pady": 4}

    target = tk.StringVar(value="sim")
    ip = tk.StringVar(value=UI.default_ip("sim"))
    speed = tk.DoubleVar(value=0.5)
    move_vel = tk.DoubleVar(value=UI.MOVE_VEL_DEFAULT["sim"])
    scan_speed = tk.DoubleVar(value=0.25)
    td_on = tk.BooleanVar(value=True)
    td_target = tk.StringVar(value="127.0.0.1:9002")
    before_start = []

    banner = tk.Label(root, text="", fg="white", bg="#c0392b", font=("Segoe UI", 11, "bold"))
    top = ttk.LabelFrame(root, text="Run  (set before Start)")
    top.pack(fill="x", **pad)
    tg = ttk.Frame(top)
    tg.grid(row=0, column=0, columnspan=6, sticky="w", padx=6)
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
    speed_box = ttk.Spinbox(top, from_=0.05, to=1.0, increment=0.05, textvariable=speed, width=5)
    field("Move speed (of built)", speed_box, 1, 2)
    field("MoveJ %", ttk.Spinbox(top, from_=UI.MOVE_VEL[0], to=UI.MOVE_VEL[1], increment=1, textvariable=move_vel,
                                 width=5), 2, 2)
    td_check = ttk.Checkbutton(top, text="Status also to TD at", variable=td_on)
    td_check.grid(row=2, column=0, sticky="w", padx=8)
    td_entry = ttk.Entry(top, textvariable=td_target, width=16)
    td_entry.grid(row=2, column=1, sticky="w")
    before_start += [td_check, td_entry]
    choices = {}
    show_pick = tk.StringVar()

    def refresh_shows():
        choices.clear()
        for c in UI.show_choices():
            choices[c["label"]] = c
        show_box["values"] = list(choices)
        show_pick.set(next((l for l, c in choices.items()
                            if os.path.abspath(c["config"]) == os.path.abspath(link.config)), ""))

    def show_changed(*_):
        c = choices.get(show_pick.get())
        if c:
            link.config = c["config"]
    ttk.Label(top, text="Show").grid(row=3, column=0, sticky="e", padx=(8, 2))
    show_box = ttk.Combobox(top, textvariable=show_pick, state="readonly", width=60, postcommand=refresh_shows)
    show_box.grid(row=3, column=1, columnspan=4, sticky="w", pady=(2, 4))
    show_box.bind("<<ComboboxSelected>>", show_changed)
    before_start.append(show_box)
    refresh_shows()

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
        ip.set(UI.default_ip(target.get()))
        speed.set(0.3 if hw else 0.5)
        move_vel.set(UI.MOVE_VEL_DEFAULT[target.get()])
        for v in ticks:
            v.set(False)
        if hw:
            banner.config(text="REAL FR20 -- the arm moves. Hand on the E-stop. STOP is a software stop.")
            banner.pack(fill="x", before=top)
            checks.pack(fill="x", after=top, **pad)
        else:
            banner.pack_forget()
            checks.pack_forget()

    def ready(what):
        c = choices.get(show_pick.get())
        if c is None or not c["ready"]:
            return "This show is not built: uv run scripts/show.py build %s" % os.path.relpath(link.config, UI.ROOT)
        if not ip.get().strip():
            return "Type the controller's IP."
        if target.get() == "hardware" and not all(v.get() for v in ticks):
            return "Tick every line of the checklist before the real arm moves (%s)." % what
        miss = UI.missing_packages()
        if miss:
            return "This Python has no %s: uv run scripts/scan_test_ui.py" % ", ".join(miss)
        return None

    def launch(goto_start):
        why = ready("Move to start pos" if goto_start else "Start")
        if why:
            messagebox.showerror("Not yet", why)
            return
        try:
            link.start(ip.get().strip(), 120.0, speed.get(),
                       [td_target.get().strip()] if td_on.get() and td_target.get().strip() else [],
                       target=target.get(), move_vel=move_vel.get(), goto_start=goto_start,
                       scan_speed=scan_speed.get(), scan_test=not goto_start)
        except ValueError as e:
            messagebox.showerror("Not allowed", str(e))
            return
        for v in ticks:
            v.set(False)

    goto_b = ttk.Button(top, text="1  Move to start pos", command=lambda: launch(True))
    goto_b.grid(row=1, column=4, padx=10)
    start_b = ttk.Button(top, text="2  Start", command=lambda: launch(False))
    start_b.grid(row=2, column=4, padx=10)
    tk.Button(top, text="STOP", bg="#c0392b", fg="white", width=8, font=("Segoe UI", 12, "bold"),
              command=lambda: threading.Thread(target=link.stop, daemon=True).start()).grid(row=1, column=5, rowspan=2,
                                                                                              padx=6)

    # --- the steps --------------------------------------------------------------
    st = ttk.LabelFrame(root, text="Steps  (only what is allowed where the arm is)")
    st.pack(fill="x", **pad)
    buttons = {}
    for i, (name, label) in enumerate(STEP_BUTTONS):
        b = ttk.Button(st, text=label, width=18,
                       command=lambda n=name: link.trigger(step_command(n, scan_speed.get())))
        b.grid(row=0, column=i, padx=4, pady=4)
        buttons[name] = b
    ttk.Label(st, text="Scan speed (of the built %.2f m/s)" % built_mps).grid(row=1, column=1, columnspan=2,
                                                                             sticky="e")
    ttk.Spinbox(st, from_=UI.SCAN_SPEED[0], to=UI.SCAN_SPEED[1], increment=0.05, textvariable=scan_speed,
                width=6).grid(row=1, column=3, sticky="w")
    mps_l = ttk.Label(st, text="")
    mps_l.grid(row=2, column=1, columnspan=3, sticky="e")
    finish_b = ttk.Button(st, text="6  Finish: back to start pos", width=26, command=link.pause)
    finish_b.grid(row=0, column=4, padx=10)

    # --- now --------------------------------------------------------------------
    now = ttk.LabelFrame(root, text="Now")
    now.pack(fill="x", **pad)
    state_l = tk.Label(now, text="NOT RUNNING", fg="white", bg="#777777", font=("Segoe UI", 16, "bold"), width=14)
    state_l.grid(row=0, column=0, rowspan=2, padx=6, pady=4, sticky="ns")
    where_l = ttk.Label(now, text="-", font=("Consolas", 13, "bold"))
    where_l.grid(row=0, column=1, sticky="w")
    prog = ttk.Progressbar(now, length=380, maximum=1.0)
    prog.grid(row=1, column=1, sticky="w")
    scan_l = ttk.Label(now, text="", font=("Consolas", 11))
    scan_l.grid(row=2, column=1, sticky="w")
    logf = ttk.LabelFrame(root, text="Stream process")
    logf.pack(fill="both", expand=True, **pad)
    text = tk.Text(logf, height=10, width=96, font=("Consolas", 9))
    text.pack(fill="both", expand=True)

    def tick():
        s = link.status
        running = link.running
        for w in before_start:
            w.configure(state="disabled" if running else "normal")
        goto_b.configure(state="disabled" if running else "normal")
        start_b.configure(state="disabled" if running else "normal")
        try:
            question = link.asks.get_nowait()
        except queue.Empty:
            question = None
        if question is not None:
            link.answer(messagebox.askyesno("Confirm -- the REAL arm will move", question + "\n\nGo on?",
                                            icon="warning", default="no"))
        age = time.time() - link.last_status if link.last_status else None
        live = running and age is not None and age < 1.0
        allowed = str(s.get("next") or "").split(",") if live else []
        for name, b in buttons.items():
            b.configure(state="normal" if name in allowed else "disabled")
        finish_b.configure(state="normal" if live and s["state"] != "PAUSED" else "disabled")
        mps_l.config(text="= %.3f m/s" % (built_mps * scan_speed.get()))
        state = str(s["state"]) if live else ("STARTING" if running else "NOT RUNNING")
        state_l.config(text=state, bg=UI.STATE_COLOUR.get(state, "#777777"))
        if live:
            where_l.config(text="%s   (%s)" % (s["clip"], s["hub"]))
            prog["value"] = float(s.get("progress") or 0.0)
            u = float(s.get("scan/u", -1.0))
            scan_l.config(text=("strip u %.2f   LEDs %s   %.3f m/s   %.1f s left"
                                % (u, "ON" if int(s.get("scan/led", 0)) else "off", float(s.get("scan/speed", 0.0)),
                                   float(s.get("time_left") or 0.0))) if u > -0.5 else
                          ("%.1f s left" % float(s.get("time_left") or 0.0) if s.get("time_left") else ""))
        else:
            where_l.config(text="-")
            prog["value"] = 0.0
            scan_l.config(text="")
        while True:
            try:
                line = link.log.get_nowait()
            except queue.Empty:
                break
            text.insert("end", line + "\n")
            text.see("end")
        root.after(100, tick)

    def on_close():
        if link.running and not messagebox.askyesno("Stop?", "The stream is running. Stop it and close?"):
            return
        link.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    tick()
    root.mainloop()


def self_test():
    """The commands the buttons send (the stream itself: scan_test.py's
    self-test, and a run on SimMachine)."""
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    check("scan carries its speed; the other steps their name", step_command("scan", 0.1) == "scan 0.1"
          and step_command("return", 0.1) == "return")
    argv = UI.stream_argv(UI.DEFAULT_CONFIG, "192.168.116.128", 120, 0.5, ["127.0.0.1:9002"], scan_test=True)
    check("Start: show_stream in scan-test mode, OSC on, status also to TD", "--scan-test" in argv and "--osc" in argv
          and argv[argv.index("--osc-out") + 1] == "127.0.0.1:9002", argv)
    go = UI.stream_argv(UI.DEFAULT_CONFIG, "192.168.116.128", 120, 0.5, scan_test=True, goto_start=True)
    check("Move to start pos: only the checked MoveJ", "--goto-start" in go and "--scan-test" not in go, go)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(self_test())
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    run_window(os.path.abspath(args[0]) if args else UI.DEFAULT_CONFIG)
