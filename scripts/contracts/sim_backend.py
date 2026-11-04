"""SimBackend: a RobotBackend of pure kinematics -- no controller, no
network, no hardware. The arm is where it was last told to be (optionally a
fixed lag behind), inside the profile's limits, at most as fast as them.

What it checks, as a controller would and so a stream finds out here first:
  - a servo target outside the joint limits             -> fault main 4  (refused, the arm stays)
  - a servo step faster than the profile's velocity
    limit x STEP_MARGIN over its cmd_t                  -> fault main 5  (refused)
  - servo_j outside servo_start .. servo_end, a target
    of the wrong length or not finite, cmd_id not rising -> fault main 6  (refused)
  - optionally the wrist singularity: |J5| under
    singular_j5_deg                                     -> fault main 10 (the FR20's singular-pose code)
A fault holds until reset_errors(); while it holds every move is refused.
These main codes are the simulator's own (BackendInfo.faults says which one
the stream may recover from); they are not Fairino's beyond 10.

    arm = SimArm("fr20")                       # one simulated arm ...
    cmd, fb = arm.connect(), arm.connect()     # ... two connections, as the stream opens them
    SimFactory("fr20").command()               # or through a BackendFactory

    python scripts/contracts/sim_backend.py    self-test
"""

import bisect
import math
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
ROOT = os.path.dirname(SCRIPTS)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

from contracts import robot as R  # noqa: E402

STEP_MARGIN = 1.1                     # as show_stream.STEP_MARGIN: rounding and sampling
FAULT_LIMIT, FAULT_OVERSPEED, FAULT_PROTOCOL, FAULT_SINGULAR = 4, 5, 6, 10
SIM_FAULTS = R.FaultPolicy(singular_pose_main=FAULT_SINGULAR)


class SimArm:
    """The simulated arm: state shared by every connection to it (thread-safe)."""

    def __init__(self, profile_id="fr20", q0=None, lag_s=0.0, realtime_moves=False, move_time_scale=1.0,
                 singular_j5_deg=None, profile_dir=None, clock=time.perf_counter):
        import robot_profile as RP
        self.prof = RP.load(profile_id, profile_dir)
        self.profile_id = profile_id
        self.n = self.prof["robot"]["num_joints"]
        self.limits = [tuple(x) for x in self.prof["robot"]["limits_deg"]]
        self.vel = RP.velocity_limits(self.prof)
        home = RP.home(self.prof)
        self.q = list(q0 if q0 is not None else (home or [0.5 * (lo + hi) for lo, hi in self.limits]))
        if not R.within_limits(self.q, self.limits):
            raise ValueError("SimArm: q0 %s outside the profile's limits" % self.q)
        self.lag_s, self.realtime_moves, self.move_time_scale = lag_s, realtime_moves, move_time_scale
        self.singular_j5_deg = singular_j5_deg
        self.clock = clock
        self.lock = threading.RLock()
        self.error = list(R.NO_ERROR)
        self.servo = False
        self.last_id = None
        self.history = [(clock(), list(self.q))]    # commanded poses, for the lagged read
        self.log = []                               # ("servo_start" | "servo_end" | "stop" | "move" | "fault", ...)
        self._chain = None

    def connect(self):
        return SimBackend(self)

    # ------------------------------------------------------------------
    def _fault(self, main, sub, why):
        if self.error == R.NO_ERROR:
            self.error = [0, int(main), int(sub)]
            self.log.append(("fault", main, sub, why))

    def _set(self, q):
        self.q = list(q)
        self.history.append((self.clock(), list(q)))
        if len(self.history) > 5000:
            del self.history[:2500]

    def actual(self):
        with self.lock:
            if self.lag_s <= 0.0 or len(self.history) < 2:
                return list(self.q)
            t = self.clock() - self.lag_s
            ts = [h[0] for h in self.history]
            k = bisect.bisect_right(ts, t) - 1
            return list(self.history[max(k, 0)][1])

    def tcp(self, q=None):
        """The flange face's position (m, base frame) at q by the URDF's FK; None without a URDF."""
        urdf = self.prof["rig"].get("urdf")
        if not urdf:
            return None
        import ur_ik
        import urdf_rig as U
        if self._chain is None:
            self._chain = U.parse_urdf(os.path.join(ROOT, urdf))["chain"]
        if q is None:
            with self.lock:
                q = list(self.q)
        Rm, p6 = ur_ik.pose_of(self._chain, list(q))
        return U._add(p6, U._mat_vec(Rm, (0.0, 0.0, float(self.prof["rig"].get("flange_offset_m", 0.0)))))


class SimBackend:
    """One connection to a SimArm; satisfies contracts.robot.RobotBackend."""

    info = None

    def __init__(self, arm):
        self.arm = arm
        self.info = R.BackendInfo("sim", arm.profile_id, 125.0, True, SIM_FAULTS, "contracts/sim_backend.py")

    # identity, state
    def model(self):
        return "SIM %s (contracts.SimBackend)" % self.arm.profile_id.upper()

    def joints(self):
        return self.arm.actual()

    def error_code(self):
        with self.arm.lock:
            return list(self.arm.error)

    def reset_errors(self):
        with self.arm.lock:
            self.arm.error = list(R.NO_ERROR)
            self.arm.log.append(("reset",))

    def prepare(self):
        pass

    def frames(self):
        return 0, 0

    def queue_length(self):
        return 0

    def diagnose(self, target_desc=None, tool=None, wobj=None):
        return "sim: error %s, joints %s" % (self.error_code(), [round(x, 2) for x in self.joints()])

    # planned move
    def move_to(self, q, vel_pct, tool=0, wobj=0):
        arm = self.arm
        with arm.lock:
            if arm.error != R.NO_ERROR:
                raise RuntimeError("MoveJ refused: the arm is in fault %s" % arm.error)
            if len(q) != arm.n or not R.finite(q):
                raise RuntimeError("MoveJ refused: target %r is not %d finite angles" % (q, arm.n))
            if not R.within_limits(q, arm.limits):
                raise RuntimeError("MoveJ refused: target %s outside the joint limits" % [round(x, 2) for x in q])
            if not 0.0 < float(vel_pct) <= 100.0:
                raise RuntimeError("MoveJ refused: vel_pct %r not in (0, 100]" % vel_pct)
            dur = max(abs(b - a) / (v * vel_pct / 100.0) for a, b, v in zip(arm.q, q, arm.vel))
            arm.log.append(("move", [round(x, 3) for x in q], vel_pct, round(dur, 3)))
        if arm.realtime_moves and dur > 0:
            time.sleep(dur * arm.move_time_scale)
        with arm.lock:
            arm._set(q)

    # streaming
    def servo_start(self):
        with self.arm.lock:
            self.arm.servo, self.arm.last_id = True, None
            self.arm.log.append(("servo_start",))

    def servo_j(self, q, cmd_t, cmd_id):
        arm = self.arm
        with arm.lock:
            if arm.error != R.NO_ERROR:
                return [arm.error[1] or 1]
            if not arm.servo:
                arm._fault(FAULT_PROTOCOL, 1, "servo_j outside servo_start .. servo_end")
                return [FAULT_PROTOCOL]
            if len(q) != arm.n or not R.finite(q) or not (cmd_t > 0):
                arm._fault(FAULT_PROTOCOL, 2, "bad target %r / cmd_t %r" % (q, cmd_t))
                return [FAULT_PROTOCOL]
            if arm.last_id is not None and cmd_id <= arm.last_id:
                arm._fault(FAULT_PROTOCOL, 3, "cmd_id %r after %r" % (cmd_id, arm.last_id))
                return [FAULT_PROTOCOL]
            if not R.within_limits(q, arm.limits):
                arm._fault(FAULT_LIMIT, 1, "target %s outside the limits" % [round(x, 2) for x in q])
                return [FAULT_LIMIT]
            for j, (a, b, v) in enumerate(zip(arm.q, q, arm.vel)):
                if abs(b - a) > v * cmd_t * STEP_MARGIN:
                    arm._fault(FAULT_OVERSPEED, j + 1, "J%d %.3f deg in %.4f s" % (j + 1, abs(b - a), cmd_t))
                    return [FAULT_OVERSPEED]
            if arm.singular_j5_deg is not None and arm.n >= 5 and abs(q[4]) < arm.singular_j5_deg:
                arm._fault(FAULT_SINGULAR, 1, "J5 %.3f deg: the wrist singularity" % q[4])
                return [FAULT_SINGULAR]
            arm.last_id = cmd_id
            arm._set(q)
            return 0

    def servo_end(self):
        with self.arm.lock:
            self.arm.servo = False
            self.arm.log.append(("servo_end",))
        return 0

    def stop(self):
        with self.arm.lock:
            self.arm.servo = False
            self.arm.log.append(("stop",))
        return 0

    # for tests
    def inject_fault(self, main, sub=0):
        with self.arm.lock:
            self.arm._fault(main, sub, "injected")


class SimFactory:
    """A BackendFactory: both connections to one SimArm."""

    def __init__(self, profile_id="fr20", **kw):
        self.arm = SimArm(profile_id, **kw)

    def command(self):
        return self.arm.connect()

    def feedback(self):
        return self.arm.connect()


def self_test():
    fails = []

    def check(label, ok, detail=""):
        print("%s  %s%s" % ("ok  " if ok else "FAIL", label, ("  -- " + str(detail)) if detail else ""))
        if not ok:
            fails.append(label)

    f = SimFactory("fr20")
    c, fb = f.command(), f.feedback()
    check("a RobotBackend, structurally", isinstance(c, R.RobotBackend) and not R.missing_methods(c),
          R.missing_methods(c))
    check("model names the profile (fairino_player.main's check)", "FR20" in c.model().upper(), c.model())
    check("starts at the profile's home, no error", c.joints() == [0.0, -90.0, 90.0, -90.0, -90.0, 0.0]
          and c.error_code() == [0, 0, 0], (c.joints(), c.error_code()))
    c.move_to([10.0, -90.0, 90.0, -90.0, -90.0, 0.0], 20.0)
    check("MoveJ: there when it returns; the other connection sees it", fb.joints()[0] == 10.0, fb.joints())
    c.servo_start()
    q = fb.joints()
    ok = all(R.is_ok(c.servo_j([q[0] + 0.1 * k] + q[1:], 0.008, k)) for k in range(1, 50))
    check("a slow stream is taken", ok and abs(fb.joints()[0] - (q[0] + 4.9)) < 1e-9, fb.joints()[0])
    code = c.servo_j([q[0] + 50.0] + q[1:], 0.008, 100)
    check("a jump is refused, a fault holds, the arm stays", not R.is_ok(code) and c.error_code()[1] == FAULT_OVERSPEED
          and abs(fb.joints()[0] - (q[0] + 4.9)) < 1e-9, (code, c.error_code()))
    refused = False
    try:
        c.move_to(q, 10.0)
    except RuntimeError:
        refused = True
    check("... and refuses a MoveJ until reset", refused)
    c.reset_errors()
    check("reset clears it", c.error_code() == [0, 0, 0])
    code = c.servo_j([200.0] + q[1:], 10.0, 101)
    check("a target outside the limits is refused (limit fault)", not R.is_ok(code) and c.error_code()[1] == FAULT_LIMIT,
          c.error_code())
    c.reset_errors()
    code = c.servo_j(q, 0.008, 5)
    check("cmd_id must rise", not R.is_ok(code) and c.error_code()[1] == FAULT_PROTOCOL, c.error_code())
    c.reset_errors()
    c.servo_end()
    code = c.servo_j(q, 0.008, 500)
    check("servo_j after servo_end is a protocol fault", not R.is_ok(code), c.error_code())
    c.reset_errors()
    s = SimArm("fr20", singular_j5_deg=1.0).connect()
    s.servo_start()
    hq = s.joints()
    s.servo_j(hq[:4] + [0.5, hq[5]], 10.0, 1)
    check("optional wrist singularity: J5 near 0 is the FR20's main code 10", s.error_code()[1] == FAULT_SINGULAR,
          s.error_code())
    lag = SimArm("fr20", lag_s=0.05)
    lc = lag.connect()
    lc.servo_start()
    h = lc.joints()
    lc.servo_j([h[0] + 0.5] + h[1:], 0.01, 1)
    first = lc.joints()[0]
    time.sleep(0.07)
    check("lag_s: the read follows the command that much later", first == h[0] and lc.joints()[0] == h[0] + 0.5,
          (first, lc.joints()[0]))
    tcp = SimArm("fr20").tcp()
    check("tcp() by the URDF's FK at home (FR20: ~0.85 m out, ~1.1 m up, profile comment)",
          tcp is not None and 0.6 < math.hypot(tcp[0], tcp[1]) < 1.1 and 0.9 < tcp[2] < 1.3, tcp)
    check("no URDF (uf850): tcp() is None, the rest still works", SimArm("uf850", q0=[0.0] * 6).tcp() is None)
    print("\nFAILED: %s" % "; ".join(fails) if fails else "\nOK")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(self_test())
