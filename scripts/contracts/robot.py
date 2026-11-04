"""The Robot contract: a profile (what the arm IS: profiles/<id>.json) plus a
backend (how to MOVE it: RobotBackend). docs/architecture/contracts.md, Robot.

RobotBackend is the duck type the show already calls -- fairino_player.Controller
as show_stream.stream, Link, recover, goto and move_checked use it, and as
show_stream's own self-test fakes it (FakeCtrl). Naming it does not change it:
Controller satisfies it as it stands (conformance.py checks that), and a new
arm's backend that satisfies it plays the show through the same stream loop.

Units and frames, for every method: joint angles in degrees, in the profile's
ROBOT frame (robot_profile.py: what the controller speaks), 1-based joints
only in messages. Error codes are [ret, main, sub] ints; [0, 0, 0] is no
error -- the shape GetRobotErrorCode returns and show_stream compares.

    python scripts/contracts/robot.py        readiness of every profile (what the toolchain needs that it has)
"""

import math
import os
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Protocol, Sequence, Tuple, runtime_checkable

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.dirname(HERE)
ROOT = os.path.dirname(SCRIPTS)
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

NO_ERROR = [0, 0, 0]


@runtime_checkable
class RobotBackend(Protocol):
    """One connection to one arm (or its simulator).

    The stream opens two of them (BackendFactory): one sends at the servo
    rate and must never be slowed down by reads, the other reads feedback and
    is also the watchdog's way to stop the arm when the sender hangs. A
    backend instance is used from one thread at a time."""

    def model(self) -> str:
        """The controller's own identity. Must contain the profile id in some
        case (fairino_player.main refuses a profile the controller does not
        name: "FR20" for fr20)."""

    def joints(self) -> List[float]:
        """The actual joints now (num_joints floats, deg, robot frame)."""

    def error_code(self) -> Sequence[int]:
        """[ret, main, sub]; [0, 0, 0] when the controller reports no fault."""

    def reset_errors(self) -> None:
        """Clear a recoverable fault (a protective stop), as the vendor UI's reset does."""

    def prepare(self) -> None:
        """Whatever the controller needs before a planned move (FR20: automatic mode, enable)."""

    def move_to(self, q: Sequence[float], vel_pct: float, tool: int = 0, wobj: int = 0) -> None:
        """A controller-planned joint move (MoveJ) to q at vel_pct of its speed. Returns once the
        controller has taken it (the FR20's blocks until the arm arrives); callers then wait on
        joints() as fairino_player.goto does. Raises RuntimeError naming why when refused."""

    def servo_start(self) -> None:
        """Enter the streaming mode (FR20: ServoMoveStart)."""

    def servo_j(self, q: Sequence[float], cmd_t: float, cmd_id: int):
        """One streamed joint target, to be reached cmd_t seconds after the previous one; cmd_id
        increases. Returns 0 (or [0, ...]) when taken; anything else is a fault for the stream."""

    def servo_end(self) -> None:
        """Leave the streaming mode."""

    def stop(self) -> None:
        """Stop motion now (a software stop -- never instead of the E-stop). Safe to call from the
        feedback connection while the other one is mid-send."""


# Optional, used when present (hasattr):
#   queue_length() -> int | None     the controller's motion queue (show_stream's report, a diagnostic)
#   frames() -> (tool, wobj)         the tool / workpiece numbers applied (fairino_player._current_frames)
#   diagnose(desc, tool, wobj) -> str  what the controller says now, for an error message
OPTIONAL_METHODS = ("queue_length", "frames", "diagnose")


class BackendFactory(Protocol):
    """How a backend is opened: the sender's connection (blocking calls, no
    timeout -- a socket timeout costs 20-30 ms stalls on Windows, measured
    2026-09-30) and the feedback connection (calls that time out), as
    show_stream.main opens P.Controller(ip) and Link(ip) today."""

    def command(self) -> RobotBackend: ...

    def feedback(self) -> RobotBackend: ...


@dataclass
class FaultPolicy:
    """What a backend's fault codes mean to the stream. FR20's are in show_stream
    as constants (SINGULAR_POSE = 10); a backend states its own here."""
    singular_pose_main: Optional[int] = None      # main code of a stop the stream may recover from by itself
    recoverable_main: Tuple[int, ...] = ()        # any other main codes the stream may recover from (none today)


@dataclass
class BackendInfo:
    """What a backend says about itself, for a report and for conformance."""
    kind: str                                     # "fairino_xmlrpc", "sim", "xarm_sdk", ...
    profile_id: str
    servo_rate_hz: float = 125.0                  # the rate the stream sends at (show_stream.RATE_HZ)
    simulated: bool = False                       # True: no physical arm can move
    faults: FaultPolicy = field(default_factory=FaultPolicy)
    notes: str = ""


FAIRINO_FAULTS = FaultPolicy(singular_pose_main=10)     # show_stream.SINGULAR_POSE


def fairino_info(profile_id="fr20", simulated=False):
    """BackendInfo of fairino_player.Controller as the show uses it today."""
    return BackendInfo("fairino_xmlrpc", profile_id, 125.0, simulated, FAIRINO_FAULTS,
                       "XML-RPC port 20003; SimMachine on 192.168.116.x (fairino_player.SIM_NET)")


def is_ok(code) -> bool:
    """servo_j's return (0, or a list starting 0) is a success."""
    c = code[0] if isinstance(code, (list, tuple)) else code
    return c == 0


def missing_methods(obj) -> List[str]:
    """The RobotBackend methods obj does not have ([] for a conforming one)."""
    names = [n for n in RobotBackend.__dict__ if not n.startswith("_") and callable(getattr(RobotBackend, n, None))]
    return [n for n in names if not callable(getattr(obj, n, None))]


# --------------------------------------------------------------------------
# the profile's half of the contract: what the toolchain reads from it
# --------------------------------------------------------------------------

def profile_readiness(profile_id, profile_dir=None):
    """[(item, status, note)] -- status "ok", "partial" or "missing" -- of what each part of the
    toolchain needs from a profile: limits, measured acceleration, home, URDF (collision, TCP, IK,
    Isaac), the IK solver's own check, the tool. Reads only."""
    import robot_profile as RP
    prof = RP.load(profile_id, profile_dir)
    r, rig = prof["robot"], prof["rig"]
    out = []

    def add(item, status, note):
        out.append((item, status, note))

    add("robot.limits_deg", "ok", "%d joints" % r["num_joints"])
    v = r.get("max_velocity_deg_s")
    add("robot.max_velocity_deg_s", "ok" if isinstance(v, list) else "partial",
        "per joint" if isinstance(v, list) else "one number for all joints (%s)" % v)
    a = r.get("max_acceleration_deg_s2")
    measured = "MEASURED" in str(r.get("_comment_acceleration", ""))
    add("robot.max_acceleration_deg_s2", "missing" if a is None else ("ok" if measured else "partial"),
        "none: fairino_player falls back to 300 deg/s^2" if a is None else
        ("measured on the arm" if measured else "datasheet / series figure, not measured on this arm"))
    add("robot.home_deg", "ok" if r.get("home_deg") else "missing",
        "the show, choreo, cell_sop and --goto-home read it" if not r.get("home_deg") else str(r["home_deg"]))
    urdf = rig.get("urdf")
    upath = os.path.join(ROOT, urdf) if urdf else None
    if not urdf:
        add("rig.urdf", "missing", "collision.load_model, motion_clip TCP, ur_ik, capability, Isaac all need one")
    elif not os.path.isfile(upath):
        add("rig.urdf", "missing", "named but not on disk: %s" % urdf)
    else:
        add("rig.urdf", "ok", urdf)
    solver = rig.get("ik_solver", "fbik")
    if solver == "ur_closed_form":
        if upath and os.path.isfile(upath):
            try:
                import ur_ik
                import urdf_rig as U
                ur_ik.analyse(U.parse_urdf(upath)["chain"])
                add("rig.ik_solver", "ok", "ur_closed_form, and ur_ik.analyse accepts the URDF")
            except Exception as e:
                add("rig.ik_solver", "missing", "ur_closed_form named, but ur_ik.analyse refuses: %s" % e)
        else:
            add("rig.ik_solver", "missing", "ur_closed_form needs the URDF")
    else:
        add("rig.ik_solver", "partial", "%s: in Houdini only (FBIK, mm-level error); the Python generators "
            "(choreo, gestures, paths, the show build) solve with ur_ik" % solver)
    add("rig.flange_offset_m", "ok" if "flange_offset_m" in rig else "missing", str(rig.get("flange_offset_m")))
    tool = prof.get("tool")
    add("tool", "ok" if tool and tool.get("urdf") else "missing",
        tool["urdf"] if tool and tool.get("urdf") else "no tool block: collision checks the bare arm")
    add("output.writer", "ok", prof["output"]["writer"])
    return out


def readiness_table(profile_id):
    rows = profile_readiness(profile_id)
    w = max(len(x[0]) for x in rows)
    return "\n".join("  %-*s  %-7s  %s" % (w, a, b, c) for a, b, c in rows)


def within_limits(q, limits, tol=1e-6):
    return all(lo - tol <= x <= hi + tol for x, (lo, hi) in zip(q, limits))


def finite(q):
    return all(isinstance(x, (int, float)) and math.isfinite(x) for x in q)


if __name__ == "__main__":
    import robot_profile as RP
    for pid in RP.list_profiles():
        print("%s\n%s\n" % (pid, readiness_table(pid)))
