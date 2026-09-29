"""The cuRobo planning service, run inside the cuRobo Docker image by
`curobo_bridge.py up` (port 8768; /work holds the robot config it wrote).

    GET  /health   {"ready": bool, "build_s": ..., "plans": n}
    POST /plan     {"start": [6 deg], "goal": [6 deg], "world": {"cuboid": {...}}}
                   -> {"ok": bool, "status": ..., "plan_s": s, "q_deg": [[6 deg] ...], "dt": s}

One MotionGen, built and warmed up at start; a world sent with a plan
replaces the last one when it differs. Python standard library besides
cuRobo; no dependency on the toolkit.
"""

import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import torch
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

PORT = 8768
CACHE_OBB = 64                     # room for this many cuboids in a world
ta = TensorDeviceType()
state = {"ready": False, "build_s": None, "plans": 0, "world": None}
lock = threading.Lock()            # one GPU plan at a time
PARKED = {"cuboid": {"parked": {"dims": [0.1, 0.1, 0.1], "pose": [50.0, 50.0, 50.0, 1.0, 0.0, 0.0, 0.0]}}}


def build():
    t0 = time.time()
    robot = json.load(open("/work/fr20.json"))["robot_cfg"]
    mg = MotionGen(MotionGenConfig.load_from_robot_config(
        robot, WorldConfig.from_dict(PARKED), tensor_args=ta, interpolation_dt=0.02, use_cuda_graph=False,
        collision_activation_distance=0.01, collision_cache={"obb": CACHE_OBB}))
    mg.warmup()
    state["build_s"] = round(time.time() - t0, 1)
    return mg


MG = None


def plan(body):
    world = body.get("world") or PARKED
    with lock:
        if world != state["world"]:
            MG.update_world(WorldConfig.from_dict(world))
            state["world"] = world
        t0 = time.time()
        start = JointState.from_position(ta.to_device([[math.radians(x) for x in body["start"]]]))
        goal = JointState.from_position(ta.to_device([[math.radians(x) for x in body["goal"]]]))
        r = MG.plan_single_js(start, goal, MotionGenPlanConfig(max_attempts=8, enable_graph=True))
        torch.cuda.synchronize()
        state["plans"] += 1
    out = {"ok": bool(r.success.item()), "status": str(r.status), "plan_s": round(time.time() - t0, 3)}
    if out["ok"]:
        p = r.get_interpolated_plan()
        out["q_deg"] = [[math.degrees(x) for x in row] for row in p.position.cpu().tolist()]
        out["dt"] = float(r.interpolation_dt)
    return out


class Handler(BaseHTTPRequestHandler):
    def _send(self, code, obj):
        data = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/health":
            self._send(200, {k: state[k] for k in ("ready", "build_s", "plans")})
        else:
            self._send(404, {"error": "unknown"})

    def do_POST(self):
        if self.path != "/plan":
            return self._send(404, {"error": "unknown"})
        if not state["ready"]:
            return self._send(503, {"ok": False, "status": "warming up"})
        try:
            body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))).decode())
            self._send(200, plan(body))
        except Exception as e:  # pylint:disable=broad-except
            self._send(200, {"ok": False, "status": "error: %r" % e})

    def log_message(self, fmt, *args):
        pass


if __name__ == "__main__":
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print("listening on %d; building MotionGen" % PORT, flush=True)
    MG = build()
    state["ready"] = True
    print("ready (%.1f s)" % state["build_s"], flush=True)
    threading.Event().wait()
