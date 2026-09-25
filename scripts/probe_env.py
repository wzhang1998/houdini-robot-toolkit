"""Measure the cell with the robot itself: touch a point with the tool tip,
record it. Read-only -- this never commands a motion.

    python scripts/probe_env.py --ip IP [--points envs/volvox_lab_points.json] [--tool-len 0.0]

Put the arm in hand-guiding (drag teach) from the WebUI or the pendant, bring
the tool tip onto a point, and at the prompt type a name for it:

    wall_tv:plane          three or more points on a wall / the floor / a
                           table top  -> a plane (the robot's side stays free)
    control_cart:box       the top corners of a box (and one on the floor if it
                           does not stand on the floor) -> a box
    operator:cylinder      points round a zone at floor level -> a cylinder
    <anything>:point       a single reference point

Several points share a name; `u` undoes the last, `q` quits. Each record
keeps the joints and the TCP by this toolkit's URDF forward kinematics (the
frame the env file uses), plus the controller's own TCP pose to cross-check.
Then: python scripts/env_from_points.py envs/volvox_lab_points.json
"""

import argparse
import json
import os
import sys
import time
import xmlrpc.client

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import collision as CL  # noqa: E402
import urdf_rig as U  # noqa: E402


def read_pose(rpc, model):
    j = rpc.GetActualJointPosDegree(1)
    if j[0] != 0:
        raise RuntimeError("GetActualJointPosDegree returned %s" % (j,))
    q = [float(x) for x in j[1:7]]
    _, tcp = CL.capsules(model, q)
    ctrl = rpc.GetActualTCPPose(1)
    return {"joints_deg": [round(x, 4) for x in q], "tcp_m": [round(x, 5) for x in tcp],
            "controller_tcp_mm_deg": [round(float(x), 3) for x in ctrl[1:7]] if ctrl[0] == 0 else None,
            "time": time.strftime("%Y-%m-%d %H:%M:%S")}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default="192.168.116.128")
    ap.add_argument("--points", default=os.path.join(ROOT, "envs", "volvox_lab_points.json"))
    ap.add_argument("--tool-len", type=float, default=0.0, help="probe tip beyond the flange face, m")
    ap.add_argument("--once", help="record one point with this name and exit (no prompt)")
    a = ap.parse_args(argv)
    model = CL.load_model("fr20", tool_len=a.tool_len)
    rpc = xmlrpc.client.ServerProxy("http://%s:20003" % a.ip)
    data = {"schema": "motionlab.points/1", "tool_len_m": a.tool_len, "points": []}
    if os.path.exists(a.points):
        with open(a.points) as f:
            data = json.load(f)

    def save():
        os.makedirs(os.path.dirname(os.path.abspath(a.points)), exist_ok=True)
        with open(a.points, "w") as f:
            json.dump(data, f, indent=1)

    if a.once:
        rec = dict(read_pose(rpc, model), name=a.once)
        data["points"].append(rec)
        save()
        print(json.dumps(rec))
        return 0
    print("recording into %s  (name:kind, u = undo, q = quit)" % a.points)
    while True:
        s = input("> ").strip()
        if s == "q":
            break
        if s == "u":
            if data["points"]:
                print("removed", data["points"].pop()["name"])
                save()
            continue
        if not s:
            continue
        rec = dict(read_pose(rpc, model), name=s)
        data["points"].append(rec)
        save()
        print("  %s  TCP %s m" % (s, rec["tcp_m"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
