"""Capability atlas bake -- the Python SOP in scenes/FR20_atlas.hiplc.

Readable copy (scripts/ vs otls/: keep in sync); the SOP runs
`import atlas_sop; atlas_sop.cook(hou.pwd())`.

Fills a grid over the FR20 workspace, in Houdini's frame (Y up, robot base at
the origin, as wenyi::robot_arm draws it), with scripts/capability.py values,
one native volume per field:

    reachable   1 / 0 for the tool direction on the node
    headroom    m/s, worst-direction TCP speed before a joint hits its limit
    wrist       |sin q5|, 0 = wrist singularity
    margin      degrees to the nearest joint limit
    nsol        branches within limits
    clear       1 if some in-limit branch reaches it without touching the
                room or itself (Cell Environment set) -- reachable is not
                usable: low, far points reach with the upper arm on the floor
    clearance   m beyond the nearest object's margin, of that branch (capped
                at 0.5); negative where it touches
    capability  fraction of tool directions that reach the voxel (Mode:
                Capability only -- 26 directions x 2 rolls, ~50x slower)

Unreachable voxels are 0 in every field. The grid is split into Slabs along
Y so a TOP network can bake slabs in parallel (one work item each); Slab -1
bakes everything in one go.

Parameters (spare parms on the SOP): voxel (m), mode (0 fixed direction,
1 capability), dir (tool direction, Houdini frame), tool_len (m, beyond the
flange), slab, slabs, ymin, ymax, radius (m, around the Y axis), env_file
(the cell; empty: no clear / clearance fields). Headroom, wrist and margin
are those of the clear branch when there is one.
"""

import math
import os
import sys
import time

import hou


def _scripts():
    s = hou.text.expandString("$HIP/../scripts")
    if s not in sys.path:
        sys.path.insert(0, s)
    return s


def _load():
    _scripts()
    import capability
    return capability


def grid(voxel, radius, ymin, ymax):
    """Voxel counts and the bounding box whose voxel centres fall on
    multiples of voxel (so every slab's voxels line up)."""
    n_xz = int(math.ceil(radius / voxel))
    j0, j1 = int(math.floor(ymin / voxel)), int(math.ceil(ymax / voxel))
    return n_xz, j0, j1


def cook(node):
    C = _load()
    geo = node.geometry()
    geo.clear()
    ev = node.evalParm
    voxel = max(0.01, ev("voxel"))
    mode = int(ev("mode"))
    d_h = (ev("dirx"), ev("diry"), ev("dirz"))
    tool_len = ev("tool_len")
    slab, slabs = int(ev("slab")), max(1, int(ev("slabs")))
    radius, ymin, ymax = ev("radius"), ev("ymin"), ev("ymax")

    root = hou.text.expandString("$HIP/..")
    model, chain, fo, vel = C.load_fr20(root)
    envp = node.parm("env_file").eval().strip() if node.parm("env_file") else ""
    cell = None
    if envp and os.path.exists(envp):
        import collision
        cell = (collision.load_model("fr20", tool_len=round(tool_len, 3)), collision.load_env(envp))
    reach_max = 1.854 + fo + tool_len + voxel          # FR20 datasheet reach + tool
    n_xz, j0, j1 = grid(voxel, radius, ymin, ymax)
    layers = list(range(j0, j1 + 1))
    if slab >= 0:
        per = int(math.ceil(len(layers) / float(slabs)))
        layers = layers[slab * per:(slab + 1) * per]
    if not layers:
        return
    xs = [i * voxel for i in range(-n_xz, n_xz + 1)]
    zs = xs
    # Houdini (x, y, z) -> URDF (x, -z, y), the inverse of urdf_rig.Z_UP_TO_Y_UP
    to_urdf = lambda p: (p[0], -p[2], p[1])
    d_u = to_urdf(d_h)
    dirs = C.sphere_directions(26) if mode == 1 else None
    names = ["reachable", "headroom", "wrist", "margin", "nsol"] + (["clear", "clearance"] if cell else [])         + (["capability"] if mode == 1 else [])
    data = {n: [] for n in names}
    # shoulder: J2's axis height; points further than the reach from it are out
    sh = C.U.forward_kinematics(chain)[1]["position"]
    t0 = time.time()
    n_eval = 0
    for z in zs:                            # volume order: x fastest, then y, then z
        for j in layers:
            y = j * voxel
            for x in xs:
                pu = to_urdf((x, y, z))
                far = math.dist(pu, sh) > reach_max or math.hypot(x, z) > radius + 1e-9
                if far:
                    m = None
                else:
                    m = C.measure(model, chain, pu, d_u, fo, vel, tool_len=tool_len, cell=cell)
                    n_eval += 1
                ok = bool(m and m["reachable"])
                data["reachable"].append(1.0 if ok else 0.0)
                data["headroom"].append(m["headroom_mps"] if ok else 0.0)
                data["wrist"].append(m["wrist"] if ok else 0.0)
                data["margin"].append(m["margin_deg"] if ok else 0.0)
                data["nsol"].append(float(m["nsol"]) if m else 0.0)
                if cell:
                    data["clear"].append(float(m["clear"]) if ok else 0.0)
                    data["clearance"].append(m["clearance_m"] if ok else -0.5)
                if mode == 1:
                    data["capability"].append(0.0 if far else C.capability_index(model, chain, pu, fo, dirs, tool_len))
    nx, ny, nz = len(xs), len(layers), len(zs)
    lo = hou.Vector3(xs[0] - voxel / 2, layers[0] * voxel - voxel / 2, zs[0] - voxel / 2)
    hi = hou.Vector3(xs[-1] + voxel / 2, layers[-1] * voxel + voxel / 2, zs[-1] + voxel / 2)
    bbox = hou.BoundingBox(lo[0], lo[1], lo[2], hi[0], hi[1], hi[2])
    name_attr = geo.addAttrib(hou.attribType.Prim, "name", "")
    for n in names:
        vol = geo.createVolume(nx, ny, nz, bbox)
        vol.setAttribValue(name_attr, n)
        # data is z-major then y then x -- the order setAllVoxels expects
        vol.setAllVoxels(data[n])
    geo.addAttrib(hou.attribType.Global, "atlas_seconds", 0.0)
    geo.setGlobalAttribValue("atlas_seconds", time.time() - t0)
    geo.addAttrib(hou.attribType.Global, "atlas_evaluated", 0)
    geo.setGlobalAttribValue("atlas_evaluated", n_eval)


def merge(node):
    """atlas_merge (Python SOP): the slab files a TOP network wrote, as one
    volume per field. Slabs tile the grid along Y; their voxels are copied
    into a single volume so iso-surfaces and slices have no seams."""
    import glob
    geo = node.geometry()
    geo.clear()
    pattern = node.evalParm("pattern")
    files = sorted(glob.glob(pattern))
    if not files:
        raise hou.NodeError("no slab files match %s -- cook the TOP network first" % pattern)
    slabs = []
    for f in files:
        g = hou.Geometry()
        g.loadFromFile(f)
        vols = {p.attribValue("name"): p for p in g.prims() if isinstance(p, hou.Volume)}
        if vols:
            any_v = next(iter(vols.values()))
            slabs.append((any_v.boundingBox().minvec()[1], g, vols))
    slabs.sort(key=lambda s: s[0])
    names = list(slabs[0][2].keys())
    nx, _, nz = slabs[0][2][names[0]].resolution()
    ny = sum(s[2][names[0]].resolution()[1] for s in slabs)
    b0 = slabs[0][2][names[0]].boundingBox()
    b1 = slabs[-1][2][names[0]].boundingBox()
    bbox = hou.BoundingBox(b0.minvec()[0], b0.minvec()[1], b0.minvec()[2],
                           b1.maxvec()[0], b1.maxvec()[1], b1.maxvec()[2])
    name_attr = geo.addAttrib(hou.attribType.Prim, "name", "")
    seconds = 0.0
    for _, g, _v in slabs:
        a = g.findGlobalAttrib("atlas_seconds")
        seconds += g.attribValue("atlas_seconds") if a else 0.0
    for n in names:
        out = [0.0] * (nx * ny * nz)
        y0 = 0
        for _, g, vols in slabs:
            v = vols[n]
            sx, sy, sz = v.resolution()
            vals = v.allVoxels()
            for k in range(sz):
                for j in range(sy):
                    src = (k * sy + j) * sx
                    dst = (k * ny + y0 + j) * nx
                    out[dst:dst + nx] = vals[src:src + sx]
            y0 += sy
        vol = geo.createVolume(nx, ny, nz, bbox)
        vol.setAttribValue(name_attr, n)
        vol.setAllVoxels(out)
    geo.addAttrib(hou.attribType.Global, "atlas_seconds", 0.0)
    geo.setGlobalAttribValue("atlas_seconds", seconds)
    geo.addAttrib(hou.attribType.Global, "atlas_slabs", 0)
    geo.setGlobalAttribValue("atlas_slabs", len(slabs))
