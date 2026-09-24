# Motion lab architecture

Where Houdini, ROS 2 and AI each sit when this toolkit grows from "animate
one robot" into a motion library for live installations. The staged work is
in `IMPLEMENTATION_PLAN.md`; this file keeps the reasoning after that plan is
done and deleted.

## The division of labour

| Part | Owns | Does not own |
|---|---|---|
| **Houdini** | What motion is *possible* and *expressive*: generation, search, simulation, spatial fields, synthetic data | The final word on feasibility |
| **ROS 2 / MoveIt 2** | What motion is *real*: collision, industrial motion types, time parameterisation, execution | Authoring |
| **AI** | Judgement and generalisation: labelling, learned motion selection, perception | Safety |

Houdini is not reduced to drawing targets. Its value is in four things that
robotics lacks artist-grade tools for:

1. **Spatial computation** — reachability, manipulability, clearance and
   speed headroom as volumes (VDB / SDF); point clouds from OAK-D as
   obstacles.
2. **Procedural scale** — motion primitives with style parameters, swept by
   PDG into thousands of variants.
3. **Simulation as a motion source** — forces, flocking, Vellum drive the TCP
   for organic motion, *constrained by the feasibility fields* so the robot
   can actually execute it.
4. **Synthetic data** — Solaris / USD / Karma for labelled perception data.

## Four layers

1. **Robot-aware Houdini.** The robot's own inverse kinematics runs inside
   Houdini, so every point carries real joint angles, limit margin, distance
   to singularity and speed. Feasibility is a live attribute, not a verdict
   after the fact.
2. **PDG motion factory.** Wedge primitives × style parameters → fast
   in-Houdini feasibility filter → ROS 2 validation → preview render →
   library manifest.
3. **AI loop.** A vision-language model rates each preview ("reads as
   curious / hesitant"), writing labels into the manifest. A motion index or
   small generative model maps style + audience state to clips at runtime.
4. **ROS bridge.** Clips leave as `trajectory_msgs/JointTrajectory`-shaped
   data; real runs come back as MCAP for sim-to-real comparison in Houdini.

## Decisions

- **One kinematics source.** Interactive preview, batch validation and
  execution all solve against the same URDF (+ SRDF for MoveIt). A preview
  that disagrees with the robot is worse than none.
- **Clips follow Cartesian paths; transitions are planned.** Sampling
  planners (OMPL) optimise arrival, not expression, so authored clips use
  path following — MoveIt's Pilz LIN / PTP / CIRC, which match Fairino's
  MoveL / MoveJ / MoveC. Only the joins between clips are planned (OMPL or
  cuRobo).
- **Closed-form IK for UR-type arms.** FR20's J2/J3/J4 axes are parallel,
  the UR layout, which has a published closed-form solution with up to 8
  branches (Hawkins 2013; ROS-Industrial `ur_kinematics`). Picking the branch
  nearest the previous frame removes the FBIK failures measured on FR20:
  ranges crossing ±180°, wrist flips, solves restarting from a singular zero
  pose. The implementation is ours; its authority check is Fairino's own
  controller IK/FK (`GetInverseKin`, `GetForwardKin`) on SimMachine.
- **FBIK stays for UF850** until a UF850 solver exists; the profile selects
  the solver.
- **ROS 2 runs in a container** (WSL2 / Docker, ROS 2 Jazzy + MoveIt 2)
  behind a small HTTP service, as Motion-Core already does for cuRobo. Batch
  work tolerates the latency; interactive preview never leaves Houdini.

## Open questions

- URDF zero == Fairino controller zero (check in SimMachine).
- ServoJ latency and jitter on FR20 — no published figures.
- Whether a learned motion model beats plain motion matching for a library
  of this size.
