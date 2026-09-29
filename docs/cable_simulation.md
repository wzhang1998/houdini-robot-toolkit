# The LED strip's cable: survey and simulation

The strip's cable runs from its bracket on the flange, clipped to the arm,
to the laptop on the red cart (the user, 2026-09-29). Two questions: where
to clip it, and which motions tangle it or pull on its two connectors.
Plan: IMPLEMENTATION_PLAN.md, Stage 13.

## What we do

1. **Geometry** (`scripts/cable_route.py`, seconds): candidate clips on
   every link (the joints' axis caps, points round each link's body, just
   outside its collision capsules; a candidate another link runs into in
   any frame, or below the floor, is dropped), every motion of the built
   shows sampled, per span the chord's longest (the cable it needs) and its
   range (the slack that swings), the layouts with 2-5 clips that keep the
   largest loop smallest. It suggests clips; it cannot tell a tangle -- a
   straight chord crosses a joint's housing that a real cable wraps round.
2. **Physics** (`scripts/isaac/cable_sim.py`, Isaac Sim 6, PhysX): the
   cable as a chain of capsules on D6 joints, clipped where the survey
   says, its two ends on spring connectors so their pull reads in newtons;
   the worst motions (or the whole library) played; per motion the pull on
   each connector, contact with each link, self contact, pinches; a video.

## Research (2026-09-29)

### Cables in Isaac Sim

- **The standard way is rigid capsules on joints.** NVIDIA's answer on
  the forum: "currently the only way to create the ropes is to use capsules
  and connect them with joints". Isaac Sim ships the example:
  `omni.physx.demos` `RigidBodyRopeDemo.py` (Window > Simulation > Demos >
  Rope; in this install under `extscache/omni.physx.demos-*/omni/physxdemos/scenes/`).
  Its setup, which `cable_sim.py` follows:
  - capsules along their **X** axis; D6 joints with translation locked,
    **twist (rotX) locked**, swing (rotY, rotZ) inside a cone (110 deg)
    with force drives (damping, a little stiffness) -- limiting one swing
    axis alone gives PhysX's "Double pyramid mode not supported";
  - rest offset 0, a contact offset, a friction material.
- **Pitfalls named on the forum**: set `physics:excludeFromArticulation`
  on the rope's joints when it is attached to an articulation (else PhysX
  parses the arm and the rope as one articulation); GPU simulation can run
  out of buffers with many ropes, the CPU pipeline is steadier; NaNs from
  bad initial transforms.
- **Isaac Sim 6 also bundles Newton 1.2** (`exts/isaacsim.pip.newton`),
  whose `builder.add_rod` models a cable as a rod with bending *and twist*
  (VBD solver) -- examples `cable_twist`, `cable_pile`, `cable_y_junction`,
  `cable_bundle_hysteresis`. Better for twist (our J1, J4, J6 all twist the
  cable), but the arm would have to run in Newton too: a later step if the
  PhysX chain's coarse twist matters.
- **Research frameworks**: DeformX (2026) couples Isaac Sim with a
  Cosserat rod engine (self collision, contact with any mesh); WireCraft
  (Isaac Lab 2.2 / Isaac Sim 4.5) benchmarks industrial wire tasks,
  including clip routing; a mass-spring-damper DLO model has been compared
  across simulators.

### How industry does it (dress packs)

- A robot's cable and hose package is a *dress pack*; external dress packs
  are among the main causes of re-teaching and downtime in robot cells,
  and most offline programming tools do not simulate the cable at all --
  hence internally routed dress packs on high-end arms.
- **IPS Cable Simulation** (Fraunhofer-Chalmers, Fraunhofer ITWM) grew out
  of Volvo Cars' need to simulate robot dress packs (from 2005): real-time
  deformation of cables and hoses; FCC also optimise robot paths to
  minimise dress-pack wear. ABB RobotStudio simulates cables and dress
  packs too. Commercial; we take their measures: pull on the ends, bend
  radius, rubbing/wear, wrapping.

### What we took

- The PhysX chain as in NVIDIA's demo (X axis, twist limited, a swing
  cone with drives, joints out of the articulation) -- it attaches to the
  arm and the room we already simulate.
- The measures of the dress-pack tools: connector pull (N), contact per
  link, self contact, pinches; the bend radius next.
- Newton's rod for the wrist's twist only if the chain's result there is
  in doubt.

## Also found on the way (2026-09-29)

Isaac Sim 6 hands a contact report's actors over as encoded ints
(`PhysicsSchemaTools.intToSdfPath`), and the robot's links only report
when their rigid bodies carry `PhysxContactReportAPI`. Before this, every
"0 contacts" our Isaac scripts printed was blind; `isaac_stage.contact_paths`
and `import_robot` fix it, and the runs were checked again (no contact).

## Sources

- [What is the correct way to build a simulated rope? - NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/what-is-the-correct-way-to-build-a-simulated-rope/244881)
- [Wire and Rope's simulation in Isaac Sim 4.5 or 4.2 - NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/wire-and-ropes-simulation-in-isaac-sim4-5-or-4-2/333863/5)
- [Usd asset for Deformable cable/Rope - NVIDIA Developer Forums](https://forums.developer.nvidia.com/t/usd-asset-for-deformable-cable-rope/261265)
- [How to create curtain-like rope simulation in Isaac Sim - GitHub discussion #474](https://github.com/isaac-sim/IsaacSim/discussions/474)
- [DeformX: A Versatile Co-Simulation Framework for Deformable Linear Objects](https://arxiv.org/abs/2606.22116), [project page](https://deformx.github.io/)
- [WireCraft: A Simulation Benchmark for Industrial DLO Manipulation](https://arxiv.org/pdf/2606.18097)
- [Performance Analysis of a Mass-Spring-Damper DLO Model in Robotic Simulation Frameworks](https://arxiv.org/html/2504.13659)
- [Isaac Lab: Interacting with a deformable object](https://isaac-sim.github.io/IsaacLab/main/source/tutorials/01_assets/run_deformable_object.html)
- [IPS Cable Simulation - Fraunhofer-Chalmers Centre](https://www.fcc.chalmers.se/software/ips/ips-cable-simulation/)
- [Path optimization for multi-robot station minimizing dresspack wear - FCC](https://www.fcc.chalmers.se/publication/path-optimization-for-multi-robot-station-minimizing-dresspack-wear/)
- [Robot Station Optimization for Minimizing Dress Pack Problems - ScienceDirect](https://www.sciencedirect.com/science/article/pii/S2212827116000342)
- [ABB Robot DressPack Options - Robots.com](https://www.robots.com/articles/dresspack-options-for-abb-robots)
- [RobotStudio Suite - ABB](https://www.abb.com/global/en/areas/robotics/products/software/robotstudio-suite)
