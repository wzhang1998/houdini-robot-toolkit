# Motion library: survey of robot choreography practice and tools

2026-09-25. Read-only research by an agent, before growing the clip library
beyond `clip_factory.py` (lines / circles / figure-8s, a pipeline test) and
`choreo.py` (Laban-effort phrases, small and near HOME). Principle: prefer
what the field already does and has. Plan items derived from this:
IMPLEMENTATION_PLAN.md, "Stage 3b: a richer library".

## Where we start

- **Placement.** `clip_factory.py` puts shapes at random centres with a
  fixed tool direction.
- **Dance phrases.** `choreo.py` layers Laban efforts onto a small move
  vocabulary. Every phrase starts and ends at HOME, and intensity is cut to
  fit the limits. That is most of why the output looks small and stays near
  HOME.
- **Atlas.** `capability.py` already computes a Zacharias-style capability
  index plus headroom, wrist distance and joint margin. The generators only
  use it to reject points after the fact.
- **Retargeting.** `retarget.py` has a direct wrist-to-TCP mapping and an
  effort-based transfer.

## 1. How practitioners author robot-arm motion

- **Keyframes in a DCC tool, then a post-processor.** This is the dominant
  method for film and stage work.
  - Bot & Dolly's *Box* was driven from Maya with their in-house BD Move.
  - andyRobot's Robot Animator (commercial Maya plugin) uses a character
    rig with IK and keyframes, then generates robot code.
  - Mimic for Maya (Autodesk Robotics Lab, MIT) works the same way, with
    RAPID, KRL and VAL3 post-processors. It has no Fairino support and
    targets Maya 2016-2020.
  - Huang Yi reports 10-20 hours of programming per minute of the
    *HUANG YI & KUKA* choreography. This is the cost a procedural pipeline
    avoids.
- **Parametric / visual programming.**
  - KUKA|prc (Grasshopper) connects parametric geometry to KUKA code.
  - RoboDK has an FR20 model and a Fairino Lua post-processor.
  - The lesson: paths are geometry from a procedural graph, which is what
    Houdini already is.
- **Interactive / behavioural (Madeline Gannon).**
  - *Mimus* (ABB IRB 6700) has no pre-planned movements. It has three
    layers:
    - depth sensors produce people tracks;
    - a behaviour layer picks the "most interesting" person and gets bored;
    - the controller checks commands against safety limits.
  - *Quipt* gives follow, mirror and avoid behaviours. *Manus* runs a group
    of arms with one shared perception system.
  - This is the model for the OAK-D phase.
- **Libraries of parameterised moves (Boston Dynamics Choreographer).**
  - Dances are sequences of stock moves on tracks.
  - Time is counted in slices (4 per beat) at the music's BPM.
  - Each move has typed parameters: pivot, amplitude or direction,
    entry/exit slices, return-to-start.
  - This is the closest analogue to `choreo.py`, and the template for its
    data model.
- **Choreorobotics (Catie Cuan).** *Breathless* (8 h, industrial arm)
  combined three sources:
  - per-joint sinusoids;
  - motion from video pose tracking;
  - improvisation in teach mode.

  A mix of cheap procedural sources gives more range than any single
  generator.

## 2. Expressive / Laban research

- **Knight & Simmons.** Laban Effort for mobile robots (2014), and Effort
  layered on task motion (HRI 2015). `choreo.py` already follows this idea.
- **Laban Shape for manipulators** (Bangalore Raghu et al., RO-MAN 2025,
  arXiv 2505.11716). Rising/sinking, spreading/enclosing and
  advancing/retreating are about how far the kinesphere is used. This
  answers "small, near HOME".
- **Animation principles as DMPs** (Hielscher, Bulling & Arras, IROS 2025,
  arXiv 2504.06735). Anticipation and follow-through as tunable DMPs. Code
  release unconfirmed.
- **Legibility** (Dragan & Srinivasa 2013). A design idea: aim and "look"
  moves at the audience.
- **Style spaces.**
  - Sripathy et al. (IROS 2022): a Valence-Arousal-Dominance cost function.
  - GenEM (Mahadevan et al., HRI 2024): an LLM writes parameterised motion
    specs.

## 3. Motion synthesis methods

- **Motion graphs** (Kovar & Gleicher 2002) and **motion matching**
  (Clavet, GDC 2016). Hub poses plus a nearest-neighbour search over
  `motion_labels` descriptors is a few dozen lines of numpy.
- **Music-to-dance models.**
  - EDGE (CVPR 2023): MIT code, but it needs a 16 GB GPU, runs on Linux,
    and outputs SMPL.
  - Bailando: non-commercial licence.
  - Both generate human bodies; retargeting to 6 DoF throws most of that
    away.
- **Dance to arm.**
  - "From Cage to Stage" (MOCO 2024): a PCA mapping from dancer space to arm
    space. Worth borrowing as a third retarget mode.
  - RMG (2025): code release unconfirmed.
- **Quality-diversity (MAP-Elites).** Keep the best solution per cell of a
  descriptor grid. With the descriptors set to our measured labels, the
  library covers the expressive space by construction. pyribs (MIT, 0.12
  released 2026-07) implements it.
- **Trajectory optimisation.** Drake has no Windows wheels. Optimising the
  parameters of a path family (CMA-ES through pyribs) is cheaper and reuses
  our IK, TOPP and collision check.

## 4. Capability maps to place motion

- Zacharias, Borst & Hirzinger (IROS 2007) introduced capability maps:
  reachable tool directions per voxel.
- Descendants: Reuleaux, ROS-Industrial REACH (Apache-2.0), RM4D (ICRA
  2025, MIT).
- In the literature they place the **base**. Placing **motion** with them
  is our step: erode the field by a path's size, and the voxels left are
  valid centres.

## 5. Tools and approaches

| Approach / tool | What | Licence | Fit | Cost |
|---|---|---|---|---|
| Houdini native (curves, VDB erode, Scatter, CHOPs Spectrum/Envelope/Lag/Spring, KineFX MotionClip / Motion Mixer) | Procedural paths, fields, music-reactive channels, clip arranging | have it | **High** | Low |
| BD Choreographer model | Move library, typed params, entry/exit in slices, BPM, tracks | design only | **High** | Low |
| Gannon Mimus / Quipt | Behaviour layer over perception; controller-side safety | design | **High** (installation phase) | -- |
| Motion graph / motion matching | Hub poses plus nearest-neighbour clip selection | idea (orangeduck ref. MIT) | **High** | Low (numpy) |
| Zacharias capability map | Directional reachability per voxel | paper | **High** (already built) | Low |
| Ruckig 0.19.4 | Jerk-limited state-to-state moves | MIT, cp313 win wheel | **High** | Low |
| pyribs 0.12 (MAP-Elites / CMA-ME) | Diverse library over a descriptor grid | MIT; needs numba / scipy / sklearn / pandas (wheels exist) | **High** | Med (run outside hython) |
| Laban Shape (2025) | Shape qualities for arm trajectories | paper | Med-High | Low |
| movement_primitives (DFKI) | DMP / ProMP | BSD-3 | Med | Low-Med |
| Cage-to-Stage PCA | Dancer space mapped to arm space | paper | Med | Low |
| AIST++ | 3D dance keypoints | CC BY 4.0 annotations + AIST DB terms; SMPL non-commercial | Med (test data) | Low |
| EDGE | Music-to-dance diffusion | MIT + data licences; GPU, Linux | Low now | High |
| Bailando | Music-to-dance | non-commercial | Skip | -- |
| Mimic / Robot Animator / KUKA\|prc | DCC keyframes, post-processor | various | Low (no Fairino) | -- |
| RoboDK | Offline programming, FR20 + Fairino post | commercial | Low-Med (cross-check) | Med |
| REACH / RM4D | Reach studies | Apache-2.0 / MIT | Low-Med (validation) | Med (ROS) |
| Drake, toppra | Trajectory optimisation / TOPP-RA | BSD / MIT, no Windows wheels | container only | -- |
| librosa / beat_this | Offline beat tracking | ISC / MIT | Med (music sync) | Med |
| madmom / essentia / aubio | Audio analysis | non-commercial / AGPL / GPL | avoid for commercial use | -- |

## 6. Recommendations

### (a) Place motion with the atlas

- Combine the baked fields into one score: capability × headroom × wrist
  distance × clearance, with a joint-margin threshold.
- Erode the score volume by each path family's extent (VDB Reshape), then
  scatter centres weighted by the score.
- Pick the tool direction from the reachable directions the capability
  index already samples. Stop defaulting to "down".
- Sample **stations / hub poses** over high-score regions at different
  levels, to replace HOME as the start and end of phrases.

### (b) Richer path families

- Lissajous, rose curves, spirals, helices, curves through atlas waypoints,
  and curves flowing along the capability gradient. Run them through the
  existing IK, TOPP and collision pipeline.
- Give each family Choreographer-style typed parameters: extent, pivot,
  entry/exit, plane, and an orientation mode (fixed / tangent / look-at).
- Use Ruckig for station-to-station moves only.
- Run pyribs MAP-Elites over family parameters:
  - descriptors = measured `motion_labels` (effort axes, level, extent,
    direction);
  - fitness = headroom / margin / smoothness.

### (c) Structured dances

- Adopt Choreographer's data model: typed parameters, entry/exit in slices,
  a BPM grid.
- Add motif and variation operators: repeat, mirror (±J1), retrograde,
  augment/diminish, fragment, accumulate, move to another station. These are
  standard composition devices; no specific source is cited.
- Add Laban Shape to the vocabulary.
- Replace HOME-only with hub poses, linked by Ruckig transitions.
- Music: CHOPs for energy curves, librosa or beat_this offline for tempo.

### Human motion

- Keep `direct` and `effort`, and add a PCA mode.
- Use AIST++ keypoints for tests and motifs. Check the AIST DB and SMPL
  terms before any commercial show.
- Audience phase, Mimus-style: OAK-D keypoints → efforts → motion matching
  over labels, behind a behaviour layer.

## 7. Suggested order

1. Atlas-driven placement: the eroded-field centre sampler, tool directions
   per centre, hub stations. Measure acceptance rate and coverage before and
   after.
2. Hub poses plus Ruckig transitions; drop HOME-only.
3. New path families with a Choreographer-style schema.
4. pyribs MAP-Elites over families and phrase specs.
5. Structured dances: motif/variation, Laban Shape, BPM sync.
6. Retargeting: AIST++, the PCA mode; later OAK-D with motion matching.

## Not verified

- Licences for Robo.Op and Reuleaux.
- Whether code is released for RMG and for Hielscher et al.
- RoboDK pricing.
- movement_primitives' scipy needs.
- Scatter density-in-volume on this Houdini build.
- KineFX MotionClip nodes with the URDF rig.

## Sources

- Gannon: [Mimus](https://atonaton.com/mimus), [Quipt](http://www.madlab.cc/quipt), [Manus](https://www.dezeen.com/2018/12/05/madeline-gannon-manus-robots-group/), [Robo.Op](https://github.com/peopleplusrobots/robo-op)
- [Bot & Dolly Box](https://nofilmschool.com/2013/10/behind-the-scenes-bot-and-dolly-box), [Mimic for Maya](https://github.com/AutodeskRoboticsLab/Mimic), [andyRobot](https://www.andyrobot.com/extolling-life-robot-animator), [KUKA|prc](https://robotsinarchitecture.com/kuka-prc/), [RoboDK FR20](https://robodk.com/robot/FAIRINO/FR20)
- [BD Choreographer](https://dev.bostondynamics.com/docs/concepts/choreography/choreographer.html), [move reference](https://dev.bostondynamics.com/docs/concepts/choreography/move_reference.html)
- [Breathless](https://arxiv.org/abs/2411.12361), [Huang Yi & KUKA](https://www.ted.com/talks/huang_yi_kuka_a_human_robot_dance_duet)
- [Knight & Simmons HRI 2015](https://www.ri.cmu.edu/publications/layering-laban-effort-features-on-robot-task-motions/), [Laban Shape](https://arxiv.org/abs/2505.11716), [DMP animation principles](https://arxiv.org/abs/2504.06735), [Dragan legibility](https://www.roboticsproceedings.org/rss09/p24.pdf), [Sripathy et al.](https://arxiv.org/abs/2203.02091), [GenEM](https://arxiv.org/abs/2401.14673), [robot dance survey](https://www.frontiersin.org/journals/computer-science/articles/10.3389/fcomp.2025.1575667/full)
- [EDGE](https://github.com/Stanford-TML/EDGE), [Bailando](https://github.com/lisiyao21/Bailando), [AIST++](https://google.github.io/aistplusplus_dataset/download.html), [SMPL licence](https://smpl.is.tue.mpg.de/modellicense.html), [Cage to Stage](https://dl.acm.org/doi/10.1145/3658852.3659090), [RMG](https://arxiv.org/abs/2503.09959)
- [Motion matching (orangeduck)](https://github.com/orangeduck/Motion-Matching), [Zacharias 2007](https://www.semanticscholar.org/paper/Capturing-robot-workspace-structure:-representing-Zacharias-Borst/53271ecd9d1adde34ef6b674a771b9b90b07bebb), [Reuleaux](https://arxiv.org/abs/1710.01328), [REACH](https://github.com/ros-industrial/reach), [RM4D](https://github.com/mrudorfer/rm4d)
- [pyribs](https://pyribs.org/), [movement_primitives](https://github.com/dfki-ric/movement_primitives), [Ruckig](https://github.com/pantor/ruckig), [toppra](https://github.com/hungpham2511/toppra), [Drake pip](https://drake.mit.edu/pip.html)
- Houdini: [KineFX MotionClips](https://www.sidefx.com/docs/houdini/character/kinefx/motionclips.html), [Motion Mixer](https://www.sidefx.com/docs/houdini/character/kinefx/motionmixer.html), [Beat CHOP](https://www.sidefx.com/docs/houdini/nodes/chop/beat.html)
