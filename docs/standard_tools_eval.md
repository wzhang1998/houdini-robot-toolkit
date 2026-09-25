# Hand-rolled vs standard tools: houdini-robot-toolkit (fr20-urdf-rig)

Date: 2026-09-25. I read the code only; nothing was edited or installed.
Package facts come from the PyPI JSON API and anaconda.org on the same day.

## Environment facts that decide most of this

| | Houdini 22 hython (py 3.13) | Player / plain Python on this PC (py 3.11.9) |
|---|---|---|
| numpy | 2.3.2 bundled | 2.2.4 |
| scipy | **not bundled** (cp313 win wheel exists, 1.18.1) | 1.17.1 |
| pybullet | no | 3.2.7 installed (PyPI has no Windows wheels; conda-forge does) |
| compiler | none. Only prebuilt wheels are realistic | same |

**Windows cp313 wheels on PyPI**

- Available: python-fcl 0.7.0.11 (BSD), ruckig 0.19.4 (MIT), trimesh 5.1.0 (MIT, pure Python), open3d 0.20.0, opencv-contrib-python 5.0.0.93 (Apache-2.0), depthai 3.10.0 (MIT), mediapipe 1.0.1 (Apache-2.0), roboticstoolbox-python 1.4.4, pytracik 0.0.3 (immature).
- Not available: toppra 0.6.10 (Linux wheels only), coal 3.0.3 (the hpp-fcl successor), pin (pinocchio 4.1.0), pybullet. coal and pinocchio are available for win-64 py313 **on conda-forge only**.

Houdini's Python 3.13 is standard CPython. A cp313-win_amd64 wheel installed with `pip install --target` into a folder on Houdini's path will load. Anything that needs compiling is effectively out.

---

## 1. Collision: `collision.py`, `cell_sop.py`, Pre-Flight "Cell"

**What we have now.** Each link is a capsule fitted to its URDF STL, so it encloses every vertex. The code computes an exact capsule-to-convex signed distance (ternary search along the segment) against box, cylinder, sphere and halfspace. It also checks self pairs with SRDF-style adjacency pruning and the slow, work and keep-out zone rules. It is pure Python and runs everywhere: in Houdini, in PDG and in the player.

**Standard alternatives**

- **python-fcl 0.7.0.11.** BSD, cp313 Windows wheel, released 2026-04. It does BVH mesh-to-primitive distance and collision (Box, Capsule, Cylinder, Sphere, Halfspace, Mesh, OcTree) and has broadphase managers. It is the same engine MoveIt uses. Its signed distance and penetration depth are weaker than coal's.
- **coal (hpp-fcl).** It has the best GJK/EPA signed distances, but on Windows it is conda-only. It does not fit Houdini's Python.
- **pybullet `getClosestPoints`.** It works, but it is effectively unmaintained (last release 2025-01) and has no Windows PyPI wheels. Not worth adopting.
- **trimesh.proximity.** Pure Python, but slow without embree or rtree. Useful for tests, not for per-frame checks.
- **Houdini VDB SDF.** VDB from Polygons followed by `hou.VDB.sample()` / `gradient()` or VEX `volumesample`. This is the right tool for scanned or arbitrary meshes inside Houdini. It is not available outside Houdini (no maintained pyopenvdb wheel).
- **MoveIt planning scene (FCL).** This is the industry-standard answer, and FAIR-INNOVATION/frcobot_ros2 already ships `fairino20_v6_moveit2_config`. It needs ROS 2, which on Windows means a Docker or WSL2 container. That is the plan's Stage 4 and does not belong in Houdini.

**Fit.** Capsules are a legitimate industry pattern: controller safety models and many planners use swept spheres or capsules. The closed-form distances are correct for convex shapes. Two real gaps:

- **Discrete sampling (a concrete finding).** `check()` tests clip frames only (24 fps). A point 1.5 m out on the arm moving at 120 deg/s travels about 0.13 m between frames. That is more than the default 0.05 m margin, so a thin obstacle can be jumped over.
- **Meshes and scans.** Any mesh or scan shape (plan item 2 of "robot_arm input 5") cannot be represented without hand simplification.

**Cost to switch.** Replacing the check with FCL means a compiled dependency in Houdini (possible via the wheel), rewriting the zone logic around it, and losing "runs anywhere, zero deps". The payoff would be marginal: our primitive distances are already exact.

**Benefit.** Using FCL as a validation oracle gives measured evidence that the capsules really bound the meshes, and that distances match at the mm level.

**Recommendation: Wrap-validate-against. Keep collision.py as the fast check.**

1. Add a test (plain Python with python-fcl) that samples random poses and compares the capsule clearance against FCL mesh-to-box clearance using the real STL links. Our value must always be ≤ FCL's (conservative), and the gap should be reported.
2. Fix the sampling. Sub-sample between frames so the largest link-point displacement per step stays below half the margin, or inflate the margin by the per-step sweep.
3. For scans, use Houdini VDB SDF inside Houdini (as the plan says) and sample it along the capsule axes. Do not grow collision.py into a mesh engine.
4. Use MoveIt/FCL in Stage 4 for transition planning only.

---

## 2. Retiming: `retime_topp.py`, `callbacks/ctrl_retime.py`

**What we have now.** A numerical-integration TOPP (forward and backward passes on x = u̇² under per-joint velocity and acceleration limits, rest-to-rest). It is followed by `fit()`, which stretches the timing locally wherever the 24 fps frames played through the player's cubic still exceed a limit. The Houdini callback is velocity-only accumulation feeding the same parameters.

**Standard alternatives**

- **toppra 0.6.10 (MIT).** The reference TOPP-RA implementation, maintained (released 2026-08). It handles velocity and acceleration (and torque), not jerk. It has **no Windows wheels**: building needs Cython and MSVC, and it depends on scipy, which Houdini lacks.
- **MoveIt TOTG (Kunz & Stilman), with Ruckig smoothing as a jerk post-process.** Standard in MoveIt 2, but ROS-only.
- **Ruckig 0.19.4 (MIT).** cp313 Windows wheels, maintained, jerk-limited, time-optimal state-to-state. **It is not a path parameteriser.** It will not follow a dense given joint path exactly. Intermediate waypoints in the Community edition are sent to Ruckig's **cloud API** (not real time, and your data leaves the machine); local waypoints need Ruckig Pro.

**Fit.** Our TOPP solves the same problem toppra solves, with the same velocity/acceleration model. It is about 200 lines, has tests, and is tuned to the real failure (a wrist-singularity acceleration spike slowing the whole clip). toppra would handle switch points more rigorously, but cannot run in Houdini without a build, and it still has no jerk. Ruckig does not replace it.

**Cost to switch.**
- toppra: a build toolchain and scipy in Houdini. High cost.
- Ruckig: one wheel. Low cost, but for a different job.

**Benefit.** Ruckig adds jerk-limited, time-optimal state-to-state moves where we currently use raised cosines and MoveJ:
- going to the clip's start and to HOME,
- clip-to-clip transitions in the clip library and show,
- a smooth ramp-down stop during ServoJ streaming (Ruckig's online, real-time mode is exactly this use case).

**Recommendation: Keep retime_topp. Validate it against toppra offline** in the Stage 4 Linux container or WSL, running the same paths and comparing duration and peak velocity/acceleration. **Adopt Ruckig for transitions and stops** (the plan already lists "Ruckig (a new dependency, ask first)"). Keep jerk limits blank until the FR20 value is known.

---

## 3. Player-side path/home check vs controller safety

**What we have now.** `home_path_check()` in `fairino_player.py` samples the joint-space MoveJ to HOME at 61 points against the env file, with obstacles and keep-out only and no tool. It is on hold, and the room is an estimate. The player already enforces velocity/acceleration conditioning.

**Standard alternatives on the controller (Fairino SDK "Security Settings", XML-RPC)**

- **Soft joint limits.** `SetLimitPositive` / `SetLimitNegative`, read back with `GetJointSoftLimitDeg`.
- **Contact detection.** `SetAnticollision` (level 1–10 or % per joint) and `SetCollisionStrategy` (error-stop, pause, continue, rebound…). `SetCollisionDetectionMethod` chooses current-based, dual-encoder or both. `CustomCollisionDetectionStart` sets joint and TCP thresholds.
- **Speed and power.** `SetVelReducePara` (TCP and joint speed caps). `SetPowerLimit`, which the manual lists as LA system only.
- **Safety parameter checksum.** `GetSafetyParamsCheckSum` returns a checksum of all safety parameters.
- **WebApp zones.** A safety plane (**TCP only**, defined by 3 plane points plus 1 side point) and "cuboid interference" zones (inside or outside, with a stop or continue strategy).
- **Singularity protection.** `SingularAvoidStart(protectMode, minShoulderPos, minElbowPos, minWristPos)`, in joint or Cartesian mode.
- **MoveIt** (Stage 4), for planning transitions offline.

**Fit.** The controller functions are the *runtime* layer. They work whatever the player's inputs are, they are what a safety review expects to see, and they need no dependency. What they cannot do:
- see arm links against room obstacles (the plane is TCP-only; zones are cuboids),
- predict a collision before it happens (contact detection reacts to one).

**The docs say nothing about whether zones, soft limits and SingularAvoid apply during ServoJ streaming. This must be tested on SimMachine and on the real arm.**

**Cost to switch.** No new code on the controller side. It is configured once in the WebApp by a person, since these are persistent safety settings.

**Recommendation: Replace the player's responsibility with the controller's.**

1. Set soft limits, collision level and strategy, the TCP speed cap, and cuboid interference zones for the keep-out boxes and walls in `envs/volvox_lab.json`. Do this in the WebApp.
2. At startup, the player **reads** them (`GetJointSoftLimitDeg`, `GetSafetyParamsCheckSum`) and refuses to stream if the checksum differs from the one recorded in the robot profile. The player should not *write* safety settings.
3. Test on SimMachine: stream a ServoJ clip into a soft limit and into a zone, and confirm the controller stops it.
4. Retire `home_path_check`, or keep it advisory only. Once the room is measured, the clip-start and HOME transitions can go through the same Pre-Flight in Houdini. Leave SingularAvoid off for ServoJ until tested: Pre-Flight already flags wrist-singularity proximity.

---

## 4. Room and robot frame alignment: `probe_env.py`, `env_from_points.py`

**What we have now.** The tool tip touches points under hand-guiding. Named groups are fitted to halfspace, box (min-area rectangle) or cylinder shapes. Our URDF FK is used, cross-checked against `GetActualTCPPose` (0.004 mm on SimMachine). The plane fit is a hand-rolled power iteration. The probe tip length is a manual `--tool-len`.

**Standard alternatives**

- **Fairino work object** (`SetWObjCoordPoint` ×3 then `ComputeWObjCoord`, 3-point). This produces **one frame**, not obstacle shapes. It is useful if the room frame should also exist on the controller, for example so the pendant, the safety zones and our env share a frame.
- **Fairino tool calibration** (4/6-point `SetToolPoint` / `ComputeTool`). This is the standard way to *measure* the probe TCP instead of typing `--tool-len`.
- **OpenCV ChArUco + `calibrateHandEye` (eye-to-hand)** for the OAK-D to robot base transform.
  - depthai 3.10 (MIT) has a cp313 Windows wheel, and the OAK's factory intrinsics can be read from the device, so there is no need to recalibrate the camera.
  - Caveat: search results report that `cv2.calibrateHandEye` / `calibrateRobotWorldHandEye` are missing from the OpenCV **5.x** Python wheels. Check `hasattr(cv2, "calibrateHandEye")`, or pin opencv-contrib-python 4.x.
  - GitHub issue #24871 reports that only the Park method is reliable on some inputs, so cross-check two methods.
- **Open3D 0.20** (cp313 Windows) `segment_plane` RANSAC for walls from OAK-D point clouds, or VDB in Houdini.

**Fit.** Nothing standard fits box, cylinder and halfspace obstacles from touch points in *our* env schema. That part is domain-specific and fine. The controller work object does not replace it.

**Recommendation: Keep probe_env and env_from_points.**

1. Use the controller's tool calibration for the probe tip and store the result.
2. Swap the power-iteration plane fit for `numpy.linalg.eigh`, since numpy is present in both runtimes. This is a small, optional change that makes the code more boring and obvious.
3. When the OAK-D arrives, **use OpenCV hand-eye, not our own code**.
4. Repeat the FK vs controller-TCP cross-check on the **real FR20**. Memory notes a URDF v6 vs controller link-length mismatch, and SimMachine ships as FR5.

---

## 5. IK: `ur_ik.py` / `urdf_rig.py`

**What we have now.** A closed-form UR-type IK derived from the URDF geometry. It returns all 8 branches, polishes each with Newton against the URDF as written, and drops any branch that FK does not verify. `urdf_rig.py` builds the KineFX rig.

**Standard alternatives**

- **ikfast.** This is the canonical analytic generator, but it needs OpenRAVE to generate the solver: an old toolchain, Linux-era and Python 2. For a UR-type arm its output is mathematically what we already have.
- **TRAC-IK.** A numeric solver that returns one solution (built on KDL). pytracik 0.0.3 has a cp313 Windows wheel but is immature.
- **pinocchio.** A dynamics library; its IK is a numeric CLIK loop. It is conda-only on Windows.
- **roboticstoolbox-python 1.4.4.** cp313 Windows wheel; numeric `ikine_LM` from the URDF. Handy as a test oracle.
- **Houdini KineFX IK Chains / Full Body IK.** Already measured to fail here: it jams on ±180 ranges, flips at the wrist, and restarts from a singular zero pose (see `ur_ik.py` docstring and README).
- **Fairino controller `GetInverseKin` / `GetForwardKin`** over XML-RPC. This is the *actual robot's* kinematic model, needs no dependency, and is reachable on SimMachine.

**Fit.** Our solver already does what ikfast would for this geometry, plus all branches and verification. That is better suited to branch-continuity retiming than TRAC-IK or pinocchio, which return one numeric solution.

**Recommendation: Keep.** Add a validation test against the **controller's** FK and IK on SimMachine configured as FR20, and later on the real arm, over random poses. The URDF-vs-controller link-length mismatch is a bigger risk than the solver itself. roboticstoolbox is an optional second oracle.

---

## 6. Motion labels (`motion_labels.py`) and retargeting (`retarget.py`)

**What we have now.**
- **Labels:** Laban Effort from joint and TCP kinematics. Weight comes from proximal joint speed (mass-weighted), time from acceleration/speed, space from chord/arc directness, and flow from the fraction of time still. It is calibrated on choreo phrases.
- **Retargeting:** keypoints are mapped to the TCP by centroid offset, axis mapping and a low-passed tool direction, then solved by IK per frame. There is also an "effort" style transfer. Note that there is **no Kabsch/SVD in the code** (grep finds none): the alignment is a fixed axis mapping.

**Standard alternatives**

- **No mature, standard Laban-Effort library exists.**
  - The accepted reference is *Larboulette & Gibet 2015, "A review of computable expressive descriptors of human motion"*. It defines weight as kinetic energy, time as acceleration, space as directness, and **flow as jerk**.
  - EMOKINE (GitHub andres-fr/emokine; Behav. Res. Methods 2024) extracts 32 kinematic statistics, including dimensionless jerk, quantity of motion and contraction. Check its licence before reuse.
  - The recent LMA papers (arXiv 2504.21154 / .21166) are research code.
- **Pose input**
  - MediaPipe Pose Landmarker (1.0.1, Apache-2.0, Windows wheel) gives 3D world landmarks from RGB.
  - depthai 3.x (MIT) for OAK-D capture. The community depthai_blazepose ran BlazePose on-device with depth, but it targeted depthai v2, so confirm it works with v3.
- **Data**
  - AIST++: CC-BY 4.0 annotations, `keypoints3d_optim` in COCO format and SMPL. It is already named in the retarget docstring and is a good public test set.
- **Rigid alignment**, if one is needed: `scipy.spatial.transform.Rotation.align_vectors` (Kabsch) in plain Python, or a 10-line numpy SVD in Houdini, since scipy is absent there.

**Fit.** Robot-arm Laban labelling from joint kinematics has no off-the-shelf equivalent. Our code is small, measured and tested (6/6 effort orderings).

**Recommendation: Keep both.**
1. Align the effort definitions with Larboulette & Gibet, or document where they deliberately differ. The main difference: our flow is "stillness fraction", while the literature uses jerk (jerk is also useful for the jerk-limit work).
2. Use MediaPipe and/or depthai for the OAK-D capture script rather than writing a pose estimator.
3. Use AIST++ `keypoints3d` as the retarget test corpus.
4. If a rigid registration is ever needed, use `align_vectors` or SVD rather than a new routine.

---

## Summary table

| # | Area | Recommendation | New dependency |
|---|---|---|---|
| 1 | Collision / Cell | Wrap-validate against python-fcl. Fix sampling between frames. VDB SDF for scans in Houdini. MoveIt in Stage 4 | python-fcl (tests only) |
| 2 | Retime | Keep TOPP and validate against toppra offline. Add **Ruckig** for transitions and stops | ruckig (MIT, wheel) |
| 3 | Player path/home check | **Replace** with controller safety config, read back and asserted by the player | none |
| 4 | Room alignment | Keep. Use controller tool calibration for the probe. OpenCV hand-eye for the OAK-D | opencv (later) |
| 5 | IK | Keep. Cross-check against controller `GetForwardKin`/`GetInverseKin` | none |
| 6 | Labels / retarget | Keep. Align with Larboulette & Gibet. MediaPipe/depthai for capture. AIST++ for tests | mediapipe/depthai (later) |

## Prioritised top 3

1. **Controller safety as the runtime guard (item 3).**
   - Configure soft limits, collision level and strategy, the TCP speed cap and cuboid interference zones on the controller.
   - The player reads back `GetSafetyParamsCheckSum` and refuses to stream on a mismatch.
   - Prove on SimMachine that these trip during ServoJ.

   This has the highest safety value and zero dependencies, and it retires the on-hold hand-built check.
2. **Make the collision check trustworthy (item 1).**
   - Sub-sample between frames: at 24 fps an arm point can move about 0.13 m, more than the 0.05 m margin.
   - Add a python-fcl mesh-vs-capsule oracle test.
   - Plan VDB SDF for scans rather than growing collision.py.
3. **Ruckig for state-to-state motion (item 2).** Use it for going to the start and HOME, for clip-to-clip transitions, and for a smooth abort ramp in the player. It is one MIT wheel with cp313 Windows builds. Keep retime_topp for path following.

Cheap bonus alongside 1: a FK/IK cross-check against the controller's own kinematics on the real FR20 (item 5). Collision, probing and retargeting all rest on the URDF matching the controller.

## Sources

- PyPI JSON API (versions and wheels): pypi.org/pypi/{python-fcl, coal, toppra, ruckig, pin, pybullet, trimesh, open3d, opencv-contrib-python, depthai, mediapipe, roboticstoolbox-python, pytracik}/json. anaconda.org conda-forge (coal, pinocchio, pybullet win-64).
- [Fairino Python SDK: Security Settings](https://fairino-doc-en.readthedocs.io/latest/SDKManual/PythonRobotSecuritySettings.html)
- [Fairino Python SDK: Movement (ServoJ, SingularAvoidStart)](https://fairino-doc-en.readthedocs.io/latest/SDKManual/PythonRobotMovement.html)
- [Fairino Python SDK: Common Settings (ComputeWObjCoord)](https://fairino-doc-en.readthedocs.io/3.7.6/SDKManual/PythonRobotCommonSettings.html)
- [Fairino Cobot manual: Safety (safety plane, interference zones)](https://manual.fairino.support/latest/CobotsManual/safety.html)
- [FAIR-INNOVATION/frcobot_ros2 (fairino20_v6_moveit2_config)](https://github.com/FAIR-INNOVATION/frcobot_ros2)
- [Ruckig: intermediate waypoints (Community cloud API vs Pro)](https://docs.ruckig.com/md_pages_2__intermediate__waypoints.html)
- [pantor/ruckig](https://github.com/pantor/ruckig)
- [toppra installation](https://hungpham2511.github.io/toppra/installation.html)
- [BerkeleyAutomation/python-fcl](https://github.com/BerkeleyAutomation/python-fcl)
- [hou.VDB (sample / gradient)](https://www.sidefx.com/docs/houdini/hom/hou/VDB.html)
- [OpenCV 5.0 ChArUco detection](https://docs.opencv.org/5.0/tutorials/objdetect/charuco_detection/charuco_detection.html)
- [opencv issue #24871 (hand-eye methods)](https://github.com/opencv/opencv/issues/24871)
- [EMOKINE](https://github.com/andres-fr/emokine)
- [EMOKINE paper (PMC)](https://pmc.ncbi.nlm.nih.gov/articles/PMC11525441/)
- [AIST++ download / licence](https://google.github.io/aistplusplus_dataset/download.html)
