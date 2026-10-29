# TurtleBot3: map-derived knowledge, then navigation without the obstacle map

Handoff for GPT 6 Astra. Updated 2026-10-26, Asia/Bangkok.
Technical details use English to keep the handoff compact. User-facing UI and explanations should use Thai.

## Continuation instructions

- Read this file before coding. It replaces previous designs and deleted implementations.
- Use the installed `caveman` skill for concise chat: `/home/aorus-ubun/.codex/skills/caveman/SKILL.md`.
- The skill exists and was read in this session. Do not reinstall or overwrite it unnecessarily.
- In another environment, discover whether this skill exists before claiming it is installed.
- Keep commands, units, negations, validation and technical facts intact. Persisted documentation should remain readable.
- User authorized implementation on 2026-10-26. Current code is in `tb3_knowledge/`.
- User has no robot available. Validate with offline simulation and isolated fake ROS only; physical commissioning remains pending.
- Never launch motion, install packages, or change the robot as part of merely reading this plan.
- Read applicable `AGENTS.md` files. Do not spawn agents unless the user or applicable instructions authorize delegation.

## Current state

- Workspace: `/home/aorus-ubun/CMU/P.4/271401-Artificial_Intelligence_for_Robotics`.
- User explicitly deleted `Mini_project/`, `Mini_project_v2/`, `Mini_project_v3/`, including all programs and knowledge.
- These directories are absent. Do not refer to old scripts or test results as current deliverables.
- Original maps remain: `/home/aorus-ubun/tb3_maps/arena.yaml` and `arena.pgm`.
- Verified YAML: trinary, resolution `0.050 m/pixel`, origin `[-1.471, -1.730, 0]`, negate `0`, occupied threshold `0.65`, free threshold `0.196`.
- Preserve originals. Inspect the actual map before choosing crop, start offsets or coordinate transforms.
- New implementation now exists in `tb3_knowledge/`, with two entry points and a Thai README.
- Example knowledge is generated from the saved arena in simulation mode and cannot drive a physical robot.
- Offline tests, real-display GUI checks and isolated fake ROS cover implementation behavior; real hardware performance remains unvalidated.

## Confirmed requirements

- TurtleBot3 Burger, LDS-02, Raspberry Pi 4; Ubuntu 24.04 / ROS 2 Jazzy in the project context.
- Arena: rectangular, approximately 2–2.5 m per side. Survey and execution use the same obstacle layout.
- Original assignment: robot 1 surveys; robot 2 navigates using knowledge and Start–Goal.
- Start–Goal may be known **before knowledge export**. Do not preserve the obsolete export-before-mission restriction.
- User wants all project computation on the notebook: SLAM, localization, generation, navigation, visualization and logging.
- Pi runs standard bringup/drivers/OpenCR interface, publishes sensors/odometry/TF, receives velocity through ROS topics.
- No project ZIP, knowledge file or custom project guard needs copying to Pi.
- Two clear Python entry points: Program 1 generates knowledge; Program 2 consumes it.
- Knowledge may contain arena boundary, Start, Goal and scalar navigation parameters.
- Knowledge must not contain interior-obstacle positions, shapes, counts, occupancy grid or a spatial encoding of those obstacles.
- LiDAR may detect obstacles during navigation. "No obstacle knowledge" means no prior obstacle geometry from the survey.
- Program 2 must compute navigation itself, rather than calling Nav2 `NavigateToPose`/planner/controller for the task.
- RViz2 remains a visualization/interaction tool. RViz2 is not the navigation algorithm.

## Proposed workflow

### Phase A: SLAM and save

1. Bring up the survey robot. Run SLAM and RViz2 on the notebook.
2. Survey manually or automatically. Cover occluded areas and inspect map quality.
3. Save YAML/PGM. Preserve unknown cells; do not treat unknown as free.

### Phase B: Program 1, `create_knowledge.py`

1. GUI selects YAML; resolve its image path automatically, with a separate PGM picker if needed.
2. Preview the full map. Detect four outer walls using exterior-connected unknown and the largest free component.
   Fit a rotated rectangle to inner wall interfaces and fill its dimensions/origin/yaw automatically.
   Require support across all four sides; if evidence is incomplete, label the known-cell bbox as provisional.
   Keep manual three-point rectangle selection as a fallback.
3. Derive dimensions from selected geometry and resolution. Manual dimensions are optional validation, not mandatory input.
4. Place the execution robot at Start. Localize it against the full map, e.g. with AMCL.
5. Let the user confirm localization quality and select Goal in the same frame.
6. Validate Start footprint and Goal clearance/connectivity using the full map in this phase.
7. Simulate the custom navigation algorithm on the map with several parameter sets and disturbance seeds.
8. Select parameters, display results, then export a single `knowledge.json` without obstacle geometry.
9. Capture the execution robot's initial pose and odometry reference together while stationary.

Full-map localization before handoff is the proposed interpretation of setup assistance, not a verified instructor ruling.
If instructor rules forbid any full-map assistance to robot 2, replace this step with a measured/configured Start pose.
Do not silently use continuous full-map localization during Phase C.

### Phase C: Program 2, `run_knowledge.py`

1. Load the exported file. The robot remains at the same Start with unchanged odometry reference.
2. Check robot identity, frames, fresh sensor data and continuity of the odometry session.
3. Dry run by default: diagnostics and visualization, no motor publisher.
4. Explicit Start/`--drive` enables navigation; clearly announce the transition.
5. Automatically manage a command guard on the notebook. No separate guard command is needed from the user.
6. Use live LiDAR, odometry and permitted knowledge to produce velocity commands.
7. Display Start, Goal, robot pose, arena boundary, local selected path and executed pose trail in RViz2.
8. Log decisions, requested velocities, sensor age, estimated error, stop reasons and knowledge fingerprint.

Do not load the original map, static costmap, pretrained spatial obstacle field or AMCL-corrected pose during navigation.
Stop full-map AMCL/SLAM localization before handoff. Retain an explicit initial frame anchor; do not depend on stale TF.
If a different physical robot executes, localize and capture odometry for **that robot**; never reuse the survey robot's odometry.
If the robot is moved manually, bringup resets, or odometry jumps, invalidate the anchor and repeat setup.

## Knowledge format

One versioned JSON file. Proposed groups:

```text
schema_version
mission
  arena_frame: width_m, height_m, origin/axis convention
  start: x_m, y_m, yaw_rad
  goal: x_m, y_m
initial_reference
  execution robot identity, odom/base frames
  synchronized start pose, odometry pose and timestamp/session checks
  odometry publisher endpoint identity (nonspatial continuity metadata)
policy
  goal, clearance, turning and revisit weights
  initial bypass-side preference
  boundary-follow distance
  side-switch hysteresis and minimum side-hold duration
  progress window and minimum progress
  local lookahead distance
hardware
  footprint, fixed safety margin, speed/acceleration/braking limits
  sensor mounting/frame conventions and freshness limits
provenance
  training summary, schema/controller version, integrity fingerprints
```

- Exact field names and ranges should be finalized before implementation.
- Policy values are global scalars, not coordinate-indexed rules that reconstruct obstacle placement.
- Arena boundary is allowed; it does not declare its interior free.
- Footprint and minimum safety margin are measured constraints, not tunable shortcuts for winning simulations.
- No saved global route or obstacle-derived waypoint list in the initial design. Use parameters as the transfer mechanism.
- Scene fingerprints identify a source; they contain no geometry and are not authentication.
- Start–Goal and hardware are context, not learned knowledge. Report policy learning separately.

## Proposed algorithm

Working name: Knowledge-Guided Gap Navigation (KGGN). A project name, not a novelty claim.

Each control cycle:

1. Validate scan/odom/TF freshness and motion-state consistency. On failure, command zero.
2. Estimate pose from the initial reference and current odometry.
3. Transform LiDAR to the current base frame; account for mounting and motion during scanning when supported.
4. Find candidate free directions/gaps using current LiDAR. Check swept footprint, turning feasibility and arena bounds.
5. Reject infeasible candidates before scoring. A single clear ray is not a traversable gap.
6. Normalize costs, then minimize:

   `C = w_goal * E_goal + w_clear * E_clear + w_turn * E_turn + w_revisit * E_revisit`

   `E_goal`: deviation from current desired heading. `E_clear`: penalty for short free clearance.
   `E_turn`: change from previous heading. `E_revisit`: return toward recently unsuccessful locations.

7. Use explicit modes:
   - GOAL: favor progress toward Goal.
   - FOLLOW: hold a bypass side and follow a detected boundary; leave when direct progress becomes feasible.
   - RECOVER: detect insufficient progress, penalize failed choices, try another feasible side; stop on exhausted recovery.
8. Apply hysteresis to reduce left/right oscillation.
9. Convert selected heading to linear/angular velocity; rotate first when needed, respect footprint and braking limits.
10. Update temporary memory and publish command, selected local path and status.

Temporal memory comes from this run, not from the surveyed obstacle map. It may contain recent poses and attempted choices.
Plan handling of invalid/out-of-range LiDAR explicitly; do not blindly mark unknown measurements as clear.
Unknown or blocked directions may require stopping. Recovery must not command blind reversing/rotation.

### Parameter selection in Program 1

- Begin with a small reproducible candidate search, not a neural network or unimplemented RL claim.
- Use the actual disclosed Start–Goal. Do not claim unseen-mission generalization from mission-specific tuning.
- Select for collision avoidance and task completion first, then time, distance, oscillation and recoveries.
- Include sensor/odometry/latency disturbances. Use independent evaluation seeds after selection.
- Record failed trials. Never choose parameters using the evaluation results and then label those results independent.
- Keep hard hardware limits fixed. Show actual runtime and evaluated candidates; do not promise a fixed training duration.

### Difference from typical Nav2 use

- Typical mapped Nav2 setup uses a global environment representation, planner and path-following controller.
- This design selects live gaps/headings, uses explicit bypass/recovery state and transferred scalar parameters.
- It directly computes velocity without invoking Nav2 navigation actions/planner/controller.
- Nav2 is configurable and supports custom plugins; these ideas are not categorically impossible in Nav2.
- Gap scoring and boundary following have prior art. Do not claim a new academic algorithm without a literature review.
- No guaranteed shortest path, completeness, faster execution, lower CPU or higher safety than Nav2.

## Coordinates, localization and visualization

- YAML origin and image bounds are not automatically physical arena origin and boundary.
- Support a rectangle rotated relative to SLAM axes. Preserve a documented planar transform; do not arbitrarily rescale maps.
- Use one chosen base reference consistently for localization, footprint, Start and odometry.
- Initial localization has uncertainty. Confirm scan alignment and pose stability rather than accepting the first pose.
- Program 1 now starts a stationary global AMCL search automatically. Wait for the matching map, active lifecycle, fresh sensors, concentrated particle weights and stable pose before capturing Start; keep manual RViz initialization available when global search remains ambiguous. Re-search invalidates the previous Start and training result. No setup motor publisher is created.
- Users may click an approximate Start in Program 1 before localization. The optional map-coordinate positional hint bounds coarse and refined scan matching to an adjustable radius (default 0.35 m); heading remains unconstrained. A hint is never a captured odometry reference. Changing it invalidates pending search results and the previous capture; clearing it restores full-map search.
- Odometry drifts after full-map localization stops. Report arrival as an estimate and measure physical Goal error separately.
- RViz should show boundary markers, Start/Goal, robot, current scan, selected local path and estimated executed trail.
- An optional straight Start–Goal line is a reference, not a collision-free global plan.
- Do not display an empty occupancy grid as proof that the arena interior is traversable.

## ROS integration and stopping

- Inputs: `/scan`, `/odom`, `/tf`, `/tf_static`; verify actual frame names and QoS.
- Check `/cmd_vel` type from the installed bringup. Support both Twist and TwistStamped; do not assume a version default.
- Use separate internal requested-command and final velocity topics. Prevent competing teleop/Nav2 command publishers.
- Keep notebook and Pi clocks synchronized for header-age checks; use monotonic time for local watchdogs.
- Guard limits velocity and expires stale controller commands while notebook-to-robot transport works.
- A notebook-side guard **cannot guarantee stopping when Wi-Fi is lost or the notebook dies**.
- Inspect the installed robot driver's command timeout separately. Do not assume ROS/OpenCR heartbeat guarantees command expiry.
- Keep a reachable physical stop. Initial trials use low speed and explicit drive enable.
- Stop/shutdown/error paths should send zero and clean up managed processes. Test these with fake hardware first.

## Implementation checklist

Implemented in `tb3_knowledge/`. Keep user-facing files few:

```text
tb3_knowledge/
  create_knowledge.py
  run_knowledge.py
  README.md
  requirements.txt
  core/       # Internal reusable code, not extra user commands
  tests/
  output/     # Generated knowledge/logs; do not commit maps or logs by default
```

1. [x] Define schema, coordinate/anchor contracts and input validation.
2. [x] Implement map loading, rectangle selection and Program 1 preview. Test real arena without modifying originals.
3. [x] Implement controller and simulator first; separate algorithm inputs from simulator ground truth.
4. [x] Evaluate all selected parameter sets and add single-file export; no training time limit setting.
5. [x] Add localization/setup integration and synchronized handoff capture; verify with fake ROS.
6. [x] Add Program 2 ROS adapter, managed PC guard, dry run and RViz configuration.
7. [x] Run isolated fake-hardware integration tests and document the two-program workflow.
8. [ ] Real-robot commissioning requires available hardware, explicit authorization and measured settings.

Avoid extra version directories, mandatory ZIP packaging, duplicate launchers or requiring four manual JSON files.
Keep simulated references distinct from real sensor estimates. Never send test velocities to the live robot domain.

## Acceptance and experiments

- GUI: select YAML/PGM, define arena, confirm Start/Goal, generate/export; show progress and recoverable errors.
  Draw scaled robot-radius circles and clearance rings at both Start and Goal. Each simulation button replaces
  the previous path/result and displays that run's simulated travel time and distance, without a comparison table.
  Keep visible pass/fail/pending feedback beside mission validation and clear success when inputs change.
  Lay out the latest-run metrics in four columns, or two columns in a narrow window, with readable values.
- Export: no obstacle geometry/grid/path; preserve policy/context and validate schema/fingerprints.
- Runtime: still loads when original maps are unavailable; no full-map subscriptions or navigation actions.
- Handoff: unchanged pose/odom succeeds; wrong robot/reset/stale anchor rejects and stops.
- Sensors/control: test malformed/stale scans, missing TF, odometry jumps, braking, bound crossing and no feasible gaps.
- Lifecycle: dry run creates no velocity publisher; managed guard cleanup, controller failure and stop service work.
- Use a separate ROS test domain and unique fake topics. Test both Twist types and competing publishers.
- Main comparison: same custom controller with fixed default parameters versus map-selected parameters.
- Additional reference: Nav2 with a full map, explicitly marked as having more prior spatial information.
- Measure success rate, physical Goal error, collisions, time, distance, recoveries, CPU/RAM and control latency.
- Report training cost separately. Do not infer superiority from one successful mission.

## Primary references

- RViz role and interaction: https://github.com/ros2/rviz/blob/rolling/rviz2/doc/index.rst
- AMCL map-based localization: https://github.com/ros-navigation/navigation2/blob/main/nav2_amcl/README.md
- ROS frames and odometry drift: https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0105.rst
- Nav2 planner/controller concepts: https://ros-navigation.github.io/mkdocs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/navigation_plugins/setup_navigation_plugins/
- TurtleBot3 bringup: https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/

Check installed package behavior and current official documentation when implementing.

## Prompt to hand to Astra

```text
อ่าน TURTLEBOT3_PLAN.md แล้วอ่านและใช้ skill caveman จาก path ที่ระบุ
ตอนนี้ให้ [คุยรายละเอียดต่อ / เริ่มพัฒนาตามแผน] ตามที่ฉันเลือก
ใช้ notebook คำนวณทั้งหมด หุ่นรับส่ง ROS topics เท่านั้น
มีสองโปรแกรม: UI สร้าง knowledge ไฟล์เดียว และโปรแกรมนำทางพร้อม RViz
knowledge ไม่เก็บสิ่งกีดขวาง Program 2 ไม่อ่านแมพเดิมและไม่เรียก Nav2 เดินแทน
โค้ด Mini_project ทุกเวอร์ชันถูกลบแล้ว เริ่มจากสถานะไฟล์จริง อย่าอ้างว่ายังมีโค้ดเก่า
เก็บแผนที่ต้นฉบับไว้ ไม่ติดตั้งหรือสั่งหุ่นเคลื่อนที่จนกว่าจะได้รับคำสั่ง
```
