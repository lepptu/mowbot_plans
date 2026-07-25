# Nav2 params review — 2026-07-25

**Scope:** `pi_ws/src/ros2-driver-converted/bringup/config/nav2_params.yaml` plus the wiring
around it (`navigation.launch.py`, `twist_mux.yaml`).
**Status: findings only — NOTHING applied yet.** Each finding has a fix plan and a
verification checklist; checkboxes track application.

## How this was verified (not just read)

- Installed copy (`install/hoverboard_driver/share/.../nav2_params.yaml`) diffed against
  source — **identical**, so the reviewed file is what runs.
- Parameter names checked against the **installed binaries** (Jazzy, Nav2 **1.3.10**):
  `strings` on `libnav2_regulated_pure_pursuit_controller.so` and
  `libcontroller_server_core.so`. Not a docs-version guess.
- cmd_vel chain checked on the **live running graph** (`ros2 topic info --verbose`,
  2026-07-25, nav stack up).
- Launch trace: `mowbot-launch-bringup.service` → `bringup.launch.py` →
  `navigation.launch.py` (composed container) + `diffbot.launch.py` (twist_mux).

Launched lifecycle nodes: controller, smoother, planner, behavior, bt_navigator,
waypoint_follower, velocity_smoother, collision_monitor. **Not launched:** docking_server,
route_server, map_saver (their yaml sections are dormant — docking is pending per the
Docking plan, fine).

---

## NP1 (HIGH) — collision_monitor is fully disconnected

**Live-verified evidence (2026-07-25):**

- CM input `/cmd_vel_smoothed`: **0 publishers** — `navigation.launch.py` remaps the
  velocity_smoother's output `cmd_vel_smoothed → cmd_vel_nav`, so nothing feeds CM.
- CM output `/cmd_vel`: **0 subscribers** — twist_mux's navigation input is `cmd_vel_nav`.
- Actual actuation chain: controller/behaviors → `/cmd_vel_raw` → velocity_smoother →
  `/cmd_vel_nav` → twist_mux (nav prio 10, e-stop lock 255) → `/cmd_vel_out` →
  `/hoverboard_base_controller/cmd_vel`.

The node runs, is lifecycle-active, and can neither slow nor stop the robot. The
`FootprintApproach` 1 s time-to-collision guard is inert.

**Why the robot still avoids obstacles fine:** avoidance never came from CM. It is
(1) obstacle/bumper layers marking costmaps, (2) `ComputePathToPose` replanning at 1 Hz
routing around marks, (3) RPP `use_collision_detection` aborting FollowPath on imminent
collision, (4) the BT recovery ladder. All healthy. CM is a *redundant last-line* guard on
raw scan points — its absence is invisible until the primary layers miss something
(e.g. the blind window right after a recovery `ClearEntireCostmap` while the costmap is
empty). The old `navigation.launch VARA2.py` (global `SetRemap /cmd_vel → /cmd_vel_nav`)
had CM live; the wiring broke in the launch-file switch.

**Important cross-ref:** BT_REVIEW R11's `LidarModeManager` flips `scan.enabled` on CM
precisely because "its 1 s source_timeout would otherwise stop the robot" — that belief is
currently false (CM can't stop anything). The R11 handling exists and becomes load-bearing
the moment NP1 is fixed, but it has **never been exercised against a live CM**.

**Fix plan — Option A (recommended): rewire, 2 edits**

1. `navigation.launch.py`, velocity_smoother entry: **remove** the
   `('cmd_vel_smoothed', 'cmd_vel_nav')` remap (keep `('cmd_vel', 'cmd_vel_raw')`).
   Smoother then publishes its default `/cmd_vel_smoothed`.
2. `nav2_params.yaml` collision_monitor: `cmd_vel_out_topic: "/cmd_vel"` → `"cmd_vel_nav"`
   (`cmd_vel_in_topic: "cmd_vel_smoothed"` already correct).

Resulting chain: … smoother → `cmd_vel_smoothed` → **CM** → `cmd_vel_nav` → twist_mux.
No twist_mux or driver changes. Teleop paths (joy/web/foxglove) unchanged — they join at
the mux and deliberately bypass CM.

Failure direction is safe: if CM dies, `cmd_vel_nav` goes silent → mux 0.5 s nav timeout →
wheels stop.

**Option B (not recommended):** decide CM is unnecessary (RPP veto + bumper + firmware/mux
e-stop deemed sufficient), delete it from launch + yaml, and update BT_REVIEW R11 notes so
the codebase stops believing it exists. Cheaper, but loses the raw-scan redundancy.

**Verification checklist after rewire (bench, wheels off ground / blocked):**

- [ ] `ros2 topic info /cmd_vel_smoothed --verbose` → 1 pub (velocity_smoother), 1 sub (CM)
- [ ] `ros2 topic info /cmd_vel_nav --verbose` → 1 pub (**collision_monitor**), 1 sub (twist_mux)
- [ ] Nav goal toward an obstacle → speed tapers (approach polygon) *before* RPP abort;
      `/collision_monitor_state` shows the polygon activating
- [ ] `lidar_enabled=false` (web toggle): robot **still drives** in bumper-only mode —
      first real test of R11's CM `scan.enabled` handling + the 2 s re-apply watcher
- [ ] LiDAR idle power-off + F9 `reassertPower()` path still behaves on mission start
- [ ] E-stop press → all motion stops (mux lock, unchanged path)
- [ ] Kill CM (`ros2 lifecycle set /collision_monitor shutdown` or component unload) while
      driving → robot stops within ~0.5 s (mux timeout)

- [ ] **Applied** (launch + yaml, build, restart) — date: ______

## NP2 (HIGH) — 9 phantom FollowPath params, silently ignored by Nav2 1.3.10

Checked against the installed RPP `.so`. **Not declared** (from newer Nav2 docs —
Kilted/Rolling era): `use_dynamic_window`, `max_linear_vel`, `min_linear_vel`,
`max_linear_accel`, `max_linear_decel`, `max_angular_vel`, `min_angular_vel`,
`max_angular_decel`, `max_allowed_time_to_collision`.

**Real** and in effect: `desired_linear_vel` (NOT set → **default 0.5 m/s** — the "N11
CEILING" comment is currently false; the observed 0.4 cap is the velocity_smoother
clipping), `max_allowed_time_to_collision_up_to_carrot` (NOT set → default **1.0 s**,
which is *good* — the intended 0.4 s would give a 0.16 m warning distance ≈ exactly the
stopping distance at the smoother's 0.5 m/s² decel, i.e. zero margin; the yaml comment
"Nimi lyhentynyt!" is backwards — the name got longer), `max_angular_accel: 4.0` and
`rotate_to_heading_angular_vel: 1.0` (both real, drive rotate-to-heading).

Mission SpeedLimit (N11) still works — it scales the internal desired speed down, so
mowing at slider speed was never affected. Unlimited segments command 0.5 and saturate the
smoother at 0.4.

**Fix plan (yaml only, FollowPath block):**

- Delete the 9 phantom lines (incl. `min_linear_vel: -0.3`, which also contradicted
  `allow_reversing: false`).
- Add: `desired_linear_vel: 0.4` and `max_allowed_time_to_collision_up_to_carrot: 1.0`
  (keep ≥ 1.0 — see margin math above).
- Fix the two wrong comments; keep `max_angular_accel` + `rotate_to_heading_angular_vel`.

Expected field delta: essentially none (smoother already enforced 0.4); approach/regulated
scaling gets slightly cleaner because RPP no longer commands above what executes.

**Verification:**

- [ ] `ros2 param get /controller_server FollowPath.desired_linear_vel` → 0.4
- [ ] `ros2 param describe` shows no leftover phantom names being *declared-by-yaml only*
- [ ] Straight transit: speed unchanged (~0.4); mowing at slider speed unchanged
- [ ] **Applied** — date: ______

## NP3 (MED) — velocity-scaled lookahead is a no-op: constant 0.6 m

`lookahead = clamp(speed × 1.5 s, 0.6, 1.5)`. Top speed 0.4 → 0.6; mowing speeds → still
clamped at 0.6. `lookahead_dist: 0.8` is unused while scaling is on; `max_lookahead_dist`
would need 1.0 m/s. Not a bug — a tuning illusion.

**Plan:** no change now. When tuning lane tracking / corner cutting, the *real* knob is
`min_lookahead_dist` (try 0.45–0.5 for tighter turns), or set
`use_velocity_scaled_lookahead_dist: false` and tune `lookahead_dist` directly. All
dynamic — live-tunable from web UI (F24 pattern) for A/B on a headland turn.

- [ ] Decision recorded (tune / leave as-is) — date: ______

## NP4 (MED) — global obstacle-layer marks are effectively permanent

Global obstacle layer marks ≤ 2.5 m but clears only by raytracing ≤ 3.0 m; once the robot
leaves, a transient mark (person, wheelbarrow, grass tuft) persists on the map-frame
costmap for the rest of the session unless re-observed through that spot. Accumulation can
block transit planning (the F6 route-skip failure mode).

**Plan:** KEEP the global obstacle layer (removing it would kill planner detours — the
avoidance behavior that demonstrably works). Add a mission-side periodic clear of
`global_costmap/clear_entirely_global_costmap` (infra already exists — BT ClearingActions
and LidarModeManager both call it). Candidate triggers, pick one:

- (a) at segment boundaries (SegmentIteration start), or
- (b) on transit start, or
- (c) simple timer (e.g. every 10 min).

(a) or (b) preferred — clears happen at natural pauses. Watch F6 skip stats before/after.

- [ ] Trigger chosen: ______  — [ ] Applied (mowing_navigation) — date: ______

## NP5 (LOW-MED) — timeout margins

- `planner_server.costmap_update_timeout: 1.0` **equals** the global costmap's 1.0 s
  update period → marginal race, occasional ComputePathToPose timeout possible.
  **Plan: 1.0 → 2.0.** Free.
- `controller_server.costmap_update_timeout: 0.30` vs local costmap 0.2 s period → 100 ms
  slack; a loaded Pi could trip a spurious FollowPath failure.
  **Plan:** first check evidence:
  `journalctl -u mowbot-launch-bringup | grep -i "timed out"` — if it ever fired, 0.30 →
  0.5; if never, optional.
- Costmap `transform_tolerance: 1.0` (both) accepts up to 1 s stale TF = 0.4 m at full
  speed, vs 0.1 s in RPP/behaviors. Presumably raised to silence warnings on the Pi.
  **Plan:** leave, but document as deliberate; optional experiment 1.0 → 0.5 and watch for
  TF warnings.

- [ ] Applied (rides along with NP2's yaml edit) — date: ______

## NP6 (LOW) — local costmap ranges exceed the 3×3 m window

`obstacle_max_range: 3.5` / `raytrace_max_range: 4.0` on a rolling window whose edge is
1.5 m from the robot. Harmless, just dead range. **Plan:** trim to ~2.2 / 2.5 (diagonal
reach) or grow the window — cosmetic, bundle with NP2.

- [ ] Applied — date: ______

## NP7 (COSMETIC) — dead/dormant config

- `voxel_layer` block in local_costmap is **not in the plugins list** → never
  instantiated. Delete (or mark `# UNUSED`).
- `docking_server` / `route_server` / `map_saver` sections: configured, not launched —
  intentional (docking pending). Add a one-line comment so the next review doesn't flag it.
- `velocity_smoother` `odom_topic`/`odom_duration` unused with `feedback: OPEN_LOOP`.
- `min_linear_vel: -0.3` contradiction disappears with NP2.

- [ ] Applied — date: ______

---

## What checked out FINE (verified, no action)

- `current_goal_checker` **exists** in 1.3.10 (binary-checked) — the yaml comment is right.
- `mowing_goal_checker` yaw 6.28 "ignore yaw" trick works (SimpleGoalChecker shortest-angle
  test always passes).
- Stamped cmd_vel consistent end-to-end (all publishers + twist_mux `use_stamped: true` +
  ros2_control side).
- Bumper marking-only layer design incl. the 500 m `obstacle_max_range` rationale
  (odom-origin cloud) — sound, in both costmaps.
- Keepout on GLOBAL costmap only — matches the field-round-6 lesson; correct.
- Local 5 Hz / global 1 Hz costmap frequencies — correct split, stock pairing; staleness
  at 0.4 m/s is well inside margins. Don't raise global (1 MB full-costmap publish per
  cycle); local→10 Hz only if speeds ever rise.
- LidarModeManager's CM `scan.enabled` handling — correct design, becomes live with NP1.
- E-stop chain: twist_mux lock (prio 255) + 1 s telemetry deadman — untouched by any fix
  here.

## Suggested order + deploy recipe

1. **NP2 + NP5 (+NP6/NP7 opportunistic)** — pure `nav2_params.yaml`, no behavior change
   expected.
2. **NP1** — launch + yaml rewire, then the full bench checklist (this is the one that
   changes runtime wiring; do it alone).
3. **NP4** — mowing_navigation change, separate deploy.
4. **NP3** — only as part of deliberate lane-tracking tuning.

Deploy per build rules (one package per command):

```
cd /home/ros-pi/pi_ws
colcon build --packages-select hoverboard_driver        # config + launch changes
sudo systemctl restart mowbot-launch-bringup.service
# NP4 later: colcon build --packages-select mowing_navigation
# sudo systemctl restart mowbot-launch-mow-mission.service
```
