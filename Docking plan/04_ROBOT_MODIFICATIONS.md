# 04 — Robot (Mowbot) Modifications

> Status: **DOCKING WORKS — first autonomous dockings 2026-09-12** (§10.12,
> 4/5 that day, 5/5 undocks). Robot side implemented: Nav2 docking server
> + custom `MowbotChargingDock` (seat microswitch + lidar V detection),
> `dock_v_detector`, bridge `dock_manager` (Dock/Undock/Cancel, sequenced
> undock, drive-out guard, automatic heading seed from the V). §2.1 is the
> as-deployed config, §10 the review + field findings (§10.7–§10.12), §11
> the TODO (remaining: stall handling, §11.4 mission gate, web UI Phase
> B/C, P4). Originally planned 2026-07-10, updated 2026-07-31. §1 charge-path
> hardware is **built and verified** (0 V on pads undocked); references
> aligned with the revised [01](01_HARDWARE.md)/[02](02_ARDUINO_FIRMWARE.md)
> (AC-side switching, dock state renumbering). Part of the docking master
> plan ([README.md](README.md)). Everything that changes on the robot:
> hardware, Nav2 config/launch, the bridge `dock_manager`, and mission
> integration. The robot's Arduino Nano and its firmware are **not
> touched** — the charge path is passive on the robot side.

## 1. Hardware changes (details/rationale in 01 §4–5) — DONE 2026-07

Built and verified: plates → 5 A fuse → ideal-diode module → battery
charge input (parallel with the existing charge jack, through the BMS
charge port); **0 V on the pads whenever undocked confirmed** — the
exact property the dock side (01 §5) relies on.

- Two contact plates on the **rear** chassis face (as built; the robot
  backs into the dock — README D10 rev. 2026-09-06), asymmetric geometry
  for polarity safety.
- Still open (needs the dock to exist): plates wipe the dock springs
  over the full funnel tolerance (±3 cm entry) — part of the 01 §8
  assembly checks.

## 2. Nav2: enable `opennav_docking` (installed, v1.3.10)

### 2.1 `nav2_params.yaml` — new `docking_server` section (draft)

```yaml
# AS DEPLOYED 2026-09-12 (first autonomous dockings) — bringup/config/nav2_params.yaml
docking_server:
  ros__parameters:
    enable_stamped_cmd_vel: True
    controller_frequency: 20.0
    initial_perception_timeout: 5.0
    wait_charge_timeout: 15.0          # charging starts ~200 ms after seating (02 §8)
    dock_approach_timeout: 60.0
    undock_linear_tolerance: 0.05
    undock_angular_tolerance: 0.1
    max_retries: 3
    base_frame: "base_link"
    fixed_frame: "odom"                # the V detection is re-measured every cycle → the smooth short-term frame (§10.10)
    dock_backwards: true               # robot BACKS in; the plugin flips the refined yaw to the server's "into the dock" convention (§10.8)
    dock_prestaging_tolerance: 0.5
    dock_plugins: ["mowbot_dock_plugin"]
    mowbot_dock_plugin:
      plugin: "mowing_navigation::MowbotChargingDock"   # custom (D8): isDocked = seat microswitch; V detection (§10.10)
      charging_threshold: 0.2              # A
      docking_threshold: 0.05              # pose fallback only (microswitch stale)
      staging_x_offset: 1.2                # POSITIVE, in front of the seated robot; V visible from ~1.3 m in (§10.11)
      staging_yaw_offset: 0.0
      telemetry_stale_s: 5.0
      microswitch_topic: "/dock/microswitch"
      battery_state_topic: "battery_state" # remapped to /dock/battery_state in navigation.launch.py
      use_external_detection_pose: true    # dock_v_detector → detected_dock_pose (lidar frame)
      external_detection_topic: "detected_dock_pose"
      external_detection_timeout: 1.0
      apex_to_base_m: 0.37                 # V apex → seated base_link (measured seated)
      filter_coef: 0.2
      target_overshoot_m: 0.25             # aim past the seated pose (graceful 1/r gain) — contact ends the approach
      lock_dist_m: 0.6                     # final stretch: lock once aligned → line-follow on the measured V axis (§10.12)
      lock_lateral_m: 0.03
      lock_yaw_deg: 3.0
    controller:
      k_phi: 3.0
      k_delta: 2.0
      v_linear_min: 0.15
      v_linear_max: 0.15
      v_angular_max: 0.5                   # cap the pivot when a tyre catches
      use_collision_detection: false       # the dock is a LiDAR obstacle (§5)
      costmap_topic: "local_costmap/costmap_raw"
      footprint_topic: "local_costmap/published_footprint"
      transform_tolerance: 0.1
      projection_time: 5.0
      simulation_time_step: 0.1
      dock_collision_threshold: 0.3
```

**Staging distance 2.0 m (2026-09-07):** chosen for the first real-dock
runs so the robot lines up well clear of the dock structure and the local
costmap inflation. Trade-off to watch: the blind reverse has no lateral
correction, so heading error at staging turns into lateral error at the
contacts — 2.0 m × sin(3°) ≈ 10 cm, beyond the ±3 cm funnel tolerance,
whereas 0.7 m × sin(3°) ≈ 4 cm. If the misalignment drill (§8.2) shows
consistent lateral misses, shorten the offset before touching anything else.

⚠ **Verification step, not gospel:** at first launch (§8.1) run
`ros2 param dump /docking_server` and reconcile names/defaults against
the installed 1.3.10 (README open question 3). Also verify whether this
version exposes `use_collision_detection` (see §5).

### 2.2 `navigation.launch.py`

- Add `opennav_docking::DockingServer` to the composable container
  (component registration verified 2026-09-07 via `ros2 component types`).
- Remap `cmd_vel` → `cmd_vel_raw` like the controller/behavior servers,
  so docking velocities flow through the velocity smoother → `cmd_vel_nav`
  → twist_mux (nav priority 10, e-stop lock) — §10.2 item 8.
- Remap `battery_state` → `/dock/battery_state` (published by the dock
  agent over zenoh, 03 §3.1).
- Add `docking_server` to the lifecycle manager `node_names` list.
- Start `dock_v_detector` (mowing_navigation) as a plain node in the same
  launch (parameters: `apex_to_base_m` 0.37, size tolerances) — it
  publishes `detected_dock_pose` whenever the lidar sees the V.
- Restart-of-bringup required on rollout (nav2_params change — same
  rollout class as F32).

### 2.3 Which pose docking uses

`dock_manager` sends `DockRobot` with `use_dock_id: false` and
`dock_pose` = dock.json pose (map frame). No dock database is
configured at all — the file is the single source of truth (README D3).
`dock_type` field carries `"mowbot_dock_plugin"`.

## 3. Bridge: new `dock_manager.{hpp,cpp}` (mowbot_mqtt_bridge)

Clone of the `goto_manager` pattern (action client + MQTT cmd/status +
interlocks), `docking:` section in `topics.yaml`:

| MQTT | Direction | Payload |
|---|---|---|
| `ros2/docking/cmd` | webui → robot | `{action: "dock"\|"undock"\|"cancel", id}` |
| `ros2/docking/status` | robot → webui, **retained** | `{state, feedback_state, physically_docked, drive_out_guard, reason, error_code, error_msg, started_at, retries, id}` (fields decided 2026-09-07, Q 5) |

`state`: `idle | powering_lidar | staging | approaching | waiting_charge |
docked | undocking | undocked | failed | canceled` — mapped from `DockRobot`/`UndockRobot`
feedback (`NAV_TO_STAGING_POSE`, `CONTROLLING`, `WAIT_FOR_CHARGE`,
`RETRY`) + results.

Field contract (the web UI is written against this — 05 §2.2/§5.3):

| Field | Type | Meaning |
|---|---|---|
| `state` | string | as above (`undocked` = terminal after a successful UndockRobot; `powering_lidar` while the F53 lidar wait runs) |
| `feedback_state` | string \| null | raw Nav2 feedback name (`NAV_TO_STAGING_POSE`, `INITIAL_PERCEPTION`, `CONTROLLING`, `WAIT_FOR_CHARGE`, `RETRY`) while an action runs |
| `physically_docked` | bool \| null | `/dock/microswitch`; `null` when the dock topic is stale (> 5 s) |
| `drive_out_guard` | bool | drive-out guard active (§3, §10.4 item 2) |
| `reason` | string | `""` = none. Bridge refusals / terminal reasons, goto convention: `estop`, `mission_active`, `mission_started`, `mission_state_unknown`, `no_dock_pose`, `bad_dock_pose`, `dock_offline`, `not_docked`, `busy`, `nav2_unavailable`, `nav2_rejected`, `nav2_timeout`, `cancel_timeout`, `lidar_off`, `bad_action`, `timeout`, `preempted`; Nav2 results mapped to `failed_to_stage`, `failed_to_detect_dock`, `failed_to_control`, `failed_to_charge`, `dock_not_valid`, `nav2_aborted` (UNKNOWN). Undock `FAILED_TO_CONTROL` with `physically_docked == false` ⇒ state `idle`, reason `undock_charge_status_stale` (warning, not failure — §10.2 item 7) |
| `error_code` | int | raw Nav2 result code (0 when none / bridge refusal) |
| `error_msg` | string | raw Nav2 `error_msg` (Logs tab / debugging) |
| `started_at` | int | epoch seconds of the current/last action, 0 when none |
| `retries` | int | Nav2 `num_retries` from feedback/result |
| `id` | string | echo of the command `id` |

Behavior:

- **`dock`**: read `~/pi_ws/mowing_data/config/dock.json` (fresh, per
  command); refuse with reason if missing/stale-schema. Send `DockRobot`
  (`use_dock_id:false`, pose+yaw from file, `navigate_to_staging_pose:
  true`).
- **`undock`**: sequenced shutdown (02 §4): publish
  `dock/charge_enable_cmd false` → firmware runs AC-off → DRAIN → K1
  opens at 0 A → wait until the dock reports cold (state IDLE, current
  < 0.1 A; 3 s timeout, proceed anyway with warn) → send `UndockRobot`
  → on result, re-publish `charge_enable_cmd true` (restore autonomy
  for the next docking — from IDLE, seat + enable is enough, no edge
  needed).
- **Re-charge while still docked** (02 §4: COMPLETE re-enters charging
  only on an `enable` 0→1 edge, since the dock goes cold at completion):
  pulse `charge_enable_cmd` false→true. Trigger policy (robot battery
  sag threshold vs. manual web-UI button) is a dock-agent/P4 decision —
  default manual.
- **Dock-online interlock** (decided 2026-09-07, Q 7): `dock` is refused
  with reason `dock_offline` when `/dock/battery_state` has not been
  received for `dock_stale_s` (5 s) — the charge-detection input the
  docking server needs. Not applied to `undock`/`cancel`.
- **Interlocks** (goto_manager conventions): mission must be idle
  (fresh `/mowing/mission_state`; `mission_state_unknown` refusal rules
  identical); e-stop → refuse/cancel; 3 s send watchdog + 5 s cancel
  watchdog (F28 lesson: never fire-and-forget action/service calls over
  zenoh).
- **Link-loss policy — deliberate difference from goto:** an in-flight
  `dock` action **continues** on MQTT link loss (the robot going home
  to charge is exactly what you want when connectivity is bad). Only an
  explicit `cancel` stops it. Document this in the `docking:` config
  comment.
- Subscribes `/dock/microswitch` + `/dock/state` (zenoh) to enrich
  retained status with `physically_docked` — the UI's "docked" truth is
  the microswitch, not the action result.
- **Automatic heading seed from the V** (2026-09-12, §11.7): while seated
  with the V behind the robot, true heading = dock.json yaw − V axis angle;
  published to `/set_pose` (both EKFs) when the estimate is off by
  > 2°. Runs after every docking, at undock start (lidar on, waits ≤ 12 s
  for the V before pulling out) and every 30 s while docked. Removes the
  manual calibration that every bringup restart / wheel slip used to cost.
- **Drive-out guard** (decided 2026-09-07, spec §10.4 item 2): manual
  motion on `/hoverboard_base_controller/cmd_vel` while seated and no
  docking action in flight ⇒ `charge_enable_cmd false` first, so K1 opens
  at 0 A before the wheels pull the contacts apart; re-enabled on switch
  release or 10 s idle. Status field `drive_out_guard`.

## 4. Charging detection path & edge cases

### 4.1 Normal flow

Approach → plates mate → microswitch → dock firmware runs the charge
sequence (K1 at 0 V → AC on → 42 V ramp, 02 §4) → current ~2 A →
`dock/battery_state.current` > 0.2 →
`SimpleChargingDock::isCharging()` true → `DockRobot` succeeds →
`dock_manager` status `docked`. (The AC-on ramp seconds sit inside
`wait_charge_timeout`'s budget, §2.1.)

### 4.2 isDocked before isCharging

With `use_external_detection_pose: false` the plugin's docked test is
pose-based (within `docking_threshold` of the dock pose). Because the
pose was **recorded by parking** (README D4), the seated pose matches
the stored pose almost exactly; RTK jitter (2–3 cm) vs the 5 cm
threshold leaves margin. If field tests show flapping: raise
`docking_threshold` to 0.08–0.10 (the microswitch + wait-for-charge
step still guarantee real contact before success), or implement the
custom plugin (README D8) whose `isDocked()` is the microswitch topic.

### 4.3 Docking with a nearly-full battery (README open question 5 — closed 2026-09-07)

Resolved by dock fw 0.2.0 (02 §4/§8): the seated robot's own consumption
(~0.35–0.40 A) flows through the charger, so **even a full pack shows real
current for at least the 60 s `COMPLETE_S` window after seating** before the
dock goes cold. With `charging_threshold: 0.2` and `wait_charge_timeout: 15`
the docking server always sees charging on a good contact. After COMPLETE the
dock stays cold until `charge_enable_cmd` is pulsed (top-up policy, §6 / §10.4).

The earlier mitigation of a `min_reported_current_while_seated` floor in the
dock agent was **dropped 2026-09-07**: never implemented, no longer needed,
and it would break `UndockRobot` (undock succeeds only once the reported
current is *below* `charging_threshold`, §10.2 item 7). The custom plugin
(`isCharging()` = dock state ∈ {3,5}, README D8) remains the fallback only if
field tests show the current-based test misbehaving.

### 4.4 Failure surface

| Failure | Detection | Outcome |
|---|---|---|
| Misaligned entry, no microswitch | no charge current → `WAIT_FOR_CHARGE` times out | docking server retries (backs up to staging, re-approaches) up to `max_retries`, then action fails → UI shows error, robot parked at dock mouth |
| Contact but overcurrent / AC weld / ramp failure | latched dock fault 1/2 or non-latching 3 (02 §3), no current | action fails; dock card shows fault; charging blocked until `clear_fault` (fault 3 auto-retries every 60 s by itself) |
| K1 closed but no charge path ("K1 no-close") | **no fault code** — dock sends `EVT:NOCURRENT` advisory; robot shows no `battery_voltage` step (02 §4) | indistinguishable from a full battery on the dock alone; `dock_manager` cross-checks robot telemetry and surfaces a warning in retained status |
| WiFi drops during `WAIT_FOR_CHARGE` | `battery_state` goes stale on the robot | treat as not-charging → retry/fail path; physical charging still starts (firmware autonomy) — recorded as a known cosmetic mismatch: robot may report failed dock while actually charging; microswitch state in `ros2/docking/status` disambiguates in the UI |
| E-stop mid-dock | existing e-stop signal | `dock_manager` cancels the action (motion stops via twist path already) |

## 5. Costmap / navigation environment around the dock

- The dock is a physical obstacle the LiDAR sees: it will be marked in
  the local costmap. The **staging pose must be far enough out
  (2.0 m for the first tests, §2.1)** that Nav2 can reach it normally; the final approach is the
  docking server's own controller, not FollowPath.
- Start with docking-server collision checking **disabled/lenient**
  (else the dock itself blocks the final 0.3 m — the classic problem;
  verify the installed version's `use_collision_detection` /
  projection params on the bench). Risk is bounded: ≤0.15 m/s over
  2.0 m into a funnel.
- **Do not** put a keepout zone over the dock approach; an optional
  thin keepout behind/beside the dock keeps transit plans from clipping
  the structure. Remember K1: keepout lives on the global costmap only.
- Dock placement inside/adjacent to a mow area: fine — mowing coverage
  paths shouldn't enter the dock footprint; if the dock sits inside an
  area polygon, add a small hole around it (area editor).
- KeepoutMonitor (F18/inside_keepout gate) only runs while a mission is
  RUNNING — docking happens mission-idle, no conflict.

## 6. Mission & safety integration (phase P4)

| Item | Change | Where |
|---|---|---|
| **Auto-dock on low battery** | `dock_manager` param `auto_dock_on_low_battery` (allowlisted + Safety-gates UI toggle): when retained mission state shows `paused` with reason `battery` (N1 gate) → send `mission_cmd stop`, wait for `idle` (30 s watchdog), then run the `dock` flow. Reuses every interlock above. | bridge `dock_manager` — zero BT changes (the BT-native alternative, an `opennav_docking_bt` DockRobot node in the mission tree, is the documented long-term option if docking ever needs to be part of a mission rather than after it) |
| **Auto-dock on mission complete** (decided 2026-09-07, Q 6) | `dock_manager` param `auto_dock_on_mission_complete` (bool, default **false**, allowlisted + Settings "Docking" toggle): on the retained `/mowing/mission_state` **edge** `running → idle` with `reason == "complete"` (what `completeMission()` emits; a manual stop or `failed` does not trigger), wait `auto_dock_delay_s` (default 5, lets the blade spin down / stats close), then run the `dock` flow with every interlock above. No mission stop needed — the mission is already idle. One attempt per completion edge; a refusal or failure surfaces in `ros2/docking/status` and is not retried automatically. | bridge `dock_manager` |
| **No mowing while docked** | `start_mission` wrapper (mowing_mission_node) refuses start when docked: subscribe `/dock/microswitch` (passive cache, same style as other gates); refusal reason `docked` in mission state; UI hint "Undock first". If the topic is absent/stale (dock Pi down but robot physically docked is unlikely — microswitch implies dock powered), allow start with a warn. | mowing_navigation |
| **Undock-then-mow convenience** | Web UI: Start button while `physically_docked` offers "Undock & start" (UI sequences undock → wait idle+undocked → start). Robot-side auto-sequencing deliberately deferred. | frontend only |
| **Blade interlock** | None needed beyond the above: blade control paths already require a running/idle-manual context, and mission start is now refused while docked. Manual blade (BladeControl) while docked is physically pointless but harmless — optionally add the same `docked` refusal to `manual_mow` later. | — |
| **Charge stats** | P4 backlog: `stats.py` charge-session records (start/end, Ah estimate from dock current) from `ros2/dock/#` — the backend already consumes the broker. | LXC backend |

## 7. topics.yaml / allowlist / ACL deltas (robot side)

- `docking:` manager section (cmd/status mapping, link-loss policy,
  dock.json path).
- Bridge param allowlist: `auto_dock_on_low_battery`, `auto_dock_on_mission_complete`, `auto_dock_delay_s` (P4).
- LXC ACL (user runs, exact commands provided at implementation):
  `webui` → `topic write ros2/docking/cmd`. (Dock-hardware ACLs are in
  03 §4.)
- Bridge rebuild + restart (new manager = new binary; nav2_msgs dep
  already present from goto_manager).

## 8. Test plan

### 8.1 Static checks on the live robot (rewritten 2026-09-07 — real dock exists, no fakes)

The original bench plan (fake `/dock/*` publishers, F32-era indoor rig with
a static map→odom TF) is **obsolete**: the dock agent is live and federated,
so fake publishers would fight the real topics, and the indoor rig fights the
live EKF. Instead, run these with the **drive motors master switch OFF**
(drive commands are ignored, nothing moves), robot anywhere, real dock on:

- [x] `docking_server` lifecycle-activates in the nav container;
      `ros2 param dump /docking_server` reconciled with §2.1 (closes README
      open question 3). **Done 2026-09-07** — activated + bonded after the
      bringup restart; all §2.1 values read back (`ros2 param get`, the
      `dump` command returns `{}` for composed nodes over zenoh). Extra
      1.3.10 names seen: `controller.simulation_time_step` (yaml key fixed),
      `navigator_bt_xml`, `controller.v_angular_max/slowdown_radius/beta/lambda`.
- [x] `/dock/battery_state` seen by the docking server (`ros2 topic info -v`
      shows the subscription; QoS compatible). **Done 2026-09-07** — publisher
      `dock_agent`, subscribers `docking_server` + `mqtt_bridge_node`, all
      RELIABLE/VOLATILE; `/dock_robot` + `/undock_robot` actions present;
      `docking_server` publishes `/cmd_vel_raw` (TwistStamped) next to the
      controller/behavior servers.
- [ ] Refusals, each with the right `reason` in `ros2/docking/status`:
      e-stop pressed, mission running, mission node stopped
      (`mission_state_unknown`), `dock.json` missing / malformed,
      `dock_offline` (stop `mowbot-dock-agent` on the dock Pi for a minute),
      `undock` while not seated (`not_docked`).
- [ ] `dock` with motors OFF: status walks `powering_lidar` → `staging` →
      (Nav2 cannot move the robot) → cancel works from `staging`; then let
      one run time out to see the `failed` / mapped reason path.
- [ ] Robot parked in the real dock by hand (Drive pad), charging: drive-out
      guard — a pad command ⇒ `charge_enable` goes false (dock card shows
      "charging DISABLED"), `drive_out_guard: true`; release ⇒ re-enabled
      after 10 s / switch release.
- [x] **Done live 2026-09-07 20:13 (real motion, not motors-OFF):** `undock`
      from CHARGING (1.56 A) ⇒ permission off → dock cold (state 0, 0.01 A)
      in < 1 s → `UndockRobot` accepted → seat switch released at +1 s →
      "reached staging pose" at +18 s → `undocked`, permission restored,
      dock IDLE. Status walked `undocking/COOLING → undocking/UNDOCKING →
      undocked` with `physically_docked` flipping true→false.
- [ ] Discipline: `setsid nohup … < /dev/null` for any ad-hoc publisher,
      never pattern-kill nav2 names while production units run, expect a
      phantom stats mission if the bridge is up.

Everything that moves the robot is in §8.2 with the real dock.

### 8.2 Field (P3 exit criteria)

- [x] (2026-09-07, one-off by hand: robot was already seated; pose read from
      `/odometry/global`, RTK h_acc 14 mm, written to `dock.json` — Phase B
      overwrites it via the real button later) Manual park (Drive pad until the microswitch clicks and charging
      starts) + **"Save dock at robot position"** (web UI Phase B) —
      `dock.json` appears on the robot, marker + 2.0 m staging marker on the
      map, backend warning shown if the dock reports "not seated".
- [ ] (undock part done 2026-09-07 — first real undock clean) Undock, then first autonomous dock from ~3 m with the robot already
      roughly on the dock axis; then from 3 m off-axis; 10/10 attempts with
      charge current confirmed before moving on.
- [ ] Dock from different yard corners (staging navigation across
      transits, keepouts respected).
- [ ] Undock → robot idle 2.0 m out (staging offset), dock cold before pull-out (AC off,
      K1 open at 0 A — watch dock card state/current during the
      sequence).
- [ ] Misalignment drill: offset the robot's approach by blocking one
      funnel side → verify retry behavior and clean failure.
- [ ] E-stop mid-dock; WiFi-off-at-contact (cosmetic mismatch of §4.4
      confirmed and readable in UI).
- [ ] Full loop: mow until N1 battery pause (temporarily raise
      `mow_battery_low_voltage`) → auto-dock (P4) → charge to complete →
      stats sane.
- [ ] Move the dock 2 m, re-record, dock again — the "relocation is
      easy" acceptance test.

## 9. Rollout order (revised 2026-09-07, Q 8)

1. Robot side: §2 nav2_params/launch, §3 `dock_manager`, §6 mission-start
   gate. In parallel, **web UI Phase B** (dock pose: fileserver/watcher,
   `/api/dock`, "Save dock at robot position", map markers — 05 §5.2); it
   has no robot-side dependency and is how every dock pose will be
   recorded, so there is no hand-written `dock.json` step.
2. Deploy (user): `bringup` restart (nav2_params + launch), bridge
   rebuild+restart, mission node rebuild+restart, robot
   `mowbot-fileserver`/`mowbot-version-watcher` restarts, LXC ACL
   (`webui` → `ros2/docking/cmd`) + backend/frontend deploy.
3. §8.1 static checks (motors OFF, real dock), then park by hand, press
   "Save dock at robot position", and the first real-dock cycles of §8.2
   — Dock/Undock sent with `mosquitto_pub` until Phase C exists.
4. **Web UI Phase C** (Dock/Undock control, 05 §5.3) once the status
   contract has survived the first real runs.
5. Rest of the §8.2 field checklist, then enable the P4 automation flags
   one at a time (`auto_dock_on_mission_complete`,
   `auto_dock_on_low_battery`).

## 10. Pre-implementation review (2026-09-07) — findings, corrections, open questions

Reviewed against the installed `opennav_docking` 1.3.10 (headers on the
robot + upstream source at tag 1.3.10), the live robot (dock topics visible
through the robot router, nav container running), the bridge/mission/web-UI
repos and 05. Nothing implemented yet; this section is the delta to apply
when §2–§7 are built. Items marked **(Q n)** need an owner decision.

### 10.1 Repository state vs GitHub (fetched 2026-09-07)

| Repo (local clone) | State |
|---|---|
| `mowbot_mqtt_bridge`, `mowing_navigation`, `ros2-driver-converted`, `mowing_msgs` (pi_ws) | in sync with `origin/main`, clean |
| `mowbot_web_ui` (`~/mowbot_web_ui_remote`) | in sync (`88f39e6`), clean |
| `mowbot_plans` | in sync (`8b3588b`) |
| `mowbot_dock` | **behind 1** (`8db6184`, docs only: HANDOFF/TODO COMPLETE resolution) — `git pull --ff-only` |
| `mowbot_dock_arduino` | not cloned on the robot Pi (workstation only) — not verified here |
| `camera_ros`, `libcamera`, `fusioncore` | third-party, behind upstream, irrelevant to docking |

### 10.2 Corrections to §2 (verified in 1.3.10)

1. **`nav2_params.yaml` already contains a `docking_server:` section** — a
   leftover of the original Nav2 template (commit `b1150d9`), never launched
   (`docking_server` is not in `lifecycle_nodes`). It has the wrong values
   for us (`dock_backwards: false`, `use_external_detection_pose: true`,
   `use_battery_status: false`, `use_collision_detection: true`,
   `wait_charge_timeout: 5.0`, `v_linear_min: 0.15`). §2.1 is therefore a
   **rewrite of that block**, not an addition. Keep its
   `enable_stamped_cmd_vel: True` (twist_mux runs `use_stamped: true`).
2. **Staging sign resolved** (`simple_charging_dock.cpp`):
   `staging = dock_pose + staging_x_offset·(cos yaw, sin yaw)`, yaw +
   `staging_yaw_offset`. With the recorded yaw = seated robot heading
   (pointing away from the dock, D10) use **`staging_x_offset: 2.0` (positive; 0.7 was the earlier draft)
   — BUT the approach target orientation must be flipped by π for the
   backward controller, done in the custom plugin's `getRefinedPose()`
   (§10.8, found live 2026-09-07)
   (positive) and `staging_yaw_offset: 0.0`**. The server itself rotates the
   approach target by π in `approachDock` when `dock_backwards` is true, and
   `resetApproach`/`undockRobot` drive with `!dock_backwards` (forward). No
   180° turn is needed anywhere. This matches the 05 §4.4 marker math.
   Drop the "VERIFY sign" note in §2.1.
3. `use_collision_detection` **exists** in the 1.3.10 controller (with
   `projection_time`, `simulation_step`, `dock_collision_threshold`,
   `costmap_topic`, `footprint_topic`). Set it **`false`** explicitly.
4. All other §2.1 names match 1.3.10 (`docking_threshold` 0.05,
   `charging_threshold` default 0.5, `dock_prestaging_tolerance`,
   `action_server_result_timeout` also exists). README open question 3 can
   be closed at first `ros2 param dump`.
5. **No dock database needed — confirmed.** `DockDatabase::initialize()`
   only warns when neither `docks` nor `dock_database` is set;
   `dock_type` selects the plugin, and with a single plugin an empty
   `dock_type` also resolves. Always send `dock_type` anyway (needed by
   `UndockRobot` after a server restart, item 7).
6. **Dock pose is transformed into `fixed_frame` AFTER the staging
   navigation** (`dockRobot()`: `goToPose` → then `getDockPoseStamped` +
   `tf2_buffer_->transform`), so odom drift during a long transit does not
   corrupt the target; only the ~10 s approach runs on odom. `fixed_frame:
   odom` stays.
7. **Undock semantics:** if the server has no prior docking (bringup
   restarted while parked in the dock) `UndockRobot` takes the robot's
   *current* pose as the dock pose and computes staging from it — works, as
   long as `dock_type` is set. After reaching staging, success requires
   `hasStoppedCharging()` (= battery current ≤ `charging_threshold`);
   otherwise it fails **immediately** with `FAILED_TO_CONTROL` ("Failed to
   control off dock") — no wait. Consequence for `dock_manager`: a
   `FAILED_TO_CONTROL` undock result with `physically_docked == false`
   must be reported as *undocked (warning: charge status stale)*, not as a
   failure — the case is a stale `/dock/battery_state` (WiFi) whose last
   value was "charging". The sequenced enable-off before `UndockRobot`
   normally makes current 0 well before pull-out.
   **Superseded 2026-09-07 by §10.7:** the undock *pre-check* is
   `isDocked() || isCharging()`; with the stock plugin both are false after a
   restart/manual park + sequenced enable-off ⇒ "Robot is not in the dock, no
   need to undock". Fixed by the custom plugin (isDocked = microswitch).
8. **`cmd_vel` plumbing missing from §2.2.** The docking server publishes
   `cmd_vel`; in `navigation.launch.py` the controller and behavior servers
   remap `cmd_vel → cmd_vel_raw` → velocity_smoother → `cmd_vel_nav` →
   twist_mux (nav priority 10, e-stop lock 255). Give the
   `docking_server` composable the **same `('cmd_vel', 'cmd_vel_raw')`
   remap**; the smoother allows reverse (`min_velocity` −0.4, deadband 0).
   Note: the collision monitor is disconnected today (nav2 review NP1), so
   the approach has no scan-based protection either way — consistent with
   the "collision checking off" decision; the twist_mux e-stop lock still
   applies.
9. `battery_state` remap → `/dock/battery_state` confirmed correct
   (`SimpleChargingDock` subscribes the relative name `battery_state`,
   volatile QoS, compatible with the agent's publisher). The topic is
   visible on the robot today (`ros2 topic list` shows all 15 `/dock/*`).
10. `opennav_docking::DockingServer` is registered as a component
    (`ros2 component types`) — the composable route works, no fallback
    needed.

### 10.3 Corrections/additions to §3 (`dock_manager`)

1. **Lidar power (F53).** `goto_manager` powers the lidar on and waits for
   scans before sending `NavigateToPose` because the mission node's F29
   idle power-off may have switched it off. The staging navigation needs
   the obstacle layers, so `dock_manager` must copy the
   `powering_lidar` step (state visible in status). `cmd_vel` activity
   re-arms the idle timer, so it will not switch off mid-docking.
2. **Refuse `dock` when the dock is unreachable — decided 2026-09-07
   (Q 7).** No `/dock/battery_state` message for > 5 s (`dock_stale_s`,
   configurable) ⇒ refuse with reason `dock_offline`. Blind docking without
   charge feedback can only end in 3 wasted retries and `FAILED_TO_CHARGE`.
   The same check is not applied to `undock` (pulling out must always be
   possible; the microswitch-release path on the dock is the fallback) nor
   to `cancel`. Also added to the §3 interlock list.
3. **Status payload — decided 2026-09-07 (Q 5): `physically_docked` and a
   goto-style string `reason` added next to the raw Nav2 `error_code` /
   `error_msg`.** Full field contract now in §3; 05 §2.2 updated to match.
   `useDock.js`/`dockStatus.js` already read `docking.state` and
   `docking.feedback_state` with exactly the state names in §3 — unchanged.
4. **§4.3 floor option dropped, `charging_threshold: 0.2` — decided
   2026-09-07 (Q 2).** The floor was never implemented in `dock_agent`
   (`bat.current` is the raw reading), fw 0.2.0 makes it unnecessary, and it
   would break `UndockRobot`'s `hasStoppedCharging()`. 0.2 A is 10× the
   noise floor and 0.15 A under the lowest real charging reading; 03 §3.1
   updated to match.
5. **Mission stop for auto-dock (§6):** the mission command path is the
   String topic `/mowing/mission_cmd` (MQTT `ros2/mission/cmd`); the
   retained `/mowing/mission_state` JSON carries `{"state","reason",…}`, so
   "paused + battery" detection works as written. From inside the bridge,
   publish `stop` on `/mowing/mission_cmd` (same path the UI uses) and wait
   for `idle` (30 s watchdog) as planned.
6. **`start_mission` docked refusal (§6):** the mission node already has the
   cached-latched-topic gate pattern (`/hoverboard/motors_allowed`,
   `mowing_mission_node.cpp` `start_mission` lambda); `/dock/microswitch`
   is latched the same way. Both the web UI and the OLED panel go through
   `/mowing/mission/start`, so one gate covers both.

### 10.4 New gaps found

1. **Parked drain after COMPLETE (P4, load-bearing).** With fw 0.2.0 the
   dock goes cold at COMPLETE and never restarts by itself; the robot keeps
   drawing ~0.33 A from the pack — roughly 8 Ah/day, i.e. a robot left in
   the dock over a weekend is *flatter* than when it arrived. §3 "default
   manual" top-up is not enough. Proposal: `dock_manager` top-up policy —
   while `physically_docked` && dock state == 5 && robot
   `hoverboard/battery_voltage` < `dock_topup_voltage` (e.g. 40.5 V, 60 s
   debounce, ≥ 30 min between pulses) ⇒ pulse `charge_enable_cmd`
   false→true. Alternative: firmware `TOPUP_INTERVAL_S` (Pi-independent,
   but blind to the pack). **(Q 3 — deferred 2026-09-07: not in the first
   implementation; manual top-up from the Dock page until then.)**
2. **Manual drive-out while charging — decided 2026-09-07 (Q 4): bridge
   interlock + UI warning.** Nothing stops the Drive pad / joystick /
   Foxglove from pulling the robot off the contacts with K1 closed under
   load — the unsequenced `EVT:EMERGENCY:SWITCH` path (source of the five
   false fault-2 latches, now graced but still an under-load DC break).
   Spec for `dock_manager` ("drive-out guard", §3):
   - Watch the twist_mux output `/hoverboard_base_controller/cmd_vel`
     (covers every manual source at once — same topic the F29 lidar idle
     manager uses), not only `cmd_vel_web`.
   - Trigger: non-zero command while `physically_docked` (microswitch)
     **and no docking action of our own in flight** (state ∉
     {approaching, waiting_charge, undocking} — the docking controller
     still commands motion for a moment after contact, and undock has
     already disabled charging itself). Then publish
     `dock/charge_enable_cmd false` once; retained status gains
     `drive_out_guard: true`.
   - Release: microswitch false, or 10 s without a non-zero command ⇒
     publish `charge_enable_cmd true`, `drive_out_guard: false`. A manual
     "Charging allowed = off" from the Dock page is not overridden: the
     guard re-enables only what it disabled itself.
   - The guard does not block the motion (the dock drains and opens K1 in
     well under a second, DRAIN_A 0.10 / DRAIN_TIMEOUT 2 s); refusing manual
     drive-out entirely was not wanted.
   - UI (05 Phase C): Drive page banner "robot is in the dock — charging is
     switched off while you drive; prefer Undock" while `physically_docked`;
     `drive_out_guard` shown in the docking status line.
3. **Phase B (`dock.json`) is not started**: `robot/fileserver.py` still
   accepts PUT only for `/mow_area/mow_areas.json`, no `/api/dock`, no
   `config/dock.json` on the robot. §3 reads that file per command, so
   the robot side reads it per command, so **05 Phase B is a
   prerequisite for the field checklist (§8.2)** — built alongside the
   robot side, §9 step 1 (decided 2026-09-07, Q 8: no hand-written file).
4. **Staging distance source.** 05 §4.4 stores `staging_offset_m` in
   `dock.json` (UI input 0.5–1.5 m), but the plugin's `staging_x_offset` is
   a static parameter (no dynamic-parameter callback in
   `SimpleChargingDock` 1.3.10). Either fix it in nav2_params (UI shows it
   read-only, `dock.json` keeps the field for the marker), or have
   `dock_manager` set the parameter before each dock and verify the plugin
   re-reads it (it does not — it reads at `configure`). **Decided (Q 1):
   fixed in nav2_params, no UI input for now** — 05 §4.4/§6 "Staging
   distance" input deferred; the marker uses `dock.json.staging_offset_m`,
   which must be written with the same value as `staging_x_offset`
   (**2.0 for the first tests**, 2026-09-07).
5. **Return-to-dock after a finished mission — decided 2026-09-07 (Q 6):
   yes, behind its own parameter.** `auto_dock_on_mission_complete`
   (default off) next to `auto_dock_on_low_battery`; trigger = the
   `/mowing/mission_state` edge to `{"state":"idle","reason":"complete"}`.
   Row added to the §6 table, params to §7.
6. **Web UI text stale after fw 0.2.0** (frontend only, not robot side):
   `lib/dockStatus.js` row 12 still says "next top-up starts
   automatically", and row 10 lacks the "Enable off→on to top up" hint that
   05 §3 now specifies. One-line fixes; bundle with Phase C.

### 10.6 (reserved)

### 10.7 Implementation finding 2026-09-07 — stock plugin cannot undock after a restart (D8 promoted)

First live run of `dock_manager` (robot parked in the real dock, COMPLETE):
`undock` ran the sequence exactly as designed — permission off → dock cold
(state 5, 0 A) → `UndockRobot` accepted — and Nav2 aborted it in 90 ms:
*"Robot is not in the dock, no need to undock"*. Cause (1.3.10
`undockRobot()` pre-check `isDocked() || isCharging()`): the stock
`SimpleChargingDock` decides "docked" by distance to a dock pose it only
remembers from a docking action **in the same server run**, so after a
bringup restart or a manual park (the §8.2 commissioning flow) it is false;
and the sequenced undock switches charging off first (K1 must open at 0 A,
02 §4), so `isCharging()` is false too. The two halves of the plan were
mutually exclusive with the stock plugin.

**Fix (README D8, now the design, not the fallback):**
`mowing_navigation::MowbotChargingDock` (`mowing_navigation` package,
`src/mowbot_charging_dock.cpp`, exported via `mowbot_dock_plugins.xml`):
`isDocked()` = `/dock/microswitch` when fresh (pose-based fallback within
`docking_threshold` when the dock Pi is unreachable), `isCharging()` =
`battery_state.current > charging_threshold`, `getStagingPose()` identical
to the stock math, `disableCharging()` = no-op (the bridge sequences it).
Loaded and active 2026-09-07 20:01. Benefits beyond the fix: the docking
approach stops on physical contact rather than on a pose estimate, and
undock works in every case the seat switch is visible.

Side note from the same run: the undock's off→on permission pulse on a
COMPLETE dock is the firmware's re-charge trigger (02 §4) — the dock started
a fresh CHARGING cycle (1.0 A) afterwards. Expected, harmless, worth knowing
when reading dock logs after an undock attempt.

### 10.8 Field finding 2026-09-07 — backward approach looped 45° (orientation convention), fixed in the plugin

Two live dock attempts (one from the staging point, one after a 4 m
converging straight run to rule out heading drift) both swung ~45° off the
axis within seconds of starting the reverse and were e-stopped. The undock
(same controller, forward) had tracked straight to 15 cm. Reading 1.3.10:
`approachDock()` rotates the target orientation by π for `dock_backwards`,
and `EgocentricPolarCoordinates` rotates the line of sight by π for
`backward` as well. The server therefore expects the dock yaw to point
**into** the dock. With our D4 convention (yaw = seated robot heading,
pointing away) the control law sees `phi = π` — "arrive facing the opposite
way" — and plans a wide loop. Deterministic, independent of localization
(§10.2 item 2 got the staging position right and this orientation part
wrong; the original "staging_yaw_offset: π?" suspicion was half-right).

**Fix:** `MowbotChargingDock::getRefinedPose()` returns the dock pose with
yaw + π. Only the approach target is produced there; `getStagingPose()`
(dock command) and the undock (robot pose) keep the un-flipped heading with
`staging_x_offset: +2.0` / `staging_yaw_offset: 0`, both verified live.
dock.json convention unchanged (seated robot pose, yaw away from the dock).

Also learned: the map-frame heading is unobserved at rest (no absolute yaw
source in either EKF, 01/nav2 config) and re-converges only while driving;
the recorded yaw was within 3° of the RTK exit track, the post-drive
estimate 12° off. Mitigations kept for later (not the cause of the loops):
record yaw from the RTK entry/exit track instead of the heading estimate;
`fixed_frame: map` so the target keeps correcting during the reverse; dock
from ≥ 4 m out so the staging leg converges heading; dual-antenna GNSS
heading long-term.

### 10.9 Field finding 2026-09-07 (runs 3–4) — heading estimate after the turn at staging is the blocker

After the §10.8 fix the reverse was straight (run 3) and the docking control
itself behaved: the robot converged on the *estimated* dock pose to 6 cm /
5° (run 4). Physically it ended ~0.4–0.6 m beside the dock both times.
Measured at the moment the reverse started (1 s into `CONTROLLING`, right
after Nav2's 180° turn-in-place at staging): heading estimate error **+19.5°
(run 3), +25.2° (run 4)** vs the RTK-measured dock axis; 13 s of reversing
later the heading estimate had converged to < 1° / 5°, but the position
estimate had inherited the error (dead-reckoning with a wrong heading, GPS
x/y pulling it back slowly — map EKF x/y process noise 1.0).

Conclusions:
- The docking code (server config, plugin, `dock_manager`) is not the
  limiting factor any more; the map-frame **heading estimate right after a
  turn-in-place is off by ~20–25°** and only converges while driving
  straight. Neither EKF has an absolute yaw source (04 config review: wheel
  vx/vy/vyaw + gyro vyaw + GPS x/y only); the BNO085 reports gyro
  calibration 0 ("unreliable"); F58b "calibration dance" and F62 spin gate
  are the mowing side of the same problem.
- `fixed_frame: map` (applied run 4) is correct and kept, but not
  sufficient: re-aiming cannot help while the position estimate is wrong.
- `dock.json` yaw was taken from the RTK exit track (−42.5°) — good.

Options (decide before the next session):
1. **Heading-independent reverse** (recommended): `dock_manager` runs the
   final approach itself — NavigateToPose to staging (Nav2 spins, error
   irrelevant afterwards), then reverse at 0.1 m/s steering on RTK only:
   lateral error to the recorded axis line from `/odometry/gps` (2 cm) and
   course from successive RTK positions; stop on the seat microswitch, then
   wait for charge current. Nav2 keeps undock and staging. ~200 lines,
   replaces the DockRobot approach (D2 "blind RTK" done literally).
2. Longer approach for convergence: `staging_x_offset` 4.0 + slower; cheap
   one-parameter try, uncertain (position lag), robot sweeps near the dock.
3. Fix heading estimation (separate project): gyro reliability/bias,
   wheel-separation calibration (a 12 % error = 22° per 180° turn), fuse
   the BNO085 absolute yaw (needs magnetometer calibration away from the
   dock), or dual-antenna GNSS heading.

### 10.10 Decision 2026-09-07 (late) — lidar detection of the dock's V target (external detection pose)

Answer to §10.9: use `use_external_detection_pose`-style detection instead
of the map heading. The dock already carries a **V-shaped target on top of
the charger** (owner design): 320 mm wide at the opening, 92 mm deep,
120° opening (arms at 60° to the axis), at LD06 height. Scan from the seated
robot (2026-09-07, lidar on): 502 pts/rev, 0.72° step, full 360° (rear NOT
cropped), intensities present; the V shows as ~100 points at 0.26–0.31 m
behind the lidar, apex on the axis (y = 0.00), intensity 90 apex / 180 arms.
Calibration from that scan: **apex → base_link = 0.31 + 0.051 = 0.36 m**
along the axis. Also seen: a very bright (I ≈ 220) straight surface 0.45 m
to one side, a wall 1.06 m to the other.

Design (next session, §11.7):
- New node `dock_v_detector` (mowing_navigation, C++): subscribe `/scan`;
  prior = the plugin's `docking/dock_pose` (map) transformed into the lidar
  frame via TF (fallback: dock.json); window ±0.6 m / ±30° around the prior;
  split points by the prior axis, fit two lines (least squares + outlier
  rejection), accept if the inter-line angle is 120° ± 15° and each arm is
  0.12–0.22 m; apex = intersection, axis = bisector pointing out of the V
  (toward the opening). Publish `detected_dock_pose` (PoseStamped, lidar
  frame, ~10 Hz) with position = apex and yaw = out-of-V direction, plus a
  marker for RViz/Foxglove. At 2 m the V is ~13 points (coarse), at 1 m
  ~26, at 0.5 m ~50 — accuracy improves exactly where it matters.
- `MowbotChargingDock`: `use_external_detection_pose` param; when a
  detection younger than `external_detection_timeout` (1 s) exists,
  `getRefinedPose` returns dock pose = apex + `v_apex_to_base_m` (0.36) ·
  u_out, yaw = −u_out (the server's "into the dock" convention, §10.8),
  through the existing low-pass filter; otherwise the dock.json prior (so
  the approach starts on the prior and hands over to the measurement).
  isDocked stays the seat microswitch.
- `fixed_frame` back to **odom** once detection is on (the detection is
  re-transformed every cycle; odom is the smooth short-term frame — the
  map heading drift that motivated `map` in §10.9 no longer matters).
- Retroreflective tape on the V arms is optional; the 0.45 m side surface
  shows the LD06 rewards it (I ≈ 220 vs 140 for a plain wall).
- **Scan from the 2 m staging pose (2026-09-07, after a clean undock): the V
  is NOT visible** — no points within 0.3 m of the expected apex 2.3 m
  behind the lidar. Cause (owner): a slight slope tilts the robot so the 2D
  scan plane misses the 92 mm-deep target at that range (3° pitch ≈ 10 cm
  at 2 m). Hardware fix: **make the V arms tall (200–250 mm vertical
  plates, centred on the seated lidar height ≈ 27 cm)**, optionally with
  reflective tape; software side: **`staging_x_offset` down to 1.0–1.2 m**
  (halves the plane offset, doubles the point count). The detector starts
  on the dock.json prior and locks on when the V appears, so far-end
  blindness is tolerable as long as the V is seen well before contact.

### 10.5 Open questions (owner)

| Q | Question | Proposed default |
|---|---|---|
| ~~1~~ | ~~Staging distance: fixed plugin parameter or per-`dock.json` value?~~ **Decided 2026-09-07: fixed in nav2_params, 2.0 m for the first tests** (`staging_x_offset: 2.0`, tune down later); no web-UI input for now (can be added later); `dock.json` keeps `staging_offset_m` for the map marker only, written with the same value. | — |
| ~~2~~ | ~~`charging_threshold` — 0.15 A (plan), 0.2 A, or ~0.3 A (HANDOFF)?~~ **Decided 2026-09-07: 0.2 A first**, tune in the field if needed; §4.3 floor option deleted. | — |
| ~~3~~ | ~~Top-up while parked (0.33 A drain after COMPLETE): robot-side voltage-triggered pulse, firmware `TOPUP_INTERVAL_S`, or manual only?~~ **Decided 2026-09-07: deferred** — first implementation is manual only (web-UI "Enable" pulse); automatic top-up is a later P4 item (§10.4 item 1 keeps the proposal). | — |
| ~~4~~ | ~~Manual drive-out while charging: auto enable-off on drive activity, UI warning, or accept the emergency break?~~ **Decided 2026-09-07: bridge drive-out guard (auto enable-off on manual motion while seated) + Drive-page warning** — spec in §10.4 item 2. | — |
| ~~5~~ | ~~Status payload: add string `reason` (goto style) next to Nav2 `error_code`/`error_msg`, and `physically_docked`?~~ **Decided 2026-09-07: yes, both** — field contract in §3. | — |
| ~~6~~ | ~~Auto-dock when a mission completes (not only on low battery)?~~ **Decided 2026-09-07: yes, with `auto_dock_on_mission_complete` (default off)** — §6 table row. | — |
| ~~7~~ | ~~Refuse `dock` when `/dock/battery_state` is stale (> 5 s)?~~ **Decided 2026-09-07: refuse, reason `dock_offline`** (`dock` only; `undock`/`cancel` unaffected). | — |
| ~~8~~ | ~~Bench first or Phase B first? (bench needs only a hand-written `dock.json`)~~ **Decided 2026-09-07: robot side and web UI Phase B built together (no hand-written `dock.json`, no fake bench), §8.1 static checks, first real-dock runs, then Phase C, then the rest of §8.2** — §9. | — |

## 11. Implementation TODO (2026-09-07 — all §10.5 decisions folded in)

Ordered as §9. Tick items as they land; deploy-side items are user-run.

### 11.1 Nav2 (`ros2-driver-converted/bringup`)

- [x] (2026-09-07) `config/nav2_params.yaml`: **rewrite** the existing stale
      `docking_server:` block to §2.1 — `dock_backwards: true`,
      `fixed_frame: odom`, `wait_charge_timeout: 15`,
      `dock_approach_timeout: 60`, `max_retries: 3`, keep
      `enable_stamped_cmd_vel: True`; plugin `mowbot_dock_plugin`
      (`SimpleChargingDock`) with `use_external_detection_pose: false`,
      `use_battery_status: true`, `charging_threshold: 0.2`,
      `use_stall_detection: false`, `docking_threshold: 0.05`,
      `staging_x_offset: 2.0`, `staging_yaw_offset: 0.0`; controller
      `use_collision_detection: false`, `v_linear_min 0.10`, `v_linear_max 0.15`.
- [x] (2026-09-07) `launch/navigation.launch.py`: add `opennav_docking::DockingServer`
      composable (name `docking_server`, remaps `('cmd_vel','cmd_vel_raw')`
      + `('battery_state','/dock/battery_state')`) and `docking_server` to
      `lifecycle_nodes`.
- [x] (2026-09-07) Lifecycle-activates; parameters reconciled with §2.1
      (closes README open question 3) — §8.1.
- [x] (2026-09-07) `mowbot-launch-bringup` restarted; `docking_server` active.
- [x] (2026-09-07) **Custom plugin `MowbotChargingDock`** in `mowing_navigation`
      (§10.7): built, `nav2_params.yaml` switched to it, bringup restarted,
      plugin created + active, subscribed to `/dock/microswitch` + `/dock/battery_state`.

### 11.2 Bridge `dock_manager` (`mowbot_mqtt_bridge`)

- [x] (2026-09-07) `include/…/dock_manager.hpp`, `src/dock_manager.cpp` (clone of
      `goto_manager` shape), `DockingConfig` in `bridge_config.{hpp,cpp}`,
      `docking:` section parsed from `topics.yaml`, wiring in
      `mqtt_bridge_node.cpp` (cmd handler, retained status republish on
      reconnect), `CMakeLists.txt`.
- [x] (2026-09-07) Commands `dock | undock | cancel` with `id` echo; `dock` reads
      `~/pi_ws/mowing_data/config/dock.json` per command (refuse
      `no_dock_pose` / `bad_dock_pose`); sends `DockRobot`
      (`use_dock_id:false`, `dock_type: mowbot_dock_plugin`,
      `navigate_to_staging_pose:true`); `undock` always sends `dock_type`.
- [x] (2026-09-07, + `already_docked` refusal) Interlocks: e-stop (refuse + cancel), mission idle / fresh
      (`mission_state_unknown`), `busy`, `nav2_unavailable`, **dock-online
      (`dock_offline`, `/dock/battery_state` stale > `dock_stale_s` 5 s,
      `dock` only)**, `not_docked` for undock; 3 s send + 5 s cancel
      watchdogs; link loss does **not** cancel (documented in config).
- [x] (2026-09-07) F53 lidar power step (`powering_lidar` state) copied from
      `goto_manager`.
- [x] (2026-09-07; live-verified up to UndockRobot accepted) Sequenced undock: `charge_enable_cmd false` → wait dock state IDLE
      & current < 0.1 A (3 s timeout, warn) → `UndockRobot` → on result
      `charge_enable_cmd true`. Undock `FAILED_TO_CONTROL` with
      `physically_docked == false` ⇒ `idle`, reason
      `undock_charge_status_stale` (warning).
- [x] (2026-09-07, code; not yet exercised) **Drive-out guard** (§10.4 item 2): `/hoverboard_base_controller/cmd_vel`
      non-zero while seated and no own action in flight ⇒
      `charge_enable_cmd false` once; re-enable on switch release or 10 s
      idle; never overrides a manual enable-off; `drive_out_guard` in status.
- [x] (2026-09-07; `undocked` terminal state added) Retained `ros2/docking/status` exactly per the §3 field table
      (`state`, `feedback_state`, `physically_docked`, `drive_out_guard`,
      `reason`, `error_code`, `error_msg`, `started_at`, `retries`, `id`);
      Nav2 feedback → `staging | approaching | waiting_charge`, results →
      `docked | failed | canceled` with mapped `reason`.
- [ ] `EVT:NOCURRENT` cross-check warning (§4.4) — optional for the first cut.
- [x] (2026-09-07 — declared, default off; **NOT allowlisted**: the dock bridge shares the node name `mqtt_bridge_node`, so `/mqtt_bridge_node/set_parameters` is ambiguous on zenoh — give the dock instance a distinct node name first) P4 params (declared now, default off, allowlisted in `param_control`):
      `auto_dock_on_low_battery`, `auto_dock_on_mission_complete`,
      `auto_dock_delay_s` (5). Triggers: mission_state `paused`/`battery`
      → `stop` on `/mowing/mission_cmd` → wait `idle` (30 s) → `dock`;
      mission_state edge → `idle`/`complete` → delay → `dock`.
- [x] (2026-09-07; params not in `param_control` yet, see above) `config/topics.yaml`: `docking:` section (cmd/status topics,
      `dock_json_path`, `dock_stale_s`, link-loss comment) + the three
      params in `param_control`.
- [x] (2026-09-07) Built + `mowbot-mqtt-bridge.service` restarted; startup line OK, refusals `already_docked` / `bad_action`, cancel→idle verified over MQTT.

### 11.3 Tests

- [ ] §8.1 static checks (motors OFF, real dock) — after 11.1 + 11.2 are
      deployed, before anything moves.
- [ ] §8.2 field checklist — first item needs Phase B (11.5); Dock/Undock
      via `mosquitto_pub -t ros2/docking/cmd` until Phase C.

### 10.11 Field log 2026-09-12 — V-guided docking, first day

- Dock link: after a ROBOT zenoh-router restart the dock's publications
  stop propagating (TCP session stays up) → restart the dock zenoh router
  (`ros2/dock/launch/cmd {"id":"dock_zenoh","action":"restart"}`).
- V visibility (V as built, on the charger): seated 102 pts at 0.31 m;
  1.3 m → 20 pts, apex ±1 cm, axis ±6°/scan; 2 m aligned → NOT visible
  (scan plane hits the charger front below the V: slope) ⇒
  **`staging_x_offset` 1.2 m**. Off-axis > ~30° only the V's outside is
  seen (convex) → concavity check in the detector.
- Detector fixes from live runs: axis = bisector of the fitted arm
  directions (chord midpoint swung ±20° with a clipped arm); size
  tolerances tightened (a 0.44 m garden corner matched at ±45 %).
- Plugin fixes from live runs: transform at the scan stamp; target
  overshoot 0.25 m past the seated pose (graceful 1/r gain → zigzag in
  place 0.35 m short of the target); filter 0.05 → 0.2; final-stretch
  target LOCK (apex < 0.6 m, lateral < 3 cm, yaw < 3°) for a straight push.
- Heading: the map heading estimate is corrupted by every slip/spin
  (5°…167° errors seen); seeded by hand via `/set_pose` (both EKFs listen)
  with the dock-axis heading. **When the V is in view the true heading is
  `dock_axis_yaw − axis_angle_in_robot_frame`** — a free heading fix for
  dock_manager to apply automatically (TODO).
- Best run (dock7, from 5.9 m): staging nav OK → V acquired at 1.48 m
  (47 cm / 20° off after Nav2's staging tolerance) → 1 cm / 2° at 0.5 m
  → twisted 7° in the last 8 cm, wedged the TYRES in the funnel mouth at
  5 cm lateral, e-stop. Owner: the funnel catches the tyres before the
  body — more tyre clearance at the mouth / guide at plate height.
- Wheel slip on the slope in front of the dock made two runs spin; a
  firmer strip under the wheel tracks would help every approach.
- **Run dock8 (after filter 0.2 + overshoot + target lock): guidance
  complete** — staging 7 cm / 18° off → 0.6 m: 6 mm / 2.1° → target locked
  → straight push → **0.44 m: 7 mm / 1.1°** → blocked at 0.43 m (seated =
  0.31 m), pivoted 11° while pushing, tyres wedged in the funnel mouth
  again. Two identical stops at 0.42–0.43 m with near-perfect alignment ⇒
  **mechanical: the funnel catches the tyres 12 cm before seating** (mouth
  width vs tyre track at wheel height, or a lip on the slope). Owner action
  before more runs. Software TODOs after that: stall handling (docking
  server `use_stall_detection` from the hoverboard joint states → back off
  and retry instead of pivoting in place), automatic heading seed from the
  V in dock_manager.

### 10.12 FIRST AUTONOMOUS DOCKING — 2026-09-12, run dock11

From 6.2 m: staging navigation 28 s → V acquired at 1.51 m (1.6 cm /
13.8° off after Nav2's staging tolerance) → guided convergence → **aligned
4 mm / 1.5° at 0.59 m** → LOCK → open-loop straight push at 0.15 m/s →
seat switch at +36 s → "Robot is charging!" → status `docked`, dock
CHARGING 1.67 A. Seated: apex 0.32 m, 0.000 m lateral, axis −1°.

What finally made the last 12 cm work (runs dock8–dock10 wedged the
tyres there with near-perfect alignment): the final push must be
**open-loop straight** — a target frozen in odom still steered the robot
because the odom yaw drifts under slipping wheels on the slope, and every
"correction" pushed a tyre into the funnel wall (13° pivot at 0.25 m/s).
Now the plugin places the target straight behind the robot's current
heading every cycle → zero steering. Speeds 0.15/0.15 m/s,
v_angular_max 0.5, filter 0.2, overshoot 0.25 m, lock at 0.6 m / 3 cm / 3°.

Operational lessons of the day: **a bringup restart resets both EKFs and
throws away the heading calibration** — re-seed via `/set_pose`
immediately (heading from the V when in view: dock axis − V axis angle);
plugin code needs a process restart (a nav2 lifecycle RESET/STARTUP does
NOT reload the .so, and STARTUP reported a failure); controller speeds
are live-settable. The lidar idle power-off switches the lidar off
between runs.

**Repeat runs 2026-09-12 afternoon:** dock12 (from staging, 11 s) ✓,
dock13 (undock → GoTo 4 m / 45° off-axis → dock) ✓, dock14 (5 m on-axis)
✗ — lock at 1.6 cm / 2.7°, then the OPEN-LOOP push drifted/yawed 18° on
the slope and stuck (helped by hand). Fix: after the lock, **gentle
line-follow on the measured V axis** (fixed 0.5 m lookahead pure pursuit
from each scan) instead of open-loop. dock15 (5.5 m / 30° off the other
side) ✓ — drift +5.2° at 0.47 m corrected to +0.7° at 0.36 m. Score:
4/5 autonomous, 5/5 with the final method. Undock 5/5.

### 11.7 Lidar V detection (§10.10) — DONE 2026-09-12 (first docking, §10.12); hardening below

- [ ] Hardware (owner): taller V arms (200–250 mm vertical, centred ≈ 27 cm
      above ground), optional reflective tape — the V is invisible from 2 m
      on the slope (§10.10).
- [x] (2026-09-12) `staging_x_offset` → 1.2 m (nav2_params + dock.json
      `staging_offset_m`).
- [x] (2026-09-12, mowing_navigation) `dock_v_detector` node (scan → V fit →
      `detected_dock_pose` + marker), params: V width 0.32, depth 0.092,
      opening 120°, window, min points.
- [ ] Bench with the seated scan: apex at (−0.31, 0.00) lidar frame, then
      from 0.5 / 1 / 2 m on the axis and ±20° off-axis (robot pushed by hand).
- [x] (2026-09-12: `apex_to_base_m` 0.37, overshoot 0.25, lock) Plugin: `use_external_detection_pose`, `v_apex_to_base_m` 0.36,
      `external_detection_timeout` 1.0, filter; prior fallback.
- [x] (2026-09-12) nav2_params: enable detection, `fixed_frame: odom`; launch: start the
      detector in bringup (lidar must be ON — dock_manager F53 covers it).
- [x] (2026-09-12) Field: dock from staging with the e-stop ready — guidance
      verified to 7 mm / 1° at 0.44 m (dock8); blocked by the funnel.
- [x] (not needed for dock11 — the open-loop push docked with the funnel as
      built; keep in mind if wedges recur) Owner: funnel mouth tyre clearance;
      firmer strip under the wheel tracks in front of the dock.
- [ ] Stall handling → planned separately in [06_STALL_HANDLING.md](06_STALL_HANDLING.md)
      (dock_manager no-progress watchdog → cancel, back off 15 cm, retry ×2,
      reason `stalled`); deferred by the owner 2026-09-12.
- [x] (2026-09-12, bridge dock_manager; first check −36.2° → −40.4°)
      dock_manager: seed the map EKF heading from the V when in view
      (`/set_pose`, yaw = dock axis − V axis angle) — after docking, at
      undock start, periodically while docked.
- [ ] Then §8.2: ten clean dockings, off-axis starts, undock cold check
      (2026-09-12 so far: 5 undocks 5/5; dockings dock11–13, dock15 ✓,
      dock14 ✗ before the line-follow fix).
- [ ] Nice-to-have: keep the lidar on during docking sessions (F29 idle
      power-off adds a power-on wait to every command).

### 11.4 Mission node (`mowing_navigation`) — P4 gate, can ride with 11.2

- [x] (2026-09-12, verified live: start refused while seated) `start_mission`: refuse when `/dock/microswitch` (latched, cached
      like `motors_allowed`, fresh < 5 s) is true — "robot is in the charging
      dock - Undock first"; absent/stale topic ⇒ allow with a warn.
- [x] (2026-09-12) Built, `mowbot-launch-mow-mission.service` restarted.

### 11.5 LXC / web UI (05 Phase B **with** 11.1–11.2; Phase C after the first real runs)

- [x] (2026-09-12; HA rule not yet) ACL: `user webui` → `topic write ros2/docking/cmd`;
      HA rule `topic docking/cmd in 1 ros2/ ha/ros2/` + mosquitto restart.
- [x] (2026-09-12, deployed; record yaw = lidar V axis via `v_axis_yaw`) Phase B: `robot/fileserver.py` writable map + `config/dock.json`
      backups; `robot/version_watcher.py` entry; backend `GET/PUT
      /api/dock`, `POST /api/dock/record` (odometry cache, staleness 3 s,
      not-seated warning), writes `staging_offset_m: 2.0`; frontend
      `DockPanel`, markers on both maps (staging = pose + 2.0 m along yaw),
      Layers toggle, Dock-page position card. No staging-distance input.
- [x] (2026-09-12, deployed) Phase C: Dock page control row + `DOCK_REASON_TEXT` keyed by
      `status.reason`; status merge rule; "Undock & start" in
      `MissionControl`; Drive-page docked warning + `drive_out_guard`;
      staging→dock dashed line while `staging/approaching`.
- [x] (2026-09-12) Fix stale fw-0.2.0 strings in `lib/dockStatus.js` (rows 10/12).
- [x] (2026-09-12) Deploy: LXC rsync + `npm run build` + backend restart; robot
      `mowbot-fileserver` / `mowbot-version-watcher` restarts.

### 11.6 Field (§8.2) and P4

- [ ] Park + "Save dock at robot position" + first autonomous dock from
      3 m, 10/10 with charge current confirmed; misalignment drill —
      if lateral misses are consistent, **shorten `staging_x_offset`** first.
- [ ] Dock from several yard corners; undock → 2.0 m out, dock cold first.
- [ ] E-stop mid-dock; WiFi-off-at-contact (§4.4 cosmetic mismatch readable
      in UI); dock relocation test.
- [ ] Enable `auto_dock_on_mission_complete`, then
      `auto_dock_on_low_battery` (temporarily raise
      `mow_battery_low_voltage`), one at a time.
- [ ] Later / deferred: automatic top-up while parked (Q 3), adjustable
      staging distance from the web UI (Q 1), Phase D HA docking buttons,
      charge-session stats (05 A2).
