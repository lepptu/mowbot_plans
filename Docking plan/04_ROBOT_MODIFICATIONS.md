# 04 — Robot (Mowbot) Modifications

> Status: **PLANNED 2026-07-10.** Part of the docking master plan
> ([README.md](README.md)). Everything that changes on the robot:
> hardware, Nav2 config/launch, the bridge `dock_manager`, and mission
> integration. The robot's Arduino Nano and its firmware are **not
> touched** — the charge path is passive on the robot side.

## 1. Hardware changes (details/rationale in 01 §4–5)

- Two contact plates on the front chassis face (rigid chassis, clear of
  bumper travel), asymmetric geometry for polarity safety.
- Wiring: plates → 5 A fuse → ideal-diode module → battery charge input
  (parallel with the existing charge jack, through the BMS charge port).
- Verify: 0 V on plates undocked; bumper still triggers freely; plates
  wipe the dock springs over the full funnel tolerance (±3 cm entry).

## 2. Nav2: enable `opennav_docking` (installed, v1.3.10)

### 2.1 `nav2_params.yaml` — new `docking_server` section (draft)

```yaml
docking_server:
  ros__parameters:
    controller_frequency: 20.0
    initial_perception_timeout: 5.0
    wait_charge_timeout: 15.0          # relay closes on microswitch, current ramps fast; >5 s default for WiFi slack
    dock_approach_timeout: 45.0
    undock_linear_tolerance: 0.05
    undock_angular_tolerance: 0.1
    max_retries: 3
    base_frame: "base_link"
    fixed_frame: "odom"                # dock pose transformed map→odom once at action start; odom is smooth for control
    dock_backwards: false              # forward docking (README D10)
    dock_prestaging_tolerance: 0.5
    dock_plugins: ["mowbot_dock_plugin"]
    mowbot_dock_plugin:
      plugin: "opennav_docking::SimpleChargingDock"
      use_external_detection_pose: false   # blind RTK docking (README D2)
      use_battery_status: true             # isCharging = battery_state.current > threshold
      charging_threshold: 0.15             # A — dock ACS712 reads ~2 A when charging
      use_stall_detection: false
      docking_threshold: 0.05              # pose-based isDocked tolerance; tune vs RTK jitter
      staging_x_offset: -0.7               # staging pose 0.7 m behind dock pose along approach axis
      filter_coef: 0.1
    controller:
      k_phi: 3.0
      k_delta: 2.0
      v_linear_min: 0.10
      v_linear_max: 0.15                   # gentle contact; hoverboard low-speed floor applies (R13 lesson: too slow may not move)
```

⚠ **Verification step, not gospel:** at first bench launch run
`ros2 param dump /docking_server` and reconcile names/defaults against
the installed 1.3.10 (README open question 3). Also verify whether this
version exposes `use_collision_detection` (see §5).

### 2.2 `navigation.launch.py`

- Add `opennav_docking::DockingServer` to the composable container
  (verify component registration; fall back to the standalone
  `opennav_docking` node executable if not composable in 1.3.10).
- Remap `battery_state` → `/dock/battery_state` (published by the dock
  agent over zenoh, 03 §3.1).
- Add `docking_server` to the lifecycle manager `node_names` list.
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
| `ros2/docking/status` | robot → webui, **retained** | `{state, feedback_state, started_at, retries, error_code, error_msg, id}` |

`state`: `idle | staging | approaching | waiting_charge | docked |
undocking | failed | canceled` — mapped from `DockRobot`/`UndockRobot`
feedback (`NAV_TO_STAGING_POSE`, `CONTROLLING`, `WAIT_FOR_CHARGE`,
`RETRY`) + results.

Behavior:

- **`dock`**: read `~/pi_ws/mowing_data/config/dock.json` (fresh, per
  command); refuse with reason if missing/stale-schema. Send `DockRobot`
  (`use_dock_id:false`, pose+yaw from file, `navigate_to_staging_pose:
  true`).
- **`undock`**: sequence (02 §4 arc-avoidance): publish
  `dock/charge_enable_cmd false` → wait until `dock/charge_current`
  < 0.3 A (3 s timeout, proceed anyway with warn) → send `UndockRobot`
  → on result, re-publish `charge_enable_cmd true` (restore autonomy
  for the next docking).
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

## 4. Charging detection path & edge cases

### 4.1 Normal flow

Approach → plates mate → microswitch → dock firmware closes relay →
current ~2 A → `dock/battery_state.current` > 0.15 →
`SimpleChargingDock::isCharging()` true → `DockRobot` succeeds →
`dock_manager` status `docked`.

### 4.2 isDocked before isCharging

With `use_external_detection_pose: false` the plugin's docked test is
pose-based (within `docking_threshold` of the dock pose). Because the
pose was **recorded by parking** (README D4), the seated pose matches
the stored pose almost exactly; RTK jitter (2–3 cm) vs the 5 cm
threshold leaves margin. If field tests show flapping: raise
`docking_threshold` to 0.08–0.10 (the microswitch + wait-for-charge
step still guarantee real contact before success), or implement the
custom plugin (README D8) whose `isDocked()` is the microswitch topic.

### 4.3 Docking with a nearly-full battery (README open question 5)

CV taper on a full pack can sit below `charging_threshold` →
`FAILED_TO_CHARGE` after `wait_charge_timeout`. Mitigations, in order:

1. `charging_threshold: 0.15` is already low; even a full pack draws a
   brief surge on connect — bench-measure it (01 §8 dummy-load test).
2. Dock agent option `min_reported_current_while_seated` (e.g. report
   `max(measured, 0.2)` in `battery_state.current` while
   state ∈ {CHARGING, CHARGE_COMPLETE}): contact is genuinely verified
   by microswitch+relay+contact-voltage, so this is honest "we are
   charging/maintaining" semantics, not a lie. Cleanest zero-plugin fix.
3. Custom plugin (`isCharging()` = dock state ∈ {2,3}) — the "right"
   long-term shape if 2 feels hacky.

### 4.4 Failure surface

| Failure | Detection | Outcome |
|---|---|---|
| Misaligned entry, no microswitch | no charge current → `WAIT_FOR_CHARGE` times out | docking server retries (backs up to staging, re-approaches) up to `max_retries`, then action fails → UI shows error, robot parked at dock mouth |
| Contact but relay fault (weld/no-close) | dock fault code ≠ 0, no current | action fails; dock card shows fault; charging blocked by firmware until `clear_fault` |
| WiFi drops during `WAIT_FOR_CHARGE` | `battery_state` goes stale on the robot | treat as not-charging → retry/fail path; physical charging still starts (firmware autonomy) — recorded as a known cosmetic mismatch: robot may report failed dock while actually charging; microswitch state in `ros2/docking/status` disambiguates in the UI |
| E-stop mid-dock | existing e-stop signal | `dock_manager` cancels the action (motion stops via twist path already) |

## 5. Costmap / navigation environment around the dock

- The dock is a physical obstacle the LiDAR sees: it will be marked in
  the local costmap. The **staging pose must be far enough out
  (0.7 m+)** that Nav2 can reach it normally; the final approach is the
  docking server's own controller, not FollowPath.
- Start with docking-server collision checking **disabled/lenient**
  (else the dock itself blocks the final 0.3 m — the classic problem;
  verify the installed version's `use_collision_detection` /
  projection params on the bench). Risk is bounded: ≤0.15 m/s over
  0.7 m into a funnel.
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
| **No mowing while docked** | `start_mission` wrapper (mowing_mission_node) refuses start when docked: subscribe `/dock/microswitch` (passive cache, same style as other gates); refusal reason `docked` in mission state; UI hint "Undock first". If the topic is absent/stale (dock Pi down but robot physically docked is unlikely — microswitch implies dock powered), allow start with a warn. | mowing_navigation |
| **Undock-then-mow convenience** | Web UI: Start button while `physically_docked` offers "Undock & start" (UI sequences undock → wait idle+undocked → start). Robot-side auto-sequencing deliberately deferred. | frontend only |
| **Blade interlock** | None needed beyond the above: blade control paths already require a running/idle-manual context, and mission start is now refused while docked. Manual blade (BladeControl) while docked is physically pointless but harmless — optionally add the same `docked` refusal to `manual_mow` later. | — |
| **Charge stats** | P4 backlog: `stats.py` charge-session records (start/end, Ah estimate from dock current) from `ros2/dock/#` — the backend already consumes the broker. | LXC backend |

## 7. topics.yaml / allowlist / ACL deltas (robot side)

- `docking:` manager section (cmd/status mapping, link-loss policy,
  dock.json path).
- Bridge param allowlist: `auto_dock_on_low_battery` (P4).
- LXC ACL (user runs, exact commands provided at implementation):
  `webui` → `topic write ros2/docking/cmd`. (Dock-hardware ACLs are in
  03 §4.)
- Bridge rebuild + restart (new manager = new binary; nav2_msgs dep
  already present from goto_manager).

## 8. Test plan

### 8.1 Bench (no dock hardware needed)

- [ ] `docking_server` lifecycle-activates in the nav container;
      `ros2 param dump /docking_server` reconciled with §2.1.
- [ ] Fake `/dock/battery_state` publisher + dock.json at a pose near
      the bench robot pose (F32-era bench rig: non-identity map→odom TF,
      fake scan, e-stop/bumper publishers): `ros2/docking/cmd dock` →
      staging nav → approach → wait-charge; flip fake current to 2 A →
      action succeeds, retained status `docked`.
- [ ] No fake current → retries ×3 → `FAILED_TO_CHARGE` surfaced in
      status with error text.
- [ ] `cancel` in each phase; e-stop mid-approach; mission-not-idle
      refusal; missing dock.json refusal.
- [ ] `undock` sequence ordering observable: enable-off → (fake)
      current-drop → UndockRobot → enable restored.
- [ ] Bench discipline (recorded lessons): `setsid nohup … < /dev/null`,
      never pattern-kill nav2 names while production units run, expect a
      phantom stats mission if the bridge is up.

### 8.2 Field (P3 exit criteria)

- [ ] Manual park + "Save dock here" + first autonomous dock from 3 m,
      10/10 attempts with charge current confirmed.
- [ ] Dock from different yard corners (staging navigation across
      transits, keepouts respected).
- [ ] Undock → robot idle 0.7 m out, relay open before pull-out
      (watch dock card current during the sequence).
- [ ] Misalignment drill: offset the robot's approach by blocking one
      funnel side → verify retry behavior and clean failure.
- [ ] E-stop mid-dock; WiFi-off-at-contact (cosmetic mismatch of §4.4
      confirmed and readable in UI).
- [ ] Full loop: mow until N1 battery pause (temporarily raise
      `mow_battery_low_voltage`) → auto-dock (P4) → charge to complete →
      stats sane.
- [ ] Move the dock 2 m, re-record, dock again — the "relocation is
      easy" acceptance test.

## 9. Rollout order

1. Bench items 8.1 with fakes (needs 03's dock agent only for real
   hardware runs — fakes are enough here).
2. `bringup` restart (nav2_params + launch), bridge rebuild+restart,
   LXC ACL/frontend deploy (user-run commands).
3. Field P3 checklist, then enable P4 automation flags one at a time.
