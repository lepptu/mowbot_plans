# Scheduled Mowing — weekly schedule (web UI + robot side)

> Status: **PLANNED 2026-09-13, no code written.** Builds on the docking
> master plan ([../Docking plan/README.md](../Docking%20plan/README.md)) —
> in particular `dock_manager` (04 §3, §6) and the web UI conventions of
> 05. Everything a scheduled run needs on the robot already exists as a
> command, parameter or service; this plan adds a **clock and a sequencer**
> around them plus a **Schedule page** in the web UI.
>
> Goal (owner, 2026-09-13): a weekly calendar in the web UI where each entry
> says *day, time, which areas (one, several or all), LiDAR obstacle
> detection on/off, perimeter LiDAR mode*. At the scheduled time the robot
> undocks by itself, mows, returns to the dock. The dock tops the battery up
> before the run. A scheduled run **never starts unless the robot is sitting
> in the charging dock.** Weeks repeat — the calendar shows weekdays only,
> no dates. Block length on the calendar = estimated duration of the run.
> **Scheduled missions must be switchable on and off** — as a whole and
> per run (§2.1).

## 0. The one-paragraph design

The schedule is one JSON file on the robot
(`~/pi_ws/mowing_data/config/schedule.json`), edited from a new **Schedule**
page through the existing backend → fileserver `PUT` path (same as
`dock.json`, sha1-preconditioned, backed up, version-watched). The **robot
bridge** gets a new `schedule_manager` (the `dock_manager`/`goto_manager`
pattern: one more manager inside `mowbot_mqtt_bridge`, zero extra processes)
that reads the file, keeps the Pi's local clock, and at `T0` runs one fixed
sequence entirely robot-side: *preconditions (docked, idle, e-stop clear,
motors on, Nav2 up, battery OK) → apply the run's profile (`area_filter`,
`lidar_enabled`, `perimeter_lidar_mode`) as transient parameters → reset
progress → undock → mission `start` → mow (charge breaks via the existing
low-battery auto-dock + resume-after-charge) → return to dock → restore the
parameters*. `T0 − lead` it asks `dock_manager` for a charge-to-full so the
pack is full when the run starts. A retained `ros2/schedule/status` topic
tells the UI what is next, what is running and why something was skipped.
The LXC only stores/serves the file and computes **duration estimates**
from the coverage metadata calibrated against real mission statistics it
already keeps.

```
 browser  ──GET/PUT /api/schedule──►  LXC backend  ──PUT config/schedule.json──►  robot fileserver
   ▲                                     │ /api/schedule/estimates                       │ (version_watcher →
   │ ros2/schedule/status (retained)     │ (coverage est × calibration from /api/stats)  │  ros2/schedule/version)
   │ ros2/schedule/cmd (run_now/cancel/  ▼                                                ▼
   └────────────── MQTT ────────────  mosquitto  ◄──────────────────────────  mowbot_mqtt_bridge
                                                                              └ schedule_manager (NEW)
                                                                                 ├ reads schedule.json (mtime poll)
                                                                                 ├ clock: Pi localtime (Europe/Helsinki)
                                                                                 ├ → param_manager (transient set, reset_progress)
                                                                                 ├ → dock_manager  (charge_full, undock→start, return policy)
                                                                                 ├ → launch_manager (mow_mission unit state/start)
                                                                                 └ publishes ros2/schedule/status
```

## 1. Facts this plan rests on (verified on the live system 2026-09-13)

### 1.1 What already exists and is reused unchanged

| Need | Exists as | Where |
|---|---|---|
| Start / stop a mission | `ros2/mission/cmd` → `/mowing/mission_cmd` (`start`, `pause`, `resume`, `stop`, `skip_segment`, `calibrate_heading`), state on retained `ros2/mission/state` `{state, reason}` | `mowing_mission_node.cpp` ~L1485, `topics.yaml` L224–262 |
| Docked start gate (§11.4) | `start_mission` refuses while `/dock/microswitch` is fresh (< 5 s) and `true`; stale ⇒ warn + start | `mowing_mission_node.cpp` ~L1360–1400 |
| Route-complete auto reset (F27) | `start` on a complete route calls `/mowing/reset_progress` then starts | same, ~L1401–1440 |
| Reset progress on demand | `/mowing/reset_progress` (`std_srvs/Trigger`), already wrapped by `param_manager` action `reset_progress` | `mowing_route_server.cpp` L79, `param_manager.cpp` L274 |
| Choose areas | route server dynamic param `area_filter` (string array of route ids `<area>_coverage`, ≤ 32, empty = all); applied at load or deferred to the next segment boundary | `mowing_route_server.cpp` L126–290, allowlist `topics.yaml` L585 |
| LiDAR obstacle detection | mission param `lidar_enabled` (bool, live: `LidarModeManager::setOperatorEnabled`) | `mowing_mission_node.cpp` L115, L1183 |
| Perimeter LiDAR mode | mission param `perimeter_lidar_mode` ∈ `off | outermost_only | all_perimeters`, read by `ConfigureSensorsForSegment` at every segment start | L128, L839; `bt_nodes/configure_sensors_for_segment.hpp` |
| Set params from the bridge | `ParamManager` (allowlist → validate → write `mowing_overrides.yaml` → async `set_parameters`) | `param_manager.{hpp,cpp}` |
| Undock / dock / charge-to-full | `DockManager::handle_command` actions `dock`, `undock`, `cancel`, `charge_full`; params `auto_dock_on_mission_complete`, `auto_dock_on_low_battery`, `auto_dock_delay_s`, `topup_*`, `storage_*`, `resume_after_charge`, `resume_at_voltage` | `dock_manager.{hpp,cpp}` |
| Undock → start sequencing | `resume_tick` / `resume_after_undock` (resume after charging, 2026-09-13) — exactly "undock, then publish `start`" | `dock_manager.cpp` L1343–1420 |
| Mission process at boot | `mowbot-launch-mow-mission.service` and `mowbot-launch-bringup.service` are both `WantedBy=multi-user.target`; the mission comes up **idle** (F27, `autostart=false`) | `robot/*.service` |
| Unit state / start from the bridge | `LaunchManager` (`running | starting | stopped | failed | unknown`), ids `bringup`, `mow_mission` | `launch_manager.cpp` L27–37 |
| Robot-side file writes from the UI | fileserver `WRITABLE` table (`/mow_area/mow_areas.json`, `/config/dock.json`), token + `If-Match` sha1, backups; `version_watcher.py` `FILES` map publishes retained `ros2/*/version` | `robot/fileserver.py` L51–68, `robot/version_watcher.py` L23–27 |
| Backend file pattern | `GET /api/dock` (+ `X-Dock-Sha1`), `PUT /api/dock` (`base_sha1`, 409 on conflict), `RobotData.put_dock` | `backend/main.py` L428–447, `robot_client.py` L239 |
| Estimated mowing time per area | `coverage/mow_coverage_paths.json` `areas[].meta.stats.est_time_min` (nominal 0.35 m/s + 8 s per segment) | served by `/api/coverage` |
| Real mission durations | `/api/stats` `missions[]` (`mow_min`, `transit_min`, `paused_min`, `areas_mowed`, `battery_delta_v`, `voltage_start/end`) | `backend/stats.py` |
| Real charge durations | `/api/dock/sessions` periods/bursts (`v_pack_start`, `duration_min`, `ah`, `end_state`) | `backend/dock_stats.py` |

### 1.2 Clocks and time zones

- Robot Pi: `Europe/Helsinki`, `NTPSynchronized=yes` (systemd-timesyncd,
  marker file `/run/systemd/timesync/synchronized`), hardware RTC present
  (`/dev/rtc0`, valid time). **The Pi's local clock is the only clock that
  interprets schedule times.**
- LXC: `Etc/UTC`. The backend never converts schedule times; it stores and
  serves the strings. The browser never converts either — "13:00" is
  displayed as "13:00". DST is handled by `localtime_r` on the Pi.

### 1.3 Measured durations — why the calendar must not use `est_time_min` raw

Completed missions from `/api/stats` (2026-07-31 … 2026-09-13):

| Date | Areas | `est_time_min` | mow + transit (min) | ratio | ΔV (start→end) |
|---|---|---|---|---|---|
| 09-13 | sivupiha2 | 72.7 | 101.0 + 14.8 = 115.8 (+10.8 paused) | **1.59** | 41.2 → 35.9 (5.3 V) |
| 09-12 | takapiha | 29.4 | 43.9 + 5.0 = 48.9 | 1.66 | 41.7 → 38.8 |
| 09-12 | etupiha | 33.8 | 43.8 + 6.3 = 50.1 | 1.48 | 41.3 → 38.4 |
| 08-05 | etupiha + takapiha | 63.2 | 81.8 + 7.7 = 89.5 (+5.6 paused) | 1.42 | 41.3 → 37.8 |
| 07-31 | takapiha | 29.4 | 30.4 + 0.9 = 31.3 | 1.06 | 36.9 → 36.0 |

Median ratio ≈ **1.5** (the generator assumes 0.35 m/s; `mowing_speed_mps`
is 0.3, plus turns, goal approach and recoveries). The one-charge endurance
is visible in the sivupiha2 run: 116 active minutes used 5.3 V and ended at
35.9 V, 0.9 V above the `mow_battery_low_voltage` cutoff (35.0). ⇒ roughly
**140 min of mowing per charge**. Consequences:

- `Full` (all four areas, Σ est 215.7 min → ≈ 324 min real) **cannot be done
  on one charge**; a scheduled Full run is a *multi-charge run* and depends
  on the low-battery auto-dock + resume-after-charge chain (04 §6).
- sivupiha1 + sivupiha2 (Σ est 152.5 → ≈ 229 min) needs one charge break.
- etupiha or takapiha alone (≈ 50 min) fit comfortably.

Charge times from `/api/dock/sessions` (fw 0.2.x, 1.6–1.7 A):

| From (robot pack) | To | Duration | Ah |
|---|---|---|---|
| 40.5 V (top-up threshold) | 41.7–42.0 V, dock state 4/5 | 41–45 min | 0.62–0.68 |
| 38.9 V | COMPLETE (5) | 56.5 min | 1.22 |
| 38.1 V | COMPLETE (5) | 102.9 min | 2.41 |
| ~35.9 V (after a low-battery stop) | COMPLETE | not yet measured — extrapolated ≈ 120 min | ≈ 2.8 |

⇒ A parked robot with top-up on (never below 40.5 V) is full again within
**45 min**; with storage mode on (39.6–40.6 V band) allow **60 min**. The
owner chose a **30 min** default lead (adjustable) — the run then starts
with the pack still charging but above `min_start_voltage`; raise the lead
once the dock statistics show how far short it falls. A charge break
inside a run costs ≈ **2 h**.

### 1.4 Current live settings that matter (2026-09-13, `mowing_overrides.yaml`)

`auto_dock_on_mission_complete: true`, `auto_dock_on_low_battery: true`,
`resume_after_charge` not set (default **false**), `topup_enabled` default
true, `storage_mode_enabled` default false, `area_filter:
["sivupiha2_coverage"]`, `lidar_enabled: true`, `perimeter_lidar_mode:
"all_perimeters"`, `mow_battery_low_voltage: 35.0`, `resume_on_start:
false` (route server; applies at route load only).

## 2. Decisions

| # | Decision | Rationale / alternative rejected |
|---|---|---|
| D1 | **The scheduler runs on the robot, inside `mowbot_mqtt_bridge`** as `schedule_manager.{hpp,cpp}` | Same reasons as every other manager (launch/param/goto/dock): no new process on the Pi, all interlocks robot-side, works with the LXC/WiFi down. Rejected: LXC backend as the clock — it would have to drive the robot over MQTT like a browser, needs write ACLs for the `backend` account, and dies with the LXC or the WiFi. |
| D2 | **Schedule = one JSON file on the robot** (`config/schedule.json`), written only by the web UI through backend → fileserver PUT; the bridge only reads it (mtime poll) | Same path as `mow_areas.json`/`dock.json`: sha1 conflict detection, backups, version topic, editable while the bridge is down. The bridge never writes it (no write races). |
| D3 | **Master enable is a bridge ROS parameter `schedule_enabled`** (allowlisted under `mqtt_bridge_node`, persisted in `mowing_overrides.yaml`, default **false**); per-run `enabled` and the estimate/charge options live in the file | One switch that works even when the fileserver is unreachable, and it follows the Settings/HA toggle pattern (05 Phase D). A `hold` (pause until a time) is bridge state in `schedule_manager_state.json`. |
| D4 | **Times are Pi local time, weekday + HH:MM, weekly repeat, no dates** | Owner requirement. DST handled by `localtime_r`. No timezone field is interpreted anywhere; the file carries `"timezone": "Europe/Helsinki"` for display only. |
| D5 | **Dock gate is hard**: a run fires only if `/dock/microswitch` is fresh **and** true (via `dock_manager`'s cache). Never started from the lawn. **Owner decision 2026-09-13:** option `wait_for_dock` (default **on**) — when on, a robot that is not docked at `T0` is waited for within the late-start window (`late_start_min`, phase `waiting_dock`) and the run starts the moment it is seated (all other preconditions re-checked); when off, not docked at `T0` ⇒ **skipped** immediately with reason `not_docked`. Either way the window's end ⇒ `not_docked` | Owner requirement ("must not be able to start if not in the dock"). Waiting reuses the existing late window and covers "I was test-driving it just before the slot"; the off setting is for owners who do not want a hand-docked robot to start by itself. The mission node's own §11.4 gate is the opposite check; both stay. |
| D6 | **Pre-charge**: at `T0 − precharge_lead_min` (**owner decision 2026-09-13: default 30, adjustable in the Rules card**) the manager requests `charge_full` from `dock_manager` (one-shot: releases a storage hold, pulses a COMPLETE dock) and suppresses the storage hold until the run ends. **The scheduler never switches `topup_enabled` (or storage mode) on or off by itself** — Settings → Docking owns those; the Schedule page only shows a hint when both are off | Measured: 45–60 min from the parked band to COMPLETE (§1.3), so 30 min may start a run slightly short of full — accepted; D7's `min_start_voltage` / `require_full_charge` are the guard, and the lead is tuned from the dock statistics later. Reuses the existing `charge_full_requested_` path — no new charging logic. |
| D7 | **Start condition at T0**: pack ≥ `min_start_voltage` (default **40.5 V**, = the top-up threshold) **or** dock reports COMPLETE. Option `require_full_charge` (default off) waits for COMPLETE up to `start_window_min` (default **30**) and then starts anyway if ≥ `min_start_voltage`, else skips `battery_low` | "Top up before the run" without making a 5-minute charger hiccup cancel the mowing. |
| D8 | **Return to dock is forced for scheduled runs**: mission `idle` with reason `complete`, `failed` **or `operator` (Stop)** ⇒ dock, regardless of the Settings toggle `auto_dock_on_mission_complete`. Low battery ⇒ stop + dock is likewise forced. **Owner decision 2026-09-13: Stop means "end the mission and dock" everywhere** — also for manual missions whenever `auto_dock_on_mission_complete` is on (extends Docking plan 04 §6 "auto-dock on mission complete" to the operator stop); **Pause** is the "hold here, I decide" button (no docking, resumable). The Stop button must say so ("Stop & dock"). | Unattended robot must go home; Pause already covers the "stay put" case, so one meaning of Stop is easier to trust. Implemented as a "run policy" flag `dock_manager` honours while a scheduled run is active (§4.3) plus the small extension of the existing on-complete rule. |
| D9 | **Charge breaks follow Settings → Docking `resume_after_charge`** (**owner decision 2026-09-13: option A, never forced**; recommend ON for scheduling); the UI warns when a run's estimate exceeds one charge and resume is off. Runs never start or **resume** inside quiet hours (`quiet_from`/`quiet_until`, default **21:00–07:00**, **owner decision 2026-09-13: accepted, both times adjustable in the Rules card**); a mission that is already mowing is not interrupted by quiet hours | Owner said other settings come from the Settings page. Quiet hours stop a Saturday "Full" run from resuming at 23:00 after its second charge. |
| D10 | **Progress is reset at every scheduled start** (`/mowing/reset_progress`) — **owner decision 2026-09-13: always, no per-run exception, no global toggle** | Weekly runs are fresh mows; without the reset, an interrupted etupiha run would leave 60 % of etupiha marked done for next week. Charge-break resumes inside a run keep the in-memory progress (reset happens only at T0). Side effect documented: a manually interrupted mission's progress is wiped by the next scheduled start. |
| D11 | **Run profile is applied as transient parameters** (`area_filter`, `lidar_enabled`, `perimeter_lidar_mode`) through a new `ParamManager::apply_transient()` — validated by the allowlist, sent with `set_parameters`, **not** written to `mowing_overrides.yaml`; the previous live values are remembered in the state file and restored when the run ends | The Settings page keeps showing the operator's own values; a bridge restart mid-run still restores (state file). Rejected: persisting via the normal `set` (Settings page would silently change every week). |
| D12 | **Duration estimate** = Σ `est_time_min` × `calibration_factor` + `overhead_min` (undock, transit to the first area, docking; default **8**), plus `charge_break_min` (default **120**) for every full `mow_min_per_charge` (default **140**) of mowing beyond the first; `calibration_factor` default **1.5**, refined by the backend from completed missions in `/api/stats` (median of (mow+transit)/Σ est over the last 10 completed missions with ≥ 90 % coverage) | §1.3. The factor and endurance are recomputed on the LXC where the history lives; the robot never needs them. |
| D13 | **Missed runs**: a run fires only within `[T0, T0 + late_start_min]` (default **30**); after that it is recorded as `skipped/missed`. Each run fires at most once per ISO week (`last_fired` per run id in the state file) | A power outage at 13:00 must not start the mower at 17:30 when the robot comes back. |
| D14 | **Overlaps**: while a scheduled run is active (any phase, including charge breaks), a later slot is skipped with `previous_run_active`; the editor warns about overlaps using the estimates | Simple and predictable; the estimate tells the owner when to place the next run. |
| D15 | **Unattended limits**: `max_run_min` per run (default = estimate × 2, floor 120) — on expiry the manager sends `stop` and docks, outcome `timeout`; a `start` that does not leave `idle` within 15 s ⇒ `start_refused`; undock failure ⇒ `undock_failed`, run ends docked | Every scheduled run must terminate on its own. |
| D16 | **No `mowing_navigation` changes** in Phase 1–3 | Everything is reachable through existing params/cmd/services. Optional later: publish the `mission_cmd` result so `start_refused` carries the mission's reason text. |
| D17 | **Skip ≠ alert storm**: skipped/failed scheduled runs raise one dismissable AlertBanner line (keyed by `last.at`) and an HA event; nothing retries on its own within the same slot | Owner sees why nothing happened, without the robot trying every minute. |

### 2.1 Enabling and disabling scheduled missions (owner requirement 2026-09-13)

Three switches, from coarse to fine. All three are honoured robot-side by
`schedule_manager`; the UI only reflects them.

| Level | What | Where it lives | How it is set | Effect |
|---|---|---|---|---|
| **Master switch** | `schedule_enabled` — "scheduled missions ON/OFF" | bridge ROS parameter, allowlisted under `mqtt_bridge_node`, persisted in `mowing_overrides.yaml` (survives reboots and bridge restarts), **default OFF after install** | Schedule page header toggle (press-again confirm when turning ON, the `DockingSettings` pattern) via `ros2/mowparams/cmd`; Settings → Docking gets the same toggle for symmetry; HA switch in Phase 4 | OFF ⇒ nothing is ever started by the clock: no pre-charge request, no `run_now` (refused `disabled`), status `enabled: false`, `next: null`. The calendar stays editable. |
| **Per-run switch** | `enabled` on each run in `schedule.json` | the schedule file | run editor toggle; quick toggle in the block's context menu (tap-and-hold / right-click) | a disabled run is drawn striped/dimmed, never fires, never pre-charges; `runs[].next_at = 0` |
| **Temporary hold** | `hold_until` (epoch) | `schedule_manager_state.json` (bridge-owned) | Up next card "Hold 24 h / until Monday / release"; `ros2/schedule/cmd {"action":"hold","hours":N}` / `{"action":"release"}`; HA button in Phase 4 | like OFF until the time passes, then the schedule resumes by itself; shown as "on hold until …" in the header and the Up next card |

Rules:

- **Switching OFF never interrupts a run that is already out.** An active
  run continues to its normal end (mowing → return to dock → params
  restored); **Stop** on the Mowbot page (= stop & dock, D8) or the
  **Cancel run** button on the Schedule page (same thing) is the way to
  end it now; **Pause** holds it in place. Rationale: an operator flipping the master switch while the
  robot is on the lawn most likely wants "no *more* runs", and an
  unattended stop on the lawn is worse than finishing. `pre-charging` /
  `waiting_charge` phases *are* aborted by OFF (nothing has moved yet).
- Turning ON does not fire a slot whose start time has already passed
  (D13 late window still applies from the original `T0`, not from the
  moment of enabling) — so enabling at 13:10 for a 13:00 run starts it,
  enabling at 13:40 does not.
- The Schedule page shows the state unmistakably: header pill
  **"Scheduled missions: ON / OFF / ON HOLD until …"**; when OFF or on
  hold the week grid gets a dimmed banner "Nothing will start
  automatically" and the Up next card says why. The Mowbot page mission
  banner is untouched (it reports the mission, not the schedule).
- Status topic carries all three: `enabled`, `hold_until`, per-run
  `enabled` inside `runs[]`, plus `active` so the UI can show "OFF, but
  the current run finishes at ≈ 15:40".
- Bridge restart / robot reboot: the master switch comes back from the
  overrides file, the hold from the state file — no surprise re-enable.

## 3. Data contracts

### 3.1 `config/schedule.json` (written by the UI via the backend, read by the bridge)

```json
{
  "version": 1,
  "timezone": "Europe/Helsinki",
  "runs": [
    { "id": "mon-etupiha",  "day": "mon", "time": "13:00", "areas": ["etupiha"],
      "lidar_enabled": true,  "perimeter_lidar_mode": "all_perimeters", "enabled": true, "label": "" },
    { "id": "tue-takapiha", "day": "tue", "time": "12:00", "areas": ["takapiha"],
      "lidar_enabled": true,  "perimeter_lidar_mode": "all_perimeters", "enabled": true, "label": "" },
    { "id": "wed-sivu",     "day": "wed", "time": "14:00", "areas": ["sivupiha1", "sivupiha2"],
      "lidar_enabled": false, "perimeter_lidar_mode": "off",            "enabled": true, "label": "" },
    { "id": "sat-full",     "day": "sat", "time": "12:00", "areas": [],
      "lidar_enabled": true,  "perimeter_lidar_mode": "outermost_only", "enabled": true, "label": "Full mow" }
  ],
  "options": {
    "precharge_lead_min": 30,
    "require_full_charge": false,
    "min_start_voltage": 40.5,
    "start_window_min": 30,
    "late_start_min": 30,
    "wait_for_dock": true,
    "quiet_from": "21:00",
    "quiet_until": "07:00",
    "max_run_factor": 2.0
  },
  "updated_at": "2026-09-13T10:00:00Z"
}
```

Rules (validated in the backend, subset re-checked by the fileserver
validator, and again by the bridge on load — the robot side is the authority
as with areas):

- `runs` ≤ 32; `id` `^[a-z0-9_-]{1,32}$` unique; `day` ∈ `mon…sun`; `time`
  `HH:MM` on a 5-minute grid; `areas` = **bare mow-area names** (as in
  `/api/areas` keys, e.g. `etupiha`), `[]` = all areas ("Full"); every name
  must have a coverage entry (`/api/coverage` `area_name`); `lidar_enabled`
  bool; `perimeter_lidar_mode` ∈ `off | outermost_only | all_perimeters`;
  `label` ≤ 40 chars; per-run optional `max_run_min` (int, overrides
  `max_run_factor`).
- The bridge maps names to route ids by appending `_coverage` (the same
  convention the backend strips in `routeAreaToName()` — see the 2026-07-08
  id-namespace lesson). A name without a coverage entry at run time is
  dropped with a warning; if nothing is left the run is skipped
  `no_areas`.
- Options: `wait_for_dock` bool (default true, D5). Ranges: lead 0–240 (default 30), window 0–120, late 0–120, voltage 34–42,
  factor 1–4; quiet hours may wrap midnight; `quiet_from == quiet_until`
  = no quiet hours.

### 3.2 `config/schedule_manager_state.json` (bridge-owned)

```json
{ "hold_until": 0,
  "last_fired": { "mon-etupiha": "2026-W37", "sat-full": "2026-W36" },
  "active": { "id": "wed-sivu", "phase": "mowing", "started_at": 1789390800,
              "restore": { "mowing_route_server": { "area_filter": ["sivupiha2_coverage"] },
                           "mowing_mission": { "lidar_enabled": true, "perimeter_lidar_mode": "all_perimeters" } } },
  "history": [ { "id": "mon-etupiha", "at": 1789218000, "outcome": "completed", "ended_at": 1789221400, "mow_min": 44.1 } ] }
```

`history` keeps the last 20 outcomes (the LXC `/api/stats` has the full
mission record; this is only for the status topic and the calendar's
"last week" ghosts).

### 3.3 MQTT — `schedule:` section of `topics.yaml`

| Topic | Dir | Payload |
|---|---|---|
| `ros2/schedule/status` | robot → UI, **retained**, republished on change + every 30 s | see below |
| `ros2/schedule/cmd` | UI/HA → robot, never retained | `{"action": "reload" \| "run_now" \| "cancel" \| "skip_next" \| "hold" \| "release", "id"?, "hours"?}` |
| `ros2/schedule/version` | version_watcher, retained | `{sha1, mtime}` — backend refetch trigger (same as `ros2/dock/pose_version`) |

Status field contract (the UI is written against this):

| Field | Type | Meaning |
|---|---|---|
| `enabled` | bool | `schedule_enabled` param |
| `hold_until` | int | epoch s, 0 = none (from `hold`) |
| `clock_ok` | bool | NTP synchronized marker present (or RTC time plausible) |
| `file` | `{sha1, loaded_at, error}` | `error` non-empty = last parse/validation failure, previous good schedule stays active |
| `runs` | `[{id, day, time, enabled, next_at, last_outcome, last_at}]` | one row per run, `next_at` = epoch of the next occurrence (0 when the run or the schedule is disabled) |
| `next` | `{id, at, precharge_at} \| null` | the earliest enabled run |
| `active` | `{id, phase, started_at, since, mission_state, mission_reason, charge_breaks} \| null` | `phase` ∈ `precharging \| waiting_dock \| waiting_charge \| preparing \| undocking \| mowing \| charging \| returning` |
| `last` | `{id, at, outcome, reason, ended_at}` | `outcome` ∈ `completed \| partial \| skipped \| failed \| canceled \| timeout`; `reason` per the list below |
| `id` | string | echo of the last command id |

Skip / end reasons (goto/dock convention, plain strings): `disabled`,
`hold`, `quiet_hours`, `not_docked`, `dock_offline`, `mission_not_idle`,
`mission_state_unknown`, `mission_unit_down`, `bringup_down`, `estop`,
`motors_off`, `nav2_unavailable`, `battery_low`, `previous_run_active`,
`missed`, `clock_unsynced`, `no_areas`, `param_refused`,
`reset_failed`, `undock_failed`, `start_refused`, `operator_stop`,
`mission_failed`, `dock_failed`, `timeout`, `bridge_restart`.

## 4. Robot side

### 4.1 `schedule_manager.{hpp,cpp}` (new, `mowbot_mqtt_bridge`)

Wiring in `mqtt_bridge_node.cpp` mirrors `dock_mgr_`: constructed when
`config_.schedule.enabled`, `command_handlers_[cmd_topic]`, retained
republish on reconnect (`status_payload()`), 1 s tick timer on the same
single-threaded executor. It gets **non-owning pointers** to
`ParamManager`, `DockManager`, `LaunchManager` (all live in the node) — the
new internal calls are listed in §4.2–4.4. ROS inputs it subscribes itself:
`hoverboard/motors_allowed` (latched Bool), `eStop_status`. Everything else
it asks `dock_manager` for (docked, dock fresh, dock state, pack voltage,
mission state + reason + freshness, undock client readiness).

**Tick (1 Hz):**

1. Reload `schedule.json` when its mtime changed (stat every 5 s; parse +
   validate; on error keep the previous schedule, set `file.error`).
2. Compute `now` (`localtime_r`), ISO week, and for every enabled run its
   next occurrence `T0` (this week if `T0 + late_start_min > now`, else next
   week). `next` = min over runs.
3. **Pre-charge**: if `schedule_enabled`, no hold, the run is enabled,
   `next.at − now ≤ precharge_lead_min·60`, no pre-charge was requested
   for this occurrence and the robot is docked with fresh dock telemetry ⇒ `dock_mgr_->request_charge_full("schedule <id>")`,
   `dock_mgr_->set_storage_suppressed(true)`, `active = {id, phase:
   precharging}`.
4. **Fire**: `now ∈ [T0, T0 + late_start_min·60]` and not fired this week
   ⇒ preconditions (§4.1.1). All good ⇒ mark fired (state file) and start
   the sequence (§4.1.2). Any failure ⇒ mark fired, `last = {skipped,
   reason}`, alert. (`T0 + late_start_min` passed without firing ⇒
   `missed`.)
5. **Active run supervision** (§4.1.3).
6. Publish status on change, at least every 30 s.

**4.1.1 Preconditions at T0 (all must hold; first failing reason wins):**
`schedule_enabled` param true (`disabled`) → `hold_until < now` (`hold`) →
not inside quiet hours (`quiet_hours`) → `clock_ok` (`clock_unsynced`) →
no active run (`previous_run_active`) → dock telemetry fresh
(`dock_offline`) → `physically_docked` (`not_docked`; with `wait_for_dock` on: stay in phase `waiting_dock` and re-check every tick until seated or `T0 + late_start_min`, then `not_docked`) → e-stop clear
(`estop`) → `motors_allowed` true (`motors_off`) → `bringup` unit running
(`bringup_down`) → `mow_mission` unit running, else start it and wait ≤ 60 s
for a fresh `idle` (`mission_unit_down`) → mission state fresh and `idle`
(`mission_not_idle` / `mission_state_unknown`) → undock action server ready
(`nav2_unavailable`) → pack ≥ `min_start_voltage` or dock COMPLETE
(`battery_low`; with `require_full_charge` wait up to `start_window_min`
in phase `waiting_charge` first).

**4.1.2 Run sequence (phase `preparing` → `undocking` → `mowing`):**

1. Resolve areas → route ids; `param_mgr_->apply_transient(...)`: capture
   the current live values of `area_filter`, `lidar_enabled`,
   `perimeter_lidar_mode` (from `ParamManager`'s confirmed `live` map or a
   `get_parameters` round trip) into `active.restore`, save the state file,
   then set the run's values (3 s service watchdog each; refusal ⇒
   `param_refused`, restore, end).
2. `param_mgr_->reset_progress(cb)` — always (D10; existing client, 5 s
   watchdog; failure ⇒ `reset_failed`, restore, end).
3. `dock_mgr_->start_mission_from_dock("sched-<id>", cb)` — the shared
   undock-then-start sequencer (§4.3). Result `undocked` + `start` sent ⇒
   phase `mowing`; anything else ⇒ `undock_failed` / `start_refused`.
4. Mission must leave `idle` within 15 s of `start` ⇒ else `start_refused`
   (with D16 later: the mission's own refusal text).

**4.1.3 Supervision while active:**

| Observation | Action |
|---|---|
| mission `running`/`paused` | phase `mowing`; count `charge_breaks` on `paused/battery` → `idle` edges |
| `dock_manager` reports `resume_pending` and robot docked | phase `charging` (the existing low-battery auto-dock + resume-after-charge chain is doing the work; forced on by the run policy §4.3) |
| quiet hours begin while `charging` | tell `dock_manager` `clear_resume("quiet_hours")` ⇒ run ends `partial/quiet_hours` |
| mission `idle` with reason `complete` | phase `returning` (the run policy makes `dock_manager` dock regardless of the Settings toggle); on `docked` ⇒ `completed`; dock failure ⇒ `failed/dock_failed` (robot stays on the lawn — alert) |
| mission `idle` with reason `failed`, `tree_failure`, `nav_failure`, `drive_telemetry_lost`, … | `returning` → outcome `failed/<reason>` (dock attempted) |
| mission `idle` with reason `operator` (Stop) | phase `returning` — dock (D8); outcome `canceled/operator_stop` once docked |
| `now − started_at > max_run_min` | send `stop`, then `returning`, outcome `timeout` |
| e-stop while mowing | the mission's own `safety_hold`; the run waits (max_run_min still ticking) |
| master switch OFF / hold set while `precharging`, `waiting_dock` or `waiting_charge` | abort the pre-charge (release storage suppression), outcome `skipped/disabled` |
| master switch OFF / hold set while `mowing`, `charging` or `returning` | nothing — the run finishes normally (§2.1); only `cancel` stops it |
| any end | `param_mgr_->restore_transient(active.restore)`, `set_storage_suppressed(false)`, clear run policy, append `history`, publish `last`, clear `active` |

Bridge restart with `active` in the state file: if the mission is
`running/paused` ⇒ re-attach as `mowing`; if docked + idle ⇒ finish
`partial/bridge_restart` and restore params; otherwise (robot idle on the
lawn) ⇒ `returning` (try to dock once).

**Commands** (`ros2/schedule/cmd`): `reload` (re-read the file now),
`run_now {id}` (run the sequence immediately — same preconditions incl. the
dock gate; for testing and "mow it now"), `cancel` (active run: `stop` the
mission and dock, outcome `canceled` — identical to pressing Stop, D8), `skip_next` (mark the next
occurrence fired), `hold {hours}` / `release` (`hold_until`).

### 4.2 `ParamManager` additions

- `bool apply_transient(node, {name: value}, done_cb)` — allowlist +
  `validate_value` as today, `set_parameters` via `apply_to_node`, **no**
  `write_overrides()`; refreshes `live` so `ros2/mowparams/status` shows the
  run's values with `source: live` (the stored value stays what the
  operator set — the Settings page shows "live ≠ stored" via the existing
  SourceBadge).
- `bool restore_transient({node: {name: value}}, done_cb)` — same path.
- `void reset_progress(done_cb)` — factor the existing `reset_progress`
  action body into a callable.
- `nlohmann::json live_value(node, name)`.

### 4.3 `DockManager` additions

- `void start_mission_from_dock(const std::string & id, std::function<void(bool ok, std::string reason)> done)`
  — refactor of `resume_tick`'s tail + `resume_after_undock` (undock with
  every existing interlock, then publish `start`) so the resume path and
  the scheduler share one sequencer. Resume keeps its own trigger
  (charged); the scheduler calls it directly.
- `void request_charge_full(const std::string & why)` — the body of the
  `charge_full` command.
- `void set_storage_suppressed(bool)` — folded into `want_full` in
  `topup_tick` (like `resume_pending_` is today).
- `void set_run_policy(bool active)` — while true: `auto_dock_on_mission_complete`
  and `auto_dock_on_low_battery` are treated as true (D8); `on_mission_state`
  reads the policy alongside the params.
- **Existing behaviour change (D8, all missions):** `on_mission_state` arms
  the auto-dock on the `→ idle` edge with reason `complete` **or `operator`**
  when `auto_dock_on_mission_complete` is on (today: `complete` only).
  Update the `topics.yaml` comment and Docking plan 04 §6 at implementation.
- Getters: `physically_docked()`, `dock_fresh()`, `dock_state()`,
  `robot_batt_v()` (+ freshness), `mission_state()`/`mission_reason()`/
  `mission_fresh()`, `estop_active()`, `undock_ready()`,
  `resume_pending()`.
- Status: add `"scheduled_run": "<id>"` while the run policy is active
  (the Dock page and Mission control can say "scheduled run").

### 4.4 `LaunchManager` additions

`std::string unit_state(const std::string & id)` and `bool start(const
std::string & id)` (the existing `handle_command` path without MQTT).

### 4.5 Config, allowlist, files

`topics.yaml`:

```yaml
schedule:
  cmd_topic: ros2/schedule/cmd
  status_topic: ros2/schedule/status
  file: /home/ros-pi/pi_ws/mowing_data/config/schedule.json
  state_file: /home/ros-pi/pi_ws/mowing_data/config/schedule_manager_state.json
  ntp_marker: /run/systemd/timesync/synchronized
  mission_launch_id: mow_mission      # launch_control id to (re)start
  bringup_launch_id: bringup
  start_settle_s: 15                  # start must leave idle within this
  unit_start_wait_s: 60
```

`param_control.nodes.mqtt_bridge_node` gains `schedule_enabled: { type: bool }`.

`robot/fileserver.py` `WRITABLE` gains `"/config/schedule.json"`
(`create: True`, validator `_validate_schedule` = JSON object, `version`
1, `runs` list ≤ 32 with id/day/time shape). `robot/version_watcher.py`
`FILES` gains `config/schedule.json → ros2/schedule/version`. Backups in
`config/backups/` (existing dir, newest 10).

LXC ACL: `user webui` + `topic write ros2/schedule/cmd`; `mowbot-remote.conf`
+ `topic schedule/cmd in 1 ros2/ ha/ros2/` for Phase 4 (bridge rule ⇒
`systemctl restart mosquitto`).

## 5. LXC backend

| Endpoint | Behaviour |
|---|---|
| `GET /api/schedule` | the cached file (fetched like `dock.json`, refetched on `ros2/schedule/version`), header `X-Schedule-Sha1`; 404 `{error:"no schedule yet"}` before the first save (UI shows an empty week) |
| `PUT /api/schedule` | body `{schedule, base_sha1}`; full validation (§3.1) incl. area names against `/api/areas` + coverage presence; `updated_at` stamped in UTC; fileserver PUT with `If-Match`; 409 on conflict ("schedule changed on the robot — reload") |
| `GET /api/schedule/estimates` | `{factor, factor_basis: {missions: n, median_ratio}, overhead_min, mow_min_per_charge, charge_break_min, areas: {etupiha: {est_min: 33.8, calibrated_min: 50.7, segments: 10}, …}}` — factor and endurance per D12; defaults when fewer than 3 usable missions; endurance from `median(active_min / battery_delta_v) × (41.5 − mow_battery_low_voltage)` clamped 60–300 |
| `GET /api/schedule/history?days=7` | optional (Phase 3): actual scheduled runs from `/api/stats` missions whose start lies within 2 min of a schedule slot — for the "last week" ghost blocks |

`config.py`: `SCHEDULE_URL = f"{ROBOT_HTTP}/config/schedule.json"`.
`robot_client.py`: `put_schedule()` = `put_dock()` clone; `fetch_once()`
also fetches the schedule (404 tolerated).

## 6. Web UI

### 6.1 Navigation

`SideNav.jsx` `PAGES`: insert `['schedule', 'Schedule']` after `map`.
`App.jsx`: `{page === 'schedule' && <SchedulePage />}`.

### 6.2 Schedule page (`pages/SchedulePage.jsx`) — layout

```
┌ Schedule ──────────────────────────────────── [Schedule: ON ●]  [+ New run] ┐
│ weekly · robot time Europe/Helsinki · ⏱ next: Wed 14:00 sivupiha1 + sivupiha2 │
├────────┬────────┬────────┬────────┬────────┬────────┬────────┬─────────────┤
│        │  Mon   │  Tue   │  Wed   │  Thu   │  Fri   │  Sat   │  Sun        │
│  6:00  │        │        │        │        │        │        │             │
│  …     │        │        │        │        │        │        │             │
│ 12:00  │        │▓takapiha│       │        │        │▓Full mow│            │
│        │        │12:00–12:50│     │        │        │12:00–20:50           │
│ 13:00  │▓etupiha│        │        │        │        │ ▒charge │            │
│        │13:00–13:57       │        │        │        │ ▓       │            │
│ 14:00  │        │        │▓sivupiha1 +     │        │ ▓       │            │
│        │        │        │ sivupiha2       │        │ ▒charge │            │
│  …     │        │        │14:00–19:35 ▒    │        │ ▓       │            │
│ 21:00  │────────┴────────┴────── quiet hours ───────┴────────┴─────────────│
├──────────────────────┬──────────────────────┬──────────────────────────────┤
│ Up next              │ This week            │ Rules                        │
│ Wed · 14:00          │ 4 runs · ≈ 15.3 h    │ [x] Top up before run: 30 min│
│ sivupiha1+sivupiha2  │ 2 need charge breaks │ [ ] Require full charge      │
│ ≈ 5 h 35 · LiDAR off │ last week: 3 ✓ 1 ⚠   │ [x] Wait for dock ≤ 30 min   │
│                      │                      │ Quiet hours [21:00]–[07:00]  │
│ pre-charge 13:30     │                      │ Late start window [30] min   │
│ ▸ Run now  ▸ Skip    │                      │                              │
└──────────────────────┴──────────────────────┴──────────────────────────────┘
```

- **Week grid** (`components/schedule/WeekGrid.jsx`): CSS grid, 7 columns
  Mon–Sun (no dates), hour rows from `min(6:00, earliest start − 1 h)` to
  `max(21:00, latest end + 1 h)`, 48 px per hour. Run blocks
  (`RunBlock.jsx`) are absolutely positioned inside the day column:
  top/height from start time and the estimate; **charge breaks drawn as
  hatched sub-bands** inside the block; label = area list or "Full mow" +
  "13:00 – 13:57 · ≈ 57 min"; LiDAR-off runs carry a small "no LiDAR"
  chip; disabled runs striped/dimmed; the active run has a pulsing outline
  and its phase text ("mowing · 34 %", "charging break 1"); quiet hours
  shaded; a "now" line on today's column (browser clock — cosmetic only);
  when the master switch is OFF or a hold is active, a dimmed banner
  across the grid: "Scheduled missions are OFF — nothing will start
  automatically" (§2.1).
  Click/tap a block → editor. Click an empty cell → new run pre-filled
  with that day/hour. No drag-and-drop in v1 (touch-first; day/time are
  edited in the form).
- **Phone (< 700 px)**: the grid becomes seven stacked day sections
  (agenda list) with the same blocks as rows; "New run" stays in the
  header.
- **Run editor** (`RunEditor.jsx`, modal on `ConfirmModal`'s styling):
  day chips Mon…Sun; time `<input type="time" step="300">`; **areas** as
  checkboxes listing every mow area that has a coverage entry, each with
  its calibrated estimate, plus "All areas (Full mow)" which clears the
  list; **LiDAR obstacle detection** toggle and **Perimeter LiDAR mode**
  select (`off / outermost_only / all_perimeters`) — defaults for a new
  run come from the live `ros2/mowparams/status` values; enabled toggle;
  label; advanced: max run minutes. A live footer shows
  "≈ 3 h 55 mowing + 1 charge break ≈ 5 h 35 · ends ≈ 19:35" and warns on:
  overlap with another run (using estimates), end after quiet hours,
  estimate > one charge while `resume_after_charge` is off ("the run will
  end after the first charge — enable resume in Settings → Docking"), and
  areas without coverage. Save = `PUT /api/schedule` of the whole document
  with `base_sha1`; Delete with confirm.
- **Up next** (`UpNextCard.jsx`): from `ros2/schedule/status.next`
  (fallback: computed in the browser when the bridge status is absent,
  with a "scheduler offline on the robot" badge); buttons **Run now**
  (`run_now` with press-again confirm — the dock gate still applies) and
  **Skip next**; shows `hold_until` with a **Release** button; shows the
  skip/end reason of `last` in plain English (`REASON_TEXT`-style map,
  e.g. `not_docked` → "the robot was not in the dock at 13:00").
- **This week**: number of enabled runs, Σ estimated hours, how many need
  charge breaks, last week's outcomes from `status.runs[].last_outcome`.
- **Header pill** "Scheduled missions: ON / OFF / ON HOLD until …" = the master switch, tappable (§2.1).
- **Rules** (`ScheduleRules.jsx`): master toggle (`schedule_enabled` via
  `ros2/mowparams/cmd`, press-again confirm when enabling — the
  `DockingSettings` pattern) and the file `options` (saved with the
  document), including **"Wait for the robot to be docked"** (`wait_for_dock`,
  D5) with the late-start window shown next to it. Hint line when `resume_after_charge` is off and any run needs
  a charge break; link to Settings → Docking.
- **Global**: `AlertBanner` gains one dismissable line for
  `status.last.outcome ∈ {skipped, failed, timeout}` newer than the
  dismissed key; `MissionControl` shows "Started by the schedule (Wed
  14:00)" while `docking.scheduled_run` / `status.active` is set; the
  **Stop button reads "Stop & dock"** whenever a stop will be followed by
  docking (scheduled run, or `auto_dock_on_mission_complete` on) and its
  confirm/help text says "ends the mission and returns to the dock — use
  Pause to hold the robot where it is" (D8);
  `RobotStatusBanner` unchanged (mowing is mowing).
- `hooks/useSchedule.js`: fetch + sha1 + PUT + estimates; subscribes
  `ros2/schedule/version` to refetch. `lib/scheduleEstimate.js`: the D12
  duration model (grid, editor and cards all use the same function).

### 6.3 Estimate rendering rules

`duration(run) = Σ calibrated_min(area) + overhead_min`; charge breaks =
`floor((Σ calibrated_min − 1) / mow_min_per_charge)` each adding
`charge_break_min`; block height = the total; hatched bands placed after
every `mow_min_per_charge` of mowing. Shown with "≈" everywhere; the
tooltip lists the components ("mowing 229 min · transit/dock 8 min · 1
charge break ≈ 120 min · factor 1.5 from 5 missions").

## 7. Home Assistant / OLED (Phase 4, optional)

`homeassistant.yaml`: sensors `Next scheduled run` (`status.next` → "Wed
14:00 sivupiha1+sivupiha2"), `Scheduled run phase` (`active.phase`),
`Last scheduled run` (`last.outcome`/`reason`); switch `Mowing schedule`
(`schedule_enabled` via `ha/ros2/mowparams/cmd` — needs a new `in` rule and
the `ha/ros2/` command topic, 2026-07-08 convention); buttons "Run next
now" / "Skip next" / "Hold 24 h" on `ha/ros2/schedule/cmd`. OLED panel: one
line "next: Wed 14:00" in the idle screen (mowbot_oled_interface reads the
retained status) — only if the owner wants it.

## 8. Security / deploy deltas

- ACL (LXC `/etc/mosquitto/acl` + working copy): `webui` `topic write
  ros2/schedule/cmd`; `mosquitto_ctrl`/`systemctl reload mosquitto`.
- Bridge rebuild (`colcon build --symlink-install --packages-select
  mowbot_mqtt_bridge` from `~/pi_ws`, one package per command) + restart
  `mowbot-mqtt-bridge.service`; fileserver + version watcher restarts.
- LXC: rsync `server_lxc/` (exclusions per memory), `npm run build` as
  `mowbot`, restart `mowbot-backend.service`; users hard-refresh.
- The `webui` credential is semi-public: `run_now` obeys every robot-side
  interlock, `cancel` only stops/docks, and the schedule file goes through
  the token-protected fileserver PUT — nothing new becomes possible for a
  LAN guest that the existing `mission/cmd` write does not already allow.

## 9. Implementation phases

| Phase | Deliverable | Touches | Depends on |
|---|---|---|---|
| **0 — Decisions** | Owner answers §11 (defaults proposed) | plan | — |
| **1 — Schedule file + page (no execution)** | fileserver + watcher entries; backend `GET/PUT /api/schedule`, `/api/schedule/estimates`; Schedule page with grid, editor, rules (options only), estimates; SideNav entry. Page shows "scheduler not running on the robot" until `ros2/schedule/status` exists | web UI repo (`robot/`, backend, frontend) | — |
| **2 — Robot scheduler** | `schedule_manager`, `ParamManager`/`DockManager`/`LaunchManager` hooks, `topics.yaml` section + `schedule_enabled` allowlist, ACL; status wired into Up next / active block / AlertBanner / MissionControl; `run_now`, `cancel`, `skip_next`, `hold` | bridge, LXC ACL, frontend | 1 |
| **3 — Charging integration + calibration** | pre-charge (`charge_full` + storage suppression), `require_full_charge`/`start_window`, quiet-hour resume block, run policy in `dock_manager`; backend calibration from stats; last-week ghosts | bridge, backend, frontend | 2, real runs for tuning |
| **4 — HA + OLED** | §7 | bridge `homeassistant.yaml`, LXC bridge rules, OLED | 2 |

Rough size: Phase 1 ≈ 1 day (the grid is the bulk), Phase 2 ≈ 1–1.5 days
(state machine + refactors + bench), Phase 3 ≈ ½ day, Phase 4 ≈ ½ day.
Rollout: 1 → 2 with `schedule_enabled` **off** and `run_now` tests →
enable one short run (etupiha) while watching → Phase 3 → one supervised
low-battery dock + resume with `resume_after_charge` on (Q3) → the Saturday
Full run last (multi-charge).

## 10. Test plan

### 10.1 Bench (robot docked, motors master switch OFF for the first rounds)

1. File round trip: create the four example runs in the UI → `schedule.json`
   on the robot (backup created, `ros2/schedule/version` retained,
   bridge log "schedule loaded: 4 runs"); edit from two browsers → second
   save gets 409 and reloads; invalid file written by hand → status
   `file.error`, previous schedule stays.
2. Clock: status `runs[].next_at` for each run matches a hand calculation
   across a DST boundary (set a run at 03:30 on the last Sunday of October
   → must not fire twice / skip).
3. `run_now` with the robot **off** the dock (switch released) ⇒ `skipped/not_docked`
   and nothing moves. Clock-fired slot with the robot off the dock:
   `wait_for_dock` on ⇒ phase `waiting_dock`, seat the robot at T0+5 min
   ⇒ run starts within seconds; leave it off ⇒ `not_docked` at
   T0+`late_start_min`; `wait_for_dock` off ⇒ `not_docked` at T0. With the dock Pi's zenoh down ⇒ `dock_offline`.
   With `schedule_enabled` false ⇒ `disabled`. Inside quiet hours ⇒
   `quiet_hours`. E-stop pressed ⇒ `estop`. Motors OFF ⇒ `motors_off`.
   `mow_mission` unit stopped ⇒ the manager starts it, waits for idle,
   proceeds (log line).
4. Transient params: `run_now` for the sivupiha run ⇒ `ros2/mowparams/status`
   shows `area_filter = [sivupiha1_coverage, sivupiha2_coverage]`,
   `lidar_enabled=false`, `perimeter_lidar_mode=off` with `source: live`;
   `mowing_overrides.yaml` unchanged; after `cancel` the three values are
   back to the stored ones. Bridge restart mid-run ⇒ restore still happens.
5. Progress reset: mark segments done manually, `run_now` ⇒ `/mowing/state`
   shows zero completed segments.
6. Pre-charge: set a run 70 min ahead with lead 60 ⇒ at T0−60 the dock
   leaves storage hold / pulses from COMPLETE (`ros2/docking/status`
   `charge_full_requested`), phase `precharging`; `require_full_charge`
   with the charger disabled ⇒ `waiting_charge` then `battery_low` after
   the window.
7. Overlap: two runs 10 min apart, first still `mowing` ⇒ second
   `previous_run_active`.
8. Missed: stop the bridge over a slot, start it 40 min later ⇒ `missed`,
   robot stays docked.
9. Switches (§2.1): master OFF ⇒ `next: null`, `run_now` refused
   `disabled`, no pre-charge request at T0−lead; per-run `enabled: false`
   ⇒ that run's `next_at = 0`, others unaffected; `hold 2h` ⇒ header "on
   hold until …", auto-release after 2 h; master OFF during `precharging`
   ⇒ `skipped/disabled` and storage suppression released; master OFF
   during `mowing` ⇒ the run continues and finishes, `cancel` stops it;
   bridge restart with OFF ⇒ stays OFF.

### 10.2 Field (motors ON, owner present)

1. Short run (etupiha, ≈ 57 min) via `run_now`: undock → mow → complete →
   auto-dock → status `completed`, params restored, Dock page shows the
   stay; AlertBanner silent.
2. Same run from the clock (set it 3 min ahead): pre-charge (lead 2 min
   for the test) then start at T0 ± 5 s.
3. Pause mid-run ⇒ robot holds, run stays active, Resume continues.
   Stop mid-run ⇒ mission ends, robot docks, `canceled/operator_stop`,
   params restored. Same Stop on a manual mission with auto-dock on ⇒
   docks too.
4. `cancel` from the UI mid-run ⇒ mission stops, robot docks, outcome
   `canceled`.
5. Multi-charge run (sivupiha1 + sivupiha2) with `resume_after_charge`
   on and a temporarily raised `mow_battery_low_voltage`: low-battery
   auto-dock → `charging` → resume → completion → dock. Then the same
   with quiet hours starting during the charge ⇒ `partial/quiet_hours`.
6. `max_run_min` = 20 on a long run ⇒ `timeout`, robot docks.
7. Leave the schedule enabled for a real week; compare block estimates
   with the `/api/stats` durations; tune `factor`, `mow_min_per_charge`,
   `charge_break_min`, `precharge_lead_min`.

## 11. Open questions for the owner (proposed defaults in bold)

1. ~~Pre-charge lead~~ **DECIDED 2026-09-13:** lead default **30 min**,
   adjustable in the Rules card (`options.precharge_lead_min`); the
   scheduler **never** switches top-up (or storage mode) on by itself —
   Settings → Docking decides. (Measured 45–60 min to full from the parked
   band, §1.3 — tune the lead from the dock statistics after the first
   weeks.)
2. ~~Robot not docked at T0~~ **DECIDED 2026-09-13:** option
   `wait_for_dock` in the Rules card, default **on** = wait within the
   late-start window (`late_start_min`, 30 min) and start the moment the
   robot is seated; **off** = skip immediately with `not_docked`. Never
   starts from the lawn either way (D5).
3. ~~Charge breaks~~ **DECIDED 2026-09-13 (option A):** scheduled runs
   follow Settings → Docking `resume_after_charge` exactly, never forced;
   the editor and the Rules card warn when a run needs a charge break and
   resume is off. Prerequisite before the first multi-charge scheduled
   run: enable resume in Settings and do one supervised low-battery
   dock + resume (resume-after-charge is untested live as of
   2026-09-13).
4. ~~Quiet hours~~ **DECIDED 2026-09-13:** yes — default 21:00–07:00,
   both times adjustable in the Rules card (`quiet_from`/`quiet_until`,
   may wrap midnight, equal = off); applies to start **and** resume, never
   interrupts a running mission.
5. ~~Progress reset~~ **DECIDED 2026-09-13:** every scheduled run starts
   from the beginning — progress is always reset at the scheduled start,
   no per-run "continue" option and no global toggle. Accepted side
   effect: a manually interrupted mission's saved progress is wiped by
   the next scheduled start (charge-break resumes inside a run are
   unaffected — the reset happens only at T0).
6. ~~Operator stop~~ **DECIDED 2026-09-13:** **Stop = end the mission
   and dock**, everywhere (scheduled runs always; manual missions when
   `auto_dock_on_mission_complete` is on); **Pause** = hold in place,
   resumable, no docking. The Stop button is labelled "Stop & dock" and
   says so in its confirm text. Cancel run on the Schedule page is the
   same action.
7. `Full` = all areas with coverage, in the route server's (optimized)
   order — OK, or should the editor let you order areas? (**no ordering**)
8. Overlapping runs are skipped (`previous_run_active`) — or queued to
   start when the previous one finishes? (**skip**)
9. Any weather/rain input? Nothing exists today; the `hold` command is the
   hook (HA automation → `ha/ros2/schedule/cmd {"action":"hold","hours":6}`).
   (**later, Phase 4**)
10. Should the OLED show the next run? (**yes if cheap**)

## 12. Deliberately not in this plan

- Drag-and-drop on the calendar (touch complexity; form editing first).
- Per-run speed / blade RPM / other mowing settings (owner: Settings page
  values apply). The transient-parameter mechanism makes adding them a
  one-line allowlist change later.
- Multiple schedules, exceptions ("skip next Monday" is the `skip_next`
  command), date-specific runs, holidays.
- A weather service. `hold` is the integration point.
- Making the LXC a fallback clock. If the robot bridge is down, nothing
  mows — by design.

## 13. Cross-references

- Docking master plan: [../Docking plan/README.md](../Docking%20plan/README.md);
  `dock_manager` contract 04 §3; automation 04 §6 (auto-dock, resume after
  charge); web UI conventions 05 (§4.5 Settings pattern, §5.3 Phase C
  "Undock & start").
- Web UI repo plans: `plans/implemented/PLAN_MOWING_SETTINGS.md`
  (param_control), `PLAN_AREA_EDITOR.md` (fileserver PUT path),
  `PLAN_UI_NAV_AND_HOME.md` (SideNav/pages).
- Mission lifecycle & gates: `mowing_navigation` BT_REVIEW (S5 lifecycle,
  N1 battery gate, F27 boot-start/auto-reset, F29 lidar idle).
