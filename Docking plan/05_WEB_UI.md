# 05 — Web UI: Dock & Charging

> Status: **PLANNED 2026-09-06 — nothing implemented.** Written against
> `mowbot_dock` `HANDOFF.md` (2026-09-06: dock built, deployed, firmware
> 0.1.4, dock Pi live at 192.168.1.91, three real charge cycles verified)
> and the web UI repo (`mowbot_web_ui`, commit `e6dd578`). This file is the
> **authoritative web-UI spec** for the dock; it supersedes 03 §6, which
> stays as history. Robot-side docking (docking_server, `dock_manager`) is
> specified in [04](04_ROBOT_MODIFICATIONS.md) and only referenced here.
>
> Open decisions for the owner are collected in **§9** — items marked
> **(Q n)** in the text depend on them; everything else is decided.

## 0. Where things stand

| Piece | State | Consequence for the UI |
|---|---|---|
| Dock hardware + Nano fw 0.1.4 | done, charging a real robot | data exists |
| Dock Pi `dock_agent` (13 `/dock/*` topics on zenoh) | done, federates through the robot router | data exists **on ROS 2 only** |
| Dock → LXC MQTT path (second bridge instance, `dock` broker account) | **not built** | the web UI sees nothing dock-related today |
| Robot side (docking_server, `dock_manager`, `dock.json`) | **not started** | no Dock/Undock control, no dock pose |
| Web UI | `robotStatus.js` has "Charging" reserved; nothing else | clean slate |

Two hard facts from the dock side that shape the UI (HANDOFF §3):

1. **State 5 COMPLETE never occurs** with fw 0.1.4. A charge ends
   `CHARGING → DRAIN → IDLE` while the robot stays seated, then the pack is
   topped up in bursts (observed 40 s–3 min CHARGING, ~8 min IDLE gaps).
   "Seated + IDLE" therefore means *resting between top-ups*, not "empty
   dock". Nothing in the UI may depend on state 5 or on
   `power_supply_status == FULL` until TODO.md §6 is decided.
2. `charger_voltage` reads ≈ 0 whenever the dock is cold; `battery_state`
   voltage/percentage are `null` (NaN) below 20 V. "0 V" is the normal idle
   value, never "battery empty".

## 1. Scope and phases

The work splits into four phases with **different prerequisites**. Phase A
needs no robot change at all and delivers most of the monitoring value, so it
goes first.

| Phase | Deliverable | Needs | Touches |
|---|---|---|---|
| **A — Telemetry** | Dock/charger status everywhere it matters: home page card, Status-page cards, alerts, derived "Charging/Docked" robot status, dock Pi health, dock logs | dock bridge instance + LXC account/ACL | `mowbot_dock` repo, LXC, frontend |
| **B — Dock pose** | Dock + staging markers on both maps; "Save dock at robot position"; "Place on map" fallback; `dock.json` on the robot fileserver | robot fileserver/watcher edits (small, in the web UI repo), backend endpoints | web UI repo (`robot/`, backend, frontend) |
| **C — Control** | Dock / Undock / Cancel buttons with live action progress; "Undock & start"; docking reasons in plain English | 04 §2–3 (docking_server + `dock_manager`) | frontend, LXC ACL |
| **D — Polish** | Charge-session statistics, HA entities, auto-dock toggle, dock service restart buttons | A (+ C for auto-dock) | backend, frontend, dock `homeassistant.yaml` |

Phase C UI can be built ahead of the robot side: it lights up when the
retained `ros2/docking/status` topic first appears.

## 2. Data sources

### 2.1 Dock telemetry over MQTT (Phase A; published by the dock bridge instance)

JSON shapes are the bridge's existing serializers (`{"data": …}` for
std_msgs; BatteryState → `{voltage, current, percentage,
power_supply_status, power_supply_health, present}` with `null` for NaN).

| MQTT topic | ROS 2 source | Retained | Rate | Notes |
|---|---|---|---|---|
| `ros2/dock/bridge_status` | bridge LWT | yes | on change | `{"online": true/false}` — the "dock Pi reachable" signal, same pattern as `ros2/bridge/status` |
| `ros2/dock/state` | `dock/state` Int32 | yes | 2 Hz throttle | 0 IDLE, 1 SEATED, 2 RAMP, 3 CHARGING, 4 DRAIN, 5 COMPLETE, 6 SELFTEST, 7 FAULT |
| `ros2/dock/microswitch` | `dock/microswitch` Bool | yes | 2 Hz | robot physically seated — **the** "docked" truth |
| `ros2/dock/relay_k1`, `ros2/dock/relay_k2` | Bool | yes | 2 Hz | 42 V DC relay / mains pilot |
| `ros2/dock/fault` | `dock/fault` Int32 | yes | 2 Hz | 0 none, 1 overcurrent (latching), 2 AC weld (latching), 3 no 42 V (auto-retry), 4 watchdog silence (report-only) |
| `ros2/dock/self_test_result` | Bool | yes | on message | last self-test outcome |
| `ros2/dock/charge_current` | Float32 | no | 2 Hz | A into the pack, ±0.02 A noise floor |
| `ros2/dock/charger_voltage` | Float32 | no | 2 Hz | ≈ 0 when cold |
| `ros2/dock/battery_state` | BatteryState | no | 1 Hz | exactly what the docking server will consume — shown for debugging in the Status page card |
| `ros2/dock/event` | String | no | every message | `EVT:BOOT:<ver>`, `EVT:VER:<ver>` (60 s heartbeat), `EVT:SELFTEST:OK\|FAIL`, `EVT:EMERGENCY:SWITCH`, `EVT:NOCURRENT` |
| `ros2/dock/pi/system` | bridge `system_stats` | no | 5 s | CPU/temp/RAM/disk/WiFi of the **dock Pi** — the same block the robot publishes on `ros2/pi/system` |
| `ros2/dock/logs/{cmd,data,services}` | bridge `log_control` | services retained | on demand | journald for dock units |
| `ros2/dock/launch/{cmd,status}` | bridge `launch_control` | status retained | on change | restart of dock units (Settings → Docking, Phase D) |

Commands (webui → dock, **never retained**): `ros2/dock/charge_enable_cmd`,
`ros2/dock/clear_fault_cmd`, `ros2/dock/self_test_cmd` (all `{"data": bool}`).

**Gap in the dock agent:** the current `charge_enable` permission is never
published — it is a private flag set by `dock/charge_enable_cmd`. The UI
cannot show "charging disabled (maintenance)" honestly without it. Add a
latched `dock/charge_enable` (Bool) publisher to `dock_agent_node`
(publish on change + at start) → `ros2/dock/charge_enable` retained. Ten
lines in the dock repo; listed in §5.1 A1.

### 2.2 Robot-side docking status (Phase C; published by `dock_manager`, 04 §3)

`ros2/docking/status` (retained): `{state, feedback_state, physically_docked,
started_at, retries, error_code, error_msg, id}`, `state ∈ idle | staging |
approaching | waiting_charge | docked | undocking | failed | canceled`.
`ros2/docking/cmd`: `{action: "dock"|"undock"|"cancel", id}`.

### 2.3 Robot battery (already available)

`ros2/hoverboard/battery_voltage`, `ros2/hoverboard/battery_state`
(current). Used to cross-check: charger reports current but the robot's own
pack voltage does not rise → the `EVT:NOCURRENT` / "K1 no-close" case (04
§4.4). The UI only *displays* both side by side; the cross-check logic
belongs to `dock_manager`.

### 2.4 Freshness rules

- **Dock offline** = `ros2/dock/bridge_status.online !== true`. Retained
  state topics keep their last value; the UI shows them greyed with
  "last seen …", never as live.
- The volatile topics (current, voltage, battery_state, pi/system) are
  stale after 10 s without a message (same `useNowTick` grey-out pattern as
  the robot's Pi card, `STALE_MS`). Stale current is shown as "—", not 0.
- Retained dock state is trusted only while the dock bridge is online.

## 3. One derived dock status (`lib/dockStatus.js`)

All pages read one function so the home card, Status page, alerts and the
robot-status banner never disagree. Inputs: dock bridge online, `state`,
`microswitch`, `fault`, `charge_enable`, fresh `current`/`voltage`, docking
action status (Phase C, may be absent). First match wins:

| # | Condition | Headline | Tone | Detail |
|---|---|---|---|---|
| 1 | dock bridge offline | **Dock offline** | idle (grey) | "dock Pi unreachable — charging continues on its own if the robot is seated" + last-seen time |
| 2 | `fault` ∈ {1, 2} | **Dock FAULT** | bad | fault text; "charging blocked until Clear fault" |
| 3 | `fault` == 3 | **Charger problem** | warn | "no 42 V after AC on — retrying every 60 s (mains? charger?)" |
| 4 | `fault` == 4 | (no headline change; badge only) | warn | "dock Pi ↔ Nano link silent" — report-only, don't scare |
| 5 | `state` == 6 | **Self-test…** | warn | — |
| 6 | docking action in `staging/approaching/waiting_charge` | **Docking…** | ok | feedback phase, retries, elapsed |
| 7 | docking action `undocking` | **Undocking…** | ok | — |
| 8 | `state` ∈ {2, 3} | **Charging** | ok | `2.1 A · 41.2 V · ~85 %` (see §3.1 for the %) |
| 9 | `state` == 4 | **Finishing charge** | ok | "draining, relay opens at 0 A" |
| 10 | `state` == 5 | **Charged** | ok | kept for the day COMPLETE exists |
| 11 | `state` == 1 | **Seated** | warn | "waiting for charger…"; after 15 s seated with no charging: "charge did not start — check Charging allowed / fault" |
| 12 | `state` == 0 && `microswitch` | **Docked · resting** | ok | "battery topped up, charger cold; next top-up starts automatically" + `charge_enable` false → "charging DISABLED" (warn) |
| 13 | `state` == 0 && !`microswitch` | **Empty** | idle | "dock cold, ready" |
| 14 | anything else / no data | **—** | idle | — |

### 3.1 Percentage and voltage display

- While the charger is live (`voltage ≥ 20`): show charger voltage and the
  dock's linear estimate (33 V → 0 %, 42 V → 100 %), labelled "under charge
  (inflated by charge current)". This number is *not* the robot's SoC.
- While cold: show the **robot's** `hoverboard/battery_voltage` + existing
  `batteryPercent()` (when the robot bridge is online), labelled "robot
  battery". Never show the dock's 0 V as the pack voltage.

### 3.2 Charge session bookkeeping (client side, Phase A; backend in D)

The browser keeps a small in-memory record per page session: time of last
`state == 3`, and a running sum of `current × Δt` while charging (2 Hz
samples). Used for the "last charged N min ago · ~0.4 Ah this session" line
on the Docked-resting row. It resets on reload and is honest about it
(shown only after at least one charging sample was seen). Persistent,
authoritative session stats come from the backend in Phase D (§4.8).

## 4. What is shown where

### 4.1 Mowbot (home) page — `DockCard`

Placed directly under `MissionControl` (the dock is where a mission starts
and ends; the cockpit should show it). Compact:

```
🔌 Dock — Charging                              [dock Pi ● online]
    2.1 A · 41.2 V · ~85 % under charge · 12 min
    [ Dock ] [ Undock ] [ Cancel ]        ← Phase C, hidden until ros2/docking/status exists
```

- Headline + tone from §3, one metrics line, small "dock Pi" dot from the
  LWT.
- Phase C: buttons, guarded exactly like `MissionControl` (robot bridge
  online, mission process not running or mission idle, not e-stopped;
  Dock/Undock double-confirm; Cancel immediate). Disabled reasons shown as
  tooltips/hints, refusals from `ros2/docking/status.error_code` mapped via
  a `DOCK_REASON_TEXT` table like `GoToPanel`'s `REASON_TEXT`.
- Phase C: when `physically_docked` and mission is idle, the mission
  **Start** button in `MissionControl` shows "Undock & start" (04 §6): UI
  sends `undock`, waits for docking status `idle` + `microswitch false`
  (30 s watchdog), then `start`. A mission refusal reason `docked` gets a
  `REASON_TEXT` entry: "robot is in the dock — Undock first".
- No maintenance controls here (they live on the Status page).

### 4.2 Status page — two cards

**Charger** card (in the cards grid, after Battery):

- value = headline from §3, tone likewise.
- detail lines: `state <name> · switch <seated/free> · K1 <on/off> · K2
  <on/off>`; `current · charger voltage`; fault text when non-zero;
  `self-test: OK/FAIL <when>`; `fw <version from EVT:BOOT/VER> · last event
  <EVT…> <ago>`.
- `<details>` **Maintenance** (collapsed; every button double-confirm,
  disabled when dock offline):
  - **Charging allowed** on/off → `charge_enable_cmd`. Off while charging
    runs the sequenced shutdown (AC off → drain → K1 open) — say so in the
    confirm text. State shown from `ros2/dock/charge_enable` (§2.1 gap).
    Turning it **off→on while docked-resting** is also the manual
    "top up now" action (02 §4 re-entry rule); label the on-button
    "Enable (starts a charge if seated)".
  - **Clear fault** → `clear_fault_cmd`; enabled only for fault 1/2; hint
    "accepted by the firmware only at 0 A".
  - **Self-test** → `self_test_cmd`; enabled only in state 0/5; result badge
    from `self_test_result` with timestamp; hint "≤ 2 s mains pulse with K1
    open; only when nothing is seated".
  - `<details>` **raw battery_state** (what the docking server sees):
    voltage/current/percentage/status/present, `null` shown as "n/a (cold)".
  - Note in the UI: these three commands are implemented but were **never
    exercised against the real Nano** (HANDOFF §6) — the first use is a test.

**Dock Pi** card (next to the robot's "Raspberry Pi" card, same renderer
`piSystem()` reused on `ros2/dock/pi/system`): CPU, temp, RAM %, disk, WiFi %
and dBm. Justified by the two dock-Pi incidents in HANDOFF §5 (WiFi power-save
drop-outs, RAM exhaustion): RAM and WiFi signal are exactly what to watch on
a Pi 3B with 900 MB and no wired link. Tone: warn when RAM > 85 % or WiFi <
30 %.

### 4.3 Global

- **AlertBanner**: `Dock FAULT — <text>` (bad) for fault 1/2; `Dock: charger
  not coming up (fault 3)` (warn); `Robot docked but charging is disabled`
  (warn) when microswitch && !charge_enable. Dock-offline is *not* an alert
  (the dock Pi being off is a normal state for a mains-powered station on a
  timer); it is visible on the cards.
- **`robotStatus.js`** (the banner on every page): fill the reserved slot.
  After the mission checks and before `Idle`:
  - docking action active → `Docking…` / `Undocking…` (ok) with the phase;
  - dock online && microswitch && state ∈ {2,3,4} → **Charging** (ok,
    detail current/voltage);
  - dock online && microswitch && state ∈ {0,5} → **Docked** (idle, detail
    "resting" / "charged");
  - dock offline → no claim (falls through to Idle). The robot's own battery
    voltage *rising* is deliberately not used as a charging heuristic.
- **TopBar**: a ⚡ next to the battery chip while §3 says Charging (Q 3).

### 4.4 Map tab — `DockPanel` + markers (Phase B)

- **Markers** on `RobotMap` and `MiniMap`: dock marker at the stored pose
  (🔌 divIcon rotated to yaw, like the robot marker) and a small dashed
  circle at the computed staging pose. Staging = stored pose shifted
  `staging_offset_m` **against the approach direction**: `pose − offset·yaw`
  for forward docking (04 §2.1 `dock_backwards: false`, README D10) but
  `pose + offset·yaw` if the robot docks backwards as `mowbot_dock`
  HANDOFF §4 states — **the two documents disagree (Q 11)**; the UI reads
  the direction from a `dock_backwards` field in `dock.json` so the marker
  and the robot side can never diverge. `LayersPanel` gains a "Dock"
  visibility toggle (persisted like the others).
- **DockPanel** ("🔌 Dock position"), mutually exclusive with the editor /
  recorder / coverage / goto panels (add to the `RobotMap` exclusivity
  conditions):
  - Current stored pose: x/y/yaw, `saved_at`, `method`; "not set" state.
  - **Save dock at robot position** (primary, two-step confirm). Enabled when
    robot bridge online and `ros2/odometry/global` is ≤ 3 s old. If the dock
    bridge is online and the microswitch is **not** pressed, the confirm
    step warns "robot is not seated — save anyway?" (the whole point of D4
    is recording while parked). Shows the backend's RTK sanity note from the
    response.
  - **Place on map** (fallback): click 1 = dock position, click 2 = the
    point the robot approaches *from*; yaw = bearing from click 2 to click 1.
    Two-step confirm, method `map_click`. Marked "rough — re-record by
    parking before first autonomous dock".
  - **Staging distance** number input (0.5–1.5 m, default 0.7) stored in
    `dock.json`; the staging marker follows live.
  - Keep-out hint: if the stored pose lies inside a mow-area outline, show
    "the dock sits inside area X — add a hole around it in the area editor"
    (04 §5).
- No "delete dock" button in v1: re-recording is the correction path; a
  `Clear` action can be added later if needed (Q 5).

### 4.5 Settings page — "Docking" panel (Phases C/D)

- **Auto-dock on low battery** toggle (bridge param
  `auto_dock_on_low_battery`, 04 §6) — Phase D, rendered only when the
  param appears in `ros2/mowparams/status` (same pattern as the gate
  toggles).
- **Dock Pi services**: Restart dock agent / Restart dock zenoh router →
  `ros2/dock/launch/cmd {id, action:"restart"}`, with live unit state from
  `ros2/dock/launch/status`. Mirrors `SystemPanel`'s zenoh-restart button.
  Restarting the agent mid-charge DTR-resets the Nano and drops the relays
  for a few seconds (HANDOFF §5) — put that in the confirm text.

### 4.6 Logs tab

Dock units via the dock bridge's `log_control`: `dock_agent`
(`mowbot-dock-agent.service`), `dock_bridge`
(`mowbot-dock-mqtt-bridge.service`), `dock_zenoh`
(`zenoh-dock-router.service`), `dock_wifi_watchdog`
(`wifi-watchdog.service`). `LogsPage` currently reads one
`ros2/logs/services` list and one `cmd/data` pair; generalise it to a list
of sources `[{prefix:'ros2/logs', group:'Robot'}, {prefix:'ros2/dock/logs',
group:'Dock'}]`, request routed by the service's source, list rendered in
two groups. Service ids are namespaced on the dock side so the two lists
never collide.

### 4.7 Launch tab

Nothing. Dock units are always-on; restart buttons live in Settings → Docking.

### 4.8 Statistics — charge sessions (Phase D)

Backend `stats.py` already consumes the broker; extend it (or add
`dock_stats.py`) subscribing `ros2/dock/#`:

- A **charge burst** = `state` enters {2,3} → leaves to {0,4,5}. Ah =
  Σ current·Δt (2 Hz samples), peak A, end charger voltage.
- A **docked period** = microswitch true … false; contains N bursts. This
  is the unit shown to the user (bursts merged), because of the top-up
  cycling: "Docked 2 h 14 min · 3 bursts · 1.8 Ah · ended 41.9 V".
- Store last 50 periods in the stats JSON; expose `GET /api/dock/sessions`;
  a small table under `Statistics` (last 10) plus totals (Ah, hours docked).
- Phase D because it only becomes meaningful once docking is routine (Q 6).

### 4.9 Home Assistant (Phase D, telemetry-only)

Dock bridge instance gets its own `homeassistant.yaml`: `binary_sensor`
charging (state ∈ {2,3}), `binary_sensor` robot_seated, `sensor` charge
current (A), charger voltage (V), dock state (enum text), dock fault (text),
dock Pi WiFi/RAM. The LXC→HA bridge already exports `ros2/#` so **no LXC
bridge-rule change** for telemetry. HA-side commands (charge enable) would
need the `ha/ros2/` inbound rule convention — deliberately not in this plan
(Q 7).

## 5. Implementation

### 5.1 Phase A — dock telemetry

**A1 — dock repo (`mowbot_dock`): second `mowbot_mqtt_bridge` instance.**

The bridge is its own repo (`git@github.com:lepptu/mowbot_mqtt_bridge.git`);
`mowing_msgs` too (`lepptu/mowing_msgs`). Both go into
`mowbot_dock_ws/src/` as **git submodules** (one source of truth, pinned
commit). `ublox_ubx_msgs` is a bridge dependency (robot GNSS serializers)
that the dock never uses — on the robot it comes from apt
(`ros-jazzy-ublox-ubx-msgs`, installed under `/opt/ros/jazzy`), so add that
package to the Dockerfile and the dock Pi, or (cleaner) add a CMake option
`MOWBOT_BRIDGE_MINIMAL=ON` to the bridge that compiles out the
GNSS/goto/manual-mow/power managers and their message deps. Start with the
submodule route (zero bridge changes); switch to the CMake option if the
qemu cross-build time hurts.

- `mowbot_dock_builder/Dockerfile`: add `libpaho-mqttpp-dev
  libpaho-mqtt-dev libyaml-cpp-dev nlohmann-json3-dev
  ros-jazzy-nav2-msgs ros-jazzy-ublox-ubx-msgs`. Runtime on the dock Pi
  (`PI_SETUP.md` §3): `libpaho-mqttpp3-1 libpaho-mqtt1.3 libyaml-cpp0.8
  ros-jazzy-nav2-msgs ros-jazzy-ublox-ubx-msgs`.
- Memory: the bridge is ~65 MB RSS on the robot. Fine on the Pi 3B with
  the zram swap in place, but it is the second-largest process there — check
  `free -m` after a day (§6.1).
- `deploy/mowbot-dock-mqtt-bridge.service`: copy of the robot unit;
  `Environment=RMW_IMPLEMENTATION=rmw_zenoh_cpp`, `After=zenoh-dock-router.service
  network-online.target`, `Restart=on-failure`, config path
  `~/mowbot_dock/config/topics.yaml`, `secrets.yaml` next to it (chmod 600,
  gitignored — `deploy.sh` must not overwrite it). `deploy.sh` restarts both
  units.
- `dock_agent_node`: add latched `dock/charge_enable` publisher (§2.1 gap).
  Also publish the firmware version once parsed from `EVT:BOOT/VER` on a
  latched `dock/firmware_version` String — cheaper for the UI than parsing
  the event stream (optional).

**A2 — dock `topics.yaml`** (new file in the dock repo, `deploy/config/`):

```yaml
broker: { host: 192.168.1.133, port: 1883, credentials_file: secrets.yaml }   # account "dock"
last_will: { topic: ros2/dock/bridge_status, payload: '{"online": false}', retain: true, qos: 1 }
stats: { topic: ros2/dock/bridge_stats, publish_rate_hz: 0.2 }
system_stats: { topic: ros2/dock/pi/system, period_s: 5, wifi_iface: wlan0 }
topics:
  # retained, latched on the ROS side; throttled because the agent republishes at 10 Hz
  - { ros2_topic: dock/state,            mqtt_topic: ros2/dock/state,            type: std_msgs/Int32,   direction: publish, retain: true,  mqtt_qos: 1, publish_rate_hz: 2, ros_qos: { reliability: reliable, durability: transient_local, depth: 1 } }
  - { ros2_topic: dock/microswitch,      mqtt_topic: ros2/dock/microswitch,      type: std_msgs/Bool,    direction: publish, retain: true,  mqtt_qos: 1, publish_rate_hz: 2, ros_qos: { reliability: reliable, durability: transient_local, depth: 1 } }
  - { ros2_topic: dock/relay_k1,         mqtt_topic: ros2/dock/relay_k1,         type: std_msgs/Bool,    direction: publish, retain: true,  mqtt_qos: 1, publish_rate_hz: 2, ros_qos: { reliability: reliable, durability: transient_local, depth: 1 } }
  - { ros2_topic: dock/relay_k2,         mqtt_topic: ros2/dock/relay_k2,         type: std_msgs/Bool,    direction: publish, retain: true,  mqtt_qos: 1, publish_rate_hz: 2, ros_qos: { reliability: reliable, durability: transient_local, depth: 1 } }
  - { ros2_topic: dock/fault,            mqtt_topic: ros2/dock/fault,            type: std_msgs/Int32,   direction: publish, retain: true,  mqtt_qos: 1, publish_rate_hz: 2, ros_qos: { reliability: reliable, durability: transient_local, depth: 1 } }
  - { ros2_topic: dock/self_test_result, mqtt_topic: ros2/dock/self_test_result, type: std_msgs/Bool,    direction: publish, retain: true,  mqtt_qos: 1,                     ros_qos: { reliability: reliable, durability: transient_local, depth: 1 } }
  - { ros2_topic: dock/charge_enable,    mqtt_topic: ros2/dock/charge_enable,    type: std_msgs/Bool,    direction: publish, retain: true,  mqtt_qos: 1,                     ros_qos: { reliability: reliable, durability: transient_local, depth: 1 } }
  # volatile streams
  - { ros2_topic: dock/charge_current,   mqtt_topic: ros2/dock/charge_current,   type: std_msgs/Float32, direction: publish, retain: false, mqtt_qos: 0, publish_rate_hz: 2 }
  - { ros2_topic: dock/charger_voltage,  mqtt_topic: ros2/dock/charger_voltage,  type: std_msgs/Float32, direction: publish, retain: false, mqtt_qos: 0, publish_rate_hz: 2 }
  - { ros2_topic: dock/battery_state,    mqtt_topic: ros2/dock/battery_state,    type: sensor_msgs/BatteryState, direction: publish, retain: false, mqtt_qos: 0, publish_rate_hz: 1 }
  - { ros2_topic: dock/event,            mqtt_topic: ros2/dock/event,            type: std_msgs/String,  direction: publish, retain: false, mqtt_qos: 1 }
  # commands — never retained
  - { mqtt_topic: ros2/dock/charge_enable_cmd, ros2_topic: dock/charge_enable_cmd, type: std_msgs/Bool, direction: subscribe }
  - { mqtt_topic: ros2/dock/clear_fault_cmd,   ros2_topic: dock/clear_fault_cmd,   type: std_msgs/Bool, direction: subscribe }
  - { mqtt_topic: ros2/dock/self_test_cmd,     ros2_topic: dock/self_test_cmd,     type: std_msgs/Bool, direction: subscribe }
log_control:
  cmd_topic: ros2/dock/logs/cmd
  data_topic: ros2/dock/logs/data
  services_topic: ros2/dock/logs/services
  services:
    dock_agent:         { unit: mowbot-dock-agent.service,       label: "Dock agent (Nano ↔ ROS 2)" }
    dock_bridge:        { unit: mowbot-dock-mqtt-bridge.service, label: "Dock MQTT bridge" }
    dock_zenoh:         { unit: zenoh-dock-router.service,       label: "Dock zenoh router" }
    dock_wifi_watchdog: { unit: wifi-watchdog.service,           label: "Dock WiFi watchdog" }
launch_control:
  cmd_topic: ros2/dock/launch/cmd
  status_topic: ros2/dock/launch/status
  launches:
    dock_agent: { unit: mowbot-dock-agent.service }
    dock_zenoh: { unit: zenoh-dock-router.service }
```

No `param_control`, `goto`, `power`, `manual_mow`, `homeassistant_discovery`
sections (HA comes in Phase D). Verify at first run that the bridge accepts a
config without those sections (they are optional in `bridge_config.cpp`).
Check the exact serializer type strings the bridge expects for Int32 /
Float32 / BatteryState in `serializers.cpp` before writing the file.

**A3 — LXC (user-run, exact commands handed over at implementation).**

- `mosquitto_passwd -b /etc/mosquitto/passwd dock <pw>`; password into
  `/root/mowbot-mqtt-credentials.txt` and the dock `secrets.yaml`.
- ACL targeted append (working copy `server_lxc/mosquitto/acl` updated too):

  ```
  user dock
  topic write ros2/dock/#
  topic read ros2/dock/charge_enable_cmd
  topic read ros2/dock/clear_fault_cmd
  topic read ros2/dock/self_test_cmd
  topic read ros2/dock/logs/cmd
  topic read ros2/dock/launch/cmd
  # under "user webui":
  topic write ros2/dock/charge_enable_cmd
  topic write ros2/dock/clear_fault_cmd
  topic write ros2/dock/self_test_cmd
  topic write ros2/dock/logs/cmd
  topic write ros2/dock/launch/cmd
  ```

- `systemctl reload mosquitto` (ACL/passwd only — no bridge-rule change, so
  no restart). `ros2/# out` already forwards dock telemetry to HA.

**A4 — frontend.**

| File | Change |
|---|---|
| `lib/dockStatus.js` (new) | §3 function + `DOCK_STATE_NAME`, `DOCK_FAULT_TEXT` tables |
| `hooks/useDock.js` (new) | one hook subscribing every `ros2/dock/*` topic, applying §2.4 freshness, returning `{online, state, seated, fault, chargeEnable, current, voltage, battery, event, fwVersion, status}`; the §3.2 client-side session accumulator lives here |
| `components/DockCard.jsx` (new) | §4.1 |
| `pages/MowbotPage.jsx` | mount `DockCard` under `MissionControl` |
| `pages/StatusPage.jsx` | Charger + Dock Pi cards (§4.2); `DockMaintenance.jsx` for the `<details>` block |
| `components/AlertBanner.jsx` | §4.3 dock alerts |
| `lib/robotStatus.js` + `RobotStatusBanner.jsx` | Charging / Docked states, new `dock` input |
| `pages/LogsPage.jsx` | multi-source (§4.6) |
| `lib/telemetry.js` | `piSystem()` reuse; `dockPercent()` for §3.1 |
| `styles.css` | `.dock-card`, badge styles |

Deploy: existing LXC rsync + `npm run build` recipe (user-run).

### 5.2 Phase B — dock pose

**B1 — robot fileserver (`robot/fileserver.py`, web UI repo).** Today PUT is
hard-coded to `/mow_area/mow_areas.json` (`WRITABLE_PATH`). Generalise to a
`WRITABLE` map `{path: backup_dir}` with `/config/dock.json →
config/backups/`, same token + `If-Match` sha1 + newest-10 backups. A missing
`dock.json` must be creatable (first save): accept `If-Match: *` or the
literal sha1 of "no file". Restart `mowbot-fileserver.service`.

**B2 — version watcher (`robot/version_watcher.py`).** Add
`config/dock.json → ros2/dock/pose_version` to `FILES`. Restart unit.
(Lesson recorded in memory: `topics.yaml` has consumers beyond the bridge —
this one reads broker creds from it; unchanged here.)

**B3 — backend (`server_lxc/backend`).**

- `robot_client.py`: fetch + cache `config/dock.json` alongside areas;
  subscribe `ros2/dock/pose_version` (refetch trigger) **and**
  `ros2/odometry/global` (keep last sample + receive time — the backend
  does *not* cache odometry today, contrary to 03 §6.2; only battery and
  datum). Add `("ros2/dock/pose_version", 1), ("ros2/odometry/global", 0)`
  to the subscription list.
- `GET /api/dock` → `dock.json` (+ `X-Dock-Sha1`), 404 when unset.
- `PUT /api/dock` body `{pose:{x,y,yaw}, staging_offset_m, method}`; validate
  (finite, |x|,|y| ≤ 1000, yaw normalised, offset 0.3–2.0); write via
  fileserver with `If-Match`.
- `POST /api/dock/record` body `{staging_offset_m?, force?}`: refuse 409 if
  the odometry sample is missing or > 3 s old; write pose from it with
  `method: robot_pose`, `saved_at` UTC. Response echoes the pose and a
  `warnings` list: `"dock reports robot not seated"` (when
  `ros2/dock/microswitch` is retained false and the dock bridge is online)
  and an RTK note if `ros2/gnss/pvt.carr_soln != 2` was seen recently.
  `force` only affects nothing server-side (the warnings are advisory; the
  UI's confirm step is the gate) — keep the endpoint simple.
- `dock.json` schema:

  ```json
  { "pose": { "x": 12.34, "y": -5.67, "yaw": 1.571, "frame": "map" },
    "staging_offset_m": 0.7,
    "dock_backwards": false,
    "saved_at": "2026-09-06T10:15:00Z",
    "method": "robot_pose" | "map_click" }
  ```

**B4 — frontend.** `components/map/DockPanel.jsx` (§4.4),
`lib/dockMarker.js` (divIcon), marker + staging circle in `RobotMap.jsx`
and `MiniMap.jsx` (fed from `/api/dock`, re-fetched on
`ros2/dock/pose_version`), `LayersPanel` toggle, exclusivity wiring in
`RobotMap.jsx`. Coordinates via the existing `latLonToXy`/`xyToLatLon`
against `config.datum`.

### 5.3 Phase C — control (after 04 §2–3 exist)

- LXC ACL: `user webui` + `topic write ros2/docking/cmd` (04 §7).
- `DockCard` buttons + `DOCK_REASON_TEXT` (mirror `dock_manager`'s reason
  codes: `estop`, `mission_active`, `mission_state_unknown`, `no_dock_pose`,
  `nav2_unavailable`, `nav2_timeout`, `failed_to_charge`,
  `failed_to_detect_dock`, `cancel_timeout`, `not_docked` (undock while not
  seated), `timeout`).
- Status merge rule: while an action is in flight the action status leads
  (§3 rows 6–7); when it ends, hardware state leads again. A `failed`
  result with `physically_docked: true` shows "docked (action reported
  failure — WiFi at contact?)" per 04 §4.4.
- "Undock & start" sequencing in `MissionControl` (§4.1).
- Map: while `staging/approaching`, draw the staging → dock approach line on
  the map (dashed) so the operator sees what the robot is trying to do.

### 5.4 Phase D — polish

Charge-session stats (§4.8), HA discovery file (§4.9), Settings → Docking
panel (§4.5), `auto_dock_on_low_battery` toggle once the bridge exposes it.

## 6. Test checklists

### 6.1 Phase A

- [ ] Dock Pi: `mowbot-dock-mqtt-bridge` up, `ros2/dock/bridge_status`
      retained `online:true` on the LXC (`mosquitto_sub -v -t 'ros2/dock/#'`).
- [ ] Robot **off**: dock card live on the home page and Status page; state 0
      / Empty; Dock Pi card shows WiFi + RAM.
- [ ] `free -m` on the dock Pi after 24 h with both services: no swap
      thrash (HANDOFF §5 history).
- [ ] Seat the robot manually: card walks Seated → Charging within ~1 s;
      current/voltage update at 2 Hz; robot-status banner says Charging.
- [ ] Wait out a full charge: DRAIN → Docked · resting; "last charged … ago"
      appears; a top-up burst later flips it back to Charging.
- [ ] Charging allowed → off mid-charge: sequenced shutdown visible (K2 off,
      DRAIN, K1 off, Empty/resting), alert "charging is disabled" while
      seated; → on: charge restarts. (First real-Nano exercise of this path.)
- [ ] Self-test from Empty: result badge + timestamp; try it during charging:
      firmware ignores it, UI button was disabled anyway.
- [ ] Unplug the dock Pi's power: LWT flips within the keepalive, cards grey
      with "last seen", no alert; power back: recovers without a reload.
- [ ] Logs tab: dock group lists four services; fetch works; robot group
      unaffected.
- [ ] Phone width: DockCard and the two Status cards stay readable.

### 6.2 Phase B

- [ ] `POST /api/dock/record` with the robot parked and seated: `dock.json`
      written, backup created, `ros2/dock/pose_version` published, markers
      appear on both maps without reload.
- [ ] Record with odometry stale (bringup stopped): 409 with a clear message
      in the panel.
- [ ] Place-on-map two-click flow produces a sensible yaw (staging circle on
      the approach side).
- [ ] Staging distance change moves the circle; value survives reload.

### 6.3 Phase C

- [ ] Buttons disabled with the right hint for: robot offline, mission
      running, e-stop, no dock pose, dock offline (Dock still allowed —
      docking needs zenoh, not MQTT — but warn).
- [ ] Full Dock → Docking… phases → Docked; Cancel mid-approach; Undock →
      dock goes cold before the robot moves (watch K1/K2 in the Charger card).
- [ ] "Undock & start" runs the sequence and refuses if undock fails.

## 7. Deploy and ownership notes

- Dock side builds only on the x86 workstation (`dock-build.sh` →
  `deploy.sh`), never on the Pi; never open VS Code Remote-SSH on the dock
  Pi (HANDOFF §5).
- LXC steps (passwd, ACL append, mosquitto reload, frontend rsync + build,
  backend restart) are user-run; hand over exact commands.
- Robot-side Phase B edits live in the web UI repo `robot/` and are
  installed on the robot Pi as today (fileserver + watcher restarts only —
  no bridge rebuild in Phases A/B).
- Never retain any `*_cmd` / `docking/cmd` message.

## 8. Deliberately not in the web UI

- Live relay toggling (K1/K2 by hand): the firmware owns the relays
  (D7); the only user-level knob is "Charging allowed".
- Tuning firmware thresholds (`COMPLETE_A`, `OVERCURRENT_A`, …): constexpr
  in firmware by design (02 §4).
- Editing the dock pose numerically: recording by parking is the
  correction path; map-click covers the rough case.
- Dock Pi power off/reboot: mains-powered station; nothing to gain, real
  risk of stranding a charge session (mirrors the "no Pi power controls in
  HA" decision).

## 9. Open questions for the owner

| # | Question | Plan's default if unanswered |
|---|---|---|
| Q 1 | Home page: is a **dock card under Mission control** the right spot, or would you rather keep the home page as-is and put everything on the Status page? | Home card (§4.1) + Status detail |
| Q 2 | Maintenance controls (Charging allowed / Clear fault / Self-test) — in the web UI at all, given they are untested on the real Nano? Or keep them CLI-only until exercised once? | In the UI, collapsed, double-confirm, with the "untested" note |
| Q 3 | TopBar ⚡ on the battery chip while charging — want it? | Yes, tiny |
| Q 4 | Robot-status banner: "Charging" / "Docked" as first-class states (they replace "Idle" whenever the robot sits in the dock)? | Yes |
| Q 5 | Dock pose: any need to **clear** the stored dock pose, or is re-recording always enough? | No clear button |
| Q 6 | Charge-session statistics (Ah, docked hours, burst count) — wanted, and in Phase D or earlier? | Phase D |
| Q 7 | Home Assistant dock entities — telemetry-only, none, or also commands? | Telemetry-only, Phase D |
| Q 8 | Dock Pi service restart buttons in Settings — useful or noise? (Agent restart drops relays for seconds.) | Include, with the warning |
| Q 9 | The COMPLETE decision (dock TODO §6): fix it in **firmware** (true state 5) or **remap in the agent** (IDLE+seated → FULL)? The UI copes either way, but "Charged" vs "Docked · resting" wording depends on it. | UI handles both; recommend the firmware route so the docking server gets honest `FULL` |
| Q 10 | Should the dock's `battery_state` raw view (§4.2) exist, or is that only debugging clutter? | Keep, inside `<details>` |
| Q 11 | **Forward or backward docking?** Plan 04 / README D10 say forward (contacts at the front, `dock_backwards: false`); `mowbot_dock` HANDOFF §4 says the robot docks backwards with contacts on the rear. Which is the built robot? | `dock.json` carries `dock_backwards`; marker math follows it; 04 needs the same fix |
