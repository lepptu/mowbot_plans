# 05 — Web UI: Dock & Charging

> Status: **Phase A IMPLEMENTED 2026-09-06** (web UI + LXC live; dock-side
> build/deploy owed on the workstation — see the Phase A status block
> below). Phases A2/B/C/D still planned. Originally written against
> `mowbot_dock` `HANDOFF.md` (2026-09-06: dock built, deployed, firmware
> 0.1.4, dock Pi live at 192.168.1.91, three real charge cycles verified)
> and the web UI repo (`mowbot_web_ui`, commit `e6dd578`). This file is the
> **authoritative web-UI spec** for the dock; it supersedes 03 §6, which
> stays as history. Robot-side docking (docking_server, `dock_manager`) is
> specified in [04](04_ROBOT_MODIFICATIONS.md) and only referenced here.
>
> Open decisions for the owner are collected in **§9** — items marked
> **(Q n)** in the text depend on them; everything else is decided.
> **Owner decisions 2026-09-06:** no dock card on the home page; the dock
> gets **its own page** ("Dock" in the side nav); maintenance controls are
> in (tested OK against the real Nano); top-bar ⚡ while charging is in;
> robot-status banner shows Charging / Docked; charge statistics on the
> Dock page; HA gets telemetry **and** commands; dock Pi restart, reboot and
> shutdown from the Dock page; backward docking. **Q 9 (COMPLETE state)
> resolved 2026-09-07: firmware** — fw 0.2.0 reaches state 5 (§0 fact 1,
> §9); no open items.

## Phase A status (2026-09-06)

Done and live:
- **Web UI** (`mowbot_web_ui` commit `16eca56`+): Dock page in the side nav
  (`pages/DockPage.jsx`, `components/dock/*`, `hooks/useDock.js`,
  `lib/dockStatus.js`), Status-page Charger card, alert strip, top-bar
  ⚡/🔌 chip, banner Charging/Docked, Logs tab Robot/Dock groups,
  `PowerPanel` shared by robot + dock. Built and deployed on the LXC.
- **LXC broker:** `dock` account (password in
  `/root/mowbot-mqtt-credentials.txt`), ACL (`user dock` block + webui dock
  command topics), three `ha/ros2/dock/*` inbound bridge rules, mosquitto
  restarted; robot bridge reconnected fine.
- **Dock repo** (`mowbot_dock` commit `9447338`): bridge + `mowing_msgs`
  submodules, `deploy/config/{topics,homeassistant}.yaml`,
  `secrets.yaml.example`, `deploy/mowbot-dock-mqtt-bridge.service`,
  Dockerfile build deps, `deploy.sh` restarts both units, PI_SETUP §9;
  `dock_agent_node` publishes latched `dock/charge_enable` +
  `dock/firmware_version` (compile-checked on the robot Pi).

**Dock bridge deployed and working 2026-09-06** (owner ran the workstation
build + PI_SETUP §9). First-start bug, found and fixed by the owner: the
bridge hard-coded its MQTT client id, so the robot and dock instances kicked
each other off the broker session in a loop. Fix: bridge commit `bc9941a`
makes `client_id` a ROS parameter (default = old value, robot unit
untouched); the dock unit passes `-p client_id:=mowbot_dock_bridge`
(`mowbot_dock` `5f9c1f0`). **Any further bridge instance must set a unique
client id.** Second bridge bug, found the same evening: latched ROS topics
reach the bridge before its MQTT session is up, so on-change topics
(`ros2/dock/charge_enable`, `ros2/dock/firmware_version` — and on the robot
`hoverboard/connected`, `estopStatus`, … after every bridge restart) never
appeared on the broker. Fixed in the bridge (retained-topic cache replayed
after each MQTT connect; verified on the robot: "replayed 14 retained
topic(s)"); the dock picks it up on its next `dock-build.sh` + `deploy.sh`
(submodule pointer bumped). Dock rebuilt and redeployed with the fix the same
evening: "replayed 7 retained topic(s)", `ros2/dock/charge_enable` and
`ros2/dock/firmware_version` (0.1.4) now retained on the broker. Remaining:
HA device check in Home Assistant (no LXC account can read
`homeassistant/#`, so it cannot be verified from the broker), the §6.1
checklist items not yet exercised.
Phase A2 (charge sessions) next.

Deviations from the text below: the headline shows the robot's own
battery voltage whenever the robot bridge is online and the charger is not
live (§3.1 said only while cold — same intent). "Last seen" for an offline
dock is shown only when the offline transition was observed in this browser
session (the LWT carries no timestamp). The `ros2/dock/pi/system` card
also shows free RAM in MB. HA telemetry + maintenance-command entities ride
in the dock bridge config (Phase A, as planned).

## 0. Where things stand

| Piece | State | Consequence for the UI |
|---|---|---|
| Dock hardware + Nano fw 0.1.4 | done, charging a real robot | data exists |
| Dock Pi `dock_agent` (13 `/dock/*` topics on zenoh) | done, federates through the robot router | data exists **on ROS 2 only** |
| Dock → LXC MQTT path (second bridge instance, `dock` broker account) | **not built** | the web UI sees nothing dock-related today |
| Robot side (docking_server, `dock_manager`, `dock.json`) | **not started** | no Dock/Undock control, no dock pose |
| Web UI | `robotStatus.js` has "Charging" reserved; nothing else | clean slate |

Two hard facts from the dock side that shape the UI (HANDOFF §3):

1. **State 5 COMPLETE is real since fw 0.2.0 (2026-09-07).** A charge ends
   `CHARGING → DRAIN → COMPLETE` once the charger current has stayed below
   0.70 A with the charger voltage in the CV region for 60 s; the dock then
   goes fully cold and `battery_state.power_supply_status == FULL`. The dock
   does **not** restart on its own: the parked robot drains its pack at
   ~0.33 A until "Charging allowed" is pulsed off→on (manual top-up; a
   robot-side policy per 04 §6 is still to build). "Seated + IDLE" is now
   either "charging disabled" (`charge_enable` false) or a < 1 s transient
   before SEATED. (The 0.1.4-era "top-up cycling" was enable-off commands
   hitting the dock, not firmware — see the dock repo's TODO §6.)
2. `charger_voltage` reads ≈ 0 whenever the dock is cold; `battery_state`
   voltage/percentage are `null` (NaN) below 20 V. "0 V" is the normal idle
   value, never "battery empty".

## 1. Scope and phases

The work splits into five phases with **different prerequisites**. Phase A
needs no robot change at all and delivers most of the monitoring value, so it
goes first.

| Phase | Deliverable | Needs | Touches |
|---|---|---|---|
| **A — Telemetry** | New **Dock page** (status, live metrics, maintenance controls, dock Pi health + restart/reboot/shutdown), compact Charger card on the Status page, alerts, derived "Charging/Docked" robot status, top-bar ⚡, dock logs, HA telemetry + maintenance commands | dock bridge instance + LXC account/ACL | `mowbot_dock` repo, LXC, frontend |
| **B — Dock pose** | Dock + staging markers on both maps; "Save dock at robot position"; "Place on map" fallback; `dock.json` on the robot fileserver | robot fileserver/watcher edits (small, in the web UI repo), backend endpoints | web UI repo (`robot/`, backend, frontend) |
| **C — Control** | Dock / Undock / Cancel buttons with live action progress; "Undock & start"; docking reasons in plain English | 04 §2–3 (docking_server + `dock_manager`) | frontend, LXC ACL |
| **A2 — Charge statistics** | Docked periods / bursts / Ah on the Dock page (§4.8) | A, plus a few real charge cycles for tuning | backend, frontend |
| **D — Polish** | auto-dock toggle, HA docking buttons (Phase C topics) | A (+ C for auto-dock) | backend, frontend, dock `homeassistant.yaml` |

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
| `ros2/dock/launch/{cmd,status}` | bridge `launch_control` | status retained | on change | restart of dock units (Dock page) |
| `ros2/dock/power/{cmd,status}` | bridge `power_control` | status retained | on change | reboot / shutdown of the dock Pi with ACK (Dock page) — same manager as the robot's `ros2/power/*` |

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
drive_out_guard, reason, error_code, error_msg, started_at, retries, id}`
(field contract decided 2026-09-07 — 04 §3 table is authoritative),
`state ∈ idle | powering_lidar | staging | approaching | waiting_charge |
docked | undocking | failed | canceled`. `reason` is the goto-style string
code the `DOCK_REASON_TEXT` table (§5.3) maps to English; `error_code` /
`error_msg` are the raw Nav2 values for the Logs tab.
`ros2/docking/cmd`: `{action: "dock"|"undock"|"cancel", id}`.

**Status 2026-09-12 (evening): Phases B and C IMPLEMENTED and deployed**
(`mowbot_web_ui` commit "Docking plan 05 Phase B + C"). Phase B: fileserver
PUT for `config/dock.json`, `ros2/dock/pose_version`, `GET/PUT /api/dock`,
`POST /api/dock/record`, `DockPanel` on the map (save + two-click place),
dock/staging markers on both maps, Dock-page position card. Deviation:
`/api/dock/record` stores as yaw the **dock axis measured by the lidar V**
(`v_axis_yaw` in `ros2/docking/status`, published by `dock_manager` while the
robot is seated with the V in view) instead of the robot's heading, which
carries the ±2° funnel play; falls back to the heading with a warning. No
staging-distance input (fixed 1.2 m in nav2, 04 Q 1). Phase C: `DockControl`
(Dock / Undock / Cancel + progress + `DOCK_REASON_TEXT`), "Undock & start"
in Mission control, Drive-page warnings (seated / docking running), dashed
staging→dock line on the map, ACL `webui → ros2/docking/cmd`. First live
use: record via the backend wrote `dock.json` with `yaw_source: lidar V
axis` (−42.3°). **2026-09-12 later:** §11.4 robot-side mission gate while docked done;
**Phase D toggles done** — the dock bridge now runs as `/dock_mqtt_bridge`
(owner, dock repo unit file `-r __node:=dock_mqtt_bridge`), the three
auto-dock params are allowlisted under `param_control.nodes.mqtt_bridge_node`
and the Settings page has a "Docking" panel (`DockingSettings.jsx`; enabling
needs a second press). Remaining: A2 (charge stats), HA docking buttons
(Phase D, dock repo `homeassistant.yaml` + LXC bridge rule).

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

All consumers read one function so the Dock page, the Status-page Charger
card, the alerts, the top-bar chip and the robot-status banner never
disagree. Inputs: dock bridge online, `state`,
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
| 10 | `state` == 5 | **Charged** | ok | "battery full, charger cold"; `charge_enable` true → "Enable off→on to top up"; fw ≥ 0.2.0 reaches this within ~60 s of the current dropping below 0.70 A |
| 11 | `state` == 1 | **Seated** | warn | "waiting for charger…"; after 15 s seated with no charging: "charge did not start — check Charging allowed / fault" |
| 12 | `state` == 0 && `microswitch` | **Docked · resting** | ok | "seated, charger cold"; `charge_enable` false → "charging DISABLED" (warn). With fw ≥ 0.2.0 and enable true this is transient (< 1 s before SEATED) or the fault-3 retry hold-off — the full-pack rest state is row 10 |
| 13 | `state` == 0 && !`microswitch` | **Empty** | idle | "dock cold, ready" |
| 14 | anything else / no data | **—** | idle | — |

### 3.1 Percentage and voltage display

- While the charger is live (`voltage ≥ 20`): show charger voltage and the
  dock's linear estimate (33 V → 0 %, 42 V → 100 %), labelled "under charge
  (inflated by charge current)". This number is *not* the robot's SoC.
- While cold: show the **robot's** `hoverboard/battery_voltage` + existing
  `batteryPercent()` (when the robot bridge is online), labelled "robot
  battery". Never show the dock's 0 V as the pack voltage.

### 3.2 Charge session bookkeeping (client side, Phase A; backend in A2)

The browser keeps a small in-memory record per page session: time of last
`state == 3`, and a running sum of `current × Δt` while charging (2 Hz
samples). Used for the "last charged N min ago · ~0.4 Ah this session" line
on the Charged / Docked-resting rows. It resets on reload and is honest about it
(shown only after at least one charging sample was seen). Persistent,
authoritative session stats come from the backend in Phase A2 (§4.8).

## 4. What is shown where

### 4.1 Dock page (new, `pages/DockPage.jsx`, side-nav entry "Dock")

Everything dock-related lives on one page; the home page stays as it is
(owner decision). Layout, top to bottom, all built from `useDock()` + §3:

**1. Headline panel** — the §3 status, large, with tone colour; a "dock Pi
● online / offline (last seen …)" dot; and under it the live metrics line:
`2.1 A · 41.2 V · ~85 % under charge · charging 12 min` (or the robot's
battery voltage while the charger is cold, §3.1).

**2. Control row (Phase C, hidden until `ros2/docking/status` exists):**
`[ Dock ] [ Undock ] [ Cancel ]`, guarded exactly like `MissionControl`
(robot bridge online, mission process not running or mission idle, not
e-stopped; Dock/Undock double-confirm; Cancel immediate). Disabled reasons
shown as hints, refusals from `ros2/docking/status.error_code` mapped via a
`DOCK_REASON_TEXT` table like `GoToPanel`'s `REASON_TEXT`. Below the
buttons the action progress line (phase, retries, elapsed) while an action
runs.

**3. Charger detail card** — the hardware as it is right now:
`state <name> · switch <seated/free> · K1 <on/off> · K2 <on/off>`;
`current · charger voltage`; `charging allowed: yes/no`; fault text when
non-zero (with fault code); `self-test: OK/FAIL <when>`; `fw <version from
EVT:BOOT/VER> · uptime`; last event line `<EVT…> <ago>`. Plus a
`<details>` **raw battery_state** (what the docking server sees):
voltage/current/percentage/status/present, `null` shown as "n/a (cold)"
(decided 2026-09-06: keep, collapsed).

**4. Maintenance panel** (owner-tested 2026-09-06, all three commands work
against the real Nano). Always visible on this page (no collapsing — the
page exists for this), every button double-confirm, disabled when the dock
bridge is offline:

- **Charging allowed** on/off → `charge_enable_cmd`. Off while charging
  runs the sequenced shutdown (AC off → drain → K1 open) — say so in the
  confirm text. State shown from `ros2/dock/charge_enable` (§2.1 gap).
  Turning it **off→on while Charged (state 5)** is the manual "top up now"
  action (02 §4 re-entry rule — the dock never restarts by itself); label
  the on-button "Enable (starts a charge if seated)".
- **Clear fault** → `clear_fault_cmd`; enabled only for fault 1/2; hint
  "accepted by the firmware only at 0 A".
- **Self-test** → `self_test_cmd`; enabled only in state 0/5; result badge
  from `self_test_result` with timestamp; hint "≤ 2 s mains pulse with K1
  open; only while the charger is cold (Empty or Docked · resting)".

**5. Dock position card (Phase B)** — stored pose x/y/yaw, `saved_at`,
`method`, staging distance; **Save dock at robot position** (same button
and pre-checks as the map panel, §4.4 — the setup flow should not require
the map) and a "Show on map" link that opens the Map tab with the Dock
layer on. Placing by clicking stays on the map.

**6. Dock Pi card** — same renderer as the robot's "Raspberry Pi" card
(`piSystem()` on `ros2/dock/pi/system`): CPU, temp, RAM %, disk, WiFi % and
dBm. Justified by the two dock-Pi incidents in HANDOFF §5 (WiFi power-save
drop-outs, RAM exhaustion): RAM and WiFi signal are exactly what to watch on
a Pi 3B with 900 MB and no wired link. Tone: warn when RAM > 85 % or WiFi
< 30 %. Next to it (decided 2026-09-06, all in Phase A):

- **Restart dock agent / Restart dock zenoh router** → `ros2/dock/launch/cmd
  {id, action:"restart"}`, live unit state from `ros2/dock/launch/status`
  (mirrors `SystemPanel`'s zenoh-restart button). Agent restart mid-charge
  DTR-resets the Nano and drops the relays for a few seconds (HANDOFF §5) —
  in the confirm text.
- **Reboot dock Pi / Shut down dock Pi** — a `DockPower` panel that is the
  robot's `RobotPower.jsx` with the topics swapped (`ros2/dock/power/cmd`,
  retained `ros2/dock/power/status` `{state: idle|rebooting|shutting_down,
  id}`, online gate = `ros2/dock/bridge_status`): `ConfirmModal`, shutdown
  with type-to-confirm, in-flight banner until the LWT flips and the bridge
  comes back. Confirm texts: *reboot* — "dock Pi ↔ Nano link drops ~1 min;
  a running charge continues on the firmware's own interlocks (fault 4
  'watchdog silence' is reported meanwhile and self-clears)"; *shutdown* —
  "the dock Pi stays off until someone unplugs and re-plugs it — there is
  no remote power-on. The Nano keeps its USB 5 V from the halted Pi and
  keeps charging on its interlocks, but the web UI is blind until the Pi
  is back." (Verify both claims on the bench, §6.1.) Factor the shared
  parts of `RobotPower.jsx` into `PowerPanel.jsx` taking the topic names
  as props rather than copy-pasting.
- A "Dock logs" link into the Logs tab (`onShowLogs('dock_agent')`, the
  existing Launch-page pattern).

Dock Pi power stays **out of Home Assistant**, same as the robot Pi
(existing decision).

**7. Charge sessions card (Phase A2, §4.8)** — open period + last 10 docked
periods + totals.

Phase C additions outside this page: when `physically_docked` and the
mission is idle, the mission **Start** button in `MissionControl` (home
page) shows "Undock & start" (04 §6): UI sends `undock`, waits for docking
status `idle` + `microswitch false` (30 s watchdog), then `start`. A mission
refusal reason `docked` gets a `REASON_TEXT` entry: "robot is in the dock —
Undock first".

### 4.2 Status page — one compact card

**Charger** card in the cards grid after Battery: value = §3 headline, tone
likewise, detail = `current · charger voltage · state name` and a "→ Dock
page" link. No controls here; the Status page stays the all-telemetry
overview and the Dock page is where you act. (The Dock Pi card is on the
Dock page only.)

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
- **TopBar**: a ⚡ next to the battery chip while §3 says Charging, and a
  🔌 while Docked · resting (decided: yes). Uses the same `useDock()` hook;
  hidden entirely when the dock bridge is offline.

### 4.4 Map tab — `DockPanel` + markers (Phase B)

- **Markers** on `RobotMap` and `MiniMap`: dock marker at the stored pose
  (🔌 divIcon rotated to yaw, like the robot marker) and a small dashed
  circle at the computed staging pose. **The robot docks backwards**
  (charging contacts on the rear — decided 2026-09-06, README D10 rev.),
  so the stored pose's yaw (robot heading when seated) points *away* from
  the dock and the staging pose lies **in front of the seated robot**:
  `staging = pose + staging_offset_m · (cos yaw, sin yaw)`. The dock
  marker icon is drawn with its "mouth" on that side. `dock.json` still
  carries `dock_backwards: true` so the marker math and the robot side
  (04 §2.1) share one source of truth. `LayersPanel` gains a "Dock"
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
    point the robot approaches *from*; yaw = bearing from click **1 to
    click 2** (the seated robot faces away from the dock, toward where it
    came from).
    Two-step confirm, method `map_click`. Marked "rough — re-record by
    parking before first autonomous dock".
  - ~~**Staging distance** number input (0.5–1.5 m, default 0.7) stored in~~ **Deferred 2026-09-07 (04 Q 1):** no input for now; `staging_offset_m` is written by the backend with the fixed value (2.0 m for the first tests, must equal nav2's `staging_x_offset`) and only feeds the marker. Originally: number input stored in
    `dock.json`; the staging marker follows live.
  - Keep-out hint: if the stored pose lies inside a mow-area outline, show
    "the dock sits inside area X — add a hole around it in the area editor"
    (04 §5).
- No "delete dock" button (decided 2026-09-06): re-recording is the only
  correction path.

### 4.5 Settings page — "Docking" panel (Phases C/D)

- **Auto-dock on low battery** toggle (bridge param
  `auto_dock_on_low_battery`, 04 §6) — Phase D, rendered only when the
  param appears in `ros2/mowparams/status` (same pattern as the gate
  toggles).
- **Auto-dock when the mission completes** toggle (bridge param
  `auto_dock_on_mission_complete`, default off; 04 §6, decided 2026-09-07)
  plus the small `auto_dock_delay_s` number — same rendering rule.
  **Implemented 2026-09-12** as `DockingSettings.jsx` (own Apply, second press
  to enable automation), together with the **top-up while parked** controls
  (`topup_enabled`, `topup_voltage`, `topup_debounce_s`,
  `topup_min_interval_s`); the Dock page shows the last automatic top-up.
- Dock Pi service restart / reboot / shutdown live on the Dock page (§4.1
  item 6), not here.

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

Nothing. Dock units are always-on; restart buttons live on the Dock page.

### 4.8 Charge-session statistics — on the Dock page (decided 2026-09-06: wanted)

Shown as the last card of the Dock page (§4.1 item 7), not under the
robot's `Statistics`. Backend `stats.py` already consumes the broker; add a
sibling `dock_stats.py` subscribing `ros2/dock/#` (own JSON file, own
lock, so dock bookkeeping never touches the mowing stats):

- A **charge burst** = `state` enters {2,3} → leaves to {0,4,5}. Ah =
  Σ current·Δt (2 Hz samples), peak A, end charger voltage.
- A **docked period** = microswitch true … false; contains N bursts. This
  is the unit shown to the user (bursts merged), because of the top-up
  cycling: "Docked 2 h 14 min · 3 bursts · 1.8 Ah · ended 41.9 V".
- Store the last 50 periods in `dock_sessions.json`; expose
  `GET /api/dock/sessions` → `{current, periods[], totals}` where `current`
  is the open period (live "docked since … · 2 bursts · 0.9 Ah so far").
- Dock page card: the open period on top, then a table of the last 10
  (start, duration, bursts, Ah, end voltage, faults seen), then totals
  (periods, hours docked, Ah). Refreshed every 10 s like the event markers.
- Robustness: a backend restart mid-period closes nothing — the open period
  is persisted with each save and resumes if the microswitch is still true
  on restart; a dock-bridge LWT offline gap inside a period is recorded as
  `gaps: n` rather than ending it (charging continues autonomously).
- Phasing: no robot dependency — it needs only Phase A telemetry. Build it
  as **Phase A2**, right after the Dock page is live and the first real
  periods have been observed (so the burst/period thresholds are tuned on
  real data), rather than waiting for Phase D.

### 4.9 Home Assistant — telemetry **and** commands (decided 2026-09-06)

The dock bridge instance gets its own `homeassistant.yaml`
(`homeassistant_discovery:` in the dock `topics.yaml`; device "Mowbot
Dock", availability = `ros2/dock/bridge_status`), published as retained
discovery configs — so the `dock` broker account needs `topic write
homeassistant/#` (the robot account already has it). Config-only work; it
can ride with Phase A, Phase C adds the docking buttons.

Telemetry entities (formulas identical to §3 / `lib/dockStatus.js`):
`binary_sensor` Charging (state ∈ {2,3}), `binary_sensor` Robot seated
(microswitch), `sensor` Charge current (A), Charger voltage (V), Dock state
(text via a value_template map), Dock fault (text), `binary_sensor` Charging
allowed, `sensor` Dock Pi WiFi (%) and RAM (%), `binary_sensor` Dock Pi
online (from the LWT). `expire_after` on the volatile ones. The LXC→HA
bridge already exports `ros2/#`, so telemetry needs **no LXC bridge-rule
change**.

Command entities follow the recorded `ha/ros2/` convention (HA publishes
`ha/ros2/<topic>` on its own broker; the LXC imports it with the prefix
stripped; **never retained**):

| Entity | Component | `command_topic` (HA side) | Payload | State |
|---|---|---|---|---|
| Dock charging allowed | `switch` | `ha/ros2/dock/charge_enable_cmd` | `{"data": true}` / `{"data": false}` | `ros2/dock/charge_enable` |
| Dock clear fault | `button` | `ha/ros2/dock/clear_fault_cmd` | `{"data": true}` | — |
| Dock self-test | `button` | `ha/ros2/dock/self_test_cmd` | `{"data": true}` | result on `ros2/dock/self_test_result` as a `binary_sensor` |
| Dock robot / Undock robot (Phase C) | `button` ×2 | `ha/ros2/docking/cmd` | `{"action": "dock"}` / `{"action": "undock"}` | `ros2/docking/status` as a `sensor` |

No dock Pi power entities in HA (matches the robot decision).

LXC `mowbot-remote.conf` needs one inbound rule per command (targeted
append to the working copy **and** `/etc/mosquitto/conf.d/mowbot-remote.conf`):

```
topic dock/charge_enable_cmd in 1 ros2/ ha/ros2/
topic dock/clear_fault_cmd   in 1 ros2/ ha/ros2/
topic dock/self_test_cmd     in 1 ros2/ ha/ros2/
topic docking/cmd            in 1 ros2/ ha/ros2/    # Phase C
```

followed by `systemctl restart mosquitto` — a reload does **not** apply
bridge changes (recorded lesson). Loop-free for the same reason as the
existing rules: only `ros2/#` is exported. Note the HA-side `switch` has no
optimistic mode: its state follows the retained `ros2/dock/charge_enable`
echo, which is why the §2.1 agent publisher is a prerequisite.

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
GNSS/goto/manual-mow managers and their message deps (the power and
launch managers stay — the dock uses them). Start with the
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
power_control:                       # reboot / shutdown of the dock Pi, same manager as the robot
  cmd_topic: ros2/dock/power/cmd
  status_topic: ros2/dock/power/status
  grace_ms: 1500
  actions: [reboot, shutdown]
homeassistant_discovery: homeassistant.yaml   # §4.9 (device "Mowbot Dock")
```

`launch_control` and `power_control` exec `sudo -n systemctl …` (robot
pattern, PLAN_POWER_CONTROL.md) — the dock's `ubuntu` user therefore needs
passwordless sudo for those commands. `deploy.sh` already assumes `sudo -n
systemctl restart` works; make it explicit in `PI_SETUP.md` with a scoped
sudoers line rather than blanket NOPASSWD:
`ubuntu ALL=(root) NOPASSWD: /usr/bin/systemctl restart mowbot-dock-agent.service, /usr/bin/systemctl restart mowbot-dock-mqtt-bridge.service, /usr/bin/systemctl restart zenoh-dock-router.service, /usr/bin/systemctl reboot, /usr/bin/systemctl poweroff`
(check the exact argv the managers build before writing it — `launch_manager`
may pass `start`/`stop` too).

No `param_control`, `goto` or `manual_mow` sections. Verify at first run that the bridge accepts a
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
  topic read ros2/dock/power/cmd
  topic write homeassistant/#
  # under "user webui":
  topic write ros2/dock/charge_enable_cmd
  topic write ros2/dock/clear_fault_cmd
  topic write ros2/dock/self_test_cmd
  topic write ros2/dock/logs/cmd
  topic write ros2/dock/launch/cmd
  topic write ros2/dock/power/cmd
  ```

- The HA command rules from §4.9 go into `mowbot-remote.conf` in the same
  session, then `systemctl restart mosquitto` (bridge-rule changes need a
  restart; a reload would suffice for ACL/passwd alone). `ros2/# out`
  already forwards dock telemetry to HA.

**A4 — frontend.**

| File | Change |
|---|---|
| `lib/dockStatus.js` (new) | §3 function + `DOCK_STATE_NAME`, `DOCK_FAULT_TEXT` tables |
| `hooks/useDock.js` (new) | one hook subscribing every `ros2/dock/*` topic, applying §2.4 freshness, returning `{online, state, seated, fault, chargeEnable, current, voltage, battery, event, fwVersion, status}`; the §3.2 client-side session accumulator lives here |
| `pages/DockPage.jsx` (new) | §4.1 layout; sub-components `components/dock/{DockHeadline,ChargerDetail,DockMaintenance,DockPiCard,DockPower}.jsx` now; `DockControl` (C), `DockPosition` (B), `DockSessions` (A2) slot in later |
| `components/SideNav.jsx` + `App.jsx` | add `['dock', 'Dock']` to `PAGES` (after Map) and the `page === 'dock'` branch (pass `onShowLogs` like `LaunchPage`) |
| `pages/StatusPage.jsx` | compact Charger card (§4.2) |
| `components/AlertBanner.jsx` | §4.3 dock alerts |
| `components/TopBar.jsx` | ⚡ / 🔌 chip (§4.3) |
| `components/RobotPower.jsx` → `PowerPanel.jsx` | topic names as props; `RobotPower` and `DockPower` become thin wrappers (§4.1 item 6) |
| `lib/robotStatus.js` + `RobotStatusBanner.jsx` | Charging / Docked states, new `dock` input |
| `pages/LogsPage.jsx` | multi-source (§4.6) |
| `lib/telemetry.js` | `piSystem()` reuse; `dockPercent()` for §3.1 |
| `styles.css` | `.dock-page`, `.dock-headline`, badge styles |

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
    "staging_offset_m": 2.0,
    "dock_backwards": true,
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
- Dock page control row (§4.1 item 2) + `DOCK_REASON_TEXT` keyed by `status.reason` (mirror `dock_manager`'s reason
  codes: `estop`, `mission_active`, `mission_state_unknown`, `no_dock_pose`,
  `nav2_unavailable`, `nav2_timeout`, `failed_to_charge`,
  `failed_to_detect_dock`, `cancel_timeout`, `not_docked` (undock while not
  seated), `timeout`).
- Status merge rule: while an action is in flight the action status leads
  (§3 rows 6–7); when it ends, hardware state leads again. A `failed`
  result with `physically_docked: true` shows "docked (action reported
  failure — WiFi at contact?)" per 04 §4.4.
- "Undock & start" sequencing in `MissionControl` (§4.1).
- Drive page (04 §10.4 item 2, decided 2026-09-07): while
  `ros2/docking/status.physically_docked` is true show a warning above the
  pad — "robot is in the dock — charging is switched off while you drive;
  prefer Undock" — and reflect `drive_out_guard` in the docking status
  line. The robot side (bridge drive-out guard) does the actual
  enable-off; the UI only explains it.
- Map: while `staging/approaching`, draw the staging → dock approach line on
  the map (dashed) so the operator sees what the robot is trying to do.

### 5.4 Phase D — polish

HA Dock/Undock buttons (§4.9, on the Phase C topics) and the Settings →
Docking panel with the `auto_dock_on_low_battery` toggle (§4.5) once the
bridge exposes that param.

## 6. Test checklists

### 6.1 Phase A

- [ ] Dock Pi: `mowbot-dock-mqtt-bridge` up, `ros2/dock/bridge_status`
      retained `online:true` on the LXC (`mosquitto_sub -v -t 'ros2/dock/#'`).
- [ ] Robot **off**: Dock page live, state 0 / Empty; Status page Charger
      card agrees; Dock Pi card shows WiFi + RAM; home page unchanged.
- [ ] `free -m` on the dock Pi after 24 h with both services: no swap
      thrash (HANDOFF §5 history).
- [ ] Seat the robot manually: card walks Seated → Charging within ~1 s;
      current/voltage update at 2 Hz; robot-status banner says Charging.
- [ ] Wait out a full charge: DRAIN → **Charged** (state 5; fw ≥ 0.2.0,
      ~60 s after the current drops below 0.70 A on a full pack); "last
      charged … ago" appears; Charging allowed off→on starts a top-up and it
      returns to Charged.
- [ ] Charging allowed → off mid-charge: sequenced shutdown visible (K2 off,
      DRAIN, K1 off, Empty/resting), alert "charging is disabled" while
      seated; → on: charge restarts.
- [ ] Self-test from Empty: result badge + timestamp; try it during charging:
      firmware ignores it, UI button was disabled anyway.
- [ ] Unplug the dock Pi's power: LWT flips within the keepalive, cards grey
      with "last seen", no alert; power back: recovers without a reload.
- [ ] Logs tab: dock group lists four services; fetch works; robot group
      unaffected.
- [ ] Restart dock agent from the Dock page while charging: relays drop,
      `EVT:BOOT`, charge resumes by itself, no latched fault.
- [ ] Reboot dock Pi from the Dock page mid-charge: ACK state shown, LWT
      offline, charging continues (watch the robot's battery current /
      the dock LED), fault 4 reported then self-clears, page recovers when
      the bridge returns — no reload.
- [ ] Shut down dock Pi (with the robot **not** seated, first time): halted
      Pi still powers the Nano over USB (LED / bench meter); page shows
      offline; physical re-plug brings everything back.
- [ ] HA: dock device appears with all telemetry entities; "Dock charging
      allowed" switch toggles and its state follows the echo; Clear fault
      and Self-test buttons act (watch the dock page); nothing retained on
      `ha/ros2/#`.
- [ ] Top bar shows ⚡ while charging, 🔌 while docked-resting, nothing when
      the dock is offline.
- [ ] Phone width: Dock page stacks cleanly; side-nav entry visible.

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
      dock goes cold before the robot moves (watch K1/K2 in the Dock page's
      charger detail card).
- [ ] "Undock & start" runs the sequence and refuses if undock fails.

## 7. Deploy and ownership notes

- Dock side builds only on the x86 workstation (`dock-build.sh` →
  `deploy.sh`), never on the Pi; never open VS Code Remote-SSH on the dock
  Pi (HANDOFF §5).
- LXC steps (passwd, ACL append, HA bridge rules, mosquitto restart,
  frontend rsync + build,
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
- Dock Pi power controls **in Home Assistant** (they exist in the web UI
  only — mirrors the robot-Pi decision).

## 9. Open questions for the owner

| # | Question | Plan's default if unanswered |
|---|---|---|
| ~~Q 1~~ | ~~Home page dock card?~~ **Resolved 2026-09-06: no.** Home page unchanged; the dock gets its own page (§4.1) plus a compact Charger card on Status (§4.2). | — |
| ~~Q 2~~ | ~~Maintenance controls in the UI?~~ **Resolved 2026-09-06: yes** — owner tested Charging allowed / Clear fault / Self-test against the real Nano, all OK. Always visible on the Dock page, double-confirm. | — |
| ~~Q 3~~ | ~~TopBar ⚡ while charging?~~ **Resolved 2026-09-06: yes.** | — |
| ~~Q 4~~ | ~~Robot-status banner "Charging" / "Docked"?~~ **Resolved 2026-09-06: yes** — the strip at the top of the Mowbot and Status pages shows Charging / Docked instead of Idle whenever the robot sits in the dock (§4.3). | — |
| ~~Q 5~~ | ~~Clear button for the dock pose?~~ **Resolved 2026-09-06: no.** Re-recording is the only correction path; an "enabled" toggle can be added later if the dock is ever removed for a season. | — |
| ~~Q 6~~ | ~~Charge-session statistics?~~ **Resolved 2026-09-06: wanted, on the Dock page.** Built as Phase A2 (backend `dock_stats.py` + Dock page card, §4.8) since it needs only Phase A telemetry. | — |
| ~~Q 7~~ | ~~HA entities?~~ **Resolved 2026-09-06: telemetry and commands** (§4.9): charging-allowed switch, clear-fault and self-test buttons now; Dock/Undock buttons with Phase C. No dock Pi power in HA. | — |
| ~~Q 8~~ | ~~Dock Pi restart buttons?~~ **Resolved 2026-09-06: yes, plus reboot and shutdown** of the dock Pi from the Dock page, same mechanism and UI as the robot Pi (§4.1 item 6). | — |
| ~~Q 9~~ | ~~The COMPLETE decision: firmware or agent remap?~~ **Resolved 2026-09-07: firmware.** fw 0.2.0 makes state 5 reachable (completion = current < 0.70 A ∧ V#1 ≥ 41 V for 60 s — the docked robot's own ~0.4 A draw through the charger made the plan's 0.15 A unreachable); first real COMPLETE 17:40:17, `FULL` now reported. The 08-23 "DRAIN→IDLE" observations were enable-off commands, not firmware. Row 10 "Charged" applies; row 12 re-worded; §0 fact 1 replaced. | — |
| ~~Q 10~~ | ~~Raw `battery_state` view?~~ **Resolved 2026-09-06: keep it**, collapsed in a `<details>` on the charger detail card (§4.1 item 3) — it shows exactly what the docking server is fed when a docking attempt fails to detect charging. | — |
| ~~Q 11~~ | ~~Forward or backward docking?~~ **Resolved 2026-09-06: backwards** — the robot's charging contacts are on the rear. README D10, 01 §4.1, 03 §6.4 and 04 §1/§2.1 updated to match. | — |
