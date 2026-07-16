# 03 — Dock Raspberry Pi (ROS 2) & Web UI Integration

> Status: **PLANNED 2026-07-10.** Part of the docking master plan
> ([README.md](README.md)). Covers everything that runs on the dock Pi,
> how it joins the robot's zenoh network and the LXC MQTT world, and the
> complete web-UI dock setup guide — including moving the dock later.

## 1. Dock Pi base system

1. Ubuntu Server 24.04 arm64, hostname `mowbot-dock`, fixed DHCP lease
   (referenced below as `<DOCK_IP>`; robot Pi = `<ROBOT_IP>`).
2. `ros-jazzy-ros-base` + `ros-jazzy-rmw-zenoh-cpp` +
   colcon/dev tools. `RMW_IMPLEMENTATION=rmw_zenoh_cpp` in `.bashrc`
   **and in every systemd unit** (recorded project gotcha: sourcing
   setup.bash alone leaves nodes on Fast DDS, invisible to zenoh peers).
3. User `ros-dock` (or reuse `ros-pi` naming), SSH key from the robot Pi
   and/or your workstation.
4. Workspace `~/dock_ws/src` — built with `colcon build --symlink-install`
   (project convention).

## 2. zenoh topology

The robot Pi already runs `zenoh-router.service`. The dock Pi runs its
**own zenoh router** whose config lists the robot router as a connect
endpoint (standard rmw_zenoh multi-host pattern — local nodes always
talk to the local router; routers federate over WiFi):

```json5
// dock zenoh router config, connect section
connect: { endpoints: ["tcp/<ROBOT_IP>:7447"] }
```

- Dock nodes keep working (and buffering nothing — plain pub/sub) when
  the robot is off; topics simply have no remote subscribers.
- The robot's `docking_server` sees `dock/*` topics whenever both ends
  are up — verify with `ros2 topic list` from the robot while the dock
  agent runs.
- `zenoh-dock-router.service` = copy of the robot's unit with the dock
  config; `After=network-online.target`.

## 3. New package: `mowbot_dock` (dock Pi workspace, own git repo)

One C++ package, one executable — `dock_agent_node`. Deliberately shaped
like the planned robot-side C++ `arduino_bridge`
(`pi_ws/src/ros2-driver-converted/plans/PLAN_ARDUINO_BRIDGE_CPP.md`):
TX timer + RX thread over termios, tolerant CSV parser. The
`serial_port.{hpp,cpp}` RAII wrapper specified there is copied verbatim
(or the package is started from that skeleton) — write it once, use it
on both machines.

```
mowbot_dock/
├── package.xml / CMakeLists.txt      # rclcpp, std_msgs, sensor_msgs
├── include/mowbot_dock/
│   ├── serial_port.hpp               # shared pattern from PLAN_ARDUINO_BRIDGE_CPP
│   └── dock_agent_node.hpp
├── src/
│   ├── serial_port.cpp
│   ├── dock_agent_node.cpp
│   └── dock_agent_main.cpp
├── config/dock_params.yaml           # port, baud, thresholds, frame ids
└── launch/dock_agent.launch.py
```

### 3.1 `dock_agent_node` ROS interface

Subscribes:

| Topic | Type | Effect |
|---|---|---|
| `dock/charge_enable_cmd` | `std_msgs/Bool` | forwarded as the protocol `chargeEnable` permission (default true at node start — matches firmware §02-4) |
| `dock/clear_fault_cmd` | `std_msgs/Bool` | edge → `clearFault` field |

Publishes (each RX frame, ~10 Hz):

| Topic | Type | Source field |
|---|---|---|
| `dock/state` | `std_msgs/Int32` | state (0–4) |
| `dock/microswitch` | `std_msgs/Bool` | seat switch |
| `dock/relay` | `std_msgs/Bool` | relay state |
| `dock/charge_current` | `std_msgs/Float32` | A |
| `dock/charger_voltage` | `std_msgs/Float32` | V |
| `dock/contact_voltage` | `std_msgs/Float32` | V |
| `dock/fault` | `std_msgs/Int32` | fault code |
| `dock/battery_state` | `sensor_msgs/BatteryState` | **the topic `docking_server` consumes** |

`dock/battery_state` composition: `current` = +charge current (A),
`voltage` = contact voltage, `power_supply_status` = `CHARGING` while
state==2, `FULL` while state==3, else `NOT_CHARGING`; `present` =
microswitch. Published at 2 Hz (steady, low-rate — it crosses WiFi).
This gives `SimpleChargingDock`'s `isCharging()` (current >
`charging_threshold`) exactly what it needs, and carries the
microswitch for the possible custom plugin later (README D8).

Behavior notes:

- Serial handling = robot-bridge pattern: 1 s open-settle + flush,
  skip lines ≠ 8 numeric fields, locale-safe float parse, throttled
  warns, node exits nonzero on port-open failure (systemd restarts).
- `uptimeS` regression (Nano rebooted) → info log + republish current
  command state.
- 10 Hz command TX keeps the Nano watchdog fed; remember firmware
  treats silence as "keep charging on interlocks", not "stop".

## 4. Second `mowbot_mqtt_bridge` instance (dock → LXC directly)

The dock Pi runs its own instance of the existing
`mowbot_mqtt_bridge` package (clone + `colcon build` on the dock Pi)
with a dock-specific `topics.yaml` + `secrets.yaml` (the 2026-07-08
config split supports exactly this). This is README decision D6 — the
web UI monitors the 42 V charger even when the robot is powered down,
and the dock gets `log_control` + `launch_control` for free (dock
service logs and restarts from the web UI Logs/Launch tabs, ids
namespaced e.g. `dock_agent`, `dock_zenoh`).

`topics.yaml` (dock instance) maps:

| ROS 2 | MQTT | dir | retain |
|---|---|---|---|
| `dock/state` | `ros2/dock/state` | publish | yes (slow-changing, UI wants last value) |
| `dock/microswitch` | `ros2/dock/microswitch` | publish | yes |
| `dock/relay` | `ros2/dock/relay` | publish | yes |
| `dock/charge_current` | `ros2/dock/charge_current` | publish | no |
| `dock/charger_voltage` | `ros2/dock/charger_voltage` | publish | no |
| `dock/contact_voltage` | `ros2/dock/contact_voltage` | publish | no |
| `dock/fault` | `ros2/dock/fault` | publish | yes |
| `ros2/dock/charge_enable_cmd` | → `dock/charge_enable_cmd` | subscribe | never |
| `ros2/dock/clear_fault_cmd` | → `dock/clear_fault_cmd` | subscribe | never |

Bridge LWT (existing mechanism) → `ros2/dock/bridge_status` so the UI
can grey the dock card when the dock Pi is offline.

**LXC side (hand the user exact commands — deploys are classifier-denied):**

- New mosquitto account `dock` (password into
  `/root/mowbot-mqtt-credentials.txt` + dock `secrets.yaml`).
- ACL (targeted append, then reload): `user dock` → `topic write
  ros2/dock/#`, `topic read ros2/dock/charge_enable_cmd`, `topic read
  ros2/dock/clear_fault_cmd`; `user webui` → add `topic write
  ros2/dock/charge_enable_cmd` + `ros2/dock/clear_fault_cmd` (and the
  robot-side `ros2/docking/cmd` from 04 §7).

## 5. systemd units on the dock Pi

| Unit | Runs | Notes |
|---|---|---|
| `zenoh-dock-router.service` | zenoh router w/ connect endpoint | enabled at boot |
| `mowbot-dock-agent.service` | `dock_agent_node` | `Restart=on-failure`, `RestartSec=5`, `Environment=RMW_IMPLEMENTATION=rmw_zenoh_cpp`, `After=zenoh-dock-router.service` |
| `mowbot-dock-mqtt-bridge.service` | dock bridge instance | same env; `After=network-online` |

All three enabled — the dock must come back unattended after a power
cut (the firmware charges autonomously meanwhile).

## 6. Web UI — dock configuration & control

### 6.1 Where the dock pose lives

`~/pi_ws/mowing_data/config/dock.json` **on the robot fileserver**
(same PUT-with-token + backups + version-watcher infra as areas):

```json
{
  "pose": { "x": 12.34, "y": -5.67, "yaw": 1.571, "frame": "map" },
  "saved_at": "2026-07-12T10:15:00Z",
  "method": "robot_pose",
  "staging_offset_m": 0.7
}
```

The pose is the **robot's `base_link` pose when fully seated in the
dock** (not the dock structure itself) — that is what `DockRobot` gets
as `dock_pose`, and what "record by parking" naturally produces. Yaw =
robot heading when seated (approach direction). It is read at
dock-command time by the robot bridge's `dock_manager` (04 §7), so
updating the file is all it takes to move the dock — no restarts
(README D3). Frame note: `map` is datum-fixed (backend-baked datum), so
the pose survives reboots; if the datum is ever changed, the dock must
be re-recorded — same caveat as all mow areas.

### 6.2 Backend (FastAPI on the LXC)

- `GET /api/dock` — current dock.json (404 if unset).
- `PUT /api/dock` — validate + authed PUT to the robot fileserver
  (backup convention: `config/backups/`, newest 10).
- `POST /api/dock/record` — **"Save dock here"**: backend takes the
  latest cached `ros2/odometry/global` MQTT sample (it already consumes
  this topic for stats), refuses if stale (> 3 s) or missing, else
  writes dock.json with `method: robot_pose`. Include σ/RTK sanity warn
  in the response if available.

### 6.3 Frontend

**DockPanel on the Map tab** (mutually exclusive with editor / recorder
/ coverage / goto panels, same pattern as `GoToPanel.jsx`):

- Dock marker (🔌 icon + short heading arrow) on RobotMap and MiniMap at
  the stored pose; small dashed marker at the computed staging pose.
- "Save dock at robot position" (primary, two-step confirm — this is
  the setup flow).
- "Place on map" fallback: two clicks — position, then a point the
  robot should approach *from* (yaw = from second point toward dock);
  useful for rough pre-placement before the first park-and-record.

**DockCard on the Mowbot page** (next to Mission control):

- State line driven by retained `ros2/docking/status` (robot-side
  action progress: staging / approaching / waiting for charge — 04 §7)
  merged with dock hardware topics (`ros2/dock/state`, current,
  voltages, microswitch, fault text).
- Buttons: **Dock** / **Undock** / **Cancel** → `ros2/docking/cmd`
  (robot side), guarded like Mission control (disabled unless mission
  idle; confirm dialogs).
- Charging view while docked: current (A), pack voltage, est. % (33 V
  empty → 42 V full), "charge complete" badge, session timer.
- Maintenance row (collapsed `<details>`): force charge-enable off/on,
  clear fault → `ros2/dock/charge_enable_cmd` / `ros2/dock/clear_fault_cmd`.
- Card greys via dock bridge LWT (dock Pi offline ≠ robot offline).

**TopBar / status banner:** while `ros2/docking/status` is active,
banner shows "Docking…"; while charging, battery chip gains a ⚡.

### 6.4 Setup / relocation guide (the answer to "if I move the dock")

First-time commissioning:

1. Build dock (01), place it (01 §4.3 RTK check!), power it up.
2. Verify dock telemetry on the Mowbot page (dock card live, state 0).
3. Drive the robot into the dock manually with the Drive pad until the
   microswitch clicks (dock card shows "seated", relay closes, current
   flows — hardware is now proven end-to-end).
4. Press **"Save dock at robot position"**. Done — pose recorded in the
   exact localization frame docking will use.
5. Undock (Undock button, or drive out backward), then press **Dock**
   from ~3 m away and watch the first autonomous docking. Repeat from a
   few directions/distances.

Moving the dock later:

1. Physically move it; check RTK quality at the new spot.
2. Drive in manually once, press "Save dock at robot position".
3. That's it — next `Dock` command uses the new pose (dock.json is read
   per-command; nothing restarts).

### 6.5 Home Assistant (optional, P4)

The dock bridge instance has its own `homeassistant.yaml` support —
add `charging` binary sensor, charge current, charger voltage, fault.
Commands from HA would need the `ha/ros2/` inbound bridge-rule
convention on the LXC; telemetry-only is zero extra config beyond the
discovery file. Recommend telemetry-only initially.

## 7. Test checklist (dock Pi layer)

- [ ] `ros2 topic hz /dock/charge_current` ≈ 10 Hz locally on the dock Pi.
- [ ] Same topic visible from the **robot** Pi (`ros2 topic list`,
      `ros2 topic echo /dock/battery_state`) — zenoh federation works.
- [ ] Robot powered off → dock topics still reach the web UI via the
      dock's own MQTT bridge; dock card live.
- [ ] `ros2/dock/charge_enable_cmd false` from the web UI (as webui
      account) opens the relay mid-charge; `true` restores; retained
      status topics update.
- [ ] Dock Pi reboot: all three units come up, card recovers, no manual
      steps.
- [ ] WiFi outage 5 min (unplug AP or block on router): charging
      continues (firmware autonomy), LWT greys the card, recovery clean.
- [ ] `POST /api/dock/record` with robot parked in dock → dock.json
      written, backup created, marker appears on both maps.
- [ ] Logs tab shows `dock_agent` unit logs via the dock bridge
      `log_control`.
