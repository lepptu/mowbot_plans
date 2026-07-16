# Mowbot Docking & Charging — Master Plan

> Status: **PLANNED 2026-07-10 — no code or hardware changes yet.**
>
> Goal: mowbot docks itself and starts charging, fully controlled and
> monitored from the web UI, with the dock position changeable from the
> map at any time.

## Plan files

| File | Scope |
|---|---|
| [01_HARDWARE.md](01_HARDWARE.md) | Charging dock hardware: charger, contacts, relay, sensors, dock structure, robot-side charge port |
| [02_ARDUINO_FIRMWARE.md](02_ARDUINO_FIRMWARE.md) | Dock Arduino Nano firmware (.hpp/.cpp structure, serial protocol, safety interlocks) |
| [03_DOCK_PI_ROS2_AND_WEBUI.md](03_DOCK_PI_ROS2_AND_WEBUI.md) | Dock Raspberry Pi: OS, zenoh, dock agent node, MQTT bridge to the LXC, web UI dock setup guide |
| [04_ROBOT_MODIFICATIONS.md](04_ROBOT_MODIFICATIONS.md) | Robot changes: charge contacts, Nav2 `docking_server`, bridge `dock_manager`, mission integration |

## The one-paragraph design

The dock is a mains-powered station with a 42 V CC/CV lithium charger,
an **Arduino Nano** doing I/O (charge relay, ACS712 current sensor,
voltage dividers, seat microswitch) and a **Raspberry Pi** running ROS 2
Jazzy that talks to the Nano over USB serial — the exact architecture
already proven on the robot (`arduino_bridge` + `mowbot_robot_arduino`
firmware). The dock Pi joins the robot's **zenoh** network over WiFi and
runs a second instance of `mowbot_mqtt_bridge` straight to the LXC
broker, so the web UI sees the charger even when the robot is off.
Docking itself is **Nav2's `opennav_docking` server** (already installed,
v1.3.10): the robot navigates to a staging pose ~0.7 m in front of the
dock, then drives blind on RTK into the contacts until charge current is
detected. The dock pose is **recorded by parking the robot in the dock
once** and pressing "Save dock here" in the web UI — this cancels any
systematic GPS offset and makes moving the dock a 2-minute job.

## Architecture

```
            MQTT (LAN/WiFi)                      MQTT (LAN)
[LXC .133] ◄──────────────────► [Robot Pi 5] ◄─┐   ┌─► [LXC .133]
 mosquitto                       mowbot_mqtt_   │   │    (same broker,
 FastAPI backend                 bridge         │   │     account "dock")
 nginx + SPA                     + NEW dock_manager │
                                 Nav2 + NEW         │
                                 docking_server     │
                                 zenoh-router ◄── zenoh tcp ──► [Dock Pi]
                                 arduino_bridge                  zenoh (client of robot router)
                                    │ USB serial                 mowbot_dock dock_agent_node
                                 [Robot Nano]                    mowbot_mqtt_bridge (2nd instance)
                                                                    │ USB serial
                                                                 [Dock Nano]
                                                                  relay 42V / ACS712 /
                                                                  V-dividers / microswitch
        [42V charger] ──► relay ──► dock contacts ──► robot contacts ──► fuse ──► ideal diode ──► battery
```

Two independent data paths, on purpose:

1. **ROS 2 (zenoh)** — dock agent topics (`dock/battery_state`,
   `dock/microswitch`, …) reach the robot's `docking_server` for
   charge/docked detection. Only needed while the robot is docking.
2. **MQTT (direct)** — the dock Pi's own bridge instance publishes
   charger telemetry to the LXC broker and accepts web-UI commands
   (force relay off, clear fault). Works with the robot powered down —
   important for a mains-powered 42 V relay you want to watch remotely.

## Key design decisions

| # | Decision | Rationale |
|---|---|---|
| D1 | Use `opennav_docking` (Nav2 Jazzy built-in), not a custom docking node | Installed already; handles staging navigation, retry logic, wait-for-charge, undock. Actions: `/dock_robot`, `/undock_robot`. |
| D2 | **Blind RTK docking** (no camera/AprilTag) in phase 1 | RTK gives 2–3 cm; mechanical funnel absorbs the rest. The robot camera + AprilTag is a documented upgrade path if field results demand it (`opennav_docking` supports an external `detected_dock_pose` topic). |
| D3 | Dock pose passed **explicitly in the `DockRobot` action** (`use_dock_id: false`), stored in `~/pi_ws/mowing_data/config/dock.json` on the robot fileserver | Moving the dock = update one JSON from the web UI. No docking-server restart, no dock-database reload. Same infra as areas/keepouts (token PUT, backups, version watcher). |
| D4 | Dock pose is **recorded by parking the robot** in the dock ("Save dock here") | Guarantees the stored pose is in exactly the localization frame the approach will use; cancels systematic GPS/antenna offsets. Map-click entry is the fallback. |
| D5 | Dock-side Nano over **USB serial** (not Pi GPIO) | Proven robot pattern (tolerant CSV parser, DTR auto-reset handling); Nano provides the ADC the Pi lacks; USB powers the Nano; keeps 42 V wiring away from Pi GPIO. |
| D6 | Dock Pi runs a **second `mowbot_mqtt_bridge` instance** | Zero new MQTT code; gets `log_control`/`launch_control` on the dock for free (view dock logs / restart dock services from the web UI); dock monitorable independently of the robot. |
| D7 | Charge relay policy is **autonomous on the dock** (microswitch + interlocks close it), robot never *needs* to command charging | Docking still works if the zenoh link hiccups at contact moment; firmware interlocks are the safety authority (see 02, §4). |
| D8 | Phase 1 uses `SimpleChargingDock` config-only; a tiny custom `MowbotChargingDock` plugin (isDocked = microswitch, isCharging = current) is the fallback if pose-based docked-detection proves unreliable | Try zero-code first; the custom plugin is ~150 lines and matches the team's existing plugin experience. |
| D9 | Robot-side contacts isolated by an **ideal-diode module + fuse** | Robot contacts must never be live at battery voltage when driving around; diode also blocks reverse current into the charger. |
| D10 | Forward docking, contacts at the front | Front bumper already defines the "nose"; `opennav_docking` docks forward by default; backing out on undock leaves the blade side away from the dock. |

## Phases

| Phase | Deliverable | Depends on |
|---|---|---|
| **P0** | Decisions confirmed, parts ordered (01 §7 shopping list) | — |
| **P1** | Dock hardware built + Nano firmware bench-tested on a desk (USB to any PC; no dock Pi needed yet) | P0 |
| **P2** | Dock Pi commissioned: OS + zenoh + `dock_agent` + bridge; charger telemetry visible in web UI | P1 |
| **P3** | Robot mods: contacts + wiring; `docking_server` in nav launch; bridge `dock_manager`; web UI dock panel + "Save dock here"; **manual Dock / Undock end-to-end** | P2 |
| **P4** | Automation: auto-dock on low battery, "docked" mission-start interlock, charge-complete handling, stats/HA polish | P3 |

Each plan file has its own test checklist; P3 ends with the full
field-test sequence in 04 §8.

## Conventions that apply (from existing project practice)

- `colcon build --symlink-install` always; C++ changes need a rebuild.
- Every systemd unit running a ROS node sets
  `Environment=RMW_IMPLEMENTATION=rmw_zenoh_cpp` (on the dock Pi too).
- LXC deploys (ACL edits, frontend build, backend restart) are handed to
  the user as exact commands — targeted append for ACL lines, never
  overwrite; `runuser -u mowbot -- npm run build` for the SPA.
- New MQTT command topics need LXC ACL lines **and** never retain
  command messages.
- All new code/comments/logs in English; public topic/param names follow
  the existing naming style.

## Open questions (tracked, non-blocking)

1. Charger: reuse the existing hoverboard charger or buy a dedicated
   42 V 2–4 A unit for the dock (recommended — see 01 §2).
2. `use_collision_detection` on the docking server during final
   approach: start disabled (dock itself is a LiDAR obstacle), revisit
   after field tests (04 §5).
3. Exact `SimpleChargingDock` parameter names/defaults in the installed
   1.3.10 — verify with `ros2 param dump /docking_server` at first bench
   launch (04 §4).
4. Dock placement vs RTK quality (multipath near walls) — measure σ of
   `/odometry/global` at the candidate spot before building the base
   (01 §6).
5. Docking with an already-full battery may fail the wait-for-charge
   step (taper current below threshold) — mitigations in 04 §4.3.
