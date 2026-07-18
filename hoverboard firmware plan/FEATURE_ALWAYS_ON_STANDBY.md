# Feature Plan: Always-On Board + Motors-Off Standby Mode

**Date:** 2026-07-18
**Status:** plan only — nothing implemented.
**Related:** `HOVERBOARD_FIRMWARE_REVIEW.md` (same folder) — this feature *requires* fixing review finding 4.1 (inactivity poweroff) and makes finding 4.4 (re-enable gate) deterministic.

## 1. Goal

1. Hoverboard mainboard is on from robot boot and **stays on forever** → serial feedback (battery voltage, temperature, currents, wheel data) always available.
2. Motors **disabled by default** — no power to the wheels, only the board electronics alive.
3. Enabling mowing mission / manual drive / explicit "enable wheels" → motors run in SPD_MODE exactly as today.
4. On docking/charging → motors off again; only measurement + serial power draw remains.

## 2. What already works today (verified in code)

- **Feedback is unconditional.** The firmware sends the feedback frame every 10 ms regardless of motor state (`main.c:461-494`). Battery voltage, board temp, DC currents, wheel speeds and tick counters all remain valid with motors disabled.
- **Hall sensors are read passively.** With motors off, `speedL/R_meas` and the wheel-tick counters still work — pushing the robot by hand produces correct odometry. Useful side effect.
- **True motor-off exists in the firmware.** `enable = 0` clears the MOE (main output enable) bit on both motor timers at the 16 kHz level (`bldc.c:138-148`) → all FETs off → genuine freewheel, zero motor power. This is exactly the "off" the feature needs.

## 3. The three blockers

| # | Blocker | Where |
|---|---|---|
| 1 | `enable` is **one-shot**: armed once after boot when inputs are within ±50 (`main.c:226-234`), and never disarmed again except on motor error (`main.c:503`) or power button (`util.c:1528`). There is **no serial path** to disable or re-enable motors. | firmware |
| 2 | **Inactivity poweroff** (review 4.1): board powers itself off after 30 min with \|cmd\| ≤ 50 — kills the battery-voltage stream exactly in the "standby forever" scenario. | firmware `config.h:178`, `main.c:527` |
| 3 | **A fully-off board cannot be switched on by software.** The power latch (`OFF_PIN`) is only driven once the MCU is running; when off, the STM32 is unpowered and serial is dead. "ROS switches the board on" is literally possible only via a small hardware mod (§7); otherwise the goal is reframed as "the board simply never turns off". | hardware |

## 4. Firmware design

- **F1 — Never power off in standby.** Remove/neuter the inactivity poweroff (fixes review 4.1 at the same time). **Keep** the `BAT_DEAD` undervoltage poweroff (last-line battery protection — at 3.37 V/cell losing telemetry is the smaller problem) and **keep** the button short-press manual poweroff.
- **F2 — Motor-enable request over serial.** Two variants:
  - **Option A (recommended): add a field.** Extend `SerialCommand` (`Inc/util.h:38-43`) with e.g. `int16_t flags`; bit 0 = "motors allowed". Update the XOR checksum on both ends and the reference `Arduino/hoverserial.ino`. Clean, self-documenting, leaves room for future bits (e.g. beeper request, current-limit profile). Cost: frame length changes 8 → 10 bytes → firmware and driver must be deployed in the same window.
  - **Option B (minimal): sentinel value.** Reserve a magic value (e.g. `steer == 0x7FFF`) meaning "motors off request"; intercept in `readInputRaw()` before `calcInputCmd()`. No struct/length change, single-point patch — but hacky and no room to grow.
  - **Version-mismatch behavior (the deciding difference):**

    | Mismatch | Option A | Option B |
    |---|---|---|
    | new firmware + old driver | Command checksum never matches (field offset moved) → firmware serial-timeout failsafe → **motors stay off, safe**; telemetry (fw→Pi) unaffected since the feedback frame is unchanged | Old driver never sends the sentinel → behaves exactly like today, **safe** |
    | old firmware + new driver | Same as above: commands rejected, **motors off, safe**, telemetry works | **Dangerous:** the sentinel (32767) passes the old firmware's `calcInputCmd` clamp as a **full-scale ±1000 command** → wheels spin. Only safe if firmware is always flashed *before* the driver. |

    So A fails safe in *both* deploy orders (worst case: no motor control until the other side is updated, telemetry keeps working); B is only safe if the flash order is never violated. → recommendation: **A**.
- **F3 — Enable state machine.** New `motorsAllowed` flag driven by the serial request:
  - `motorsAllowed == 0` → force `enable = 0` immediately (MOE cleared on the next 16 kHz tick), ignore steer/speed.
  - `motorsAllowed` 0→1 → run the **existing** arming gate unchanged (no motor errors, both inputs within ±50) before setting `enable = 1`. The driver guarantees inputs are zero while disabled (D2), so arming is deterministic.
  - Serial timeout continues to behave as today (OPEN_MODE + zero command) — an extra layer under the new flag.
- **F4 — Report actual enable state in feedback, no struct change needed.** The `cmdLed` field (uint16) is effectively free in this build (no sideboard connected; only low byte defined upstream). Use e.g. bit 8 = "motors enabled", bit 9 = "motor error present". Driver decodes and publishes it.
- Decide on the **arming beeps** (`beepShort(6); beepShort(4)` in `main.c:227-228`): they will now fire on every mission start. Keep (nice audible confirmation) or silence.

## 5. Driver design (ros2-driver-converted)

- **D1 — `motors_enabled` dynamic bool parameter** on the existing `hoverboard_driver_node` (same infra as `use_pid` — the parameter callback already exists). **Default `false`** → wheels off after every boot, which is the wanted standby state. BT nodes, the web UI backend, or the OLED panel flip it via the standard SetParameters service (`ros2 param set /hoverboard_driver_node motors_enabled true`).
- **D2 — Enforce zero while disabled.** While `motors_enabled == false`, `write()` sends steer = 0, speed = 0 (+ flag bit 0 = 0), ignoring whatever diff_drive commands. This both guarantees the firmware arming gate passes the instant enabling happens, and prevents a queued cmd_vel from twitching the wheels at the moment of enable.
- **D3 — Publish actual state.** Decode the F4 status bit from feedback → publish `hoverboard/motors_enabled` (Bool, on change). BT sequence: set param → wait for topic true → start driving. This also covers review finding 4.4 (board brownout mid-mission): state drops to false, BT can zero cmd_vel and re-arm.
- **D4 — Enable policy (DECIDED 2026-07-18): explicit enable + auto-disable timer.**
  - `motors_enabled` (param, D1) is the *authority*: "motors allowed". Set true at mission start / manual drive, false at dock/idle.
  - While allowed, the driver auto-drops the firmware flag to standby after **`auto_disable_timeout` of continuous zero commands — default 120 s**, exposed as a dynamic parameter so the web-UI settings page can change it (`ros2 param set /hoverboard_driver_node auto_disable_timeout <sec>` from the web backend; same SetParameters infra as `motors_enabled`).
  - When nonzero commands resume while `motors_enabled` is still true, the driver **re-arms automatically** (re-asserts the flag; firmware arming gate passes since D2 held inputs at zero). Cost: ~one command cycle + the arming beeps.
  - A forgotten "disable" in a BT branch can therefore never leave the motors holding torque on the dock for more than 2 min.

## 6. Mission / BT / docking integration

- Mission start & manual-drive select → `motors_enabled = true`, wait for confirmation, drive.
- Dock arrival, mission end, e-stop/idle → `motors_enabled = false`.
- Charging needs nothing special: the charge port goes through the BMS to the pack; the board being on with motors disabled is exactly the wanted "measurements only" draw.
- With the voltage stream now permanently available: publish `sensor_msgs/BatteryState` and wire the low-battery return-to-dock condition (review §6.1) — this feature is the prerequisite for that.

## 7. Remote power-on — ALREADY EXISTS (relay via Arduino bridge)

**Resolved 2026-07-18:** the robot already has a relay acting as the hoverboard power button, controlled by `arduino_bridge.py` (`scripts/arduino_bridge.py`):

- `hoverBtnR1_pulse` (Bool) — one `True` = 0.2 s "button press" (`hoverBtnR1_pulse_duration`)
- `hoverBtnR1_cmd` (Bool) — steady relay state
- `hoverBtnR1_status` (Bool) — echoed state from the Arduino

**Boot sequence:** at ROS2 startup, check `hoverboard/connected` (from the driver); if `false` after a grace period → publish one `hoverBtnR1_pulse` → wait for `connected == true`. Retry with backoff. This gives literally "ROS2 switches the hoverboard on during boot" with hardware that already exists.

**Two cautions:**
1. **The pulse is a toggle, not "on".** Pressing the button while the board is ON runs `poweroffPressCheck()` → short press = power off. Never pulse blindly; always gate on `hoverboard/connected` (or add a small state machine in the BT/bringup that knows the intent).
2. **Never hold `hoverBtnR1_cmd` true for ≥5 s while the board is on.** The firmware's long-press handling (`util.c:1534-1545`) has **two** branches, both hazardous with relay control (both persist to flash at the next proper shutdown, both only require the wheels to be stationary — which describes a standby robot):
   - **Single long press → `adcCalibLim()`** (`util.c:485`, needs `AUTO_CALIBRATION_ENA`): records the live input min/mid/max for up to 20 s and adopts them as the new input mapping. With the driver streaming zeros, the recorded range is ≈[0,0] → the input gets misclassified/disabled or gets a degenerate mapping → wheels dead or absurd scaling, **persisted**. Recovery needs re-calibration or full chip erase.
   - **Long press + second short press → `updateCurSpdLim()`** (`util.c:584`): rescales `i_max` and `n_max` by where the inputs sit in their range. Serial inputs at zero = mid-range = factor 0.5 → **current limit silently halved to ~7.5 A and `n_max` halved to ~500 rpm**. The n_max change breaks the driver's "1 unit = 1 RPM" assumption (review §2 coupling 1) — every commanded speed would be off by 2×, and it survives reboots.

   **Mitigation for the fork:** compile out *both* branches for the robot build (remove `AUTO_CALIBRATION_ENA` *and* the `updateCurSpdLim()` call), and use only the pulse topic (0.2 s) — never the steady `hoverBtnR1_cmd` — for the hoverboard relay.

## 8. Caveats / behavior changes to be aware of

- **Parked behavior changes.** Today (SPD_MODE, enable=1, target 0) the FOC loop *actively holds* 0 rpm — the parked robot resists being pushed and burns power doing it. In standby the wheels freewheel: the robot rolls freely on a slope. **Decision 2026-07-18:** dock will be flat → freewheel standby is fine there. For pauses *during* a mowing mission on sloped ground: deferred idea — use the BNO085 IMU to check level before disarming mid-mission (disarm only if pitch/roll below threshold; otherwise keep motors holding). Not needed for v1; the auto-disable timer must simply be long enough that normal mission pauses don't disarm on a slope, or mid-mission disarm can be skipped entirely (disarm only at dock/idle).
- **Standby draw is not zero.** Board electronics ≈ 1-2 W (MCU, buck, LEDs, hall supplies) — and it is **not visible** in `left/right_dc_curr` (those sense the motor DC link only). Measure once with a meter to know the real off-dock standby budget. Multi-day storage off the dock will drain the pack until BAT_DEAD fires; for long storage, use the physical button.
- **Deployment coupling (Option A):** frame length changes, so flash the STM32 and deploy the driver in the same maintenance window (driver down → flash → start new driver). Option B avoids this.
- The wheels being disabled does not stop the feedback-side wheel data — tick counters keep counting if the robot is pushed/rolls, which is correct and desirable for odometry.

## 9. Implementation checklist (when approved)

**Firmware fork** (flash via st-flash/PlatformIO, ST-Link):
1. F1 inactivity-poweroff removal (= review fix 4.1)
2. F2 command frame extension — **Option A (decided)**: `flags` field, bit 0 = motors allowed
3. F3 enable state machine
4. F4 status bit in `cmdLed`
5. Compile out both button long-press branches (`AUTO_CALIBRATION_ENA` + `updateCurSpdLim()` call) — relay-safety, see §7
6. Update `Arduino/hoverserial.ino` reference + a short protocol note in the fork's README

**Driver** (`hardware/`):
1. `protocol.hpp` struct + checksum update (Option A: `flags` field)
2. D1 `motors_enabled` parameter, D2 zero-enforcement, D3 status topic, D4 auto-disable timer + `auto_disable_timeout` parameter (default 120 s)
3. Natural moment to include the §3.8 jitter bundle and the correctness fixes from the review (same files)

**BT / UI:**
1. Enable/disable calls at mission start/end, manual-drive toggle, dock arrival
2. Surface motor state + battery voltage in web UI / OLED panel
3. Web-UI settings field for `auto_disable_timeout` (backend → SetParameters on `hoverboard_driver_node`)

## 10. Additional telemetry worth adding (2026-07-18)

The BLDC model computes more per-motor data than the feedback frame carries (`ExtY` in `BLDC_controller.h:220-229`: `z_errCode`, `n_mot`, `a_elecAngle`, `iq`, `id`). Prioritized candidates:

**Tier 1 — no frame change: pack into the free `cmdLed` bits (extends F4).**
| Bits | Signal | Why |
|---|---|---|
| 0-3 | `rtY_Left.z_errCode` | Motor fault reason: 1 = hall disconnected, 2 = hall short, 4 = motor blocked. Today a fault just beeps and stops — ROS never learns why. |
| 4-7 | `rtY_Right.z_errCode` | same |
| 8 | motors-enabled state | already planned (F4) |
| 9 | `timeoutFlgSerial` | firmware's own view of the command link — complements the driver-side `connected`, distinguishes "link down" from "board off" |

Feedback frame length stays unchanged → old driver keeps parsing during rollout.

**Tier 2 — already in the frame, just unused by the driver:** `cmd1`/`cmd2` echo what the firmware *accepted*. Publishing (or comparing against what was sent) gives end-to-end command-path verification — review §6.3.

**Tier 3 — new feedback fields (frame grows; acceptable since the command frame deploy is coupled anyway): `iq` per motor. — DECIDED 2026-07-18: will be implemented (see DEPLOY_TODO.md 1.4/2.1/2.4).**
The FOC q-axis current is the *torque-producing* current — strictly better than DC-link current for stuck/stall/slip detection: at stall the DC-link current stays low (dc ≈ iq × duty, and duty is small at low speed) while `iq` shoots up. If the mower's stuck-detection is a goal, `iq` L/R is the signal to add; the DC-link currents then remain what they're good at (power consumption / energy bookkeeping).

**Not available from this board:** battery current draw of the electronics themselves (DC-link sensors see motors only), charge current, per-cell voltages (BMS territory), any second temperature (boardTemp is the STM32 die — calibration caveats in review 4.5).

**ROS-side presentation:** instead of more raw Float64 topics, aggregate into `diagnostic_msgs/DiagnosticArray` (rqt_robot_monitor / web-UI health panel) + `sensor_msgs/BatteryState` (voltage + summed motor current). Implement together with the §3.8 jitter bundle publisher rework so the new topics are born realtime-safe (RealtimePublisher, on-change or 1 Hz).

## 11. Open decisions (updated 2026-07-18 — ALL RESOLVED, plan ready for implementation)

Note on firmware flashing after removing the long-press branches: unaffected. Flashing goes over the SWD pins (ST-Link, `make flash` / PlatformIO) and only needs the board *powered* — the button's power-on role is the hardware latch circuit, not firmware code, and the normal short-press on/off path stays. The relay pulse (§7) can power the board for flashing sessions too.

1. ~~Protocol variant~~ — **RESOLVED: Option A** (flags field). Confirmed 2026-07-18; see the mismatch table in §4 F2 for the rationale.
2. ~~Enable policy~~ — **RESOLVED: explicit enable + auto-disable timer** (see §5 D4). Consequence to remember: a mission pause longer than the timeout disarms → freewheel until commands resume (auto re-arm); on sloped ground see the deferred IMU idea in §8.
3. ~~Hardware power-on mod~~ — **RESOLVED: already exists.** Relay via `arduino_bridge.py` (`hoverBtnR1_pulse`); see §7 incl. the toggle and ≥5 s-hold cautions.
4. ~~Arming beeps~~ — **RESOLVED: keep.** Note: these are the two short beeps at *motor arming* (not the power-on melody). Today arming happens once per board power-on; with this feature it fires at **every mission start / manual-drive enable** — accepted, doubles as an audible "wheels are live" warning. The power-on melody stays rare (actual power-ons only).
5. ~~Auto-disable timeout value~~ — **RESOLVED: 2 min default, configurable via web-UI settings** (dynamic parameter `auto_disable_timeout`, see §5 D4).
6. ~~Slope parking~~ — **RESOLVED:** dock will be flat → freewheel standby OK. IMU level-check before mid-mission disarm noted as deferred idea in §8.
