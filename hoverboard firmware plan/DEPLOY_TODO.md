# Hoverboard Firmware + Driver — Unified Deploy TODO

**Date:** 2026-07-18
**Sources:** `HOVERBOARD_FIRMWARE_REVIEW.md` (findings) + `FEATURE_ALWAYS_ON_STANDBY.md` (feature design) — this file is the actionable merge of both. Section references (§) point back to those docs for rationale.
**Workflow:** Phase 1 (firmware) is done and verified **first**; Phase 2 (driver) only starts after the Phase 1 gate passes. Phase 3 (BT/UI) after that.

## Decided parameters (recap — do not re-litigate)

| Item | Decision |
|---|---|
| Protocol | **Option A**: new `int16_t flags` field in `SerialCommand`, bit 0 = motors allowed (frame 8 → 10 bytes) |
| Feedback status bits (in free `cmdLed` field, frame length unchanged) | bits 0-3 = `z_errCode` left, 4-7 = `z_errCode` right, 8 = motors enabled, 9 = `timeoutFlgSerial` |
| Enable policy | explicit `motors_enabled` + auto-disable after `auto_disable_timeout` (default **120 s**, web-UI-configurable); silent auto re-arm while still "allowed" |
| Arming beeps | keep |
| Long-press button branches | remove **both** (`adcCalibLim` + `updateCurSpdLim`) |
| Buzzer-disable knob from upstream | not wanted |
| Upstream cherry-pick | `8df24d4` IRQ priorities only; **no rebase** |
| Power-on | existing relay (`hoverBtnR1_pulse`, 0.2 s) — no hardware changes |
| Tier-3 telemetry | **YES** — `iq_l`/`iq_r` added to feedback (frame 26 → 30 bytes); best stall/stuck signal |
| Driver PID path | **DELETE** — closed-loop speed control stays where it already runs: firmware SPD_MODE (16 kHz PI speed loop inside the FOC cascade). Review findings 3.3/3.4 become moot. |
| Serial-timeout beep | stays disabled (no action — already disabled in fork) |

*(All decisions resolved 2026-07-18 — nothing open.)*

---

## PHASE 1 — FIRMWARE (`/home/ros-pi/hoverboard_firmware`, STM32, flash via ST-Link)

**Status 2026-07-18: all code tasks DONE and pushed — commit `e375def` (feature) on top of `56e2ffc` (IRQ cherry-pick); rollback tag `pre-standby` = old firmware. Remaining: build (1.12), flash (1.13), gate tests (G1.x).**

Implementation notes Phase 2 must match:
- **Feedback field order** (30 bytes): `start, cmd1, cmd2, speedR_meas, speedL_meas, wheelR_cnt, wheelL_cnt, left_dc_curr, right_dc_curr, iq_l, iq_r, batVoltage, boardTemp, cmdLed, checksum` — `iq_l/iq_r` sit **between `right_dc_curr` and `batVoltage`**, not at the end.
- **Disarm-on-timeout added** (design refinement in 1.3): motors also disarm when the serial link times out (~0.8 s), not only when flags bit 0 is cleared — driver dead ⇒ full standby/freewheel instead of upstream's OPEN_MODE-with-PWM-active. Arming additionally requires the link to be alive (`timeoutFlgSerial == 0`).
- `FLASH_WRITE_KEY` bumped 0x1002 → 0x1012 (stale flash calibrations ignored on first boot of the new firmware).

### Feature work

- [x] **1.1 Remove inactivity poweroff** (= review 4.1). `main.c:527-534` + `INACTIVITY_TIMEOUT` in `config.h:178`. Keep `BAT_DEAD` undervoltage poweroff (`main.c:500`) and button short-press poweroff untouched.
- [x] **1.2 Protocol Option A.** Add `int16_t flags` to `SerialCommand` (`Inc/util.h:38-43`); include it in the command checksum (`util.c:1249`). Bit 0 = motors allowed.
- [x] **1.3 Enable state machine** (feature §4 F3). New `motorsAllowed` driven by flags bit 0: when 0 → force `enable = 0` immediately and ignore steer/speed; on 0→1 → existing arming gate unchanged (no `z_errCode`, inputs within ±50, `main.c:226`). Serial-timeout failsafe behavior unchanged underneath.
- [x] **1.4 Feedback telemetry** (feature §4 F4 + §10): status bits in `cmdLed` — bits 0-3 `rtY_Left.z_errCode`, 4-7 `rtY_Right.z_errCode`, 8 = motors enabled, 9 = `timeoutFlgSerial` — **and** append `int16_t iq_l, iq_r` (`rtY_Left/Right.iq`) to the feedback struct + checksum (frame 26 → 30 bytes).
- [x] **1.5 Remove both long-press branches** (feature §7 caution 2): delete `AUTO_CALIBRATION_ENA` (`config.h:183`) and the `updateCurSpdLim()` call in `poweroffPressCheck()` (`util.c:1539`). Short-press poweroff stays.
- [x] **1.6 Cherry-pick upstream `8df24d4`** (review §6.5): UART/DMA interrupt priorities 0→1 in `setup.c`/`control.c` so serial can never preempt the 16 kHz FOC ISR. Already fetched into the repo (`git cherry-pick 8df24d4` or apply by hand).
- [x] **1.7 Coupling guards** (review §2): `#error` in `config.h` if `N_MOT_MAX != 1000` or steer/speed coefficients differ from defaults while `VARIANT_USART` — protects the driver's "1 unit = 1 RPM" and mixer-inversion assumptions.
- [x] **1.8** ~~Serial-timeout beep~~ — resolved: stays disabled, no code change (`main.c:507-508` untouched).

### Cleanup (small, do while in there)

- [x] **1.9** `uint16_t up_down[6]` → `int16_t` (review 4.2, `bldc.c:95`).
- [x] **1.10** Fix stale calibration comments (review 4.5, `config.h:78` area).
- [x] **1.11** Update `Arduino/hoverserial.ino` reference + short protocol note (new frame layout, cmdLed bits) in the fork README.

### Build, flash, commit

- [x] **1.12** Build — done 2026-07-22 on the flashing PC (VSCode + PlatformIO). Needed `platform_packages = platformio/tool-openocd@~2.1100.0` in the VARIANT_USART env for the stlink upload; committed as `39f67ab`.
- [x] **1.13** Flash — done 2026-07-22 via VSCode+PlatformIO on the flashing PC. (Original instructions kept below for future reflashes.) Flash via ST-Link — **from the separate flashing PC** (coding happens on the Pi). Preferred: build on the Pi (`pio run` → `.pio/build/VARIANT_USART/firmware.bin`), copy only that file to the PC (`scp ros-pi@<pi-ip>:.../firmware.bin .`), then flash:
  - Linux PC (`stlink-tools`): `st-flash --reset write firmware.bin 0x8000000`
  - Windows PC (STM32CubeProgrammer): `STM32_Programmer_CLI -c port=SWD -w firmware.bin 0x8000000 -rst` (or the GUI, program at 0x08000000)
  - Alternative: clone the fork on the PC and `pio run -t upload` (PlatformIO installs its own toolchain).
  - Wiring: SWDIO, SWCLK, GND to the SWD header; board powered from its own battery (button/relay pulse); leave ST-Link 3.3 V disconnected. No chip unlock needed (board already runs custom firmware).
- [x] **1.14** Commit + push to `lepptu/hoverboard-firmware-hack-FOC` (tag the pre-change commit first for rollback, e.g. `pre-standby`).

### PHASE 1 GATE — verify before touching the driver

Interim state is **intentional**: old driver + new firmware = motors can never arm (command checksum mismatch → failsafe), robot immobile but safe.

- [x] **G1.1 PASSED 2026-07-22** — 8 consecutive frames captured on the Pi: `cd ab` markers exactly every 30 bytes, checksum valid, fields sane (batV 40.20 V, temp 36.3 °C, currents ≈0, status word = disarmed + serialTimeout as expected). Board boots and feedback streams at 100 Hz. Since the feedback frame grew to 30 bytes, the **old driver's telemetry is dead until Phase 2 — expected** (checksum rejects). Verify frames directly instead: quick serial dump on the Pi (e.g. `python3 -c "import serial;s=serial.Serial('/dev/ttyAMA0',115200);d=s.read(120);print(d.hex(' '))"` — look for repeating `cd ab` start markers spaced 30 bytes apart).
- [x] **G1.2 PASSED 2026-07-22** — old driver rejects every frame (~100 checksum warns/s in journal), status word bit 8 = 0 (disarmed), no arming beeps. Motors cannot arm with the old driver. NOTE: journald WARN spam continues until Phase 2 — stop `mowbot-launch-bringup` or power the board off between test sessions if it bothers.
- [ ] **G1.3** Idle >35 min → board **stays on** (inactivity poweroff gone). *User will verify manually at some point — just note whether the board is still on/streaming after >35 min of idle. Quick check: `journalctl -u mowbot-launch-bringup.service --since "1 minute ago" | grep -c "checksum mismatch"` — nonzero = still streaming (while the old driver is running).*
- [x] **G1.4 PASSED 2026-07-22** — button short press powers off, next press powers back on; wheels stay freewheeling after power-on (disarmed-by-default standby confirmed).
- [x] **G1.5 PASSED 2026-07-22** — relay pulse via `/hoverBtnR1_pulse` powered the board on from off; feedback started streaming within seconds.
- [x] **G1.6 PASSED 2026-07-22** — >5 s hold does nothing; calibration/limit long-press branches confirmed gone.

---

## PHASE 2 — DRIVER (`/home/ros-pi/pi_ws/src/ros2-driver-converted`, package `hoverboard_driver`)

**Status 2026-07-23: all code tasks DONE, built (`--symlink-install`), deployed and verified on hardware — commit `5f65e5d`, rollback tag `pre-standby-driver`. An adversarial multi-agent review (16 agents) ran before the build: 13 findings, 11 confirmed, all fixed or accepted-with-rationale. Notable extra fixes beyond the plan: right-wheel POSITION sign removed (firmware already mirror-compensates odom_r — the historical negation made position oppose velocity), velocities zeroed on link loss (no phantom odometry), disarm frame on deactivate, curvature-preserving wheel-speed scaling, executor startup-race guard. Accepted w/o fix: 0→cruise step after brownout re-arm (firmware rate limiter smooths ~100 ms), restart-detection false-positive window (~1.5e-6/gap).**

**Live verification 2026-07-23 (G2.1 PASSED + more):** driver activates with "Motors start disarmed (standby)" message; board powered via relay; `connected: true`, battery 41.29 V, temp 36.7 °C, errors 0, iq 0.0, `motors_enabled: false`, **`firmware_serial_timeout: false` — proves the firmware accepts the new 10-byte commands (both protocol directions verified without spinning a wheel)**; zero checksum warnings, zero overrun warnings in the startup window; latched status topics deliver instantly.

### Protocol + feature work

- [x] **2.1 `protocol.hpp`:** add `flags` to `SerialCommand` (before `checksum`) and `iq_l`/`iq_r` to `SerialFeedback` (**between `right_dc_curr` and `batVoltage`** — see Phase 1 implementation notes for the authoritative field order); update both checksum computations (`hoverboard_driver.cpp:412,526`); add `static_assert(sizeof(SerialCommand) == 10)` / `static_assert(sizeof(SerialFeedback) == 30)`.
- [x] **2.2 D1:** `motors_enabled` dynamic bool parameter, **default false**, on `hoverboard_driver_node` (same pattern as `use_pid`).
- [x] **2.3 D2:** while disabled → send steer = 0, speed = 0, flag bit 0 = 0, ignoring diff_drive commands.
- [x] **2.4 D3:** decode `cmdLed` bits → publish `hoverboard/motors_enabled` (Bool, on change), motor error codes (per wheel), firmware link-timeout flag; publish `iq` per wheel (A, scaling: raw fixdt / `A2BIT_CONV` = 50 per amp — verify on bench).
- [x] **2.5 D4:** auto-disable timer — drop the flag after `auto_disable_timeout` (dynamic param, default 120.0 s) of continuous zero commands; silently re-assert on next nonzero command while `motors_enabled` still true.

### Correctness fixes (review §3 — same files, do in the same pass)

- [x] **2.6** Right-encoder wrap fix (3.1): unwrap **before** negating (`hoverboard_driver.cpp:442,541`). Also board-restart check on raw wrapped counts, not unwrapped positions (3.2, line 564).
- [x] **2.7** **Delete the PID path** (decided): remove `pid.cpp`/`pid.hpp`, the `pids[2]` members, the `use_pid`/`f`/`p`/`i`/`d`/`i_clamp_*`/`antiwindup` parameters and their callback branches, the `setParameters` calls in `write()`, and the `control_toolbox` dependency from CMakeLists. `write()` keeps only the direct rad/s → RPM path (+ ±1000 clamp from 2.10). Review findings 3.3/3.4 are resolved by deletion. Closed-loop speed control remains in firmware SPD_MODE.
- [x] **2.8** `max_velocity /= wheel_radius` out of `on_activate` → `on_init` (3.5).
- [x] **2.9** `exit(-1)` → `return CallbackReturn::ERROR` on serial-open failure (3.6).
- [x] **2.10** Smaller items (3.7): function-statics in `on_encoder_update` → members; `connected` only counts checksum-valid frames; guard `std::stod` on missing hardware params; handle short serial writes; remove `prev_msg` + misleading `DEFAULT_PORT`; clamp speed/steer to ±1000 before the int16 casts.

### Jitter bundle (review §3.8 — fixes the known ros2_control jitter warnings)

- [x] **2.11** Internal node onto its own executor thread; delete `rclcpp::spin_some` from `read()` (mutex/atomics for `pid_config`).
- [x] **2.12** Debug publishers → `realtime_tools::RealtimePublisher` and/or param-gated at ≤10 Hz; `connected` on change only.
- [x] **2.13** Chunked serial reads (buffer, then feed `protocol_recv`).
- [x] **2.14** `RCLCPP_WARN_THROTTLE` for checksum mismatches.

### Optional (can defer to Phase 3)

- [ ] **2.15** `sensor_msgs/BatteryState` (voltage + summed current + percentage) and/or `DiagnosticArray` aggregation (review §6.1, feature §10).

### Build + deploy (per repo build rules)

- [x] **2.16** `cd /home/ros-pi/pi_ws && colcon build --symlink-install --packages-select hoverboard_driver` (one package, one build per command; **`--symlink-install` per user decision 2026-07-22**), then restart the ROS2 systemd units (`mowbot-launch-bringup` — user stopped it for the Phase 2 work).
- [x] **2.17** Commit + push driver repo (tag pre-change state for rollback).

### PHASE 2 GATE — end-to-end verification

- [x] **G2.1 PASSED 2026-07-23** — see live verification above.
- [x] **G2.2 PASSED 2026-07-24 (user)** — param set true → beep, holding torque, drives from web-UI.
- [x] **G2.3 PASSED 2026-07-24** — 2-min auto-disable verified by user. Found+fixed (`0319989`): re-setting `motors_enabled true` after an auto-disable did not re-arm (param was still true → no edge); now any set-to-true arms — verified live with a 5 s test timeout. *Silent re-arm by just driving (without touching the param) is designed in but still unexercised — try it once during the odometry drive.*
- [x] **G2.4 PASSED 2026-07-24 (user)** — disable while driving → immediate freewheel.
- [ ] **G2.5** Odometry: drive >61 m (or bench-spin a wheel >100 revs) → **no position jump** on the right wheel (`hoverboard/right_wheel/position` continuous) — proves 2.6.
- [ ] **G2.6** Jitter: controller_manager warnings gone/greatly reduced over a >30 min session.
- [ ] **G2.7** Telemetry: battery voltage/temp/currents update at all times, motors enabled or not; unplug a hall connector briefly (bench!) → error code appears on the new error topic.
- [ ] **G2.8** Kill the driver mid-drive → motors stop within ~1 s (firmware failsafe unchanged).

---

## PHASE 3 — BT / UI / INTEGRATION

### Manual-drive arming design (added 2026-07-24)

How `motors_enabled true` comes about when the user powers on and wants to drive manually — grounded in the actual cmd_vel flow: web drive pad → MQTT bridge → `cmd_vel_web` → twist_mux (joystick 100 > web 75 > foxglove 50 > nav 10, all behind the F31 e-stop lock) → diff_drive → driver.

**Recommended (hybrid): one explicit tap per session, machinery does the rest.**
- Boot default stays `motors_enabled: false` (standby, telemetry only).
- The web UI drive/settings page gets a prominent **"Motors armed" toggle**; its state mirrors the latched `hoverboard/motors_enabled` topic. One tap → beep → armed; after that the already-verified silent re-arm / 2-min auto-disable machinery handles everything (drive whenever, standby when idle).
- Joystick: map a button to the same arm/disarm action.
- Missions: BT sets true at mission start, false at dock arrival / mission end (3.2) — no user action.
- **Plumbing:** add a small `arm_request` **Bool topic** mapped to the parameter (in the driver's helper node or a tiny standalone node). Topics are trivial to publish from the MQTT bridge, a joystick mapping, and the BT alike; parameter-service calls are clumsy to bridge. All arming paths then converge on one mechanism.

**Considered alternative ("standing permission"):** set `motors_enabled true` once after boot and leave it — boot arms once (2 min holding torque), then standby/silent-re-arm makes manual driving "just work" with zero new UI. Viable because every twist_mux source is deliberate, deadman'd (0.5 s), and behind the e-stop lock; rejected-by-default because any active source (incl. nav with an accidentally running mission) would move the robot, and each boot/session ends with 2 min of holding torque. Can be revisited if the toggle feels like friction in practice.

**Open decision for Phase 3 implementation:** hybrid (recommended) vs standing permission. Everything else in this section is agnostic to that choice.

- [ ] **3.1** Boot orchestration: if `hoverboard/connected` false after grace period → single `hoverBtnR1_pulse` → wait for connected (never pulse blindly — it's a toggle; feature §7).
- [ ] **3.2** BT: `motors_enabled true` at mission start (wait for `hoverboard/motors_enabled` true before driving); `false` at dock arrival / mission end / idle. Manual-drive arming per the design block above.
- [ ] **3.2b** `arm_request` Bool topic → parameter mapping (the plumbing from the design block); web-UI "Motors armed" toggle via MQTT bridge; joystick arm/disarm button.
- [ ] **3.3** Web-UI settings field for `auto_disable_timeout` (backend → SetParameters on `hoverboard_driver_node`).
- [ ] **3.4** Surface in web UI / OLED panel: motor state, battery voltage (+ percentage if 2.15 done), motor error codes.
- [ ] **3.4b** Web UI: show per-wheel pushing **force in newtons** alongside iq — display-layer only: `N = k × iq_current` (driver topics stay in amps). `k` [N/A] is a web-UI setting, default ≈ 6 N/A until calibrated. **Calibration (once):** tether the robot to a luggage/fish scale, arm motors, command a slow speed so the wheels stall against the tether, read scale (kg × 9.81 = N) and the summed `hoverboard/*/iq_current` at the same moment → k = N / A. Stall is the most accurate regime for this (no back-EMF/speed losses), and the single constant absorbs all firmware unit conventions (peak/RMS, A2BIT_CONV). Repeat at two pull strengths to confirm linearity.
- [ ] **3.5** Low-battery return-to-dock condition in BT (needs the always-on telemetry from Phase 1).
- [ ] **Deferred ideas:** IMU level-check before mid-mission disarm (feature §8); command echo verification via `cmd1/cmd2` (review §6.3).

## Rollback notes

- Firmware: reflash the tagged pre-change build (`git checkout pre-standby && make && make flash`). Old firmware + new driver = motors never arm (fail-safe both ways — that's why Option A).
- Driver: rebuild from the pre-change tag; old driver + new firmware = no motor control **and no telemetry** (feedback frame grew to 30 bytes) — the interim state is bench-only, verify feedback with the serial dump from G1.1.
- Both docs this file was merged from stay authoritative for *rationale*; this file is authoritative for *sequence and completion state* — tick the boxes here.
