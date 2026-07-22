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
- [ ] **G1.3** Idle >35 min → board **stays on** (inactivity poweroff gone). *In progress 2026-07-22 — board powered on and left idle; timed re-check scheduled.*
- [ ] **G1.4** Button short press still powers off; second press/relay pulse powers on.
- [x] **G1.5 PASSED 2026-07-22** — relay pulse via `/hoverBtnR1_pulse` powered the board on from off; feedback started streaming within seconds.
- [ ] **G1.6** Hold button >5 s → nothing happens except (possibly) poweroff path — calibration/limit branches gone.

---

## PHASE 2 — DRIVER (`/home/ros-pi/pi_ws/src/ros2-driver-converted`, package `hoverboard_driver`)

### Protocol + feature work

- [ ] **2.1 `protocol.hpp`:** add `flags` to `SerialCommand` (before `checksum`) and `iq_l`/`iq_r` to `SerialFeedback` (**between `right_dc_curr` and `batVoltage`** — see Phase 1 implementation notes for the authoritative field order); update both checksum computations (`hoverboard_driver.cpp:412,526`); add `static_assert(sizeof(SerialCommand) == 10)` / `static_assert(sizeof(SerialFeedback) == 30)`.
- [ ] **2.2 D1:** `motors_enabled` dynamic bool parameter, **default false**, on `hoverboard_driver_node` (same pattern as `use_pid`).
- [ ] **2.3 D2:** while disabled → send steer = 0, speed = 0, flag bit 0 = 0, ignoring diff_drive commands.
- [ ] **2.4 D3:** decode `cmdLed` bits → publish `hoverboard/motors_enabled` (Bool, on change), motor error codes (per wheel), firmware link-timeout flag; publish `iq` per wheel (A, scaling: raw fixdt / `A2BIT_CONV` = 50 per amp — verify on bench).
- [ ] **2.5 D4:** auto-disable timer — drop the flag after `auto_disable_timeout` (dynamic param, default 120.0 s) of continuous zero commands; silently re-assert on next nonzero command while `motors_enabled` still true.

### Correctness fixes (review §3 — same files, do in the same pass)

- [ ] **2.6** Right-encoder wrap fix (3.1): unwrap **before** negating (`hoverboard_driver.cpp:442,541`). Also board-restart check on raw wrapped counts, not unwrapped positions (3.2, line 564).
- [ ] **2.7** **Delete the PID path** (decided): remove `pid.cpp`/`pid.hpp`, the `pids[2]` members, the `use_pid`/`f`/`p`/`i`/`d`/`i_clamp_*`/`antiwindup` parameters and their callback branches, the `setParameters` calls in `write()`, and the `control_toolbox` dependency from CMakeLists. `write()` keeps only the direct rad/s → RPM path (+ ±1000 clamp from 2.10). Review findings 3.3/3.4 are resolved by deletion. Closed-loop speed control remains in firmware SPD_MODE.
- [ ] **2.8** `max_velocity /= wheel_radius` out of `on_activate` → `on_init` (3.5).
- [ ] **2.9** `exit(-1)` → `return CallbackReturn::ERROR` on serial-open failure (3.6).
- [ ] **2.10** Smaller items (3.7): function-statics in `on_encoder_update` → members; `connected` only counts checksum-valid frames; guard `std::stod` on missing hardware params; handle short serial writes; remove `prev_msg` + misleading `DEFAULT_PORT`; clamp speed/steer to ±1000 before the int16 casts.

### Jitter bundle (review §3.8 — fixes the known ros2_control jitter warnings)

- [ ] **2.11** Internal node onto its own executor thread; delete `rclcpp::spin_some` from `read()` (mutex/atomics for `pid_config`).
- [ ] **2.12** Debug publishers → `realtime_tools::RealtimePublisher` and/or param-gated at ≤10 Hz; `connected` on change only.
- [ ] **2.13** Chunked serial reads (buffer, then feed `protocol_recv`).
- [ ] **2.14** `RCLCPP_WARN_THROTTLE` for checksum mismatches.

### Optional (can defer to Phase 3)

- [ ] **2.15** `sensor_msgs/BatteryState` (voltage + summed current + percentage) and/or `DiagnosticArray` aggregation (review §6.1, feature §10).

### Build + deploy (per repo build rules)

- [ ] **2.16** `cd /home/ros-pi/pi_ws && colcon build --packages-select hoverboard_driver` (one package, one build per command), then restart the ROS2 systemd units.
- [ ] **2.17** Commit + push driver repo (tag pre-change state for rollback).

### PHASE 2 GATE — end-to-end verification

- [ ] **G2.1** Boot: board powered (relay orchestration or already on), feedback flowing, `motors_enabled` false, **no arming beeps**, wheels freewheel.
- [ ] **G2.2** `ros2 param set /hoverboard_driver_node motors_enabled true` → arming beeps, `hoverboard/motors_enabled` → true, wheels drive on `/cmd_vel`.
- [ ] **G2.3** Stop commanding for >2 min → auto-disable (freewheel, status false); nonzero command → silent re-arm + drives.
- [ ] **G2.4** `motors_enabled false` → immediate freewheel even while commands stream.
- [ ] **G2.5** Odometry: drive >61 m (or bench-spin a wheel >100 revs) → **no position jump** on the right wheel (`hoverboard/right_wheel/position` continuous) — proves 2.6.
- [ ] **G2.6** Jitter: controller_manager warnings gone/greatly reduced over a >30 min session.
- [ ] **G2.7** Telemetry: battery voltage/temp/currents update at all times, motors enabled or not; unplug a hall connector briefly (bench!) → error code appears on the new error topic.
- [ ] **G2.8** Kill the driver mid-drive → motors stop within ~1 s (firmware failsafe unchanged).

---

## PHASE 3 — BT / UI / INTEGRATION

- [ ] **3.1** Boot orchestration: if `hoverboard/connected` false after grace period → single `hoverBtnR1_pulse` → wait for connected (never pulse blindly — it's a toggle; feature §7).
- [ ] **3.2** BT: `motors_enabled true` at mission start / manual-drive select (wait for `hoverboard/motors_enabled` true before driving); `false` at dock arrival / mission end / idle.
- [ ] **3.3** Web-UI settings field for `auto_disable_timeout` (backend → SetParameters on `hoverboard_driver_node`).
- [ ] **3.4** Surface in web UI / OLED panel: motor state, battery voltage (+ percentage if 2.15 done), motor error codes.
- [ ] **3.5** Low-battery return-to-dock condition in BT (needs the always-on telemetry from Phase 1).
- [ ] **Deferred ideas:** IMU level-check before mid-mission disarm (feature §8); command echo verification via `cmd1/cmd2` (review §6.3).

## Rollback notes

- Firmware: reflash the tagged pre-change build (`git checkout pre-standby && make && make flash`). Old firmware + new driver = motors never arm (fail-safe both ways — that's why Option A).
- Driver: rebuild from the pre-change tag; old driver + new firmware = no motor control **and no telemetry** (feedback frame grew to 30 bytes) — the interim state is bench-only, verify feedback with the serial dump from G1.1.
- Both docs this file was merged from stay authoritative for *rationale*; this file is authoritative for *sequence and completion state* — tick the boxes here.
