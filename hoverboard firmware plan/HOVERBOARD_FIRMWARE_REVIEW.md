# Hoverboard Firmware + ROS2 Driver Review

**Date:** 2026-07-18
**Reviewed:**
- Firmware: `/home/ros-pi/hoverboard_firmware` (fork of EFeru hoverboard-firmware-hack-FOC, remote `lepptu/hoverboard-firmware-hack-FOC`, HEAD `1fb5689`)
- Driver: `/home/ros-pi/pi_ws/src/ros2-driver-converted` — only the hoverboard hardware-interface part (`hardware/`, `description/ros2_control/`, `bringup/config/hoverboard_controllers.yaml`)

**Scope:** review only, no code changes made.

---

## 1. What the fork changes (baseline understanding)

Local commits on top of upstream + the robaka "wheelcount" merge:

| Commit | Change |
|---|---|
| `2cfb9bb`, `24955d6` | Control + feedback moved from USART2 → USART3 (`CONTROL_SERIAL_USART3 0`, `FEEDBACK_SERIAL_USART3`), debug ASCII output on USART2, USART3 feedback checksum fixed to include wheel counts |
| `1fb5689` | `left_dc_curr` / `right_dc_curr` added to the feedback frame (and to the checksum on both UART branches) |
| `7679262` | platformio `monitor_port` → `/dev/ttyUSB0` |
| wheelcount merge | `odom_l`/`odom_r` hall-tick counters in `bldc.c`, reported modulo 9000 in `wheelR_cnt`/`wheelL_cnt` |

Active configuration: `VARIANT_USART`, FOC + **SPD_MODE** (closed-loop speed in firmware), `N_MOT_MAX 1000`, `I_MOT_MAX 15 A`, `I_DC_MAX 17 A`, field weakening off. Makefile build (no VARIANT env) and PlatformIO build (`default_envs = VARIANT_USART`) now produce the same configuration — consistent. ✔

## 2. Protocol compatibility — verified OK ✔

Checked firmware ↔ driver byte-for-byte:

- **Feedback struct** (13 × 16-bit = 26 bytes): field order in `Src/main.c:123-137` matches `hardware/include/hoverboard_driver/protocol.hpp:17-31` exactly, including the two new current fields. No padding on either side (all members 16-bit). ✔
- **Checksum**: firmware USART3 branch (`main.c:487`) XORs all 11 payload fields; driver (`hoverboard_driver.cpp:412-423`) computes the identical XOR. ✔ (This was the `24955d6` fix — before it the USART3 checksum omitted wheel counts.)
- **Command struct** `{start, steer, speed, checksum}` matches `usart_process_command()` in `Src/util.c:1246-1264`. ✔
- **Units**: `batVoltage`/100 → V ✔, `boardTemp`/10 → °C ✔, `*_dc_curr`/100 → A ✔, `speed*_meas` RPM × 0.10472 → rad/s ✔.
- **Failsafe**: firmware zeroes commands and drops to OPEN_MODE 0.8 s after last valid command frame (`SERIAL_TIMEOUT 160`, `util.c:1005-1011`). Driver writes at 50 Hz → ample margin. Together with diff_drive `cmd_vel_timeout: 1.5` there are two independent safety layers. ✔

### Two "it works by coincidence" couplings — document them, or pin them down

1. **RPM scaling only works because `N_MOT_MAX == 1000`.** In SPD_MODE the firmware maps input ±1000 → ±`N_MOT_MAX` RPM (`BLDC_controller.c:1890-1915`). The driver converts rad/s → RPM and sends that number raw (`hoverboard_driver.cpp:509-510`), i.e. it assumes 1 unit = 1 RPM. True today only because `N_MOT_MAX = 1000`. If `N_MOT_MAX` is ever changed, every velocity in ROS silently scales.
2. **Per-wheel command reconstruction only works because `STEER_COEFFICIENT = 0.5` and `SPEED_COEFFICIENT = 1.0`** (the defaults, `config.h:193-194`). The driver sends `speed=(L+R)/2`, `steer=L−R`; the firmware mixer (`util.c:1648-1665`) computes `cmdL = speed + 0.5·steer = L`, `cmdR = speed − 0.5·steer = R`. Change either coefficient and left/right wheel speeds get cross-coupled.

**Recommendation:** add a comment in both repos stating these invariants (or `#error` guards in the fork's `config.h` if `N_MOT_MAX != 1000` / coefficients differ from defaults while `VARIANT_USART` is set).

---

## 3. Bugs — ROS2 driver

### 3.1 Right-wheel encoder wrap handling is broken (currently masked) — **highest priority**

`hoverboard_driver.cpp:442` negates the right count **before** unwrapping:

```
on_encoder_update(time, -msg.wheelR_cnt, msg.wheelL_cnt);
```

`wheelR_cnt` is in [0, 8999] (firmware modulo 9000), so `right` is in **[−8999, 0]**. The wrap detector (`hoverboard_driver.cpp:541-546`) uses `low_wrap = 2700`, `high_wrap = 6300`, which assume the value lives in [0, 9000). A negative value is *always* `< low_wrap` and *never* `> high_wrap`, so `multR` never increments or decrements. Result: `posR` is a sawtooth — every time the firmware counter crosses 0 (every 9000 ticks = 100 wheel revolutions ≈ **61 m** of travel with the 0.0975 m wheels), the right wheel position jumps by ∓8999 ticks (≈ ∓61 m). The left wheel (not negated) unwraps correctly.

**Why you haven't seen it:** `hoverboard_controllers.yaml` sets `position_feedback: false`, so diff_drive_controller computes /odom from the *velocity* interface. The corruption currently only reaches `/joint_states` (wheel angle in RViz/TF) and the debug `hoverboard/right_wheel/position` topic.

**Why it still matters:** if `position_feedback` is ever flipped to `true` (it is the diff_drive default!), odometry teleports ~61 m every 61 m of driving. It also silently invalidates any future use of wheel position (e.g. docking approach distance tracking).

**Fix (when you get to it):** unwrap first, negate after — e.g. pass `msg.wheelR_cnt` into the wrap logic and use `-posR` at the end, or feed `(9000 - msg.wheelR_cnt) % 9000`.

### 3.2 Board-restart compensation stops working after the first wrap

`hoverboard_driver.cpp:564`: restart detection requires `abs(posL) < 5 && abs(posR) < 5`. But `posL/posR` include the `mult × 9000` unwrap offset, so after >61 m of cumulative travel (`multL ≥ 1`) the condition can never be true and a mid-mission firmware power-cycle produces a position jump instead of being absorbed. (Also masked today by `position_feedback: false`; same latent-severity as 3.1. The check should compare the *raw* wrapped counts, not the unwrapped positions.)

### 3.3 `PID::setOutputLimits()` arguments swapped

`hoverboard_driver.cpp:307/312` call `setOutputLimits(-max_velocity, max_velocity)` but the signature is `setOutputLimits(double output_max, double output_min)` (`pid.hpp` / `pid.cpp:86`). So `out_max_ = -max_velocity`, `out_min_ = +max_velocity` — inverted.

### 3.4 PID output clamp is disabled — and the int16 cast can wrap

`pid.cpp:59`: `//output = clamp(output, out_min_, out_max_);` — the output limit is never applied (which is also why bug 3.3 has gone unnoticed). With `use_pid: true`, a large error + feed-forward can produce outputs way beyond ±1000; `(int16_t)steer` / `(int16_t)speed` (`hoverboard_driver.cpp:524-525`) is undefined behaviour for out-of-range doubles and in practice can wrap to a large value of the *opposite sign* — i.e. a runaway-reverse command. The firmware clamps inputs to ±1000 (`calcInputCmd`, `util.c:779-797`), but a wrapped value like −29536 clamps to −1000 = full reverse. **If PID mode is ever enabled, fix 3.3 + re-enable the clamp first.** Today `use_pid` defaults to `false` and nothing in bringup sets it, so this is dormant.

- Related nit: `pid.cpp:105` logs the *upper* limit in the lower-clamp branch.

### 3.5 `max_velocity` shrinks on every re-activation

`hoverboard_driver.cpp:293`: `max_velocity /= wheel_radius;` runs in `on_activate()` and mutates the member. A deactivate → activate cycle (lifecycle transition, controller_manager restart of the interface) divides again: 1.0 → 10.26 → 105.2 rad/s… The PID limits are re-derived from it each time. Convert once in `on_init()` or use a local.

### 3.6 `exit(-1)` on serial-open failure kills the whole controller_manager

`hoverboard_driver.cpp:314-318`. Should `return CallbackReturn::ERROR` so ros2_control can report/retry instead of taking down every controller (including any future ones unrelated to the base).

### 3.7 Smaller items

- **Function-static state in `on_encoder_update`** (`lastPosL/lastPosR/lastPubPosL/nodeStartFlag`, lines 557-559): survives deactivate/reactivate while the member state (`multR`, `last_wheelcount*`) is reset — inconsistent unwrap after a restart; also prevents ever running two instances. Should be members.
- **`hoverboard/connected` is not "protocol OK"**: `last_read` updates when *any* bytes arrive (`hoverboard_driver.cpp:369-370`), even if every frame fails checksum. Consider updating it only on valid frames.
- `on_init()` does `std::stod(info_.hardware_parameters["wheel_radius"])` etc. without checking presence — a missing xacro param throws an uncaught exception. Same for `device`.
- `write()` ignores a short write (`rc < sizeof(command)` not handled), only `rc < 0`.
- `direction_correction` is always 1 (never parameterised) and `prev_msg` is unused — dead code.
- `config.hpp` `DEFAULT_PORT "/dev/ttyAMA2"` is unused and misleading; the real device comes from the xacro (`/dev/ttyAMA0`).
- Byte-at-a-time `::read()` syscalls (up to 1024/cycle) — see 3.8.
- `rclcpp::spin_some()` + publishers inside the ros2_control read/write path — see 3.8, this is the likely cause of the observed jitter warnings.
- No clamp of `speed`/`steer` to ±1000 before the cast even in non-PID mode. Currently safe (1.0 m/s ≈ 98 RPM; spin turn ≈ ±240 steer), but a one-line `CLAMP` would make the write path robust regardless of upstream limits.

### 3.8 Control-loop jitter — explains the ros2_control overrun/jitter warnings

The driver does substantial non-realtime work inside the 50 Hz `read()`/`write()` cycle. This is the classic cause of controller_manager "loop took too long" / high-jitter warnings, which this robot is known to produce. Mechanisms, in likely order of impact:

1. **`rclcpp::spin_some(hardware_publisher)` inside `read()`** (`hoverboard_driver.cpp:351`). Every control cycle runs an executor pass on the internal node — parameter service requests, discovery traffic, whatever happens to be queued. Its duration is unpredictable by nature, which is exactly what jitter is.
2. **Plain (non-realtime) publishers firing from the control loop.** Each valid feedback frame (100 Hz → two per control cycle) triggers 8 `publish()` calls (velocity ×2, position ×2, current ×2, voltage, temperature), plus `connected` and `cmd` ×2 once per cycle — roughly **19 DDS publishes per 20 ms cycle**, each doing serialization and rmw work with occasional slow outliers. ros2_control hardware interfaces normally use `realtime_tools::RealtimePublisher` (try-lock in the RT path, actual publish on a helper thread) precisely to avoid this.
3. **Byte-at-a-time serial reads** (`hoverboard_driver.cpp:366`): ~45 `::read()` syscalls per cycle instead of 1-2 chunked reads. Smaller, but constant overhead on every cycle.
4. **Bursty console logging**: on serial resync a run of `RCLCPP_WARN` checksum-mismatch messages (`hoverboard_driver.cpp:446`) can fire back-to-back; console logging in the control loop causes visible latency spikes. If the jitter warnings come in bursts rather than steadily, this is the telltale.

**Mitigations (the "jitter bundle"):**
- Move the internal node onto its own executor thread and delete `spin_some` from `read()`. The parameter callback only writes a few doubles into `pid_config` — a mutex or `std::atomic` members suffice.
- Switch the debug topics to `realtime_tools::RealtimePublisher`, and/or gate them behind a parameter at a lower rate (10 Hz is plenty for debugging). Publish `connected` only on change.
- Read the serial port in chunks into a buffer, then feed `protocol_recv()` from it.
- Throttle the checksum-mismatch warning (`RCLCPP_WARN_THROTTLE`).

**Caveat:** some residual jitter may not be the driver's fault — a non-`PREEMPT_RT` Pi kernel with controller_manager at default scheduling priority produces occasional overruns even with a perfect hardware interface. The items above are the part fixable in this code; if warnings persist afterwards, look at kernel/priority (`SCHED_FIFO` for controller_manager, isolated core) rather than the driver.

None of the correctness bugs (3.1-3.6) affect timing — jitter is purely this section.

---

## 4. Bugs / risks — firmware

### 4.1 Inactivity auto-poweroff can fire during slow mowing — **operationally the most important finding**

`config.h:178` `INACTIVITY_TIMEOUT 30` + `main.c:527-534`: the board powers itself off after 30 min during which `|cmdL|` and `|cmdR|` never exceed **50**. In SPD_MODE units, 50 = 50 RPM ≈ **0.51 m/s** with the 0.0975 m wheels. A mower creeping along at ≤0.5 m/s for a long slow mission — or sitting docked/idle streaming zero commands — is "inactive" by this definition, and the board will hard power-off mid-operation. Recovery requires the physical power button. Turns help (outer wheel spikes above 50), which is probably why this hasn't bitten yet.

**Recommendation:** in the fork, either raise `INACTIVITY_TIMEOUT` a lot, lower the 50-count threshold (e.g. treat any nonzero command — or any valid serial traffic — as activity), or gate the poweroff on serial-timeout as well. This also interacts with the docking plan: a docked robot will lose its motor board after 30 min unless this is changed.

### 4.2 `up_or_down()` stores negative values in a `uint16_t` array

`bldc.c:95`: `uint16_t up_down[6] = {0,-1,-2,0,2,1};` returned through `int16_t`. Works on GCC/ARM via wraparound, but it is an implementation-defined conversion and genuinely confusing. Should be `int16_t`. Also note index 3 (a 3-step hall jump, direction-ambiguous) maps to 0 — hall glitches silently drop up to 3 ticks. Harmless at 16 kHz sampling vs ≤1500 steps/s, just know odometry ticks are "best effort" under electrical noise.

### 4.3 Serial-timeout beep disabled

`main.c:507-508`: the 3-beep warning for serial timeout is commented out. Presumably intentional (annoying during Pi reboots), but it removes the only local audible cue that the Pi↔board link died while the failsafe is holding the motors at zero. Consider re-enabling for field debugging, or making it conditional.

### 4.4 Motor-enable gate after firmware restart

`main.c:226`: motors enable only when both inputs are within ±50 *after* boot. If the board browns out / restarts mid-mission while nav is commanding ≥50 (≥0.5 m/s), the motors stay disabled until commands drop near zero. The driver publishes `hoverboard/connected`; the BT/mission layer should command zero velocity on a false→true transition to guarantee re-enable. Worth a note in the BT review docs.

### 4.5 Notes / cosmetics

- **Stale calibration comments**: `config.h:78` says "In this case 43.00 V" but the calibrated value is 3970 (39.70 V), `BAT_CALIB_ADC 1492`. Values look genuinely calibrated for this board; just fix the comments so future-you trusts them.
- **Wheelcount commit message** ("overflows after 10 complete wheelturns") is wrong: 9000 ticks / 90 ticks-per-rev = **100 revolutions ≈ 61 m** between wraps. `TICKS_PER_ROTATION 90` (15 pole pairs × 6 hall states) in the driver matches the hardware. ✔
- **`AUTO_CALIBRATION_ENA`** (`config.h:183`) + `FLASH_WRITE_KEY 0x1002`: a >5 s power-button hold enters input auto-calibration and *persists* new input limits to flash — on a serial-controlled robot this can only skew the ±1000 mapping. Consider commenting it out in the fork.
- **DEBUG on USART2** (`config.h:253`): ASCII printf every 125 ms on the left sensor cable. Harmless, occasionally useful; disable if the left cable ever gets repurposed (buttons/sideboard) since the config validation will then error anyway.
- The XOR checksum is weak (misses swapped words, paired bit errors). Fine at 100 Hz with the 0.8 s failsafe behind it; CRC-8/16 would be an upgrade if you ever touch the protocol again.
- DC-current sign convention (`main.c:437-438` negates `i_DCLink`): verify on the bench which sign means "drawing power" before using the current topics for stall detection logic.

---

## 5. System-level observations (config interplay)

| Item | State | Note |
|---|---|---|
| controller_manager `update_rate` | 50 Hz | Feedback arrives at 100 Hz; driver drains up to 1024 bytes/cycle → processes ~2 frames per update, keeps the latest. Fine. |
| `position_feedback` | `false` | /odom built from velocity states — this is what masks driver bugs 3.1/3.2. If you ever want position-based odom (less noise at low speed), fix 3.1/3.2 first. |
| `open_loop` | `false` | ✔ uses measured wheel velocity. |
| Velocity resolution | 1 RPM ≈ 0.01 m/s | `speed*_meas` is integer RPM; at mowing speed (0.3 m/s ≈ 29 RPM) quantization is ~3 %. EKF smooths it. Idea: derive velocity from wheel-tick deltas instead for finer low-speed resolution. |
| `use_pid` | `false` (default, unset anywhere) | Correct choice: the firmware's 16 kHz FOC speed loop closes the loop; a 50 Hz ROS PID on top would mostly fight the firmware's rate-limiter (`RATE 480`) and low-pass (`FILTER 0.1`, ~50 ms). Keep off; consider deleting the PID path to simplify (after fixing 3.3/3.4 or as part of removal). |
| `wheel_radius` | 0.0975 in **two places** | `hoverboard_driver.ros2_control.xacro:9` and `hoverboard_controllers.yaml:18` — keep them in sync (only the yaml one affects odometry; the xacro one affects the unused-in-practice `max_velocity` conversion). |
| max speed budget | 1.0 m/s ≈ 98 RPM | Far below `N_MOT_MAX 1000`; spin turn adds ±~24 RPM. Loads of headroom. |
| Serial device | `/dev/ttyAMA0` (xacro) | `config.hpp` default is a red herring (see 3.7). |

## 6. Ideas / future improvements

1. **Publish battery as `sensor_msgs/BatteryState`** (voltage is already on `hoverboard/battery_voltage`) and wire a low-battery condition into the BT — return-to-dock trigger. The firmware's own `BAT_DEAD` poweroff (3.37 V/cell) is the last line, not a planner input.
2. **Use the DC current feedback** (the whole point of commit `1fb5689`) for stuck/stall/slip detection in the BT — e.g. current high + wheel velocity ≈ 0 → stuck recovery. Verify sign convention first (see 4.5).
3. **Echo check:** feedback `cmd1/cmd2` mirror what the firmware accepted — comparing them to what was sent would give an end-to-end "commands are actually arriving" health signal (better than the current byte-level `connected`).
4. **Driver hardening bundle** (one small PR when you're ready): fix 3.1/3.2, return ERROR instead of `exit`, move unit conversion out of `on_activate`, clamp before the int16 casts, add `static_assert(sizeof(SerialFeedback) == 26)`. The jitter bundle (§3.8) can ride along in the same PR or be done separately — it touches the same file but is independent of the correctness fixes.
5. **Firmware fork vs. upstream — delta checked 2026-07-18.** Upstream `EFeru/hoverboard-firmware-hack-FOC` `main` is 105 commits ahead of the fork point (`eb20cc0`); most is README/issue-template noise, HOVERCAR Multi-Mode-Drive, and DEBUG_SERIAL_PROTOCOL console fixes — irrelevant to this VARIANT_USART robot. The items that matter:
   - **Worth cherry-picking: interrupt-priority fix (`8df24d4`, PR #496).** Upstream demotes all UART/DMA interrupts (USART2/3 IRQs, DMA1 ch2/3 = USART3, ch6/7 = USART2) from priority 0 to 1 so the 16 kHz motor-control ADC/DMA ISR can never be preempted by serial traffic. This robot pushes 100 Hz feedback + 50 Hz commands + debug printf through exactly those interrupts — the fix directly hardens FOC timing. Applies cleanly to `setup.c`/`control.c`.
   - ~~Buzzer disable knob (`700d921`)~~ — **not wanted** (decided 2026-07-18; beeps are kept per the feature plan).
   - **Do NOT take blindly: `STARTUP_DELAY_MS` (`da8d64d`, 2026).** Upstream now waits 500 ms after MCU boot before engaging the power latch, to prevent spurious power-on from UART back-powering (a known hoverboard phenomenon — the Pi's TX can leak power into the board). **This breaks the 0.2 s relay pulse power-on** (`hoverBtnR1_pulse_duration` in `arduino_bridge.py`): the "button" must be held longer than the delay + boot time (~1 s) or the latch never engages. If this commit is ever picked up in a rebase, lengthen the pulse first.
   - **Validation, nothing to gain: upstream adopted official wheel odometry (`ad72a8e`, May 2026)** — literally the same algorithm the fork has carried since the robaka wheelcount merge (modulo 9000, same hall LUT, counts in feedback + checksum on both USARTs, behind `ENABLE_ODOMETRY`). A future rebase is therefore much easier than previously noted: the fork's only remaining protocol delta vs. upstream is the two `dc_curr` fields (plus the planned `flags` field from the feature plan).
   - Skippable: `calcAvgSpeed` fix (`5612af0`) only changes behavior for single-motor/inverted builds; field-weakening model fix (`9d9501a`) is moot with `FIELD_WEAK_ENA 0`.

   **Recommendation:** no rebase; cherry-pick `8df24d4` as part of the feature-plan firmware work.
6. **Docking interplay:** before implementing docking, decide the answer to 4.1 (idle poweroff) and 4.4 (re-enable after restart) — both directly affect "robot sits on dock for hours" behaviour.

## 7. Priority summary

| # | Finding | Where | Severity | Currently visible? |
|---|---|---|---|---|
| 1 | Inactivity poweroff during slow mowing / docked idle (30 min, threshold ≈0.5 m/s) | firmware `config.h:178`, `main.c:527` | **High (operational)** | Only on long slow missions / idle |
| 2 | Right-wheel encoder wrap broken (negate-before-unwrap) | driver `hoverboard_driver.cpp:442,541` | **High (latent)** | Masked by `position_feedback: false` |
| 3 | PID path unsafe if enabled (clamp disabled + swapped limits + int16 wrap) — *resolved 2026-07-18: path will be deleted (DEPLOY_TODO 2.7); firmware SPD_MODE is the closed loop* | driver `pid.cpp:59`, `hoverboard_driver.cpp:307` | High (dormant) | Only if `use_pid: true` |
| 4 | Control-loop jitter (spin_some + ~19 publishes/cycle in RT path) | driver `hoverboard_driver.cpp:351`, §3.8 | **Medium-High (observed)** | Yes — the known ros2_control jitter warnings |
| 4b | Board-restart odom compensation fails after 61 m | driver `hoverboard_driver.cpp:564` | Medium (latent) | Masked, as #2 |
| 5 | `exit(-1)` kills controller_manager on port failure | driver `hoverboard_driver.cpp:317` | Medium | On serial misconfig |
| 6 | `max_velocity` re-divided each activation | driver `hoverboard_driver.cpp:293` | Medium | On lifecycle restart |
| 7 | Motor re-enable gate after firmware restart | firmware `main.c:226` | Medium (operational) | On board brownout mid-mission |
| 8 | `uint16_t` array with negative values | firmware `bldc.c:95` | Low (works) | No |
| 9 | Assorted nits (dead code, comments, weak checksum, debug pubs) | both | Low | No |

Protocol compatibility between the fork and the driver is **correct** — structs, checksums, units and scaling all line up (two of them by lucky-but-undocumented coupling, see §2).
