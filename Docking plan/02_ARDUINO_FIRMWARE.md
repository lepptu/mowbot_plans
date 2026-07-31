# 02 — Dock Arduino Nano Firmware

> Status: **REVISED 2026-07-31** (originally planned 2026-07-10) — brought in line
> with the revised [01_HARDWARE.md](01_HARDWARE.md): **two-relay AC-side switching**
> (K1 42 V DC + K2 mains pilot, strict AC-last-on / AC-first-off sequencing),
> **voltage divider #2 deleted**, **pre-mow self-test**, **DTR reboot promoted to a
> normal session-abort path**, Nano powered from the dock 5 V rail (USB = data +
> remote flash only). Part of the docking master plan ([README.md](README.md)).
> Modeled directly on the proven robot firmware
> `~/Arduino/mowbot_robot_arduino/src/main.cpp` (10 Hz CSV status, newline-framed
> commands, 2 s watchdog, ACS712 auto-zero, EMA filtering) — same idioms, but
> written in English and split into modules instead of one `main.cpp`.

## 1. Communication: USB serial (decision)

**USB serial to the dock Pi, 115200 baud** — same as the robot bridge.

Why not Pi GPIO/UART or discrete I/O lines:

- The USB link carries **data and remote flashing only** — the Nano is powered
  from the dock 5 V rail (J2), not from the Pi (01 §3.3). One data cable, no
  5 V↔3.3 V level shifting, and relay coils never load the Pi's USB budget.
- The tolerant line-oriented CSV protocol and its parser already exist
  (robot side) and will be ported to the dock agent (03 §3).
- The Nano's ADC does the analog work the Pi can't; keeping the 42 V
  domain entirely on the Nano side of a USB cable keeps the Pi safe.
- **DTR auto-reset is a design feature here, not just a quirk** (01 §3.3):
  every serial-port open (dock-agent restart, reconnect) resets the Nano →
  the base pulldowns drop **both relays** in hardware → the dock goes cold →
  a mid-charge session **aborts by design**. After reboot the firmware
  re-handshakes (`EVT:BOOT`) and, if the robot is still seated and enable is
  set, simply runs a fresh charge sequence. §4 treats this as a **normal
  path, not a fault**. Auto-reset staying enabled is also what lets the Pi
  reflash the dock firmware remotely with avrdude.

## 2. Project layout (PlatformIO, like the robot firmware)

```
~/Arduino/ROSMower_dock/
├── platformio.ini            # board: nanoatmega328 (new bootloader variant as needed)
└── src/
    ├── main.cpp              # setup() + loop() only: init, tick order, timing
    ├── pins.hpp              # all pin constants + polarity notes in one place
    ├── sensors.hpp/.cpp      # ACS712 (auto-zero + EMA), divider #1, microswitch debounce
    ├── charge_control.hpp/.cpp # two-relay state machine + interlocks + fault latch
    └── protocol.hpp/.cpp     # serial RX (command parse) + TX (status frame) + watchdog
```

Module responsibilities (what goes in each .hpp/.cpp):

| Module | Contents |
|---|---|
| `pins.hpp` | `K1_PIN` (D5, 42 V DC relay) and `K2_PIN` (D6, 230 V pilot) — both **active-high discrete NPN low-side drivers** (01 §3), not relay modules; `MICROSWITCH_PIN` (D2, `INPUT_PULLUP`, closed = seated); `LED_PIN` (D4, single color); `V_CHARGER_PIN` (A0, divider #1, ratio ×11); `ACS712_PIN` (A1, 185 mV/A). Reserved and documented as such: D0/D1 (USB serial), D13 (bootloader LED — never an output). Matches the 01 §3.1 pin map exactly. |
| `sensors.hpp/.cpp` | `void sensorsInit()` — 100-sample ACS712 zero calibration at boot (copy of the robot's proven block; both relays open **and** the charger unpowered at that moment, so a guaranteed true zero); `void sensorsRead()` — every loop: EMA-filtered current (`smoothFactor` ≈ 0.05, faster than the mow motor's 0.01 since charge current is steady), divider #1 scaled to volts (discard the first conversion after the ADC channel switch, or average 8–16 reads — 01 §3), microswitch debounced (~20 ms). Getters: `chargeCurrentA()`, `chargerVoltage()`, `robotSeated()`. |
| `charge_control.hpp/.cpp` | The two-relay state machine + interlocks below. `chargeControlTick(...)` called every loop; owns **both** relay pins exclusively and enforces the sequencing invariant (§4). Fault latch + `faultCode()`. |
| `protocol.hpp/.cpp` | `bool protocolReadCommand(...)` — non-blocking `Serial.readStringUntil('\n')` equivalent with `strtok` split (robot pattern), feeds the watchdog timestamp; `void protocolSendStatus(...)` at 10 Hz; watchdog check (2000 ms → treat as `enable` unchanged, report fault 4). |

`main.cpp` stays ~60 lines. **Safe state first** (01 §3.1 rule): the very first
lines of `setup()` write both driver outputs LOW *before* `pinMode()` — each pin
goes from pulled-safe (10 kΩ base pulldown) straight to driven-safe with no
glitch. Then: init modules, and
`loop(){ sensorsRead(); protocol tick; chargeControlTick(...); status @10Hz; }`.

## 3. Serial protocol

Mirrors the robot protocol style: ASCII CSV, newline-terminated, both
directions ~10 Hz, tolerant parsing (wrong field count ⇒ skip line).
Since robot FW 2.1.0 the style also includes comma-free `EVT:<TYPE>[:detail]`
event lines beside the status frame — adopted here: `EVT:BOOT:<ver>` once at
startup (the Pi's re-handshake trigger after a DTR reset), `EVT:VER:<ver>`
every 60 s, `EVT:SELFTEST:OK|FAIL` after a self-test, `EVT:EMERGENCY:SWITCH`
on a seat-switch release under load, `EVT:NOCURRENT` advisory (see §4).

**Pi → Nano (command frame, 4 fields):**

```
<chargeEnable>,<clearFault>,<ledMode>,<selfTest>\n
e.g. "1,0,0,0\n"
```

| Field | Meaning |
|---|---|
| `chargeEnable` | 0/1 — permission to charge (see §4: permission, not command) |
| `clearFault` | 1 = clear a latched fault (edge-acted, then send 0 again) |
| `ledMode` | 0 auto, 1 force off (reserved; auto = state-driven) |
| `selfTest` | 1 = run the pre-mow charger self-test (edge-acted; accepted only in IDLE/COMPLETE, see §4) |

**Nano → Pi (status frame, exactly 8 fields):**

```
<state>,<microswitch>,<k1>,<k2>,<chargeCurrent %.2f>,<chargerVoltage %.1f>,<faultCode>,<uptimeS>\n
e.g. "3,1,1,1,1.87,41.8,0,3721\n"
```

Still exactly 8 numeric fields — `contactVoltage` (deleted divider #2) is
replaced by the K2/AC state, so the "skip any line that isn't exactly 8
numeric fields" parser convention carries over unchanged.

| Field | Meaning |
|---|---|
| `state` | 0 IDLE, 1 SEATED, 2 RAMP, 3 CHARGING, 4 DRAIN, 5 COMPLETE, 6 SELFTEST, 7 FAULT (renumbered vs the 2026-07-10 plan; 03/04 tables must use these values) |
| `microswitch` | debounced seat switch (1 = robot seated) |
| `k1` | commanded K1 state (42 V DC relay) |
| `k2` | commanded K2/AC state (mains pilot) |
| `chargeCurrent` | filtered A (float, `.` decimal — Pi parses locale-independently) |
| `chargerVoltage` | divider #1, V — reads ≈ 0 whenever AC is off (that is the *normal* idle value now) |
| `faultCode` | 0 none, 1 overcurrent (latching), 2 AC-relay weld / charger live when commanded off (latching), 3 no 42 V after AC-on — charger or AC path dead (non-latching, auto-retry), 4 watchdog silence (report-only) |
| `uptimeS` | seconds since boot (a reset to 0 tells the Pi a DTR reboot happened) |

Firmware may also emit `WARNING:...` free-text lines (robot convention).

## 4. Charge control state machine & interlocks (the safety core)

Two principles:

1. **`chargeEnable` from the Pi is a *permission*, not a direct relay
   command.** The firmware alone decides when the relays may close, and every
   unsafe condition opens them regardless of what the Pi says. Default
   permission is ON at boot (D7 in README: charging must work even if the
   dock Pi is down — the Nano is the authority; the Pi's enable=0 is for
   maintenance/manual override and the sequenced undock). Watchdog nuance:
   because default is permissive, loss of serial (Pi crash) does **not**
   stop an ongoing charge — charging continues on firmware interlocks alone;
   fault 4 is reported once serial returns. This is the deliberate inverse
   of the robot firmware's watchdog (where silence must stop the blade).
   (A Pi *reconnect* does stop the charge — via DTR reset, §1 — and that is
   fine: the session restarts cleanly.)
2. **Sequencing invariant: K1 never makes or breaks current in normal
   operation** (01 §2). AC-last-on: K1 closes at 0 V *before* the charger is
   energized. AC-first-off: the charger is de-energized and the current has
   drained *before* K1 opens at 0 A. The firmware is the enforcer of this
   invariant; the only unsequenced break paths are the emergency ones below.

```
IDLE (cold: K1 open, AC off, V#1 ≈ 0)
  ──seat + enable + no latched fault──► close K1 (at 0 V) → SEATED
SEATED ──K1_SETTLE_MS (~100 ms, contact bounce done)──► AC on (K2) → RAMP
RAMP ──V#1 ≥ 40 V within RAMP_TIMEOUT_S──► CHARGING
RAMP ──timeout──► AC off, K1 open → IDLE, report fault 3
                  (non-latching: retry after RETRY_HOLDOFF_S while seated+enable —
                   survives a mains outage with no Pi involved)
CHARGING ──current < 0.15 A for 60 s (CV taper)──► AC off → DRAIN
CHARGING ──enable 1→0 (sequenced undock, 04 §6)──► AC off → DRAIN
DRAIN ──current < 0.10 A (typ. well under 1 s: output caps drain into pack)──►
        open K1 (at 0 A) → COMPLETE (if from taper) / IDLE (if from enable-off)
DRAIN ──current still > 0.10 A after DRAIN_TIMEOUT_S──► AC relay weld!
        open K1 anyway (under load — the 01 §8 fault path), latch FAULT 2
COMPLETE (cold, robot still seated) ──microswitch released──► IDLE
COMPLETE ──enable 0→1 edge──► re-sequence via SEATED (see re-entry rule below)
IDLE/COMPLETE ──selfTest edge──► SELFTEST: K1 stays open, AC on ≤ 2 s,
        V#1 ≥ 40 V ⇒ EVT:SELFTEST:OK else EVT:SELFTEST:FAIL → AC off → back
any live state ──microswitch released──► open BOTH immediately → IDLE
        (emergency, unsequenced; EVT:EMERGENCY:SWITCH)
any live state ──current > 4.0 A for 50 ms──► open BOTH, latch FAULT 1
AC commanded off ──V#1 rising, or V#1 > 35 V past WELD_WINDOW_S──► latch FAULT 2
FAULT (1/2) ──clearFault edge from Pi (accepted only at 0 A)──► IDLE
```

Additional rules:

- **Welded-AC-relay detection is three detectors, not one** (01 §2/§3): (a)
  current refuses to die during DRAIN — the sharpest one; (b) V#1 *rising*
  while AC is commanded off — caps never rise on their own; (c) V#1 still
  > 35 V once `WELD_WINDOW_S` (5 min) after AC-off has passed. The window
  exists because the charger's output caps decay only through the 110 kΩ
  divider — τ is on the order of minutes, so "still high shortly after
  AC-off" proves nothing; *steady/rising vs. decaying* is the discriminator.
- **COMPLETE does not re-enter charging on its own.** The dock goes fully
  cold at completion (no trickle — the old plan's "relay stays closed" is
  gone). With the robot seated and enable still 1, an automatic re-sequence
  would oscillate on battery surface-charge sag; instead re-entry requires
  an explicit `enable` 0→1 edge from the dock agent (which decides using
  robot battery telemetry — ripple to 03/04).
- **"K1 failed to close" is not fully detectable on the dock alone**: it
  looks like RAMP-OK followed by ~0 A, which is also what a full battery
  looks like (taper below threshold — README open question 5). So it is an
  advisory, not a fault: `EVT:NOCURRENT` after `NOCURRENT_S` below 0.10 A;
  the dock agent cross-checks against the robot's `battery_voltage` step
  (01 §3, 04). The taper logic then closes the session normally either way.
- Sequenced undock (replaces the old "open-before-arc" rule, whose
  DC-break-under-CC-load problem no longer exists): dock agent drops
  `enable` → AC off → DRAIN → K1 opens at 0 A → agent confirms `state`
  IDLE/cold → only then `UndockRobot` (04 §6). The microswitch-release
  path above is the unsequenced fallback if the robot leaves early.
- K1 re-close holdoff 1 s (microswitch chatter); close only from IDLE.
- `chargeControlSafeState()` — **both** relays open, LED to pattern — called
  at boot before calibration and by the overcurrent/weld/emergency paths.
- Status LED (D4, single color, 01 §3): **off** = cold (IDLE, COMPLETE),
  **solid** = live sequence (SEATED/RAMP/CHARGING/DRAIN/SELFTEST),
  **blinking** = FAULT.
- All thresholds are `constexpr` in `charge_control.hpp`, documented in one
  block, easy to tune during bench tests: `COMPLETE_A` 0.15, `COMPLETE_S`
  60, `OVERCURRENT_A` 4.0, `OVERCURRENT_MS` 50, `RAMP_V_OK` 40.0,
  `RAMP_TIMEOUT_S` **placeholder 5 — set from the measured AC-on→42 V ramp
  (01 §8)**, `K1_SETTLE_MS` 100, `DRAIN_A` 0.10, `DRAIN_TIMEOUT_S` 2,
  `WELD_V` 35.0, `WELD_WINDOW_S` 300, `RETRY_HOLDOFF_S` 60, `NOCURRENT_S` 5,
  `RECLOSE_HOLDOFF_S` 1, `SELFTEST_S` 2, `WD_TIMEOUT_MS` 2000.
  (Runtime-tunable via protocol = later, only if field tests demand it.)

## 5. What is deliberately NOT in the firmware

- No PWM/analog outputs, no motor logic — this stays a simple I/O node.
- No charge algorithm: the CC/CV charger does constant-current /
  constant-voltage; the pack's BMS does cell-level protection. The
  firmware only gates *whether* the charger is energized and reaches the
  contacts, in the right order.
- No persistent state (EEPROM): after power loss both relays are open by
  hardware (pulldowns), and the state machine re-derives everything from
  inputs within one second.

## 6. Bench test checklist (firmware + control PCB on 5 V; no mains needed — a bench PSU on J1 plays the charger, mains-side tests live in 01 §8)

- [ ] **Boot safety**: LED/scope on D5 and D6 through power-up, reset button,
      and serial-port open — never a glitch pulse (pulldowns + safe-state-first
      verified together).
- [ ] Status frames at 10 Hz, exactly 8 fields, `uptimeS` counts,
      `EVT:BOOT` once at startup.
- [ ] ACS712 auto-zero: `chargeCurrent` reads 0.00 ±0.05 A at rest.
- [ ] **Full sequence** (bench PSU on J1 switched by hand as the "charger",
      dummy load as the "battery"): seat switch + enable=1 → K1 closes
      *first*, K2 ~100 ms later, never the other way; PSU on → RAMP→CHARGING;
      taper (reduce load) → 60 s → DRAIN → K1 opens only after current
      < 0.10 A → COMPLETE, dock cold.
- [ ] COMPLETE holds on static enable=1 (no re-sequence); `enable` 0→1 edge
      re-enters the sequence.
- [ ] Ramp timeout: PSU left off → fault 3 reported, both relays open,
      auto-retry after 60 s, recovers by itself once the PSU is on.
- [ ] Weld simulation: keep 42 V on J1 with K2 commanded off → FAULT 2
      (rise/window detectors); simulate DRAIN-stuck (PSU stays on through
      DRAIN) → FAULT 2 + K1 opens under load.
- [ ] Release microswitch mid-charge → **both** relays open < 50 ms,
      `EVT:EMERGENCY:SWITCH`, state IDLE.
- [ ] Overcurrent injection (short through a 10 Ω power resistor or
      current-limited supply) → FAULT 1 latched, relays stay open,
      `clearFault` recovers (and is ignored while current ≠ 0).
- [ ] Watchdog: stop sending commands mid-charge → charging continues
      (§4 principle 1), fault 4 reported on next command; enable=0 →
      sequenced shutdown, not an instant K1 break.
- [ ] Kill USB (unplug) mid-charge → charging continues on interlocks
      (Nano is J2-powered — this test is only possible thanks to that).
- [ ] **Serial reconnect mid-charge** (reopen the port): DTR reset → both
      relays drop → dock cold → `EVT:BOOT`, re-handshake, fresh sequence
      restarts cleanly — a normal path, no fault latched (01 §8 item).
- [ ] `selfTest` from IDLE and from COMPLETE → 2 s K2 pulse with K1 open,
      correct OK/FAIL EVT; `selfTest` during CHARGING → ignored with WARNING.
- [ ] Full-battery dock (PSU + no load): immediate taper → quick COMPLETE,
      `EVT:NOCURRENT` advisory, **no** fault.
- [ ] 24 h soak with battery: reaches COMPLETE, dock cold afterwards, no
      spurious faults, contacts cool.
