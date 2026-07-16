# 02 — Dock Arduino Nano Firmware

> Status: **PLANNED 2026-07-10.** Part of the docking master plan
> ([README.md](README.md)). Modeled directly on the proven robot
> firmware `~/Arduino/mowbot_robot_arduino/src/main.cpp` (10 Hz CSV
> status, newline-framed commands, 2 s watchdog, ACS712 auto-zero,
> EMA filtering) — same idioms, but written in English and split into
> modules instead of one `main.cpp`.

## 1. Communication: USB serial (decision)

**USB serial to the dock Pi, 115200 baud** — same as the robot bridge.

Why not Pi GPIO/UART or discrete I/O lines:

- USB powers the Nano — one cable, no 5 V↔3.3 V level shifting.
- The tolerant line-oriented CSV protocol and its parser already exist
  (robot side) and will be ported to the dock agent (03 §3).
- The Nano's ADC does the analog work the Pi can't; keeping the 42 V
  domain entirely on the Nano side of a USB cable keeps the Pi safe.
- Known quirk carried over: opening the port toggles DTR → the Nano
  auto-resets and spends ~0.5–1 s in `setup()` (including current-sensor
  auto-zero). The Pi-side parser must tolerate garbage/silence at open
  (it already does on the robot).

## 2. Project layout (PlatformIO, like the robot firmware)

```
~/Arduino/ROSMower_dock/
├── platformio.ini            # board: nanoatmega328 (new bootloader variant as needed)
└── src/
    ├── main.cpp              # setup() + loop() only: init, tick order, timing
    ├── pins.hpp              # all pin constants + polarity notes in one place
    ├── sensors.hpp/.cpp      # ACS712 (auto-zero + EMA), dividers, microswitch debounce
    ├── charge_control.hpp/.cpp # relay state machine + interlocks + fault latch
    └── protocol.hpp/.cpp     # serial RX (command parse) + TX (status frame) + watchdog
```

Module responsibilities (what goes in each .hpp/.cpp):

| Module | Contents |
|---|---|
| `pins.hpp` | `RELAY_PIN` (note active level of the relay module!), `ACS712_PIN` (A0), `V_CHARGER_PIN` (A1), `V_CONTACT_PIN` (A2), `MICROSWITCH_PIN` (D2, `INPUT_PULLUP`), `LED_*_PIN`. Divider ratio + ACS712 mV/A constants. |
| `sensors.hpp/.cpp` | `void sensorsInit()` — 100-sample ACS712 zero calibration at boot (copy of the robot's proven block, relay guaranteed open so true zero); `void sensorsRead()` — every loop: EMA-filtered current (`smoothFactor` ≈ 0.05, faster than the mow motor's 0.01 since charge current is steady), divider voltages scaled to volts, microswitch debounced (~20 ms). Getters: `chargeCurrentA()`, `chargerVoltage()`, `contactVoltage()`, `robotSeated()`. |
| `charge_control.hpp/.cpp` | The state machine + interlocks below. `chargeControlTick(bool enableCmd)` called every loop; owns the relay pin exclusively. Fault latch + `faultCode()`. |
| `protocol.hpp/.cpp` | `bool protocolReadCommand(...)` — non-blocking `Serial.readStringUntil('\n')` equivalent with `strtok` split (robot pattern), feeds the watchdog timestamp; `void protocolSendStatus(...)` at 10 Hz; watchdog check (2000 ms → treat as `enable=0`). |

`main.cpp` stays ~60 lines: init modules, then
`loop(){ sensorsRead(); wd=protocol tick; chargeControlTick(...); status @10Hz; }`.

## 3. Serial protocol

Mirrors the robot protocol style: ASCII CSV, newline-terminated, both
directions ~10 Hz, tolerant parsing (wrong field count ⇒ skip line).
Since robot FW 2.1.0 the style also includes comma-free `EVT:<TYPE>[:detail]`
event lines beside the status frame (`EVT:BOOT:<ver>` once at startup,
`EVT:VER:<ver>` every 60 s, safety events) — adopt the same pattern here.

**Pi → Nano (command frame, 3 fields):**

```
<chargeEnable>,<clearFault>,<ledMode>\n
e.g. "1,0,0\n"
```

| Field | Meaning |
|---|---|
| `chargeEnable` | 0/1 — permission to charge (see §4: permission, not command) |
| `clearFault` | 1 = clear a latched fault (edge-acted, then send 0 again) |
| `ledMode` | 0 auto, 1 force off (reserved; auto = state-driven) |

**Nano → Pi (status frame, exactly 8 fields):**

```
<state>,<microswitch>,<relay>,<chargeCurrent %.2f>,<chargerVoltage %.1f>,<contactVoltage %.1f>,<faultCode>,<uptimeS>\n
e.g. "2,1,1,1.87,42.0,41.2,0,3721\n"
```

| Field | Meaning |
|---|---|
| `state` | 0 IDLE, 1 ROBOT_SEATED, 2 CHARGING, 3 CHARGE_COMPLETE, 4 FAULT |
| `microswitch` | debounced seat switch (1 = robot seated) |
| `relay` | commanded relay state |
| `chargeCurrent` | filtered A (float, `.` decimal — Pi parses locale-independently) |
| `chargerVoltage` | divider #1, V |
| `contactVoltage` | divider #2, V (relay verification) |
| `faultCode` | 0 none, 1 overcurrent, 2 relay-verify failed (weld/no-close), 3 charger voltage missing, 4 watchdog trip |
| `uptimeS` | seconds since boot (detects Nano resets from the Pi side) |

Firmware may also emit `WARNING:...` free-text lines (robot convention);
the Pi parser skips any line that isn't exactly 8 numeric fields.

## 4. Charge control state machine & interlocks (the safety core)

**Principle: `chargeEnable` from the Pi is a *permission*, not a direct
relay command. The firmware alone decides when the relay may close, and
every unsafe condition opens it regardless of what the Pi says.**
Default permission is ON at boot (D7 in README: charging must work even
if the dock Pi is down — the Nano is the authority; the Pi's enable=0 is
for maintenance/manual override). Watchdog nuance: because default is
permissive, loss of serial (Pi crash) does **not** stop an ongoing
charge — it latches nothing and charging continues on firmware
interlocks alone; fault 4 is reported once serial returns. This is the
deliberate inverse of the robot firmware's watchdog (where silence must
stop the blade).

```
IDLE ──microswitch pressed + chargerVoltage OK + no fault + enable──► CLOSE relay (state ROBOT_SEATED)
ROBOT_SEATED ──current > 0.3 A within 3 s──► CHARGING
CHARGING ──current < 0.15 A for 60 s (CV taper)──► CHARGE_COMPLETE (relay stays closed, trickle ok)
CHARGE_COMPLETE ──current rises again (battery sag / robot load)──► CHARGING
any state ──microswitch released──► OPEN relay immediately → IDLE
any state ──overcurrent > 4.0 A (50 ms)──► OPEN relay, latch FAULT 1
ROBOT_SEATED ──contactVoltage ≁ chargerVoltage after close (±2 V, 500 ms)──► FAULT 2
any closed state ──contactVoltage > 5 V while relay commanded open──► FAULT 2 (weld!)
IDLE ──chargerVoltage < 40 V──► report FAULT 3 (non-latching; clears when charger returns)
FAULT (1/2) ──clearFault edge from Pi──► IDLE
```

Additional rules:

- **Open-before-arc:** when `enable` goes 0 or undock is expected, the
  relay opens while current may still flow — acceptable fallback, but
  the normal undock flow (04 §6) has the robot side stop pulling
  current... it can't (charger is CC) — so instead the dock agent drops
  `enable` **before** the robot physically leaves (dock_manager
  sequences relay-off → wait current < 0.3 A confirmation → UndockRobot;
  the microswitch-release path is the unsequenced fallback).
- Relay close happens only from IDLE→ROBOT_SEATED, never bounces on
  microswitch chatter (debounce + 1 s re-close holdoff).
- `stopEverything()` equivalent: `chargeControlSafeState()` — relay
  open, LED to fault pattern — called at boot before calibration and by
  the overcurrent/weld paths.
- All thresholds (`0.3 A`, `0.15 A`, `60 s`, `4.0 A`, `40 V`) are
  `constexpr` in `charge_control.hpp` — documented in one block, easy to
  tune during bench tests. (Runtime-tunable via protocol = later, only
  if field tests demand it.)

## 5. What is deliberately NOT in the firmware

- No PWM/analog outputs, no motor logic — this stays a simple I/O node.
- No charge algorithm: the CC/CV charger does constant-current /
  constant-voltage; the pack's BMS does cell-level protection. The
  firmware only gates *whether* the charger reaches the contacts.
- No persistent state (EEPROM): after power loss the state machine
  re-derives everything from inputs within one second.

## 6. Bench test checklist (firmware alone, USB to any PC)

- [ ] Boot with relay module disconnected from mains path: status frames
      at 10 Hz, exactly 8 fields, `uptimeS` counts.
- [ ] ACS712 auto-zero: `chargeCurrent` reads 0.00 ±0.05 A at rest.
- [ ] Press microswitch by hand with charger on + enable=1 → relay
      clicks, state 1→2 with a dummy load, contactVoltage ≈ chargerVoltage.
- [ ] Release microswitch under load → relay opens < 50 ms, state 0.
- [ ] Overcurrent injection (short through a 10 Ω power resistor or
      current-limited supply) → FAULT 1 latched, relay stays open,
      `clearFault` recovers.
- [ ] Watchdog: stop sending commands mid-charge → charging continues
      (see §4), faultCode 4 reported on next command; enable=0 →
      relay opens.
- [ ] Kill USB (unplug) mid-charge → charging continues on interlocks;
      replug → status resumes (Pi-side reopen handles DTR reset).
- [ ] Charger off → FAULT 3 reported, relay never closes on switch press.
- [ ] 24 h soak with battery: reaches CHARGE_COMPLETE, no spurious
      faults, contacts cool.
