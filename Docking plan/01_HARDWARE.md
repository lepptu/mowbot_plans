# 01 — Dock & Charging Hardware

> Status: **REVISED 2026-07-31** (originally planned 2026-07-10). Major changes in this
> revision: **mains-side (AC) switching** added, **voltage divider #2 deleted**, dock
> electronics consolidated onto a **hand-built control PCB**, Pi↔Nano USB promoted to the
> permanent comms link. The matching sequencing/state-machine changes land in **02 §4**
> (separate edit). Part of the docking master plan ([README.md](README.md)).
> Battery assumed: 10S li-ion "hoverboard" pack, 36 V nominal / 42.0 V full — confirmed by
> the existing `hoverboard/battery_voltage` telemetry and the 42 V charging requirement.

## 1. Electrical block diagram

```
230 V AC (RCD/vikavirtasuoja-protected outdoor socket — Schuko, i.e. NON-POLARIZED)
 │
 ├── ALWAYS ON ── 5 V PSU (≥3 A) ──┬── Dock Raspberry Pi
 │                                 │        └── USB ── Nano  (data + remote flash ONLY;
 │                                 │                          the Nano is POWERED from J2)
 │                                 └── control PCB 5 V in (J2)
 │
 └── SWITCHED ── K2 pilot relay (on PCB, out via J6)
                    └── G2R-2-SN coil (230 VAC, ~0.9 VA)
                           └── G2R-2 contacts break BOTH L and N
                                  └── 42 V CC/CV charger (2–4 A)
                                         └── fuse 5 A ── ACS712-05B ── K1 (DC relay) ── DOCK CONTACTS (+/–)
                                                             │
                                                             └── V-divider #1 → A0 (charger side)

 Common V−: 42 V−, 5 V 0 V, contact − and USB ground are ONE bus,
 logic ground tied to it at a single star point on the PCB (see 3.3).

DOCK CONTROL PCB (perfboard, see 3.3): Arduino Nano (socketed) + K1 + K2 + drivers
 ├── D2  seat microswitch (INPUT_PULLUP)      ├── A0  divider #1
 ├── D5  K1 driver (42 V DC relay)            ├── A1  ACS712 OUT
 ├── D6  K2 driver (230 V pilot)              └── D9 status LED (optional)
 └── J1 42 V in · J7 42 V out · J2 5 V in · J5 microswitch · J3 ACS712 · J6 pilot out

ROBOT SIDE: see 04 — pads sit at 0 V behind the ideal diode whenever undocked.
```

## 2. Charger

- **Dedicated 42.0 V CC/CV lithium charger, 2 A (or 4 A if the pack's
  BMS allows)**, left permanently at the dock. Reusing the household
  hoverboard charger works electrically but means carrying it back and
  forth — buy a second one (~20–40 €).
- CC/CV behavior is what makes "charge complete" detectable: current
  tapers toward 0 as the pack approaches 42 V. The dock firmware calls
  it complete below ~0.15 A (tunable).
- **The AC side is switched.** The original plan kept the charger
  permanently energized and switched only its DC output, to avoid any
  230 V work in the DIY enclosure. That constraint is void — the builder
  is a certified electrician — so the design inverts: the charger's
  mains feed goes through an Omron G2R-2-SN (both poles, see §3),
  piloted from the control PCB. The dock stays **plug-connected**
  (appliance construction), with the mains section built to code:
  both conductors switched, segregated mains island, proper strain
  relief.
- Why AC switching is the better design here:
  - The charger is energized **only during charge sessions and
    self-tests** — no cheap SMPS sitting hot 24/7 in an outdoor box.
    Less electrolytic aging, no no-load burst-mode whine, and a real
    reduction in unattended fire risk.
  - Standby draw disappears (~1–2 W ⇒ 10–20 kWh/yr — minor money, but
    the thermal argument above is the real one).
  - Breaking 230 V AC at the charger's ~0.5–0.8 A is trivial for any
    relay (arc quenches at the zero crossing) — the DC-arc problem the
    old plan engineered around vanishes from normal operation entirely.
- Operational consequences (firmware side, detail in 02 §4):
  - **Startup ramp**: the charger needs a moment from AC-on to a stable
    42 V. Measure the actual ramp time on the bench (§8) and use it as
    the 02 §4 timeout — "AC on but no 42 V on divider #1 within N s" ⇒
    fault.
  - **Welded AC relay detection**: a *steady* 42.0 V on divider #1 when
    AC is commanded off. (After a normal AC-off the charger's output
    caps decay slowly through the divider — steady vs. decaying is what
    distinguishes a weld from residual charge.)
  - **Pre-mow self-test**: 2 s AC pulse with K1 open, expect 42 V on
    #1 — so the robot never departs toward a dead charger.
- Normal sequence (K1 never switches under load): robot seats → K1
  closes at **0 V** across it → AC on → ramp verified on #1 → charge →
  complete/undock: **AC off first** → output caps drain into the pack,
  current → 0 in well under a second → K1 opens at **0 A** → robot
  leaves with dead contacts and the whole dock goes cold.

## 3. Switching, sensing, protection (dock side)

| Item | Recommendation | Notes |
| ---- | -------------- | ----- |
| AC relay (mains side) | **Omron G2R-2-SN 230AC(S)** in a **P2RF-08-E** DIN-rail socket; wire the two **NO** contacts to break **both L and N** | The Schuko feed is non-polarized — single-pole switching might break neutral only, so both conductors go through the relay. Coil (230 VAC, ~0.9 VA ≈ 4 mA) is piloted by K2 on the control PCB; de-energized = charger dead (fail-safe direction). Appliance-grade single-gap contacts are acceptable: no safety property rests on this relay alone, and a welded contact is detected on divider #1. SMPS inrush at 2–4 dockings/day is decade-scale contact wear. Mechanical flag + LED on the relay = state visible through the box. |
| Charge relay K1 (42 V DC) | On-PCB relay, contacts ≥5 A; discrete low-side driver: **2N3904 + 470 Ω** base resistor + **1N4148 flyback (cathode/K band to +5 V!)** + **10 kΩ base–emitter pulldown** | Role changed by AC switching: **sequencing and isolation only.** With AC-last-on / AC-first-off (02 §4), K1 closes at 0 V and opens at 0 A — it never makes or breaks current in normal operation. **RC snubber (100 Ω + 100 nF) across the contact: OPTIONAL — not populated by default.** Only relevant to the double-fault case (welded AC relay + emergency break under load); leave pad space and fit later if wanted. The old series-DPDT requirement is dropped. NPN low-side drivers are inherently boot-glitch-safe: a floating Nano pin during reset ⇒ base off ⇒ relay open. |
| Current sensor | **ACS712-05B module** in the **+ leg** of the charge line | Same part/pattern as the robot's mow-motor current sense — the auto-zero and EMA filtering code carries over 1:1; zero-calibrate at boot **before K1 closes**. Hall sensor = galvanically isolated from the 42 V path; 2–4 A sits nicely in ±5 A. The module's onboard cap is the VIOUT filter — do **not** hang extra capacitance directly on OUT (datasheet limit ~10 nF on VIOUT). If A1 jitters with relays clacking, add **1 kΩ series + 100 nF at the Nano pin** instead. |
| Voltage divider #1 (charger side) | 100 kΩ / 10 kΩ → 42 V ⇒ 3.82 V at A0 | Now the **only** voltage measurement (see below). Four jobs: charger present/healthy, AC-on ramp verification (with timeout), welded-AC-relay detection, pre-mow self-test. Thévenin source ≈ 9.1 kΩ — inside the ATmega's <10 kΩ ADC source spec; the pin cap (next row) tops up the sample-and-hold. |
| ADC / input protection | **100 nF ceramic from A0 to GND, placed at the Nano pin** (filter corner ≈ 175 Hz + charge reservoir for the ADC S/H cap); optional 5.1 V zener across the 10 k as a clamp. On D2: 100 nF + 20–50 ms firmware debounce if the switch run is long | Ceramics are non-polarized — either orientation. In firmware, discard the first ADC conversion after switching channels (or average 8–16 reads). |
| Fuse | 5 A blade fuse on the charger output | Sized ~2× charge current. First element after the charger, so it protects the ACS712 and everything downstream, including a short at the contacts. |
| Seat microswitch | Lever/roller microswitch at the dock end-stop → **D2** (`INPUT_PULLUP`), switch closes to GND | The **only** signal that allows K1 to close (02 §4). Mount it to trigger ~5 mm *before* the mechanical end-stop, guaranteeing contact overlap. Wired this way a broken wire reads "not docked" — fails safe. |
| Status LED (optional) | 1× single-color LED on **D9**, ~330–470 Ω series resistor to GND, active-high | State by blink pattern instead of color: **off = idle, solid = charging, blinking = fault** — readable at a glance in the yard. D9 = Timer1 PWM pin, so dimming via `analogWrite` is a free future option. Optional but cheap. |
| TVS | e.g. SMBJ48A across the contact pair on the dock side | Absorbs disconnect transients. |

**Why the contacts are never live when empty:** an empty dock has the
charger **unpowered** (G2R open) *and* K1 open — double isolation to
0 V contacts, stronger than the original single-relay design. K1 closes
only when the microswitch is pressed and interlocks pass; AC energizes
only after that. Enforced in firmware (02 §4) and cross-checked by
divider #1 + ACS712 telemetry.

**Why divider #2 was deleted:** in the original design the DC relay was
the only thing between a permanently live 42 V source and the contacts,
so a welded relay meant live contacts on an empty dock — divider #2
existed to catch exactly that. With AC switching, the contacts can only
be live if the charger is powered, and divider #1 sees that directly. A
welded K1 alone is now harmless (cutting AC kills the contacts
regardless); the dangerous state requires **both** relays to fail
closed, which #1 still flags as "charger live when commanded off".
"K1 failed to close" is caught as 42 V on #1 with zero ACS712 current
and no battery-voltage step on the robot. Presence detection was never
#2's job (the ideal diode blocks battery voltage from the contacts) —
that is the microswitch, full stop. Minimal sensing set: **D2 + divider
#1 + ACS712.**

### 3.1 Nano pin map

| Pin(s) | Function | Why |
| ------ | -------- | --- |
| D0/D1 | **reserved** — USB serial to the Pi | Permanent ROS 2 link + remote flashing; never wire I/O here. |
| D2 | Seat microswitch (`INPUT_PULLUP`) | D2/D3 are the interrupt-capable pair. |
| D5 | K1 driver (42 V relay) | Plain digital pin, no boot-time strings attached. |
| D6 | K2 driver (230 V pilot) | Ditto; D4/D7 are the spares for future outputs. |
| D9 | Status LED (optional) | Single-color, series resistor to GND; blink patterns carry the state. Timer1 PWM pin — dimmable later without rewiring (Timer1 is otherwise unused; D3/D10/D11 stay reserved for interrupt/SPI). |
| D10–D13 | keep free (SPI) | **D13 = onboard LED, toggles during the bootloader — never a relay.** |
| A0 | Divider #1 | |
| A1 | ACS712 OUT | |
| A4/A5 | keep free (I²C) | Future OLED/sensor, same as the robot's display work. |

Firmware rule: **safe state first.** The first lines of `setup()` write
the drivers' off state *before* `pinMode()` — the pin goes from
pulled-safe straight to driven-safe with no glitch. Together with the
10 k base pulldowns, both relays stay open through reset and the
bootloader window. Safe state = both relays open = charger dead,
contacts isolated.

### 3.2 Dock control PCB (hand-built)

One perfboard (raster board) carries the Nano, K1, K2, both discrete
drivers, and the passives. Hand-soldered; 90° trace corners are fine.

- **Connectors — all screw terminals:** J1 42 V in (from charger),
  J7 42 V out (to contacts), J2 5 V in, J5 microswitch, J3 ACS712
  module (5 V / GND / OUT), J6 pilot out (230 V to the G2R coil).
  Nano in a socket, not soldered.
- **Heavy current as real wire, not pad chains:** the 4 A path
  J1 → fuse → ACS712 → K1 → J7 and its V− return are laid as ≥1 mm²
  solid bus wire soldered flat along the board. Solder-bridged pad runs
  are resistive and brittle.
- **Grounding:** one common V− bus (42 V−, 5 V 0 V, contact −, USB
  ground). The heavy charge return flows only in the J1↔J7 stretch;
  logic grounds (divider bottom, caps, emitters, ACS712 GND) tap the
  bus at a **single star point** near the Nano. Charge current never
  routes through logic traces or the Nano's ground pins.
- **Decoupling:** 220–470 µF electrolytic across J2 (this one **is**
  polarized — stripe to GND) + 100 nF ceramics at the Nano and at each
  relay driver. A0's 100 nF sits on the A0 net with a short loop to the
  bus (per §3 table).
- **Mains island (K2 contact side + J6):**
  - Copper **cleared 2–3 hole rows (≥5 mm bare laminate)** around every
    230 V node, including K2's contact pins. Spacing is the insulation.
  - **UV-cured solder mask** over the mains section afterwards as the
    environmental seal — backup, not a substitute for spacing. Clean
    all flux (IPA scrub, dry) before masking; two thin coats from two
    angles; full cure; inspect joint crests for pinholes under bright
    light and re-dab.
  - K2's cleared zone extends to its contact pins but not its coil
    pins; verify K2's coil-to-contact isolation in its datasheet
    (common 5 V power relays: 1.5–4 kV — fine).
  - **OPTIONAL — not populated by default:** 100 Ω + 100 nF
    **X2-rated** RC snubber across K2's contacts (the G2R coil is a
    small inductive load; across mains it must be an X2 cap, not a
    plain ceramic). Leave two spare hole positions; fit only if §8
    testing shows ADC spikes or Nano hiccups at the moment K2
    switches. Equivalent alternative: RC across the G2R coil at the
    DIN socket, off-board.
  - Wiring to the G2R socket: **H05V-K 0.5–0.75 mm²**, both conductors
    routed together, strain-relieved so a tug lands on the anchor, not
    the pads.
- **Mechanics:** strain relief (zip-tie anchors through spare holes) on
  all heavy wires; mounting holes clear of the mains island.

### 3.3 Dual power & the Pi↔Nano USB link

- The Nano is **powered from the dock 5 V rail (J2)** and **permanently
  USB-connected to the Pi** — the USB carries the ROS 2 serial link and
  remote firmware flashing, same pattern as the robot's Pi↔hoverboard
  link. Relay coils and the ACS712 always draw from the dock rail, so
  the Pi's USB budget only ever sees the data link.
- This dual-feed works because the Nano reference design has a
  **Schottky diode between USB VBUS and the 5 V rail**: the external
  5.0 V wins, USB sits reverse-biased ~0.3 V below, nothing backfeeds
  into the Pi. **Verify the diode on the actual clone before first
  connection** (§8) — with both supplies present 24/7 this check is
  mandatory, not a bench nicety.
- Do **not** feed 5 V into VIN (it wants 7–12 V into the linear
  regulator); the 5 V pin / J2 rail is the correct injection point.
- **DTR auto-reset:** every serial-port open (ROS 2 node restart,
  reconnect) pulses DTR and reboots the Nano — ~2 s of bootloader with
  floating pins. The pulldowns + safe-state-first `setup()` make this
  fail safe: both relays drop, the dock goes cold, a mid-charge
  reconnect **aborts the session by design**. 02 §4 must treat "Nano
  rebooted → re-handshake → restart sequence" as a normal path, not a
  fault. Auto-reset stays enabled on purpose: it is what lets the Pi
  reflash dock firmware remotely with avrdude — no laptop trips to the
  yard.
- Grounds also join via the USB cable shield alongside the PSU wiring —
  a harmless loop at this scale; keep the USB cable short.

## 4. Contacts and mechanical design

### 4.1 Contact system (dock ⇄ robot)

Commercial-mower pattern, forward drive-in:

- **Dock:** two vertical spring-loaded contact blades (leaf springs) or
  heavy pogo-style plungers on a small tower/back wall, ~40–60 mm
  horizontal separation, faces angled so the robot's plates **wipe**
  across them on entry (self-cleaning against oxidation).
- **Robot:** two fixed plates (stainless or nickel-plated brass,
  ~20×40 mm) on the front chassis face, mounted on the rigid chassis —
  **not** on the moving bumper (the bumper must still be able to
  trigger without breaking the charge path; verify the bumper's travel
  clears the plates).
- Polarity safety: make the geometry asymmetric (plates at different
  heights, or one wide/one narrow) so reversed contact is mechanically
  impossible; the robot-side ideal diode is the electrical backstop.
- Spring travel ≥ 8–10 mm so RTK jitter and tire sink can't break
  contact mid-charge.
- Material: stainless spring steel or nickel/gold-plated brass. Plain
  copper will oxidize outdoors; the wiping action plus plating handles
  Finnish weather. Contacts live under the dock roof (see 4.2).

### 4.2 Dock structure

- **Baseplate** (plywood + outdoor paint, or HDPE sheet): flat, staked
  or screwed to ground anchors so repeated dockings don't shift it —
  a shifted dock invalidates the stored pose (re-record takes 2 min,
  but stability is better).
- **Wheel funnels / side guides**: two rails converging from ~1.5× robot
  width at the entry to wheel-width + ~2 cm at the seat position. This
  is what turns "RTK got us within ±4 cm" into "contacts aligned within
  ±5 mm". Make the last 20 cm parallel so the robot arrives square.
- **End-stop** with the microswitch; robot drives against it gently
  (docking approach speed will be ~0.1–0.15 m/s).
- **Roof/hood** over the contact tower and electronics box: rain cover
  for contacts, charger, relay box, Pi. Electronics in an IP54+ box
  with glands; charger needs airflow — don't seal it in. The G2R-2-SN
  in its DIN socket and the mains terminal blocks live in a segregated
  mains section of the box (or their own small box), separated from the
  ELV electronics.
- Entry approach: leave ≥1.5 m of straight, obstacle-free run-up in
  front of the dock (the staging pose sits ~0.7 m out, and Nav2 needs
  room to line up).

### 4.3 Placement (affects RTK — read before building)

- Best: open sky view, ≥2–3 m from walls/metal roofs (multipath).
- **Before fixing the base**: park the robot at the candidate spot and
  watch σx/σy of `/odometry/global` (same signal the localization gate
  uses) for a few minutes, plus RTK status on the Status page. If the
  spot can't hold RTK-fixed reliably, docking will be flaky — move the
  dock, not the tuning.
- Mains: existing RCD-protected outdoor socket; the dock plugs in with
  a normal outdoor extension-grade cable and **stays plug-connected**
  (appliance construction).

## 5. Robot-side charge path

Covered in **04** — not duplicated here. The one property this file
relies on from that side: the ideal diode keeps the robot's charge pads
at **0 V whenever nothing is connected**, and blocks any back-feed
toward the dock.

## 6. Dock Raspberry Pi & power

- **Pi 4 (2 GB+) or better** running Ubuntu Server 24.04 arm64 +
  ROS 2 Jazzy base. A Pi Zero 2 W (512 MB) is *not* recommended —
  Jazzy + zenoh + the bridge fit, but with no headroom and painful
  builds. A spare Pi 5 obviously works.
- Official 5 V/3 A+ PSU or a Mean Well 5 V module in the electronics
  box, on the **always-on** mains branch, feeding both the Pi and the
  control PCB (J2). The Nano is **powered from the dock 5 V rail**;
  the Pi's USB is attached permanently for ROS 2 serial + remote
  flashing (see 3.3) — coils and sensors never load the Pi's USB
  budget.
- Ethernet-over-WiFi: the dock Pi joins the house WLAN (fixed DHCP
  lease — the zenoh config and MQTT ACL reference it by IP).

## 7. Shopping list (rough)

| Item | ~Cost € |
| ---- | ------- |
| 42 V 2 A CC/CV lithium charger | 20–40 |
| Raspberry Pi 4 (2–4 GB) + SD + 5 V PSU | 60–80 |
| Arduino Nano (clone) | 5–10 |
| Omron G2R-2-SN 230AC(S) — AC relay | on hand (0) |
| P2RF-08-E DIN-rail socket | 5 |
| Control-PCB parts: perfboard, K1 + K2 relays, 2N3904 ×2, 1N4148 ×2, resistors, 100 nF ceramics, 220–470 µF, screw terminals, Nano socket (snubber parts 100 Ω + 100 nF X2: **optional**, see §3) | 10–20 |
| UV-curing solder mask + IPA | 10 |
| ACS712-05B module | 3–5 |
| Microswitch, fuse+holder, TVS, resistors/zeners, LED | 10 |
| Contact material (leaf springs / pogo plungers + plates) | 10–30 |
| IP54 enclosure, glands, baseplate materials, roof | 30–50 |
| **Total** | **~165–255** |

## 8. Hardware bench checklist (before any software integration)

Pre-power (multimeter ritual):

- [ ] Buzz out **K1 and K2 pinouts** on the physical relays: which pin
      pair is the coil, and which contact is genuinely **NO** with the
      coil unpowered. The fail-safe logic depends on being on NO.
- [ ] Both **1N4148 flyback bands (K)** face +5 V. Backwards = dead
      short through the transistor on first energize.
- [ ] **No continuity** between the mains island (K2 contacts, J6) and
      any ELV net; cleared copper rows verified by eye under bright
      light, before and after masking.
- [ ] Nano clone **VBUS Schottky check**: diode drop in diode mode
      between USB VBUS and the 5 V pin (or ~4.6–4.8 V on the 5 V pin
      when powered from USB alone). Mandatory before the Pi and dock
      PSU are ever connected together.

Powered, bench:

- [ ] Charger direct to mains (bypass the G2R): 42.0 V open-circuit,
      3.82 V on divider #1 at A0.
- [ ] Pilot chain: D6 high → K2 → G2R pulls in; **both poles** make and
      break (meter on each); G2R drops out on D6 low, on Nano reset,
      and on 5 V loss.
- [ ] **Measure AC-on → 42 V ramp time** on divider #1 and record it —
      this number becomes the 02 §4 timeout.
- [ ] K1 close with charger off: **0 V across the contacts** at the
      moment of closing, no arc (this is the normal sequence).
- [ ] Full sequence ×20–30: seat switch → K1 close → AC on → ramp →
      charge (dummy load) → AC off → current < 0.1 A → K1 open.
      Inspect both relays' behavior; contacts should look untouched.
- [ ] Dummy load (e.g. the robot battery via clip leads): ACS712 reads
      plausible current; compare against a multimeter.
- [ ] **Fault path only**: K1 break under 2 A load a few times —
      inspect contacts. Normal operation never does this; this test
      validates the emergency path (and the snubber, if fitted).
- [ ] Serial reconnect mid-charge: opening the Pi's serial port resets
      the Nano → both relays drop → dock goes cold → re-handshake and
      a fresh sequence restart cleanly (02 §4 normal path, not a
      fault).
- [ ] Microswitch clicks ~5 mm before the end-stop with the robot
      pushed in by hand.
- [ ] Robot plates hit dock springs with the robot pushed in by hand,
      ≥5 mm spring compression margin on both sides, at ±3 cm lateral
      entry offsets (funnel test).
- [ ] Rain test: hose the closed dock, verify electronics dry.
