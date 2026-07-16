# 01 — Dock & Charging Hardware

> Status: **PLANNED 2026-07-10.** Part of the docking master plan
> ([README.md](README.md)). Battery assumed: 10S li-ion "hoverboard"
> pack, 36 V nominal / 42.0 V full — confirmed by the existing
> `hoverboard/battery_voltage` telemetry and the 42 V charging
> requirement.

## 1. Electrical block diagram

```
230 V AC (RCD/vikavirtasuoja-protected outdoor circuit)
 ├── 42 V CC/CV lithium charger (2–4 A)
 │      └── fuse 5 A ── ACS712-05B ── charge RELAY ── DOCK CONTACTS (+/–)
 │                          │              │
 │                          │              ├── V-divider #2 (contact side)
 │                          └── V-divider #1 (charger side)
 └── 5 V PSU (≥3 A) ── Dock Raspberry Pi ── USB ── Arduino Nano
                                                     ├── relay driver
                                                     ├── ACS712 analog in
                                                     ├── 2× divider analog in
                                                     ├── seat microswitch in
                                                     └── status LED out

ROBOT SIDE:
 ROBOT CONTACTS (+/–) ── fuse 5 A ── ideal-diode module ── battery charge input
                                                            (parallel with the
                                                             existing charge jack)
```

## 2. Charger

- **Dedicated 42.0 V CC/CV lithium charger, 2 A (or 4 A if the pack's
  BMS allows)**, left permanently at the dock. Reusing the household
  hoverboard charger works electrically but means carrying it back and
  forth — buy a second one (~20–40 €).
- CC/CV behavior is what makes "charge complete" detectable: current
  tapers toward 0 as the pack approaches 42 V. The dock firmware calls
  it complete below ~0.15 A (tunable).
- The charger stays powered whenever the dock has mains; the **relay
  switches its DC output**, not the AC side. Rationale: no 230 V
  switching in the DIY enclosure (Finnish regs make fixed mains wiring
  the electrician's domain — the dock plugs into an existing outdoor
  socket), and the charger's output capacitors then also pre-charge
  through the relay in a controlled sequence (relay closes before any
  current is demanded, see 02 §4).

## 3. Switching, sensing, protection (dock side)

| Item | Recommendation | Notes |
|---|---|---|
| Charge relay | 5 V opto-isolated relay **module** with ≥10 A/250 VAC (≥5 A/30 VDC) contacts; wire **both poles of a DPDT in series** for extra DC margin, or use a 30 A automotive relay | 42 V DC arcs more than AC on break. Mitigated three ways: contacts in series, an RC snubber (100 Ω + 100 nF) across the contact, and control logic that opens the relay only after current < 0.3 A whenever possible (breaking under load is the fallback path only). A DC solid-state MOSFET switch is a fine alternative if you prefer no moving parts. |
| Current sensor | **ACS712-05B module** in the charge line | Same part/pattern as the robot's mow-motor current sense — the auto-zero calibration and EMA filtering code carries over 1:1. Hall sensor = galvanically isolated from the 42 V path. 2–4 A charge current sits nicely in the ±5 A range. |
| Voltage divider #1 (charger side) | 100 kΩ / 10 kΩ → 42 V ⇒ 3.82 V at the ADC | Detects "charger present and healthy" before closing the relay. |
| Voltage divider #2 (contact side, after relay) | identical | Verifies the relay actually opened/closed (welded-contact detection) and that voltage reaches the contacts. |
| ADC protection | 5.1 V zener + 100 nF to GND on both divider taps, 10 kΩ series into the Nano pin | Nano ADC is 10-bit/5 V; dividers under load are high-impedance so the RC also filters noise. |
| Fuse | 5 A blade fuse on the charger output | Sized ~2× charge current. |
| Seat microswitch | Lever/roller microswitch at the dock end-stop, actuated by the robot chassis when fully seated; wired NO → Nano input with `INPUT_PULLUP` | This is the **only** signal that allows the relay to close (02 §4). Mount it so it triggers ~5 mm *before* the mechanical end-stop, guaranteeing contact overlap. |
| Status LED | 1× bicolor or 2× LED on a Nano pin | idle / charging / fault at a glance in the yard. Optional but cheap. |
| TVS | e.g. SMBJ48A across the contact pair on the dock side | Absorbs disconnect transients. |

**Why the contacts are never live when empty:** the relay stays open
until the microswitch is pressed *and* interlocks pass, so a child or a
rake touching the dock contacts finds 0 V. This is the core safety
property of the design — enforced in firmware (02 §4) and re-checked by
divider #2 telemetry.

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
  with glands; charger needs airflow — don't seal it in.
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
  a normal outdoor extension-grade cable.

## 5. Robot-side charge path (details in 04)

- Contacts → **5 A fuse** → **ideal-diode module** (LM5050/LM74610-class
  "ideal diode" board, ≥5 A, ≥50 V) → battery charge input, wired in
  parallel with the existing charge jack (through the BMS charge port —
  the pack's BMS keeps doing cell protection exactly as with the wall
  charger).
- The ideal diode (a) keeps the robot's exposed plates at 0 V when
  undocked, (b) blocks reverse current, (c) drops only millivolts —
  unlike a plain Schottky, the pack still reaches full 42.0 V.
- No robot firmware/Nano changes needed: the charge path is passive.
  Charge detection on the robot side comes for free via the existing
  `hoverboard/battery_voltage` telemetry (voltage steps up when the
  charger engages) — the authoritative signal is still the dock's
  current sensor.

## 6. Dock Raspberry Pi & power

- **Pi 4 (2 GB+) or better** running Ubuntu Server 24.04 arm64 +
  ROS 2 Jazzy base. A Pi Zero 2 W (512 MB) is *not* recommended —
  Jazzy + zenoh + the bridge fit, but with no headroom and painful
  builds. A spare Pi 5 obviously works.
- Official 5 V/3 A+ PSU or a Mean Well 5 V module in the electronics
  box. The Nano is powered from the Pi's USB port (same as the robot).
- Ethernet-over-WiFi: the dock Pi joins the house WLAN (fixed DHCP
  lease — the zenoh config and MQTT ACL reference it by IP).

## 7. Shopping list (rough)

| Item | ~Cost € |
|---|---|
| 42 V 2 A CC/CV lithium charger | 20–40 |
| Raspberry Pi 4 (2–4 GB) + SD + 5 V PSU | 60–80 |
| Arduino Nano (clone) | 5–10 |
| Relay module (2-channel, opto) or automotive relay + driver | 5–15 |
| ACS712-05B module | 3–5 |
| Ideal-diode module ≥5 A/50 V (robot side) | 5–10 |
| Microswitch, fuses+holders ×2, TVS, resistors/zeners, LED | 10 |
| Contact material (leaf springs / pogo plungers + plates) | 10–30 |
| IP54 enclosure, glands, baseplate materials, roof | 30–50 |
| **Total** | **~150–250** |

## 8. Hardware bench checklist (before any software integration)

- [ ] Charger alone: 42.0 V open-circuit at divider #1.
- [ ] Relay closed by hand (jumper): 42 V at contacts, divider #2 follows.
- [ ] Dummy load (e.g. the robot battery via clip leads): ACS712 reads
      plausible current; compare against a multimeter.
- [ ] Relay break under 2 A load 20× — inspect contacts (this is the
      worst-case path; normal operation breaks at <0.3 A).
- [ ] Microswitch clicks ~5 mm before the end-stop with the robot
      pushed in by hand.
- [ ] Robot plates hit dock springs with the robot pushed in by hand,
      ≥5 mm spring compression margin on both sides, at ±3 cm lateral
      entry offsets (funnel test).
- [ ] Ideal diode: 0 V on robot plates when undocked; charge current
      flows when mated; no back-feed with charger off.
- [ ] Rain test: hose the closed dock, verify electronics dry.
