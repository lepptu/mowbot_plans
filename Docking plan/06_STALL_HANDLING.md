# 06 — Docking Stall Handling (tyre caught in the funnel)

> Status: **PLANNED 2026-09-12, not started.** Part of the docking master
> plan ([README.md](README.md)); extends the bridge `dock_manager` of
> [04 §3](04_ROBOT_MODIFICATIONS.md). Deferred by the owner until the
> line-follow final stretch (04 §10.12) has shown how often the funnel
> still catches a tyre.

## 1. Problem (field evidence 2026-09-12, 04 §10.11–10.12)

Twice the robot arrived at the funnel mouth aligned to a few millimetres
and 1–3°, then stopped 12 cm before the seated position (V apex at
0.43–0.45 m instead of 0.31 m) with a tyre caught on the funnel, and
pivoted 11–18° while the docking controller kept pushing. Today the only
escape is Nav2's `dock_approach_timeout` (60 s), which fails the action
outright — the docking server retries only on *failed charging*, never
on a blocked approach — so the robot grinds for a minute, then reports a
generic failure. Pivoting while blocked was observed to push the tyre
deeper, never to free it.

## 2. Design (all in `dock_manager`, no Nav2 changes)

### 2.1 Detection

Inputs the manager already has: docking feedback (`CONTROLLING`), the V
measurement `detected_dock_pose` (apex distance in the lidar frame), the
seat microswitch.

Stall := state `approaching` with feedback `CONTROLLING` **and** the V
apex distance has not decreased by more than `stall_progress_m` (0.02 m)
for `stall_time_s` (3 s) **and** not seated **and** apex distance <
`stall_zone_m` (0.7 m — only the final stretch; farther out the graceful
convergence may legitimately dwell).

Without a fresh V detection (dock offline) no stall is declared — the
approach then runs to the server's own timeout as today.

### 2.2 Reaction

1. **Cancel** the `DockRobot` action (existing cancel path, 5 s watchdog).
2. **Back off**: publish `TwistStamped` on the bridge's own velocity
   input (`cmd_vel_web`, twist_mux priority 75 — above Nav2, below the
   joystick) at +0.15 m/s, zero angular, for `stall_backoff_m` (0.15 m,
   measured on the V apex distance or 1.5 s as a fallback). Straight
   along the robot's own axis — no steering while a tyre may still touch.
   Status: `state: approaching`, `feedback_state: STALL_BACKOFF`.
3. **Retry**: re-issue the dock flow (`DockRobot` with
   `navigate_to_staging_pose: true` — the robot is ~0.9 m from staging,
   so Nav2 takes it back out and the guided approach starts fresh;
   `retries` in the status counts these too).
4. After `stall_max_retries` (2) → terminal `failed`, reason `stalled`
   (add to 05 §5.3 `DOCK_REASON_TEXT`: "robot got stuck at the dock
   mouth — check the funnel / tyres"). The charge permission is untouched
   (never disabled during a dock approach).

### 2.3 Interlocks

- The drive-out guard ignores the manager's own back-off (it already
  ignores motion while a docking action is in flight — keep the state
  `approaching` during the back-off).
- E-stop / cancel during the back-off stop the twist immediately (zero
  twist published once, then the 0.5 s mux deadman does the rest).
- Link loss: same policy as docking (continue).

### 2.4 Parameters (`docking:` section of `topics.yaml`)

| Param | Default | Meaning |
|---|---|---|
| `stall_handling_enabled` | true | feature switch |
| `stall_zone_m` | 0.7 | only within this apex distance |
| `stall_progress_m` | 0.02 | "no progress" threshold |
| `stall_time_s` | 3.0 | dwell before declaring a stall |
| `stall_backoff_m` | 0.15 | straight forward back-off |
| `stall_max_retries` | 2 | then `failed` / `stalled` |
| `drive_cmd_out_topic` | `cmd_vel_web` | mux input used for the back-off |

### 2.5 Not in scope

- No wiggle in place (observed to make it worse).
- No change to the funnel — if the geometry wedges the tyres at perfect
  alignment, the mechanism retries twice and reports; the fix is
  mechanical (04 §10.11).
- Nav2's `use_stall_detection` (joint-state velocity/effort) is not used:
  the hoverboard driver publishes no effort, and the server would not
  retry on it anyway.

## 3. Test plan

- [ ] Bench (motors OFF, robot at ~0.5 m from the dock, V in view): send
      `dock` — the approach cannot move → stall declared after 3 s →
      cancel → back-off twist visible on `cmd_vel_web` → retry → second
      stall → `failed` / `stalled`. Status transitions and reason checked.
- [ ] Field: provoke a wedge (small obstacle at the funnel mouth) →
      back-off frees the tyre → retry docks; remove the obstacle, confirm
      normal dockings unaffected (no false stalls during the slow last
      centimetres before the seat switch).

## 4. Effort

~120 lines in `dock_manager.{hpp,cpp}` + config parsing + topics.yaml +
05 reason text; one bridge rebuild + restart (no bringup restart, no
heading loss).
