# Console Firmware — Host-Loss Laser Watchdog + USB-Wedge Fix (Spec)

**Date:** 2026-06-13
**Target repo:** `openmotion-console-fw` (STM32H743)
**Status:** Spec only — companion to
`2026-06-13-disconnect-reconnect-hardening-design.md` in `openmotion-sdk`.
This document does **not** implement firmware; it specifies what the console
firmware must do so the host SDK can keep the system safe across USB
disconnects.

## Why

Empirical bench characterization (openmotion-sdk, 2026-06-13) found:

- With the trigger running (`TriggerStatus=2`, laser ON), **yanking the USB
  link leaves the laser firing** — the host can no longer send `stop_trigger`,
  and the console has **no autonomous way to stop**.
- Cutting USB **while the trigger is running** further leaves the console's
  **command interface unresponsive** after re-enumeration (the COM port
  returns, but no command is answered) until a **mains power cycle**.

The console is mains-powered; USB is only the data link. So a USB disconnect
does not de-power the laser. Host-side mitigations (bounded reconnect,
force-safe-on-connect) cannot help while the host cannot talk to the console.
**Only the firmware can guarantee laser-off when the host disappears.**

## Requirement 1 — Host-loss laser watchdog

While the trigger is active, the console firmware MUST stop the trigger (laser
off) if it has not heard from the host within a bounded window.

- **Trigger condition:** trigger running AND no qualifying host activity for
  `T_watchdog` ms.
- **Qualifying host activity:** any received host command on the
  command interface (UART/CDC), or a dedicated heartbeat (see Requirement 3).
- **Action on timeout:** execute the same path as `OW_CTRL_STOP_TRIG`
  (laser off, trigger disarmed), and latch a status bit indicating the
  watchdog fired so the host can observe it on reconnect.
- **`T_watchdog` selection:** longer than the worst-case normal gap between
  host commands during a scan (so it never trips mid-scan), short enough to be
  acceptable for laser safety. Proposed starting point **500 ms–1 s**, made a
  compile-time/config constant. Must be validated against the host's actual
  command/telemetry cadence during a scan.
- **Recovery:** after the watchdog fires, a normal `set_trigger` +
  `start_trigger` from the host must re-arm cleanly.

## Requirement 2 — Command interface must survive USB disconnect

A USB disconnect (host yanks the link / hub powered down) while the trigger is
running MUST NOT leave the command interface permanently wedged.

- On USB suspend / disconnect / reset detection, the firmware MUST reset the
  command-interface receive path (CDC/UART RX state machine, any DMA ring, and
  parser state) so that, after the host re-enumerates and reconnects, commands
  are parsed and answered **without a mains power cycle**.
- Acceptance: after a USB cut-while-triggering and restore, a host `ping`
  (`OW_CMD_PING`) receives a normal response within the host's connect window
  (~2 s), with no power cycle.
- Investigate root cause: a blocked ISR, a stalled DMA on the abruptly-severed
  endpoint, or a parser stuck mid-frame are the likely culprits.

## Requirement 3 — Heartbeat protocol (decision)

Choose one and document it in `CommandHandling.md`:

- **Option A — piggyback:** treat *any* received command (including the
  existing telemetry polls the host already issues at ~10 Hz) as the heartbeat.
  No new opcode. Simplest; couples the watchdog to existing host polling
  cadence.
- **Option B — dedicated heartbeat opcode:** add an explicit
  `OW_CTRL_HEARTBEAT` the host sends on a fixed interval while a scan is armed.
  Decouples safety from incidental traffic; requires an SDK change to emit it.

Recommendation: **Option A** to start (no protocol change, leverages the
existing ~10 Hz telemetry polling which is well within a 500 ms–1 s window),
with Option B as a fallback if telemetry cadence proves too irregular.

## Host-side observability (already available)

- `get_trigger_json()` returns `TriggerStatus` (`2` = ON, `1` = OFF) — a live
  state read the host (and HIL tests) use to verify the laser state.
- Add (firmware) a **watchdog-fired latch** to the trigger/status payload so the
  host can log that an auto-safe occurred and surface it to the operator.

## Out of scope

- Sensor firmware (the sensors do not control the laser).
- Any change to the histogram/IMU streaming paths.
- The host-side SDK changes (covered by the companion SDK design doc).
```
