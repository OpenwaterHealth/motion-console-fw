# Demod Frames — Design

**Date:** 2026-06-11
**Branch:** `feature/demod-frames` (off `next`)
**Sources:** "Demod Frames" email thread (E. Head / H. Tang / B. Hartl, Sep 2025); Unified Board FPGA Memory Map 700-00010 rev 0.8 (H. Tang).

## Background

The Seed FPGA on the unified board drives a DDS modulation device. The plan
(per B. Hartl) has always been to interleave **dark** and **demod** frames
periodically among normal laser frames; the demod frames carry modulated
light for demodulation processing. The FPGA-side hooks exist, but the
console firmware never drove them — this branch adds that.

## Register interface (Seed FPGA, I2C addr 0x41, TCA9548 mux 1 / channel 5)

| Reg | Function |
|---|---|
| `0x00–0x01` | Modulation phase word, `PHASEREG[11:0]`; phase = 2π/4096 × PHASEREG |
| `0x02–0x03` | Modulate current (16-bit DAC, 20 µA/step) — already settable via `SEED_DDS_GAIN` register map entry |
| `0x06–0x07` | Modulate current limit (19 µA/step) — already settable via `SEED_DDS_CL` |
| `0x0A–0x0D` | Modulation frequency word (32-bit) — register-map entry `SEED_DDS_FREQ` added by this branch |
| `0x20` | STATIC CONTROL: `D[0]` = 1 modulate ON, 0 OFF |
| `0x22` | DYNAMIC CONTROL (write-only): `D[0]` = 1 → modulation configure (FPGA loads frequency + phase into the modulation device) |

Configure sequence (per H. Tang): write frequency + phase registers →
strobe `0x22` `D[0]` → toggle `0x20` `D[0]` for modulation on/off. For
cycle-by-cycle interleaving, the MCU writes the modulation-on register at
the end of the trigger pulse preceding the demod pulse and clears it after
the demod pulse falls.

## Firmware design

New module `Core/Src/demod.c` / `Core/Inc/demod.h`:

- **Config** (`Demod_Config_t`): `demodPulseInterval` (every Nth laser cycle
  is a demod frame; 0 = feature disabled), `modFrequencyWord`,
  `modPhaseWord`. Frequency/phase are **raw register words** — the
  Hz/radian conversion formulas live in the DDS data sheet and depend on
  its reference clock, so they are the host's responsibility.
- **Host surface**: `OW_CTRL_SET_DEMOD` (0x29) / `OW_CTRL_GET_DEMOD` (0x2A),
  JSON payload mirroring the trigger-config commands. `SET` stores the
  config and, when a frequency or phase word is supplied, writes it to the
  Seed FPGA and strobes the configure bit. Fields absent from the JSON keep
  their current values.
- **Scheduling**: cycle K is a demod frame iff
  `interval > 0 && K >= NUM_DARK_FRAMES_AT_START && K % interval == 0 && !dark(K)`.
  Dark frames win on collision (that demod slot is skipped, not shifted),
  keeping both patterns deterministic for the host. The dark formula is
  factored into `Trigger_CycleIsDark()` so trigger and demod cannot drift.
- **Runtime mechanism** (mirrors the proven `pdc_poll` ISR→main-loop
  pattern): the LSYNC rising-edge ISR records the cycle number + tick
  (last-value-wins, no queue — the state machine self-corrects from the
  newest cycle). `Demod_Tick()` in the main loop waits until the cycle's
  laser pulse has completed (delay + skip delay + width + 2 ms), then
  writes `0x20` `D[0]` = `Demod_CycleIsDemod(cycle + 1)` — only on state
  changes, so steady state costs zero I2C traffic. A staleness guard skips
  the write if the main loop stalled past one frame period (never flip
  modulation mid-pulse).
- **Safety/cleanliness**: modulation is forced OFF at trigger start (unknown
  hardware state), trigger stop (including safety/USB interlock stops —
  the OFF request is serviced from the main loop since `Trigger_Stop` can
  run in ISR context), and when the host sets `interval = 0`. With
  `interval == 0` (default) the module performs **no** Seed FPGA I2C
  traffic at all — zero behavior change for existing systems.
- **Data tagging**: PDC samples gain `flags` bit 1 = `PDC_FLAG_DEMOD_SLOT`
  (additive next to the dark bit) so analytics can identify demod frames in
  the per-frame PDC stream.

## Out of scope / follow-ups

- SDK (`omotion/MotionConsole.py`): add `set_demod_config()` /
  `get_demod_config()` wrappers and the Hz→frequency-word formula once the
  DDS reference clock is confirmed (per the email thread: "check the data
  sheet" / "check with George").
- Modulation current + current limit defaults: settable today via the
  persistent motion-config JSON (`SEED_DDS_GAIN`, `SEED_DDS_CL`) or
  `OW_CTRL_I2C_WR`; actual values TBD by laser bring-up.
- Hardware validation: scope the seed drive / photodiode during an
  interleaved run and confirm the demod pulse is modulated and neighbors
  are not.
