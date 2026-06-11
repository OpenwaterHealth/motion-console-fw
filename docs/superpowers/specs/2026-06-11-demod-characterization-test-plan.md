# Demod Frame Characterization — Test Plan

**Date:** 2026-06-11
**Feature under test:** console-fw `feature/demod-frames` + SDK `feature/demod-config`
**Companion design doc:** [2026-06-11-demod-frames-design.md](./2026-06-11-demod-frames-design.md)

## Intent and hypotheses

The demod frame feature was speculative — the FPGA hooks were never driven
before this branch, and the system designer did not anticipate flashing
modulation for a single 25 ms frame. This plan characterizes what actually
happens.

Physical intent: on a demod frame, the DDS-modulated seed sweeps fast enough
within the camera exposure to wash out the speckle pattern, so **contrast
collapses toward 0 while mean intensity stays identical**.

| # | Hypothesis | Phase |
|---|---|---|
| H0 | The Seed FPGA image on our bench actually implements the modulation registers (gate) | 0 |
| H1 | Modulation ON (steady state) collapses contrast; mean unchanged | 2 |
| H2 | A single flashed frame reaches the same contrast floor as steady state (ring-up < ~20 ms lead) | 3–4 |
| H3 | Modulation is fully off by the next frame (ring-down < ~20 ms lag); K±1 frames unaffected | 3–4 |
| H4 | Demod frames land exactly on schedule and the PDC `demod_slot` flags agree | 3 |

### Decision tree

- **Phase 0 fails** (regs NACK / read zeros) → stop; sync with Henry — the
  bench FPGA image may predate the feature.
- **Phase 2 fails** (no contrast drop even with continuous modulation) → it's
  a values problem (frequency word / modulation current), not an interleave
  problem. Pull the Phase 5 sweeps forward; get George's tested I2C values.
- **Phase 3 shows partial suppression** (single-frame floor worse than
  continuous floor) → ring-up is slow; run the Phase 4d dwell-length sweep to
  measure the time constant in frame units.
- **K+1 neighbors show contrast deficit** → ring-down is slow; quantify vs
  trigger frequency (4c) and decide between accepting a K+1 exclusion window
  in analytics vs hardware follow-up.

## Setup

- Console (this branch) + **one** sensor module + **static scattering
  phantom**, fixed geometry, laser/TEC warmed up ≥ 10 min. A static phantom is
  mandatory: it gives a high, stable baseline contrast; any motion destroys
  the measurement.
- SDK from worktree `C:\Users\ethan\Projects\.worktrees\sdk-demod-config`.
- Default scan: 40 Hz, 60 s (~2400 frames) unless stated.
- **Bench caution:** the bench sensor has the known HISTO EP wedge on
  mid-stream stops — power-cycle (`tests/shelly.py cycle` in sensor-fw)
  between streaming runs if captures start failing.
- Record for every run: fw `VERSION`, Seed FPGA `REVISION/MINOR/MAJOR/ID`,
  full trigger JSON, full demod JSON, modulation current + current limit,
  phantom ID, camera mask.

## Metrics

Per frame, per camera (from raw histograms via the pipeline's MomentsStage,
or the standard CsvSink output):

- `u1` — mean intensity (pedestal-subtracted where relevant)
- `K` — contrast = std / (u1 − pedestal)
- PDC: `pdc_mA`, `dark_slot`, `demod_slot` per frame

Aggregates per frame-population (normal / dark / demod / demod±1):
median ± IQR. Headline ratios:

- **S** = K(demod) / K(normal) — suppression ratio (want → 0)
- **F** = K(demod, single-frame) / K(modulation continuous) — floor ratio
  (want ≈ 1; F < 1 is impossible, F ≫ 1 means ring-up too slow)
- **M** = u1(demod) / u1(normal) — mean ratio (want ≈ 1)

**Frame classification is by signature, not index.** Demod: normal u1, low K.
Dark: u1 ≈ pedestal. Normal: everything else. Then verify the demod set is
periodic with the configured interval at constant offset, and that its count
matches the PDC `demod_slot` count. Never assume console↔sensor frame-index
alignment.

## Phase 0 — Register sanity (laser disabled)

1. `OW_CTRL_I2C_RD` Seed `ID` (0x16, expect 1), `REVISION/MINOR/MAJOR`
   (0x13–0x15) — record. NACK/zeros → stop (decision tree).
2. `set_demod_config` with distinctive words (e.g. freq `0x0DDC0FFE`, phase
   `0x0ABC`), then `I2C_RD` regs 0x0A–0x0D and 0x00–0x01 raw. **This is the
   first real-hardware check of byte order** — the firmware writes LSB-first
   per the memory map's D[7:0]-in-first-register convention.
3. `get_demod_config` echo matches what was set.
4. Manual `I2C_WR` Static Control 0x20 = 0x01 → read back → 0x00; confirm
   no faults in Seed `STATUS` (0x12) and safety telemetry.

## Phase 1 — Baseline (demod disabled)

Standard scan, `DemodPulseInterval = 0`. Establishes per-camera K and u1
distributions, normal dark cadence, clean safety telemetry, zero
`demod_slot` flags, no Seed FPGA I2C traffic (firmware log quiet).

## Phase 2 — Continuous-modulation reference (the gate)

Configure freq/phase (+ modulation current/limit via `SEED_DDS_GAIN` /
`SEED_DDS_CL` or motion config), strobe configure, then **hold modulation ON
for an entire scan** via manual `I2C_WR` 0x20=1 before trigger start (bypasses
the interleave logic on purpose).

- Expect contrast collapse scan-wide and mean within a few % of Phase 1.
- Record the **steady-state floor** K_mod per settings — every single-frame
  result is judged against this floor.
- If the mean shifts: note magnitude; modulation drive may be asymmetric
  around the CW operating point → flag for laser-team rebalance before
  claiming "intensity identical".

## Phase 3 — Single-frame interleave (the feature)

- Run A: `DemodPulseInterval` **coprime** with the dark interval (e.g. demod
  every 10, darks every 7) — collisions rare, clean statistics.
- Run B: intervals sharing multiples — deliberately exercise the
  dark-wins-on-collision skip and confirm the demod slot is skipped, not
  shifted.

Analyze: demod cluster present and periodic; **S**, **F**, **M**; K±1
neighbor distributions vs normal population; PDC flag agreement; PDC drop
counters; safety clean.

**Provisional acceptance:** F ≥ 0.9 equivalent (single-frame K within ~10% of
the continuous floor), M within 2%, K±1 within 2σ of the normal population.

## Phase 4 — Ring-up / ring-down characterization

Timing context at 40 Hz: firmware writes modulation ON ~3–5 ms after pulse
K−1 (≈20 ms of ring-up lead before pulse K), and OFF ~3–5 ms after pulse K
(≈20 ms of ring-down lag before pulse K+1).

- **4a.** F from Phases 2–3: F ≈ 1 → ring-up completes within ~20 ms. Done.
- **4b.** Trigger-frequency sweep 40 / 20 / 10 Hz (lead ≈ 20 / 45 / 95 ms).
  F improving with longer lead brackets the ring-up time between the two
  lead times.
- **4c.** K+1 neighbor deficit vs the same sweep brackets ring-down.
- **4d.** (only if F < 0.9 at all frequencies) Small firmware extension:
  `DemodFrameCount = M` consecutive demod frames per event (M = 1, 2, 4, 8).
  Plot K vs position-in-block — a direct readout of the ring-up time
  constant in frame units; block frame 1 reproduces the single-frame case.
- **4e.** (optional, electrical) Scope the seed monitor photodiode / board
  test point with Henry or George for sub-ms ring measurement; sample
  `SEED_ADC_CD`/`SEED_ADC_VD` before/during modulation for an electrical
  cross-check.

## Phase 5 — Parameter sweeps

With the interleave fixed at a working point:

- **Frequency word**: log-spaced sweep (ref clock unknown → sweep raw words;
  record words, derive Hz once the DDS datasheet formula is confirmed).
  Expect deeper suppression as modulation period ≪ camera exposure.
- **Modulation current** (`SEED_DDS_GAIN`): suppression vs drive; find the
  minimum current that reaches the floor. Set `SEED_DDS_CL` appropriately
  first and watch for safety-FPGA faults at high drive.
- **Phase word**: spot-check 0 / π/2 / π — expected no effect for
  single-source washout; confirm.

Output: map of K floor vs (freq word, mod current); recommended operating
point = minimum settings that reach the floor.

## Phase 6 — Robustness / safety

- 30-min interleaved soak: no missed/extra demod frames (flag count vs
  expected), PDC drops nominal, TEC stable, safety status clean, no demod
  I2C failure logs.
- Trigger start/stop ×20 with demod enabled: modulation forced OFF each stop
  (verify via 0x20 readback between runs); no stuck-ON state.
- `set_demod_config` mid-scan (new interval, new freq): takes effect, no
  glitch, no fault from the 0x22 strobe mid-stream.
- Disable mid-scan (`interval = 0`): modulation forced OFF within a frame.
- Safety interlock trip during a demod frame: clean stop, modulation OFF.

## Deliverables

- Per-run CSVs + a plot per run: contrast vs frame index with demod (and
  dark) frames highlighted; K histograms by frame class; summary table of
  S / F / M per configuration. Store under `scan_data/demod-characterization/`.
- A short findings memo updating the design doc with: measured floor,
  ring-up/ring-down bounds, recommended operating point, and whether the
  single-frame flash is viable or the dwell extension (4d) is needed.

## Known unknowns / dependencies

| Item | Status |
|---|---|
| DDS reference clock & freq-word formula | Unknown — datasheet + Henry; sweep raw words meanwhile |
| Tested freq/current values from FPGA bring-up | Ask George (per email thread) |
| Frequency-word byte order on real hardware | Verified in Phase 0 step 2 |
| Mean-intensity shift under modulation | Measured in Phase 2; laser team if non-zero |
| Bench sensor HISTO wedge | Power-cycle between streaming runs |
| Safety FPGA response to modulated drive | Watched throughout; limits set before sweeps |
