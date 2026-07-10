# Seed-current report — demod-frame characterization

**Prepared for:** laser engineering
**Date:** 2026-07-10
**Status of testing:** HALTED per laser-engineering directive (max seed
current 140 mA). Bench restored to production firmware and configuration.
**Purpose:** give a complete, honest account of what seed-current waveforms
the demod-frame work commanded, what was actually delivered, what the
protection layers do and do not guarantee, and a proposed measurement plan
that stays at or below 140 mA. This document is the basis for any future
request to test above 140 mA.

---

## 1. Why the demod feature modulates the seed current

A "demod frame" is a camera frame whose speckle contrast is deliberately
collapsed by sweeping the seed wavelength during the 500 µs exposure
(current modulation → wavelength chirp → speckle decorrelation). The
feature is required to deliver low contrast at intensity similar to normal
frames. Contrast collapse is demonstrated (washout to the camera shot-noise
floor); intensity parity is not yet achieved — modulation currently costs
~42 % of mean optical power at the operating point tested. **Per
2026-07-10 direction, reduced intensity on demod frames is acceptable**,
which removes the need to raise any current above the normal operating
point for compensation.

## 2. Seed drive chain and protection layers (Unified Console, DVT-1A)

Signal path: CW DAC (AD5689R ch B) sets the operating point; a DDS triangle
(AD9837) times a gain DAC (AD5689R ch A) is summed onto it via an analog
multiplier (AD633); the sum drives an AD8674/OPA547 V-to-I stage
(4.02 Ω + 10 mΩ sense) into the seed.

| Layer | What it actually does | What it does NOT do |
|---|---|---|
| CW gain word (FPGA reg 0x04) | Sets CW DAC output; nominal 0.0688 mA/step (inferred from firmware comments — **never electrically calibrated**) | Does not bound the modulated peak |
| CW_CL (reg 0x08) | FPGA gate: silently drops CW-word writes ≥ this raw word | Not a physical current limit; different nominal scale (0.081 mA/step) than the gain word |
| DDS gain word / DDS_CL (0x02/0x06) | Modulation amplitude DAC and its raw-word write gate | Word→delivered-swing relation is nonlinear and **unmeasured in mA** |
| Over-current comparator (INA826 + LTC1440, "SEED_COMPARED") | Trips at ≈ 510 mA at the sense resistor | Far above the 140 mA envelope — no protection at these levels |
| OPA547 ILIM (R335 21 k) | Power-stage hardware current limit | Amp-class, not mA-class |

**Key honesty point:** every register "limit" in this system gates DAC
*words*, not measured current. There is presently **no instrumented
verification of delivered peak seed current** — the current-sense probe
placement was attempted but landed on the wrong net (board-revision
confusion, since resolved via the DVT-1A schematic). Instrumenting this
node is step 1 of the proposed plan.

## 3. Production operating point, as found on the bench unit

- `laser_params.json` writes CW word **2062** ≈ **141.9 mA nominal** (at the
  inferred 0.0688 mA/step). If the 140 mA directive is exact, note the
  shipped configuration is nominally ~1.9 mA above it — or the scale
  estimate is imprecise, which the calibration below resolves. Flagging for
  your interpretation.
- `laser_params.json` also writes CW_CL = **2469** (raw-word gate).
- Modulation (DDS gain) is 0 in production; the demod feature is disabled
  (`DemodPulseInterval = 0`).

## 4. History of modulation testing (complete)

All testing on one DVT-1A bench unit, operator present or on-call, 40 Hz ×
500 µs pulsed regime (2 % laser duty), runs 10–60 s. Registers restored to
production values after every run (verified by readback).

| Period | Config | What was commanded | What was actually delivered |
|---|---|---|---|
| Jun 11–12 (rounds 1–4, FPGA image 1.1.0) | CW 2062 + mod words 312–30000 @ 6.1 kHz–1.5625 MHz | Words ≥ 864 silently dropped by gate (DAC = 0); ≤ 863 accepted | Modulation delivered only for words ≤ 863; swing never measured in mA |
| Jun 12–15 (rounds 5–6, images 1.2.0–1.3.0) | CW 2062 + mod words ≤ 1060 (gate raised to 1111 under a +20 mA authorization) | Words accepted | **Nothing delivered** — these images carried an FPGA pin-constraint bug (FSYNC on the wrong ball): the DDS never received a single SPI word. All "modulation" observations were a DC path artifact |
| Jul 9 (image 1.5.0, bugs fixed) | CW 2062 + mod words 200–1060 @ 6.1–97.7 kHz; ~25 runs, 10–60 s each | Delivered — first genuine modulation since round 1 | Optical effects large (washout to shot floor; −42 % mean). **Delivered current swing unmeasured**; drive-chain nodes showed multi-volt saturated swings, so peaks plausibly exceeded 142 mA nominal by tens of mA during modulated intervals |
| Jul 9 late (final ladder) | CW 2062 + mod words 120–500 @ 48.8 kHz, 9 × 15 s runs | Delivered (subset; ~half of configure strobes drop) | Same unmeasured-swing caveat |

Authorizations in effect at the time: June "+20 mA over the gate value";
2026-07-09 "peak ≤ 180 mA" (Ethan). **Both are superseded by the 140 mA
directive and testing is halted.** No configuration ever raised the CW
set-point above the production word (2062); the compensation experiments
that would have (CW-trim to ~155–185 mA) were planned but never executed.

## 5. What we do not know (the gaps this plan closes)

1. **Word → mA calibration** of both DACs (nominal scales come from
   firmware comments, not measurement).
2. **Delivered modulation swing** (pk-pk mA) vs gain word — including
   whether historical runs' peaks exceeded 140/160/180 mA and by how much.
3. The waveform shape at the seed (the drive chain saturates; the modulation
   is not the small-signal triangle the design intended).
4. Whether washout survives at a reduced operating point (see §6) — early
   evidence says depth is not the binding constraint, which is favorable.

## 6. Proposed measurement plan — everything at or below 140 mA peak

Accepting reduced demod-frame intensity inverts the old approach: instead
of raising current on demod frames, we *lower* the modulation midpoint so
the modulated peak stays under the normal operating point.

1. **Instrument the current sense** (MAX4372 output, DVT-1A print, attended
   clip placement) and calibrate DC: step CW word 2062 → 1200 downward,
   record sense voltage per word. Yields the true word→mA curve. Never
   exceeds the production operating point.
2. **Measure swing vs gain word at a reduced midpoint**: set CW word ≈ 1500
   (~103 mA nominal), enable modulation at the smallest useful words, and
   measure delivered swing directly. Walk the midpoint/word grid upward
   only while measured peak ≤ 140 mA, with margin.
3. **Re-verify washout at the reduced midpoint** (cameras + PDC): if demod
   frames wash to the shot-noise floor at ~100–120 mA midpoint — plausible,
   since washout showed no depth-starvation at the words tested — the
   feature fits entirely under 140 mA and **no request for higher current
   will be made**.
4. Deliverables to laser engineering either way: measured word→mA curves,
   measured peak/swing per configuration, waveform captures at the sense
   node, and duty-cycle accounting. If (3) fails, a specific,
   instrument-backed envelope request above 140 mA follows, quantifying
   exactly the peak, duration, and duty needed and the SNR cost of the
   alternatives.

## 7. Standing controls during any future testing

- Peak enforcement by *measurement* (sense-node capture on every
  configuration change), not by register words alone.
- Word gates set to hug the approved envelope; registers restored to
  production values after every run, verified by readback.
- All runs logged (PDC, telemetry, safety flags) and reported on
  bloodflow-app issue #171.
