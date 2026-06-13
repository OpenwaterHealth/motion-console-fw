# Demod Modulation Chain — Hardware Verification Plan

**Date:** 2026-06-13
**Purpose:** independently verify the three analog claims from the remote
characterization ([findings doc](./2026-06-12-demod-characterization-findings.md),
addenda 1–2) with probes on the unified console board.
**Companion:** seed-fpga PR #3 (digital fixes, all counter-verified).

The claims to verify:

| # | Claim (measured remotely, PDC/camera only) | Verdict if confirmed |
|---|---|---|
| C1 | Modulation folds back above gain word ~860; old `DDS_CL = 864` is calibrated to an analog clip point | Depth ceiling ≈ ±12 mA is a hardware limiter, not firmware |
| C2 | Modulation amplitude rolls off ~10× between 6.1 kHz and 1.5625 MHz (driver BW ~tens of kHz) | The MHz regime the washout needs is undeliverable |
| C3 | Deliverable modulation is AM-heavy (−8.8 % mean power at full depth) with insufficient chirp for washout | Feature physics requires an analog re-spec |

## Setup (15 min)

- Saleae (analog channels needed; Logic 8 / Pro 8 fine) or scope.
- Ground: any `SYS_GND` test point (TP3 area on sheet 20 is convenient).
- Probe points (Unified Console schematic 700-00010 rev 0.5, sheet 20):

| Ch | Point | Package / access | What it shows |
|---|---|---|---|
| A0 | **U22 (AD5689R) VOUTA** | LFCSP — micro-grabber on pin or series R pad | Gain DAC output (drives modulation amplitude) |
| A1 | **U19 (AD9837) VOUT** (pin 10) | LFCSP — or the 49.9 Ω R21 pad | DDS triangle source, ~600 mVpp |
| A2 | **U14 (AD633) W** (pin 7) | SOIC-8 — easy grabber | Multiplier output = scaled modulation + CW sum |
| A3 | **CW_MODULATE** net (into the OPA27 V-to-I) / TP3 | test point | Signal actually driving the seed current stage |
| D0–D2 | U22 SCLK/SYNC/SDIN (pins 10/11/12) | optional | SPI decode: words actually reaching the DAC |

- Software side: I drive all stimulus from this machine
  (`scripts/demod_characterization.py` + inline I2C). Tell me when probes
  are on; each experiment below is one command for me.
- Note: experiments E1/E2 need `DDS_CL` temporarily raised 864 → 1111
  (the +20 mA-equivalent step already used remotely); restored after.

## E1 — Depth ceiling / foldback (verifies C1) — ~20 min

Stimulus: laser params applied, modulation configured at **6.1 kHz**
(freq word 65535), trigger running, armed. I step the gain word as a slow
staircase: 0 → 200 → 400 → 600 → 800 → 860 → 880 → 920 → 1000 → 1100,
4 s per step.

Watch:
1. **A0 (VOUTA)**: should be a clean monotonic DC staircase, ~76 µV/LSB
   (860 ≈ 66 mV, 1100 ≈ 84 mV). If the staircase tracks all the way up →
   DAC and FPGA SPI are fine and the foldback is downstream (expected).
   If VOUTA itself misbehaves → revisit; D0–D2 SPI decode arbitrates.
2. **A2 (AD633 W) and A3 (CW_MODULATE)**: triangle amplitude should grow
   with the staircase until the clip point, then visibly distort/clip/
   collapse between steps 860 → 880. **The waveform at the moment of
   collapse is the money capture** — clipping (flat tops), foldback
   (amplitude drops), or protection chatter (bursts) distinguish the
   mechanism.
3. Note the exact VOUTA voltage at which the collapse occurs → that is
   the analog limiter threshold the firmware gate should clamp to.

## E2 — Driver bandwidth (verifies C2) — ~15 min

Stimulus: gain word fixed at 800 (safely below clip), armed, trigger
running. I step the DDS frequency word: 6.1 kHz → 24 kHz → 98 kHz →
390 kHz → 1.5625 MHz (words 65535 / 262143-ish [23:16]=0 constraint:
use 0x00FFFF, 0x03FFFF is NOT reachable on stock images — fixed image is
on the bench, all words fine), 5 s per step.

Watch A3 (CW_MODULATE) and, if probed, the seed current sense node:
- Measure modulation amplitude at each frequency → Bode points.
- Expected if C2 holds: amplitude ≈ flat to some corner in the tens of
  kHz, then rolling off ~20 dB/decade; ~10× down at 1.5625 MHz.
- Deliverable: the corner frequency — the single number the laser team
  needs for the driver re-spec.

## E3 — AM vs chirp at the laser (verifies C3) — ~20 min, optional scope-only

This one needs an optical discriminator, not just electrical:
- Easiest: the seed monitor photodiode / PDC analog node on a scope while
  modulating at 6.1 kHz, word 800: the depth of the power envelope IS the
  AM fraction (remote measurement says ~9 %). If an optical spectrum
  analyzer or wavemeter is available (probably not on this bench), the
  chirp could be measured directly; otherwise C3 rests on: measured AM +
  measured null contrast effect + the unmeasured chirp coefficient.
- Also worth one capture: A1 (DDS VOUT) during per-pulse gating at
  1.5625 MHz with the trigger running — confirms the DDS output truly
  starts/stops per pulse (the counters say it does; this is the visual).

## Wrap-up

- Restore `DDS_CL = 864`, gain word 0 (I do this; verify via `sanity`).
- Record: clip-point VOUTA voltage (E1), driver corner frequency (E2),
  AM depth (E3) → append to the findings doc; forward with PR #3 to
  Henry + Brad for the re-spec decision.

## Reference numbers from the remote characterization

| Quantity | Value |
|---|---|
| Max effective gain word | ~860 (collapse 860→880) |
| PDC mean-power shift at 860, 6.1 kHz | −8.8 % (≈ ±12 mA peak) |
| Same word at 1.5625 MHz | −0.8 % (~10× rolloff) |
| Contrast change, all configs | ≤ ~1 % (null) |
| DDS: AD9837, MCLK 25 MHz (ECS-2033-250) | f = word × 25 MHz / 2²⁸ |
| Gain DAC: AD5689R VOUTA, ~76 µV/LSB | word 860 ≈ 66 mV |
