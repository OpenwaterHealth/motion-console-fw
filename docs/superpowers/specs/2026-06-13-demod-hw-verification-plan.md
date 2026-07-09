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

> **REVISED 2026-07-08 after schematic re-verification — three errors in the
> original table are corrected below; see the addendum at the bottom for the
> full trail.** Original errors: (1) AD633 W cited with the PDIP pin number —
> the fitted AD633JRZ is SOIC-8 (Y1=1, Y2=2, −VS=3, Z=4, **W=5**, +VS=6,
> X1=7, X2=8); pin 7 is the X1 *input*. (2) R21 (49.9 Ω) is the 25 MHz MCLK
> series resistor from oscillator Y1, not a DDS VOUT pad. (3) TP3 is a
> SYS_GND test point — ground clip only, not a CW_MODULATE access.

- Saleae (analog channels needed). **Logic Pro 8 preferred** — the plain
  Logic 8's analog path (~10 MS/s, ≈1 MHz BW) self-attenuates at the
  1.5625 MHz E2 point; if only a Logic 8 is available, take the top Bode
  point with a scope or correct for the analyzer's own rolloff.
- Ground: any `SYS_GND` test point (TP3 on sheet 20 is convenient).
- Probe points (Unified Console schematic 700-00010 rev 0.5, sheets 19–20):

| Ch | Point | Package / access | What it shows |
|---|---|---|---|
| A0 | **U22 (AD5689R) VOUTA** | DAC-side pad of R31 (10K) | Gain DAC staircase. **Log the absolute voltage at word 860** — ~66 mV means the FPGA writes raw 16-bit LSBs; ~4.2 V means a left-justified short word. Instantly settles the word-scaling question. |
| A1 | **U19 (AD9837) VOUT** (pin 10) | U19-side pad of R7 (510 Ω) | DDS triangle source, ~600 mVpp |
| A2 | **TP1 (sheet 19)** — CW_MODULATE at the seed-driver input | dedicated test point — easiest clip on the board | Same net as AD633 W (pin 5): multiplier output = scaled modulation + CW sum. No SOIC grabber needed. |
| A3 | **U16 (MAX4372FEUK) OUT** (sheet 19, SOT23-5) | easy grabber | **Actual seed current** — 50 V/V across the 10 mΩ shunt R91 → 0.5 V per amp. Command (A2) vs. delivered (A3) side-by-side localizes both the clip and the rolloff. |
| A4 | optional: **U54 (OPA27) OUT** (pin 6) or X1-side of R17 | pad | Isolates the U54 gain stage for E2 (slew-limit hypothesis below) |
| D5–D7 | U22 SCLK/SYNC/SDIN (pins 10/11/12) | optional | SPI decode: words actually reaching the DAC |

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
| Gain DAC: AD5689R VOUTA, ~76 µV/LSB | word 860 ≈ 66 mV **if raw LSBs — unverified, see A0 note** |

## Addendum — schematic re-verification (2026-07-08)

Full pass over sheets 19–20 before the bench session. Corrections already
folded into the probe table above; new facts and sharpened hypotheses:

### Architecture facts (sheet 20, CW & MODULATE)

- **AD633 X input is differential with a 2.05 V offset subtraction**: X1 =
  the OPA27-buffered DDS triangle (via R17 49.9 Ω); **X2 = +2.05 V** from
  the TS4061VIBT-205 shunt reference (IC1), bypassed by C319 4.7 µF. So
  `W = (triangle − 2.05 V) · GAIN/10 + CW`. C319 sits on the *reference*,
  not the signal — no LPF on the modulation path here.
- **Direct AM mechanism for C3**: any mismatch between the triangle's mean
  and 2.05 V produces a DC product term ∝ GAIN — i.e. raising the gain word
  *shifts mean power* even before any clipping. E3 should log mean(X1) vs.
  2.05 V; if mean(X1) ≠ 2.05 V, the −8.8 % PDC shift may be mostly this
  offset error (a calibratable artifact!), not inherent AM.
- Y1 = GAIN net ← U22 **VOUTA** through R31 10K + C95 0.1 µF (RC ≈ 159 Hz,
  τ ≈ 1 ms — fine at 4 s/step; note for firmware Plan B: per-frame gain
  writes settle in ~5 ms through this RC, not µs). Y2 = GND. Z = CW ←
  VOUTB through R35/C96 (same RC).
- U54 (OPA27) is a ~×6.9 non-inverting gain stage (R3 3K / R4 510) buffering
  the DDS — it is *not* the V-to-I; the sheet-20 attribution in the findings
  doc was wrong.

### Architecture facts (sheet 19, SEED DRIVER — the actual V-to-I)

- Chain: CW_MODULATE → **TP1** → R329 1K → U99B/U99C/U99D (AD8674 quad) →
  R333 1K → **U100 OPA547F** (power op-amp) → R331 4.02 Ω → R91 10 mΩ →
  SEED. Annotations: "2.06 V @ 500 mA" across R331; power table CW 400 mA /
  MODULATE 300 mA max.
- **U100 has a current-limit pin: R335 21K sets OPA547 ILIM** — a concrete
  candidate for the E1 foldback (peak clipping of the modulation triangle →
  mean drops, no status flags, exactly what PDC saw).
- **Compensation poles in the U99 chain**: C489 270 pF ∥ R326 10K ≈
  **59 kHz** (dominant), C492 1000 pF ∥ R327 510 ≈ 312 kHz, C488 270 pF ∥
  R325 1K ≈ 590 kHz — matches "driver BW ~ tens of kHz" (C2). Additionally
  **U54's OPA27 slew limit (1.9 V/µs) caps a full-swing triangle above
  ~230 kHz** on sheet 20 — E2 with A1 vs A4 vs A2 vs A3 assigns the rolloff
  to a stage instead of inferring it.
- Monitoring: U16 MAX4372 (50 V/V) across R91 → seed-current waveform at
  0.5 V/A (probe A3); INA826 (U101, G=1.6) + LT6200 + LTC1440 comparator
  chain (SEED_COMPARED / SEED_VREF) — a second protection-ish path worth
  remembering if E1 shows chatter rather than clean clipping. TP2 sits in
  this monitor chain (U99A output via R338).

### What E1 should now settle in one capture

With A0 (VOUTA), A2 (TP1 = W), A3 (seed current) simultaneously: (a) the
word→DAC scaling (A0 absolute voltage), (b) whether the 860→880 collapse is
W-side (AD633/upstream) or current-side (OPA547 ILIM / V-to-I saturation),
(c) the exact commanded voltage at the clip → the value the firmware gain
gate should clamp to.
