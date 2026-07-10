# Demod Frames — next-session plan (post laser-team answer)

**Date:** 2026-07-09 (end of probe-session day)
**State:** feature functionally proven — washout to shot-noise floor on
cams 1–3 (A/B v17), fw interleave validated (S=0.469 on prior coupling),
three root causes fixed (DDS_CL ordering, seed_mod_ss pin, MOD-SCK CDC).
Console parked on production 1.8.0; seed FPGA left on fixed image 1.5.0.
**AUTHORIZATION WITHDRAWN 2026-07-10: laser engineering set max seed
current to 140 mA — the CW operating point itself. ALL modulation testing
halted pending a report (see 2026-07-10-seed-current-report-for-laser-team.md)
and a new approved envelope. Reduced demod-frame intensity is now accepted,
so the plan inverts: lower the modulation midpoint below 140 mA instead of
raising CW above it. The 180 mA text below is retained for history only.**

~~AUTHORIZATION (Ethan, 2026-07-09 EOD): peak seed current ≤ 180 mA for
demod testing.~~ CW gain-word ceiling = 2616 (0.0688 mA/step); set CW_CL
just above (raw-word gate). NOTE: FPGA CL registers gate DAC words, not
physical peaks — every session verifies the measured peak on the current
sense. Full compensation of the −42% sag at today's saturated swing does
NOT fit under 180 (needs ~178–186 mA before swing) → the swing-reduction
step below is load-bearing, not optional.

## Sequence under the 180 mA ceiling

0. Re-enter demod state: DFU demod fw, sanity (expect FPGA 1.5.0), write
   the ceiling registers FIRST.
1. **Measure delivered swing** at word 300 @ 49 kHz, CW 140 mA — needs the
   current-sense node correctly probed (prior A3 clip was on a wrong net;
   re-place per DVT-1A print). Output: swing_peak mA + compliance check of
   past runs vs 180.
2. **Washout knee — REVISED RANGE**: ladder words ~120–500 at 49 & 98 kHz.
   Re-reading v15: words ≤100 likely never modulated at all (single-sample
   verify false-passes; d25/d50 sat at baseline PDC and didn't wash) —
   the shape is a THRESHOLD near word 100–200 (candidate mechanism:
   AD633 Y-input offset, ±5–30 mV ≈ words 65–390 at 76 uV/LSB) then
   clipping above ~word 400–1000. Map threshold + linear window; smallest
   reliably-washing word + margin becomes the production gain word.
   Upgrade --verify-mod to 3-sample averaged PDC before this ladder.
3. **CW-trim calibration** (continuous mode, knee swing): closed loop
   against PDC — step demod-window CW word until armed-PDC ==
   disarmed-PDC (±2 %), hard cap word 2616, measured peak ≤ 180 verified
   on the sense. Expected landing: CW ≈ 155–165 mA + small swing. Record
   trim word + camera means.
4. **demod.c per-frame CW swap**: extend the interleave scheduler to write
   CW_trim on arm and CW_normal on disarm (two extra 2-byte I2C writes in
   the inter-pulse window; budget fine). New JSON field `DemodCwWord` in
   SET_DEMOD/GET_DEMOD + SDK setter. Rebuild, DFU, verify.
5. **Acceptance run** (this coupling or phantom): fw interleave N=10,
   60 s × 3 repeats → `analyze` per camera. Targets: S ≤ 0.35 on cams 1–3
   (shot-floor-limited), **M = 1.00 ± 0.05 on every camera**, K_nbr ==
   K_norm (no bleed), interval(frac) = 10 (≥0.95).
4. Repeat acceptance on the June phantom arrangement for the record.

## Fallback — if the knee sag exceeds the 180 mA headroom

1. Calibrate to the ceiling → partial compensation; measure residual ΔM
   and the far-camera SNR cost.
2. Decision memo to Brad/Henry: accept partial M, or prioritize the
   **analog re-bias** (center the modulation on the multiplier's 2.05 V
   reference / re-scale the summing so the swing is symmetric) — removes
   the compensation requirement entirely.

## Independent of the answer

- **DDS config-strobe reliability** (drops ~50 % of configures): proper RTL
  fix in `dds_control_interface` config handshake (same CDC family as the
  SCK bug) or a config-done readback register; until then the harness
  `--verify-mod` retry stands. Multi-sample PDC averaging in verify.
- **Per-frame PDC demod-slot flag** plumbing into frames.csv so `analyze`
  scores against firmware truth (adjudicates the adjacent-pair detections
  seen at interval 10).
- **Henry sync on seed-fpga PR #3** (4 fixes now) + the warning that the
  1.1.0 production lineage carries the latent SCK CDC hazard.
- Regenerate the findings deck — the June narrative (analog-infeasibility)
  is fully superseded.

## Bench-state notes for pickup

- Console: production **1.8.0**. Seed FPGA: **1.5.0** (fixed image —
  normal scans unaffected; demod work should keep it).
- Demod console fw: `feature/demod-frames` @ D[1] fix, hex prebuilt in the
  worktree (`build/Debug/motion-console-fw.hex`) — DFU + Shelly cycle to
  re-enter demod work (~4 min).
- Registers idle: gain 0, CL 988 (POR; laser params write 864 on connect),
  static 0x0000.
- Harness: sdk `feature/demod-config` — collect has `--verify-mod`,
  `--arm-delay`, `--rearm-interval`, `--static-word`, interleave D[1]
  prep. **Pool dark frames across runs** (≥2 runs or ≥30 s) before any K
  correction; 14 s runs catch ≤1 dark.
- Camera geometry (right module): cams 0/7 farthest from launch, 4/5
  closest; mask 0x0F = cams 0–3 (0 = far).

## Parameter reference — full demod control surface + planned test values

### Host demod config (SET_DEMOD JSON)
| Parameter | What it does | Prod/default | Test values |
|---|---|---|---|
| DemodPulseInterval | every Nth cycle = demod frame; 0 = off | 0 | 10 (acceptance), 5 & 20 (sensitivity), 0 for continuous cal runs |
| ModulationFrequencyWord | f = word x 25 MHz / 2^28; M = 2 f 500 us | - | 524280 (48.8 kHz, M~49), 1048560 (97.7 kHz, M~98); 65535 as low-M ref |
| ModulationPhaseWord | DDS start phase (12-bit) | 0 | 0 only (revisit on pulse-sync artifacts) |
| DemodCwWord (to add) | per-demod-frame CW trim word | n/a | calibrated, expected ~2255-2400 (155-165 mA) |

### Seed FPGA registers (I2C)
| Register | What it does | Prod | Test values |
|---|---|---|---|
| DDS gain 0x02 | mod amplitude DAC (76 uV/LSB); THRESHOLD ~word 100-200 (AD633 Y-offset candidate), clips above ~400-1000 | 0 | ladder 120-500 (threshold + linear window); 300 known-good ref |
| DDS_CL 0x06 | raw-word gate, silently drops | 864 | 1111 during tests; restore 864 |
| CW gain 0x04 | CW DAC, 0.0688 mA/step | 2037 (140.1 mA) | closed-loop trim up from 2037, HARD CAP 2616 (=180 mA authorized) |
| CW_CL 0x08 | raw-word gate (scale mismatch: 0.081 label vs 0.0688 gain scale) | 2048 | 2617 during trim; restore 2048 |
| Static 0x20 | D0 arm, D1 laser_active | 0x0000 | 0x0003 cont / 0x0003<->0x0002 interleave / 0x0000 teardown |
| Dynamic 0x22 | AD9837 configure strobe (~50% drop) | strobe | exercised via --verify-mod retry, count logged |

### Trigger context (unchanged)
40 Hz, 500 us pulses, skip 600/1800 (pool darks across runs for K correction).

### Harness knobs
--verify-mod (ratio 1.10, upgrade to 3-sample avg); --arm-delay 2.5 s short
runs; 15 s ladder / 60 s acceptance; right module 0x0F.

## Proposed values — principal variables (2026-07-09 EOD consolidation)

| # | Variable | Tomorrow (char.) | Production proposal | Basis |
|---|---|---|---|---|
| 1 | DemodPulseInterval | 10 (+5/20 spot) | 40 (pending Brad) | stats speed vs 2.5% frame cost |
| 2 | ModulationFrequencyWord | 524280 & 1048560 | 1048560 (97.7 kHz) | max sweeps/pulse per mA of swing |
| 3 | ModulationPhaseWord | 0 | 0 | no mechanism at M~100 |
| 4 | DDS gain (0x02) | ladder 120-500 | knee + ~30% (expect 250-400) | smallest washing swing; must clear Y-offset threshold |
| 5 | DDS_CL (0x06) | 550 | prod gain x1.25 | protection hugs operating point; 864 was never calibrated |
| 6 | CW gain (0x04) normal | 2037 | 2037 unchanged | laser team's operating point |
| 7 | DemodCwWord (new) | closed-loop, cap 2616 (=180 mA); expect 2255-2400 | per-unit factory cal + PDC-parity check | unit/temp-dependent L-I compensation |
| 8 | CW_CL (0x08) | 2617 transient, restore 2048 | 2048 normal; fw raises to trim+1 during demod scans only | keep normal-ops protection intact |
| 9 | Static (0x20) | 0x0003 / 0x0003<->0x0002 / 0x0000 | firmware-owned exclusively | host writes were scaffolding |
| 10 | Trigger | 40 Hz / 500 us / skip 600 | unchanged | calibration envelope |
| 11 | Verify policy | 3-sample PDC verify, ratio 1.10, retry <=4 | fw double-strobe + RTL fix before ship | single-sample verify false-passes |

## Overnight session scope (bench window 02:15, granted 2026-07-09)

Authorized unattended work, in order:
1. Re-enter demod state: DFU demod hex (worktree build/Debug) + Shelly
   cycle + sanity (expect FPGA 1.5.0). If console unresponsive: Shelly
   cycle, one retry, else PARK AND REPORT.
2. Ladder words 120/160/200/240/280/320/400/500 x {49k, 98k}, 15 s runs,
   3-sample verify-mod, DDS_CL 550. Saleae ch8-11 amplitude capture per
   step if Logic 2 cooperates (optical ladder proceeds regardless).
   Deliverables: threshold word, linear window, knee, sag(word).
3. CW-trim closed loop at the knee. UNATTENDED SUB-CAP: CW word <= 2450
   (~168.6 mA set-point) leaving >= 11 mA swing margin under the 180 mA
   peak authorization until the swing is measured in mA (attended AM step:
   move current-sense clip, verify peak, extend to 2616 if headroom).
   CW_CL 2451 transient during trim runs only; restore 2048.
4. A/B v18: A = regular, B = knee swing + trimmed CW -> per-camera K/mean
   table (pool darks!).
5. Code + build (NOT flash) demod.c DemodCwWord per-frame swap + SDK setter.
6. Park: registers restored (gain 0, DDS_CL 864, CW 2037, CW_CL 2048,
   static 0, trigger off), console LEFT on demod fw for the attended
   morning session, results to #171, runbook + memory updated.

Hard rules: no FPGA flashing; CW word never > 2450; DDS gain never > 500;
restore after every run; abort-and-park on anything anomalous (safety
flags, repeated I2C errors, verify never passing).
