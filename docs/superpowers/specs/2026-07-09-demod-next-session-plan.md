# Demod Frames — next-session plan (post laser-team answer)

**Date:** 2026-07-09 (end of probe-session day)
**State:** feature functionally proven — washout to shot-noise floor on
cams 1–3 (A/B v17), fw interleave validated (S=0.469 on prior coupling),
three root causes fixed (DDS_CL ordering, seed_mod_ss pin, MOD-SCK CDC).
Console parked on production 1.8.0; seed FPGA left on fixed image 1.5.0.
Open decision with the laser team: CW headroom for demod-frame intensity
compensation (statement delivered to Ethan 2026-07-09).

## Branch A — laser team approves CW ceiling ≥ ~word 3700 (≈255 mA)

1. **CW-trim calibration** (continuous mode, good coupling, word 300 @
   48.8 kHz): closed loop against PDC — step demod-window CW word until
   armed-PDC == disarmed-PDC (±2 %). Record trim word + camera means.
2. **demod.c per-frame CW swap**: extend the interleave scheduler to write
   CW_trim on arm and CW_normal on disarm (two extra 2-byte I2C writes in
   the inter-pulse window; budget fine). New JSON field `DemodCwWord` in
   SET_DEMOD/GET_DEMOD + SDK setter. Rebuild, DFU, verify.
3. **Acceptance run** (this coupling or phantom): fw interleave N=10,
   60 s × 3 repeats → `analyze` per camera. Targets: S ≤ 0.35 on cams 1–3
   (shot-floor-limited), **M = 1.00 ± 0.05 on every camera**, K_nbr ==
   K_norm (no bleed), interval(frac) = 10 (≥0.95).
4. Repeat acceptance on the June phantom arrangement for the record.

## Branch B — ceiling < word 3400

1. Calibrate to the approved ceiling → partial compensation; measure
   residual ΔM and the far-camera SNR cost.
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
