# Demod Frame Characterization — Findings (Bench Session 1)

**Date:** 2026-06-12
**Setup:** Unified console (feature/demod-frames fw, Debug build), left sensor
module on static phantom, 4 cameras (mask 0x0F), 40 Hz / 500 µs pulses,
CW 140 mA. Harness: openmotion-sdk `feature/demod-config`
`scripts/demod_characterization.py`. Seed FPGA rev 1.1.0.
**Test plan:** [2026-06-11-demod-characterization-test-plan.md](./2026-06-11-demod-characterization-test-plan.md)

## Verdict

**The demod-frame feature does not work on current hardware — for two
independent root causes, both now isolated.** The MCU/firmware/SDK chain is
fully verified end-to-end on the bench; the blockers are in the Seed FPGA
pin assignment and in the analog modulation chain.

## What was verified working (Phase 0: ALL PASS)

- `OW_CTRL_SET_DEMOD`/`GET_DEMOD`, register writes/readbacks, configure
  strobe, arm/disarm writes — all land correctly over UART→I2C.
- The reg-0x0C FPGA write bug (predicted from `registers.v:183`) was
  reproduced on silicon byte-for-byte and the firmware's byte-ordered
  write mitigation verified (word `0x0A00BEEF` lands exactly).
- AD9837 DDS configure works: after the 0x22 strobe the DDS runs and
  measurably modulates the seed current (see below).
- TA FPGA trigger counter increments during scans (trigger reaches the
  unified board); safety/status registers stayed clean throughout,
  including at modulation amplitude word 30000.

## Root cause 1 — the Seed FPGA never sees the trigger

`dds_control_interface.v` gates modulation per laser pulse: DDS start on
trigger rising edge (when Static Control D[0] arms it), DDS stop (reset)
on the falling edge. **No trigger edge ever arrives:**

- Disarm test: with the DDS left free-running by the configure strobe and
  the arm bit clear, the modulation signature persisted through an entire
  scan — a single falling edge would have stopped it. The arm bit changes
  nothing (armed vs disarmed runs identical).
- Schematic (700-00010 rev 0.5): net `TRIGGER` routes from the MCU to the
  **TA FPGA** (sheet 23). The Seed FPGA sheet (22) has no trigger net; the
  FPGA's `trigger` input (top.v pin 17) is wired to **`SEED_LASER_DISABLE_N`**
  — a static enable, matching top.v's commented-out original pin role
  (`//output seed_laser_disable, // Pin 17`). The FPGA repurposed the pin;
  the board did not.

Consequences: per-pulse gating is impossible on this board + FPGA image;
the DDS free-runs continuously from the moment of configure; Static
Control D[0] is a no-op.

## Root cause 2 — modulation depth saturates ~50× too shallow

With the DDS free-running through every pulse (unintended but a perfect
continuous-modulation reference), contrast never collapsed at any
amplitude or frequency tested. The slow-modulation discriminator
(6.1 kHz ≈ 3 cycles/pulse → pulse energy becomes modulation-phase
dependent) measures true optical depth via frame-to-frame mean spread:

| run | gain word | freq | cam3 K | cam3 mean | frame-mean spread |
|---|---|---|---|---|---|
| baseline (no mod) | 0 | — | 0.528 | 207.3 | 1.73 |
| continuous | 312 | 1.5625 MHz | 0.529 | 207.7 | — |
| continuous | 1000 | 1.5625 MHz | 0.561 | 207.9 | 1.99 |
| continuous | 8000 | 1.5625 MHz | 0.561 | 206.2 | — |
| continuous | 30000 | 1.5625 MHz | 0.553 | 206.2 | — |
| slow | 1000 | 6.1 kHz | — | 209.7 (+1.2%) | 2.82 (+63%) |
| slow | 30000 | 6.1 kHz | — | 205.3 | 2.31 |

- The modulation **does** reach the laser (slow-mod mean shift +1.2%,
  frame-energy spread +63% — real, optical, reproducible).
- But the effect **saturates by gain word ≈ 1000** and does not grow to
  word 30000 (gain register read back 30000, no over-current fault,
  status clean). Effective depth ≈ 1–3 mA on 140 mA CW — versus the
  ~tens of mA needed for meaningful wavelength sweep.
- Net contrast effect of saturated modulation: **K rises ~6%** (uniform
  shift across all frames — added intensity noise), the opposite of the
  intended collapse. Mean intensity is unchanged at 1.5625 MHz (≤0.3%).

Where the saturation lives is not resolvable over I2C: candidates are the
AD5689R VOUTA → AD633 Y-input path, the AD633 W output clamp, or the
summing into the OPA27 V-to-I stage (schematic sheet 20). Needs a scope.

## Paths forward

1. **Henry (FPGA):** (a) the trigger pin assignment vs board routing —
   which net should `dds_control_interface.trigger` use, or should gating
   move to a different mechanism? (b) one-character fix in registers.v
   (`[23:0]` → `[23:16]`) for the reg-0x0C frequency-byte bug.
2. **Scope session:** AD9837 VOUT (600 mVpp expected), AD5689R VOUTA
   (should track gain word ×76 µV/LSB), AD633 W, and `CW_MODULATE` while
   sweeping the gain word — find the clamp.
3. **Firmware Plan B (ready to implement):** per-frame gating without the
   trigger — the demod scheduler writes the **gain word** (reg 0x02,
   N ↔ 0) between pulses instead of the dead Static Control bit. The
   AD5689R settles in µs; the existing demod.c state machine needs only
   the register/payload swapped. Blocked until root cause 2 is fixed —
   gating a too-shallow modulation buys nothing.
4. **Brad (system):** confirm the demod-frame contrast-collapse physics
   assumes a wavelength sweep of meaningful depth; with the current
   analog chain the achievable sweep appears ~2 orders of magnitude short.

## Addendum — bench rounds 2–5 (2026-06-12, same day)

Root cause 1 is **fixed and verified**; root cause 2 is **explained** (a third
FPGA logic flaw) but its fix is gated on physical bench presence.

- **Pin-99 + active-low trigger inversion verified working** (seed-fpga PR #3,
  image rev 1.2.0/1.3.0): debug counters added at I2C regs 0x17–0x19 read,
  with the trigger at 40 Hz: disarmed 87 edges / 0 starts / 87 stops per 2 s;
  armed 88/88/88. Per-pulse gating + MCU arm scheduling fully functional.
  (Round-3 confusion was a stale FPGA build; images now carry unique minor
  revisions so `sanity` fingerprints the chip.)
- **The "amplitude saturation" was an artifact of a third FPGA bug**:
  `dds_gain_control.v` silently drops gain writes unless
  `gain_word < dds_current_limit_reg` (raw words, mismatched scales; bench
  limit = 864). The laser-params pass writes gain 0 first, so every run that
  requested word ≥ 864 actually ran with the modulation DAC at **zero**. All
  earlier "saturation" data points above 864 were DAC=0 runs.
- True accepted-range transfer (gated, slow-mod mean-shift probe, words
  0→850): weak monotonic amplitude scaling (+0.6 % optical mean at 850)
  plus a small DAC-independent feedthrough floor. The curve above word 863
  has never been measured.
- Extension past the gate requires raising DDS_CL (or fixing the FPGA
  gate) — **deferred to Monday with a human at the bench**; loosening a
  limit register on a live laser unattended was deliberately not done.

### Monday runbook

1. Bench present: temporarily write `SEED_DDS_CL` = 20000 (reg 0x06; gate
   comparison only — ADC monitor + safety FPGAs unaffected), then re-run the
   slow-mod amplitude sweep upward (2000 / 8000 / 16000), watching Seed
   STATUS bit 0 and camera mean between steps. Restore CL = 864 after.
2. If the mean-shift keeps scaling: compute the word for ~30–60 mA effective
   depth, re-run the interleave at 1.5625 MHz, and look for the contrast
   collapse — the original feature acceptance test.
3. Saleae (optional now): only needed if the extended sweep still
   under-delivers — probe AD5689R VOUTA staircase, AD633 W, CW_MODULATE.
4. FPGA follow-ups for Henry: clamp-not-drop on the gain gate, scale-match
   the comparison, consider mainlining the 0x17–0x19 debug counters.

## Addendum 2 — round 6 (weekend remote session, user-authorized +20 mA)

With explicit authorization for up to 20 mA extra seed current, the gain
gate (`SEED_DDS_CL`) was raised by exactly that increment in its own scale
(864 → 1111), the previously-unreachable amplitude range was measured, and
the bench restored (CL = 864, gain = 0) afterward. Peak modulation used:
≈ ±12 mA (PDC −8.8 %) — within budget. Key results, all PDC-verified
(source-side photodiode, speckle-immune):

1. **Modulation effectiveness peaks at gain word ≈ 860 and collapses
   sharply between 860 and 880** (PDC −8.8 % → −2.7 %), with no status
   flags. The old production CL value (864) evidently sits just below an
   analog clip/foldback threshold — it was calibrated, not arbitrary. Max
   usable amplitude on this hardware = word ~860.
2. **The modulation chain is bandwidth-limited.** Identical word 860:
   6.1 kHz → PDC −8.8 %; 1.5625 MHz → −0.8 %. The current driver rolls the
   amplitude off ~10× by 1.5625 MHz (V-to-I loop bandwidth likely tens of
   kHz). The DDS frequency range the washout concept assumed is not
   deliverable by the analog stage.
3. **No contrast washout even at full deliverable strength.** At 6.1 kHz,
   word 860 (~9 % power modulation, 3 triangle sweeps per pulse), speckle
   contrast is unchanged (K 0.5421 vs 0.5385 reference; the 1.5625 MHz
   acceptance run with verified per-pulse gating was equally null:
   armed 0.5361 vs disarmed 0.5390, interleave per-phase flat to 0.1 %).
   The wavelength excursion at deliverable depth is insufficient for this
   phantom — or the modulation is predominantly AM rather than chirp.

### Conclusion

Every digital layer of the demod-frame feature is verified working:
console firmware scheduling, SDK, FPGA per-pulse gating (counter-proven),
DDS configuration. **The physical effect the feature exists for — contrast
collapse with unchanged intensity — does not occur on this hardware**, for
two measured analog reasons: (a) modulation bandwidth ≪ required MHz
rates, and (b) deliverable modulation depth capped at ~±12 mA by an
analog limiter, producing ~9 % AM and no measurable speckle decorrelation.
Mean intensity is also *not* preserved at usable depth (−8.8 % PDC), so
both halves of the original intent fail as built.

Decision needed (Brad/Henry): re-spec the modulation path (driver
bandwidth + depth + intended chirp), or rethink the demod-frame concept.
Monday scope work can confirm the foldback mechanism and driver BW
directly; the Saleae plan stands.

## Artifacts

Per-run CSVs, metadata, and reports:
`openmotion-sdk worktree scan_data/demod-characterization/{baseline,
continuous, cont-amp1000, cont-probe, disarm-amp1000, slow6khz-amp1000,
amp8000, amp30000, slow6khz-amp30000}/`.
