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

## Artifacts

Per-run CSVs, metadata, and reports:
`openmotion-sdk worktree scan_data/demod-characterization/{baseline,
continuous, cont-amp1000, cont-probe, disarm-amp1000, slow6khz-amp1000,
amp8000, amp30000, slow6khz-amp30000}/`.
