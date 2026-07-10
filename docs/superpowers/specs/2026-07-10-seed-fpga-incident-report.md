# Incident report — seed-FPGA I/O failure during demod knee-ladder characterization

**Date of incident:** 2026-07-10, 10:04:30–10:05:15 (bench local, −0700)
**System:** Open-Motion unified console, DVT-1A PCBA (700-00010 schematic rev 0.4), right sensor module attached
**Activity at time of failure:** demod-frame depth-ladder characterization (v19 series), authorized bench session
**Written:** 2026-07-10, same day, from run metadata, harness logs, capture files, and issue #171 comment history
**Companion documents:** `2026-07-10-seed-current-report-for-laser-team.md` (same directory); bloodflow-app issue #171 comments (IDs referenced throughout)

---

## 1. Summary

During an eight-run modulation-depth ladder, between the end of run w400
(10:05:01) and the first verification samples of run w500 (~10:05:05–15),
the seed FPGA's bottom-edge I/O bank stopped driving its output pins. The
pins on that bank carry the SPI buses to the two seed-drive converter
chips (AD5689R dual DAC, AD9837 DDS). From that moment, no command could
physically reach either chip, although every register write continued to
be accepted and read back correctly by the FPGA's (healthy) digital
fabric.

The seed laser itself was never mis-commanded, never over-driven, and was
not initially affected: the DAC held its last-loaded CW setpoint
(~130–140 mA) and the seed continued lasing normally, unmodulated, through
the remainder of the run. The seed went dark ~50 minutes later at a
routine power cycle (≈10:55), when the DACs reset to zero-scale and — the
command path being dead — could never be reloaded.

Root cause is mechanical, not electrical or procedural: the Saleae
logic-analyzer ground clip detached from SYS_GND ("popped off," found by
Ethan ~11:2x), an event consistent in time and location with lifting or
cracking pins/joints on the FPGA package edge that carries the affected
bank. The failure was conclusively localized by a differential test at
~12:0x: the FPGA's internal SPI logic emits perfectly (384 clean edges
observed on top-bank debug mirror pins) while the chip-side pins stay
flat.

Two diagnostic wrong turns along the way are documented in full in §6
(a retracted +5 V-rail theory; garbage analog readings caused by the same
detached ground), as is an emission-accounting correction (§7): post-fault
diagnostics fired the TA unseeded (ASE emission) for roughly 1–2 minutes
cumulative while reports described the laser as "inoperative."

---

## 2. Context: system state and why this experiment was running

### 2.1 Hardware/firmware state at session start

| Item | State |
|---|---|
| Console firmware | `feature/demod-frames` build (D[1] fix), version string 1.5.4-rc.1-45-gfe97ed4, DFU-flashed at ~09:55 that morning |
| Seed FPGA image | 1.5.0 (carries the seed_mod_ss pin fix and single-domain MOD-SCK fix from 2026-07-09) |
| Laser params (production) | CW word 2062 (≈130–145 mA per the affine transfer, "≈140 mA operating point"), DDS gain 0, DDS_CL 864, CW_CL 2469 |
| Trigger regime | 40 Hz, 500 µs pulses (2 % duty), skip 600/1800, TA trigger and SyncOut enabled |
| Probes | Saleae Logic Pro 16. Analog: ch8 = VOUTA (soldered), ch9 = DDS VOUT (R7 pad), ch10 = TP1-area node, ch11 = unidentified drive-chain node. Digital: ch12/13 on FPGA DAC-SPI mirror test points TP19/TP22, ch15 on U19 pin 7 (SCLK). Ground: croc clip on SYS_GND. |
| Authorization | Ethan, 2026-07-09: peak seed ≤ 180 mA for demod testing. (Superseded ~10:45 the same morning by the laser-engineering 140 mA directive — after the incident, coincidentally.) |

### 2.2 The experiment (v19 knee ladder)

Purpose: map delivered modulation depth vs the DDS gain word — locate the
suspected turn-on threshold (AD633 Y-input offset, predicted word
~100–390) and the smallest word that still fully washes out speckle. This
was step 2 of the committed next-session plan; the CW operating point was
deliberately left at the production value throughout.

Eight 15-second modulated runs (gain words 120, 160, 200, 240, 280, 320,
400, 500) at 48.8 kHz (frequency word 524280), preceded by a 35 s
unmodulated baseline. Every modulated run used the harness's
`--verify-mod` check: 3-sample averaged PDC with modulation armed vs
momentarily disarmed; ratio outside 1.10:1 (either direction) = verified
modulating; on failure, re-strobe the DDS config and re-write the gain
word, up to 4 attempts.

### 2.3 Exact per-run command sequence

Every ladder run issued, in order (all register I/O via the console's
I2C passthrough to the seed FPGA at mux 1 / channel 5 / addr 0x41):

1. Connect; camera configure (right module, mask 0x0F).
2. `apply_laser_power` — replays production laser_params.json:
   TA_PULSE_WIDTH, TA_CURRENT_DRV=122, SEED_DDS_GAIN←0, SEED_CW_GAIN←2062,
   SEED_DDS_CL←864, SEED_CW_CL←2469, EE/OPT safety-FPGA parameters.
3. DDS_CL ← 550 (ladder override), readback-verified.
4. SET_DEMOD {interval 0, freq word 524280, phase 0} → firmware stages
   freq/phase and strobes Dynamic Control (0x22) → FPGA SPIs the AD9837
   (reset, freq LSB/MSB, phase, triangle-run).
5. DDS gain word (0x02) ← ladder word, readback-verified.
6. Scan start (15 s) → trigger config + start (40 Hz, 500 µs, TA enabled).
7. +2.5 s: Static Control (0x20) ← 0x0003 (D0 arm + D1 laser_active);
   gain word re-written (post-scan-start ordering rule).
8. Verify loop: 3 PDC samples armed → 0x20←0x0002 → 3 samples disarmed →
   0x20←0x0003; ratio test; on failure re-strobe 0x22 + re-write gain.
9. Scan end; teardown: gain←0, 0x20←0x0000, DDS_CL←864; trigger stopped.
   CW word untouched (2062) throughout.

---

## 3. The failure, minute by minute

Times from run `meta.json` (`collected_at` = run completion) and the
harness log of background task bccd4xszx.

| Time (−0700) | Event |
|---|---|
| ~09:55 | Demod firmware DFU-flashed; Shelly power cycle; console back at 09:5x |
| ~10:00 | Harness `sanity`: ALL PASS, FPGA fingerprints [1,5,0] |
| 10:01:41 | v19-base-f49k completes (35 s unmodulated baseline; darks donor) |
| 10:02–10:04 | w120 ✓ (verify ratio 0.307), w160 ✓ (0.718), w200 ✓ (2.231), w240 ✓ (1.004→1.897, 2 attempts), w280 ✓ (1.88), w320 ✓ (1.873) |
| **10:05:01** | **w400 completes — verify ratio 1.871 on the first attempt. Last confirmed modulation ever delivered by this board.** |
| ~10:05:03 | w500 setup: apply_laser_power, DDS_CL←550 (readback OK), SET_DEMOD, gain←500 (readback OK), scan start, arm at +2.5 s |
| ~10:05:05–15 | **w500 verify: 1.006, 1.017, 1.000, 1.000 — four attempts, each preceded by a config re-strobe and gain re-write, all flat.** The FPGA I/O bank died in this window (or in the seconds before it, during setup). |
| 10:05:32 | w500 completes "normally" at the register level: 2,428 frame rows written, teardown restores registers. The seed lased unmodulated at its held ~140 mA setpoint for the entire run — which is precisely why armed/disarmed PDC were identical. |
| ~10:45 | Ethan relays the laser-engineering directive: 140 mA max, all modulation testing halted. (Independent of the not-yet-recognized fault.) |
| **≈10:55** | Halt actions: console DFU-restored to production 1.8.0 + Shelly power cycle. **The power cycle resets both AD5689R channels to power-on zero-scale; with the command path dead, they can never be reloaded. This is the moment the seed actually went dark.** |

Key mechanical correlate: the Saleae SYS_GND croc clip was later found
detached. The exact detachment moment is not observable in the data, but
the FPGA fault brackets to 10:04:30–10:05:15, and a clip under tension
letting go at the board is the only mechanical event on the bench in that
window. Working hypothesis: same event — the clip's departure stressed
the FPGA package edge (TQFP100) whose bottom-row pins (18–36 region) carry
both converter SPI buses.

What was being commanded at the moment of failure, exhaustively: CW word
2062 (production, unchanged all morning); DDS gain 500 under gate 550;
frequency word 524280; Static Control toggles 0x0003/0x0002; Dynamic
Control strobes. Per the schematic-derived transfer (see the analog-chain
memo), word 500's delivered modulation depth is micro-to-milliamp scale.
Nothing electrical distinguishes w500 from the seven runs before it.

---

## 4. What died, what survived, and when the seed actually turned off

**Died at ~10:05:** the FPGA's bottom-edge I/O bank outputs — physically,
the pins carrying: SEED_DAC_MOSI (20), SEED_DAC_SS (21), SEED_DAC_SCK
(24), SEED_LDAC_n (27), SEED_MOD_MOSI (29), SEED_MOD_SCK (30),
SEED_MOD_SS (36) (and by geography anything else on that row).
Consequences: no new DAC loads, no DDS configuration, no per-pulse DDS
start/stop.

**Survived throughout:** the FPGA digital fabric (I2C register file,
gates, revision/status/counters — every readback correct all day); the
±8.8 V analog chain (probed nodes actively driven); both converter chips'
supplies (5 V confirmed at C57 by DMM); the seed diode and TA; the
top-bank debug mirror pins.

**The seed's actual timeline:** the AD5689R holds its outputs without SPI
traffic. CW channel B, last loaded with 2062 during w500's own
apply_laser_power (or the identical value from w400's), held the seed at
its normal operating point through w500 and afterward. The seed lased,
healthy and unmodulated, until the 10:55 power cycle zeroed the DAC.
Total time as an "orphan" (lasing on a held setpoint with a dead command
path): ~50 minutes, all at the production operating point. It was then
dark for the rest of the day. At no point was it over-driven; at no point
did any protection (OPA547 ILIM ≈451 mA, over-current comparator) trip.

---

## 5. Fault localization — the evidence chain

1. **w500 verify pattern** (10:05): armed/disarmed PDC ratio exactly 1.000
   ×4 with re-strobes — healthy unmodulated laser, commands not landing.
2. **Post-halt config attempts** (11:0x–11:3x): 0-for-17 DDS configuration
   attempts across two power cycles and both bring-up paths (raw register
   writes on 1.8.0; production apply_laser_power path). Register
   readbacks perfect throughout.
3. **DMM, by Ethan** (~11:4x): **5 V present at C57** (AD9837 supply) —
   killing the rail-failure hypothesis (§6.2).
4. **Trusted-ground analog readings** (11:34, after ground repair):
   AD9837 VOUT = 3.0 mV (healthy parked value: ~340 mV); AD5689R VOUTA =
   3.2 mV with word 300 loaded (expected 22.9 mV); ±8.8 V chain nodes
   actively driven. Both 5 V-supplied converter outputs at zero, chain
   alive.
5. **The mirror discriminator** (~12:0x, conclusive): the FPGA mirrors its
   DAC SPI onto top-bank test points TP19/TP22 (a deliberate debug feature
   of the 1.3.0+ images). Eight gain writes produced **384 edges on the
   SCK mirror and 56 on the MOSI mirror — bit-perfect — while U19's SCLK
   pin at the chip recorded zero edges through two config strobes.**
   FPGA logic transmitting; bottom-bank pins dead. Q.E.D.

---

## 6. Diagnostic wrong turns (kept for the record)

### 6.1 Garbage analog readings from the same detached ground
All Saleae analog measurements between the clip's departure (~10:05) and
its rediscovery/repair (~11:2x) were unreferenced: channels showed a
common ~99 kHz switcher tone, physically impossible DC values, and a
"dead DDS at 0 V" that was, at that stage, unmeasurable rather than
proven. The first "0/17 config failures" were judged partly on these
readings. Lesson: §9.3.

### 6.2 The retracted +5 V-rail theory
With both converter outputs dead and digital alive, a shared +5 V feed
failure was the most economical single fault, and was posted as the
closed diagnosis (#171 comment 4938405862). Ethan's C57 measurement
(5 V present) falsified it within the hour; retraction and the corrected
diagnosis are on the issue (comment 4938517276). The PDU rail-monitor
comparison (all 16 channels identical to the previous day) had already
hinted the monitored rails were fine.

### 6.3 Timestamp error in initial reports
Early incident comments quoted the fault at "~10:32"; run metadata places
w400/w500 at 10:05:01/10:05:32. The fault window is 10:04:30–10:05:15.
This report is authoritative on timing.

---

## 7. Emission accounting during and after the fault (corrected)

Correction credited to Ethan: **an unseeded TA still emits ASE** when
triggered. Earlier comments describing the post-fault system as "laser
inoperative" were seed-only statements. Actual emission history:

| Window | Seed | TA / emission |
|---|---|---|
| 10:05 – 10:55 | **Lasing normally** at held ~140 mA setpoint (unmodulated) | Normal seeded operation during w500's remainder; trigger off between runs |
| 10:55 – 11:09 | Dark (DAC zeroed by power cycle) | No trigger → no emission |
| 11:09 tp1-ab legs A+B | Dark | Trigger ON ~15 s, TA at power-on default drive (laser params deliberately skipped) — **unseeded TA firing, drive level = POR default** |
| 11:1x strobe retries + post-cycle retry | Dark | Trigger ON ~10 s, same POR-default condition |
| 11:2x "rails" variant | Dark | `apply_laser_power` ran (TA_CURRENT_DRV=122) then trigger ON ~15 s — **unseeded TA firing at configured drive** |
| 11:34 tp1-ab2 | Dark | Same configuration, trigger ON ~40 s — **unseeded TA firing at configured drive** |
| ~12:0x mirror test | Dark | No trigger — no emission |
| Since | Dark | Trigger off, bench parked |

Cumulative unseeded-TA operation: roughly 1–2 minutes of 40 Hz / 500 µs
(2 % duty) windows. Open question referred to laser engineering: this
TA's tolerance for brief unseeded operation. Standing rules adopted in
response: seed-dead ⇒ no trigger starts; unavoidable diagnostics run with
EnableTaTrigger=false and without laser params; console powered off during
board repair; after repair, the seed drive path must verify electrically
before the first trigger.

---

## 8. Current state (as of writing)

- Console: production-lineage **1.8.1-rc.0** (next-tip build, flashed at
  Ethan's request, built `-DBARE_METAL=ON` — note `next` now defaults to
  the bootloader-hosted layout at 0x08020400, which does not boot on
  bootloader-less bench units).
- Seed FPGA: image 1.5.0; digital fully functional; bottom-bank outputs
  dead pending repair.
- Registers: gain 0, static 0x0000, trigger off; CW register reads 2062
  but the DAC output is unloaded (zero) and unloadable until repair.
- Seed: dark. TA: idle. No emission possible without a trigger start.
- Probes: ground restored; analog/digital clips still placed.

## 9. Repair plan and corrective actions

1. **Repair:** console powered off → magnifier inspection of FPGA pins
   ~18–36 (lifted/bent/cracked joints), continuity pin-to-net for the
   seven SPI signals, reflow as needed; DMM the VCCIO decap row while
   there. Post-repair, the mirror-vs-chip SPI test (5 min, no emission)
   re-run as acceptance.
2. **Post-repair verification order:** electrical (VOUTA/VOUTB track
   words) → first seeded trigger → PDC confirms light → the staged
   low-power TP1 A/B (CW word 1700 ≈ 80–117 mA under both scale models).
3. **Probe practice:** ground crocs get strain relief / soldered grounds
   for multi-day campaigns; after any clip event, re-verify a known DC
   reference before trusting any channel (the 99 kHz common tone is the
   floating-reference signature).
4. **Verification practice:** modulation is verified by the DDS's own
   output or optical effect, never by register readback (this incident's
   register file said "everything is fine" all day — truthfully, and
   uselessly).
5. **Reporting practice:** emission statements must cover the TA
   explicitly, not just the seed (§7).

## 10. Data index

- Run data: `scan_data/demod-characterization/v19-*` (ladder + baseline),
  `tp1-ab-110932/` (post-fault A/B + strobe checks), `tp1-ab2-113413/`,
  `spi-bank-check-*/` (mirror discriminator), all in the
  `sdk-demod-config` worktree.
- Harness log of the fatal ladder: background task output bccd4xszx
  (session scratchpad).
- Issue #171 comments: 4938277648 (initial fault report; timing since
  corrected), 4938405862 (rail theory — retracted), 4938517276 (localized
  diagnosis), 4938528786 (unseeded-TA correction).
- Related documents: seed-current report (same directory);
  dvt1a-seed-drive-analog-chain memo (derived transfer functions).
