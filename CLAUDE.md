# openmotion-console-fw — Claude guide

Firmware for the console-controller MCU (STM32H743VIH6, Cortex-M7 480 MHz, 2 MB flash, 1 MB SRAM). Talks to the host SDK over UART (921 600 baud, primary), exposes a USB CDC for debug, and programs the on-board Lattice MachXO2 FPGAs over I2C.

Cross-repo context + shared protocol: [../CLAUDE.md](../CLAUDE.md). Authoritative protocol spec: [`CommandHandling.md`](./CommandHandling.md).

## Build / flash

```powershell
# Configure + build (CMakePresets.json drives toolchain selection)
cmake --preset Debug
cmake --build build/Debug          # → build/Debug/motion-console-fw.{elf,hex,bin,map}

cmake --preset Release
cmake --build build/Release

# Flash to a connected console
python scripts/deploy.py --config Debug         # builds, enters DFU, dfu-utils, soft-resets
python scripts/deploy.py --config Release --post-reset
```

- **Flashing is DFU over USB** (STM32 bootloader VID/PID `0x0483:0xDF11`), not OpenOCD or ST-Link. `scripts/deploy.py` is the entry point; helpers in `scripts/_deploy_helpers.py`.
- Toolchain file: `cmake/gcc-arm-none-eabi.cmake`. CI uses `ghcr.io/openwaterhealth/stm32-build-env:latest` (ARM GCC 15.2.rel1). Local Dockerfile present for the same.
- `build/Debug/generated/version.h` is regenerated at configure time from `git describe`.
- Hardware config is in `motion-console-fw.ioc` (STM32CubeMX).

## Layout (current line counts; parent CLAUDE.md was stale)

| File | Lines | Purpose |
|---|---:|---|
| `Core/Src/main.c` | 1847 | Init, main loop, ISR handlers. HAL clock setup, IRQ priorities, UART4 / USB CDC / I2C / SPI bring-up. |
| `Core/Src/uart_comms.c` | 404 | UART4 packet framing with CRC-16. **Primary** host transport. |
| `Core/Src/if_commands.c` | 761 | Command dispatcher — `process_if_command()`. |
| `Core/Src/if_fpga_prog.c` | 393 | FPGA programming interface (UART command handler side). |
| `Core/Src/XO2_api.c` | 717 | Lattice MachXO2 I2C ECA low-level. |
| `Core/Src/XO2_cmds.c` | 1499 | XO2 programming state machine. |
| `Core/Src/trigger.c` | 479 | FSYNC / LSYNC trigger generation. PDC buffer drain. **Safety interlocks** at lines 21–22 (`_usb_trigger_interlock`, `_safety_trigger_interlock`). |
| `Core/Src/ad5761r.c` | 805 | SPI 16-bit DAC for TEC control. |
| `Core/Src/motion_config.c` | 296 | Persistent JSON config in flash (sector 7 bank 2). |
| `Core/Src/pdc_buffer.c`, `pdc_poll.c` | — | Per-frame PDC sample ring buffer (newer). Host drains via `OW_CTRL_GET_PDC_BUFFER` (0x25). |
| `Core/Src/odometer.c` | — | Frame counters: system + laser odometer (newer; opcodes 0x26/0x27/0x28). |
| `Core/Src/flash_eeprom.c` | — | Flash sector erase / write helpers (used by `motion_config.c`). |
| `Core/Src/logging.c` | — | Debug logging to USB CDC. |
| `Core/Src/led_driver.c`, `Core/Src/tca9548.c`, `Core/Src/mcp42u83.c`, `Core/Src/fpga_register_map.c` | — | Drivers / helpers. |
| `Core/Inc/common.h` | — | **Shared with sensor-fw and SDK.** Packet types, command opcodes, error codes, max payload (`COMMAND_MAX_SIZE = 2048`). |
| `CommandHandling.md` | — | Protocol spec — packet format, all opcodes, error codes. Read this before changing the wire. |

Total `.c` files in `Core/Src/`: **34**. The list above is just the high-traffic ones.

## Hardware abstractions

| Device | Header | Source |
|---|---|---|
| MAX6663 fan controller | `fan_driver.h` | `fan_driver.c` |
| MAX31875 temp (×3) | `max31875.h` | `max31875.c` |
| ADS7924 ADC | `ads7924.h` | `ads7924.c` |
| ADS7828 ADC | `ads7828.h` | `ads7828.c` |
| TCA9548 I2C mux | `tca9548.h` | `tca9548.c` |
| PCA9535 GPIO expander | `pca9535.h` | `pca9535.c` |
| AD5761R 16-bit DAC (TEC) | `ad5761r.h` | `ad5761r.c` (805 lines) |
| Lattice XO2 FPGA (I2C ECA) | `if_fpga_prog.h`, `XO2_api.h` | `if_fpga_prog.c`, `XO2_api.c`, `XO2_cmds.c` |

## Protocol quick reference

Packet types (defined in `Core/Inc/common.h`):
`OW_ACK` 0xE0, `OW_NAK` 0xE1, `OW_CMD` 0xE2, `OW_RESP` 0xE3, `OW_DATA` 0xE4, `OW_JSON` 0xE5, `OW_I2C_PASSTHRU` 0xE9, **`OW_CONTROLLER` 0xEA** (most motion/console commands), `OW_FPGA_PROG` 0xEB, `OW_BAD_PARSE` 0xEC, `OW_BAD_CRC` 0xED, `OW_UNKNOWN` 0xEE, `OW_ERROR` 0xEF.

Error codes: `OW_CODE_SUCCESS` 0x00, `OW_CODE_IDENT_ERROR` 0xFD, `OW_CODE_DATA_ERROR` 0xFE, `OW_CODE_ERROR` 0xFF.

Notable `OW_CONTROLLER` commands (selected): `PING`, `VERSION`, `ECHO`, `HWID`, `DFU`, `RESET`, `I2C_SCAN`, `SET/GET_IND`, `SET/GET_TRIG`, `START/STOP_TRIG`, `SET/GET_FAN`, `I2C_RD/WR`, `GET_FSYNC/LSYNC`, `TEC_DAC`, `READ_ADC`, `READ_GPIO`, `GET_TEMPS`, `TECADC`, `TEC_STATUS`, `BOARDID`, `PDUMON`, `GET_PDC_BUFFER` (0x25), `GET_SYSTEM_ODO` (0x26), `GET_LASER_ODO` (0x27), `RESET_ODO` (0x28). Full list in `common.h` and `CommandHandling.md`.

There is **no explicit protocol-version opcode** — versioning rides git tags (semver: e.g. `1.5.8`, `1.5.8-rc.2`).

## Gotchas

- **TEC trigger interlocks** (`trigger.c:21-22`): `_usb_trigger_interlock` and `_safety_trigger_interlock` are both volatile flags. `trigger_start()` (line 262 area) checks state before firing. **Both must clear** or trigger silently won't arm.
- **Persistent config flash sector** (`motion_config.c`): JSON lives in `ADDR_FLASH_SECTOR_7_BANK2` (0x0801F000, 128 KB sector). Full sector erase on save. `motion_cfg_t` layout is locked by `_Static_assert(sizeof(motion_cfg_t) == MOTION_CFG_PAGE_SIZE, ...)` — adding fields without bumping the layout breaks compatibility.
- **I2C mux gates everything**: TCA9548 routes all peripheral I2C traffic. If mux init in `main.c` fails or is skipped, temperatures / ADCs / drivers all appear dead. Check the init sequence first when "the bus is broken."
- **PDC buffer must be drained** (`pdc_buffer.c`, `pdc_poll.c`): SRAM ring buffer fills; host must poll `OW_CTRL_GET_PDC_BUFFER` (0x25) regularly or frames drop silently.
- **UART is the primary host link** (UART4 @ 921 600). USB CDC is debug/logging only — `CDC_handle_TxCpltCallback` is a callback path, not a command path.
- **Trigger frequency:** default 40.0 Hz (`trigger.c:19`); JSON `TriggerFrequencyHz` is parsed at line 85. Parent CLAUDE.md claims 1–100 Hz — confirm against `jsonToTriggerConfigData` before relying on the bounds.
- **XO2_cmds.c TODO** (~line 140): "This delay time may be excessive." Not a blocker; FPGA programming may be slower than necessary.

## CI / releases

- `.github/workflows/build-firmware.yml` — push to `main`/`next`, tag push, or manual dispatch. Builds in Docker, uploads `.hex` / `.bin` to a GitHub Release, autogenerates release notes from commit log since prior tag.
- `.github/workflows/safety-security-scan.yml` — exists (static analysis / linting).
- Tags use **semantic versioning** (e.g. `1.5.4-rc1`, `1.5.5`, `1.5.8-rc.2`).
- Branching: feature branches off `next`, PR to `next`, `next` → `main` for tagging.

## "Start here" by task

| Task | First files |
|---|---|
| Add a new command | `Core/Inc/common.h` (add enum) → `CommandHandling.md` (spec) → `Core/Src/if_commands.c` (add case in `process_if_command()`). Also add SDK side in `openmotion-sdk/omotion/MotionConsole.py`. |
| Add a new hardware driver | New `Core/Inc/<dev>.h` + `Core/Src/<dev>.c` → register in `CMakeLists.txt` → init in `Core/Src/main.c`. |
| Debug UART issues | `Core/Src/uart_comms.c` (framing) → `Core/Inc/uart_comms.h` (structs) → check UART4 ISR in `Core/Src/stm32h7xx_it.c`. |
| Trigger / timing | `Core/Src/trigger.c`. Verify both interlock flags clear. |
| Persistent config | `Core/Src/motion_config.c` + `Core/Inc/motion_config.h`. Mind the sector-7-bank-2 boundary and the static_assert. |
| FPGA programming | `Core/Src/if_fpga_prog.c` → `Core/Src/XO2_api.c` → `Core/Src/XO2_cmds.c`. |
| Flash / DFU issues | `scripts/deploy.py` + `scripts/_deploy_helpers.py`; check the `.ioc` for USB bootloader config. |
