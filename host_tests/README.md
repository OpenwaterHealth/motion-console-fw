# Host tests

Standalone GCC-built unit tests for pure-C modules in `Core/Src/` that have no STM32 HAL dependencies. Run on a host machine (Linux, macOS, or Windows with MSYS2/MinGW/LLVM installed):

    cd host_tests
    make run

Expected output for each test:

    <module> host tests OK

These tests are not part of the firmware CMake build. They are a fast sanity check for the pure-data-structure modules; on-hardware verification (full firmware build + bench scan) is still required and is covered by the integration tasks in the corresponding implementation plan.

## Requirements

- gcc, clang, or any host C compiler reachable as `gcc`, `cc`, or via `make CC=...`
- GNU Make (or compatible)

## Why not part of CMake?

The firmware CMake project uses the ARM cross-compiler (`arm-none-eabi-gcc`), which cannot produce host-runnable binaries. The host tests need a separate, much simpler build path.
