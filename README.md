# STM32G431 Impedance Analyzer (WIP)

A firmware and toolset for generating and analyzing frequency sweeps on an STM32G431 "blackpill" board using its dual 4MSa/s ADC, DAC etc.

Contents
- `src/` — Firmware source code (`main.c` plus one `.c/.h` pair per module)
- `libopencm3/` — libopencm3 firmware library (submodule)
- `app/` — Octave and Python bindings for sending commands and receiving data (lock-in output and ADC frames), with examples (WIP).
- `docs/` — Reference material (datasheets, manuals, pin configurations, etc.)

Requirements
- ARM embedded toolchain (e.g. `arm-none-eabi-gcc`)
- `make`
- `libopencm3` checked out as a submodule
- Python 3 for the GUI (`python3 sweepgui/app.py`) and Octave for `getspec.m` if you use the analysis scripts

Build
- Initialize libopencm3 (if not already):

  git submodule update --init --recursive

- Build firmware:

  make

- Build outputs appear in `build/` (ELF, disassembly, size, etc.)

Flash
- Use your preferred SWD/flashing tool (for example `st-flash` or DFU) to program the generated ELF/bin. The makefile also contains two programming commands: `make prog` (for st-flash) and `make dfu`.

Notes
- This repository contains firmware sources and desktop tools intended for development and measurement. Adjust flashing and build steps to match your hardware and toolchain.

Todo
- DDS/Lock-in base done, missing some things (see `ddsli.c`).
- PLL and application code for controlling the DDS/Lock-in for measurements.
- Improving the USB CDC/ADC interface.
  - Add package structure with checksum and ack?
- Finish octave/python bindings and applications.

