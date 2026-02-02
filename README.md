# STM32G431 Impedance Analyzer

A small firmware and toolset for generating and analyzing frequency sweeps on an STM32G431 "blackpill" board using its dual 4MSa/s ADC, DAC etc.

Contents
- `main.c` — firmware entry point
- `modules/` — firmware modules (AD9833 driver, ADC, DAC, SPI, USB serial, utilities)
- `libopencm3/` — libopencm3 firmware library (submodule)
- `sweepgui/` — Python GUI and helper scripts for running and saving sweeps

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
- Use your preferred SWD/flashing tool (for example `st-flash` or DFU) to program the generated ELF/bin.

Notes
- This repository contains firmware sources and desktop tools intended for development and measurement. Adjust flashing and build steps to match your hardware and toolchain.

Todo
- Quite a lot of things: DAC waveform generation (currently uses a AD9833 DDS frequency generator), run the lockin detection/quadrature demodulation in STM32 instead of in the app/GUI, and apply an onboard PLL loop for resonance measurements.

GUI Screenshot

![GUI Screenshot](docs/screenshot.png)

Data flowchart for DSP processing algorithm for embedded/stm32. There are several peripherals and CPU processing involved, with several buffers in between. Every circular buffer is divided in two halfs for simultaneous and continuous data processing/source/sink.

Data sources:
1. ADC, triggered by timer at given sample rate, automatic DMA transfer to circular buffer with two interrupts per buffer cycle (half and complete), dual ADC mode
2. CPU phase calculator, generates a circular buffer of phases (modular/overflows, -pi to pi over 16 bits) for DDS (inputs, only at beginning: initial phase, increment/frequency, then run as needed)

Data sinks:
3. DAC, triggered by timer at same data rate as ADC, circular buffer (2 ISR call per cycle), for using as DDS
4. LPF output FIFO, for demodulated/filtered data output, slower rate

Data processing routines:
5. CORDIC via DMA, turns a phase buffer into sin/cos buffer, runs with CPU trigger on half buffer.
6. CPU linear combination, calculates A*sin + B*cos from sin/cos buffer (A,B constants), output to DAC buffer, runs with CPU trigger on half buffer.
7. DMA memcpy, make a backup of sin/cos buffer to a secondary sin/cos buffer.
8. CPU demodulator/mixer, takes both sin/cos buffer and ADC buffer to give their product element by element (mixing).
9. CPU LPF, for filtering the demodulator, output once per x samples (~n integer periods of dds)

Buffers:
10. Phase buffer, runs 2 half-buffers ahead, loaded by CPU, read by CORDIC.
11. sin/cos buffer 1, written by CORDIC DMA, read by CPU linear combiner and DMA memcpy.
12. sin/cos buffer 2, written by DMA memcpy, read by demodulator.
13. DAC buffer, written by CPU linear combiner, read by DAC DMA.
14. ADC buffer, written by ADC DMA, read by demodulator.
15. Demodulator buffer, written by demodulator, read by LPF.
16. LPF output FIFO, written by LPF, read by USB output at slower rate

Temporal data/process diagram/slices (1 delta t = 1 half buffer):
```
+---------+---------+---------+---------+
|  t - 1  | t (now) |  t + 1  |  t + 2  |
+---------+---------+---------+---------+
|         |         |  Phase  |  Phase  |
|         | sincos1 | sincos1 |         |
|         |   DAC   |   DAC   |         |
| sincos2 | sincos2 |         |         |
|   ADC   |   ADC   |         |         |
| demoout |         |         |         |
+---------+---------+---------+---------+
| CPU mix |   ADC   | CORDIC  | CPUphase|
| CPU LPF |   DAC   | CPU lc  |         |
|         |  DMAmc  |         |         |
+---------+---------+---------+---------+
```

Every half buffer cycle:
- CPU calculate phase for next 2 half buffers ahead.
- CODIC runs on next phase buffer half (CPU triggered), writing next sincos1 half buffer.
- CPU runs linear combination on next sincos1 half buffer, after CODIC finishes (only thing that need to wait something), writing next DAC half.
- CPU mix (8) and LPF (9) runs on the last complete half from ADC/DMA/sincos2, outputting results to FIFO.
- DMA memcpy (7) copy current sincos1 to current sincos2, CPU triggered.

While, autonomously:
- ADC and DAC runs on current buffer half with DMA.