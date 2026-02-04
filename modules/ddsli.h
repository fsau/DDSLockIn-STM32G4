#pragma once

#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * DDS + Lock-In Demodulator Interface
 * ============================================================================
 *
 * Streaming DDS generator and lock-in demodulator driven by half-buffer
 * events.
 *
 * Processing model:
 *   - ADC/DAC run autonomously (DMA + timers)
 *   - DSP pipeline advances one step per half-buffer
 *   - Demodulated output is produced at a lower rate and stored in a FIFO
 *
 * External code pulls demodulated samples from the FIFO using the public
 * output interface defined below.
 * ============================================================================
 */

/* --------------------------------------------------------------------------
 * Data Flow and Processing Model
 * --------------------------------------------------------------------------
 *
 * Data sources:
 * 1. ADC
 *    - Triggered by timer at given sample rate
 *    - Automatic DMA transfer to circular buffer
 *    - Two interrupts per buffer cycle (half and complete)
 *    - Dual ADC mode
 *
 * 2. CPU phase calculator
 *    - Generates a circular buffer of phases for DDS
 *    - Modular/overflows, -pi to pi over 16 bits
 *    - Inputs only at beginning: initial phase, increment/frequency
 *    - Then runs as needed
 *
 * Data sinks:
 * 3. DAC
 *    - Triggered by timer at same data rate as ADC (initially)
 *    - Circular buffer
 *    - Two ISR calls per cycle
 *    - Used as DDS output
 *
 * 4. LPF output FIFO
 *    - Stores demodulated/filtered data
 *    - Slower output rate
 *
 * Data processing routines:
 * 5. CORDIC via DMA
 *    - Converts phase buffer into sin/cos buffer
 *    - CPU-triggered on half-buffer events
 *
 * 6. CPU linear combination
 *    - Computes A*sin + B*cos from sin/cos buffer
 *    - A, B are constants
 *    - Output written to DAC buffer
 *    - CPU-triggered on half-buffer events
 *
 * 7. DMA memcpy NOT USED ANYMORE
 *    - Copies sin/cos buffer to secondary sin/cos buffer
 *
 * 8. CPU demodulator / mixer
 *    - Multiplies sin/cos buffer with ADC buffer
 *    - Element-by-element mixing
 *
 * 9. CPU LPF
 *    - Filters demodulator output
 *    - Produces one output per x samples
 *    - Typically ~n integer periods of DDS
 *
 * Buffers:
 * 10. Phase buffer
 *     - Runs two half-buffers ahead
 *     - Written by CPU, read by CORDIC
 *
 * 11. Sin/cos buffer TRIPLE BUFFERED
 *     - Written by CORDIC DMA
 *     - Read by CPU linear combiner and DMA memcpy
 *
 * 12. Sin/cos buffer 2 NOT USED ANYMORE
 *     - Written by DMA memcpy
 *     - Read by demodulator
 *
 * 13. DAC buffer
 *     - Written by CPU linear combiner
 *     - Read by DAC DMA
 *
 * 14. ADC buffer
 *     - Written by ADC DMA
 *     - Read by demodulator
 *
 * 15. Demodulator buffer
 *     - Written by demodulator
 *     - Read by LPF
 *
 * 16. LPF output FIFO
 *     - Written by LPF
 *     - Read by USB output at slower rate
 *
 * Temporal data / process diagram
 * (1 delta t = 1 half-buffer):
 *
 *   +---------+---------+---------+---------+
 *   |  t - 1  | t (now) |  t + 1  |  t + 2  |
 *   +---------+---------+---------+---------+
 *   |         |         |  Phase  |  Phase  |
 *   |         | sincos1 | sincos1 |         |
 *   |         |   DAC   |   DAC   |         |
 *   | sincos2 | sincos2 |         |         |
 *   |   ADC   |   ADC   |         |         |
 *   | demoout |         |         |         |
 *   +---------+---------+---------+---------+
 *   | CPU mix |   ADC   | CORDIC  | CPUphase|
 *   | CPU LPF |   DAC   | CPU lc  |         |
 *   |         |  DMAmc  |         |         |
 *   +---------+---------+---------+---------+
 *
 * Every half-buffer cycle:
 * 1. CORDIC runs on next phase buffer half (CPU-triggered),
 *    writing next sincos1 half-buffer.
 *
 * 2. DMA memcpy copies current sincos1 to current sincos2
 *    (CPU-triggered).
 *
 * 3. CPU demodulator runs on the last complete half-buffer from
 *    ADC and sincos2:
 *      - Mixing
 *      - LPF
 *    Outputs results to FIFO.
 *
 * 4. CPU calculates phase for the next two half-buffers ahead.
 *
 * 5. CPU runs linear combination on next sincos1 half-buffer,
 *    after CORDIC finishes (only dependency),
 *    writing next DAC half-buffer.
 *
 * While autonomously:
 * - ADC and DAC run on current buffer halves using DMA.
 * - Half/full buffer interrupts control data flow.
 * --------------------------------------------------------------------------
 */

/* --------------------------------------------------------------------------
 * Public output sample type
 * -------------------------------------------------------------------------- */

/* Demodulated / filtered output sample. */
typedef struct {
    float frequency;
    uint16_t dds_amplitude;
    int32_t chA[2];
    int32_t chB[2];
} ddsli_output_t;

/* --------------------------------------------------------------------------
 * Setup / Initialization
 * -------------------------------------------------------------------------- */

// Samples per half-buffer (total buffer size = 2 * HB_LEN * sizeof(type))
#ifndef LEGACY_MODE
#define HB_LEN 256U
#else
#define HB_LEN 16
#endif

/* Initialize internal state, buffers, and peripherals.
 * Must be called once before any processing.
 */
void ddsli_setup(void);

/* --------------------------------------------------------------------------
 * Execution Control
 * -------------------------------------------------------------------------- */

/* Returns true if one half-buffer processing step is pending. */
bool ddsli_step_ready(void);

/* Execute one half-buffer processing step.
 * Must be called only when ddsli_step_ready() returns true.
 */
bool ddsli_step(void);

/* --------------------------------------------------------------------------
 * Output interface
 * -------------------------------------------------------------------------- */

/* Check whether at least one output sample is available in the FIFO. */
bool ddsli_output_ready(void);

/* Pop one demodulated output sample from the FIFO.
 *
 * Returns true if a sample was written to *out.
 * Returns false if the FIFO was empty.
 */
bool ddsli_output_pop(ddsli_output_t *out);
