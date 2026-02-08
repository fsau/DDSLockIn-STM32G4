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
 * Public output sample type
 * -------------------------------------------------------------------------- */

/* Demodulated / filtered output sample. */
typedef struct {
    float frequency;
    uint16_t dds_amplitude;
    float chA[3];
    float chB[3];
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
