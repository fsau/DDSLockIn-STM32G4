#pragma once

#include <stdbool.h>
#include <stdint.h>

/* ============================================================================
 * DDS + Lock-In Demodulator Interface
 * ============================================================================
 *
 * Streaming DDS signal generator with synchronous lock-in demodulation.
 *
 * Processing model:
 *   - ADC and DAC run autonomously using DMA and timers
 *   - Buffers are circular and split into half-buffers
 *   - Half-buffer and peripheral completion events set internal flags
 *   - The DSP pipeline advances incrementally based on those flags
 *
 * Demodulated results are produced at a lower rate and pushed into an
 * internal FIFO. External code consumes results through the output API.
 *
 * The processing step function may be called from a low-priority ISR
 * (PendSV) or polled from the main loop.
 * ============================================================================
 */

/* --------------------------------------------------------------------------
 * Public types & definitions
 * -------------------------------------------------------------------------- */

/* Phase representation:
 *   Full scale (2^31) corresponds to pi radians
 *   Range: [-pi, +pi)
 */
typedef int64_t ddsli_phase_t;              /* Q32.32 */
typedef int64_t ddsli_phase_inc_t;          /* Q32.32 */
typedef int64_t ddsli_phase_inc_delta_t;    /* Q32.32 */

typedef struct {
    ddsli_phase_t           phase;            /* Phase accumulator */
    ddsli_phase_inc_t       phase_inc;        /* Phase increment per sample */
    ddsli_phase_inc_delta_t phase_inc_delta;  /* Frequency slope */
} ddsli_phase_ctrl_t;

/* DDS output linear combination coefficients */
typedef struct {
    int16_t A1;   /* sin coefficient (Q1.15) → DAC channel 1 */
    int16_t A2;   /* sin/cos coefficient (Q1.15) → DAC channel 2 */
} ddsli_out_ctrl_t;

/* Demodulated output sample */
typedef struct {
    ddsli_phase_ctrl_t frequency;   /* Phase state associated with this output */
    ddsli_out_ctrl_t   ddsli_amplitude;
    float              chA[3];      /* I, Q, DC */
    float              chB[3];      /* I, Q, DC */
} ddsli_output_t;

/* --------------------------------------------------------------------------
 * Buffer sizing
 * -------------------------------------------------------------------------- */

/* Samples per half-buffer */
#define HB_LEN 256U

/* Output FIFO depth (demodulated samples) */
#define LPF_FIFO_LEN 64U

/* ADC / reference capture depth (half-buffers) */
#define CAPT_BUFF_HALVES 4

/* --------------------------------------------------------------------------
 * Setup / Initialization
 * -------------------------------------------------------------------------- */

/* Initialize internal state, buffers, and peripherals.
 * Must be called once before starting processing.
 *
 * ADC, DAC, DMA, CORDIC, and internal buffers are initialized,
 * but external trigger sources may be enabled afterward.
 */
void ddsli_setup(void);

/* Configure DDS frequency and optional linear sweep */
void ddsli_set_frequency(float f, float sweep, float sample_f);

/* Configure independent frequencies and sweeps for dual DDS outputs */
void ddsli_set_frequency_dual(float f, float fb,
                              float sweep, float sweepb,
                              float sample_f);

/* --------------------------------------------------------------------------
 * Execution control
 * -------------------------------------------------------------------------- */

/* Returns true if there is pending processing work.
 * Indicates that ddsli_step() should be called.
 */
bool ddsli_step_ready(void);

/* Advance the DDS / demodulation pipeline.
 *
 * One call may advance only part of the pipeline depending on
 * which peripheral events have occurred.
 *
 * Intended to be called repeatedly while ddsli_step_ready() is true.
 *
 * Return values:
 *   >0 : processing step executed
 *    0 : no work performed
 *   <0 : step skipped (unexpected buffer phase)
 */
int8_t ddsli_step(void);

/* --------------------------------------------------------------------------
 * Lock-in output interface
 * -------------------------------------------------------------------------- */

/* Returns true if at least one demodulated output sample is available */
bool ddsli_output_ready(void);

/* Pop one demodulated output sample from the FIFO.
 *
 * Returns true if a sample was written to *out.
 * Returns false if the FIFO was empty.
 */
bool ddsli_output_pop(ddsli_output_t *out);

/* --------------------------------------------------------------------------
 * ADC / reference capture interface
 * -------------------------------------------------------------------------- */

/* Trigger capture of the next n half-buffers (n <= CAPT_BUFF_HALVES).
 *
 * Captured ADC and reference (sine/cosine) data are stored internally
 * and may be read once capture completes.
 */
void ddsli_capture_buffers(uint8_t n);

/* Returns true when the requested capture has completed */
bool ddsli_capture_ready(void);

/* Read captured ADC and reference buffers.
 *
 * Copies up to `len` samples starting at offset `ofs` from the internal
 * capture buffers into user-provided memory.
 *
 * - Data layout: 32-bit words (packed 2×16-bit samples)
 * - `ofs` and `len` are expressed in samples (32-bit words)
 * - Either `adc` or `ref` may be NULL
 *
 * Returns true on success.
 * Returns false if capture is not complete or parameters are out of range.
 */
bool ddsli_capture_read(uint32_t *adc, uint32_t *ref, uint32_t len, uint32_t ofs);

/* Direct access to capture buffers.
 * Note: buffers may reside in CCM memory.
 */
uint32_t *ddsli_get_capt_adc(void);
uint32_t *ddsli_get_capt_ref(void);
