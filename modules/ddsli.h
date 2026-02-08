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
#define HB_LEN 256U

// Output FIFO samples size
#define LPF_FIFO_LEN 64U

// ADC/reference capture half-buffers
#define CAPT_BUFF_HALVES 4

/* Initialize internal state, buffers, and peripherals.
 * Must be called once before any processing.
 */
void ddsli_setup(void);

void dds_set_frequency(float f, float sweep, float sample_f);

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
 * Lock-In Output interface
 * -------------------------------------------------------------------------- */

/* Check whether at least one output sample is available in the FIFO. */
bool ddsli_output_ready(void);

/* Pop one demodulated output sample from the FIFO.
 *
 * Returns true if a sample was written to *out.
 * Returns false if the FIFO was empty.
 */
bool ddsli_output_pop(ddsli_output_t *out);

/* --------------------------------------------------------------------------
 * ADC/Reference Capture Interface 
 * -------------------------------------------------------------------------- */

/* Trigger a buffer capture for next n frames/half-buffers where
 * n <= CAPT_BUFF_HALVES
 */
void ddsli_capture_buffers(uint8_t n);

/* Returns true if capture buffers are ready for reading */
bool ddsli_capture_ready(void);

/* Reads captured buffers */
bool dssli_capture_read(uint32_t *adc, uint32_t *ref);

/* Or get the buffer directly (check if it's CCM/RAM) */
uint32_t *ddsli_get_capt_adc(void);
uint32_t *ddsli_get_capt_ref(void);