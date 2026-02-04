/*
 * DDS + Lock-In Demodulator
 *
 * Streaming DDS signal generator with synchronous demodulation.
 * Fully deterministic processing using circular buffers split into halves.
 * ADC and DAC run continuously via DMA, CPU processes completed half-buffers.
 */

#include "ddsli.h"
#include <stdint.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/g4/nvic.h>
#include "cordic.h"
#include "adc.h"
#include "dac.h"

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

// int32 full-scale (2^31) corresponds to pi radians, range: [-pi, +pi)
typedef int64_t  dds_phase_t; // Q32.32
typedef int64_t  dds_phase_inc_t; // Q32.32 (fractional part = acc)
typedef int64_t  dds_phase_inc_delta_t; // Q32.32

typedef struct {
    dds_phase_t               phase;               // Current phase accumulator
    dds_phase_inc_t           phase_inc;           // Phase increment per sample
    dds_phase_inc_delta_t     phase_inc_delta;     // Q32.32 frequency slope
} dds_phase_ctrl_t;

// Dual ADC sample format (STM32 dual ADC interleaved). Used for DAC too.
typedef union {
    uint32_t raw;
    struct {
        uint16_t adc1_data;  // Channel 0 (ADC1), 12-bit right-aligned
        uint16_t adc2_data;  // Channel 1 (ADC2), 12-bit right-aligned
    };
} dual_adc_sample_t;

// Compressed 12 bit samples for oscilloscope output
// 4 samples in 3×32 bits, 25% less memory
typedef union {
    uint32_t data[3];
} dual_adc_compressed_t;

typedef union {
    uint32_t raw;
    struct {
        int16_t phase; 
        int16_t amp;
    };
} cordic_in_phase_t;

typedef union {
    uint32_t raw;
    struct {
        int16_t cos; 
        int16_t sin;
    };
} cordic_out_sample_t;

typedef struct {
    int16_t A1;           // Coefficient for sin (Q1.15)
    int16_t B1;           // Coefficient for cos (Q1.15)
    int16_t A2;           // Coefficient for sin (Q1.15)
    int16_t B2;           // Coefficient for cos (Q1.15)
    int16_t output_scale; // Additional scaling
} dds_out_ctrl_t;

// // Demodulation products for dual channel lock-in
// typedef struct {
//     int16_t ch0_cos;   // Channel 0 mixed with cos
//     int16_t ch0_sin;   // Channel 0 mixed with sin  
//     int16_t ch1_cos;   // Channel 1 mixed with cos
//     int16_t ch1_sin;   // Channel 1 mixed with sin
// } demod_products_t;

uint32_t steps_counter = 0;

// -----------------------------------------------------------------------------
// Buffers & globals
// -----------------------------------------------------------------------------

volatile cordic_in_phase_t phase_buf[2 * HB_LEN];    // CPU → CORDIC
volatile cordic_out_sample_t sincos_buf[3 * HB_LEN]; // CORDIC output: [cos0, sin0, cos1, sin1, ...]
volatile dual_adc_sample_t dac_buf[2 * HB_LEN];               // CPU → DAC DMA
volatile dual_adc_sample_t adc_buf[2 * HB_LEN];               // ADC DMA → CPU
volatile dual_adc_sample_t adc_osci_out[6 * HB_LEN / 4];

// LPF output FIFO (lower rate output)
#define LPF_FIFO_LEN 64U
volatile ddsli_output_t lpf_fifo[LPF_FIFO_LEN];
volatile uint32_t lpf_fifo_wr = 0;
volatile uint32_t lpf_fifo_rd = 0;

// Default DAC output coefficients
dds_out_ctrl_t dds_linear_comb = {
    .A1 = 0x7FFF,
    .B1 = 0,
    .A2 = 0,
    .B2 = 0x7FFF,
    .output_scale = 1
};

dds_phase_ctrl_t phase_dds;

// Private state
volatile int *dac_half_flag_ptr = NULL;
volatile int *dac_full_flag_ptr = NULL;
volatile int *cordic_done_flag_ptr = NULL;
volatile bool *cordic_busy_flag_ptr = NULL;

uint32_t ddsli_current_half = 0; // 0 to 6
bool dac_needs_restart = false;

// -----------------------------------------------------------------------------
// DDS Phase Control
// -----------------------------------------------------------------------------

static inline void dds_generate_phase_halfbuffer(
    dds_phase_ctrl_t *ctrl,
    volatile cordic_in_phase_t *dst,
    uint32_t len
)
{
    dds_phase_t phase = ctrl->phase;           // Q32.32
    dds_phase_inc_t inc = ctrl->phase_inc;     // Q32.32
    const dds_phase_inc_delta_t slope = ctrl->phase_inc_delta; // Q32.32

    for (uint32_t i = 0; i < len; i++) {
        // Frequency sweep update
        inc += slope;
        // Phase update
        phase += inc;
        // Export to CORDIC:
        //   high 16 bits = amplitude
        //   low 16 bits = phase
        dst[i].amp = 0x6000;
        dst[i].phase = phase >> 48;
    }

    ctrl->phase     = phase;
    ctrl->phase_inc = inc;
}

// Convenience wrapper for phase generation half-buffer processing
static inline void dds_generate_phase_halfbuffer_idx(
    uint32_t buffer_half_idx)
{
    volatile cordic_in_phase_t *dst = &phase_buf[buffer_half_idx * HB_LEN];
    dds_generate_phase_halfbuffer(&phase_dds, dst, HB_LEN);
}

// Frequency management
static inline void dds_set_frequency(float f, float sweep, float sample_f)
{
    // Calculate phase increment
    volatile dds_phase_inc_t phase_inc = (dds_phase_inc_t)((f * (1ULL << 32)) / sample_f);
    phase_dds.phase_inc = phase_inc * (1ULL << 32);

    // Calculate frequency sweep parameters
    if (sweep != 0.0f) {
        float sweep_rate = sweep / sample_f; // Normalized sweep rate
        dds_phase_inc_delta_t delta = (dds_phase_inc_delta_t)((sweep_rate * (1ULL << 32)) / sample_f);
        phase_dds.phase_inc_delta = delta;
    } else {
        phase_dds.phase_inc_delta = 0;
    }
}

// -----------------------------------------------------------------------------
// Linear Combination: A*sin + B*cos → DAC
// -----------------------------------------------------------------------------

// Process half-buffer: A*sin + B*cos → DAC samples
// Called after CORDIC completes
static inline void dds_process_linear_combination(
    volatile cordic_out_sample_t *sincos_src,
    volatile dual_adc_sample_t *dac_dest,
    uint32_t len,
    dds_out_ctrl_t *params
)
{
    const int32_t A1 = params->A1;
    const int32_t B1 = params->B1;
    const int32_t A2 = params->A2;
    const int32_t B2 = params->B2;
    const int32_t scale = params->output_scale;

    for (uint32_t i = 0; i < len; i++) {
        // Extract sin/cos from interleaved buffer
        int32_t cos_val = (int32_t)sincos_src[i].cos;  // Even indices: cos
        int32_t sin_val = (int32_t)sincos_src[i].sin;  // Odd indices: sin

        // A*sin + B*cos
        int64_t combined1 = (int64_t)A1 * sin_val + (int64_t)B1 * cos_val;
        int64_t combined2 = (int64_t)A2 * sin_val + (int64_t)B2 * cos_val;


        // Apply scaling if needed
        if (scale != 1) {
            combined1 = (combined1 * scale) >> 15; // Q1.15 scaling
            combined2 = (combined2 * scale) >> 15; // Q1.15 scaling
        }

        // Scale back to Q1.15
        combined1 >>= 15;
        combined2 >>= 15;

        // Clamp to 16-bit range
        dual_adc_sample_t result;

        if (combined1 > 32767) {
            result.adc1_data = 32767;
        } else if (combined1 < -32768) {
            result.adc1_data = -32768;
        } else {
            result.adc1_data = (int32_t)combined1;
        }

        if (combined2 > 32767) {
            result.adc2_data = 32767;
        } else if (combined2 < -32768) {
            result.adc2_data = (uint32_t)(-32768);
        } else {
            result.adc2_data = (int32_t)combined2;
        }

        // Write to DAC buffer (scale 16-bit to 12-bit)
        // DAC expects 12-bit right-aligned in 16-bit word
        dac_dest[i].raw = result.raw;
    }
}

// Convenience wrapper for half-buffer processing
static inline void dds_process_dac_halfbuffer(uint32_t dac_half_idx, 
                                              uint32_t sincos_half_idx)
{
    volatile cordic_out_sample_t *sincos_src = &sincos_buf[sincos_half_idx * HB_LEN];
    volatile dual_adc_sample_t *dac_dest = &dac_buf[dac_half_idx * HB_LEN];
    dds_process_linear_combination(sincos_src, dac_dest, HB_LEN, &dds_linear_comb);
}

// -----------------------------------------------------------------------------
// Dual ADC Mixing: ADC × sincos → Demodulation products → Output FIFO
// -----------------------------------------------------------------------------

static inline void dds_mix_adc_sincos(
    volatile dual_adc_sample_t *adc_src,
    volatile cordic_out_sample_t *sincos_src,
    // volatile demod_products_t *demod_dest, // use fifo directly
    uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        // Get dual ADC sample
        dual_adc_sample_t adc_sample = adc_src[i];
        
        // Convert ADC to signed offset binary (12-bit: 0-4095 -> -2048 to +2047)
        int32_t adc_ch0 = ((int32_t)adc_sample.adc1_data) - 2048;  // -2048 to +2047
        int32_t adc_ch1 = ((int32_t)adc_sample.adc2_data) - 2048;
        
        // Get sincos values (Q1.15)
        int32_t cos_val = (int32_t)sincos_src[i].cos;  // cos
        int32_t sin_val = (int32_t)sincos_src[i].sin;  // sin
        
        // Calculate products: ADC (Q4.12) × sincos (Q1.15) = Q5.27
        // Scale down to Q1.15 for storage: right shift by (4+12) = 16
        int64_t prod_ch0_cos = (int64_t)adc_ch0 * cos_val;
        int64_t prod_ch0_sin = (int64_t)adc_ch0 * sin_val;
        int64_t prod_ch1_cos = (int64_t)adc_ch1 * cos_val;
        int64_t prod_ch1_sin = (int64_t)adc_ch1 * sin_val;
        
        // Store to local buffer buffer
        static int32_t acc_ch0_cos = 0;
        static int32_t acc_ch0_sin = 0;
        static int32_t acc_ch1_cos = 0;
        static int32_t acc_ch1_sin = 0;
        acc_ch0_cos += (int32_t)(prod_ch0_cos >> 16);
        acc_ch0_sin += (int32_t)(prod_ch0_sin >> 16);
        acc_ch1_cos += (int32_t)(prod_ch1_cos >> 16);
        acc_ch1_sin += (int32_t)(prod_ch1_sin >> 16);
        // Decimate and store to FIFO directly
        if ((i+1) % 128 == 0) {
            // Buffer has to be divisible by decimation factor
            uint32_t next_wr = (lpf_fifo_wr + 1) % LPF_FIFO_LEN;
            // if (next_wr != lpf_fifo_rd) {
            ddsli_output_t output = {0};
            // Convert accumulated Q1.15 to float (-1.0 to +1.0)
            float scale = 1.0f / (16 * 32768.0f);
            output.chA[0] = (float)(acc_ch0_cos >> 16) * scale;
            output.chA[1] = (float)(acc_ch0_sin >> 16) * scale;
            output.chB[0] = (float)(acc_ch1_cos >> 16) * scale;
            output.chB[1] = (float)(acc_ch1_sin >> 16) * scale;
            lpf_fifo[lpf_fifo_wr] = output;
            lpf_fifo_wr = next_wr;
            // }
            acc_ch0_cos = 0;
            acc_ch0_sin = 0;
            acc_ch1_cos = 0;
            acc_ch1_sin = 0;
        }
    }
}

// Convenience wrapper for half-buffer mixing
static inline void dds_mix_adc_halfbuffer(uint32_t adc_half_idx, uint32_t sincos_offset)
{
    volatile dual_adc_sample_t *adc_src = &adc_buf[adc_half_idx * HB_LEN];
    volatile cordic_out_sample_t *sincos_src = &sincos_buf[sincos_offset * HB_LEN];
    
    dds_mix_adc_sincos(adc_src, sincos_src, HB_LEN);
}

// -----------------------------------------------------------------------------
// Setup / Initialization
// -----------------------------------------------------------------------------

void ddsli_setup(void)
{
    // Assumes ADC and DAC are triggered by same source (a timer, for
    // example) that is currently disabled, otherwise buffers will
    // be unsynchronized. After setup you can turn them on as will.
    // Seems to work fine, but another solution might be a ADC/DAC
    // counter on ISR.

    // Initialize peripherals.
    adc_dualcirc_dma_init((uint32_t *)adc_buf, 2 * HB_LEN);
    dac_init();
    cordic_init();

    nvic_set_priority(NVIC_PENDSV_IRQ, 0xFF); // min priority
    nvic_enable_irq(NVIC_PENDSV_IRQ);

    // Set pointers to DMA flags
    dac_half_flag_ptr = &dac_half_flag;
    dac_full_flag_ptr = &dac_full_flag;
    cordic_done_flag_ptr = &cordic_transfer_done;
    cordic_busy_flag_ptr = &cordic_transfer_in_progress;
    
    // Clear flags
    (*dac_half_flag_ptr) = 0;
    (*dac_full_flag_ptr) = 0;
    (*cordic_done_flag_ptr) = 0;
    (*cordic_busy_flag_ptr) = 0;
    
    // Initialize DDS phase
    phase_dds.phase = 0;
    phase_dds.phase_inc = (1ULL << 32) * 
    (uint64_t)((32767.75f * (1ULL << 32)) / 1000000.0f); 
    // phase_dds.phase_inc_delta = 1000000;
    
    // Generate initial phases for both halves
    dds_generate_phase_halfbuffer_idx(0);
    dds_generate_phase_halfbuffer_idx(1);
    
    // Start CORDIC for first half
    cordic_start_dma((volatile uint32_t *)&phase_buf[0], 
                     (volatile uint32_t *)&sincos_buf[0], 
                     HB_LEN);
    
    // Blocking wait for CORDIC completion only
    while (*cordic_busy_flag_ptr) {
        __asm__("nop");
    }
    
    // Start CORDIC for second half
    cordic_start_dma((volatile uint32_t *)&phase_buf[HB_LEN], 
                     (volatile uint32_t *)&sincos_buf[HB_LEN], 
                     HB_LEN);
    
    // Blocking wait for CORDIC completion only
    while (*cordic_busy_flag_ptr) {
        __asm__("nop");
    }
    
    ddsli_current_half = 0;
    steps_counter = 0;

    dac_start((volatile uint32_t *)dac_buf, 2 * HB_LEN);
    // ADC and DAC triggers can be enabled now
}

// -----------------------------------------------------------------------------
// Execution Control
// -----------------------------------------------------------------------------

bool ddsli_step_ready(void)
{
    // Check DAC flags
    if (*dac_half_flag_ptr || *dac_full_flag_ptr) {
        return true;
    }
    
    // // Check CORDIC completion
    // if (cordic_running && *cordic_done_flag_ptr) {
    //     return true;
    // }
    
    return false;
}

bool ddsli_step(void)
{
    uint8_t sincos_thirds = ddsli_current_half % 3;
    uint8_t buffers_half = ddsli_current_half % 2;

    // Check if DAC buffer is ready to be updated (half or full buffer played)
    if (!(*dac_half_flag_ptr) &&
        !(*dac_full_flag_ptr) &&
        !(*cordic_done_flag_ptr))
    {
        return 0; // DAC still playing, not ready for next step
    }
    else if (*cordic_done_flag_ptr)
    {
        // CORDIC finished
        (*cordic_done_flag_ptr)--;

        // Process DAC half-buffer
        dds_process_dac_halfbuffer(buffers_half, sincos_thirds);
        ddsli_current_half = (ddsli_current_half + 1) % 6;
        return 2;
    }
    else if (*dac_half_flag_ptr)
    {
        ddsli_current_half &= ~1;
        (*dac_half_flag_ptr)--;
    }
    else if (*dac_full_flag_ptr)
    {
        ddsli_current_half |= 1;
        (*dac_full_flag_ptr)--;
    }

    // Start CORDIC DMA
    if (cordic_start_dma((volatile uint32_t *)&phase_buf[(!buffers_half) * HB_LEN], 
                        (volatile uint32_t *)&sincos_buf[sincos_thirds * HB_LEN], 
                        HB_LEN) != 0)
    {
        return 0; // CORDIC busy or error
    }

    // Generate next DDS phase half-buffer
    dds_generate_phase_halfbuffer_idx(buffers_half);

    // Mix ADC half-buffer with previous CORDIC output
    dds_mix_adc_halfbuffer(buffers_half, ((sincos_thirds - 2) % 3)); 
    
    return 1;
}

bool ddsli_output_ready(void)
{
    return lpf_fifo_wr != lpf_fifo_rd;
}

bool ddsli_output_pop(ddsli_output_t *out)
{
    if (lpf_fifo_wr == lpf_fifo_rd) {
        return false; // FIFO empty
    }
    
    if (out) {
        *out = lpf_fifo[lpf_fifo_rd];
    }
    lpf_fifo_rd = (lpf_fifo_rd + 1) % LPF_FIFO_LEN;
    return true;
}

// PendSV Handler (runs after all higher priority ISRs)
void pend_sv_handler(void) {
    SCB_ICSR |= SCB_ICSR_PENDSVCLR;
    gpio_set(GPIOC, GPIO6);
    ddsli_step();
    gpio_clear(GPIOC, GPIO6);
}
