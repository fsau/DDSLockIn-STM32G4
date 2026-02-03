/*
 * DDS + Lock-In Demodulator
 *
 * Streaming DDS signal generator with synchronous demodulation.
 * Fully deterministic processing using circular buffers split into halves.
 * ADC and DAC run continuously via DMA, CPU processes completed half-buffers.
 */

#include <stdint.h>
#include "ddsli.h"
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
        int16_t amp;
        int16_t phase; 
    };
} codic_in_phase_t;

typedef union {
    uint32_t raw;
    struct {
        int16_t sin;
        int16_t cos; 
    };
} codic_out_sample_t;

typedef struct {
    int16_t A;           // Coefficient for sin (Q1.15)
    int16_t B;           // Coefficient for cos (Q1.15)
    int16_t output_scale; // Additional scaling
} dds_out_ctrl_t;

// Demodulation products for dual channel lock-in
typedef struct {
    int16_t ch0_cos;   // Channel 0 mixed with cos
    int16_t ch0_sin;   // Channel 0 mixed with sin  
    int16_t ch1_cos;   // Channel 1 mixed with cos
    int16_t ch1_sin;   // Channel 1 mixed with sin
} demod_products_t;

// -----------------------------------------------------------------------------
// Buffers & globals
// -----------------------------------------------------------------------------

volatile codic_in_phase_t phase_buf[2 * HB_LEN];    // CPU → CORDIC
volatile codic_out_sample_t sincos1_buf[2 * HB_LEN]; // CORDIC output: [cos0, sin0, cos1, sin1, ...]
volatile codic_out_sample_t sincos2_buf[2 * HB_LEN]; // Snapshot for demodulation
volatile dual_adc_sample_t dac_buf[2 * HB_LEN];               // CPU → DAC DMA
volatile dual_adc_sample_t adc_buf[2 * HB_LEN];               // ADC DMA → CPU
volatile demod_products_t demod_buf[HB_LEN];                  // Demodulator intermediate (HB_LEN products)
volatile dual_adc_sample_t adc_osci_out[6 * HB_LEN / 4];

// LPF output FIFO (lower rate output)
#define LPF_FIFO_LEN 64U
volatile ddsli_output_t lpf_fifo[LPF_FIFO_LEN];
volatile uint32_t lpf_fifo_wr = 0;
volatile uint32_t lpf_fifo_rd = 0;

// Default DAC output coefficients
dds_out_ctrl_t dds_linear_comb = {
    .A = 0x7FFF,        // 1.0 in Q1.15
    .B = 0,
    .output_scale = 1
};

dds_phase_ctrl_t phase_dds;

// Private state
volatile int *dac_half_flag_ptr = NULL;
volatile int *dac_full_flag_ptr = NULL;
volatile int *cordic_done_flag_ptr = NULL;

uint32_t current_half = 0;
bool cordic_running = false;
bool dac_needs_restart = false;

// -----------------------------------------------------------------------------
// DDS Phase Control
// -----------------------------------------------------------------------------

static inline void dds_generate_phase_halfbuffer(
    dds_phase_ctrl_t *ctrl,
    volatile codic_in_phase_t *dst,
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
        //   top 16 bits = amplitude
        //   next 16 bits = phase
        dst[i].raw = 0x7FFF0000u | (uint32_t)(phase >> 16);
    }

    ctrl->phase     = phase;
    ctrl->phase_inc = inc;
}

// Convenience wrapper for phase generation half-buffer processing
static inline void dds_generate_phase_halfbuffer_idx(
    uint32_t half_buffer_idx)
{
    codic_in_phase_t *dst = (half_buffer_idx == 0) ? 
                            &phase_buf[0] : 
                            &phase_buf[HB_LEN];
    
    dds_generate_phase_halfbuffer(&phase_dds, dst, HB_LEN);
}

// Frequency management
static inline void dds_set_frequency(float f, float sweep, float sample_f)
{
    // Calculate phase increment
    dds_phase_inc_t phase_inc = (dds_phase_inc_t)((f * (1ULL << 32)) / sample_f);
    phase_dds.phase_inc = phase_inc;

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
    volatile codic_out_sample_t *sincos_src,
    volatile dual_adc_sample_t *dac_dest,
    uint32_t len,
    dds_out_ctrl_t *params
)
{
    const int32_t A = params->A;
    const int32_t B = params->B;
    const int32_t scale = params->output_scale;

    for (uint32_t i = 0; i < len; i++) {
        // Extract sin/cos from interleaved buffer
        int32_t cos_val = (int32_t)sincos_src[i].cos;  // Even indices: cos
        int32_t sin_val = (int32_t)sincos_src[i].sin;  // Odd indices: sin

        // A*sin + B*cos
        int64_t combined = (int64_t)A * sin_val + (int64_t)B * cos_val;

        // Apply scaling if needed
        if (scale != 1) {
            combined = (combined * scale) >> 15; // Q1.15 scaling
        }

        // Scale back to Q1.15
        combined >>= 15;

        // Clamp to 16-bit range
        int32_t result;
        if (combined > 32767) {
            result = 32767;
        } else if (combined < -32768) {
            result = -32768;
        } else {
            result = (int32_t)combined;
        }

        // Write to DAC buffer (scale 16-bit to 12-bit)
        // DAC expects 12-bit right-aligned in 16-bit word
        dac_dest[i].raw = ((uint16_t)((result + 32768) >> 4) << 16) | 
                          ((uint16_t)((result + 32768) >> 4) & 0xFFFF);
    }
}

// Convenience wrapper for half-buffer processing
static inline void dds_process_dac_halfbuffer(uint32_t half_buffer_idx)
{
    codic_out_sample_t *sincos_src;
    dual_adc_sample_t *dac_dest;

    if (half_buffer_idx == 0) {
        sincos_src = &sincos1_buf[0];      // First half of sincos1
        dac_dest = &dac_buf[0];            // First half of DAC buffer
    } else {
        sincos_src = &sincos1_buf[2 * HB_LEN]; // Second half of sincos1
        dac_dest = &dac_buf[HB_LEN];       // Second half of DAC buffer
    }

    dds_process_linear_combination(sincos_src, dac_dest, HB_LEN, &dds_linear_comb);
}

// Coefficient management
static inline void dds_set_linear_combination_coeffs(int32_t A_new, int32_t B_new)
{
    dds_linear_comb.A = A_new;
    dds_linear_comb.B = B_new;
}

static inline void dds_set_output_scale(int32_t scale)
{
    dds_linear_comb.output_scale = scale;
}

static inline void dds_get_linear_combination_coeffs(int32_t *A_ptr, int32_t *B_ptr)
{
    if (A_ptr) *A_ptr = dds_linear_comb.A;
    if (B_ptr) *B_ptr = dds_linear_comb.B;
}

// -----------------------------------------------------------------------------
// Dual ADC Mixing: ADC × sincos → Demodulation products
// -----------------------------------------------------------------------------

// Multiply half-buffer of dual ADC samples with sincos
// ADC samples: interleaved dual 12-bit (ADC1/ADC2)
// sincos values: Q1.15 (-32768 to +32767)
// Output: demod_buf stores demod_products_t for each sample
static inline void dds_mix_adc_sincos(
    volatile dual_adc_sample_t *adc_src,
    volatile codic_out_sample_t *sincos_src,
    volatile demod_products_t *demod_dest,
    uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        // Get dual ADC sample
        dual_adc_sample_t adc_sample = adc_src[i];
        
        // Convert ADC to signed offset binary (12-bit: 0-4095 -> -2048 to +2047)
        int32_t adc_ch0 = ((int32_t)adc_sample.adc1_data) - 2048;  // -2048 to +2047
        int32_t adc_ch1 = ((int32_t)adc_sample.adc2_data) - 2048;
        
        // Get sincos values (Q1.15)
        int32_t cos_val = (int32_t)sincos_src[i].cos;      // cos
        int32_t sin_val = (int32_t)sincos_src[i].sin;  // sin
        
        // Calculate products: ADC (Q4.12) × sincos (Q1.15) = Q5.27
        // Scale down to Q1.15 for storage: right shift by (4+12) = 16
        int64_t prod_ch0_cos = (int64_t)adc_ch0 * cos_val;
        int64_t prod_ch0_sin = (int64_t)adc_ch0 * sin_val;
        int64_t prod_ch1_cos = (int64_t)adc_ch1 * cos_val;
        int64_t prod_ch1_sin = (int64_t)adc_ch1 * sin_val;
        
        demod_products_t products = {
            .ch0_cos = (int16_t)(prod_ch0_cos >> 32),
            .ch0_sin = (int16_t)(prod_ch0_sin >> 32),
            .ch1_cos = (int16_t)(prod_ch1_cos >> 32),
            .ch1_sin = (int16_t)(prod_ch1_sin >> 32)
        };
        
        // Store to demod buffer
        demod_dest[i] = products;
    }
}

// Convenience wrapper for half-buffer mixing
void dds_mix_adc_halfbuffer(uint32_t half_buffer_idx)
{
    const dual_adc_sample_t *adc_src;
    const codic_out_sample_t *sincos_src;
    
    if (half_buffer_idx == 0) {
        adc_src = &adc_buf[0];      // First half of ADC
        sincos_src = &sincos2_buf[0]; // First half of sincos2
    } else {
        adc_src = &adc_buf[HB_LEN]; // Second half of ADC
        sincos_src = &sincos2_buf[2 * HB_LEN]; // Second half of sincos2
    }
    
    dds_mix_adc_sincos(adc_src, sincos_src, &demod_buf[0], HB_LEN);
}

// Simple LPF processing - accumulate demod products and output when ready
void dds_process_lpf(void)
{
    static demod_products_t lpf_accumulator = {0};
    static uint32_t lpf_counter = 0;
    const uint32_t LPF_DECIMATION = 16; // Output every 16 samples
    
    // Accumulate current half-buffer
    for (uint32_t i = 0; i < HB_LEN; i++) {
        demod_products_t products = demod_buf[i];
        lpf_accumulator.ch0_cos += products.ch0_cos;
        lpf_accumulator.ch0_sin += products.ch0_sin;
        lpf_accumulator.ch1_cos += products.ch1_cos;
        lpf_accumulator.ch1_sin += products.ch1_sin;
        lpf_counter++;
        
        // Check if we should output
        if (lpf_counter >= LPF_DECIMATION) {
            // Check FIFO space
            uint32_t next_wr = (lpf_fifo_wr + 1) % LPF_FIFO_LEN;
            if (next_wr != lpf_fifo_rd) {
                ddsli_output_t output;
                
                // Convert accumulated Q1.15 to float (-1.0 to +1.0)
                float scale = 1.0f / (LPF_DECIMATION * 32768.0f);
                output.chA[0] = (float)lpf_accumulator.ch0_cos * scale;
                output.chA[1] = (float)lpf_accumulator.ch0_sin * scale;
                output.chB[0] = (float)lpf_accumulator.ch1_cos * scale;
                output.chB[1] = (float)lpf_accumulator.ch1_sin * scale;
                
                // TODO: Set actual frequency and amplitude
                output.frequency = 0.0f;
                output.dds_amplitude = 0.0f;
                
                lpf_fifo[lpf_fifo_wr] = output;
                lpf_fifo_wr = next_wr;
            }
            
            // Reset accumulator
            lpf_accumulator.ch0_cos = 0;
            lpf_accumulator.ch0_sin = 0;
            lpf_accumulator.ch1_cos = 0;
            lpf_accumulator.ch1_sin = 0;
            lpf_counter = 0;
        }
    }
}

// void ddsli_setup(void)
// {
//     // Setup ADC & enable circular mode (adc.h)
//     // Setup DAC to circular mode (dac.h)
//     // Setup CORDIC to sin/cos op (cordic.h)
//     // Setup DMA memcpy function
//     // Setup buffers and local states (zeroed)
//     // Load pointer to DMA flags (half and full)
// }

// bool ddsli_step_ready(void)
// {
//     // Return if some DMA flag is set
// }

// void ddsli_step(void)
// {
//     // Determine which half-buffer is ready
//     static uint32_t half_buffer_idx = 0;

//     // DDS routines:
//     // 1. Run CORDIC on phase_buf → sincos1_buf (DMA)
//     // 2. Generate DDS phase for next half-buffer (CPU)
//     // 3. Calculate linear combination for DAC half-buffer
//     //    (CPU, after 1 finishes)

//     // Demodulation routines:
//     // 4. Copy sincos1_buf → sincos2_buf (DMA)
//     // 5. Mix ADC half-buffer with sincos2_buf (CPU) → demod_buf
//     // 6. Process demod_buf → LPF FIFO
// }

// -----------------------------------------------------------------------------
// Setup / Initialization
// -----------------------------------------------------------------------------

void ddsli_setup(void)
{
    // Initialize peripherals
    adc_dualcirc_dma_init((volatile uint32_t *)adc_buf, 2 * HB_LEN);
    dac_init();
    cordic_init();
    
    // Set pointers to DMA flags
    dac_half_flag_ptr = &dac_half_flag;
    dac_full_flag_ptr = &dac_full_flag;
    cordic_done_flag_ptr = &cordic_transfer_done;
    
    // Clear flags
    dac_dma_clear_flags();
    cordic_transfer_done = 0;
    
    // Initialize DDS phase
    phase_dds.phase = 0;
    phase_dds.phase_inc = (uint64_t)((1000.0f * (1ULL << 32)) / 1000000.0f); // 1kHz @ 1MHz
    phase_dds.phase_inc_delta = 0;
    
    // Generate initial phases for both halves
    dds_generate_phase_halfbuffer_idx(0);
    dds_generate_phase_halfbuffer_idx(1);
    
    // Process first half through CORDIC and DAC
    codic_in_phase_t *phase_src = &phase_buf[0];
    codic_out_sample_t *sincos_dst = &sincos1_buf[0];
    
    // Start CORDIC for first half
    cordic_start_dma((volatile uint32_t *)phase_src, 
                     (volatile uint32_t *)dac_buf, 
                     HB_LEN);
    
    cordic_running = true;
    current_half = 0;

    dac_start((volatile uint32_t *)dac_buf, 2 * HB_LEN);
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
    
    // Check CORDIC completion
    if (cordic_running && *cordic_done_flag_ptr) {
        return true;
    }
    
    return false;
}

bool ddsli_step(void)
{
    // Check if DAC buffer is ready to be updated (half or full buffer played)
    if (!(*dac_half_flag_ptr) && !(*dac_full_flag_ptr)) {
        return false; // DAC still playing, not ready for next step
    }
    
    // Clear DAC flags
    *dac_half_flag_ptr = 0;
    *dac_full_flag_ptr = 0;
    
    // Generate phase for the next half that will be played
    uint32_t next_half = current_half;
    dds_generate_phase_halfbuffer_idx(next_half);
    
    // Start CORDIC for this half
    codic_in_phase_t *phase_src = (next_half == 0) ? 
                                 &phase_buf[0] : 
                                 &phase_buf[HB_LEN];
    
    codic_out_sample_t *sincos_dst = (next_half == 0) ? 
                                    &sincos1_buf[0] : 
                                    &sincos1_buf[HB_LEN];
    
    // Start CORDIC DMA
    if (cordic_start_dma((volatile uint32_t *)phase_src, 
                        (volatile uint32_t *)sincos_dst, 
                        HB_LEN) != 0) {
        return false; // CORDIC busy or error
    }
    
    // // Blocking wait for CORDIC completion only
    // while (!(*cordic_done_flag_ptr)) {
    //     // Busy wait for CORDIC
    // }
    
    // // Clear CORDIC flag
    // *cordic_done_flag_ptr = 0;
    
    // Process CORDIC results to DAC buffer
    // dds_process_dac_halfbuffer(next_half);
    
    // Move to next half for next iteration
    current_half = (next_half + 1) % 2;
    
    return true; // Step completed successfully
}

// Output FIFO interface implementation (from ddsli.h)
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
