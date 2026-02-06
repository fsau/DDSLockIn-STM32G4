/*
 * DDS + Lock-In Demodulator
 *
 * Streaming DDS signal generator with synchronous demodulation.
 * Fully deterministic processing using circular buffers split into halves.
 * ADC and DAC run continuously via DMA, CPU processes completed half-buffers.
 */

 /*
    TODO:

    - Add residuals to demod (1 or per basis?)
    - Change timers triggers to two timers:
        - ADC (slave) after DAC (master), adjustable timing
        - Check what's causing phase offset, is it only timing?
    - Fix half-buffer flags & DMA interrupts: flags for every buffer piece
    - ADC/sincos capture buffers for oscilloscope view/diagnostics
 */

#include "ddsli.h"
#include <stdint.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/g4/nvic.h>
#include "cordic.h"
#include "adc.h"
#include "dac.h"
#include "dma_memcpy.h"

// -----------------------------------------------------------------------------
// Type definitions
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

// Compressed 12 bit samples for oscilloscope output
// 4 samples in 3×32 bits, 25% less memory
typedef union {
    uint32_t data[3];
} dual_adc_compressed_t;

typedef struct {
    int16_t A1;           // Coefficient for sin (Q1.15)
    int16_t B1;           // Coefficient for cos (Q1.15)
    int16_t A2;           // Coefficient for sin (Q1.15)
    int16_t B2;           // Coefficient for cos (Q1.15)
    int16_t output_scale; // Additional scaling
} dds_out_ctrl_t;

// -----------------------------------------------------------------------------
// Buffers & globals
// -----------------------------------------------------------------------------

// #define SKIP_CPU_LC

#define CAPT_BUFF_HALVES 4
volatile dual_adc_sample_t adc_captbuf[CAPT_BUFF_HALVES * HB_LEN];             
volatile cordic_out_sample_t sincos_captbuf[CAPT_BUFF_HALVES * HB_LEN]; 

#ifdef SKIP_CPU_LC
#define SINCOS_BUFF_HALVES 2
#define dac_buf sincos_buf
#else
#define SINCOS_BUFF_HALVES 3
volatile dual_adc_sample_t dac_buf[2 * HB_LEN];        
#endif
volatile cordic_in_phase_t phase_buf[2 * HB_LEN]; 
volatile cordic_out_sample_t sincos_buf[SINCOS_BUFF_HALVES * HB_LEN]; 
volatile dual_adc_sample_t adc_buf[2 * HB_LEN];             
#define LPF_FIFO_LEN 1U
volatile ddsli_output_t lpf_fifo[LPF_FIFO_LEN];
volatile uint32_t lpf_fifo_wr = 0;
volatile uint32_t lpf_fifo_rd = 0;

// Default DAC output coefficients (unused currently)
dds_out_ctrl_t dds_linear_comb = {
    .A1 = 0x7FFF,
    .B1 = 0,
    .A2 = 0,
    .B2 = 0x7FFF,
    .output_scale = 1
};
dds_phase_ctrl_t phase_dds;
volatile int *dac_half_flag_ptr = NULL;
volatile int *dac_full_flag_ptr = NULL;
volatile int *cordic_done_flag_ptr = NULL;
volatile bool *cordic_busy_flag_ptr = NULL;
uint32_t ddsli_current_half = 0; // 0 to 6
bool stop_flag = 0;

// -----------------------------------------------------------------------------
// DDS Phase Vector Generation
// -----------------------------------------------------------------------------

static inline void dds_generate_phase_halfbuffer(
    dds_phase_ctrl_t *ctrl,
    volatile cordic_in_phase_t *dst,
    uint32_t len
)
{
    dds_phase_t phase = ctrl->phase;           
    dds_phase_inc_t inc = ctrl->phase_inc;     
    const dds_phase_inc_delta_t slope = ctrl->phase_inc_delta; 

    for (uint32_t i = 0; i < len; i++) {
        inc += slope;
        phase += inc;
        dst[i].amp = 0x6000;
        dst[i].phase = phase >> 48;
    }

    ctrl->phase     = phase;
    ctrl->phase_inc = inc;
}

static inline void dds_generate_phase_halfbuffer32(
    dds_phase_ctrl_t *ctrl,
    volatile cordic_in_phase_t *dst,
    uint32_t len
)
{
    int32_t phase = ctrl->phase >> 32;           
    int32_t inc = ctrl->phase_inc >> 32;         
    int32_t slope = ctrl->phase_inc_delta >> 32; 

    for (uint32_t i = 0; i < len; i++) {
        inc += slope;
        phase += inc;
        dst[i].amp = 0x6000;
        dst[i].phase = phase >> 16;
    }

    ctrl->phase     = (int64_t)phase << 32;
    ctrl->phase_inc = (int64_t)inc << 32;
}

// Convenience wrapper for phase generation half-buffer processing
static inline void dds_generate_phase_halfbuffer_idx(
    uint32_t buffer_half_idx)
{
    volatile cordic_in_phase_t *dst = &phase_buf[(buffer_half_idx%2) * HB_LEN];
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
// Vector Linear Combination: A*sin + B*cos → DAC
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
            result.adc2_data = (uint16_t)(-32768);
        } else {
            result.adc2_data = (int32_t)combined2;
        }

        // Write to DAC buffer (scale 16-bit to 12-bit)
        // DAC expects 12-bit right-aligned in 16-bit word
        dac_dest[i].raw = result.raw;
    }
}

// Simple copy
static inline void dds_process_linear_combination_copy(
    volatile cordic_out_sample_t *sincos_src,
    volatile dual_adc_sample_t *dac_dest,
    uint32_t len,
    dds_out_ctrl_t *params
)
{
    dma_memcpy32((uint32_t*)sincos_src,(uint32_t*)dac_dest, len);
}

// Convenience wrapper for half-buffer processing
static inline void dds_process_dac_halfbuffer(uint32_t dac_half_idx, 
                                              uint32_t sincos_half_idx)
{
    volatile cordic_out_sample_t *sincos_src = &sincos_buf[((sincos_half_idx)&SINCOS_BUFF_HALVES) * HB_LEN];
    volatile dual_adc_sample_t *dac_dest = (dual_adc_sample_t*)&dac_buf[((dac_half_idx)%2) * HB_LEN];
    dds_process_linear_combination_copy(sincos_src, dac_dest, HB_LEN, &dds_linear_comb);
}

// -----------------------------------------------------------------------------
// Dual ADC Mixing: ADC × sincos → Demodulation products → Output FIFO
// -----------------------------------------------------------------------------

static inline void dds_mixold_adc_sincos(
    volatile dual_adc_sample_t *adc_src,
    volatile cordic_out_sample_t *sincos_src,
    uint32_t len)
{
    float acc_ch0_cos = 0.0f;
    float acc_ch0_sin = 0.0f;
    float acc_ch1_cos = 0.0f;
    float acc_ch1_sin = 0.0f;

    for (uint32_t i = 0; i < len; i++) {
        /* ADC: 12-bit unsigned → signed float */
        float adc_ch0 = (float) adc_src[i].adc1_data - 2048.0f;
        float adc_ch1 = (float) adc_src[i].adc2_data - 2048.0f;
        float cos_val = (float) sincos_src[i].cos / (float) 32768.0f;
        float sin_val = (float) sincos_src[i].sin / (float) 32768.0f;
        acc_ch0_cos += adc_ch0 * cos_val;
        acc_ch0_sin += adc_ch0 * sin_val;
        acc_ch1_cos += adc_ch1 * cos_val;
        acc_ch1_sin += adc_ch1 * sin_val;
    }

    uint32_t next_wr = (lpf_fifo_wr + 1) % LPF_FIFO_LEN;
    const float scale = 1.0f / 4096.0f / (len / 2);
    lpf_fifo[lpf_fifo_wr].chA[0] = acc_ch0_cos * scale;
    lpf_fifo[lpf_fifo_wr].chA[1] = acc_ch0_sin * scale;
    lpf_fifo[lpf_fifo_wr].chB[0] = acc_ch1_cos * scale;
    lpf_fifo[lpf_fifo_wr].chB[1] = acc_ch1_sin * scale;
    lpf_fifo_wr = next_wr;
}

static inline void dds_capt_adc_sincos(
    volatile dual_adc_sample_t *adc_src,
    volatile cordic_out_sample_t *sincos_src,
    uint32_t len)
{
    static uint8_t half = 0;
    half = (half + 1) % CAPT_BUFF_HALVES;
    dual_adc_sample_t *dbadc = (dual_adc_sample_t*)&adc_captbuf[half * HB_LEN];
    cordic_out_sample_t *dbsin = (cordic_out_sample_t*)&sincos_captbuf[half * HB_LEN];

    for(uint32_t i = 0; i < len; i++)
    {
        dbadc[i].raw = adc_src[i].raw;
        dbsin[i].raw = sincos_src[i].raw;
    }
}

static inline void dds_mix_adc_sincos(
    volatile dual_adc_sample_t   *adc_src,
    volatile cordic_out_sample_t *sincos_src,
    uint32_t len)
{
    /* Accumulators: H^T x */
    float sx0 = 0.0f, sx1 = 0.0f, sx2 = 0.0f;
    float sy0 = 0.0f, sy1 = 0.0f, sy2 = 0.0f;

    /* Accumulators: H^T H */
    float scc = 0.0f, sss = 0.0f, scs = 0.0f;
    float sc  = 0.0f, ss  = 0.0f;
    float s1  = (float)len;

    for (uint32_t i = 0; i < len; i++) {
        float x0 = (float)adc_src[(i)%(len*SINCOS_BUFF_HALVES)].adc1_data - 2048.0f;
        float x1 = (float)adc_src[(i)%(len*SINCOS_BUFF_HALVES)].adc2_data - 2048.0f;

        float c = (float)sincos_src[(i)%(len*SINCOS_BUFF_HALVES)].cos * (1.0f / 32768.0f);
        float s = (float)sincos_src[(i)%(len*SINCOS_BUFF_HALVES)].sin * (1.0f / 32768.0f);

        /* H^T x */
        sx0 += x0 * c;
        sx1 += x0 * s;
        sx2 += x0;

        sy0 += x1 * c;
        sy1 += x1 * s;
        sy2 += x1;

        /* H^T H */
        scc += c * c;
        sss += s * s;
        scs += c * s;
        sc  += c;
        ss  += s;
    }

    /*
     * Normal matrix:
     * [ scc  scs  sc ]
     * [ scs  sss  ss ]
     * [ sc   ss   s1 ]
     */

    float det =
        scc * (sss * s1 - ss * ss) -
        scs * (scs * s1 - ss * sc) +
        sc  * (scs * ss - sss * sc);

    if (det == 0.0f) {
        return; /* ill-conditioned */
    }

    float inv_det = 1.0f / det;

    /* Inverse(H^T H) */
    float i00 =  (sss * s1 - ss * ss) * inv_det;
    float i01 = -(scs * s1 - ss * sc) * inv_det;
    float i02 =  (scs * ss - sss * sc) * inv_det;

    float i10 = i01;
    float i11 =  (scc * s1 - sc * sc) * inv_det;
    float i12 = -(scc * ss - scs * sc) * inv_det;

    float i20 = i02;
    float i21 = i12;
    float i22 =  (scc * sss - scs * scs) * inv_det;

    /* Solve for channel A */
    float IA = i00 * sx0 + i01 * sx1 + i02 * sx2;
    float QA = i10 * sx0 + i11 * sx1 + i12 * sx2;
    float DCA = i20 * sx0 + i21 * sx1 + i22 * sx2;

    /* Solve for channel B */
    float IB = i00 * sy0 + i01 * sy1 + i02 * sy2;
    float QB = i10 * sy0 + i11 * sy1 + i12 * sy2;
    float DCB = i20 * sy0 + i21 * sy1 + i22 * sy2;

    uint32_t next_wr = (lpf_fifo_wr + 1) % LPF_FIFO_LEN;

    lpf_fifo[lpf_fifo_wr].chA[0] = IA;
    lpf_fifo[lpf_fifo_wr].chA[1] = QA;
    lpf_fifo[lpf_fifo_wr].chA[2] = DCA;

    lpf_fifo[lpf_fifo_wr].chB[0] = IB;
    lpf_fifo[lpf_fifo_wr].chB[1] = QB;
    lpf_fifo[lpf_fifo_wr].chB[2] = DCB;

    lpf_fifo_wr = next_wr;
}

// Convenience wrapper for half-buffer mixing
static inline void dds_mix_adc_halfbuffer(uint32_t adc_half_idx, uint32_t sincos_offset)
{
    volatile dual_adc_sample_t *adc_src = &adc_buf[((adc_half_idx)%2) * HB_LEN];
    volatile cordic_out_sample_t *sincos_src = &sincos_buf[((sincos_offset)%SINCOS_BUFF_HALVES) * HB_LEN];
    
    dds_mix_adc_sincos(adc_src, sincos_src, HB_LEN);
    // dds_capt_adc_sincos(adc_src, sincos_src, HB_LEN);
}

// -----------------------------------------------------------------------------
// CORDIC Control
// -----------------------------------------------------------------------------

static inline void dds_codic_halfbuffer(uint32_t buffer_half_idx, uint32_t sincos_offset)
{
    volatile cordic_in_phase_t *src = &phase_buf[((buffer_half_idx)%2) * HB_LEN];
    volatile cordic_out_sample_t *dst = &sincos_buf[((sincos_offset)%SINCOS_BUFF_HALVES) * HB_LEN];

    cordic_start_dma((volatile uint32_t *)src,
                     (volatile uint32_t *)dst, 
                        HB_LEN);

}

// -----------------------------------------------------------------------------
// Setup / Initialization
// -----------------------------------------------------------------------------

void ddsli_setup(void)
{
    // Assumes ADC and DAC are triggered by same source (a timer, for
    // example) that is currently disabled, otherwise buffers will
    // be unsynchronized. After setup you can turn them on as needed.

    // Initialize peripherals.
    adc_dual_dma_circular_init((uint32_t *)adc_buf, 2 * HB_LEN);
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
    (uint64_t)((32767.75f * (1ULL << 32)) / 2000000.0f); 
    phase_dds.phase_inc_delta = 1000000000;
    
    // Generate initial phases for both halves
    dds_generate_phase_halfbuffer_idx(0);
    dds_generate_phase_halfbuffer_idx(1);
    
    ddsli_current_half = 0;

    dds_codic_halfbuffer(0, 0);
    // Blocking wait for CORDIC completion only
    while (*cordic_busy_flag_ptr) {
        __asm__("nop");
    }

    dds_codic_halfbuffer(1, 1);
    dds_generate_phase_halfbuffer_idx(0);
    while (*cordic_busy_flag_ptr) {
        __asm__("nop");
    }

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

/*
Initilization steps: (sincos -> DAC is omitted for simplicity)

 1. Load phase 0 and 1
 2. Run CORDIC on phase 0 and 1 -> sincos 0 and 1 -> dac 0 and 1
 3. Load phase 2
 4. Set halfs = 1
 5. Start DAC/ADC
- Wait DAC half buffer: dac/adc 0 done. halfs = 2
 6. Run CORDIC on phase 2 -> sincos 2 -> dac 2
 7. Run demod on adc 0/sincos 0
 8. Load phase 3 
- Wait DAC full buffer: dac/adc 3 done. halfs = 3
 9. Run CORDIC on phase 3 -> sincos 3 -> dac 3
 10. Run demod for adc 1/sincos 1
 11. Load phase 4 
- Wait DAC half buffer: dac/adc 0 done. halfs = 4
 12. Run CORDIC on phase 4 -> sincos 4 -> dac 4 
 13. Run demod for adc 2/sincos 2
 14. Load phase 5
- Wait....
*/

void ddsli_stop(void)
{
    stop_flag = 1;
}

bool ddsli_step(void)
{
    uint8_t sincos_thirds = ddsli_current_half % SINCOS_BUFF_HALVES;
    uint8_t buffers_half = ddsli_current_half % 2; // last complete half

    // Check if DAC buffer is ready to be updated (half or full buffer played)
    if (!(*dac_half_flag_ptr) &&
        !(*dac_full_flag_ptr) &&
        !(*cordic_done_flag_ptr))
    {
        timer_disable_counter(TIM6); // shoudn't happen
        return 0; // DAC still playing, not ready for next step
    }
    else if (*cordic_done_flag_ptr)
    {
        (*cordic_done_flag_ptr)--;

        dds_process_dac_halfbuffer(buffers_half, sincos_thirds);

        if(stop_flag) { // custom debug halt for checking buffers
            // stop_flag = 0;
            timer_disable_counter(TIM6);
            // dac_stop();
            // adc_stop();
            return 3;
        }

        ddsli_current_half = (ddsli_current_half + 1) % (2*SINCOS_BUFF_HALVES); 
        return 2;
    }
    else if (*dac_half_flag_ptr)
    {
        (*dac_half_flag_ptr)--;
        if(buffers_half == 1) return 0; // skip if not the expected half
    }
    else if (*dac_full_flag_ptr)
    {
        (*dac_full_flag_ptr)--;
        if(buffers_half == 0) return 0;
    }
   
    dds_codic_halfbuffer(buffers_half, sincos_thirds);
    dds_generate_phase_halfbuffer_idx(!buffers_half);
    dds_mix_adc_halfbuffer(buffers_half, (sincos_thirds + 1) % SINCOS_BUFF_HALVES); 
    
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
    lpf_fifo_rd = (lpf_fifo_rd + 2) % LPF_FIFO_LEN;
    return true;
}

// PendSV Handler (runs after all higher priority ISRs)
void pend_sv_handler(void) {
    SCB_ICSR |= SCB_ICSR_PENDSVCLR;
    gpio_set(GPIOC, GPIO6);
    ddsli_step();
    gpio_clear(GPIOC, GPIO6);
}
