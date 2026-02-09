/*
 * DDS + Lock-In Demodulator
 *
 * Streaming DDS signal generator with synchronous demodulation.
 * Continuous processing using circular buffers split into halves.
 * ADC and DAC buffers run via DMA, CPU processes completed half-buffers
 * using ISR/flags for signalling. CODIC used for calculating sine/cosine
 * in real-time.
 * The step function can be called on main or on a low priority ISR.
 */

/*
   TODO:

   - Add demods with residuals/harmonics (1 or per basis?) and overload detection
   - Fix half-buffer flags & DMA interrupts: flags for every buffer piece
       - Actually not sure if that will be useful, maybe only some checks once
         in a while?
         - Check buffer ISR counters & DMA data remaining counter
         - What action to take if they're not in sync?
            - Priority should be running the DDS/DAC continuously
            - Needs function to reset ADC (including its DMA) and trigger/enable
              it on the right moment
              - Add "dirty ADC buffer" flag for ignoring bad samples
   - Single phase half-buffer: less memory, just trigger cordic afterwards?
       - Cordic while demod is fast enough? So we don't have to wait CORDIC finish
       - Bad: cant CORDIC while calculating next phases (but can do while demod)
       - Good: can CORDIC two independent phases in a single run, if contiguous
   - OPA preamp and OPA DAC2 output, allow independent channel selection/switching
   - Calibration routines? Measuring Vref and internally connecting DAC-ADC?
   - Smooth/piecewise continuous amplitude control
   - LSR with int32/64 instead of floats? Not sure if better/faster
   - Adjust DDS/DAC/ADC/demux instances on the fly (currently hardcoded)
       - Set function pointer to each step and put #defines to global vars
       - Put global vars in a instance struct?
*/

#include "ddsli.h"
#include <stdint.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/g4/nvic.h>
#include "cordic.h"
#include "adc.h"
#include "dac.h"
#include "dma_memcpy.h"
#include "utils.h"

// -----------------------------------------------------------------------------
// Private type definitions
// -----------------------------------------------------------------------------

// Dual ADC sample format (STM32 dual ADC interleaved). Used for DAC too.
typedef union
{
    uint32_t raw;
    struct
    {
        uint16_t adc1_data; // Channel 0 (ADC1), 12-bit right-aligned
        uint16_t adc2_data; // Channel 1 (ADC2), 12-bit right-aligned
    };
} dual_adc_sample_t;

typedef union
{
    uint32_t raw;
    struct
    {
        int16_t phase;
        int16_t amp;
    };
} cordic_in_phase_t;

typedef union
{
    uint32_t raw;
    struct
    {
        int16_t cos;
        int16_t sin;
    };
} cordic_out_sample_t;

// -----------------------------------------------------------------------------
// Buffers & globals
// -----------------------------------------------------------------------------

#define SINCOS_BUFF_HALVES 3
#define DDS_GEN_INSTANCES 2

__attribute__((section(".ccm_data")))
volatile dual_adc_sample_t adc_captbuf[CAPT_BUFF_HALVES * HB_LEN];
__attribute__((section(".ccm_data")))
volatile cordic_out_sample_t sincos_captbuf[CAPT_BUFF_HALVES * HB_LEN];
__attribute__((section(".ccm_data")))
ddsli_phase_ctrl_t phase_captbuf[CAPT_BUFF_HALVES];

volatile cordic_in_phase_t phase_buf[2 * HB_LEN * DDS_GEN_INSTANCES];
volatile ddsli_phase_ctrl_t phase_buf_phase[2 * DDS_GEN_INSTANCES];

volatile cordic_out_sample_t sincos_buf[SINCOS_BUFF_HALVES * HB_LEN * DDS_GEN_INSTANCES];
volatile ddsli_phase_ctrl_t sincos_buf_phase[SINCOS_BUFF_HALVES * DDS_GEN_INSTANCES];

volatile dual_adc_sample_t dac_buf[2 * HB_LEN];
volatile dual_adc_sample_t adc_buf[2 * HB_LEN];

volatile ddsli_output_t lpf_fifo[LPF_FIFO_LEN];
volatile uint32_t lpf_fifo_wr = 0;
volatile uint32_t lpf_fifo_rd = 0;

// DAC output coefficients
ddsli_out_ctrl_t ddsli_linear_comb = {
    .A1 = 0x7FFF,
    // .B1 = 0,
    .A2 = 0,
    // .B2 = 0x7FFF,
    // .output_scale = 1
};

volatile ddsli_phase_ctrl_t phase_dds, phase_ddsB;
volatile int *adc_half_flag_ptr = NULL;
volatile int *adc_full_flag_ptr = NULL;
volatile int *dac_half_flag_ptr = NULL;
volatile int *dac_full_flag_ptr = NULL;
volatile int *cordic_done_flag_ptr = NULL;
volatile bool *cordic_busy_flag_ptr = NULL;
uint32_t ddsli_current_half = 0; // 0 to 6
uint8_t capture_buffer = 0;

// -----------------------------------------------------------------------------
// DDS Phase Vector Generation
// -----------------------------------------------------------------------------

static inline void ddsli_generate_phase_halfbuffer(
    ddsli_phase_ctrl_t *ctrl,
    volatile cordic_in_phase_t *dst,
    uint32_t len)
{
    ddsli_phase_t phase = ctrl->phase;
    ddsli_phase_inc_t inc = ctrl->phase_inc;
    ddsli_phase_inc_delta_t slope = ctrl->phase_inc_delta;

    for (uint32_t i = 0; i < len; i++)
    {
        inc += slope;
        phase += inc;
        dst[i].amp = 0x7000;
        dst[i].phase = phase >> 48;
    }

    ctrl->phase = phase;
    ctrl->phase_inc = inc;
}

static inline void ddsli_generate_phase_halfbuffer32(
    ddsli_phase_ctrl_t *ctrl,
    volatile cordic_in_phase_t *dst,
    uint32_t len)
{
    int32_t phase = ctrl->phase >> 32;
    int32_t inc = ctrl->phase_inc >> 32;
    int32_t slope = ctrl->phase_inc_delta >> 32;

    for (uint32_t i = 0; i < len; i++)
    {
        inc += slope;
        phase += inc;
        dst[i].amp = 0x4000;
        dst[i].phase = phase >> 16;
    }

    ctrl->phase = (int64_t)phase << 32;
    ctrl->phase_inc = (int64_t)inc << 32;
}

// Convenience wrapper for phase generation half-buffer processing
static inline void ddsli_generate_phase_halfbuffer_idx(
    uint32_t buffer_half_idx)
{
    volatile cordic_in_phase_t *dst = &phase_buf[(buffer_half_idx % 2) * HB_LEN];
    volatile cordic_in_phase_t *dstB = &phase_buf[((buffer_half_idx % 2) + 2) * HB_LEN];
    phase_buf_phase[buffer_half_idx % 2] = phase_dds;
    phase_buf_phase[(buffer_half_idx % 2) + 2] = phase_ddsB;
    ddsli_generate_phase_halfbuffer(&phase_dds, dst, HB_LEN);
    ddsli_generate_phase_halfbuffer(&phase_ddsB, dstB, HB_LEN);
}

// Frequency management
void ddsli_set_frequency(float f, float sweep, float sample_f)
{
    // Calculate phase increment
    volatile ddsli_phase_inc_t phase_inc = (ddsli_phase_inc_t)((f * (1ULL << 32)) / sample_f);
    phase_dds.phase_inc = phase_inc * (1ULL << 32);
    // phase_dds.sample_rate = sample_f;

    // Calculate frequency sweep parameters
    if (sweep != 0.0f)
    {
        float sweep_rate = sweep / sample_f; // Normalized sweep rate
        ddsli_phase_inc_delta_t delta = (ddsli_phase_inc_delta_t)((sweep_rate * (1ULL << 32)) / sample_f);
        phase_dds.phase_inc_delta = delta;
    }
    else
    {
        phase_dds.phase_inc_delta = 0;
    }
}

// Frequency management
void ddsli_set_frequency_dual(float f, float fb, float sweep, float sweepb, float sample_f)
{
    volatile ddsli_phase_inc_t phase_inc = (ddsli_phase_inc_t)((f * (1ULL << 32)) / sample_f);
    phase_dds.phase_inc = phase_inc * (1ULL << 32);

    volatile ddsli_phase_inc_t phase_incb = (ddsli_phase_inc_t)((fb * (1ULL << 32)) / sample_f);
    phase_ddsB.phase_inc = phase_incb * (1ULL << 32);

    if (sweep != 0.0f)
    {
        float sweep_rate = sweep / sample_f; // Normalized sweep rate
        ddsli_phase_inc_delta_t delta = (ddsli_phase_inc_delta_t)((sweep_rate * (1ULL << 32)) / sample_f);
        phase_dds.phase_inc_delta = delta;
    }
    else
    {
        phase_dds.phase_inc_delta = 0;
    }

    if (sweepb != 0.0f)
    {
        float sweep_rate = sweepb / sample_f;
        ddsli_phase_inc_delta_t delta = (ddsli_phase_inc_delta_t)((sweep_rate * (1ULL << 32)) / sample_f);
        phase_ddsB.phase_inc_delta = delta;
    }
    else
    {
        phase_ddsB.phase_inc_delta = 0;
    }
}

// -----------------------------------------------------------------------------
// Vector Linear Combination: A*sin + B*cos → DAC
// -----------------------------------------------------------------------------

// Process half-buffer: A*sin → DAC samples
// Called after CORDIC completes
static inline void ddsli_process_dac_single90deg(
    volatile cordic_out_sample_t *sincos_src,
    volatile dual_adc_sample_t *dac_dest,
    uint32_t len,
    ddsli_out_ctrl_t *params)
{
    const int16_t A1 = params->A1;
    const int16_t A2 = params->A2;

    for (uint32_t i = 0; i < len; i++)
    {
        int16_t sin_val = (int16_t)sincos_src[i].sin;
        int16_t cos_val = (int16_t)sincos_src[i].cos;

        int32_t combined1 = (int32_t)A1 * sin_val;
        int32_t combined2 = (int32_t)A2 * cos_val;

        // Scale back to Q1.15
        combined1 >>= 15;
        combined2 >>= 15;

        dac_dest[i].adc1_data = combined1;
        dac_dest[i].adc2_data = combined2;
    }
}

// Process half-buffer: A*sin → DAC samples
// Called after CORDIC completes
static inline void ddsli_process_dac_double(
    volatile cordic_out_sample_t *sincos_src,
    volatile dual_adc_sample_t *dac_dest,
    uint32_t len,
    ddsli_out_ctrl_t *params)
{
    const int16_t A1 = params->A1;
    const int16_t A2 = params->A2;

    for (uint32_t i = 0; i < len; i++)
    {
        int16_t sin_val1 = (int16_t)sincos_src[i].sin;
        int16_t sin_val2 = (int16_t)sincos_src[i + SINCOS_BUFF_HALVES * HB_LEN].sin;

        int32_t combined1 = (int32_t)A1 * sin_val1;
        int32_t combined2 = (int32_t)A2 * sin_val2;

        // Scale back to Q1.15
        combined1 >>= 15;
        combined2 >>= 15;

        dac_dest[i].adc1_data = combined1;
        dac_dest[i].adc2_data = combined2;
    }
}

static inline void ddsli_process_dac_copy_double(
    volatile cordic_out_sample_t *sincos_src,
    volatile dual_adc_sample_t *dac_dest,
    uint32_t len,
    ddsli_out_ctrl_t *params)
{
    for (uint32_t i = 0; i < len; i++)
    {
        int16_t phase, phaseB;
        phase = sincos_src[i].sin;
        phaseB = sincos_src[i + SINCOS_BUFF_HALVES * HB_LEN].sin;
        dac_dest[i].adc1_data = phase;
        dac_dest[i].adc2_data = phaseB;
    }
}

// Simple copy
static inline void ddsli_process_dac_copy_single90deg(
    volatile cordic_out_sample_t *sincos_src,
    volatile dual_adc_sample_t *dac_dest,
    uint32_t len,
    ddsli_out_ctrl_t *params)
{
    dma_memcpy32((uint32_t *)sincos_src, (uint32_t *)dac_dest, len);
}

// Convenience wrapper for half-buffer processing
static inline void ddsli_process_dac_halfbuffer(uint32_t dac_half_idx,
                                                uint32_t sincos_half_idx)
{
    volatile cordic_out_sample_t *sincos_src = &sincos_buf[((sincos_half_idx)&SINCOS_BUFF_HALVES) * HB_LEN];
    volatile dual_adc_sample_t *dac_dest = (dual_adc_sample_t *)&dac_buf[((dac_half_idx) % 2) * HB_LEN];
    ddsli_process_dac_copy_double(sincos_src, dac_dest, HB_LEN, &ddsli_linear_comb);
}

// -----------------------------------------------------------------------------
// Dual ADC Mixing: ADC × sincos → Demodulation products → Output FIFO
// -----------------------------------------------------------------------------

// Mix/multiplier + block integrator and pass to FIFO.
// Needs better windowing and real low pass filtering.
// But hey it works and is faster than least squares.
static inline ddsli_output_t ddsli_mix_adc_sincos(
    volatile dual_adc_sample_t *adc_src,
    volatile cordic_out_sample_t *sincos_src,
    uint32_t len)
{
    float acc_ch0_cos = 0.0f;
    float acc_ch0_sin = 0.0f;
    float acc_ch1_cos = 0.0f;
    float acc_ch1_sin = 0.0f;

    for (uint32_t i = 0; i < len; i++)
    {
        /* ADC: 12-bit unsigned → signed float */
        float adc_ch0 = (float)adc_src[i].adc1_data - 2048.0f;
        float adc_ch1 = (float)adc_src[i].adc2_data - 2048.0f;
        float cos_val = (float)sincos_src[i].cos;
        float sin_val = (float)sincos_src[i].sin;
        acc_ch0_cos += adc_ch0 * cos_val;
        acc_ch0_sin += adc_ch0 * sin_val;
        acc_ch1_cos += adc_ch1 * cos_val;
        acc_ch1_sin += adc_ch1 * sin_val;
    }

    ddsli_output_t ret;

    ret.chA[0] = acc_ch0_cos;
    ret.chA[1] = acc_ch0_sin;
    ret.chA[2] = 0;

    ret.chB[0] = acc_ch1_cos;
    ret.chB[1] = acc_ch1_sin;
    ret.chB[2] = 0;

    return ret;
}

// Least squares regression with 2 components + DC using floats.
// Currently returns the fitted values.
static inline ddsli_output_t ddsli_lsr_adc_sincos(
    volatile dual_adc_sample_t *adc_src,
    volatile cordic_out_sample_t *sincos_src,
    uint32_t len)
{
    /* Accumulators: H^T x */
    float sx0 = 0.0f, sx1 = 0.0f, sx2 = 0.0f;
    float sy0 = 0.0f, sy1 = 0.0f, sy2 = 0.0f;

    /* Accumulators: H^T H */
    float scc = 0.0f, sss = 0.0f, scs = 0.0f;
    float sc = 0.0f, ss = 0.0f;
    float s1 = (float)len;

    for (uint32_t i = 0; i < len; i++)
    {
        float x0 = (float)adc_src[i].adc1_data - 2048.0f;
        float x1 = (float)adc_src[i].adc2_data - 2048.0f;

        float c = (float)sincos_src[i].cos * (1.0f / 32768.0f);
        float s = (float)sincos_src[i].sin * (1.0f / 32768.0f);

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
        sc += c;
        ss += s;
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
        sc * (scs * ss - sss * sc);

    float i00 = 0, i01 = 0, i02 = 0, i10 = 0, i11 = 0, i12 = 0, i20 = 0, i21 = 0, i22 = 0,
          IA = 0, QA = 0, DCA = 0, IB = 0, QB = 0, DCB = 0;

    if (det != 0.0f)
    {
        float inv_det = 1.0f / det;

        /* Inverse(H^T H) */
        i00 = (sss * s1 - ss * ss) * inv_det;
        i01 = -(scs * s1 - ss * sc) * inv_det;
        i02 = (scs * ss - sss * sc) * inv_det;

        i10 = i01;
        i11 = (scc * s1 - sc * sc) * inv_det;
        i12 = -(scc * ss - scs * sc) * inv_det;

        i20 = i02;
        i21 = i12;
        i22 = (scc * sss - scs * scs) * inv_det;

        /* Solve for channel A */
        IA = i00 * sx0 + i01 * sx1 + i02 * sx2;
        QA = i10 * sx0 + i11 * sx1 + i12 * sx2;
        DCA = i20 * sx0 + i21 * sx1 + i22 * sx2;

        /* Solve for channel B */
        IB = i00 * sy0 + i01 * sy1 + i02 * sy2;
        QB = i10 * sy0 + i11 * sy1 + i12 * sy2;
        DCB = i20 * sy0 + i21 * sy1 + i22 * sy2;
    }

    ddsli_output_t ret;

    ret.chA[0] = IA;
    ret.chA[1] = QA;
    ret.chA[2] = DCA;

    ret.chB[0] = IB;
    ret.chB[1] = QB;
    ret.chB[2] = DCB;

    return ret;
}

// Convenience wrapper for half-buffer mixing and capture
static inline void ddsli_mix_adc_halfbuffer(uint32_t adc_half_idx, uint32_t sincos_offset)
{
    cm_disable_interrupts();
    volatile dual_adc_sample_t *adc_src = &adc_buf[((adc_half_idx) % 2) * HB_LEN];
    volatile cordic_out_sample_t *sincos_src = &sincos_buf[((sincos_offset) % SINCOS_BUFF_HALVES) * HB_LEN];
    ddsli_phase_ctrl_t curr_phase = sincos_buf_phase[sincos_offset % SINCOS_BUFF_HALVES];
    ddsli_output_t out;

    if (capture_buffer)
    {
        if (capture_buffer >= CAPT_BUFF_HALVES)
            capture_buffer = CAPT_BUFF_HALVES;

        uint32_t ofs = (CAPT_BUFF_HALVES - capture_buffer) * HB_LEN;

        cm_enable_interrupts();
        dma_memcpy32((uint32_t *)&adc_captbuf[ofs], (uint32_t *)adc_src, HB_LEN);
        dma_memcpy32((uint32_t *)&sincos_captbuf[ofs], (uint32_t *)sincos_src, HB_LEN);
        cm_disable_interrupts();
        phase_captbuf[CAPT_BUFF_HALVES - capture_buffer] = curr_phase;

        capture_buffer--;
    }
    cm_enable_interrupts();

    out = ddsli_lsr_adc_sincos(adc_src, sincos_src, HB_LEN);

    cm_disable_interrupts();
    uint32_t next_wr = (lpf_fifo_wr + 1) % LPF_FIFO_LEN;

    lpf_fifo[lpf_fifo_wr].frequency = curr_phase;

    lpf_fifo[lpf_fifo_wr].chA[0] = out.chA[0];
    lpf_fifo[lpf_fifo_wr].chA[1] = out.chA[1];
    lpf_fifo[lpf_fifo_wr].chA[2] = out.chA[2];

    lpf_fifo[lpf_fifo_wr].chB[0] = out.chB[0];
    lpf_fifo[lpf_fifo_wr].chB[1] = out.chB[1];
    lpf_fifo[lpf_fifo_wr].chB[2] = out.chB[2];

    lpf_fifo_wr = next_wr;
    cm_enable_interrupts();
}

// -----------------------------------------------------------------------------
// CORDIC Control
// -----------------------------------------------------------------------------

cordic_in_phase_t *cordic_next_src = NULL;
cordic_out_sample_t *cordic_next_dst = NULL;
uint16_t cordic_next_size = 0;

static inline void ddsli_codic_halfbuffer(uint32_t buffer_half_idx, uint32_t sincos_offset)
{
    cm_disable_interrupts();
    volatile cordic_in_phase_t *src = &phase_buf[((buffer_half_idx) % 2) * HB_LEN];
    volatile cordic_out_sample_t *dst = &sincos_buf[((sincos_offset) % SINCOS_BUFF_HALVES) * HB_LEN];

    sincos_buf_phase[sincos_offset % SINCOS_BUFF_HALVES] = phase_buf_phase[buffer_half_idx % 2];
    sincos_buf_phase[(sincos_offset % SINCOS_BUFF_HALVES) + SINCOS_BUFF_HALVES] = phase_buf_phase[(buffer_half_idx % 2) + 2];

    cm_enable_interrupts();
    cordic_start_dma((volatile uint32_t *)src,
                     (volatile uint32_t *)dst,
                     HB_LEN);
    cm_disable_interrupts();

    cordic_next_src = &src[2 * HB_LEN];
    cordic_next_dst = &dst[3 * HB_LEN];
    cordic_next_size = HB_LEN;

    cm_enable_interrupts();
}

static inline bool ddsli_codic_pending(void)
{
    cm_disable_interrupts();
    if ((cordic_next_src != NULL) && (cordic_next_dst != NULL) && cordic_next_size)
    {
        cm_enable_interrupts();
        cordic_start_dma((volatile uint32_t *)cordic_next_src,
                         (volatile uint32_t *)cordic_next_dst,
                         cordic_next_size);
        cm_disable_interrupts();

        cordic_next_src = NULL;
        cordic_next_dst = NULL;
        cordic_next_size = 0;
        cm_enable_interrupts();
        return 1;
    }
    else
    {
        cm_enable_interrupts();
        return 0;
    }
}

// -----------------------------------------------------------------------------
// Setup / Initialization
// -----------------------------------------------------------------------------

void ddsli_setup(void)
{
    // Assumes ADC and DAC are triggered alternately (with a timer, for
    // example) but is currently disabled, otherwise buffers will
    // be unsynchronized. After setup you can turn them on.
    // Note that they have to be initialized/powered (rcc clocks etc), but
    // with counter or trigger/compare outputs disabled.

    // Initialize peripherals.
    dac_init();
    adc_dual_dma_circular_init((uint32_t *)adc_buf, 2 * HB_LEN);
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
                          (uint64_t)((32768.0f * (1ULL << 32)) / 1000000.0f);
    phase_dds.phase_inc_delta = 100000;
    // phase_dds.sample_rate = 1e6;

    phase_ddsB.phase = 0;
    phase_ddsB.phase_inc = (1ULL << 32) *
                           (uint64_t)((32768.0f * (1ULL << 32)) / 1000000.0f);
    phase_ddsB.phase_inc_delta = 0;
    // phase_ddsB.sample_rate = 1e6;

    // Generate initial phases for both halves
    ddsli_generate_phase_halfbuffer_idx(0);
    ddsli_generate_phase_halfbuffer_idx(1);

    ddsli_current_half = 0;

    ddsli_codic_halfbuffer(0, 0);
    // Blocking wait for CORDIC completion only
    while (*cordic_busy_flag_ptr)
    {
        __asm__("nop");
    }

    ddsli_codic_halfbuffer(1, 1);
    ddsli_generate_phase_halfbuffer_idx(0);
    while (*cordic_busy_flag_ptr)
    {
        __asm__("nop");
    }

    dac_start((volatile uint32_t *)dac_buf, 2 * HB_LEN);
    // ADC and DAC trigger sources can be enabled now
}

// -----------------------------------------------------------------------------
// Execution Control
// -----------------------------------------------------------------------------

bool ddsli_step_ready(void)
{
    // Check DAC&CORDIC flags
    if (*dac_half_flag_ptr || *dac_full_flag_ptr || *cordic_done_flag_ptr)
    {
        return true;
    }

    return false;
}

/*
Initilization steps:
Note that adc/dac/phase buffers indexes are modulo 2, sincos is modulo 3

 1. Load phase 0 and 1
 2. Run CORDIC on phase 0 and 1 -> sincos 0 and 1 -> dac 0 and 1
 3. Load phase 2
 4. Start DAC/ADC
- Wait DAC half buffer: dac/adc 0 done. halfs = 2
 5. Run CORDIC on phase 2 -> sincos 2 -> dac 2
 6. Run demod on adc 0/sincos 0
 7. Load phase 3
- Wait DAC full buffer: dac/adc 3 done. halfs = 3
 8. Run CORDIC on phase 3 -> sincos 3 -> dac 3
 9. Run demod for adc 1/sincos 1
 10. Load phase 4
- Wait DAC half buffer: dac/adc 0 done. halfs = 4
 11. Run CORDIC on phase 4 -> sincos 4 -> dac 4
 12. Run demod for adc 2/sincos 2
 13. Load phase 5
- Repeat...
*/

int8_t ddsli_step(void)
{
    cm_disable_interrupts();

    uint8_t sincos_thirds = ddsli_current_half % SINCOS_BUFF_HALVES;
    uint8_t buffers_half = ddsli_current_half % 2; // last complete half

    // Check if DAC buffer is ready to be updated (half or full buffer played)
    if (!(*dac_half_flag_ptr) &&
        !(*dac_full_flag_ptr) &&
        !(*cordic_done_flag_ptr))
    {
        cm_enable_interrupts();
        return 0;
    }
    else if (*cordic_done_flag_ptr)
    {
        (*cordic_done_flag_ptr)--;

        if (!ddsli_codic_pending())
        {
            cm_enable_interrupts();
            ddsli_process_dac_halfbuffer(buffers_half, sincos_thirds);
            cm_disable_interrupts();
            ddsli_current_half = (ddsli_current_half + 1) % (2 * SINCOS_BUFF_HALVES);
        }

        cm_enable_interrupts();
        return 2;
    }
    else if (*dac_half_flag_ptr)
    {
        (*dac_half_flag_ptr)--;
        if (buffers_half == 1)
        {            
            cm_enable_interrupts();
            return -1; // skip if not the expected half
        }
    }
    else if (*dac_full_flag_ptr)
    {
        (*dac_full_flag_ptr)--;
        if (buffers_half == 0)
        {
            cm_enable_interrupts();
            return -1;
        }
    }

    cm_enable_interrupts();

    ddsli_codic_halfbuffer(buffers_half, sincos_thirds);
    ddsli_generate_phase_halfbuffer_idx(!buffers_half);

    cm_disable_interrupts();

    if (*cordic_done_flag_ptr)
    {
        (*cordic_done_flag_ptr)--;

        cm_enable_interrupts();
        if (!ddsli_codic_pending())
        {
            // This should not happen, but if it does...
            ddsli_mix_adc_halfbuffer(buffers_half, (sincos_thirds + 1) % SINCOS_BUFF_HALVES);
            ddsli_process_dac_halfbuffer(buffers_half, sincos_thirds);

            cm_disable_interrupts();
            ddsli_current_half = (ddsli_current_half + 1) % (2 * SINCOS_BUFF_HALVES);
            cm_enable_interrupts();
            return 2;
        }
    }

    cm_enable_interrupts();

    ddsli_mix_adc_halfbuffer(buffers_half, (sincos_thirds + 1) % SINCOS_BUFF_HALVES);

    return 1;
}

bool ddsli_output_ready(void)
{
    cm_disable_interrupts();
    bool ret = lpf_fifo_wr != lpf_fifo_rd;
    cm_enable_interrupts();

    return ret;
}

bool ddsli_output_pop(ddsli_output_t *out)
{
    cm_disable_interrupts();

    if (lpf_fifo_wr == lpf_fifo_rd)
    {
        cm_enable_interrupts();
        return false; // FIFO empty
    }

    if (out)
    {
        *out = lpf_fifo[lpf_fifo_rd];
    }

    lpf_fifo_rd = (lpf_fifo_rd + 1) % LPF_FIFO_LEN;

    cm_enable_interrupts();
    return true;
}

/* Trigger a buffer capture for next n frames/half-buffers where
 * n <= CAPT_BUFF_HALVES
 */
void ddsli_capture_buffers(uint8_t n)
{
    cm_disable_interrupts();
    if(capture_buffer) return;
    if (n > CAPT_BUFF_HALVES)
    {
        n = CAPT_BUFF_HALVES;
    }
    capture_buffer = n;
    cm_enable_interrupts();
}

/* Returns true if capture buffers are ready for reading */
bool ddsli_capture_ready(void)
{
    cm_disable_interrupts();
    bool ret = (capture_buffer == 0);
    cm_enable_interrupts();
    return ret;
}

/* Reads captured buffers */
bool ddsli_capture_read(uint32_t *adc, uint32_t *ref, uint32_t len, uint32_t ofs)
{
    const uint32_t cap_len = CAPT_BUFF_HALVES * HB_LEN;

    cm_disable_interrupts();
    if (capture_buffer != 0)
    {
        cm_enable_interrupts();
        return false; /* Capture not complete */
    }

    if (ofs >= cap_len)
    {
        cm_enable_interrupts();
        return false; /* Invalid offset */
    }

    if ((ofs + len) > cap_len)
        len = cap_len - ofs;

    if (adc) {
        for (uint32_t i = 0; i < len; i++)
            adc[i] = adc_captbuf[ofs + i].raw;
    }

    if (ref) {
        for (uint32_t i = 0; i < len; i++)
            ref[i] = sincos_captbuf[ofs + i].raw;
    }
    cm_enable_interrupts();

    return true;
}

/* Or get the buffer directly (watchout it might be in CCM) */
uint32_t *ddsli_get_capt_adc(void)
{
    return (uint32_t *)adc_captbuf;
}

uint32_t *ddsli_get_capt_ref(void)
{
    return (uint32_t *)sincos_captbuf;
}

// Main update routine, runs in low priority ISR for low latency
void pend_sv_handler(void)
{
    SCB_ICSR |= SCB_ICSR_PENDSVCLR;
    gpio_set(GPIOC, GPIO6); // tracing/debug output
    ddsli_step();
    gpio_clear(GPIOC, GPIO6);
}
