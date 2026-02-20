/*
 * DDS + Lock-In Demodulator
 *
 * DDS signal generator with synchronous demodulation.
 * Continuous processing using circular buffers split into halves.
 * ADC and DAC buffers run via DMA, CPU processes completed half-buffers
 * using ISR/flags for signalling. CODIC used for calculating sine/cosine
 * in real-time.
 * The step function can be called on main or on a low priority ISR.
 *
 * ---------------------------------------------
 * Internal architecture overview
 * ---------------------------------------------
 *
 * This module implements a pipelined DDS + digital lock-in using
 * double-buffered phase, sin/cos, DAC and ADC paths. The design
 * overlaps CPU, CORDIC, DMA, DAC and ADC activity to achieve
 * continuous operation with deterministic latency.
 * 
 * ================================================================
 * 1) Buffers and definitions
 * ================================================================
 *
 * The following buffers are used internally. All buffers are split
 * into half-buffers of length HB_LEN and advanced in a pipelined
 * manner.
 *
 * Capture buffers (debug / acquisition mode):
 *
 *  - adc_captbuf[CAPT_BUFF_HALVES * HB_LEN]
 *        Captured ADC samples (dual ADC packed format).
 *
 *  - sincos_captbuf[CAPT_BUFF_HALVES * HB_LEN]
 *        Captured reference sin/cos samples corresponding to ADC.
 *
 *  - phase_captbuf[CAPT_BUFF_HALVES]
 *        Phase state associated with each captured half-buffer.
 *
 * DDS generation buffers:
 *
 *  - phase_buf[2 * HB_LEN * DDS_GEN_INSTANCES]
 *        Phase samples fed to the CORDIC.
 *        Double-buffered, packed by DDS instance using a fixed offset.
 *
 *  - phase_buf_phase[2 * DDS_GEN_INSTANCES]
 *        Phase accumulator state associated with each phase half-buffer.
 *
 *  - sincos_buf[SINCOS_BUFF_HALVES * HB_LEN * DDS_GEN_INSTANCES]
 *        Sin/Cos output from the CORDIC.
 *        Three half-buffers are maintained to satisfy pipeline latency:
 *          - past    : used for lock-in demodulation
 *          - current : in-flight (unused)
 *          - future  : used for DAC synthesis
 *
 *  - sincos_buf_phase[SINCOS_BUFF_HALVES * DDS_GEN_INSTANCES]
 *        Phase state associated with each sincos half-buffer.
 *
 * DAC / ADC streaming buffers:
 *
 *  - dac_buf[2 * HB_LEN]
 *        DAC output samples (dual-channel packed format).
 *        Double-buffered.
 *
 *  - adc_buf[2 * HB_LEN]
 *        ADC input samples (dual-channel packed format).
 *        Double-buffered.
 *
 * Lock-in output FIFO:
 *
 *  - lpf_fifo[LPF_FIFO_LEN]
 *        FIFO of demodulated output samples produced by the CPU.
 *
 * Relevant compile-time definitions:
 *
 *  - HB_LEN              : samples per half-buffer
 *  - CAPT_BUFF_HALVES    : number of capture half-buffers
 *  - SINCOS_BUFF_HALVES  : number of sincos pipeline half-buffers (3)
 *  - DDS_GEN_INSTANCES   : number of DDS generators (currently 2)
 *  - LPF_FIFO_LEN        : demodulated output FIFO depth
 *
 * Notes:
 *  - Multiple DDS generators are implemented by packing buffers
 *    contiguously; buffer size is multiplied by DDS_GEN_INSTANCES
 *    and accessed using a fixed per-instance offset.
 *  - ADC and DAC buffers are single-instance; channel separation
 *    is resolved during synthesis and demodulation.
 *
 *
 * ================================================================
 * 2) Data path (single DDS)
 * ================================================================
 *
 * Nominal signal flow (operations are in []):
 *
 *   [CPU phase accumulator]
 *        -> phase_buf
 *        -> [CORDIC]
 *        -> sincos_buf
 *        -> [CPU LC or DMA memcpy]
 *        -> dac_buf
 *        -> [DAC]
 *
 * In parallel:
 *                       sincos_buf 
 *                            |
 *                            v
 *    [ADC] -> adc_buf -> [CPU demux] -> out_fifo
 *
 * ================================================================
 * 3) Temporal pipeline
 * ================================================================
 *
 * Each logical "step" corresponds to one half-buffer duration
 * (DDS_BUF_LEN samples). Buffers overlap as follows:
 *
 * |    t = n-1    |  t = n (now)  |    t = n+1    |   t = n+2    |
 * |---------------|---------------|---------------|--------------|
 * |               |               | phase_buf[1]  | phase_buf[0] |
 * | sincos_buf[0] | sincos_buf[1] | sincos_buf[3] |              |
 * |               |  dac_buf[0]   |  dac_buf[1]   |              |
 * |  adc_buf[0]   |  adc_buf[1]   |               |              |
 * |  <--out_fifo  |               |               |              |
 *
 * Buffer index toggles every half-buffer interrupt.
 *
 * ================================================================
 * 4) Per-step operations
 * ================================================================
 *
 * For each half-buffer transition (ping-pong step n):
 *
 *  1) CPU phase generation (2 steps ahead):
 *        phase[(n+2)%2] = phase_accumulator(frequency, phase_inc)
 *
 *  2) CORDIC processing (1 step ahead):
 *        sincos[(n+1)%3] = CORDIC(phase[(n+1)%2])
 *
 *  3) CPU or DMA DAC preparation (1 step ahead, after CORDIC):
 *        dac[(n+1)%2] = f(sincos[(n+1)%3])
 *
 *  4) Hardware I/O (current step):
 *        - DAC reads dac[n]
 *        - ADC writes adc[n]
 *
 *  5) CPU lock-in / demodulation (1 step behind):
 *        out = demod(adc[n-1], sincos[n-1])
 *        push(out_fifo, out)
 *
 * Notes:
 *  - Three sincos generations are in memory:
 *        n-1 : used for previous step demodulation
 *        n   : storage/unused
 *        n+1 : used for next step DAC generation
 *
 * ================================================================
 * 5) Multiple DDS
 * ================================================================
 *
 * For dual DDS operation:
 *
 *  - phase_buf[] and sincos_buf[] are logically independent per DDS
 *  - Buffers are stored contiguously:
 *        total buffer size is doubled and a fixed offset is used
 *        to address the second DDS
 *  - Each DDS channel applies its own amplitude modulation
 *  - Demodulation uses the corresponding sin/cos set per channel
 *
 * The pipeline timing remains identical; only buffer sizing and
 * addressing (base + offset) change.
 *
 * Note:
 *  - The DAC generation functions are intentionally simple and can
 *    be easily adapted for summing, multiplying, or other synthesis
 *    schemes if required.
 *
 * ================================================================
 * This structure allows:
 *  - deterministic latency
 *  - zero gaps in DAC/ADC streaming
 *  - clear separation between operations, each running on its own buffers
 *
 * Users may modify buffer sizes, formats, or processing stages
 * as long as the ping-pong ordering is preserved.
 */

/*
   TODO:

   - Add demods with residuals/harmonics (1 or per basis?) and overload detection
   - Fix half-buffer flags & DMA interrupts: flags for every buffer piece
       - Actually not sure if that would be useful, maybe only some checks once
         in a while?
         - Check buffer ISR counters & DMA data remaining counter
         - What action to take if they're not in sync?
            - Priority should be running the DDS/DAC continuously
            - Needs function to reset ADC (including its DMA) and trigger/enable
              it on the right moment (maybe preload timer and start on DAC interrupt?)
              - Also add "dirty ADC buffer" flag for ignoring bad samples
   - Single phase half-buffer: less memory, just trigger cordic afterwards?
       - Cordic while demod is fast enough? So we don't have to wait CORDIC finish
       - Bad: cant CORDIC while calculating next phases (but can do while demod)
       - Good: can CORDIC two independent phases in a single run, if contiguous
   - OPA preamp and OPA DAC2 output, allow independent channel selection/switching
   - Calibration routines? Measuring Vref and internally connecting DAC-ADC?
   - Smooth/piecewise continuous amplitude control
   - LSR with int32/64 instead of floats? Not sure if better/faster (with FPU)
   - Adjust DDS/DAC/ADC/demux instances on the fly (currently hardcoded)
       - Set function pointer to each step and transfer #defines to vars
       - Put global vars in a singele struct? (per instance)
        - Options:
            - General:
                - Half buffer length
                - Sample rate
            - DDS: 
                - Enable
                - 1or2 independent frequencies
                - 1or2 phase_bufs halves (per freq)
                - 2or3 sincos_bufs halves (per freq, demod requires 3)
                - Phase method: 64 or 32 bit calc, cordic or CPU/LUT
            - DAC:
                - Enable/disable ouput
                - Skip LC? (use sincos as dac buffer)
                - Single or double channel?
                - DAC method: 90deg fix/copy, 90deg x amp[2], 4 params LI, dual
            - ADC and demod:
                - Enable/disable capture+demod
                - Capture buffer max len
                - Input vs reference channels (if double freq)
                - Demod method: mix, lsr
                - Output LPF/decimator
     - Test running cordic in same place
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
#include "timers.h"
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

// Frequency management (dual DDS)
void ddsli_set_frequency_dual(float f, float fb, float sweep, float sweepb, float sample_f)
{
    ddsli_phase_inc_t phase_inc = (ddsli_phase_inc_t)((f * (1ULL << 32)) / sample_f);
    phase_dds.phase_inc = phase_inc * (1ULL << 32);

    ddsli_phase_inc_t phase_incb = (ddsli_phase_inc_t)((fb * (1ULL << 32)) / sample_f);
    phase_ddsB.phase_inc = phase_incb * (1ULL << 32);

    if (sweep != 0.0f)
    {
        float sweep_rate = sweep / sample_f; // Normalized sweep rate
        ddsli_phase_inc_delta_t delta = (ddsli_phase_inc_delta_t)(sweep_rate * (1ULL << 32)) / sample_f;
        phase_dds.phase_inc_delta = delta;
    }
    else
    {
        phase_dds.phase_inc_delta = 0;
    }

    if (sweepb != 0.0f)
    {
        float sweep_rate = sweepb / sample_f;
        ddsli_phase_inc_delta_t delta = (ddsli_phase_inc_delta_t)(sweep_rate * (1ULL << 32)) / sample_f;
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

// Process half-buffer for single DDS, quadrature output (90°)
// A1*sin → DAC CH1, A2*cos → DAC CH2
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

// Process half-buffer for double DDS
// A1*sin1 → DAC CH1, A2*sin2 → DAC CH2
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

// Simple copy for dual DDS
// sin1 → DAC CH1, sin2 → DAC CH2
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

// Simple copy using DMA (quadrature/90° output)
static inline void ddsli_process_dac_copy_single90deg(
    volatile cordic_out_sample_t *sincos_src,
    volatile dual_adc_sample_t *dac_dest,
    uint32_t len,
    ddsli_out_ctrl_t *params)
{
    dma_memcpy32((uint32_t *)sincos_src, (uint32_t *)dac_dest, len);
}

// Convenience wrapper for half-buffer processing
// Called after CORDIC completes
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

static inline void ddsli_codic_halfbuffer_single(uint32_t buffer_half_idx, uint32_t sincos_offset)
{
    cm_disable_interrupts();
    volatile cordic_in_phase_t *src = &phase_buf[((buffer_half_idx) % 2) * HB_LEN];
    volatile cordic_out_sample_t *dst = &sincos_buf[((sincos_offset) % SINCOS_BUFF_HALVES) * HB_LEN];

    sincos_buf_phase[sincos_offset % SINCOS_BUFF_HALVES] = phase_buf_phase[buffer_half_idx % 2];

    cm_enable_interrupts();
    cordic_start_dma((volatile uint32_t *)src,
                     (volatile uint32_t *)dst,
                     HB_LEN);
}

cordic_in_phase_t *cordic_next_src = NULL;
cordic_out_sample_t *cordic_next_dst = NULL;
uint16_t cordic_next_size = 0;

static inline void ddsli_codic_halfbuffer_double(uint32_t buffer_half_idx, uint32_t sincos_offset)
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

/*
 * Timing and synchronization assumptions:
 *
 *  - DAC and ADC are driven by hardware timers (TIM4 for DAC, TIM3 for ADC).
 *  - Timer configuration (register setup) is performed on ddsli_setup()
 *    by calling adc_dac_timer_init().
 *  - During ddsli_setup(), timer counters and trigger/compare outputs
 *    must remain disabled to avoid buffer desynchronization.
 *
 *  - DAC, ADC, DMA and CORDIC peripherals are initialized inside
 *    ddsli_setup() via dac_init(), adc_dual_dma_circular_init() and
 *    cordic_init().
 *
 * Timer behavior (configured in timer.c):
 *
 *  - TIM4 provides the DAC timing reference and triggers DAC conversion
 *    on its update event.
 *  - Due to the DAC internal one-cycle pipeline latency, the DAC DMA
 *    request is sourced from a TIM4 compare event instead of the DAC
 *    default request source.
 *
 *  - TIM3 is configured as a slave only for startup synchronization:
 *        TIM3 starts on the TIM4 update trigger to guarantee deterministic
 *        ADC/DAC phase alignment at t = 0.
 *    After startup, TIM3 runs freely and independently.
 *
 *  - TIM3 and TIM4 use identical prescaler and auto-reload values.
 *
 * Buffer indexing:
 *
 *  - phase and adc/dac buffers are indexed modulo 2 (ping-pong)
 *  - sincos buffers are indexed modulo 3
 *
 * Initialization sequence (ddsli_setup):
 *
 *  1) Initialize peripherals:
 *      - DAC, ADC, CORDIC
 *      - NVIC and internal synchronization flags
 *
 *  2) Initialize DDS state:
 *      - Initialize phase accumulators
 *      - Generate phase half-buffers 0 and 1
 *
 *  3) Prime DDS pipeline (timers still stopped):
 *      - phase[0] -> sincos[0] -> dac[0]
 *      - phase[1] -> sincos[1] -> dac[1]
 *      - Generate next phase half-buffer (overwriting)
 *
 *  4) Start DAC DMA:
 *      - DAC DMA is started, but no conversions occur until timers
 *        are enabled externally.
 *
 *  After this function returns:
 *    - ADC/DAC timer triggers may be enabled via adc_dac_timer_start()
 *    - The pipeline is fully primed and ready for streaming
 *
 * Steady-state operation:
 *
 *  - Half-buffer events advance the pipeline:
 *      * DAC reads current half-buffer
 *      * ADC writes current half-buffer
 *      * CORDIC computes future sincos
 *      * CPU demodulates past adc/sincos
 *      * CPU generates future phase data (2 steps ahead)
 */

void ddsli_setup(void)
{
    // Initialize peripherals.
    dma_memcpy_init();
    adc_dac_timer_init();
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

    ddsli_codic_halfbuffer_double(0, 0);
    // Blocking wait for CORDIC completion only
    while (*cordic_busy_flag_ptr)
    {
        __asm__("nop");
    }

    ddsli_codic_halfbuffer_double(1, 1);
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
            cm_enable_interrupts();
            return 2;
        }
        else
        {
            cm_enable_interrupts();
            return 1;
        }
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

    ddsli_codic_halfbuffer_double(buffers_half, sincos_thirds);
    ddsli_generate_phase_halfbuffer_idx(!buffers_half);

    cm_disable_interrupts();

    if (*cordic_done_flag_ptr)
    {
        (*cordic_done_flag_ptr)--;

        cm_enable_interrupts();
        if (!ddsli_codic_pending())
        {
            // This should not happen on double mode, but if it does...
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

int32_t ddsli_output_count(void)
{
    int32_t ret;
    cm_disable_interrupts();
    int32_t w = lpf_fifo_wr, r = lpf_fifo_rd;
    if(w >= r) ret = w - r;
    else ret = LPF_FIFO_LEN - r + w; // overflow
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
