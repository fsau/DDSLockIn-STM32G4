#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include "adc.h"

volatile uint32_t adc_dma_buf[ADC_BUF_LEN];
volatile uint32_t adc_dma_done = 0;

void adc_dual_freerun_setup(void)
{
    uint8_t ch1 = 0;
    uint8_t ch2 = 1;

    /* --- Clocks --- */
    rcc_periph_clock_enable(RCC_ADC12);
    rcc_periph_clock_enable(RCC_DMA1);

    /* --- ADC clock prescaler (from AHB) --- */
    adc_set_clk_prescale(ADC1,ADC_CCR_PRESC_DIV16);
    adc_set_clk_prescale(ADC2,ADC_CCR_PRESC_DIV16);

    /* --- Power down before config --- */
    adc_power_off(ADC1);
    adc_power_off(ADC2);

    /* --- Enable DMA on ADC1 (master) --- */
    adc_enable_dma(ADC1);
    // adc_enable_dma_circular_mode(ADC1);

    /* --- Dual regular simultaneous mode --- */
    ADC_CSR(ADC1) =
    (ADC_CSR(ADC1) & ~ADC_CCR_DUAL_MASK) | ADC_CCR_DUAL_REGULAR_SIMUL;

    /* --- ADC1 configuration --- */
    adc_set_continuous_conversion_mode(ADC1);
    adc_set_sample_time(ADC1, ch1, ADC_SMPR_SMP_2DOT5CYC);
    adc_set_regular_sequence(ADC1, 1, &ch1);

    /* --- ADC2 configuration --- */
    adc_set_continuous_conversion_mode(ADC2);
    adc_set_sample_time(ADC2, ch2, ADC_SMPR_SMP_2DOT5CYC);
    adc_set_regular_sequence(ADC2, 1, &ch2);

    /* --- Enable ADCs --- */
    adc_power_on(ADC1);
    adc_power_on(ADC2);

    /* --- Calibration (mandatory on G4) --- */
    adc_calibrate(ADC1);
    adc_calibrate(ADC2);

    /* --- DMA configuration --- */
    dma_channel_reset(DMA1, DMA_CHANNEL1);

    dma_set_peripheral_address(
        DMA1,
        DMA_CHANNEL1,
        (uint32_t)&ADC_CDR(ADC1)
    );

    dma_set_memory_address(
        DMA1,
        DMA_CHANNEL1,
        (uint32_t)adc_dma_buf
    );

    dma_set_number_of_data(
        DMA1,
        DMA_CHANNEL1,
        ADC_BUF_BLEN
    );

    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);

    /* 32-bit packed samples: [ADC2|ADC1] */
    dma_set_peripheral_size(
        DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_32BIT
    );
    dma_set_memory_size(
        DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_32BIT
    );

    dma_set_priority(
        DMA1, DMA_CHANNEL1, DMA_CCR_PL_VERY_HIGH
    );

    dma_enable_circular_mode(DMA1, DMA_CHANNEL1);

    dma_enable_transfer_complete_interrupt(
        DMA1, DMA_CHANNEL1
    );

    nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 2);
    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);

    dma_enable_channel(DMA1, DMA_CHANNEL1);

    /* --- Start conversions ---
     * In dual mode, starting ADC1 is sufficient
     */
    adc_start_conversion_regular(ADC1);
}

void adc_sample_dual_freerun(
    uint16_t *buf1,
    uint16_t *buf2)
{
    uint32_t last_dma_done = 0;

    last_dma_done = adc_dma_done;

    /* Wait for a new DMA transfer-complete event */
    while (adc_dma_done == last_dma_done) {
        __asm__("nop");
    }

    uint32_t *src = &adc_dma_buf[ADC_BUF_LEN];

    /* De-interleave samples */
    for (int i = 0; i < BUFF_N; i++) {
        uint32_t v = src[i];
        // buf1 = v;
        // buf1[i] = (uint16_t)(v & 0xFFFF);        /* ADC1 */
        // buf2[i] = (uint16_t)(v >> 16);           /* ADC2 */
    }
}

void dma1_channel1_isr(void)
{
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF)){
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
        adc_dma_done++;
    }
}