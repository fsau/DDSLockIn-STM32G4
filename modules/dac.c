/* Minimal DAC circular DMA output for STM32G4 + libopencm3
 * TIM6 trigger -> DAC (via TRGO) with DMA circular moving samples from memory
 */

#include "dac.h"
#include "utils.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/dmamux.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/g4/rcc.h>
#include <libopencm3/cm3/nvic.h>

/* Configuration choices - adapt if needed */
#define DAC_INSTANCE DAC1

/* DMA channel used (choose appropriate DMA1 channel) */
#define DAC_DMA_CHANNEL DMA_CHANNEL2
#define DAC_DMAMUX_CHANNEL DMA_CHANNEL2

static volatile int dac_running_flag = 0;
static const uint32_t *user_buf = NULL;
static size_t user_len = 0;

volatile int dac_half_flag = 0;
volatile int dac_full_flag = 0;
volatile int dac_err_flag = 0;
volatile int dma_undr_flag = 0;

void dac_init(void)
{
    /* Enable clocks */
    rcc_periph_clock_enable(RCC_DAC1);
    rcc_periph_clock_enable(RCC_DMAMUX1);
    rcc_periph_clock_enable(RCC_DMA1);
    rcc_periph_clock_enable(RCC_TIM6);

    /* TIM6 expected running/configured by caller */

    dac_set_mode(DAC_INSTANCE, DAC_MCR_SINFORMAT1 | DAC_MCR_MODE1_E_BUFF |
                               DAC_MCR_SINFORMAT2 | DAC_MCR_MODE2_E_BUFF);
    dac_disable(DAC_INSTANCE, DAC_CHANNEL_BOTH);
    dac_trigger_enable(DAC_INSTANCE, DAC_CHANNEL_BOTH);
    dac_set_trigger_source(DAC_INSTANCE, DAC_CR_TSEL1_T6 | DAC_CR_TSEL2_T6);
    dac_dma_enable(DAC_INSTANCE, DAC_CHANNEL1);
    
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO4);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO5);
}

int dac_start(volatile uint32_t *samples, size_t length)
{
    if (!samples || length == 0)
        return -1;
    if (dac_running_flag)
        return -2;

    /* remember buffer so we can restart on underrun */
    user_buf = (const uint32_t *)samples;
    user_len = length;

    /* Configure DMA1 channel for peripheral<-memory circular transfers */
    dma_channel_reset(DMA1, DAC_DMA_CHANNEL);
    dma_clear_interrupt_flags(DMA1, DAC_DMA_CHANNEL, DMA_FLAGS);
    dma_set_peripheral_address(DMA1, DAC_DMA_CHANNEL, (uint32_t)&DAC_DHR12LD(DAC1));
    dma_set_memory_address(DMA1, DAC_DMA_CHANNEL, (uint32_t)samples);
    dma_set_number_of_data(DMA1, DAC_DMA_CHANNEL, (uint16_t)length);
    dma_set_peripheral_size(DMA1, DAC_DMA_CHANNEL, DMA_CCR_PSIZE_32BIT);
    dma_set_memory_size(DMA1, DAC_DMA_CHANNEL, DMA_CCR_MSIZE_32BIT);
    dma_enable_memory_increment_mode(DMA1, DAC_DMA_CHANNEL);
    dma_disable_peripheral_increment_mode(DMA1, DAC_DMA_CHANNEL);
    dma_set_read_from_memory(DMA1, DAC_DMA_CHANNEL);
    dma_enable_circular_mode(DMA1, DAC_DMA_CHANNEL);
    dma_set_priority(DMA1, DAC_DMA_CHANNEL, DMA_CCR_PL_VERY_HIGH);

    /* Configure DMAMUX to route D'MA channel to DAC request */
    dmamux_reset_dma_channel(DMAMUX1, DAC_DMAMUX_CHANNEL);
    dmamux_set_dma_channel_request(DMAMUX1, DAC_DMAMUX_CHANNEL, DMAMUX_CxCR_DMAREQ_ID_DAC1_CH1);
    dmamux_enable_request_generator(DMAMUX1, DAC_DMAMUX_CHANNEL);

    /* Enable DMA interrupts (HT, TC, TE) and NVIC for the channel */
    dma_enable_half_transfer_interrupt(DMA1, DAC_DMA_CHANNEL);
    dma_enable_transfer_complete_interrupt(DMA1, DAC_DMA_CHANNEL);
    dma_enable_transfer_error_interrupt(DMA1, DAC_DMA_CHANNEL);

    dma_channel_enable_irq_with_priority(DAC_DMA_CHANNEL, 0);

    /* Enable DMA channel */
    dma_enable_channel(DMA1, DAC_DMA_CHANNEL);

    /* Enable DAC; TIM6 is expected to provide TRGO externally */
    dac_enable(DAC_INSTANCE, DAC_CHANNEL_BOTH);

    nvic_enable_irq(NVIC_TIM6_DAC13UNDER_IRQ);

    dac_running_flag = 1;
    return 0;
}

/* Flag accessors */
int dac_dac_half_flag(void) { return dac_half_flag; }
int dac_dac_full_flag(void) { return dac_full_flag; }
int dac_dma_error_flag(void) { return dac_err_flag; }
void dac_dma_clear_flags(void) { dac_half_flag = dac_full_flag = dac_err_flag = 0; }

void dac_stop(void)
{
    if (!dac_running_flag)
        return;

    dma_disable_channel(DMA1, DAC_DMA_CHANNEL);
    dma_clear_interrupt_flags(DMA1, DAC_DMA_CHANNEL, DMA_FLAGS);
    dac_dma_disable(DAC_INSTANCE, DAC_CHANNEL1);
    dac_disable(DAC_INSTANCE, DAC_CHANNEL1);
    /* Do not stop TIM6 here; it may be used by other peripherals */
    dac_running_flag = 0;
}

int dac_running(void)
{
    return dac_running_flag;
}

/* DMA IRQ handler for the configured DAC DMA channel.
 * Sets half/full/error flags for caller to poll and clears DMA interrupt flags.
 */
#if DAC_DMA_CHANNEL == 1
void dma1_channel1_isr(void)
#elif DAC_DMA_CHANNEL == 2
void dma1_channel2_isr(void)
#elif DAC_DMA_CHANNEL == 3
void dma1_channel3_isr(void)
#elif DAC_DMA_CHANNEL == 4
void dma1_channel4_isr(void)
#elif DAC_DMA_CHANNEL == 5
void dma1_channel5_isr(void)
#elif DAC_DMA_CHANNEL == 6
void dma1_channel6_isr(void)
#elif DAC_DMA_CHANNEL == 7
void dma1_channel7_isr(void)
#else
void dma1_channel1_isr(void) { }
#endif
{
    /* Half-transfer */
    if (dma_get_interrupt_flag(DMA1, DAC_DMA_CHANNEL, DMA_HTIF)) {
        dma_clear_interrupt_flags(DMA1, DAC_DMA_CHANNEL, DMA_HTIF);
        dac_half_flag++;
    }

    /* Transfer complete (end of buffer) */
    if (dma_get_interrupt_flag(DMA1, DAC_DMA_CHANNEL, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DAC_DMA_CHANNEL, DMA_TCIF);
        dac_full_flag++;
    }

    /* Transfer error */
    if (dma_get_interrupt_flag(DMA1, DAC_DMA_CHANNEL, DMA_TEIF)) {
        dma_clear_interrupt_flags(DMA1, DAC_DMA_CHANNEL, DMA_TEIF);
        dac_err_flag++;

        /* try to stop gracefully */
        dma_disable_channel(DMA1, DAC_DMA_CHANNEL);
        dac_dma_disable(DAC_INSTANCE, DAC_CHANNEL1);
        dac_disable(DAC_INSTANCE, DAC_CHANNEL1);
        dac_running_flag = 0;
    }
    SCB_ICSR |= SCB_ICSR_PENDSVSET;
}

void tim6_dac13under_isr(void)
{
    /* Check channel1 DMA underrun */
    if (DAC_SR(DAC_INSTANCE) & DAC_SR_DMAUDR1) {
        /* clear underrun flag by writing 1 */
        DAC_SR(DAC_INSTANCE) = DAC_SR_DMAUDR1;

        dma_undr_flag = 1;

        /* Stop DMA channel and clear flags */
        dma_disable_channel(DMA1, DAC_DMA_CHANNEL);
        dma_clear_interrupt_flags(DMA1, DAC_DMA_CHANNEL, DMA_FLAGS);

        /* Reprogram memory address/count from saved buffer and restart */
        if (user_buf && user_len) {
            dma_set_memory_address(DMA1, DAC_DMA_CHANNEL, (uint32_t)user_buf);
            dma_set_number_of_data(DMA1, DAC_DMA_CHANNEL, (uint16_t)user_len);
            dma_clear_interrupt_flags(DMA1, DAC_DMA_CHANNEL, DMA_FLAGS);
            dma_enable_channel(DMA1, DAC_DMA_CHANNEL);

            /* Ensure DAC DMA requests are enabled */
            dac_dma_enable(DAC_INSTANCE, DAC_CHANNEL1);
            dac_enable(DAC_INSTANCE, DAC_CHANNEL1);

            dac_running_flag = 1;
        }
    }
}
