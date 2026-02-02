#include "cordic.h"
#include <stdlib.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/cordic.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/dmamux.h>
#include <libopencm3/stm32/g4/dmamux.h>
#include <libopencm3/stm32/g4/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/g4/nvic.h>

#define CORDIC_DMA_WRITE_CHANNEL DMA_CHANNEL3
#define CORDIC_DMA_READ_CHANNEL  DMA_CHANNEL4
#define CORDIC_DMAMUX_WRITE_CH   DMA_CHANNEL3
#define CORDIC_DMAMUX_READ_CH    DMA_CHANNEL4

static volatile int transfer_in_progress = 0;
static volatile int transfer_done = 0;

void cordic_init(void)
{
    /* Enable peripheral clocks */
    rcc_periph_clock_enable(RCC_CORDIC);
    rcc_periph_clock_enable(RCC_DMAMUX1);
    rcc_periph_clock_enable(RCC_DMA1);

    /* configure CORDIC for 16-bit argument/result (returns packed 32-bit results) */
    cordic_set_function(CORDIC_CSR_FUNC_COS);
    cordic_set_precision(CORDIC_CSR_PRECISION_ITER_20);
    cordic_set_argument_width_16bit();
    cordic_set_result_width_16bit();
    cordic_set_number_of_arguments_1();
    cordic_set_number_of_results_1(); /* one 32-bit read contains two 16-bit results */
}

int cordic_start_dma(volatile uint32_t *write_buf32, volatile uint32_t *read_buf32, size_t len)
{
    if (!write_buf32 || !read_buf32 || len == 0)
        return -1;
    if (transfer_in_progress)
        return -2; /* busy */

    /* configure DMAMUX: map dma channels to cordic requests */
    dmamux_reset_dma_channel(DMAMUX1, CORDIC_DMAMUX_WRITE_CH);
    dmamux_set_dma_channel_request(DMAMUX1, CORDIC_DMAMUX_WRITE_CH, DMAMUX_CxCR_DMAREQ_ID_CORDIC_WRITE);

    dmamux_reset_dma_channel(DMAMUX1, CORDIC_DMAMUX_READ_CH);
    dmamux_set_dma_channel_request(DMAMUX1, CORDIC_DMAMUX_READ_CH, DMAMUX_CxCR_DMAREQ_ID_CORDIC_READ);

    /* Reset and clear DMA write channel (memory -> peripheral) */
    dma_channel_reset(DMA1, CORDIC_DMA_WRITE_CHANNEL);
    dma_clear_interrupt_flags(DMA1, CORDIC_DMA_WRITE_CHANNEL, DMA_IFCR_CIF(CORDIC_DMA_WRITE_CHANNEL));
    dma_set_peripheral_address(DMA1, CORDIC_DMA_WRITE_CHANNEL, (uint32_t)&CORDIC_WDATA);

    /* Use user-supplied write buffer (32-bit words). Library won't modify it. */
    dma_set_memory_address(DMA1, CORDIC_DMA_WRITE_CHANNEL, (uint32_t)write_buf32);
    dma_set_number_of_data(DMA1, CORDIC_DMA_WRITE_CHANNEL, (uint16_t)len);
    /* 32-bit peripheral write (to trigger) and 32-bit memory words */
    dma_set_peripheral_size(DMA1, CORDIC_DMA_WRITE_CHANNEL, DMA_CCR_PSIZE_32BIT);
    dma_set_memory_size(DMA1, CORDIC_DMA_WRITE_CHANNEL, DMA_CCR_MSIZE_32BIT);
    dma_enable_memory_increment_mode(DMA1, CORDIC_DMA_WRITE_CHANNEL);
    dma_disable_peripheral_increment_mode(DMA1, CORDIC_DMA_WRITE_CHANNEL);
    dma_set_read_from_memory(DMA1, CORDIC_DMA_WRITE_CHANNEL);
    dma_set_priority(DMA1, CORDIC_DMA_WRITE_CHANNEL, DMA_CCR_PL_HIGH);

    /* Reset and clear DMA read channel (peripheral -> memory). Two results per input */
    dma_channel_reset(DMA1, CORDIC_DMA_READ_CHANNEL);
    dma_clear_interrupt_flags(DMA1, CORDIC_DMA_READ_CHANNEL, DMA_IFCR_CIF(CORDIC_DMA_READ_CHANNEL));
    dma_set_peripheral_address(DMA1, CORDIC_DMA_READ_CHANNEL, (uint32_t)&CORDIC_RDATA);

    dma_set_memory_address(DMA1, CORDIC_DMA_READ_CHANNEL, (uint32_t)read_buf32);
    dma_set_number_of_data(DMA1, CORDIC_DMA_READ_CHANNEL, (uint16_t)(len));
    dma_set_peripheral_size(DMA1, CORDIC_DMA_READ_CHANNEL, DMA_CCR_PSIZE_32BIT);
    dma_set_memory_size(DMA1, CORDIC_DMA_READ_CHANNEL, DMA_CCR_MSIZE_32BIT);
    dma_enable_memory_increment_mode(DMA1, CORDIC_DMA_READ_CHANNEL);
    dma_disable_peripheral_increment_mode(DMA1, CORDIC_DMA_READ_CHANNEL);
    dma_set_read_from_peripheral(DMA1, CORDIC_DMA_READ_CHANNEL);
    dma_set_priority(DMA1, CORDIC_DMA_READ_CHANNEL, DMA_CCR_PL_HIGH);

    /* enable transfer-complete interrupt on read channel and NVIC */
    dma_enable_transfer_complete_interrupt(DMA1, CORDIC_DMA_READ_CHANNEL);
    /* enable NVIC for the selected DMA1 channel */
    switch (CORDIC_DMA_READ_CHANNEL) {
    case 1: nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ); break;
    case 2: nvic_enable_irq(NVIC_DMA1_CHANNEL2_IRQ); break;
    case 3: nvic_enable_irq(NVIC_DMA1_CHANNEL3_IRQ); break;
    case 4: nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ); break;
    case 5: nvic_enable_irq(NVIC_DMA1_CHANNEL5_IRQ); break;
    case 6: nvic_enable_irq(NVIC_DMA1_CHANNEL6_IRQ); break;
    case 7: nvic_enable_irq(NVIC_DMA1_CHANNEL7_IRQ); break;
    default: break;
    }

    /* enable cordic dma requests */
    cordic_enable_dma_write();
    cordic_enable_dma_read();
    /* reset transfer flags and enable DMA channels. Start read channel first */
    transfer_done = 0;
    dma_enable_channel(DMA1, CORDIC_DMA_READ_CHANNEL);
    dma_enable_channel(DMA1, CORDIC_DMA_WRITE_CHANNEL);

    transfer_in_progress = 1;
    return 0;
}

int cordic_transfer_complete(void)
{
    if (transfer_done) {
        transfer_done = 0; /* clear on read */
        return 1;
    }
    return 0;
}

void cordic_abort(void)
{
    if (!transfer_in_progress)
        return;

    dma_disable_channel(DMA1, CORDIC_DMA_READ_CHANNEL);
    dma_disable_channel(DMA1, CORDIC_DMA_WRITE_CHANNEL);

    dma_clear_interrupt_flags(DMA1, CORDIC_DMA_READ_CHANNEL, DMA_IFCR_CIF(CORDIC_DMA_READ_CHANNEL));
    dma_clear_interrupt_flags(DMA1, CORDIC_DMA_WRITE_CHANNEL, DMA_IFCR_CIF(CORDIC_DMA_WRITE_CHANNEL));

    cordic_disable_dma_read();
    cordic_disable_dma_write();

    transfer_in_progress = 0;
}

/* DMA IRQ handler: unpack results and mark transfer done.
 * A handler is provided only for the configured read channel (1..7).
 */
#if CORDIC_DMA_READ_CHANNEL == 1
void dma1_channel1_isr(void)
#elif CORDIC_DMA_READ_CHANNEL == 2
void dma1_channel2_isr(void)
#elif CORDIC_DMA_READ_CHANNEL == 3
void dma1_channel3_isr(void)
#elif CORDIC_DMA_READ_CHANNEL == 4
void dma1_channel4_isr(void)
#elif CORDIC_DMA_READ_CHANNEL == 5
void dma1_channel5_isr(void)
#elif CORDIC_DMA_READ_CHANNEL == 6
void dma1_channel6_isr(void)
#elif CORDIC_DMA_READ_CHANNEL == 7
void dma1_channel7_isr(void)
#else
/* no handler */
void dma1_channel1_isr(void) { }
#endif
{
    /* check transfer complete flag */
    if (dma_get_interrupt_flag(DMA1, CORDIC_DMA_READ_CHANNEL, DMA_GIF)) {
        /* clear TC flags */
        dma_clear_interrupt_flags(DMA1, CORDIC_DMA_READ_CHANNEL, DMA_GIF);

        /* disable channels and cordic DMA requests */
        dma_disable_channel(DMA1, CORDIC_DMA_READ_CHANNEL);
        dma_disable_channel(DMA1, CORDIC_DMA_WRITE_CHANNEL);
        dma_disable_transfer_complete_interrupt(DMA1, CORDIC_DMA_READ_CHANNEL);

        cordic_disable_dma_read();
        cordic_disable_dma_write();

        /* Flush CORDIC result-ready flag by reading any remaining RDATA. */
        while (cordic_is_result_ready()) {
            (void)cordic_read_32bit_result();
        }

        transfer_in_progress = 0;
        transfer_done = 1;
    }
}
