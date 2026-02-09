#include "dma_memcpy.h"
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/dmamux.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <string.h>

// Use DMA1 Channel 5 (commonly available for memory-to-memory)
#define DMA_MEMCPY_DMA       DMA1
#define DMA_MEMCPY_CHANNEL   DMA_CHANNEL5
#define DMA_MEMCPY_IRQ       NVIC_DMA1_CHANNEL5_IRQ
#define DMAMUX_CHANNEL       5  // DMAMUX channel for DMA1_CH5

// Private state
static volatile bool dma_transfer_complete = false;

/**
 * DMA1 Channel 5 interrupt handler
 */
void dma1_channel5_isr(void)
{
    if (dma_get_interrupt_flag(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL, DMA_TCIF);
        dma_transfer_complete = true;
    }
}

/**
 * Initialize DMA for memory-to-memory transfers
 * Return true if initialization successful
 */
bool dma_memcpy_init(void)
{
    // Enable DMA1 clock
    rcc_periph_clock_enable(RCC_DMA1);

    // Enable DMAMUX1 clock (required for G4)
    rcc_periph_clock_enable(RCC_DMAMUX1);

    // Disable DMA channel before configuration
    dma_disable_channel(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL);

    // Clear any pending interrupts
    dma_clear_interrupt_flags(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL,
        DMA_TCIF | DMA_HTIF | DMA_TEIF | DMA_GIF);

    // Configure DMAMUX: request ID 0 = memory-to-memory (software trigger)
    dmamux_set_dma_channel_request(DMAMUX1, DMAMUX_CHANNEL, 0);

    // Configure DMA channel for memory-to-memory
    dma_channel_reset(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL);
    dma_set_read_from_memory(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL);
    dma_enable_memory_increment_mode(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL);
    dma_enable_peripheral_increment_mode(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL);
    dma_set_peripheral_size(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL, DMA_CCR_PSIZE_32BIT);
    dma_set_memory_size(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL, DMA_CCR_MSIZE_32BIT);
    dma_set_priority(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL, DMA_CCR_PL_MEDIUM);
    dma_enable_mem2mem_mode(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL);

    // Enable transfer complete interrupt only
    dma_enable_transfer_complete_interrupt(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL);

    // Enable DMA channel interrupt in NVIC
    nvic_enable_irq(DMA_MEMCPY_IRQ);
    nvic_set_priority(DMA_MEMCPY_IRQ, 0);

    dma_transfer_complete = false;

    return true;
}

/**
 * Perform a synchronous DMA memory copy
 *  - dest Destination address
 *  - src Source address
 *  - size Number of bytes to copy
 * Return true if copy successful, false on error
 */
bool dma_memcpy32(volatile uint32_t *dest, volatile uint32_t *src, uint32_t size)
{
    // Reset transfer state
    dma_transfer_complete = false;
    dma_disable_channel(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL);

    // Configure DMA addresses and transfer count
    dma_set_peripheral_address(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL, (uint32_t)src);
    dma_set_memory_address(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL, (uint32_t)dest);
    dma_set_number_of_data(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL, size);

    // Clear any pending interrupts
    dma_clear_interrupt_flags(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL,
        DMA_TCIF | DMA_HTIF | DMA_TEIF | DMA_GIF);

    // Start DMA transfer
    dma_enable_channel(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL);

    return true;
}
