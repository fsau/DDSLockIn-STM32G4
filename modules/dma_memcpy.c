#include "dma_memcpy.h"
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/dmamux.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include "utils.h"

#define DMA_MEMCPY_DMA DMA1
#define DMA_MEMCPY_CHANNEL DMA_CHANNEL5
#define DMA_MEMCPY_IRQ NVIC_DMA1_CHANNEL5_IRQ
#define DMAMUX_CHANNEL 5

#define DMA_MEMCPY_FIFO_DEPTH 8

#define DMA_MEMCPY_ID(slot, seq) (((uint32_t)(seq) << 16) | ((uint32_t)(slot) & 0xFFFF))
#define DMA_MEMCPY_ID_SLOT(id) ((uint16_t)((id) & 0xFFFF))
#define DMA_MEMCPY_ID_SEQ(id) ((uint16_t)((id) >> 16))

typedef struct
{
    volatile uint32_t *dest;
    volatile uint32_t *src;
    uint32_t words;
    uint16_t seq;
    volatile bool done;
} dma_memcpy_req_t;

static dma_memcpy_req_t dma_fifo[DMA_MEMCPY_FIFO_DEPTH];
static volatile uint8_t dma_fifo_head = 0;
static volatile uint8_t dma_fifo_tail = 0;
static volatile uint8_t dma_fifo_count = 0;
static volatile uint16_t dma_seq = 1; /* 0 reserved = invalid */
static volatile bool dma_busy = false;

/**
 * Initialize DMA for memory-to-memory transfers
 * Return true if initialization successful
 */
bool dma_memcpy_init(void)
{
    // Enable DMA1 clock
    rcc_periph_clock_enable(RCC_DMA1);
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

    return true;
}

static void dma_memcpy_start_next(void)
{
    if (dma_fifo_count == 0)
    {
        dma_busy = false;
        return;
    }

    dma_memcpy_req_t *r = &dma_fifo[dma_fifo_tail];

    dma_disable_channel(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL);

    dma_set_peripheral_address(DMA1, DMA_CHANNEL5, (uint32_t)DMA_ADDR(r->dest));
    dma_set_memory_address(DMA1, DMA_CHANNEL5, (uint32_t)DMA_ADDR(r->src));
    dma_set_number_of_data(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL,
                           r->words);

    dma_clear_interrupt_flags(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL,
                              DMA_TCIF | DMA_HTIF | DMA_TEIF | DMA_GIF);

    dma_busy = true;
    dma_enable_channel(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL);
}

/**
 * Perform a DMA memory copy
 *  - dest Destination address
 *  - src Source address
 *  - size Number of bytes to copy
 * Return true if copy successful, false on error
 */
uint32_t dma_memcpy32(volatile uint32_t *dest,
                      volatile uint32_t *src,
                      uint32_t size)
{
    if (size == 0)
        return 0;

    if (dma_fifo_count >= DMA_MEMCPY_FIFO_DEPTH)
        return 0;

    uint8_t slot = dma_fifo_head;
    dma_memcpy_req_t *r = &dma_fifo[slot];

    r->dest = dest;
    r->src = src;
    r->words = size;
    r->seq = dma_seq++;
    r->done = false;

    if (dma_seq == 0)
        dma_seq = 1;

    dma_fifo_head = (dma_fifo_head + 1) % DMA_MEMCPY_FIFO_DEPTH;
    dma_fifo_count++;

    if (!dma_busy)
        dma_memcpy_start_next();

    return DMA_MEMCPY_ID(slot, r->seq);
}

bool dma_memcpy_is_complete(uint32_t id)
{
    if (id == 0)
        return false;

    uint16_t slot = DMA_MEMCPY_ID_SLOT(id);
    uint16_t seq = DMA_MEMCPY_ID_SEQ(id);

    if (slot >= DMA_MEMCPY_FIFO_DEPTH)
        return false;

    dma_memcpy_req_t *r = &dma_fifo[slot];

    /* Sequence mismatch = overwritten or invalid */
    if (r->seq != seq)
        return false;

    return r->done;
}

void dma1_channel5_isr(void)
{
    if (dma_get_interrupt_flag(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL, DMA_TCIF))
    {
        dma_clear_interrupt_flags(DMA_MEMCPY_DMA, DMA_MEMCPY_CHANNEL, DMA_TCIF);

        dma_memcpy_req_t *r = &dma_fifo[dma_fifo_tail];
        r->done = true;

        dma_fifo_tail = (dma_fifo_tail + 1) % DMA_MEMCPY_FIFO_DEPTH;
        dma_fifo_count--;

        dma_memcpy_start_next();
    }
}