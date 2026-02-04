#include "utils.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dma.h>

// Helper function to enable and set priority for DMA channel
void dma_channel_enable_irq_with_priority(uint32_t dma_channel, uint8_t priority)
{
    uint32_t irq;
    
    // Map DMA channel to IRQ number
    switch (dma_channel) {
    case DMA_CHANNEL1:
        irq = NVIC_DMA1_CHANNEL1_IRQ;
        break;
    case DMA_CHANNEL2:
        irq = NVIC_DMA1_CHANNEL2_IRQ;
        break;
    case DMA_CHANNEL3:
        irq = NVIC_DMA1_CHANNEL3_IRQ;
        break;
    case DMA_CHANNEL4:
        irq = NVIC_DMA1_CHANNEL4_IRQ;
        break;
    case DMA_CHANNEL5:
        irq = NVIC_DMA1_CHANNEL5_IRQ;
        break;
    case DMA_CHANNEL6:
        irq = NVIC_DMA1_CHANNEL6_IRQ;
        break;
    case DMA_CHANNEL7:
        irq = NVIC_DMA1_CHANNEL7_IRQ;
        break;
    default:
        return; // Invalid channel
    }
    
    // Set priority and enable
    nvic_set_priority(irq, priority);
    nvic_enable_irq(irq);
}