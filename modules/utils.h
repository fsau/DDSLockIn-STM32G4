#pragma once
#include <stdlib.h>
#include <stdint.h>

// Helper function to enable and set priority for DMA channel
void dma_channel_enable_irq_with_priority(uint32_t dma_channel, uint8_t priority);