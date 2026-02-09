#pragma once

#include <stdlib.h>
#include <stdint.h>

/* Device-specific CCM addresses */
#define CCM_BASE        0x10000000UL
#define CCM_ALIAS_BASE  0x20005800UL   /* adjust per RM */
#define CCM_ALIAS_OFFS  (CCM_ALIAS_BASE - CCM_BASE)

// Helper function to enable and set priority for DMA channel
void dma_channel_enable_irq_with_priority(uint32_t dma_channel, uint8_t priority);

// Helper function for printing floats without snprintf
char *fmt_f(char *p, float x, int width, int decimals);
