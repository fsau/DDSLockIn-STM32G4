#pragma once

#include <stdlib.h>
#include <stdint.h>

/* Device-specific CCM addresses */
#define CCM_BASE        0x10000000UL
#define CCM_END         0x10002800UL
#define CCM_ALIAS_BASE  0x20005800UL

#define CCM_ALIAS(p) \
    ((void *)((uintptr_t)(p) - CCM_BASE + CCM_ALIAS_BASE))
    
#define DMA_ADDR(p) ( \
    (((uintptr_t)(p) >= CCM_BASE) && \
        ((uintptr_t)(p) <  CCM_END)) \
        ? (void *)((uintptr_t)(p) - CCM_BASE + CCM_ALIAS_BASE) \
        : (void *)(p) )

// Helper function to enable and set priority for DMA channel
void dma_channel_enable_irq_with_priority(uint32_t dma_channel, uint8_t priority);

// Helper function for printing floats without snprintf
char *fmt_f(char *p, float x, int width, int decimals);

void *amemset(void *dst, int c, size_t n);
void *amemcpy(void *dst, const void *src, size_t n);

void systick_setup(uint32_t sysclk_hz);
void delay_ms(uint32_t ms);
void bp_here(void);
void jump_to_dfu(void);

extern volatile uint32_t clock_ticks;