#include "utils.h"
#include <string.h>
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

char *fmt_f(char *p, float x, int width, int decimals)
{
    /* scale = 10^decimals */
    int32_t scale = 1;
    for (int i = 0; i < decimals; i++)
        scale *= 10;

    /* convert with rounding */
    int32_t v = (int32_t)(x * scale + (x >= 0 ? 0.5f : -0.5f));

    char tmp[24];
    char *t = tmp;

    if (v < 0) {
        *t++ = '-';
        v = -v;
    }

    int32_t ip = v / scale;
    int32_t fp = v % scale;

    /* integer part */
    char ibuf[12];
    int n = 0;
    do {
        ibuf[n++] = '0' + (ip % 10);
        ip /= 10;
    } while (ip);

    while (n--)
        *t++ = ibuf[n];

    if (decimals) {
        *t++ = '.';
        for (int32_t d = scale / 10; d; d /= 10) {
            *t++ = '0' + (fp / d);
            fp %= d;
        }
    }

    int len = t - tmp;

    /* right-align (space padded) */
    while (len < width) {
        *p++ = ' ';
        width--;
    }

    memcpy(p, tmp, len);
    return p + len;
}