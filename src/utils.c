#include "utils.h"
#include "usbserial.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/scb.h>

volatile uint32_t clock_ticks = 0;

void sys_tick_handler(void)
{
    clock_ticks++; // ms
}

// Initialize SysTick for millisecond ticks
void systick_setup(uint32_t sysclk_hz)
{
    // SysTick = SYSCLK / 1000 -> 1ms tick
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(sysclk_hz / 1000 - 1);
    systick_clear();
    systick_counter_enable();
    systick_interrupt_enable();
    nvic_enable_irq(NVIC_SYSTICK_IRQ);
}

// Blocking delay in milliseconds
void delay_ms(uint32_t ms)
{
    uint32_t start = clock_ticks;
    while ((clock_ticks - start) < ms)
        __asm__("nop");
}

void *amemset(void *dst, int c, size_t n)
{
    uint8_t *d = (uint8_t *)dst;
    uint8_t v = (uint8_t)c;

    if((n <= 0)||(n > 0x10000)) return dst;

    // cm_disable_interrupts();
    while (n--)
        *d++ = v;
    // cm_enable_interrupts();

    return dst;
}

void *amemcpy(void *dst, const void *src, size_t n)
{
    uint8_t *d = (uint8_t *)dst;
    const uint8_t *s = (const uint8_t *)src;

    if((n <= 0)||(n > 0x10000)) return dst;

    // cm_disable_interrupts();
    while (n--)
        *d++ = *s++;
    // cm_enable_interrupts();

    return dst;
}

// Helper function to enable and set priority for DMA channel
void dma_channel_enable_irq_with_priority(uint32_t dma_channel, uint8_t priority)
{
    uint32_t irq;

    // Map DMA channel to IRQ number
    switch (dma_channel)
    {
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

    if (v < 0)
    {
        *t++ = '-';
        v = -v;
    }

    int32_t ip = v / scale;
    int32_t fp = v % scale;

    /* integer part */
    char ibuf[12];
    int n = 0;
    do
    {
        ibuf[n++] = '0' + (ip % 10);
        ip /= 10;
    } while (ip);

    while (n--)
        *t++ = ibuf[n];

    if (decimals)
    {
        *t++ = '.';
        for (int32_t d = scale / 10; d; d /= 10)
        {
            *t++ = '0' + (fp / d);
            fp %= d;
        }
    }

    int len = t - tmp;

    /* right-align (space padded) */
    while (len < width)
    {
        *p++ = ' ';
        width--;
    }

    amemcpy(p, tmp, len);
    return p + len;
}

// Fixed breakpoint for debug
void bp_here(void)
{
    __asm__ volatile("bkpt #0");
} 

void jump_to_dfu(void) // not working very well...
{
    usbserial_disconnect();
    nvic_disable_irq(NVIC_SYSTICK_IRQ);

    // Disable SysTick if used
    SCB_ICSR |= SCB_ICSR_PENDSTCLR;

    // Set vector table to bootloader (system memory)
    SCB_VTOR = 0x1FFF0000;

    // Get bootloader entry point
    uint32_t *bootloader_entry = (uint32_t *)(SCB_VTOR + 4);
    uint32_t jump_address = *bootloader_entry;

    // Set stack pointer from bootloader's vector table
    __asm__ volatile("msr msp, %0" : : "r"(*(volatile uint32_t *)SCB_VTOR));

    // Jump to bootloader
    void (*bootloader)(void) = (void (*)(void))jump_address;
    bootloader();

    // Never returns
    while (1)
        ;
}
