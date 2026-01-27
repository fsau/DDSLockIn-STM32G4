#include <stdio.h>
#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include "usbserial.h"

uint32_t clock_ticks = 0;

// SysTick interrupt handler
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

// Delay in milliseconds
void delay_ms(uint32_t ms)
{
    uint32_t start = clock_ticks;
    while ((clock_ticks - start) < ms)
        __asm__("nop");
}

int main(void)
{
	// usbd_device *usbd_dev;
    struct rcc_clock_scale pllconfig = rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_170MHZ];
	rcc_clock_setup_pll(&pllconfig);
    systick_setup(170000000); // 1kHz

    usbserial_init();

    /* Enable GPIOC clock */
    rcc_periph_clock_enable(RCC_GPIOC);

    /* PC13 as push-pull output */
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT,
                    GPIO_PUPD_NONE, GPIO6);

    /* Optional: slower edge, less EMI */
    gpio_set_output_options(GPIOC,
                            GPIO_OTYPE_PP,
                            GPIO_OSPEED_2MHZ,
                            GPIO6);

	while (1) {
        static uint32_t lasttick = 0;

        if((clock_ticks%1000) > 500){
            gpio_set(GPIOC, GPIO6);
        } else {
            gpio_clear(GPIOC, GPIO6);
        }

        if((clock_ticks%1000)==0)
        {
            if(clock_ticks != lasttick){
                lasttick = clock_ticks;
                char teststr[64];
                uint8_t s = usbserial_read_rx(teststr,64);
                usbserial_send_tx(teststr,s);
            }
        }
	}
}
