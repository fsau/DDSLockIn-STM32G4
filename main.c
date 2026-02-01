#include <stdio.h>
#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

#include "usbserial.h"
#include "adc.h"
#include "spi.h"
#include "ad9833.h"

volatile uint32_t clock_ticks = 0;
static uint32_t lasttick = 0;

void sys_tick_handler(void)
{
    clock_ticks++; // ms

    // if((clock_ticks/1000) != lasttick){
    //     gpio_toggle(GPIOC, GPIO6);
    //     lasttick = clock_ticks/1000;
    // }
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

void bp_here(void) {__asm__ volatile ("bkpt #0");;}

uint16_t ch0[ADC_BUF_LEN], ch1[ADC_BUF_LEN];

uint8_t dpot_pos = 0;

void dpot_init(void)
{
    // Initialize UD (direction) pin
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO10);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO10);
    
    // Initialize INC (increment) pin  
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO11);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO11);

    // Set direction to increase
    gpio_set(GPIOC, GPIO10);
    for(int j = 0; j < 1000; j++) __asm__("nop");
    
    // Move to maximum position (100)
    for(uint8_t i = 0; i < 100; i++)
    {
        gpio_clear(GPIOC, GPIO11);
        for(int j = 0; j < 1000; j++) __asm__("nop");
        gpio_set(GPIOC, GPIO11);
        for(int j = 0; j < 1000; j++) __asm__("nop");
    }
    
    dpot_pos = 100;
    gpio_clear(GPIOC, GPIO10); // Set direction back to default (decrease)
}

void dpot_set(uint8_t new_pos)
{
    if(new_pos > dpot_pos) {
        // Need to increase
        gpio_set(GPIOC, GPIO10); // Set direction to increase
        for(int j = 0; j < 10000; j++) __asm__("nop");
        
        uint8_t steps = new_pos - dpot_pos;
        for(uint8_t i = 0; i < steps; i++) {
            gpio_clear(GPIOC, GPIO11);
            for(int j = 0; j < 10000; j++) __asm__("nop");
            gpio_set(GPIOC, GPIO11);
            for(int j = 0; j < 10000; j++) __asm__("nop");
        }
        
        gpio_clear(GPIOC, GPIO10); // Reset direction
    } 
    else if(new_pos < dpot_pos) {
        // Need to decrease
        gpio_clear(GPIOC, GPIO10); // Set direction to decrease
        for(int j = 0; j < 10000; j++) __asm__("nop");
        
        uint8_t steps = dpot_pos - new_pos;
        for(uint8_t i = 0; i < steps; i++) {
            gpio_clear(GPIOC, GPIO11);
            for(int j = 0; j < 10000; j++) __asm__("nop");
            gpio_set(GPIOC, GPIO11);
            for(int j = 0; j < 10000; j++) __asm__("nop");
        }
    }
    // If equal, do nothing
    
    dpot_pos = new_pos;
}

int main(void)
{
    struct rcc_clock_scale pllconfig = rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_170MHZ];
	rcc_clock_setup_pll(&pllconfig);
    systick_setup(170000000); // 1kHz
    rcc_periph_clock_enable(RCC_GPIOC);

    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT,
                    GPIO_PUPD_NONE, GPIO6);
    gpio_set_output_options(GPIOC,
                            GPIO_OTYPE_PP,
                            GPIO_OSPEED_2MHZ,
                            GPIO6);
    gpio_set(GPIOC, GPIO6);

    usbserial_init();
    adc_dual_dma_init();
    adc_timer_trigger_init();
    spi_setup();
    ad9833_init();
    dpot_init();

	while (1) {
        uint8_t buf[64];
        uint8_t len = usbserial_read_rx(buf,64);
        // usbserial_send_tx(buf,len);

        for(uint32_t i = 0; i < len; i++)
        {
            static uint8_t freq_status = 0;
            static uint32_t freqw_buf = 0;

            if(freq_status != 0)
            {
                bool endflag = 0;
                if((buf[i] >= '0') && (buf[i] <= '9'))
                {
                    freq_status++;
                    uint8_t digit = buf[i] - '0';
                    freqw_buf *= 10;
                    freqw_buf += digit;
                }
                else
                {
                    endflag=1;
                }

                if((freq_status > 10) || endflag)
                {
                    freqw = freqw_buf;
                    freq_status = 0;
                    freqw_buf = 0;
                    ad9833_set_freq_word(freqw);
                    // uint8_t buf[32];
                    // uint8_t s = snprintf(buf,32,"f=%d\r\n",freqw);
                    // usbserial_send_tx(buf,s);
                }
            }
            else if (buf[i] == 'M' || buf[i] == 'm')
            {
                adc_capture_buffer(ch0,ch1);
                usbserial_send_tx((uint8_t*)ch0,sizeof(ch0));
                usbserial_send_tx((uint8_t*)ch1,sizeof(ch1));
            }
            else if (buf[i] == 'F' || buf[i] == 'f')
            {
                freq_status = 1;
                freqw_buf = 0;
            }
            else if (buf[i] == 'A' || buf[i] == 'a')
            {
                static uint8_t pos = 0;
                pos = (pos+1)%100;
                dpot_set(pos);
            }
        }

        if((clock_ticks/100) != lasttick)
        {
            lasttick = clock_ticks/100;
            gpio_toggle(GPIOC, GPIO6);
            ad9833_set_freq_word(freqw);
        }
	}
}
