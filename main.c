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
    if(new_pos > 100) new_pos = 100; // Limit to maximum
    
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
    
    // Optional: Send confirmation back
    // uint8_t buf[32];
    // uint8_t s = snprintf(buf, 32, "DPOT set to %d\r\n", dpot_pos);
    // usbserial_send_tx(buf, s);
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

    // Command parsing states
    enum {
        CMD_IDLE,
        CMD_FREQ,
        CMD_DPOT
    } cmd_state = CMD_IDLE;
    
    uint32_t cmd_value = 0;
    uint8_t cmd_digits = 0;

	while (1) {
        uint8_t buf[64];
        uint8_t len = usbserial_read_rx(buf, 64);

        for(uint32_t i = 0; i < len; i++)
        {
            switch(cmd_state) {
                case CMD_IDLE:
                    if(buf[i] == 'F' || buf[i] == 'f') {
                        // Start frequency command
                        cmd_state = CMD_FREQ;
                        cmd_value = 0;
                        cmd_digits = 0;
                    }
                    else if(buf[i] == 'A' || buf[i] == 'a') {
                        // Start dpot command
                        cmd_state = CMD_DPOT;
                        cmd_value = 0;
                        cmd_digits = 0;
                    }
                    else if(buf[i] == 'M' || buf[i] == 'm') {
                        // ADC capture command (immediate)
                        adc_capture_buffer(ch0, ch1);
                        usbserial_send_tx((uint8_t*)ch0, sizeof(ch0));
                        usbserial_send_tx((uint8_t*)ch1, sizeof(ch1));
                    }
                    // Add other immediate commands here if needed
                    break;
                    
                case CMD_FREQ:
                    if(buf[i] >= '0' && buf[i] <= '9' && cmd_digits < 10) {
                        // Accumulate digits
                        cmd_value = cmd_value * 10 + (buf[i] - '0');
                        cmd_digits++;
                    } else {
                        // Non-digit or max digits reached - execute command
                        freqw = cmd_value;
                        ad9833_set_freq_word(freqw);
                        
                        // Reset to idle state
                        cmd_state = CMD_IDLE;
                        cmd_value = 0;
                        cmd_digits = 0;
                        
                        // Optional: send confirmation
                        // uint8_t confirm_buf[32];
                        // uint8_t s = snprintf(confirm_buf, 32, "Freq set to %lu\r\n", freqw);
                        // usbserial_send_tx(confirm_buf, s);
                    }
                    break;
                    
                case CMD_DPOT:
                    if(buf[i] >= '0' && buf[i] <= '9' && cmd_digits < 3) {
                        // Accumulate digits (max 3 digits for 0-100)
                        cmd_value = cmd_value * 10 + (buf[i] - '0');
                        cmd_digits++;
                    } else {
                        // Non-digit or max digits reached - execute command
                        if(cmd_value > 100) cmd_value = 100; // Clamp to max
                        dpot_set((uint8_t)cmd_value);
                        
                        // Reset to idle state
                        cmd_state = CMD_IDLE;
                        cmd_value = 0;
                        cmd_digits = 0;
                    }
                    break;
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