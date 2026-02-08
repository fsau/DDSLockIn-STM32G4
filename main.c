#include <stdio.h>
#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/scb.h>

#include "dma_memcpy.h"
#include "usbserial.h"
#include "adc.h"
#include "spi.h"
#include "cordic.h"
#include "dac.h"
#include "ddsli.h"

volatile uint32_t angles[100];
volatile uint32_t results[100] = {0}; // Each angle gives cos and sin (2 results)
 
volatile uint32_t clock_ticks = 0;
uint32_t lasttick = 0;

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

/*
 * Configure timers for DAC (TIM4) and ADC (TIM3)
 *
 * TIM4 provides the DAC timing reference and triggers DAC conversion on the update
 * event. To compensate for the DAC internal one-cycle pipeline latency, the DAC
 * DMA request is sourced from a TIM4 compare event rather than the DAC’s default
 * DMA request source.
 *
 * TIM3 is configured as a slave only for startup synchronization: it starts on
 * the TIM4 update trigger to guarantee a deterministic phase relationship at
 * t = 0. After startup, TIM3 runs freely and independently.
 *
 * Both timers are programmed with identical prescaler and auto-reload values.
 * These parameters must always match and must not be modified while running,
 * as doing so breaks the DAC/ADC phase relationship.
 *
 * Optional PWM outputs are enabled for external timing observation and debugging.
 */
void load_adc_dac_timer(void)
{
    rcc_periph_clock_enable(RCC_TIM4);  // Master timer: DAC
    rcc_periph_clock_enable(RCC_TIM3);  // Slave timer: ADC
    
    uint32_t timer_clk = 170000000;     // APB2 timer clock (170 MHz)
    uint32_t adc_rate = 1000000;        // 2 MSa/s
    uint32_t prescaler = 0;             // no prescaler
    uint32_t arr = (timer_clk / (adc_rate * (prescaler + 1))) - 1;
    
    timer_set_prescaler(TIM4, prescaler);
    timer_set_period(TIM4, arr);
    timer_set_master_mode(TIM4, TIM_CR2_MMS_UPDATE);
    
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_set_output_options(GPIOB,
                            GPIO_OTYPE_PP,
                            GPIO_OSPEED_2MHZ,
                            GPIO9);
    gpio_set_af(GPIOB, GPIO_AF2, GPIO9);
    timer_set_oc_mode(TIM4, TIM_OC4, TIM_OCM_PWM1);
    timer_set_oc_value(TIM4, TIM_OC4, arr / 2); // 50% duty
    timer_enable_oc_output(TIM4, TIM_OC4); // Debug PWM output
    TIM_DIER(TIM4) |= TIM_DIER_CC4DE; // Enable DMA output for DAC

    timer_set_prescaler(TIM3, prescaler);
    timer_set_period(TIM3, arr);
    timer_set_master_mode(TIM3, TIM_CR2_MMS_UPDATE);
    timer_slave_set_mode(TIM3,TIM_SMCR_SMS_TM); // Starts after TIM4 update
    timer_slave_set_trigger(TIM3,TIM_SMCR_TS_ITR3);

    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0);
    gpio_set_output_options(GPIOB,
                            GPIO_OTYPE_PP,
                            GPIO_OSPEED_2MHZ,
                            GPIO0);
    gpio_set_af(GPIOB, GPIO_AF2, GPIO0);
    timer_set_oc_mode(TIM3, TIM_OC4, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM1);
    timer_set_oc_value(TIM3, TIM_OC4, arr / 2); // Tune ADC timing here
    timer_set_oc_value(TIM3, TIM_OC3, arr / 2); // Debug PWM duty
    timer_enable_oc_output(TIM3, TIM_OC3); // Debug PWM output
    timer_enable_oc_output(TIM3, TIM_OC4); // ADC
}

void start_adc_dac_timer(void)
{
    timer_enable_counter(TIM4);
}

void stop_adc_dac_timer(void)
{
    timer_disable_counter(TIM4);
}

void jump_to_dfu(void)
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
    __asm__ volatile ("msr msp, %0" : : "r" (*(volatile uint32_t *)SCB_VTOR));
    
    // Jump to bootloader
    void (*bootloader)(void) = (void (*)(void))jump_address;
    bootloader();
    
    // Never returns
    while(1);
}

uint8_t print_int_str(uint8_t *buf, uint16_t len, int64_t value)
{
    uint8_t pos = len-1, sign = 0;

    if(value < 0) {
        sign = 1;
        value = -value;
    }

    if(value == 0) {
        buf[0] = '0';
        return 1;
    }

    while(value && pos)
    {
        buf[pos--] = '0' + (value % 10);
        value /= 10;
    }

    if(sign && pos) {
        buf[pos--] = '-';
    }

    for(uint8_t i = 0; i < len - 1 - pos; i++)
    {
        buf[i] = buf[pos + 1 + i];
    }

    return len - 1 - pos;
}

int main(void)
{
    SCB_VTOR = 0x08000000;

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

    dma_memcpy_init();
    usbserial_init();
    spi_setup();
    load_adc_dac_timer();

#ifdef LEGACY_MODE
    adc_dual_dma_init();
    cordic_init();
    dac_init();
   for(uint32_t i = 0; i < 33; i++)
    {
        angles[i] = ((uint32_t)0x6A00<<16) + (i * 0xFFFF) / 33; // Full scale angles
    }
    cordic_start_dma(angles, results, 33);

    for(uint32_t i = 0; i < 1000000; i+=1) __asm__("nop");

    dac_start(results, 33);
#else
    ddsli_setup();
#endif

    for(uint32_t i = 0; i < 1000000; i+=1) __asm__("nop");

    start_adc_dac_timer();

    // Command parsing states
    enum {
        CMD_IDLE,
        CMD_FREQ,
        CMD_DPOT
    } cmd_state = CMD_IDLE;
    
    uint32_t cmd_value = 0;
    uint8_t cmd_digits = 0;

    while(clock_ticks<300) __asm__("nop");

	while (1)
    {
        uint8_t buf[64];
        uint8_t len = usbserial_read_rx(buf, 64);
        usbserial_send_tx(buf,len);

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
                        // TODO: new capture ADC buffer
                    }
                    else if(buf[i] == 'D' || buf[i] == 'd') {
                        // Jump to DFU bootloader
                        // jump_to_dfu();
                    }
                    else if(buf[i] == 'S' || buf[i] == 's') {
                        // stop_adc_dac_timer();
                    }
                    break;
                case CMD_FREQ:
                    if(buf[i] >= '0' && buf[i] <= '9' && cmd_digits < 10) {
                        // Accumulate digits
                        cmd_value = cmd_value * 10 + (buf[i] - '0');
                        cmd_digits++;
                    } else {
                        // Non-digit or max digits reached - execute command
                        // freqw = cmd_value;
                        // ad9833_set_freq_word(freqw);
                        
                        // Reset to idle state
                        cmd_state = CMD_IDLE;
                        cmd_value = 0;
                        cmd_digits = 0;
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
                        // dpot_set((uint8_t)cmd_value);
                        
                        // Reset to idle state
                        cmd_state = CMD_IDLE;
                        cmd_value = 0;
                        cmd_digits = 0;
                    }
                    break;
            }
        }

        if((clock_ticks/2) != lasttick)
        {
            lasttick = clock_ticks/2;
            // ad9833_set_freq_word(freqw);

            static float acc_ch0_cos = 0;
            static float acc_ch0_sin = 0;
            static float acc_ch1_cos = 0;
            static float acc_ch1_sin = 0;
            static int32_t n = 0;
            ddsli_output_t output;
            while(ddsli_output_pop(&output))
            {
                acc_ch0_cos += output.chA[0];
                acc_ch0_sin += output.chA[1];
                acc_ch1_cos += output.chB[0];
                acc_ch1_sin += output.chB[1];
                n++;

                if(n >= 500) {
                    // gpio_toggle(GPIOC, GPIO6);
                    acc_ch0_cos /= n;
                    acc_ch0_sin /= n;
                    acc_ch1_cos /= n;
                    acc_ch1_sin /= n;

                    uint8_t outbuf[100];
                    uint8_t s = snprintf(outbuf, 100,
                        "CH0 COS: %f SIN: %f | CH1 COS: %f SIN: %f\r\n",
                        (float)acc_ch0_cos,
                        (float)acc_ch0_sin,
                        (float)acc_ch1_cos,
                        (float)acc_ch1_sin
                    );
                        
                    usbserial_send_tx(outbuf, s);
                    // uint8_t testb[] = "test ";
                    // usbserial_send_tx(testb,sizeof(testb));

                    acc_ch0_cos = 0;
                    acc_ch0_sin = 0;
                    acc_ch1_cos = 0;
                    acc_ch1_sin = 0;
                    n = 0;
                }
            }
        }
	}
}