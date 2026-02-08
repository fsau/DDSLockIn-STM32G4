#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/cortex.h>

#include "dma_memcpy.h"
#include "usbserial.h"
#include "adc.h"
#include "cordic.h"
#include "dac.h"
#include "ddsli.h"
#include "timers.h"

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
        static float acc_ch0_cos = 0;
        static float acc_ch0_sin = 0;
        static float acc_ch1_cos = 0;
        static float acc_ch1_sin = 0;
        static float acc_f = 0;
        static int32_t n = 0;
        uint8_t len = usbserial_read_rx(buf, 64);
        // usbserial_send_tx(buf,len);

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
                        ddsli_capture_buffers(4);
                        while(!ddsli_capture_ready()) __asm__("nop");
                        usbserial_send_tx((uint8_t*)ddsli_get_capt_adc(),4*4*HB_LEN);
                        // usbserial_send_tx((uint8_t*)ddsli_get_capt_adc()+2*4*HB_LEN,2*4*HB_LEN);
                    }
                    else if(buf[i] == 'D' || buf[i] == 'd') {
                        if(n >= 50) {
                            acc_ch0_cos /= n;
                            acc_ch0_sin /= n;
                            acc_ch1_cos /= n;
                            acc_ch1_sin /= n;

                            uint8_t outbuf[128];

                            float f = acc_f/(float)(1.844674407371e13);

                            /* Amplitudes */
                            float amp0 = sqrtf(acc_ch0_cos*acc_ch0_cos + acc_ch0_sin*acc_ch0_sin);
                            float amp1 = sqrtf(acc_ch1_cos*acc_ch1_cos + acc_ch1_sin*acc_ch1_sin);

                            /* Phases (radians) */
                            float phi0 = atan2f(acc_ch0_cos, acc_ch0_sin);
                            float phi1 = atan2f(acc_ch1_cos, acc_ch1_sin);

                            /* Relative quantities */
                            float amp_ratio = (amp0 != 0.0f) ? (amp1 / amp0) : 0.0f;
                            float dphi = phi0 - phi1;

                            /* Wrap to [-pi, pi] */
                            if (dphi >  M_PI) dphi -= 2.0f*M_PI;
                            if (dphi < -M_PI) dphi += 2.0f*M_PI;

                            uint8_t s = snprintf(
                                (char *)outbuf, sizeof(outbuf),
                                "f %.3f Hz | CH0 %6.5f V %4.3f° | CH1 %6.5f V %4.3f° | "
                                "rel %7.5f %7.3f°\r\n", f,
                                amp0/3510.571, phi0 * (180.0f/M_PI),
                                amp1/3510.571, phi1 * (180.0f/M_PI),
                                amp_ratio,
                                dphi * (180.0f/M_PI)
                            );
                                
                            usbserial_send_tx(outbuf, s);

                            acc_ch0_cos = 0;
                            acc_ch0_sin = 0;
                            acc_ch1_cos = 0;
                            acc_ch1_sin = 0;
                            acc_f = 0;
                            n = 0;
                        }
                    }
                    else if(buf[i] == 'S' || buf[i] == 's') {
                        stop_adc_dac_timer();
                    }
                    else if(buf[i] == 'R' || buf[i] == 'r') {
                        stop_adc_dac_timer();
                        start_adc_dac_timer();
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
                        dds_set_frequency((float)cmd_value/10.73741824f, 0.0f, 1000000);
                        
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
            ddsli_output_t output;
            while(ddsli_output_pop(&output))
            {
                acc_ch0_cos += output.chA[0];
                acc_ch0_sin += output.chA[1];
                acc_ch1_cos += output.chB[0];
                acc_ch1_sin += output.chB[1];
                acc_f = output.frequency;
                n++;
            }
        }
	}
}