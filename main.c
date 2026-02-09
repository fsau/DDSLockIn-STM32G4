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

volatile uint32_t clock_ticks = 0;
uint32_t lasttick = 0;
uint32_t auto_capture_dly = 0;

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
    __asm__ volatile ("msr msp, %0" : : "r" (*(volatile uint32_t *)SCB_VTOR));

    // Jump to bootloader
    void (*bootloader)(void) = (void (*)(void))jump_address;
    bootloader();

    // Never returns
    while(1);
}

int main(void)
{
    SCB_VTOR = 0x08000000;

    struct rcc_clock_scale pllconfig = rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_170MHZ];
	rcc_clock_setup_pll(&pllconfig);
    systick_setup(170000000); // 1kHz
    rcc_periph_clock_enable(RCC_GPIOC);
    cm_enable_interrupts();

    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT,
                    GPIO_PUPD_NONE, GPIO6);
    gpio_set_output_options(GPIOC,
                            GPIO_OTYPE_PP,
                            GPIO_OSPEED_2MHZ,
                            GPIO6);
    gpio_set(GPIOC, GPIO6);

    dma_memcpy_init();
    usbserial_init();
    adc_dac_timer_load();
    ddsli_setup();

    for(uint32_t i = 0; i < 1000000; i+=1) __asm__("nop");

    adc_dac_timer_start();

    // Command parsing states
    enum {
        CMD_IDLE,
        CMD_FREQ,
        CMD_CAPT
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
        static uint64_t acc_f = 0;
        static int32_t n = 0;
        uint8_t len = usbserial_read_rx(buf, 64);
        // usbserial_send_tx(buf,len);

        for(uint32_t i = 0; i < len; i++)
        {
            if(buf[i] >= '0' && buf[i] <= '9' && cmd_digits < 10) {
                cmd_value = cmd_value * 10 + (buf[i] - '0');
                cmd_digits++;
            }
            else
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
                            // Start auto capture
                            cmd_state = CMD_CAPT;
                            cmd_value = 0;
                            cmd_digits = 0;
                        }
                        else if(buf[i] == 'M' || buf[i] == 'm') {
                            ddsli_capture_buffers(4);
                            while(!ddsli_capture_ready()) __asm__("nop");
                            usbserial_send_tx((uint8_t*)ddsli_get_capt_adc(),4*4*HB_LEN);
                        }
                        else if(buf[i] == 'D' || buf[i] == 'd') {
                            ddsli_output_t output;
                            while(ddsli_output_pop(&output))
                            {
                                acc_ch0_cos += output.chA[0];
                                acc_ch0_sin += output.chA[1];
                                acc_ch1_cos += output.chB[0];
                                acc_ch1_sin += output.chB[1];
                                acc_f += output.frequency.phase_inc>>30;
                                n++;
                            }
                            if(n >= 50) {
                                acc_ch0_cos /= n;
                                acc_ch0_sin /= n;
                                acc_ch1_cos /= n;
                                acc_ch1_sin /= n;
                                acc_f /= 4*n;

                                uint8_t outbuf[128];

                                double f = (double)acc_f/(4294.96f);

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
                        else if(buf[i] == 'P' || buf[i] == 'p') {
                            static uint32_t dcnt = 0;
                            ddsli_output_t buf[16];
                            while(ddsli_output_pop(&buf[dcnt]))
                            {
                                dcnt=(dcnt+1)%16;
                                if(dcnt==0)
                                {
                                    usbserial_send_tx((uint8_t*)buf, sizeof(buf));
                                    break;
                                }
                            }
                        }
                        else if(buf[i] == 'S' || buf[i] == 's') {
                            adc_dac_timer_stop();
                        }
                        else if(buf[i] == 'R' || buf[i] == 'r') {
                            adc_dac_timer_start();
                        }
                        break;
                    case CMD_FREQ:
                        ddsli_set_frequency((float)cmd_value/10.73741824f, 0.0f, 1000000);

                        cmd_state = CMD_IDLE;
                        cmd_value = 0;
                        cmd_digits = 0;
                        break;

                    case CMD_CAPT:
                        auto_capture_dly = cmd_value;

                        cmd_state = CMD_IDLE;
                        cmd_value = 0;
                        cmd_digits = 0;
                        break;
                }
            }
        }

        if(auto_capture_dly)
        {
            static uint32_t acd_cnt = 0;
            if((clock_ticks/auto_capture_dly) != acd_cnt)
            {
                acd_cnt = clock_ticks/auto_capture_dly;
                ddsli_capture_buffers(4);
                while(!ddsli_capture_ready()) __asm__("nop");
                usbserial_send_tx((uint8_t*)ddsli_get_capt_adc(),4*4*HB_LEN);
            }
        }

        if((clock_ticks/2) != lasttick)
        {
            lasttick = clock_ticks/2;
        }

	}
}