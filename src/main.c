#include <stdlib.h>
#include <math.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
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
#include "utils.h"

typedef enum {
    CMD_ACT_NONE = 0,
    CMD_ACT_SET_FREQ,
    CMD_ACT_SET_FREQ_DUAL_SWEEP,
    CMD_ACT_CTRL_STOP,
    CMD_ACT_CTRL_RESTART,
    CMD_ACT_CAPTURE_ONCE,
    CMD_ACT_PRINT_DEMOD,
    CMD_ACT_PRINT_PACKETS,
    CMD_ACT_ADJUST_AUTOCAP,
    CMD_ACT_TOGGLE_AUTOCAP,
    CMD_ACT_TOGGLE_AUTOOUT,
    CMD_ACT_ADJUST_FA,
    CMD_ACT_ADJUST_FB,
    CMD_ACT_ADJUST_SWA,
    CMD_ACT_ADJUST_SWB,
    CMD_ACT_TOGGLE_ECHO
} cmd_action_t;

typedef enum {
    CMD_IDLE,
    CMD_INPUT_FREQ,
    CMD_CTRL,
    CMD_CAPT
} cmd_state_t;

typedef struct {
    cmd_state_t state;
    uint64_t    value;
    bool        neg;
    uint8_t     digits;
} cmd_parser_t;

cmd_action_t cmd_parse_byte(cmd_parser_t *p, uint8_t c, uint64_t *arg);

void print_output_packet(void);
void print_capture_buff(void);
void print_legible_demod(void);

int main(void)
{
    SCB_VTOR = 0x08000000; // fix for reset after DFU, still missing something

    struct rcc_clock_scale pllconfig = rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_170MHZ];
    rcc_clock_setup_pll(&pllconfig);
    systick_setup(170000000);

    // LED output:
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT,
                    GPIO_PUPD_NONE, GPIO6);
    gpio_set_output_options(GPIOC,
                            GPIO_OTYPE_PP,
                            GPIO_OSPEED_2MHZ,
                            GPIO6);
    gpio_set(GPIOC, GPIO6);

    usbserial_init(); // Communications
    ddsli_setup(); // Everything else

    for (uint32_t i = 0; i < 1000000; i += 1)
        __asm__("nop");

    cm_enable_interrupts();
    adc_dac_timer_start(); // Start DDS+ADC

    bool echo_serial = false;
    uint32_t auto_capture_dly = 1;
    int32_t auto_output = 0;
    int32_t auto_capture = 0;
    float fa=33768.0, fb=33768.1, swa=1.0, swb=0.0;
    float sample_f = 1000000.0;

    ddsli_set_frequency_dual(fa, fb, swa, swb, sample_f);

    while (clock_ticks < 300)
        __asm__("nop");

    cmd_parser_t parser = { .state = CMD_IDLE };

    while (1)
    {
        uint8_t buf[64] = {0};
        uint8_t len = usbserial_read_rx(buf, 64);

        if(echo_serial && len) usbserial_send_tx(buf,len); 

        for (uint32_t i = 0; i < len; i++)
        {
            uint64_t arg = 0;
            cmd_action_t act = cmd_parse_byte(&parser, buf[i], &arg);

            switch (act)
            {
            case CMD_ACT_SET_FREQ:
                ddsli_set_frequency_dual((float)arg / 10.73741824f,
                                         (float)arg / 10.73741824f,
                                    0.0f, 0.0f, 1000000);
                break;

            case CMD_ACT_SET_FREQ_DUAL_SWEEP:
                ddsli_set_frequency_dual(fa, fb, swa, swb, sample_f);
                break;

            case CMD_ACT_ADJUST_FA:
                fa = (float)arg/1000.0;
                break;

            case CMD_ACT_ADJUST_FB:
                fb = (float)arg/1000.0;
                break;

            case CMD_ACT_ADJUST_SWA:
                swa = (float)arg;
                break;

            case CMD_ACT_ADJUST_SWB:
                swb = (float)arg;
                break;

            case CMD_ACT_ADJUST_AUTOCAP:
                auto_capture_dly = arg;
                break;

            case CMD_ACT_CTRL_STOP:
                adc_dac_timer_stop();
                break;

            case CMD_ACT_CTRL_RESTART:
                adc_dac_timer_restart();
                break;

            case CMD_ACT_CAPTURE_ONCE:
                if(ddsli_capture_ready()) ddsli_capture_buffers(4);
                auto_capture++;
                break;

            case CMD_ACT_PRINT_DEMOD:
                print_legible_demod();
                break;

            case CMD_ACT_PRINT_PACKETS:
                auto_output++;
                break;

            case CMD_ACT_TOGGLE_AUTOCAP:
                if(ddsli_capture_ready()) ddsli_capture_buffers(4);
                auto_capture = (auto_capture==-1)?0:-1;
                break;

            case CMD_ACT_TOGGLE_AUTOOUT:
                auto_output = (auto_output==-1)?0:-1;
                break;

            case CMD_ACT_TOGGLE_ECHO:
                echo_serial = !echo_serial;
                break;
            
            default:
                break;
            }
        }

        if(auto_capture!=0)
        {
            static uint32_t last_clock = 0;
            if( (clock_ticks/auto_capture_dly != last_clock) &&
                ddsli_capture_ready() )
            {
                last_clock = clock_ticks/auto_capture_dly;
                print_capture_buff();
                ddsli_capture_buffers(4);
                if(auto_capture > 0) auto_capture--;
            }
        }

        if(auto_output!=0)
        {
            if(ddsli_output_count()>=16)
            {
                print_output_packet();
                if(auto_output > 0) auto_output--;
            }
        }
    }
}

cmd_action_t cmd_parse_byte(cmd_parser_t *p, uint8_t c, uint64_t *arg)
{
    // Digit accumulation
    if (c >= '0' && c <= '9' && p->digits < 18)
    {
        p->value = p->value * 10 + (c - '0');
        p->digits++;
        return CMD_ACT_NONE;
    }
    else if(c == '-')
    {
        p->neg = !p->neg;
        return CMD_ACT_NONE;
    }

    switch (p->state)
    {
    case CMD_IDLE:
        if (c == 'F' || c == 'f') {
            p->state  = CMD_INPUT_FREQ;
            p->value  = 0;
            p->digits = 0;
            p->neg = 0;
        }
        else if (c == 'A' || c == 'a') {
            p->state  = CMD_CAPT;
            p->value  = 0;
            p->digits = 0;
            p->neg = 0;
        }
        else if (c == 'C' || c == 'c') {
            p->state = CMD_CTRL;
        }
        else if (c == 'M' || c == 'm') {
            return CMD_ACT_CAPTURE_ONCE;
        }
        else if (c == 'D' || c == 'd') {
            return CMD_ACT_PRINT_DEMOD;
        }
        else if (c == 'P' || c == 'p') {
            return CMD_ACT_PRINT_PACKETS;
        }
        else if (c == 'U' || c == 'u') {
            return CMD_ACT_SET_FREQ_DUAL_SWEEP;
        }
        else if (c == 'E' || c == 'e') {
            return CMD_ACT_TOGGLE_ECHO;
        }
        break;

    case CMD_INPUT_FREQ:
        *arg = p->value;
        p->state = CMD_IDLE;
        p->value = 0;
        p->digits = 0;
        p->neg = 0;
        if (c == 'A' || c == 'a')
            return CMD_ACT_ADJUST_FA;
        if (c == 'B' || c == 'b')
            return CMD_ACT_ADJUST_FB;
        if (c == 'Q' || c == 'q')
            return CMD_ACT_ADJUST_SWA;
        if (c == 'G' || c == 'g')
            return CMD_ACT_ADJUST_SWB;
        return CMD_ACT_SET_FREQ;

    case CMD_CTRL:
        p->state = CMD_IDLE;
        if (c == 'S' || c == 's')
            return CMD_ACT_CTRL_STOP;
        if (c == 'R' || c == 'r')
            return CMD_ACT_CTRL_RESTART;
        if (c == 'M' || c == 'm')
            return CMD_ACT_TOGGLE_AUTOCAP;
        if (c == 'P' || c == 'p')
            return CMD_ACT_TOGGLE_AUTOOUT;
        break;

    case CMD_CAPT:
        *arg = p->value;
        p->state = CMD_IDLE;
        p->value = 0;
        p->digits = 0;
        p->neg = 0;
        return CMD_ACT_ADJUST_AUTOCAP;
    }

    return CMD_ACT_NONE;
}

void print_capture_buff(void)
{
    while (!ddsli_capture_ready()) __asm__("nop");
    uint8_t header[4] = {0x55,0x55,0x55,0x00};
    usbserial_send_tx(header, 4);
    usbserial_send_tx((uint8_t *)ddsli_get_capt_adc(), 4 * 4 * HB_LEN);
}

void print_output_packet(void)
{
    uint32_t dcnt = 0;
    ddsli_output_t obuf;
    // uint8_t header[4] = {0x55,0x55,0x55,0x20};
    // usbserial_send_tx(header, 4);

    while (dcnt < 16)
    {
        while (ddsli_output_pop(&obuf))
        {
            uint8_t header[4] = {0x55,0x55,0x55,0x20};
            usbserial_send_tx(header, 4);
            usbserial_send_tx((uint8_t *)&obuf, sizeof(obuf));
            dcnt++;
            if (dcnt >= 64) break;
        }
    }
}

void print_legible_demod(void)
{
    float acc_ch0_cos = 0;
    float acc_ch0_sin = 0;
    float acc_ch1_cos = 0;
    float acc_ch1_sin = 0;
    uint64_t acc_f = 0;
    int32_t n = 0;
    while (n <= 50)
    {
        ddsli_output_t output;
        if (ddsli_output_pop(&output))
        {
            acc_ch0_cos += output.chA[0];
            acc_ch0_sin += output.chA[1];
            acc_ch1_cos += output.chB[0];
            acc_ch1_sin += output.chB[1];
            acc_f += output.frequency.phase_inc >> 30;
            n++;
        }
    }

    acc_ch0_cos /= n;
    acc_ch0_sin /= n;
    acc_ch1_cos /= n;
    acc_ch1_sin /= n;
    acc_f /= 4 * n;

    {
        uint8_t outbuf[128];
        char *p = (char *)outbuf;

        float f = (float)acc_f / 4294.96f;
        float amp0 = sqrtf(acc_ch0_cos * acc_ch0_cos +
                        acc_ch0_sin * acc_ch0_sin);
        float amp1 = sqrtf(acc_ch1_cos * acc_ch1_cos +
                        acc_ch1_sin * acc_ch1_sin);
        float phi0 = atan2f(acc_ch0_cos, acc_ch0_sin);
        float phi1 = atan2f(acc_ch1_cos, acc_ch1_sin);
        float amp_ratio = (amp0 != 0.0f) ? (amp1 / amp0) : 0.0f;
        float dphi = phi0 - phi1;

        if (dphi > M_PI)  dphi -= 2.0f * M_PI;
        if (dphi < -M_PI) dphi += 2.0f * M_PI;

        *p++ = 'f'; *p++ = ' ';
        p = fmt_f(p, f, 0, 3);
        *p++ = ' '; *p++ = 'H'; *p++ = 'z';

        *p++ = ' '; *p++ = '|'; *p++ = ' ';
        *p++ = 'C'; *p++ = 'H'; *p++ = '0'; *p++ = ' ';
        p = fmt_f(p, amp0 / 3510.571f, 6, 5);
        *p++ = ' '; *p++ = 'V'; *p++ = ' ';
        p = fmt_f(p, phi0 * (180.0f / M_PI), 4, 3);
        *p++ = 0xC2; *p++ = 0xB0;

        *p++ = ' '; *p++ = '|'; *p++ = ' ';
        *p++ = 'C'; *p++ = 'H'; *p++ = '1'; *p++ = ' ';
        p = fmt_f(p, amp1 / 3510.571f, 6, 5);
        *p++ = ' '; *p++ = 'V'; *p++ = ' ';
        p = fmt_f(p, phi1 * (180.0f / M_PI), 4, 3);
        *p++ = 0xC2; *p++ = 0xB0;

        *p++ = ' '; *p++ = '|'; *p++ = ' ';
        *p++ = 'r'; *p++ = 'e'; *p++ = 'l'; *p++ = ' ';
        p = fmt_f(p, amp_ratio, 7, 5);
        *p++ = ' ';
        p = fmt_f(p, dphi * (180.0f / M_PI), 7, 3);
        *p++ = 0xC2; *p++ = 0xB0;

        *p++ = '\r'; *p++ = '\n';

        usbserial_send_tx(outbuf, p - (char *)outbuf);
    }
}
