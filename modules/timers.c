#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/cortex.h>
#include "timers.h"

void adc_dac_timer_init(void)
{
    rcc_periph_clock_enable(RCC_TIM4); // Master timer: DAC
    rcc_periph_clock_enable(RCC_TIM3); // Slave timer: ADC

    uint32_t timer_clk = 170000000; // APB2 timer clock (170 MHz)
    uint32_t adc_rate = 1000000;    // 2 MSa/s
    uint32_t prescaler = 0;         // no prescaler
    uint32_t arr = (timer_clk / (adc_rate * (prescaler + 1))) - 1;

    timer_set_prescaler(TIM4, prescaler);
    timer_set_period(TIM4, arr);
    timer_set_master_mode(TIM4, TIM_CR2_MMS_UPDATE);
    timer_enable_preload(TIM4);

    // gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    // gpio_set_output_options(GPIOB,
    //                         GPIO_OTYPE_PP,
    //                         GPIO_OSPEED_2MHZ,
    //                         GPIO9);
    // gpio_set_af(GPIOB, GPIO_AF2, GPIO9);
    // timer_set_oc_mode(TIM4, TIM_OC4, TIM_OCM_PWM1);
    // timer_set_oc_value(TIM4, TIM_OC4, arr / 2); // 50% duty
    // timer_enable_oc_output(TIM4, TIM_OC4); // Debug PWM output
    TIM_DIER(TIM4) |= TIM_DIER_CC4DE; // Enable DMA output for DAC

    timer_set_prescaler(TIM3, prescaler);
    timer_set_period(TIM3, arr);
    timer_set_master_mode(TIM3, TIM_CR2_MMS_UPDATE);
    timer_slave_set_mode(TIM3, TIM_SMCR_SMS_TM); // Starts after TIM4 update
    timer_slave_set_trigger(TIM3, TIM_SMCR_TS_ITR3);
    timer_enable_preload(TIM3);

    // gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0);
    // gpio_set_output_options(GPIOB,
    //                         GPIO_OTYPE_PP,
    //                         GPIO_OSPEED_2MHZ,
    //                         GPIO0);
    // gpio_set_af(GPIOB, GPIO_AF2, GPIO0);
    // timer_set_oc_mode(TIM3, TIM_OC3, TIM_OCM_PWM1);
    timer_set_oc_mode(TIM3, TIM_OC4, TIM_OCM_PWM1);
    // timer_set_oc_value(TIM3, TIM_OC3, arr / 2); // Debug PWM duty
    timer_set_oc_value(TIM3, TIM_OC4, arr / 2); // Tune ADC timing here
    // timer_enable_oc_output(TIM3, TIM_OC3); // Debug PWM output
    timer_enable_oc_output(TIM3, TIM_OC4); // ADC
}

void adc_dac_timer_start(void)
{
    timer_enable_counter(TIM4);
    timer_enable_counter(TIM3);
}

void adc_dac_timer_stop(void)
{
    cm_disable_interrupts();
    // Wait for timer to roll
    uint32_t t = TIM_CNT(TIM4);
    while (TIM_CNT(TIM4) >= t)
        t = TIM_CNT(TIM4);
    timer_disable_counter(TIM3);
    timer_disable_counter(TIM4);
    TIM_CNT(TIM3) = 0;
    TIM_CNT(TIM4) = 0;
    cm_enable_interrupts();
}

void adc_dac_timer_adjust(uint32_t rate, uint8_t prescaler)
{
    uint32_t arr = (170000000UL / (rate * (prescaler + 1))) - 1;

    cm_disable_interrupts();
    // Wait for timer to roll
    uint32_t t = TIM_CNT(TIM4);
    while (TIM_CNT(TIM4) >= t)
        t = TIM_CNT(TIM4);
    timer_set_prescaler(TIM4, prescaler);
    timer_set_period(TIM4, arr);
    timer_set_prescaler(TIM3, prescaler);
    timer_set_period(TIM3, arr);
    cm_enable_interrupts();
}