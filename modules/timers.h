#pragma once

#include <stdlib.h>

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

void adc_dac_timer_load(void);
void adc_dac_timer_start(void);
void adc_dac_timer_stop(void);
void adc_dac_timer_adjust(uint32_t rate, uint8_t prescaler);