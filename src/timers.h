#pragma once

#include <stdlib.h>

/*
 * Configure timers for DAC (TIM4) and ADC (TIM3)
 *
 * TIM4 provides the DAC timing reference and triggers DAC conversion on
 * the update event.
 *
 * To compensate for the DAC internal one-cycle pipeline latency, the DAC
 * DMA request is sourced from a TIM4 compare event rather than the DAC’s
 * default DMA request source, so DAC DMA always runs one half-cycle ahead
 * of the DAC conversion.
 *
 * TIM3 is configured as a slave only for startup synchronization:
 *   - TIM3 starts on the TIM4 update trigger to guarantee a deterministic
 *     ADC/DAC phase relationship at t = 0.
 *   - After startup, TIM3 runs freely and independently.
 *
 * Both timers are programmed with identical prescaler and auto-reload
 * values. These parameters must always match to preserve the DAC/ADC
 * phase relationship.
 *
 * Optional PWM outputs may be enabled for external timing observation
 * and debugging.
 *
 * Control functions:
 *
 *  - adc_dac_timer_init()
 *        Configure TIM3 and TIM4 registers.
 *        Timers remain stopped and trigger outputs disabled.
 *
 *  - adc_dac_timer_start()
 *        Start timers, enabling DAC/ADC triggering.
 *
 *  - adc_dac_timer_stop()
 *        Stop both timers safely.
 *
 *  - adc_dac_timer_restart()
 *        Restart timers after adc_dac_timer_stop().
 *
 *  - adc_dac_timer_adjust(rate, prescaler)
 *        Adjust timer rate while running.
 *        Updates are applied in a phase-coherent manner so the DAC/ADC
 *        relationship is preserved.
 */

void adc_dac_timer_init(void);
void adc_dac_timer_start(void);
void adc_dac_timer_restart(void);
void adc_dac_timer_stop(void);
void adc_dac_timer_adjust(uint32_t rate, uint8_t prescaler);