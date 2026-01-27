#ifndef ADC_H
#define ADC_H

#include <stdint.h>

#define ADC_BUF_LEN 1024

extern volatile uint32_t adc_buffer[ADC_BUF_LEN];
extern volatile uint8_t adc_capture_complete;

/* Initialize ADC1+ADC2 in dual simultaneous mode with DMA for 170MHz clock */
void adc_dual_dma_init(void);

/* Capture one buffer of ADC1 and ADC2 data (blocking) */
void adc_capture_buffer(uint16_t *adc1_data, uint16_t *adc2_data);

/* Start continuous mode (free-running) */
void adc_start_continuous_mode(void);

/* Stop continuous mode */
void adc_stop_continuous_mode(void);

void adc_capture_buffer_no_dma(uint16_t *adc1_data, uint16_t *adc2_data, uint32_t num_samples);

void timer_adc_trigger_init(void);

#endif /* ADC_H */