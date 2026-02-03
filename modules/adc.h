#ifndef ADC_H
#define ADC_H

#include <stdint.h>

#define ADC_BUF_LEN 1024

extern volatile uint32_t adc_buffer[ADC_BUF_LEN];
extern volatile uint8_t adc_capture_complete;

/* Initialize ADC1+ADC2 in dual simultaneous mode, default buffer */
void adc_dual_dma_init(void);

/* Initialize ADC1+ADC2 in dual simultaneous mode, circular buff */
void adc_dualcirc_dma_init(void *buf, uint32_t len);

/* Capture one buffer of ADC1 and ADC2 data (blocking) */
void adc_capture_buffer(uint16_t *adc1_data, uint16_t *adc2_data);

/* Start continuous mode (free-running) */
void adc_start_continuous_mode(void);

/* Stop continuous mode */
void adc_stop_continuous_mode(void);

#endif /* ADC_H */