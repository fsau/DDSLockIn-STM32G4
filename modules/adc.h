#ifndef ADC_H_FILE
#define ADC_H_FILE

#define ADC_BUF_LEN 512
#define ADC_BUF_BLEN ADC_BUF_LEN*2
#define BUFF_N 512

void delay_ms(uint32_t ms);
void adc_single_setup(void);
uint16_t adc_read_ch0(void);
void adc_dual_freerun_setup(void);
void adc_sample_dual_freerun(uint16_t *buf1,uint16_t *buf2);

#endif
