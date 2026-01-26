#ifndef AD9833_H_FILE
#define AD9833_H_FILE

#define DDS_MCLK   25000000UL

void ad9833_init(void);
void ad9833_set_frequency(float freq);
void ad9833_set_freq_word(uint32_t ftw);

#endif