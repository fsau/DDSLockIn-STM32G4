#include <libopencm3/stm32/gpio.h>
#include <stdint.h>
#include "spi.h"
#include "ad9833.h"

volatile uint32_t freqw = 352187;

void ad9833_write16(uint16_t word)
{
    gpio_clear(FSYNC_PORT, FSYNC_PIN);

    for(uint32_t i = 0; i < 1000; i++);
    spi_tx8(word >> 8);
    for(uint32_t i = 0; i < 100; i++);
    spi_tx8(word & 0xFF);
    for(uint32_t i = 0; i < 1000; i++);

    gpio_set(FSYNC_PORT, FSYNC_PIN);
    for(uint32_t i = 0; i < 1000; i++);
}

void ad9833_init(void)
{
    gpio_mode_setup(FSYNC_PORT, GPIO_MODE_OUTPUT,
                    GPIO_PUPD_NONE, FSYNC_PIN);
    gpio_set(FSYNC_PORT, FSYNC_PIN);

    for(uint32_t i = 0; i < 100000; i++);

    ad9833_write16(0x2100);   // RESET=1, B28=1
}

void ad9833_set_frequency(float freq)
{
    uint32_t ftw =
        (uint64_t)freq * (1UL << 28) / DDS_MCLK;

    uint16_t lsb = 0x4000 | (ftw & 0x3FFF);
    uint16_t msb = 0x4000 | ((ftw >> 14) & 0x3FFF);

    ad9833_write16(0x2000);

    ad9833_write16(lsb);
    ad9833_write16(msb);
}

void ad9833_set_freq_word(uint32_t ftw)
{
    uint16_t lsb = 0x4000 | (ftw & 0x3FFF);
    uint16_t msb = 0x4000 | ((ftw >> 14) & 0x3FFF);

    ad9833_write16(0x2000);

    ad9833_write16(lsb);
    ad9833_write16(msb);
}
