#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>

#include "spi.h"

void spi_setup(void)
{
    /* Clocks */
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_SPI2);

    /* PB13=SCK, PB14=MISO, PB15=MOSI */
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE,
                    GPIO13 | GPIO14 | GPIO15);

    /* Output config for SCK + MOSI */
    gpio_set_output_options(GPIOB,
                            GPIO_OTYPE_PP,
                            GPIO_OSPEED_2MHZ,
                            GPIO13 | GPIO15);

    gpio_set_af(GPIOB, GPIO_AF5, GPIO13 | GPIO14 | GPIO15);

    /* Reset SPI */
    rcc_periph_reset_pulse(RST_SPI2);

    /* SPI configuration */
    spi_disable(SPI2);

    spi_set_master_mode(SPI2);

    /* Baudrate = fPCLK / 128 */
    spi_set_baudrate_prescaler(SPI2, SPI_CR1_BR_FPCLK_DIV_256);

    /* SPI mode 3: CPOL=1, CPHA=1 */
    spi_set_clock_polarity_1(SPI2);
    spi_set_clock_phase_0(SPI2);

    /* 8-bit data */
    spi_set_data_size(SPI2, SPI_CR2_DS_8BIT);

    /* MSB first */
    spi_send_msb_first(SPI2);

    /* Software NSS */
    spi_enable_software_slave_management(SPI2);
    spi_set_nss_high(SPI2);
    spi_disable_crc(SPI2);

    /* Enable SPI */
    spi_enable(SPI2);
}

void spi_tx8(uint8_t data)
{
    spi_send8(SPI2, data);
}
