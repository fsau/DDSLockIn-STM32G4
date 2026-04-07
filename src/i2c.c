#include "i2c.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#define I2C_GPIO_PORT GPIOA
#define I2C_SCL_PIN GPIO8
#define I2C_SDA_PIN GPIO9

static void i2c_wait_busy(void)
{
    while (i2c_busy(I2C2));
}

void i2c_init(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_I2C2);

    gpio_mode_setup(I2C_GPIO_PORT,
                    GPIO_MODE_AF,
                    GPIO_PUPD_NONE,
                    I2C_SCL_PIN | I2C_SDA_PIN);

    gpio_set_output_options(I2C_GPIO_PORT,
                            GPIO_OTYPE_OD,
                            GPIO_OSPEED_50MHZ,
                            I2C_SCL_PIN | I2C_SDA_PIN);

    gpio_set_af(I2C_GPIO_PORT,
                GPIO_AF4,
                I2C_SCL_PIN | I2C_SDA_PIN);

    i2c_peripheral_disable(I2C2);

    i2c_set_prescaler(I2C2, 255);
    i2c_set_scl_low_period(I2C2, 40);
    i2c_set_scl_high_period(I2C2, 40); 
    i2c_set_data_hold_time(I2C2, 40);
    i2c_set_data_setup_time(I2C2, 40);

    i2c_peripheral_enable(I2C2);
}

bool i2c_write(uint8_t addr, const uint8_t *data, uint16_t len)
{
    // i2c_wait_busy();

    i2c_transfer7(I2C2,
                  addr,
                  (uint8_t *)data,
                  len,
                  NULL,
                  0);

    return true;
}

bool i2c_read(uint8_t addr, uint8_t *data, uint16_t len)
{
    // i2c_wait_busy();

    i2c_transfer7(I2C2,
                  addr,
                  NULL,
                  0,
                  data,
                  len);

    return true;
}

bool i2c_write_read(uint8_t addr,
                           uint8_t reg,
                           uint8_t *data,
                           uint16_t len)
{
    // i2c_wait_busy();

    i2c_transfer7(I2C2,
                  addr,
                  &reg,
                  1,
                  data,
                  len);

    return true;
}