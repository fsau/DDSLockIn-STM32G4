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

void i2c_transfer(uint32_t i2c, uint8_t addr, const uint8_t *w, size_t wn, uint8_t *r, size_t rn)
{
	/*  waiting for busy is unnecessary. read the RM */
	if (wn) {
		i2c_set_7bit_address(i2c, addr);
		i2c_set_write_transfer_dir(i2c);
		i2c_set_bytes_to_transfer(i2c, wn);
		if (rn) {
			i2c_disable_autoend(i2c);
		} else {
			i2c_enable_autoend(i2c);
		}
		i2c_send_start(i2c);

		while (wn--) {
			bool wait = true;
			while (wait) {
				if (i2c_transmit_int_status(i2c)) {
					wait = false;
				}
				while (i2c_nack(i2c)); /* FIXME Some error */
			}
			i2c_send_data(i2c, *w++);
		}
		/* not entirely sure this is really necessary.
		 * RM implies it will stall until it can write out the later bits
		 */
		if (rn) {
			while (!i2c_transfer_complete(i2c));
		}
	}

	if (rn) {
		/* Setting transfer properties */
		i2c_set_7bit_address(i2c, addr);
		i2c_set_read_transfer_dir(i2c);
		i2c_set_bytes_to_transfer(i2c, rn);
		/* start transfer */
		i2c_send_start(i2c);
		/* important to do it afterwards to do a proper repeated start! */
		i2c_enable_autoend(i2c);

		for (size_t i = 0; i < rn; i++) {
			while (i2c_received_data(i2c) == 0);
			r[i] = i2c_get_data(i2c);
		}
	}
}

bool i2c_write(uint8_t addr, const uint8_t *data, uint16_t len)
{
    // i2c_wait_busy();

    i2c_transfer(I2C2,
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

    i2c_transfer(I2C2,
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

    i2c_transfer(I2C2,
                  addr,
                  &reg,
                  1,
                  data,
                  len);

    return true;
}