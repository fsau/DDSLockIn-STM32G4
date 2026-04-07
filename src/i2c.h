#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Initialize I2C peripheral */
void i2c_init(void);

/* Write bytes to a device */
bool i2c_write(uint8_t addr, const uint8_t *data, uint16_t len);

/* Read bytes from a device */
bool i2c_read(uint8_t addr, uint8_t *data, uint16_t len);

/* Write register then read */
bool i2c_write_read(uint8_t addr,
                           uint8_t reg,
                           uint8_t *data,
                           uint16_t len);