#ifndef SPI_H_FILE
#define SPI_H_FILE

#define FSYNC_PORT GPIOA
#define FSYNC_PIN  GPIO4

void spi_setup(void);
void spi_tx8(uint8_t data);
// void dds_write16(uint16_t word);

#endif