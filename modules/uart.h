#pragma once
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

void uart_init(void);
bool uart_busy(void);
bool uart_send(const void *data, size_t len);
extern uint32_t adc_dma_done;