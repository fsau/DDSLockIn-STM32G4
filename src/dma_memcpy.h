#ifndef DMA_MEMCPY_H
#define DMA_MEMCPY_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Function prototypes
bool dma_memcpy_init(void);
uint32_t dma_memcpy32(volatile uint32_t *dest, volatile uint32_t *src, uint32_t size);
bool dma_memcpy_is_complete(uint32_t id);

#ifdef __cplusplus
}
#endif

#endif // DMA_MEMCPY_H