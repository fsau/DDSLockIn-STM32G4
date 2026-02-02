/* Minimal DAC circular DMA output (TIM6 trigger)
 * This is a renamed copy of the previous dac_dma module.
 */

#ifndef DAC_H
#define DAC_H

#include <stdint.h>
#include <stddef.h>

// Initialize clocks and peripherals (DAC, DMA). Call once.
void dac_init(void);

// Start circular DMA output. Buffer must remain valid while running.
// samples: pointer to uint16_t samples (12-bit), length: number of samples
// Returns 0 on success.
int dac_start(volatile uint16_t *samples, size_t length);

// Stop DMA and DAC output.
void dac_stop(void);

// Query running state (non-zero when active)
int dac_running(void);

// DMA interrupt flags (half-transfer, transfer-complete, error)
int dac_dma_half_flag(void);
int dac_dma_full_flag(void);
int dac_dma_error_flag(void);
void dac_dma_clear_flags(void);

#endif // DAC_H
