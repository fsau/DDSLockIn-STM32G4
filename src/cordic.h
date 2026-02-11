/* Minimal DMA-driven CORDIC helper for STM32G4 + libopencm3
 * Provides a simple API to convert an array of phase angles (Q1.31 or int32_t radians scaled)
 * into interleaved sine/cosine outputs using the hardware CORDIC and DMA.
 *
 * Usage (minimal):
 *  - call cordic_init();
 *  - call cordic_start_dma(in_buf, out_buf, len); // len = number of samples
 *  - wait for cordic_transfer_complete() or poll cordic_transfer_complete()
 */

#ifndef CORDIC_H
#define CORDIC_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

extern volatile bool cordic_transfer_in_progress;
extern volatile int cordic_transfer_done;

// Initialize CORDIC peripheral and required DMA/clocks. Must be called once.
void cordic_init(void);

// Start DMA-driven conversion.
// in_angles: pointer to array of int32_t phase words (radians scaled to Q1.31)
// out_buf: pointer to array where results will be stored as int32_t pairs: [cos0, sin0, cos1, sin1, ...]
// len: number of angles to convert
// Returns 0 on success, non-zero on failure (e.g., busy)
// Start DMA-driven conversion using user-provided 32-bit write/read buffers.
// write_buf32: pointer to array of packed 32-bit write words (lower 16 = angle,
//              upper 16 = additional argument such as 0x7FFF). Must remain valid
//              until transfer completes.
// read_buf32: pointer to array where CORDIC will write 32-bit results (len entries).
// len: number of samples
// Returns 0 on success, negative on error, -2 if busy.
int cordic_start_dma(volatile uint32_t *write_buf32, volatile uint32_t *read_buf32, size_t len);

// Returns non-zero if DMA transfer finished (and peripheral idle). Clears internal flag on read.
int cordic_transfer_complete(void);

// Abort any ongoing transfer.
void cordic_abort(void);

#endif // CORDIC_H
