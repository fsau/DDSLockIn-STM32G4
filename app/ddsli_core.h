#ifndef DDSLI_CORE_H
#define DDSLI_CORE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== CONFIG ===================== */

#define RX_BUF_SIZE (1 << 24)   /* 16 MB */

#define BLOCK_FIFO_LEN   1024

#define ADC_FIFO_LEN     64
#define ADC_PACKET_BYTES 4096
#define ADC_SAMPLES_PER_PACKET (ADC_PACKET_BYTES / 2)

/* ===================== TYPES ===================== */

/* Opaque context */
typedef struct ddsli_ctx ddsli_ctx_t;

/* Parsed DDS / lock-in block */
typedef struct {
    uint64_t phase;
    uint64_t phase_inc;
    uint64_t phase_inc_delta;

    uint16_t ampA1;
    uint16_t ampA2;

    float chA[3];
    float chB[3];
} ddsli_block_t;

/* ===================== LIFECYCLE ===================== */

/**
 * Open DDSLI device on a serial port.
 *
 * @param port  Serial device path (e.g. "/dev/ttyUSB0")
 * @param baud  Baud rate (termios speed constant, e.g. B115200)
 * @return      Context pointer or NULL on failure
 */
ddsli_ctx_t *ddsli_open(const char *port, int baud);

/**
 * Close device and free all resources.
 */
void ddsli_close(ddsli_ctx_t *ctx);

/* ===================== READ API ===================== */

/**
 * Pop one parsed DDSLI block from the FIFO.
 *
 * Non-blocking: returns 0 if no block is available.
 *
 * @return 1 if a block was read, 0 otherwise
 */
int ddsli_read_block(ddsli_ctx_t *ctx, ddsli_block_t *out);

/**
 * Pop one raw ADC packet.
 *
 * The buffer must hold exactly one packet:
 *   ADC_PACKET_BYTES / 2 samples (uint16_t).
 *
 * Non-blocking: returns 0 if no packet is available.
 *
 * @return 1 if a packet was read, 0 otherwise
 */
int ddsli_read_adc_packet(ddsli_ctx_t *ctx, uint16_t *out);

/* ===================== CONTROL ===================== */

/**
 * Toggle output stream on the device.
 *
 * Sends "cp" over the serial link.
 *
 * @return write() return value
 */
int ddsli_toggle_out_strem(ddsli_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* DDSLI_CORE_H */