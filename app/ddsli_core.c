#include "ddsli_core.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdint.h>
#include <glob.h>

#define DDSLI_DEFAULT_BAUD B115200

/* ===================== FIFO TYPES ===================== */

typedef struct {
    ddsli_block_t buf[BLOCK_FIFO_LEN];
    size_t rd, wr, count;
} block_fifo_t;

typedef struct {
    uint16_t buf[ADC_FIFO_LEN][ADC_SAMPLES_PER_PACKET];
    size_t rd, wr, count;
} adc_fifo_t;

/* ===================== CONTEXT ===================== */

struct ddsli_ctx {
    int fd;

    uint8_t rxbuf[RX_BUF_SIZE];
    size_t  rxlen;

    pthread_t       rx_thread;
    pthread_mutex_t lock;
    atomic_int      running;

    block_fifo_t block_fifo;
    adc_fifo_t   adc_fifo;
};

/* ===================== FIFO HELPERS ===================== */

static void block_fifo_push(ddsli_ctx_t *c, const ddsli_block_t *b)
{
    if (c->block_fifo.count == BLOCK_FIFO_LEN)
        return;

    // fprintf(stderr,
    //     "FIFO PUSH idx=%zu phase=%llu\n",
    //     c->block_fifo.wr,
    //     (unsigned long long)b->phase);

    c->block_fifo.buf[c->block_fifo.wr] = *b;
    c->block_fifo.wr = (c->block_fifo.wr + 1) % BLOCK_FIFO_LEN;
    c->block_fifo.count++;
}

static int block_fifo_pop(ddsli_ctx_t *c, ddsli_block_t *out)
{
    if (!c->block_fifo.count)
        return 0;

    // fprintf(stderr,
    //     "FIFO POP idx=%zu phase=%llu\n",
    //     c->block_fifo.rd,
    //     (unsigned long long)out->phase);

    *out = c->block_fifo.buf[c->block_fifo.rd];
    c->block_fifo.rd = (c->block_fifo.rd + 1) % BLOCK_FIFO_LEN;
    c->block_fifo.count--;

    return 1;
}

static void adc_fifo_push(ddsli_ctx_t *c, const uint8_t *raw)
{
    if (c->adc_fifo.count == ADC_FIFO_LEN)
        return;
    // fprintf(stderr,"ADC FIFO PUSH. id=%ld\n", c->adc_fifo.wr);
    memcpy(c->adc_fifo.buf[c->adc_fifo.wr], raw, ADC_PACKET_BYTES);
    c->adc_fifo.wr = (c->adc_fifo.wr + 1) % ADC_FIFO_LEN;
    c->adc_fifo.count++;
}

static int adc_fifo_pop(ddsli_ctx_t *c, uint16_t *out)
{
    if (!c->adc_fifo.count)
        return 0;

    // fprintf(stderr,"ADC FIFO POP. id=%ld\n", c->adc_fifo.rd);
    memcpy(out,
           c->adc_fifo.buf[c->adc_fifo.rd],
           ADC_PACKET_BYTES);

    c->adc_fifo.rd = (c->adc_fifo.rd + 1) % ADC_FIFO_LEN;
    c->adc_fifo.count--;
    return 1;
}

/* ===================== BLOCK PARSER ===================== */

static void parse_block(const uint8_t *b, ddsli_block_t *o)
{
    memcpy(&o->phase,           b +  0, 8);
    memcpy(&o->phase_inc,       b +  8, 8);
    memcpy(&o->phase_inc_delta, b + 16, 8);

    memcpy(&o->ampA1, b + 24, 2);
    memcpy(&o->ampA2, b + 26, 2);

    memcpy(o->chA, b + 28, 12);
    memcpy(o->chB, b + 40, 12);
}

/* ===================== RX PARSER ===================== */

static void parse_rx_buffer(ddsli_ctx_t *ctx)
{
    size_t pos = 0;

    while (pos + 4 <= ctx->rxlen) {

        if (ctx->rxbuf[pos]     != 0x55 ||
            ctx->rxbuf[pos + 1] != 0x55 ||
            ctx->rxbuf[pos + 2] != 0x55) {
            pos++;
            continue;
        }

        uint8_t type = ctx->rxbuf[pos + 3];

        /* ---------- BLOCK PACKET ---------- */
        if (type == 0x20) {
            if (pos + 4 + BLOCK_DATA_BYTES > ctx->rxlen)
                break;

            const uint8_t *p = ctx->rxbuf + pos + 4;
            uint8_t term = p[BLOCK_DATA_BYTES];
            ddsli_block_t blk;
            parse_block(p, &blk);

            // fprintf(stderr, "PARSE BLOCK at pos=%zu rxlen=%zu type=%02x header=%02x%02x%02x phase=%lu\n",
            // pos, ctx->rxlen, type, ctx->rxbuf[pos], ctx->rxbuf[pos+1], ctx->rxbuf[pos+2], blk.phase);

            block_fifo_push(ctx, &blk);

            pos += 4 + BLOCK_DATA_BYTES;
            continue;
        }

        /* ---------- ADC STREAM PACKET ---------- */
        if (type == 0x00) {
            if (pos + 4 + ADC_PACKET_BYTES > ctx->rxlen)
                break;

            adc_fifo_push(ctx, ctx->rxbuf + pos + 4);
            pos += 4 + ADC_PACKET_BYTES;
            continue;
        }

        pos++;
    }

    if (pos) {
        memmove(ctx->rxbuf, ctx->rxbuf + pos, ctx->rxlen - pos);
        ctx->rxlen -= pos;
    }
}

/* ===================== RX THREAD ===================== */

static void *rx_thread_fn(void *arg)
{
    ddsli_ctx_t *ctx = arg;

    while (atomic_load(&ctx->running)) {

        ssize_t r = read(ctx->fd,
                         ctx->rxbuf + ctx->rxlen,
                         RX_BUF_SIZE - ctx->rxlen);

        if (r <= 0) {
            usleep(1000);
            continue;
        }

        pthread_mutex_lock(&ctx->lock);
        ctx->rxlen += (size_t)r;
        parse_rx_buffer(ctx);
        pthread_mutex_unlock(&ctx->lock);
    }

    return NULL;
}

/* ===================== PUBLIC READ API ===================== */

int ddsli_read_block(ddsli_ctx_t *ctx, ddsli_block_t *out)
{
    int ok;
    pthread_mutex_lock(&ctx->lock);
    ok = block_fifo_pop(ctx, out);
    pthread_mutex_unlock(&ctx->lock);
    return ok;
}

int ddsli_read_adc_packet(ddsli_ctx_t *ctx, uint16_t *out)
{
    int ok;
    pthread_mutex_lock(&ctx->lock);
    ok = adc_fifo_pop(ctx, out);
    pthread_mutex_unlock(&ctx->lock);
    return ok;
}

/* ===================== SERIAL ===================== */

static char *find_first_ttyacm(void)
{
    glob_t g;
    if (glob("/dev/ttyACM*", 0, NULL, &g) != 0)
        return NULL;

    if (g.gl_pathc == 0) {
        globfree(&g);
        return NULL;
    }

    /* strdup so caller owns it */
    char *dev = strdup(g.gl_pathv[0]);
    globfree(&g);
    return dev;
}

static int serial_open(const char *port, int baud)
{
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0)
        return -1;

    struct termios t;
    tcgetattr(fd, &t);
    cfmakeraw(&t);
    cfsetspeed(&t, baud);
    t.c_cflag |= CLOCAL | CREAD;
    tcsetattr(fd, TCSANOW, &t);

    return fd;
}

/* ===================== LIFECYCLE ===================== */

ddsli_ctx_t *ddsli_open(const char *port, speed_t baud)
{
    char *auto_port = NULL;

    if (!port) {
        auto_port = find_first_ttyacm();
        if (!auto_port)
            return NULL;
        port = auto_port;
    }

    if (!baud)
        baud = DDSLI_DEFAULT_BAUD;

    ddsli_ctx_t *ctx = calloc(1, sizeof(*ctx));
    if (!ctx) {
        free(auto_port);
        return NULL;
    }

    ctx->fd = serial_open(port, baud);
    free(auto_port);

    if (ctx->fd < 0) {
        free(ctx);
        return NULL;
    }

    pthread_mutex_init(&ctx->lock, NULL);
    atomic_store(&ctx->running, 1);

    if (pthread_create(&ctx->rx_thread, NULL,
                       rx_thread_fn, ctx) != 0) {
        close(ctx->fd);
        pthread_mutex_destroy(&ctx->lock);
        free(ctx);
        return NULL;
    }

    return ctx;
}

void ddsli_close(ddsli_ctx_t *ctx)
{
    if (!ctx)
        return;

    atomic_store(&ctx->running, 0);

    close(ctx->fd);

    pthread_join(ctx->rx_thread, NULL);

    pthread_mutex_destroy(&ctx->lock);
    free(ctx);
}

/* ===================== CONTROL ===================== */

int ddsli_toggle_out_stream(ddsli_ctx_t *ctx)
{
    return write(ctx->fd, "cp", 2);
}

int ddsli_send_cmd(ddsli_ctx_t *ctx, char *str, uint32_t strlen)
{
    return write(ctx->fd, str, strlen);
}