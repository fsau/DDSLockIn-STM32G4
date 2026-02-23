#include <octave/oct.h>
#include <vector>
#include <cstdint>

extern "C" {
#include "ddsli_core.h"
}

static ddsli_ctx_t *ctx = NULL;

/* ===================== OPEN ===================== */

DEFUN_DLD(ddsli_open, args, ,
          "ddsli_open(port, baud)")
{
    std::string port = args(0).string_value();
    int baud = args(1).int_value();

    ctx = ddsli_open(port.c_str(), baud);
    if (!ctx)
        error("Failed to open serial");

    return octave_value();
}

/* ===================== CONTROL ===================== */

DEFUN_DLD(ddsli_toggle_out_stream, args, ,
          "ddsli_toggle_out_stream()")
{
    if (!ctx)
        error("DDSLI not open");

    ddsli_toggle_out_strem(ctx);
    return octave_value();
}

/* ===================== READ BLOCKS ===================== */

DEFUN_DLD(ddsli_read_blocks, args, ,
          "Read DDSLI blocks")
{
    int n = 1;
    if (args.length() > 0)
        n = args(0).int_value();

    if (n <= 0)
        error("n must be positive");

    std::vector<ddsli_block_t> v;
    v.reserve(n);

    /* backend now provides single-block pop */
    for (int i = 0; i < n; i++) {
        ddsli_block_t b;
        if (!ddsli_read_block(ctx, &b))
            break;
        v.push_back(b);
    }

    Cell out(v.size(), 1);

    for (size_t i = 0; i < v.size(); i++) {
        octave_map m;

        octave_map freq;
        freq.assign("phase",
                    octave_value(int64_t(v[i].phase)));
        freq.assign("phase_inc",
                    octave_value(int64_t(v[i].phase_inc)));
        freq.assign("phase_inc_delta",
                    octave_value(int64_t(v[i].phase_inc_delta)));

        m.assign("frequency", octave_value(freq));

        FloatRowVector chA(3), chB(3);
        for (int k = 0; k < 3; k++) {
            chA(k) = v[i].chA[k];
            chB(k) = v[i].chB[k];
        }

        /* must wrap vectors as octave_value */
        m.assign("chA", octave_value(chA));
        m.assign("chB", octave_value(chB));

        out(i) = m;
    }

    return octave_value(out);
}

/* ===================== READ ADC ===================== */

DEFUN_DLD(ddsli_read_adc, args, ,
          "Read raw ADC packet")
{
    if (!ctx)
        error("DDSLI not open");

    const int samples = ADC_PACKET_BYTES / 2;
    std::vector<uint16_t> buf(samples);

    if (!ddsli_read_adc_packet(ctx, buf.data()))
        return octave_value(); /* empty if no data */

    uint16NDArray out(dim_vector(1, samples));
    for (int i = 0; i < samples; i++)
        out(i) = buf[i];

    return octave_value(out);
}

/* ===================== CLOSE ===================== */

DEFUN_DLD(ddsli_close, args, ,
          "ddsli_close()")
{
    if (ctx) {
        ddsli_close(ctx);
        ctx = NULL;
    }
    return octave_value();
}