#include <octave/oct.h>
#include <vector>
#include <cstdint>

extern "C" {
#include "ddsli_core.h"
}

static ddsli_ctx_t *ctx = NULL;

/* ===================== OPEN ===================== */

DEFUN_DLD(ddsli_open, args, ,
          "ddsli_open([port], [baud])")
{
    if (ctx)
        error("DDSLI already open");

    const char *port = NULL;
    speed_t baud = 0;

    if (args.length() >= 1 && args(0).is_string())
        port = args(0).string_value().c_str();

    if (args.length() >= 2)
        baud = (speed_t) args(1).int_value();

    ctx = ddsli_open(port, baud);
    if (!ctx)
        error("Failed to open DDSLI");

    return octave_value();
}

/* ===================== CONTROL ===================== */

DEFUN_DLD(ddsli_toggle_out_stream, args, ,
          "ddsli_toggle_out_stream()")
{
    if (!ctx)
        error("DDSLI not open");

    ddsli_toggle_out_stream(ctx);
    return octave_value();
}

DEFUN_DLD(ddsli_send_cmd, args, ,
          "ddsli_send_cmd(cmd)")
{
    if (!ctx)
        error("DDSLI not open");

    if (args.length() != 1 || !args(0).is_string())
        error("ddsli_send_cmd expects a string");

    std::string cmd = args(0).string_value();

    int ret = ddsli_send_cmd(ctx,
                             const_cast<char *>(cmd.data()),
                             (uint32_t)cmd.size());

    return octave_value(ret);
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

DEFUN_DLD(ddsli_read_blocks_matrix, args, ,
          "Read all available DDS blocks as matrices")
{
    if (!ctx)
        error("DDSLI not open");

    std::vector<ddsli_block_t> blocks;
    ddsli_block_t b;

    while (ddsli_read_block(ctx, &b)) {
        blocks.push_back(b);
    }

    octave_idx_type n = blocks.size();
    if (n == 0)
        return octave_value(octave_map());

    FloatMatrix chA(3, n);
    FloatMatrix chB(3, n);

    int64NDArray phase(dim_vector(1, n));
    int64NDArray phase_inc(dim_vector(1, n));
    int64NDArray phase_inc_delta(dim_vector(1, n));

    for (octave_idx_type i = 0; i < n; i++) {

        phase(i)           = blocks[i].phase;
        phase_inc(i)       = blocks[i].phase_inc;
        phase_inc_delta(i) = blocks[i].phase_inc_delta;

        for (int k = 0; k < 3; k++) {
            chA(k, i) = blocks[i].chA[k];
            chB(k, i) = blocks[i].chB[k];
        }
    }

    octave_map out;
    out.assign("phase",           octave_value(phase));
    out.assign("phase_inc",       octave_value(phase_inc));
    out.assign("phase_inc_delta", octave_value(phase_inc_delta));
    out.assign("chA",             octave_value(chA));
    out.assign("chB",             octave_value(chB));

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

DEFUN_DLD(ddsli_read_adc_matrix, args, ,
          "Read all available ADC packets as a matrix")
{
    if (!ctx)
        error("DDSLI not open");

    std::vector<uint16_t> tmp(ADC_SAMPLES_PER_PACKET);
    std::vector<std::vector<uint16_t>> packets;

    while (ddsli_read_adc_packet(ctx, tmp.data())) {
        packets.push_back(tmp);
    }

    octave_idx_type np = packets.size();
    if (np == 0)
        return octave_value(uint16NDArray(dim_vector(0, 0)));

    uint16NDArray out(dim_vector(ADC_SAMPLES_PER_PACKET, np));

    for (octave_idx_type p = 0; p < np; p++) {
        for (int i = 0; i < ADC_SAMPLES_PER_PACKET; i++) {
            out(i, p) = packets[p][i];
        }
    }

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