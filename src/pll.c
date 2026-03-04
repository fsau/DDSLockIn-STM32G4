/*
 * Frequency control (per dds):
 *   PLL vars:
 *   - input source (ch): a, b
 *   - phase setpoint
 *   - f_min, f_max, f_init
 *   - tau/filtering
 *   - at f_max/min: limit/wrap/reset to f_init
*/

#include "pll.h"

void update_pll(pll_instance *pll, ddsli_output_t out, ddsli_phase_ctrl_t *dds)
{
    float a, b;
    if(pll->conf.source == 1)
    {
        a = out.chA[0];
        b = out.chA[1];
    }
    else
    {
        a = out.chB[0];
        b = out.chB[1];
    }

    float phase_error = a/b - pll->conf.phase_sp; // using tangent, maybe lookup table for angles?

    if(((phase_error > 0) && (dds->phase_inc < pll->conf.f_max)) || 
       ((phase_error < 0) && (dds->phase_inc > pll->conf.f_min)) )
    {
        dds->phase_inc_delta = pll->conf.tau*phase_error;
    }
    else
    {
        dds->phase_inc_delta = 0;

        switch(pll->conf.max_act)
        {
        case PLL_MAX_LIM:
            break;
        case PLL_MAX_WRAP:
            dds->phase_inc = (phase_error > 0) ? pll->conf.f_min : pll->conf.f_max;
            break;
        case PLL_MAX_RESET:
            dds->phase_inc = pll->conf.f_init;
            break;
        }
    }
}
