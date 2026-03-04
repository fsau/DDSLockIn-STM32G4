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


void update_pll(ddsli_output_t out, ddsli_phase_ctrl_t *f_ctrl)
{
    
}